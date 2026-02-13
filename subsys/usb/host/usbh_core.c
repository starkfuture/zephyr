/*
 * Copyright (c) 2022,2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sys/iterable_sections.h>

#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usb_hub.h>
#include <zephyr/drivers/usb/uhc.h>

#include "usbh_internal.h"
#include "usbh_device.h"
#include "usbh_ch9.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhs, CONFIG_USBH_LOG_LEVEL);

static K_KERNEL_STACK_DEFINE(usbh_stack, CONFIG_USBH_STACK_SIZE);
static struct k_thread usbh_thread_data;

static K_KERNEL_STACK_DEFINE(usbh_bus_stack, CONFIG_USBH_STACK_SIZE);
static struct k_thread usbh_bus_thread_data;

K_MSGQ_DEFINE(usbh_msgq, sizeof(struct uhc_event),
	      CONFIG_USBH_MAX_UHC_MSG, sizeof(uint32_t));

K_MSGQ_DEFINE(usbh_bus_msgq, sizeof(struct uhc_event),
	      CONFIG_USBH_MAX_UHC_MSG, sizeof(uint32_t));

#define USBH_DEVICE_EVENT_LISTENER_MAX 8
#define HUB_MONITOR_MAX 8
#define HUB_MONITOR_ENQUEUE_RETRIES 8
#define HUB_MONITOR_ENQUEUE_RETRY_DELAY_MS 20
#define HUB_PORT_INIT_FAIL_BACKOFF_MS 2000
#define HUB_PORT_RETRY_GUARD_MAX 32

struct usbh_device_event_listener {
	usbh_device_event_cb_t cb;
	void *user_data;
};

static struct usbh_device_event_listener usbh_dev_event_listeners[USBH_DEVICE_EVENT_LISTENER_MAX];
static struct k_mutex usbh_dev_event_lock;

/** Hub monitor state for event-driven hotplug */
struct hub_monitor {
	struct usb_device *hub;
	struct uhc_transfer *xfer;
	atomic_t pending;
	bool active;
};

struct hub_port_retry_guard {
	uint8_t hub_addr;
	uint8_t port;
	int64_t next_retry_ms;
};

static struct hub_monitor hub_monitors[HUB_MONITOR_MAX];
static struct hub_port_retry_guard hub_port_retry_guards[HUB_PORT_RETRY_GUARD_MAX];
K_SEM_DEFINE(hub_evt_sem, 0, 1);
static K_KERNEL_STACK_DEFINE(usbh_hub_evt_stack, CONFIG_USBH_STACK_SIZE);
static struct k_thread usbh_hub_evt_thread_data;

static void usbh_hub_enumerate_children(struct usbh_contex *const ctx,
					struct usb_device *const hub);

static struct hub_port_retry_guard *hub_port_retry_guard_get(uint8_t hub_addr,
							      uint8_t port,
							      bool create)
{
	struct hub_port_retry_guard *free_slot = NULL;

	for (int i = 0; i < HUB_PORT_RETRY_GUARD_MAX; i++) {
		if (hub_port_retry_guards[i].hub_addr == hub_addr &&
		    hub_port_retry_guards[i].port == port) {
			return &hub_port_retry_guards[i];
		}

		if (create && free_slot == NULL &&
		    hub_port_retry_guards[i].hub_addr == 0U) {
			free_slot = &hub_port_retry_guards[i];
		}
	}

	if (create && free_slot != NULL) {
		free_slot->hub_addr = hub_addr;
		free_slot->port = port;
		free_slot->next_retry_ms = 0;
		return free_slot;
	}

	return NULL;
}

static bool hub_port_retry_blocked(struct usb_device *const hub, uint8_t port)
{
	struct hub_port_retry_guard *guard = hub_port_retry_guard_get(hub->addr, port, false);

	return guard != NULL && guard->next_retry_ms > k_uptime_get();
}

static void hub_port_retry_mark_failed(struct usb_device *const hub, uint8_t port)
{
	struct hub_port_retry_guard *guard = hub_port_retry_guard_get(hub->addr, port, true);

	if (guard != NULL) {
		guard->next_retry_ms = k_uptime_get() + HUB_PORT_INIT_FAIL_BACKOFF_MS;
	}
}

static void hub_port_retry_clear(struct usb_device *const hub, uint8_t port)
{
	struct hub_port_retry_guard *guard = hub_port_retry_guard_get(hub->addr, port, false);

	if (guard != NULL) {
		guard->next_retry_ms = 0;
	}
}

int usbh_device_event_register(usbh_device_event_cb_t cb, void *user_data)
{
	int ret = 0;

	if (cb == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&usbh_dev_event_lock, K_FOREVER);
	for (int i = 0; i < USBH_DEVICE_EVENT_LISTENER_MAX; i++) {
		if (usbh_dev_event_listeners[i].cb == cb &&
		    usbh_dev_event_listeners[i].user_data == user_data) {
			ret = -EALREADY;
			goto out;
		}
	}

	for (int i = 0; i < USBH_DEVICE_EVENT_LISTENER_MAX; i++) {
		if (usbh_dev_event_listeners[i].cb == NULL) {
			usbh_dev_event_listeners[i].cb = cb;
			usbh_dev_event_listeners[i].user_data = user_data;
			goto out;
		}
	}

	ret = -ENOMEM;
out:
	k_mutex_unlock(&usbh_dev_event_lock);
	return ret;
}

int usbh_device_event_unregister(usbh_device_event_cb_t cb, void *user_data)
{
	int ret = -ENOENT;

	if (cb == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&usbh_dev_event_lock, K_FOREVER);
	for (int i = 0; i < USBH_DEVICE_EVENT_LISTENER_MAX; i++) {
		if (usbh_dev_event_listeners[i].cb == cb &&
		    usbh_dev_event_listeners[i].user_data == user_data) {
			usbh_dev_event_listeners[i].cb = NULL;
			usbh_dev_event_listeners[i].user_data = NULL;
			ret = 0;
			break;
		}
	}
	k_mutex_unlock(&usbh_dev_event_lock);

	return ret;
}

void usbh_notify_device_event(struct usbh_contex *const ctx,
			      struct usb_device *const udev,
			      enum usbh_device_event event)
{
	struct usbh_device_event_listener listeners[USBH_DEVICE_EVENT_LISTENER_MAX];

	if (ctx == NULL || udev == NULL) {
		return;
	}

	k_mutex_lock(&usbh_dev_event_lock, K_FOREVER);
	memcpy(listeners, usbh_dev_event_listeners, sizeof(listeners));
	k_mutex_unlock(&usbh_dev_event_lock);

	for (int i = 0; i < USBH_DEVICE_EVENT_LISTENER_MAX; i++) {
		if (listeners[i].cb != NULL) {
			listeners[i].cb(ctx, udev, event, listeners[i].user_data);
		}
	}
}

/** USB 2.0 Hub Descriptor (Table 11-13), minimal fields */
struct usb_hub_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bNbrPorts;
	uint16_t wHubCharacteristics;
	uint8_t bPwrOn2PwrGood;
	uint8_t bHubContrCurrent;
} __packed;

#define USB_DESC_HUB			0x29

/** Port status bits from USB 2.0 hub class wPortStatus field. */
#define USB_HUB_PORT_CONNECTION		BIT(0)
#define USB_HUB_PORT_ENABLE		BIT(1)
#define USB_HUB_PORT_LOW_SPEED		BIT(9)
#define USB_HUB_PORT_HIGH_SPEED		BIT(10)
#define USB_HUB_PORT_C_CONNECTION	BIT(16)
#define USB_HUB_PORT_C_ENABLE		BIT(17)
#define USB_HUB_PORT_C_SUSPEND		BIT(18)
#define USB_HUB_PORT_C_OVER_CURRENT	BIT(19)
#define USB_HUB_PORT_C_RESET		BIT(20)

static bool is_hub_class(struct usb_device *const udev)
{
	if (udev->dev_desc.bDeviceClass == USB_BCC_HUB) {
		return true;
	}
	if (udev->cfg_desc != NULL) {
		struct usb_cfg_descriptor *cfg = udev->cfg_desc;
		struct usb_desc_header *dhp;
		void *desc_end = (uint8_t *)cfg + cfg->wTotalLength;

		dhp = (void *)((uint8_t *)cfg + cfg->bLength);
		while ((void *)dhp < desc_end && dhp->bLength != 0) {
			if (dhp->bDescriptorType == USB_DESC_INTERFACE) {
				struct usb_if_descriptor *if_desc =
					(struct usb_if_descriptor *)dhp;

				if (if_desc->bInterfaceClass == USB_BCC_HUB) {
					return true;
				}
			}
			dhp = (void *)((uint8_t *)dhp + dhp->bLength);
		}
	}
	return false;
}

static int usbh_req_clear_hub_feature(struct usb_device *const hub,
				      const uint16_t feature)
{
	const uint8_t bmRequestType = (USB_REQTYPE_DIR_TO_DEVICE << 7) |
				      (USB_REQTYPE_TYPE_CLASS << 5) |
				      USB_REQTYPE_RECIPIENT_DEVICE;

	return usbh_req_setup(hub, bmRequestType, USB_HCREQ_CLEAR_FEATURE,
			      feature, 0, 0, NULL);
}

static int usbh_req_clear_port_feature(struct usb_device *const hub,
				       const uint8_t port, const uint16_t feature)
{
	const uint8_t bmRequestType = (USB_REQTYPE_DIR_TO_DEVICE << 7) |
				      (USB_REQTYPE_TYPE_CLASS << 5) |
				      USB_REQTYPE_RECIPIENT_OTHER;

	return usbh_req_setup(hub, bmRequestType, USB_HCREQ_CLEAR_FEATURE,
			      feature, port, 0, NULL);
}

/** True if hub interrupt payload reports any port/hub status change bits. */
static bool hub_status_has_change(struct net_buf *buf)
{
	if (buf == NULL || buf->len == 0) {
		return false;
	}

	/*
	 * Bitmap bit 0 is hub-level status change. For child enumeration we only
	 * care about downstream port bits (bit 1+). Treating bit 0 as a child
	 * change can cause pointless rescan storms on hubs that keep it asserted.
	 */
	if ((buf->data[0] & ~BIT(0)) != 0U) {
		return true;
	}

	for (size_t i = 1; i < buf->len; i++) {
		if (buf->data[i] != 0U) {
			return true;
		}
	}

	return false;
}

static void usbh_hub_ack_change_bits(struct usb_device *const hub, const uint8_t port,
				     uint32_t port_status)
{
	int ret;

	if (port_status & USB_HUB_PORT_C_CONNECTION) {
		ret = usbh_req_clear_port_feature(hub, port, USB_HCFS_C_PORT_CONNECTION);
		if (ret != 0) {
			LOG_WRN("Failed to clear C_PORT_CONNECTION on hub %u port %u: %d",
				hub->addr, port, ret);
		}
	}

	if (port_status & USB_HUB_PORT_C_ENABLE) {
		ret = usbh_req_clear_port_feature(hub, port, USB_HCFS_C_PORT_ENABLE);
		if (ret != 0) {
			LOG_WRN("Failed to clear C_PORT_ENABLE on hub %u port %u: %d",
				hub->addr, port, ret);
		}
	}

	if (port_status & USB_HUB_PORT_C_SUSPEND) {
		ret = usbh_req_clear_port_feature(hub, port, USB_HCFS_C_PORT_SUSPEND);
		if (ret != 0) {
			LOG_WRN("Failed to clear C_PORT_SUSPEND on hub %u port %u: %d",
				hub->addr, port, ret);
		}
	}

	if (port_status & USB_HUB_PORT_C_OVER_CURRENT) {
		ret = usbh_req_clear_port_feature(hub, port, USB_HCFS_C_PORT_OVER_CURRENT);
		if (ret != 0) {
			LOG_WRN("Failed to clear C_PORT_OVER_CURRENT on hub %u port %u: %d",
				hub->addr, port, ret);
		}
	}

	if (port_status & USB_HUB_PORT_C_RESET) {
		ret = usbh_req_clear_port_feature(hub, port, USB_HCFS_C_PORT_RESET);
		if (ret != 0) {
			LOG_WRN("Failed to clear C_PORT_RESET on hub %u port %u: %d",
				hub->addr, port, ret);
		}
	}
}

static int hub_status_change_cb(struct usb_device *const udev,
				struct uhc_transfer *const xfer)
{
	struct hub_monitor *mon = (struct hub_monitor *)xfer->priv;

	if (mon != NULL && mon->active && xfer->err == 0) {
		if (xfer->buf != NULL && xfer->buf->len > 0 &&
		    hub_status_has_change(xfer->buf)) {
			atomic_set(&mon->pending, 1);
			k_sem_give(&hub_evt_sem);
		}

		/* Re-arm status pipe; keep callback free of chapter-9 calls. */
		if (xfer->buf != NULL) {
			net_buf_reset(xfer->buf);
		}
		(void)uhc_ep_enqueue(((struct usbh_contex *)udev->ctx)->dev, xfer);
	}

	return 0;
}

static void usbh_hub_event_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct usbh_contex *ctx;
	struct hub_monitor *mon;

	while (true) {
		k_sem_take(&hub_evt_sem, K_FOREVER);

		for (int i = 0; i < HUB_MONITOR_MAX; i++) {
			mon = &hub_monitors[i];
			if (!mon->active || mon->hub == NULL) {
				continue;
			}
			if (atomic_cas(&mon->pending, 1, 0)) {
				ctx = mon->hub->ctx;
				usbh_hub_enumerate_children(ctx, mon->hub);
			}
		}
	}
}

static const struct usb_ep_descriptor *hub_find_status_change_ep(struct usb_device *const hub)
{
	for (int i = 1; i < 16; i++) {
		struct usb_ep_descriptor *ep = hub->ep_in[i].desc;

		if (ep != NULL &&
		    (ep->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_INTERRUPT) {
			return ep;
		}
	}
	return NULL;
}

static void usbh_hub_monitor_start(struct usbh_contex *const ctx,
				   struct usb_device *const hub)
{
	const struct usb_ep_descriptor *ep_desc;
	uint8_t ep_addr;
	struct hub_monitor *mon = NULL;
	struct uhc_transfer *xfer;
	const size_t hub_status_size = 4;
	int ret = 0;

	ep_desc = hub_find_status_change_ep(hub);
	if (ep_desc == NULL) {
		LOG_WRN("Hub addr %u has no interrupt IN endpoint", hub->addr);
		return;
	}
	ep_addr = ep_desc->bEndpointAddress;

	for (int i = 0; i < HUB_MONITOR_MAX; i++) {
		if (hub_monitors[i].active && hub_monitors[i].hub == hub) {
			return;
		}
	}

	for (int i = 0; i < HUB_MONITOR_MAX; i++) {
		if (!hub_monitors[i].active) {
			mon = &hub_monitors[i];
			break;
		}
	}
	if (mon == NULL) {
		LOG_ERR("No free hub monitor slot");
		return;
	}

	xfer = uhc_xfer_alloc_with_buf(ctx->dev, ep_addr, hub,
				       (void *)hub_status_change_cb, mon,
				       hub_status_size);
	if (xfer == NULL) {
		LOG_ERR("Failed to alloc hub monitor transfer");
		return;
	}

	/* Keep monitor periodic transfer as interrupt-type in UHC backend. */
	xfer->interval = ep_desc->bInterval > 0U ? ep_desc->bInterval : 1U;

	mon->hub = hub;
	mon->xfer = xfer;
	mon->active = true;
	atomic_set(&mon->pending, 0);

	for (int attempt = 0; attempt < HUB_MONITOR_ENQUEUE_RETRIES; attempt++) {
		ret = uhc_ep_enqueue(ctx->dev, xfer);
		if (ret == 0) {
			LOG_INF("Hub monitor started for addr %u", hub->addr);
			return;
		}

		/* Initial child traffic can transiently starve transfer slots. */
		k_sleep(K_MSEC(HUB_MONITOR_ENQUEUE_RETRY_DELAY_MS));
	}

	LOG_ERR("Failed to enqueue hub monitor for addr %u after %u retries (%d)",
		hub->addr, HUB_MONITOR_ENQUEUE_RETRIES, ret);
	mon->active = false;
	mon->hub = NULL;
	(void)uhc_xfer_free(ctx->dev, xfer);
	mon->xfer = NULL;
}

void usbh_hub_monitor_stop(struct usb_device *const hub)
{
	struct hub_monitor *mon;

	for (int i = 0; i < HUB_MONITOR_MAX; i++) {
		mon = &hub_monitors[i];
		if (mon->active && mon->hub == hub) {
			mon->active = false;
			mon->hub = NULL;
			(void)uhc_ep_dequeue(((struct usbh_contex *)hub->ctx)->dev, mon->xfer);
			(void)uhc_xfer_free(((struct usbh_contex *)hub->ctx)->dev, mon->xfer);
			mon->xfer = NULL;
			break;
		}
	}
}

static void usbh_hub_enumerate_children(struct usbh_contex *const ctx,
					struct usb_device *const hub)
{
	struct usb_hub_descriptor hub_desc;
	uint8_t nports;
	uint32_t port_status;
	int ret;
	struct usb_device *child;

	if (!is_hub_class(hub)) {
		return;
	}

	LOG_INF("Hub detected (USB_BCC_HUB), enumerating children");

	/* Read hub descriptor: class GET_DESCRIPTOR type 0x29 */
	{
		struct net_buf *buf = usbh_xfer_buf_alloc(hub, sizeof(hub_desc));

		if (!buf) {
			LOG_ERR("Failed to alloc hub descriptor buf");
			return;
		}
		ret = usbh_req_setup(hub,
				    USB_REQTYPE_DIR_TO_HOST << 7 |
				    USB_REQTYPE_TYPE_CLASS << 5 |
				    USB_REQTYPE_RECIPIENT_DEVICE,
				    USB_HCREQ_GET_DESCRIPTOR,
				    (USB_DESC_HUB << 8) | 0, 0,
				    sizeof(hub_desc), buf);
		if (ret != 0 || buf->len < 7) {
			usbh_xfer_buf_free(hub, buf);
			LOG_ERR("Failed to read hub descriptor");
			return;
		}
		memcpy(&hub_desc, buf->data, sizeof(hub_desc));
		usbh_xfer_buf_free(hub, buf);
	}

	nports = hub_desc.bNbrPorts;
	LOG_INF("Hub has %u ports", nports);

	/* Acknowledge hub-level latched status changes, if present. */
	(void)usbh_req_clear_hub_feature(hub, USB_HCFS_C_HUB_LOCAL_POWER);
	(void)usbh_req_clear_hub_feature(hub, USB_HCFS_C_HUB_OVER_CURRENT);

	/* Power on ports */
	for (uint8_t p = 1; p <= nports; p++) {
		ret = usbh_req_set_hcfs_ppwr(hub, p);
		if (ret != 0) {
			LOG_WRN("Failed to power port %u", p);
		}
	}
	k_msleep(20); /* bPwrOn2PwrGood * 2ms */

	/* Enumerate each port */
	for (uint8_t p = 1; p <= nports; p++) {
		ret = usbh_req_get_hcfs_ppst(hub, p, &port_status);
		if (ret != 0) {
			LOG_WRN("Failed to get port %u status", p);
			continue;
		}
		usbh_hub_ack_change_bits(hub, p, port_status);
		if (!(port_status & USB_HUB_PORT_CONNECTION)) {
			hub_port_retry_clear(hub, p);
			/* Disconnected: remove subtree if we had a child here */
			child = usbh_device_get_by_parent_port(ctx, hub, p);
			if (child != NULL) {
				usbh_device_free_subtree(child);
			}
			continue;
		}

		/* Avoid duplicate: already have device at this parent+port */
		child = usbh_device_get_by_parent_port(ctx, hub, p);
		if (child != NULL) {
			continue;
		}

		if (hub_port_retry_blocked(hub, p)) {
			continue;
		}

		LOG_INF("Port %u connected, resetting", p);

		ret = usbh_req_set_hcfs_prst(hub, p);
		if (ret != 0) {
			LOG_WRN("Failed to reset port %u", p);
			continue;
		}
		k_msleep(20);

		ret = usbh_req_get_hcfs_ppst(hub, p, &port_status);
		if (ret != 0 || !(port_status & USB_HUB_PORT_ENABLE)) {
			LOG_WRN("Port %u reset failed or not enabled", p);
			continue;
		}
		usbh_hub_ack_change_bits(hub, p, port_status);

		child = usbh_device_alloc(ctx);
		if (child == NULL) {
			LOG_ERR("Failed to allocate child for port %u", p);
			continue;
		}
		child->parent = hub;
		child->hub_port = p;
		child->state = USB_STATE_DEFAULT;
		if (port_status & USB_HUB_PORT_LOW_SPEED) {
			child->speed = USB_SPEED_SPEED_LS;
		} else if (port_status & USB_HUB_PORT_HIGH_SPEED) {
			child->speed = USB_SPEED_SPEED_HS;
		} else {
			child->speed = USB_SPEED_SPEED_FS;
		}

		LOG_INF("hub addr %u port %u", hub->addr, p);

		if (usbh_device_init_child(child) != 0) {
			LOG_ERR("Failed to init child on port %u", p);
			hub_port_retry_mark_failed(hub, p);
			usbh_device_free(child);
			continue;
		}
		hub_port_retry_clear(hub, p);
		LOG_INF("child VID %04x:%04x class %02x",
			child->dev_desc.idVendor, child->dev_desc.idProduct,
			child->dev_desc.bDeviceClass);
		usbh_notify_device_event(ctx, child, USBH_DEVICE_EVENT_ENUMERATED);
		if (is_hub_class(child)) {
			usbh_hub_enumerate_children(ctx, child);
		}
	}

	/* Start event-driven monitor for hotplug */
	usbh_hub_monitor_start(ctx, hub);
}

static int usbh_event_carrier(const struct device *dev,
			      const struct uhc_event *const event)
{
	int err;

	if (event->type == UHC_EVT_EP_REQUEST) {
		err = k_msgq_put(&usbh_msgq, event, K_NO_WAIT);
	} else {
		err = k_msgq_put(&usbh_bus_msgq, event, K_NO_WAIT);
	}

	return err;
}

static void dev_connected_handler(struct usbh_contex *const ctx,
				  const struct uhc_event *const event)
{

	LOG_DBG("Device connected event");
	if (ctx->root != NULL) {
		LOG_ERR("Device already connected");
		usbh_device_free_subtree(ctx->root);
		ctx->root = NULL;
	}

	ctx->root = usbh_device_alloc(ctx);
	if (ctx->root == NULL) {
		LOG_ERR("Failed allocate new device");
		return;
	}

	ctx->root->state = USB_STATE_DEFAULT;

	if (event->type == UHC_EVT_DEV_CONNECTED_HS) {
		ctx->root->speed = USB_SPEED_SPEED_HS;
	} else {
		ctx->root->speed = USB_SPEED_SPEED_FS;
	}

	if (usbh_device_init(ctx->root)) {
		LOG_ERR("Failed to reset new USB device");
		return;
	}
	usbh_notify_device_event(ctx, ctx->root, USBH_DEVICE_EVENT_ENUMERATED);
	if (is_hub_class(ctx->root)) {
		usbh_hub_enumerate_children(ctx, ctx->root);
	}
}

static void dev_removed_handler(struct usbh_contex *const ctx)
{
	if (ctx->root != NULL) {
		usbh_device_free_subtree(ctx->root);
		ctx->root = NULL;
		LOG_DBG("Device removed");
	} else {
		LOG_DBG("Spurious device removed event");
	}
}

static int discard_ep_request(struct usbh_contex *const ctx,
			      struct uhc_transfer *const xfer)
{
	const struct device *dev = ctx->dev;

	if (xfer->buf) {
		LOG_HEXDUMP_INF(xfer->buf->data, xfer->buf->len, "buf");
		uhc_xfer_buf_free(dev, xfer->buf);
	}

	return uhc_xfer_free(dev, xfer);
}

static ALWAYS_INLINE int usbh_event_handler(struct usbh_contex *const ctx,
					    struct uhc_event *const event)
{
	int ret = 0;

	switch (event->type) {
	case UHC_EVT_DEV_CONNECTED_LS:
		LOG_ERR("Low speed device not supported (connected event)");
		break;
	case UHC_EVT_DEV_CONNECTED_FS:
	case UHC_EVT_DEV_CONNECTED_HS:
		dev_connected_handler(ctx, event);
		break;
	case UHC_EVT_DEV_REMOVED:
		dev_removed_handler(ctx);
		break;
	case UHC_EVT_RESETED:
		LOG_DBG("Bus reset");
		break;
	case UHC_EVT_SUSPENDED:
		LOG_DBG("Bus suspended");
		break;
	case UHC_EVT_RESUMED:
		LOG_DBG("Bus resumed");
		break;
	case UHC_EVT_RWUP:
		LOG_DBG("RWUP event");
		break;
	case UHC_EVT_ERROR:
		LOG_DBG("Error event %d", event->status);
		break;
	default:
		break;
	};

	return ret;
}

static void usbh_bus_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct usbh_contex *uhs_ctx;
	struct uhc_event event;

	while (true) {
		k_msgq_get(&usbh_bus_msgq, &event, K_FOREVER);

		uhs_ctx = (void *)uhc_get_event_ctx(event.dev);
		usbh_event_handler(uhs_ctx, &event);
	}
}

static void usbh_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct usbh_contex *uhs_ctx;
	struct uhc_event event;
	usbh_udev_cb_t cb;
	int ret;

	while (true) {
		k_msgq_get(&usbh_msgq, &event, K_FOREVER);

		__ASSERT(event.type == UHC_EVT_EP_REQUEST, "Wrong event type");
		uhs_ctx = (void *)uhc_get_event_ctx(event.dev);
		cb = event.xfer->cb;

		if (event.xfer->cb) {
			ret = cb(event.xfer->udev, event.xfer);
		} else {
			ret = discard_ep_request(uhs_ctx, event.xfer);
		}

		if (ret) {
			LOG_ERR("Failed to handle request completion callback");
		}
	}
}

int usbh_init_device_intl(struct usbh_contex *const uhs_ctx)
{
	int ret;

	ret = uhc_init(uhs_ctx->dev, usbh_event_carrier, uhs_ctx);
	if (ret != 0) {
		LOG_ERR("Failed to init device driver");
		return ret;
	}

	sys_dlist_init(&uhs_ctx->udevs);

	STRUCT_SECTION_FOREACH(usbh_class_data, cdata) {
		/*
		 * For now, we have not implemented any class drivers,
		 * so just keep it as placeholder.
		 */
		break;
	}

	return 0;
}

static int uhs_pre_init(void)
{
	k_mutex_init(&usbh_dev_event_lock);

	k_thread_create(&usbh_thread_data, usbh_stack,
			K_KERNEL_STACK_SIZEOF(usbh_stack),
			usbh_thread,
			NULL, NULL, NULL,
			K_PRIO_COOP(9), 0, K_NO_WAIT);

	k_thread_name_set(&usbh_thread_data, "usbh");

	k_thread_create(&usbh_bus_thread_data, usbh_bus_stack,
			K_KERNEL_STACK_SIZEOF(usbh_bus_stack),
			usbh_bus_thread,
			NULL, NULL, NULL,
			K_PRIO_COOP(9), 0, K_NO_WAIT);

	k_thread_name_set(&usbh_thread_data, "usbh_bus");

	k_thread_create(&usbh_hub_evt_thread_data, usbh_hub_evt_stack,
			K_KERNEL_STACK_SIZEOF(usbh_hub_evt_stack),
			usbh_hub_event_thread,
			NULL, NULL, NULL,
			K_PRIO_COOP(9), 0, K_NO_WAIT);

	k_thread_name_set(&usbh_hub_evt_thread_data, "usbh_hub_evt");

	return 0;
}

SYS_INIT(uhs_pre_init, POST_KERNEL, CONFIG_USBH_INIT_PRIO);
