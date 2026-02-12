/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_USBH_INTERNAL_H
#define ZEPHYR_INCLUDE_USBH_INTERNAL_H

#include <stdint.h>
#include <zephyr/usb/usbh.h>

int usbh_init_device_intl(struct usbh_contex *const uhs_ctx);

/* Stop hub status monitor before freeing a hub device */
void usbh_hub_monitor_stop(struct usb_device *const hub);

void usbh_notify_device_event(struct usbh_contex *const ctx,
			      struct usb_device *const udev,
			      enum usbh_device_event event);

#endif /* ZEPHYR_INCLUDE_USBH_INTERNAL_H */
