// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - https://www.ti.com
 * Written by Jean-Jacques Hiblot <jjhiblot@ti.com>
 */

#define LOG_CATEGORY UCLASS_USB_GADGET_GENERIC

#include <dm.h>
#include <dm/device-internal.h>
#include <g_dnl.h>
#include <linux/printk.h>
#include <linux/usb/gadget.h>

#if CONFIG_IS_ENABLED(DM_USB_GADGET)
static inline const struct usb_gadget_generic_ops *
usb_gadget_generic_dev_ops(struct udevice *dev)
{
	return (const struct usb_gadget_generic_ops *)dev->driver->ops;
}

int dm_usb_gadget_handle_interrupts(struct udevice *dev)
{
	const struct usb_gadget_generic_ops *ops;

	ops = usb_gadget_generic_dev_ops(dev);
	if (!ops)
		return -EFAULT;
	if (!ops->handle_interrupts)
		return -ENOSYS;

	return ops->handle_interrupts(dev);
}

int udc_device_get_by_index(int index, struct udevice **udev)
{
	struct udevice *dev = NULL;
	int ret;

	/*
	 * 已有 g_dnl 函数在用 UDC（例如在 Optimus 线刷态或 fastboot 里执行的命令又要起 ums、fastboot）：嵌套的命令出错退出时会 udc_device_put() 把正在用的控制器移除，所以在取控制器时就拒绝
	 */
	if (CONFIG_IS_ENABLED(USB_GADGET_DOWNLOAD) && g_dnl_busy()) {
		pr_err("USB device controller is in use by another gadget\n");
		return -EBUSY;
	}

	ret = uclass_get_device_by_seq(UCLASS_USB_GADGET_GENERIC, index, &dev);
	if (!ret && dev) {
		*udev = dev;
		return 0;
	}

	ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, index, &dev);
	if (!ret && dev) {
		*udev = dev;
		return 0;
	}

	pr_err("No USB device found\n");
	return -ENODEV;
}

int udc_device_put(struct udevice *udev)
{
#if CONFIG_IS_ENABLED(DM_DEVICE_REMOVE)
	return device_remove(udev, DM_REMOVE_NORMAL);
#else
	return -ENOSYS;
#endif
}
#else
/* Backwards hardware compatibility -- switch to DM_USB_GADGET */
static int legacy_index;
int udc_device_get_by_index(int index, struct udevice **udev)
{
	legacy_index = index;
	return board_usb_init(index, USB_INIT_DEVICE);
}

int udc_device_put(struct udevice *udev)
{
	return board_usb_cleanup(legacy_index, USB_INIT_DEVICE);
}

__weak int dm_usb_gadget_handle_interrupts(struct udevice *dev)
{
	return 0;
}
#endif

#if CONFIG_IS_ENABLED(DM)
UCLASS_DRIVER(usb_gadget_generic) = {
	.id		= UCLASS_USB_GADGET_GENERIC,
	.name		= "usb_gadget",
	.flags		= DM_UC_FLAG_SEQ_ALIAS,
};
#endif
