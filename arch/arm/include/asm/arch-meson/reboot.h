/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * fastboot 的重启标记 → Amlogic 重启原因（board-common.c 的 fastboot_set_reboot_flag() 用）。只是一张表，放在头文件里，sandbox 的单元测试直接核对它（test/cmd/fastboot.c）
 */

#ifndef __MESON_REBOOT_H__
#define __MESON_REBOOT_H__

#include <fastboot.h>
#include <linux/errno.h>
#include "sm.h"

/*
 * reboot-bootloader 与 reboot-fastboot 记 fastboot（4）：厂商 BL33 见到 4 进它的 fastboot，见到 bootloader（7）照常启动；主线作 BL33 时由 m12.env 的冷启动入口按原因分流。
 * reboot-recovery 不支持：厂商把原因 2 当 factory_reset（出厂 env 的 switch_bootmode：setenv reboot_mode_android factory_reset; saveenv; run recovery_from_flash），chainload 下复位后先跑厂商 BL33，等于发起恢复出厂；主线自己还没有 recovery 路径。返回 -EINVAL，fastboot 回 FAIL、不复位
 */
static inline long meson_fastboot_reboot_reason(enum fastboot_reboot_reason reason)
{
	switch (reason) {
	case FASTBOOT_REBOOT_REASON_BOOTLOADER:
	case FASTBOOT_REBOOT_REASON_FASTBOOTD:
		return REBOOT_REASON_FASTBOOT;
	default:
		return -EINVAL;
	}
}

#endif
