/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * CM211-1（S905L3B，p212 板基，由厂商 u-boot chainload 的 u-boot.ext）的板级配置头，经 CONFIG_SYS_CONFIG_NAME="cm211-1" 选中，在 meson64.h 的默认之上只改两处；启动顺序与入口在 board/amlogic/p212/cm211-1.env（CONFIG_ENV_SOURCE_FILE），bootcmd 是 run cm211_boot。
 *
 * 1. distro boot 的设备只留 usb0 与 mmc1（eMMC，别名见 dts/u-boot/cm211-1-u-boot.dtsi）：bootcmd 不走 distro boot（它认分区根目录的 boot.scr 与 EFI 可移动介质，随手插着的盘会被当成启动盘），这组变量只给提示符下手工用；mmc0 是 SD（本机没有接出卡槽）、mmc2 是 SDIO WiFi，网络一级由 cm211-1.env 的有界 TFTP 承担，romusb/usbdfu 只在 BootROM 从 USB 启动时有用，都不列。
 *
 * 2. kernel_comp_addr_r 从 0x0d080000 上移到 0x20000000：booti 把压缩的 Image（Image.gz）解压到这里，不查保留区；0x0d080000 到 secmon@10000000（no-map，BL31）只有 47.5 MiB，解压后更大的 arm64 Image 会写进 BL31。0x20000000 到启动脚本的暂存区 0x2d000000 有 208 MiB，下方 ramdisk_addr_r（0x13000000）到这里也有 208 MiB；未压缩的 Image 不经过这里。meson64.h 的 CFG_EXTRA_ENV_SETTINGS 在 env_default.h 展开时才取 KERNEL_COMP_ADDR_R，所以在包含之后重定义即生效；构建门禁核对 u-boot-initial-env 里的实际值。
 */
#ifndef __CM211_1_CONFIG_H
#define __CM211_1_CONFIG_H

#define BOOT_TARGET_DEVICES(func) \
	func(USB, usb, 0) \
	func(MMC, mmc, 1)

#include <configs/meson64.h>

#undef KERNEL_COMP_ADDR_R
#define KERNEL_COMP_ADDR_R	__stringify(0x20000000)

#endif /* __CM211_1_CONFIG_H */
