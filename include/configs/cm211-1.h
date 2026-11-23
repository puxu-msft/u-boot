/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * CM211-1（S905L3B，p212 板基，由厂商 u-boot chainload 的 u-boot.ext）的板级配置头，经 CONFIG_SYS_CONFIG_NAME="cm211-1" 选中，在 meson64.h 的默认之上只改两处。
 *
 * 1. distro boot 顺序 usb0 mmc0 mmc1 pxe dhcp，与镜像自带的 u-boot-r3300l.bin 相同：U 盘优先是本机的救援顺序；mmc0 = SD、mmc1 = eMMC（别名见 dts/u-boot/cm211-1-u-boot.dtsi），mmc2 是 SDIO WiFi，不扫；romusb/usbdfu 只在 BootROM 从 USB 启动时有用，chainload 下用不到。
 *
 * 2. kernel_comp_addr_r 从 0x0d080000 上移到 0x20000000：booti 把压缩的 Image（Image.gz）解压到这里，不查保留区；0x0d080000 到 secmon@10000000（no-map，BL31）只有 47.5 MiB，解压后更大的 arm64 Image 会写进 BL31。0x20000000 到启动脚本的暂存区 0x2d000000 有 208 MiB，下方 ramdisk_addr_r（0x13000000）到这里也有 208 MiB；未压缩的 Image 不经过这里。meson64.h 的 CFG_EXTRA_ENV_SETTINGS 在 env_default.h 展开时才取 KERNEL_COMP_ADDR_R，所以在包含之后重定义即生效；构建门禁核对 u-boot-initial-env 里的实际值。
 */
#ifndef __CM211_1_CONFIG_H
#define __CM211_1_CONFIG_H

#define BOOT_TARGET_DEVICES(func) \
	func(USB, usb, 0) \
	func(MMC, mmc, 0) \
	func(MMC, mmc, 1) \
	BOOT_TARGET_PXE(func) \
	BOOT_TARGET_DHCP(func)

#include <configs/meson64.h>

#undef KERNEL_COMP_ADDR_R
#define KERNEL_COMP_ADDR_R	__stringify(0x20000000)

#endif /* __CM211_1_CONFIG_H */
