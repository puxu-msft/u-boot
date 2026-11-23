// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 */

#include <cpu_func.h>
#include <fastboot.h>
#include <init.h>
#include <net.h>
#include <asm/arch/boot.h>
#include <env.h>
#include <errno.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <asm/ptrace.h>
#include <linux/libfdt.h>
#include <linux/err.h>
#include <asm/arch/mem.h>
#include <asm/arch/sm.h>
#include <asm/armv8/mmu.h>
#include <asm/unaligned.h>
#include <efi_loader.h>
#include <fdt_support.h>
#include <fdtdec.h>
#include <u-boot/crc.h>

#include <asm/psci.h>
#include <linux/psci.h>

DECLARE_GLOBAL_DATA_PTR;

__weak int board_init(void)
{
	return 0;
}

#ifndef CONFIG_SPL_BUILD
int dram_init(void)
{
	const fdt64_t *val;
	int offset;
	int len;

	offset = fdt_path_offset(gd->fdt_blob, "/memory");
	if (offset < 0)
		return -EINVAL;

	val = fdt_getprop(gd->fdt_blob, offset, "reg", &len);
	if (len < sizeof(*val) * 2)
		return -EINVAL;

	/* Use unaligned access since cache is still disabled */
	gd->ram_size = get_unaligned_be64(&val[1]);

	return 0;
}
#endif

__weak int meson_ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	meson_init_reserved_memory(blob);

	return meson_ft_board_setup(blob, bd);
}

/*
 * DT 的 /reserved-memory 已经把 [start, start + size) 整段交给某个启用的子节点时返回 true，两种情形：
 * - 静态区：reg 覆盖这段，且节点不带 reusable。reusable 允许内核把这段借给别的用途（静态 CMA 就是交给伙伴系统放可迁移页），不算保留。
 * - 动态池：alloc-ranges 恰好一段、覆盖这段，且池的 size 等于这段的长度，池只能整段落在这里。alloc-ranges 有多段时 Linux 可以把池分在别段，这段就没人保留。
 * 这时再加 /memreserve/ 是多余的，对动态池还有害：Linux 先按 /memreserve/ 占住内存，再在 alloc-ranges 里给池分配，区间已被占就分配失败。例如 Amlogic 厂商内核 DT 的 linux,secmon 是 alloc-ranges = <0x05000000 0x400000>、size = 0x400000 的 shared-dma-pool，正好压在 BL31 上，secmon 驱动要从它分配与安全世界的共享内存。
 * 动态池带 reusable 时同样接受，这是为与厂商 BL33 一致：厂商的 rsvmem check 把 linux,secmon 改成单段 alloc-ranges = BL31 区、size 按 BL31 大小取整到 4 MiB，不加 /memreserve/，而这个池在厂商 DT 里是 reusable 的。代价是池被分配之前，这段可能暂放可迁移页。
 * 子节点的 size 与 alloc-ranges 按 /reserved-memory 的 #address-cells/#size-cells 解析（reserved-memory 绑定要求它们与根节点相同）。
 */
static bool meson_fdt_rsvmem_covers(const void *fdt, u64 start, u64 size)
{
	const fdt32_t *p, *psize;
	int parent, node, ac, sc, len, slen, i;
	u64 end = start + size, base, length;

	parent = fdt_path_offset(fdt, "/reserved-memory");
	if (parent < 0)
		return false;
	ac = fdt_address_cells(fdt, parent);
	sc = fdt_size_cells(fdt, parent);
	if (ac < 1 || ac > 2 || sc < 1 || sc > 2)
		return false;

	fdt_for_each_subnode(node, fdt, parent) {
		if (!fdtdec_get_is_enabled(fdt, node))
			continue;

		p = fdt_getprop(fdt, node, "reg", &len);
		if (fdt_getprop(fdt, node, "reusable", NULL))
			p = NULL;
		for (i = 0; p && (i + ac + sc) * 4 <= len; i += ac + sc) {
			base = fdt_read_number(p + i, ac);
			length = fdt_read_number(p + i + ac, sc);
			if (base <= start && end <= base + length)
				return true;
		}

		psize = fdt_getprop(fdt, node, "size", &slen);
		p = fdt_getprop(fdt, node, "alloc-ranges", &len);
		if (!psize || slen != sc * 4 || !p || len != (ac + sc) * 4)
			continue;
		base = fdt_read_number(p, ac);
		length = fdt_read_number(p + ac, sc);
		if (base <= start && end <= base + length &&
		    fdt_read_number(psize, sc) == length)
			return true;
	}

	return false;
}

void meson_board_add_reserved_memory(void *fdt, u64 start, u64 size)
{
	int ret;

	if (meson_fdt_rsvmem_covers(fdt, start, size)) {
		debug("Zone @ 0x%llx already described by /reserved-memory\n",
		      start);
	} else {
		ret = fdt_add_mem_rsv(fdt, start, size);
		if (ret)
			printf("Could not reserve zone @ 0x%llx\n", start);
	}

	if (IS_ENABLED(CONFIG_EFI_LOADER))
		efi_add_memory_map(start, size, EFI_RESERVED_MEMORY_TYPE);
}

int meson_generate_serial_ethaddr(void)
{
	u8 mac_addr[ARP_HLEN];
	char serial[SM_SERIAL_SIZE];
	u32 sid;
	u16 sid16;

	if (!meson_sm_get_serial(serial, SM_SERIAL_SIZE)) {
		sid = crc32(0, (unsigned char *)serial, SM_SERIAL_SIZE);
		sid16 = crc16_ccitt(0, (unsigned char *)serial,	SM_SERIAL_SIZE);

		/* Ensure the NIC specific bytes of the mac are not all 0 */
		if ((sid & 0xffffff) == 0)
			sid |= 0x800000;

		/* Non OUI / registered MAC address */
		mac_addr[0] = ((sid16 >> 8) & 0xfc) | 0x02;
		mac_addr[1] = (sid16 >>  0) & 0xff;
		mac_addr[2] = (sid >> 24) & 0xff;
		mac_addr[3] = (sid >> 16) & 0xff;
		mac_addr[4] = (sid >>  8) & 0xff;
		mac_addr[5] = (sid >>  0) & 0xff;

		eth_env_set_enetaddr("ethaddr", mac_addr);
	} else
		return -EINVAL;

	return 0;
}

static void meson_set_boot_source(void)
{
	const char *source;

	switch (meson_get_boot_device()) {
	case BOOT_DEVICE_EMMC:
		source = "emmc";
		break;

	case BOOT_DEVICE_NAND:
		source = "nand";
		break;

	case BOOT_DEVICE_SPI:
		source = "spi";
		break;

	case BOOT_DEVICE_SD:
		source = "sd";
		break;

	case BOOT_DEVICE_USB:
		source = "usb";
		break;

	default:
		source = "unknown";
	}

	env_set("boot_source", source);
}

__weak int meson_board_late_init(void)
{
	return 0;
}

int board_late_init(void)
{
	meson_set_boot_source();

	if (CONFIG_IS_ENABLED(DFU) && CONFIG_IS_ENABLED(EFI_LOADER)) {
		/* Generate dfu_string for EFI capsule updates */
		meson_setup_capsule();
	}

	return meson_board_late_init();
}

#if CONFIG_IS_ENABLED(FASTBOOT) && !defined(CONFIG_XPL)
/*
 * fastboot reboot-bootloader/reboot-fastboot/reboot-recovery 的重启标记：不写 misc 的 BCB（厂商 u-boot 在线刷探测之前读它，chainload 期间写进去会改变下一次厂商路径的启动），改为 Amlogic 重启原因，随 PSCI SYSTEM_RESET 的 x1 交给 BL31（它无条件把 x1 写成原因，先经 SiP 0x82000049 设的会被覆盖）。
 * 取值按厂商原因表：bootloader 与 fastboot 都用 fastboot（4）——厂商 BL33 见到 4 进它的 fastboot，见到 bootloader（7）照常启动；recovery 用 2。主线作 BL33 时由 m12.env 的冷启动入口分流（4 → fastboot 后进救援，2 暂按普通启动）；chainload 下复位后先跑厂商 BL33，由它按原因处理
 */
static unsigned long meson_reboot_reason;

int fastboot_set_reboot_flag(enum fastboot_reboot_reason reason)
{
	switch (reason) {
	case FASTBOOT_REBOOT_REASON_BOOTLOADER:
	case FASTBOOT_REBOOT_REASON_FASTBOOTD:
		meson_reboot_reason = REBOOT_REASON_FASTBOOT;
		return 0;
	case FASTBOOT_REBOOT_REASON_RECOVERY:
		meson_reboot_reason = REBOOT_REASON_RECOVERY;
		return 0;
	default:
		return -EINVAL;
	}
}

unsigned long psci_sys_reset_arg(void)
{
	return meson_reboot_reason;
}
#endif

#if defined(CONFIG_XPL) || !CONFIG_IS_ENABLED(SYSRESET)
void reset_cpu(void)
{
	/*
	 * We do not have BL31 running yet, so no PSCI.
	 * Instead, let the watchdog reset the board.
	 */
	for (;;)
		;
}
#endif
