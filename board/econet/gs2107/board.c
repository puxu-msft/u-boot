/* SPDX-License-Identifier: GPL-2.0+ */

#include <bootm.h>
#include <dm.h>
#include <env.h>
#include <init.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <stdio.h>

#if CONFIG_IS_ENABLED(AIROHA_ETH)
extern U_BOOT_DRIVER(airoha_eth);
#endif

/* Set to 0 if the EN7528 USB T-PHY pokes hang on a particular GS2107 board revision. */
#ifndef GS2107_USB_INIT
#define GS2107_USB_INIT 1
#endif

#if defined(CONFIG_USB_XHCI_MTK) && GS2107_USB_INIT
#define EN7528_SCU_RST_CTRL1 0xbfb00834
#define EN7528_SCU_RST_USB 0xbfb000ec
#define EN7528_RST_USB_HOST_P0 BIT(22)
#define EN7528_RST_USB_PHY_P0 BIT(6)
#define EN7528_RST_USB_PHY_P1 BIT(7)
#define EN7528_USB_PHY_BASE 0xbfa80000
#define EN7528_USB_PHY_U2_P0 0x300
#define EN7528_USB_PHY_U2_P1 0x1300
#define EN7528_USB_PHY_ACR3 0x1c
#define EN7528_USB_PHY_ACR3_ENABLE 0xc0240000
#define SCU_R(off) __raw_readl((void __iomem *)(0xbfb00000 + (off)))
#define SCU_W(off, v) __raw_writel((v), (void __iomem *)(0xbfb00000 + (off)))

static void en7528_usb_init(void)
{
	u32 v;

	/* GS2107 USB2 T-PHY bring-up copied from the v1 EN7528 port; the SCU sequence is from the vendor MU3H path. */
	v = SCU_R(0x90);
	SCU_W(0x90, (v & 0x9fffffff) | 0x60000000);
	mdelay(10);
	v = SCU_R(0xa8);
	SCU_W(0xa8, v | 0x600);
	v = SCU_R(0xac);
	SCU_W(0xac, v & ~0x30000000);
	v = SCU_R(0xa8);
	SCU_W(0xa8, v | 0x6000);
	v = SCU_R(0xa8);
	SCU_W(0xa8, v & ~0x1800);
	v = SCU_R(0xac);
	SCU_W(0xac, v & ~0x08000000);
	v = SCU_R(0xec);
	SCU_W(0xec, v & ~0x80000000);
	mdelay(1);
	v = SCU_R(0xec);
	SCU_W(0xec, v | 0x80000000);
	mdelay(6);
	v = SCU_R(0x834);
	SCU_W(0x834, v & ~EN7528_RST_USB_HOST_P0);
	v = SCU_R(0xec);
	SCU_W(0xec, v & ~(EN7528_RST_USB_PHY_P0 | EN7528_RST_USB_PHY_P1));
	mdelay(1);
	__raw_writel(EN7528_USB_PHY_ACR3_ENABLE, (void __iomem *)(EN7528_USB_PHY_BASE + EN7528_USB_PHY_U2_P0 + EN7528_USB_PHY_ACR3));
	__raw_writel(EN7528_USB_PHY_ACR3_ENABLE, (void __iomem *)(EN7528_USB_PHY_BASE + EN7528_USB_PHY_U2_P1 + EN7528_USB_PHY_ACR3));
	mdelay(1);
}
#endif

/*
 * 启动看门狗：timer 5（寄存器模型见 devices/gs2107/docs/27 §1，与内核 en7528_wdt 一致）。
 * arch_preboot_os()（mach-en75xx/preboot.c 的 en75xx_prepare_linux）在跳内核前把 CTRL 整个清零，
 * 所以要在其后的 board_preboot_os() 里按需重新打开：默认 env 的 gs_nand_boot/gs_rescue_boot 在 bootm 前
 * setenv gs_wdt 1。LOAD=0xffffffff 在 112.5 MHz 下约 38 s，内核 en7528_wdt 发现计时器在跑就接手代喂，
 * 直到 procd 打开设备；这样内核早期卡死也会复位，bootcount 累加。gs_wdt 不在 env 可写白名单里，
 * 存储的 env 设不了它。
 */
#define GS2107_WDT_CTRL		0xbfbf0100
#define GS2107_WDT_LOAD		0xbfbf012c
#define GS2107_WDT_RELOAD	0xbfbf0138
#define GS2107_WDT_TIMER_ENABLE	BIT(5)
#define GS2107_WDT_IRQ_ENABLE	BIT(21)
#define GS2107_WDT_RESET_ENABLE	BIT(25)

void board_preboot_os(void)
{
	u32 ctrl;

	if (env_get_yesno("gs_wdt") != 1)
		return;
	__raw_writel(0xffffffff, (void __iomem *)GS2107_WDT_LOAD);
	ctrl = __raw_readl((void __iomem *)GS2107_WDT_CTRL);
	ctrl &= ~GS2107_WDT_IRQ_ENABLE;
	ctrl |= GS2107_WDT_TIMER_ENABLE | GS2107_WDT_RESET_ENABLE;
	__raw_writel(ctrl, (void __iomem *)GS2107_WDT_CTRL);
	__raw_writel(1, (void __iomem *)GS2107_WDT_RELOAD);
	printf("## GS2107: boot watchdog armed (~38 s, CTRL=%08x)\n", ctrl);
}

int board_init(void)
{
	return 0;
}

int board_late_init(void)
{
#if defined(CONFIG_USB_XHCI_MTK) && GS2107_USB_INIT
	en7528_usb_init();
#endif
#if CONFIG_IS_ENABLED(AIROHA_ETH)
	struct udevice *dev;
	int ret;

	/* Probe the EN751221-family Ethernet parent after the console is up and just before initr_net() enumerates UCLASS_ETH devices. */
	ret = uclass_get_device_by_driver(UCLASS_MISC, DM_DRIVER_GET(airoha_eth), &dev);
	if (ret && ret != -ENODEV)
		printf("GS2107 Ethernet probe failed: %d\n", ret);
#endif
	return 0;
}
