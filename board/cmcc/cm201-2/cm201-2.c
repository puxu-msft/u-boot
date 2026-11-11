// SPDX-License-Identifier: GPL-2.0+
/*
 * CM201-2（Hi3798MV300）的板级初始化。
 *
 * U-Boot 没有这个 SoC 的 CRG 时钟驱动，因此设备树给 eMMC 与 FEMAC 配固定时钟，由本文件让 CRG 与之一致。在 TF-A 之前运行的厂商 fastboot 会让 eMMC 控制器停在它最后用过的时序模式（其 CIU mux 会切到 50、100 或 200 MHz，HS400 还会设置控制器侧的 DDR 与 DLL 状态），核心热复位不触及其中任何一项，所以这里把控制器恢复到已知状态：CIU 设为 50 MHz 并用 fastboot 自己在该速率下用的相位，采样 DLL 退出 tuning 模式，再给控制器一个软复位脉冲，把厂商寄存器（DDR_REG、ENABLE_SHIFT、TUNING_CTRL）恢复为默认值。
 *
 * 寄存器布局：PERI_CRG40（0xa0）bit 0 BIU 门控，bit 1 CIU 门控，bit 4 软复位，bits 10:8 CIU mux（0 100M，1 50M，2 25M，3 200M，6 150M），bits 14:12 采样相位，bits 18:16 驱动相位（步进 45 度）；PERI_CRG231（0x39c）bit 16 SAP DLL 模式（1 = 不在 tuning）；PERI_CRG52（0xd0）bit 0 FEMAC 接口门控，bit 1 FEMAC 核心门控；PERI_CRG226（0x388）bit 0 FE PHY 门控。出处：Linux hi3798mv200 CRG 驱动与厂商 fastboot 自己的 eMMC 时钟例程。
 */

#include <init.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>

#define CRG_BASE		0xf8a22000UL

#define PERI_CRG40_EMMC		(CRG_BASE + 0xa0)
#define  EMMC_BIU_CKEN		BIT(0)
#define  EMMC_CIU_CKEN		BIT(1)
#define  EMMC_SRST_REQ		BIT(4)
#define  EMMC_CLK_SEL		GENMASK(10, 8)
#define  EMMC_CLK_SEL_50M	1
#define  EMMC_SAMPLE_PHASE	GENMASK(14, 12)
#define  EMMC_DRV_PHASE		GENMASK(18, 16)
#define  EMMC_PHASE_0		0
#define  EMMC_PHASE_180		4

#define PERI_CRG231_SAP_DLL	(CRG_BASE + 0x39c)
#define  SAP_DLL_DLLMODE	BIT(16)

#define PERI_CRG52_FEMAC	(CRG_BASE + 0xd0)
#define  FEMAC_BUS_CKEN		BIT(0)
#define  FEMAC_CKEN		BIT(1)

#define PERI_CRG226_FEPHY	(CRG_BASE + 0x388)
#define  FEPHY_CKEN		BIT(0)

static void cm201_emmc_clock_init(void)
{
	u32 val = readl(PERI_CRG40_EMMC);

	val &= ~(EMMC_CLK_SEL | EMMC_SAMPLE_PHASE | EMMC_DRV_PHASE);
	val |= FIELD_PREP(EMMC_CLK_SEL, EMMC_CLK_SEL_50M) |
	       FIELD_PREP(EMMC_SAMPLE_PHASE, EMMC_PHASE_0) |
	       FIELD_PREP(EMMC_DRV_PHASE, EMMC_PHASE_180);

	setbits_le32(PERI_CRG231_SAP_DLL, SAP_DLL_DLLMODE);

	/* 与厂商 fastboot 初始化时给该模块的脉冲相同。 */
	writel(val | EMMC_SRST_REQ, PERI_CRG40_EMMC);
	udelay(1000);
	writel((val & ~EMMC_SRST_REQ) | EMMC_BIU_CKEN | EMMC_CIU_CKEN,
	       PERI_CRG40_EMMC);
	udelay(1000);
}

static void cm201_femac_clock_init(void)
{
	setbits_le32(PERI_CRG52_FEMAC, FEMAC_BUS_CKEN | FEMAC_CKEN);
	setbits_le32(PERI_CRG226_FEPHY, FEPHY_CKEN);
}

int board_init(void)
{
	cm201_emmc_clock_init();
	cm201_femac_clock_init();

	return 0;
}
