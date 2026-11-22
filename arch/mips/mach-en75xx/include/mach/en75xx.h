/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Generic register map for the EcoNet/Airoha EN75xx MIPS SoCs.
 *
 * The EN7512/EN7521 (MIPS 34Kc, big-endian) and the EN7528 (MIPS 1004Kc,
 * little-endian) share the same TCBoot-derived boot model and almost the
 * same peripheral register map. Values that differ between the two are
 * selected here on CONFIG_SOC_EN7528.
 */
#ifndef __MACH_EN75XX_H__
#define __MACH_EN75XX_H__

/* --- Peripheral bases (identical across the family) --- */
#define EN75XX_FLASH_BASE		0xbfc00000
#define EN75XX_SFC_BASE			0xbfa10000
#define EN75XX_CHIP_SCU_BASE		0xbfa20000
#define EN75XX_FE_SRAM_BASE		0xbfa30000
#define EN75XX_FE_SRAM_CACHED		0x9fa30000
#define EN75XX_SYSCTL_BASE		0xbfb00000
#define EN75XX_SMC_BASE			0xbfb10000
#define EN75XX_DRAMC_BASE		0xbfb20000
#define EN75XX_INTC_BASE		0xbfb40000
#define EN75XX_UART0_BASE		0xbfbf0000
#define EN75XX_TIMER_BASE		0xbfbf0100
#define EN75XX_WDT_CTRL			0xbfbf0100	/* airoha WDT; writel(0) stops it */
#define EN75XX_GPIO_BASE		0xbfbf0200

/* --- SYSCTL scratch/handoff registers (identical) --- */
#define EN75XX_RESET_CONTROL		(EN75XX_SYSCTL_BASE + 0x040)
#define EN75XX_FE_SRAM_SEL		(EN75XX_SYSCTL_BASE + 0x958)
#define EN75XX_SCREG_WR0		(EN75XX_SYSCTL_BASE + 0x280)
#define EN75XX_REG_SAVE_INFO		(EN75XX_SYSCTL_BASE + 0x284)

/* 物理地址低端窗口里能当 RAM 用的上限（448 MiB）：0x1c000000 起留给外设映射（寄存器岛在 0x1fa00000 起），512 MiB 的 DDR 只用得到前 448 MiB。厂商内核按 448 MiB 初始化 QDMA 缓冲（cal_qdma_init_buffer(448)），Linux 侧各板 DTS 的 /memory 也是这个值 */
#define EN75XX_RAM_WINDOW_SIZE		0x1c000000

/* --- Flash image layout: DDR blob offset is common; SPL/U-Boot offsets
 *     differ because the EN7528 DDR blob is much larger (see below). --- */
#define EN75XX_DDR_BLOB_OFFSET		0x00008000

/* --- DRAM-size / clock info in REG_SAVE_INFO (0xbfb00284).
 *     Field positions differ per SoC - see the EN7528/EN7512 blocks below.
 *     Read as: (val >> _SHIFT) & _MASK. --- */

#if defined(CONFIG_SOC_EN7528)
/*
 * EN7528 (MIPS 1004Kc, little-endian).
 *
 * The DDR stage (en7528_ddr.bin, extracted from the vendor
 * bootloader) is the full FE-SRAM first stage: it self-inits SoC registers,
 * calibrates DDR3 and hands control back via *(EN75XX_SCREG_WR0). It is
 * loaded at the FE-SRAM base 0x9fa30000 (NOT +0x2800 like EN7512) and is
 * 0x5a70 bytes.
 *
 * NOTE (hardware bring-up): the blob's own entry word is `j 0xbfc00060`,
 * i.e. it is a plain subroutine: entry 0x9fa30000 = NOP pad -> stub @+0x290
 * -> main(), and one jalr *(SCREG_WR0) hands control back to us.
 * The exact entry/handoff must be validated over UART — see
 * doc/board/econet/en7528.rst. The constants below build a coherent
 * image and are the starting point for that bring-up.
 */
#define EN75XX_DDR_BLOB_ADDR		(EN75XX_FE_SRAM_CACHED + 0x0000)
#define EN75XX_DDR_BLOB_SIZE		0x00005a70
/* The 0xf0a0-byte blob at flash 0x8000 ends at 0x170a0, so SPL starts at
 * 0x20000 (EN7512 could use 0x10000 with its 0x4f70 blob). */
#define EN75XX_SPL_IMAGE_OFFSET		0x00020000
#define EN75XX_UBOOT_IMAGE_OFFSET	0x00050000
/* Keep TPL scratch clear of the (large) EN7528 blob image. */
#define EN75XX_SPL_HEADER_ADDR		(EN75XX_FE_SRAM_BASE + 0x0f000)
#define EN75XX_TPL_STACK_ADDR		(EN75XX_FE_SRAM_BASE + 0x0fff0)
#define EN75XX_TPL_SAVED_RA		(EN75XX_FE_SRAM_BASE + 0x0ff00)
#define EN75XX_SAVE_DRAM_SHIFT		20
#define EN75XX_SAVE_DRAM_MASK		0xfff		/* MB, bits [31:20] */
#define EN75XX_SAVE_CLK_SHIFT		10
#define EN75XX_SAVE_CLK_MASK		0x3ff		/* MHz, bits [19:10] */
#define EN75XX_COUNT_HZ_PER_CLK	2000000	/* CP0 Count = clk_field * 2 MHz (measured) */
#else
/*
 * EN7512/EN7521 (MIPS 34Kc, big-endian). The vendor spram DDR-only stage
 * is linked for and entered at 0x9fa32800 and is 0x4f70 bytes.
 */
#define EN75XX_DDR_BLOB_ADDR		(EN75XX_FE_SRAM_CACHED + 0x2800)
#define EN75XX_DDR_BLOB_SIZE		0x00004f70
#define EN75XX_SPL_IMAGE_OFFSET		0x00010000
#define EN75XX_UBOOT_IMAGE_OFFSET	0x00050000
#define EN75XX_SPL_HEADER_ADDR		(EN75XX_FE_SRAM_BASE + 0x40)
#define EN75XX_TPL_STACK_ADDR		(EN75XX_FE_SRAM_BASE + 0xbff0)
#define EN75XX_TPL_SAVED_RA		(EN75XX_FE_SRAM_BASE + 0x00)
#define EN75XX_SAVE_DRAM_SHIFT		0
#define EN75XX_SAVE_DRAM_MASK		0xfff		/* MB, bits [11:0] */
#define EN75XX_SAVE_CLK_SHIFT		12
#define EN75XX_SAVE_CLK_MASK		0xfff		/* MHz, bits [23:12] */
#define EN75XX_COUNT_HZ_PER_CLK	500000
#endif

#ifndef __ASSEMBLY__
#include <stddef.h>
#include <linux/types.h>

int en75xx_sfc_read(u32 offset, void *dst, size_t len);
int en75xx_sfc_init(void);
void en75xx_run_ddr_blob(void);
void en75xx_prepare_linux(void);
#endif

#endif /* __MACH_EN75XX_H__ */
