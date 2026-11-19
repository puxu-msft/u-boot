// SPDX-License-Identifier: GPL-2.0+
/*
 * EN75xx (EN7528) Linux-handoff helpers.
 *
 * 跳内核前关掉 WDT、让启动核加入 CM 一致性域。缓存维护用 arch/mips/lib/cache.c 的通用实现：EN7528 的 L2（Config2：1024 组 × 8 路 × 32 B = 256 KB）是写回式，平台选上 MIPS_L2_CACHE，行长由 Config2 探测，flush/invalidate 逐级处理 L1 与 L2。
 */

#include <bootm.h>
#include <stdio.h>
#include <asm/addrspace.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/system.h>
#include <mach/en75xx.h>

#define EN75XX_GCR_CL_COHERENCE        0x2008

void en75xx_prepare_linux(void)
{
	unsigned long cmgcr, cm;

	/* Airoha WDT — same block as EN75XX_TIMER_BASE. */
	writel(0, (void __iomem *)EN75XX_WDT_CTRL);

	/*
	 * mips_cm_map() never writes GCR_CL_COHERENCE, so the boot core is
	 * out of the coherent domain and a CM/CPS kernel's vector fetches
	 * go stale. Same join as arch/mips/mach-mtmips/mt7621.
	 */
	cmgcr = read_c0_cmgcrbase();
	cm = ((unsigned long)cmgcr << 4) + CKSEG1;
	writel(0xff, (void __iomem *)(cm + EN75XX_GCR_CL_COHERENCE));
	instruction_hazard_barrier();
	printf("## EN75xx: WDT off, CM COHERENCE=0xff GCR@%08lx\n", cm);
}

void arch_preboot_os(void)
{
	en75xx_prepare_linux();
}
