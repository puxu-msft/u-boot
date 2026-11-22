// SPDX-License-Identifier: GPL-2.0+
/*
 * EN75xx (EN7528) Linux-handoff helpers.
 *
 * 跳内核前关掉看门狗以外的定时器、让启动核加入 CM 一致性域。缓存维护用 arch/mips/lib/cache.c 的通用实现：EN7528 的 L2（Config2：1024 组 × 8 路 × 32 B = 256 KB）是写回式，平台选上 MIPS_L2_CACHE，行长由 Config2 探测，flush/invalidate 逐级处理 L1 与 L2。
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

	/*
	 * 定时器块里只留看门狗（timer 5 与复位使能），其余定时器关掉。看门狗在跑就带着进内核，由 Linux 的
	 * en7528_wdt 接手代喂（它要两位同时置位才认为在跑）；没开时这两位本来就是 0，等于整个清零。没编看门狗
	 * 驱动的板子照旧整个清零：上一级引导留下的看门狗没人管，不带进内核。
	 */
	u32 keep = IS_ENABLED(CONFIG_WDT_EN7528) ?
		   EN75XX_WDT_TIMER_ENABLE | EN75XX_WDT_RESET_ENABLE : 0;
	u32 wdt;

	writel(readl((void __iomem *)EN75XX_WDT_CTRL) & keep,
	       (void __iomem *)EN75XX_WDT_CTRL);
	wdt = readl((void __iomem *)EN75XX_WDT_CTRL);

	/*
	 * mips_cm_map() never writes GCR_CL_COHERENCE, so the boot core is
	 * out of the coherent domain and a CM/CPS kernel's vector fetches
	 * go stale. Same join as arch/mips/mach-mtmips/mt7621.
	 */
	cmgcr = read_c0_cmgcrbase();
	cm = ((unsigned long)cmgcr << 4) + CKSEG1;
	writel(0xff, (void __iomem *)(cm + EN75XX_GCR_CL_COHERENCE));
	instruction_hazard_barrier();
	printf("## EN75xx: timers off (watchdog %s), CM COHERENCE=0xff GCR@%08lx\n",
	       (wdt & keep) == (EN75XX_WDT_TIMER_ENABLE | EN75XX_WDT_RESET_ENABLE) ?
	       "kept running" : "off", cm);
}

void arch_preboot_os(void)
{
	en75xx_prepare_linux();
}
