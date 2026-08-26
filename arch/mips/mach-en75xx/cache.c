// SPDX-License-Identifier: GPL-2.0+
/*
 * EN75xx (EN7528) cache and Linux-handoff helpers.
 *
 * The CM L2 on this part hangs on Hit_Writeback_Inv_SD (cache op 0x17).
 * Override the generic MIPS flush routines so they never issue that op,
 * decompress kernels through KSEG1, and join the CM coherent domain
 * before jumping to Linux.
 */

#include <cpu_func.h>
#include <image.h>
#include <stdio.h>
#include <asm/addrspace.h>
#include <asm/cache.h>
#include <asm/cacheops.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/system.h>
#include <mach/en75xx.h>

#define EN75XX_L1_LINE	32
#define EN75XX_L2_LINE	32
#define EN75XX_GCR_CL_COHERENCE	0x2008

#define cache_loop(start, end, lsize, op) do {				\
	const void *_addr = (const void *)((start) & ~((lsize) - 1));	\
	const void *_aend = (const void *)(((end) - 1) & ~((lsize) - 1)); \
	if (!(lsize) || (start) == (end))				\
		break;							\
	for (; _addr <= _aend; _addr += (lsize))			\
		mips_cache((op), _addr);				\
} while (0)

void flush_cache(ulong start_addr, ulong size)
{
	if (!size)
		return;

	/*
	 * Write back L1 D so U-Boot's own cached stores reach DRAM.
	 * Do not Hit_Writeback_Inv_SD (hangs). Do not invent an L2 line
	 * size here: Hit_Invalidate_SD on U-Boot's working set during
	 * DM init fails.
	 */
	cache_loop(start_addr, start_addr + size, EN75XX_L1_LINE,
		   HIT_WRITEBACK_INV_D);
	cache_loop(start_addr, start_addr + size, EN75XX_L1_LINE,
		   HIT_INVALIDATE_I);
	sync();
	instruction_hazard_barrier();
}

void flush_dcache_range(ulong start_addr, ulong stop)
{
	if (start_addr == stop)
		return;

	cache_loop(start_addr, stop, EN75XX_L1_LINE, HIT_WRITEBACK_INV_D);
	sync();
}

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
