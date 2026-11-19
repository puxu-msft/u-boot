// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, <wd@denx.de>
 */

#include <cpu_func.h>
#include <malloc.h>
#include <asm/cache.h>
#include <asm/cacheops.h>
#include <asm/cm.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/system.h>
#include <linux/bug.h>

DECLARE_GLOBAL_DATA_PTR;

static void probe_l2(void)
{
#ifdef CONFIG_MIPS_L2_CACHE
	unsigned long conf2, sl;
	bool l2c = false;

	if (!(read_c0_config1() & MIPS_CONF_M))
		return;

	conf2 = read_c0_config2();

	if (__mips_isa_rev >= 6) {
		l2c = conf2 & MIPS_CONF_M;
		if (l2c)
			l2c = read_c0_config3() & MIPS_CONF_M;
		if (l2c)
			l2c = read_c0_config4() & MIPS_CONF_M;
		if (l2c)
			l2c = read_c0_config5() & MIPS_CONF5_L2C;
	}

	if (l2c && IS_ENABLED(CONFIG_MIPS_CM)) {
		gd->arch.l2_line_size = mips_cm_l2_line_size();
	} else if (l2c) {
		/* We don't know how to retrieve L2 config on this system */
		BUG();
	} else {
		sl = (conf2 & MIPS_CONF2_SL) >> MIPS_CONF2_SL_SHF;
		gd->arch.l2_line_size = sl ? (2 << sl) : 0;
	}
#endif
}

void mips_cache_probe(void)
{
#ifdef CONFIG_SYS_CACHE_SIZE_AUTO
	unsigned long conf1, il, dl;

	conf1 = read_c0_config1();

	il = (conf1 & MIPS_CONF1_IL) >> MIPS_CONF1_IL_SHF;
	dl = (conf1 & MIPS_CONF1_DL) >> MIPS_CONF1_DL_SHF;

	gd->arch.l1i_line_size = il ? (2 << il) : 0;
	gd->arch.l1d_line_size = dl ? (2 << dl) : 0;
#endif
	probe_l2();
}

static inline unsigned long icache_line_size(void)
{
#ifdef CONFIG_SYS_CACHE_SIZE_AUTO
	return gd->arch.l1i_line_size;
#else
	return CONFIG_SYS_ICACHE_LINE_SIZE;
#endif
}

static inline unsigned long dcache_line_size(void)
{
#ifdef CONFIG_SYS_CACHE_SIZE_AUTO
	return gd->arch.l1d_line_size;
#else
	return CONFIG_SYS_DCACHE_LINE_SIZE;
#endif
}

static inline unsigned long scache_line_size(void)
{
#ifdef CONFIG_MIPS_L2_CACHE
	return gd->arch.l2_line_size;
#else
	return CONFIG_SYS_SCACHE_LINE_SIZE;
#endif
}

#define cache_loop(start, end, lsize, ops...) do {			\
	const void *addr = (const void *)(start & ~(lsize - 1));	\
	const void *aend = (const void *)((end - 1) & ~(lsize - 1));	\
	const unsigned int cache_ops[] = { ops };			\
	unsigned int i;							\
									\
	if (!lsize)							\
		break;							\
									\
	for (; addr <= aend; addr += lsize) {				\
		for (i = 0; i < ARRAY_SIZE(cache_ops); i++)		\
			mips_cache(cache_ops[i], addr);			\
	}								\
} while (0)

void __weak flush_cache(ulong start_addr, ulong size)
{
	unsigned long ilsize = icache_line_size();
	unsigned long dlsize = dcache_line_size();
	unsigned long slsize = scache_line_size();

	/* aend will be miscalculated when size is zero, so we return here */
	if (size == 0)
		return;

	if ((ilsize == dlsize) && !slsize) {
		/* flush I-cache & D-cache simultaneously */
		cache_loop(start_addr, start_addr + size, ilsize,
			   HIT_WRITEBACK_INV_D, HIT_INVALIDATE_I);
		goto ops_done;
	}

	/* flush D-cache */
	cache_loop(start_addr, start_addr + size, dlsize, HIT_WRITEBACK_INV_D);

	/* flush L2 cache */
	cache_loop(start_addr, start_addr + size, slsize, HIT_WRITEBACK_INV_SD);

	/* flush I-cache */
	cache_loop(start_addr, start_addr + size, ilsize, HIT_INVALIDATE_I);

ops_done:
	/* ensure cache ops complete before any further memory accesses */
	sync();

	/* ensure the pipeline doesn't contain now-invalid instructions */
	instruction_hazard_barrier();
}

void __weak flush_dcache_range(ulong start_addr, ulong stop)
{
	unsigned long lsize = dcache_line_size();
	unsigned long slsize = scache_line_size();

	/* aend will be miscalculated when size is zero, so we return here */
	if (start_addr == stop)
		return;

	cache_loop(start_addr, stop, lsize, HIT_WRITEBACK_INV_D);

	/* flush L2 cache */
	cache_loop(start_addr, stop, slsize, HIT_WRITEBACK_INV_SD);

	/* ensure cache ops complete before any further memory accesses */
	sync();
}

/*
 * 范围首尾所在的缓存行若有一部分在 [start, stop) 之外，对这一行执行 op（写回作废）：同一行里范围外的字节属于别的对象，可能还是脏的，直接作废会把它们丢掉。
 */
#define cache_wbinv_partial_lines(start, stop, lsize, op) do {		\
	if (!(lsize))							\
		break;							\
	if ((start) & ((lsize) - 1))					\
		mips_cache(op, (const void *)((start) & ~((lsize) - 1)));	\
	if ((stop) & ((lsize) - 1))					\
		mips_cache(op, (const void *)(((stop) - 1) & ~((lsize) - 1))); \
} while (0)

void __weak invalidate_dcache_range(ulong start_addr, ulong stop)
{
	unsigned long lsize = dcache_line_size();
	unsigned long slsize = scache_line_size();

	/* aend will be miscalculated when size is zero, so we return here */
	if (start_addr == stop)
		return;

	/*
	 * 先把只有一部分在范围内的首尾两行写回作废，再整段作废，同 Linux arch/mips/mm/c-r4k.c 的 r4k_dma_cache_inv。L1 在前：它写回的脏数据落在 L2，紧接着由 L2 的写回送到内存。整段作废同样先 L1 后 L2，免得 L1 里范围内的脏行在 L2 作废之后才被逐出、又落进 L2。
	 */
	cache_wbinv_partial_lines(start_addr, stop, lsize, HIT_WRITEBACK_INV_D);
	cache_wbinv_partial_lines(start_addr, stop, slsize, HIT_WRITEBACK_INV_SD);

	cache_loop(start_addr, stop, lsize, HIT_INVALIDATE_D);

	/* invalidate L2 cache */
	cache_loop(start_addr, stop, slsize, HIT_INVALIDATE_SD);

	/* ensure cache ops complete before any further memory accesses */
	sync();
}

int dcache_status(void)
{
	unsigned int cca = read_c0_config() & CONF_CM_CMASK;
	return cca != CONF_CM_UNCACHED;
}

void dcache_enable(void)
{
	puts("Not supported!\n");
}

void dcache_disable(void)
{
	/* change CCA to uncached */
	change_c0_config(CONF_CM_CMASK, CONF_CM_UNCACHED);

	/* ensure the pipeline doesn't contain now-invalid instructions */
	instruction_hazard_barrier();
}

#ifdef CONFIG_SYS_NONCACHED_MEMORY
static unsigned long noncached_start;
static unsigned long noncached_end;
static unsigned long noncached_next;

void noncached_set_region(void)
{
}

int noncached_init(void)
{
	phys_addr_t start, end;
	size_t size;

	/* If this calculation changes, update board_f.c:reserve_noncached() */
	end = ALIGN(mem_malloc_start, MMU_SECTION_SIZE) - MMU_SECTION_SIZE;
	size = ALIGN(CONFIG_SYS_NONCACHED_MEMORY, MMU_SECTION_SIZE);
	start = end - size;

	debug("mapping memory %pa-%pa non-cached\n", &start, &end);

	noncached_start = start;
	noncached_end = end;
	noncached_next = start;

	return 0;
}

phys_addr_t noncached_alloc(size_t size, size_t align)
{
	phys_addr_t next = ALIGN(noncached_next, align);

	if (next >= noncached_end || (noncached_end - next) < size)
		return 0;

	debug("allocated %zu bytes of uncached memory @%pa\n", size, &next);
	noncached_next = next + size;

	return CKSEG1ADDR(next);
}
#endif /* CONFIG_SYS_NONCACHED_MEMORY */
