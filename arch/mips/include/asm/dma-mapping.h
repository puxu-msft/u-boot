/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_MIPS_DMA_MAPPING_H
#define __ASM_MIPS_DMA_MAPPING_H

#include <cpu_func.h>
#include <malloc.h>
#include <asm/addrspace.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/types.h>

/*
 * 一致内存（同 Linux 非一致 MIPS 的 arch_dma_prep_coherent 加 arch_dma_set_uncached）：从堆里按 ARCH_DMA_MINALIGN 对齐分配、长度向上取整到整行，这段内存在各级缓存里的行先写回作废，之后 CPU 只经返回的 KSEG1 不缓存别名访问，与设备之间不需要 dma_map/unmap；*handle 是交给设备的物理地址。
 * 返回的是同一段内存的 KSEG1 别名而不是 map_physmem(MAP_NOCACHE)：后者经 ioremap 可能由平台给出别的映射（如 Octeon 的 XKPHYS），dma_free_coherent 就换不回 memalign 给的 KSEG0 指针；堆在 KSEG0 里，KSEG0/KSEG1 别名互换是确定的。
 */
static inline void *dma_alloc_coherent(size_t len, unsigned long *handle)
{
	void *vaddr;

	len = ALIGN(len, ARCH_DMA_MINALIGN);
	vaddr = memalign(ARCH_DMA_MINALIGN, len);
	if (!vaddr)
		return NULL;

	flush_dcache_range((ulong)vaddr, (ulong)vaddr + len);
	*handle = virt_to_phys(vaddr);

	return (void *)CKSEG1ADDR((ulong)vaddr);
}

static inline void dma_free_coherent(void *addr)
{
	free((void *)CKSEG0ADDR((ulong)addr));
}

#endif /* __ASM_MIPS_DMA_MAPPING_H */
