// SPDX-License-Identifier: GPL-2.0+

#include <init.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <mach/en75xx.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	/*
	 * The DDR stage (blob, or the stock bootbase on the chainload path)
	 * publishes the DRAM size in REG_SAVE_INFO. EN7528: MB in bits [31:20]
	 * (for example, 0x20038400 => 512 MiB).
	 */
	u32 val = __raw_readl((void __iomem *)EN75XX_REG_SAVE_INFO);
	u32 size_mb = (val >> EN75XX_SAVE_DRAM_SHIFT) & EN75XX_SAVE_DRAM_MASK;

	/*
	 * Some bootbases do not publish a valid DRAM size in REG_SAVE_INFO
	 * (the field reads back implausibly small). Fall back to sizing the
	 * DRAM at runtime in that case, probing through the uncached KSEG1
	 * alias to avoid cache-aliasing false positives and capping the scan
	 * at 256 MiB to stay clear of the SoC register island at 0x1fa00000.
	 */
	if (size_mb < 64)
		gd->ram_size = get_ram_size((long *)0xa0000000, SZ_256M);
	else
		gd->ram_size = (phys_size_t)size_mb << 20;

	/* gd->ram_size 只算低端窗口里的 RAM：LMB 按它划可加载区，bootm 按它改写内核 DT 的 /memory */
	gd->ram_size = min_t(phys_size_t, gd->ram_size, EN75XX_RAM_WINDOW_SIZE);

	return 0;
}

int print_cpuinfo(void)
{
	u32 val = __raw_readl((void __iomem *)EN75XX_REG_SAVE_INFO);
	u32 clk = (val >> EN75XX_SAVE_CLK_SHIFT) & EN75XX_SAVE_CLK_MASK;

#if defined(CONFIG_SOC_EN7528)
	printf("CPU:   EcoNet/Airoha EN7528 MIPS1004Kc\n");
#else
	printf("CPU:   EcoNet/Airoha EN7512/EN7521 MIPS34K\n");
#endif
	if (clk)
		printf("Clock: %u MHz\n", clk);
	else
		printf("Clock: unknown\n");

	return 0;
}

ulong notrace get_tbclk(void)
{
	u32 val = __raw_readl((void __iomem *)EN75XX_REG_SAVE_INFO);
	u32 clk = (val >> EN75XX_SAVE_CLK_SHIFT) & EN75XX_SAVE_CLK_MASK;

	/*
	 * CP0 Count rate per SoC: EN7528 = clk_field * 2 MHz (hardware-measured
	 * via sleep timing), EN7512 = clk_field / 2. See EN75XX_COUNT_HZ_PER_CLK.
	 */
	if (clk)
		return (ulong)clk * EN75XX_COUNT_HZ_PER_CLK;

	return CONFIG_SYS_MIPS_TIMER_FREQ;
}

phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
	/*
	 * DRAM is aliased into KSEG0 at 0x80000000; gd->ram_size is already
	 * clamped to the 448 MiB low window (dram_init), so the top would be
	 * 0x9c000000. Relocating into high DRAM (0x9exxxxxx) hung on this SoC
	 * (cache-flush over the large range / unstable high DRAM), and U-Boot
	 * gains nothing from a high top, so clamp to the proven-good 0x90000000
	 * (256 MiB); Linux still gets all of gd->ram_size through the /memory
	 * node bootm writes.
	 */
	phys_addr_t top = 0x80000000UL + gd->ram_size;

	if (top > 0x90000000UL)
		top = 0x90000000UL;

	return top;
}

void _machine_restart(void)
{
	__raw_writel(0x80000000, (void __iomem *)EN75XX_RESET_CONTROL);

	for (;;)
		;
}
