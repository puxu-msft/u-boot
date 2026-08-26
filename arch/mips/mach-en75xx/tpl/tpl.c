// SPDX-License-Identifier: GPL-2.0+

#include <compiler.h>
#include <linux/byteorder/generic.h>
#include <linux/types.h>
#include <asm/io.h>
#include <mach/en75xx.h>

#define IH_MAGIC	0x27051956
#define IH_HDR_SIZE	64

struct en75xx_legacy_header {
	u32 magic;
	u32 hcrc;
	u32 time;
	u32 size;
	u32 load;
	u32 ep;
	u32 dcrc;
	u8 os;
	u8 arch;
	u8 type;
	u8 comp;
	u8 name[32];
};

static u32 get_be32(u32 val)
{
	return be32_to_cpu(val);
}

/*
 * 16550 register access differs by SoC endianness:
 *   EN7512 (big-endian): 8-bit registers live in the top byte of each 32-bit
 *   word, i.e. at byte offset (reg << 2) + 3.
 *   EN7528 (little-endian, reg-io-width=4, reg-shift=2): 32-bit word access at
 *   (reg << 2); the 8-bit value sits in the low byte.
 */
#if defined(CONFIG_SOC_EN7528)
static inline void tpl_uart_wr(u32 reg, u32 val)
{
	__raw_writel(val, (void __iomem *)(EN75XX_UART0_BASE + (reg << 2)));
}

static inline u32 tpl_uart_rd(u32 reg)
{
	return __raw_readl((void __iomem *)(EN75XX_UART0_BASE + (reg << 2)));
}
#else
static inline void tpl_uart_wr(u32 reg, u32 val)
{
	__raw_writeb(val, (void __iomem *)(EN75XX_UART0_BASE + (reg << 2) + 3));
}

static inline u32 tpl_uart_rd(u32 reg)
{
	return __raw_readb((void __iomem *)(EN75XX_UART0_BASE + (reg << 2) + 3));
}
#endif

#define UART_RBR_THR	0	/* +0x00: receive/transmit, DLL when DLAB=1 */
#define UART_IER_DLM	1	/* +0x04: interrupt enable, DLM when DLAB=1 */
#define UART_IIR_FCR	2	/* +0x08: FIFO control */
#define UART_LCR	3	/* +0x0c: line control (DLAB) */
#define UART_MCR	4	/* +0x10: modem control */
#define UART_LSR	5	/* +0x14: line status (bit5 = THR empty) */

static void tpl_putc(u8 ch)
{
	while (!(tpl_uart_rd(UART_LSR) & 0x20))
		;
	tpl_uart_wr(UART_RBR_THR, ch);
}

static void tpl_hang(u8 code)
{
	tpl_putc(code);
	tpl_putc('\r');
	tpl_putc('\n');

	for (;;)
		;
}

static void tpl_uart_init(void)
{
#if defined(CONFIG_SOC_EN7528)
	/*
	 * EN7528 UART fractional divider: 115200 8N1 = XYD 0xEA00FDE8
	 * (X=round(baud*13/25)=0xEA00, Y=65000), DLL=1, DLM=0. XYD is reg 11 @
	 * +0x2c.
	 */
	tpl_uart_wr(UART_LCR, 0x80);		/* DLAB = 1 */
	tpl_uart_wr(11, 0xea00fde8);		/* XYD fractional divider */
	tpl_uart_wr(UART_RBR_THR, 0x01);	/* DLL = 1 */
	tpl_uart_wr(UART_IER_DLM, 0x00);	/* DLM = 0 */
	tpl_uart_wr(UART_LCR, 0x03);		/* 8N1, DLAB = 0 */
	tpl_uart_wr(UART_IIR_FCR, 0x07);	/* enable + clear FIFOs */
	tpl_uart_wr(UART_MCR, 0x00);
	tpl_uart_wr(UART_IER_DLM, 0x00);
#else
	void __iomem *base = (void __iomem *)EN75XX_UART0_BASE;

	__raw_writeb(0x80, base + 0x0f);
	__raw_writel(0xea00fde8, base + 0x2c);
	__raw_writeb(0x01, base + 0x03);
	__raw_writeb(0x00, base + 0x07);
	__raw_writeb(0x03, base + 0x0f);
	__raw_writeb(0x0f, base + 0x0b);
	__raw_writeb(0x00, base + 0x13);
	__raw_writeb(0x00, base + 0x27);
	__raw_writeb(0x00, base + 0x07);
#endif
}

void __noreturn tpl_main(void)
{
	struct en75xx_legacy_header *hdr =
		(struct en75xx_legacy_header *)EN75XX_SPL_HEADER_ADDR;
	void (*entry)(void);
	u32 load, size, ep;
	int ret;

	tpl_uart_init();
	if (en75xx_sfc_init())
		tpl_hang('I');

	ret = en75xx_sfc_read(EN75XX_DDR_BLOB_OFFSET,
			      (void *)EN75XX_DDR_BLOB_ADDR,
			      EN75XX_DDR_BLOB_SIZE);
	if (ret)
		tpl_hang('F');

	en75xx_run_ddr_blob();

	ret = en75xx_sfc_read(EN75XX_SPL_IMAGE_OFFSET, hdr, IH_HDR_SIZE);
	if (ret || get_be32(hdr->magic) != IH_MAGIC)
		tpl_hang('H');

	size = get_be32(hdr->size);
	load = get_be32(hdr->load);
	ep = get_be32(hdr->ep);

	if (!size || (load & 0xe0000000) != 0x80000000 ||
	    (ep & 0xe0000000) != 0x80000000 || load + size < load)
		tpl_hang('L');

	/* Write through KSEG1 so no dirty cache lines hide the SPL image. */
	ret = en75xx_sfc_read(EN75XX_SPL_IMAGE_OFFSET + IH_HDR_SIZE,
			      (void *)(load | 0x20000000), size);
	if (ret)
		tpl_hang('S');

	__asm__ volatile("sync" : : : "memory");
	entry = (void (*)(void))ep;
	entry();
	__builtin_unreachable();
}
