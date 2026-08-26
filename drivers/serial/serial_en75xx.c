// SPDX-License-Identifier: GPL-2.0+
/* EcoNet/Airoha EN75xx UART */

#include <dm.h>
#include <errno.h>
#include <serial.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/types.h>

/* 16550 register indices (the byte/word offset is derived per SoC below) */
#define R_RBR		0	/* THR / DLL when DLAB=1 */
#define R_IER		1	/* DLM when DLAB=1 */
#define R_FCR		2
#define R_LCR		3
#define R_MCR		4
#define R_LSR		5
#define R_MISCC		9
#define R_XYD		11	/* EN7512 fractional divider (32-bit) */

#define UART_LCR_DLAB	0x80
#define UART_LCR_8N1	0x03
#define UART_LSR_DR	0x01
#define UART_LSR_THRE	0x20

struct en75xx_serial_plat {
	void __iomem *base;
	bool le;		/* EN7528: little-endian, 32-bit word access */
	u32 uartclk;		/* EN7528 UART source clock (0 => EN7512) */
};

/*
 * EN7512 (big-endian): the 8-bit registers live in the top byte of each
 * 32-bit word, i.e. byte offset (reg << 2) + 3, byte access.
 * EN7528 (little-endian, reg-io-width=4, reg-shift=2): 32-bit word access at
 * (reg << 2), value in the low byte.
 */
static u8 uart_rd(struct en75xx_serial_plat *plat, u32 reg)
{
	if (plat->le)
		return readl(plat->base + (reg << 2)) & 0xff;

	return readb(plat->base + (reg << 2) + 3);
}

static void uart_wr(struct en75xx_serial_plat *plat, u32 reg, u32 val)
{
	if (plat->le)
		writel(val, plat->base + (reg << 2));
	else
		writeb(val, plat->base + (reg << 2) + 3);
}

static void en75xx_serial_hw_init(struct en75xx_serial_plat *plat,
				  int baudrate)
{
	if (baudrate <= 0)
		baudrate = CONFIG_BAUDRATE;

	/*
	 * EN7512 (BE) and EN7528 (LE) share the divider: Y=65000,
	 * X=round(baud*13/25), packed into the 32-bit XYD reg (11 @ +0x2c)
	 * as {X[31:16],Y[15:0]}. 16-bit regs go through uart_wr()
	 * (endian-correct); the XYD store is a native 32-bit write, valid
	 * on both. For example, XYD == 0xEA00FDE8 at 115200.
	 */
	u32 div_x = DIV_ROUND_CLOSEST((u32)baudrate * 13, 25);

	if (div_x > 0xffff)
		div_x = 0xffff;

	uart_wr(plat, R_LCR, UART_LCR_DLAB);
	__raw_writel((div_x << 16) | 65000, plat->base + (R_XYD << 2));
	uart_wr(plat, R_RBR, 1);
	uart_wr(plat, R_IER, 0);
	uart_wr(plat, R_LCR, UART_LCR_8N1);
	uart_wr(plat, R_FCR, 0x0f);
	uart_wr(plat, R_MCR, 0);
	uart_wr(plat, R_MISCC, 0);
	uart_wr(plat, R_IER, 0);
}

static int en75xx_serial_putc(struct udevice *dev, const char ch)
{
	struct en75xx_serial_plat *plat = dev_get_plat(dev);

	if (!(uart_rd(plat, R_LSR) & UART_LSR_THRE))
		return -EAGAIN;

	uart_wr(plat, R_RBR, ch);
	return 0;
}

static int en75xx_serial_getc(struct udevice *dev)
{
	struct en75xx_serial_plat *plat = dev_get_plat(dev);

	if (!(uart_rd(plat, R_LSR) & UART_LSR_DR))
		return -EAGAIN;

	return uart_rd(plat, R_RBR);
}

static int en75xx_serial_pending(struct udevice *dev, bool input)
{
	struct en75xx_serial_plat *plat = dev_get_plat(dev);
	u8 lsr = uart_rd(plat, R_LSR);

	if (input)
		return !!(lsr & UART_LSR_DR);

	return !(lsr & UART_LSR_THRE);
}

static int en75xx_serial_setbrg(struct udevice *dev, int baudrate)
{
	struct en75xx_serial_plat *plat = dev_get_plat(dev);

	en75xx_serial_hw_init(plat, baudrate);
	return 0;
}

static int en75xx_serial_of_to_plat(struct udevice *dev)
{
	struct en75xx_serial_plat *plat = dev_get_plat(dev);

	plat->base = dev_remap_addr(dev);
	if (!plat->base)
		return -EINVAL;

	plat->le = (bool)dev_get_driver_data(dev);
	if (plat->le)
		plat->uartclk = dev_read_u32_default(dev, "clock-frequency",
						     2000000);

	return 0;
}

static int en75xx_serial_probe(struct udevice *dev)
{
	struct en75xx_serial_plat *plat = dev_get_plat(dev);

	en75xx_serial_hw_init(plat, CONFIG_BAUDRATE);
	return 0;
}

static const struct dm_serial_ops en75xx_serial_ops = {
	.putc = en75xx_serial_putc,
	.pending = en75xx_serial_pending,
	.getc = en75xx_serial_getc,
	.setbrg = en75xx_serial_setbrg,
};

static const struct udevice_id en75xx_serial_ids[] = {
	{ .compatible = "econet,en7512-uart", .data = 0 },
	{ .compatible = "econet,en7528-uart", .data = 1 },
	{ .compatible = "airoha,en7523-uart", .data = 1 },
	{ }
};

U_BOOT_DRIVER(serial_en75xx) = {
	.name = "serial_en75xx",
	.id = UCLASS_SERIAL,
	.of_match = en75xx_serial_ids,
	.of_to_plat = en75xx_serial_of_to_plat,
	.plat_auto = sizeof(struct en75xx_serial_plat),
	.probe = en75xx_serial_probe,
	.ops = &en75xx_serial_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
