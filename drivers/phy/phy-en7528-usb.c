// SPDX-License-Identifier: GPL-2.0
/*
 * EcoNet EN7528 USB PHY
 *
 * 移植自 OpenWrt target/linux/econet 的 Linux 驱动（patches-6.18/914，Ahmed Naseef，依据 keenetic/kernel-49 的厂商 GPL 代码）。寄存器序列与原厂 3.18 内核一致：u3phy_config_en7528 写两个 U2 口的 ACR3 和 U3 PHYA 的 RX_IMPSEL，u2_slew_rate_cal_en7528 用频率计逐口校准 HS 发送斜率。
 *
 * PHY 说明符的参数是 PHY 类型（PHY_TYPE_USB2 / PHY_TYPE_USB3），两类各管一组寄存器：USB2 一个实例覆盖两个 U2 口。
 */

#include <dm.h>
#include <generic-phy.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <dt-bindings/phy/phy.h>

/* 频率计，两个 U2 口共用 */
#define EN7528_USB_PHY_FMCR0			0x100
#define   EN7528_USB_PHY_MONCLK_SEL		GENMASK(27, 26)
#define   EN7528_USB_PHY_FREQDET_EN		BIT(24)
#define   EN7528_USB_PHY_CYCLECNT		GENMASK(23, 0)
#define EN7528_USB_PHY_FMMONR0			0x10c
#define EN7528_USB_PHY_FMMONR1			0x110
#define   EN7528_USB_PHY_FRCK_EN		BIT(8)

/* U2 口寄存器组的基址 */
#define EN7528_USB_PHY_U2_P0_BASE		0x300
#define EN7528_USB_PHY_U2_P1_BASE		0x1300
#define EN7528_USB_PHY_NUM_U2_PORTS		2

/* U2 口内的偏移 */
#define EN7528_USB_PHY_ACR0			0x10
#define   EN7528_USB_PHY_HSTX_SRCAL_EN		BIT(23)
#define   EN7528_USB_PHY_HSTX_SRCTRL		GENMASK(18, 16)
#define EN7528_USB_PHY_ACR3			0x1c
#define EN7528_USB_PHY_ACR3_ENABLE		0xc0240000

/* U3 PHYA（基址 +0xb00） */
#define EN7528_USB_PHY_U3_PHYA_REG11		0xb2c
#define   EN7528_USB_PHY_RX_IMPSEL		GENMASK(13, 12)

/* 斜率值 = (FM_DET_CYCLE_CNT / FM_OUT) * REF_CK * SR_COEF / SR_COEF_DIVISOR，四舍五入；测不到频率时用 DEFAULT_SR */
#define EN7528_USB_PHY_FM_DET_CYCLE_CNT		1024
#define EN7528_USB_PHY_REF_CK			20	/* MHz */
#define EN7528_USB_PHY_SR_COEF			28
#define EN7528_USB_PHY_SR_COEF_DIVISOR		1000
#define EN7528_USB_PHY_DEFAULT_SR		4

#define EN7528_USB_PHY_FREQDET_SLEEP_US		1000
#define EN7528_USB_PHY_FREQDET_TIMEOUT_US	10000

struct en7528_usb_phy_priv {
	void __iomem *base;
};

static const unsigned int en7528_u2_port_bases[EN7528_USB_PHY_NUM_U2_PORTS] = {
	EN7528_USB_PHY_U2_P0_BASE,
	EN7528_USB_PHY_U2_P1_BASE,
};

static void en7528_usb_phy_slew_rate_calibration(struct udevice *dev,
						 void __iomem *base,
						 unsigned int port_id,
						 unsigned int port_base)
{
	void __iomem *acr0 = base + port_base + EN7528_USB_PHY_ACR0;
	u32 fm_out;
	u32 srctrl;

	setbits_le32(acr0, EN7528_USB_PHY_HSTX_SRCAL_EN);
	udelay(1000);

	setbits_le32(base + EN7528_USB_PHY_FMMONR1, EN7528_USB_PHY_FRCK_EN);

	/* 频率计测的口由 MONCLK_SEL 选：口 0 取 0，口 1 取 1 */
	clrsetbits_le32(base + EN7528_USB_PHY_FMCR0, EN7528_USB_PHY_MONCLK_SEL,
			FIELD_PREP(EN7528_USB_PHY_MONCLK_SEL, port_id));
	clrsetbits_le32(base + EN7528_USB_PHY_FMCR0, EN7528_USB_PHY_CYCLECNT,
			FIELD_PREP(EN7528_USB_PHY_CYCLECNT,
				   EN7528_USB_PHY_FM_DET_CYCLE_CNT));
	setbits_le32(base + EN7528_USB_PHY_FMCR0, EN7528_USB_PHY_FREQDET_EN);

	/* 超时不算错：fm_out 仍为 0，下面改用缺省斜率 */
	(void)readl_poll_sleep_timeout(base + EN7528_USB_PHY_FMMONR0, fm_out,
				       fm_out,
				       EN7528_USB_PHY_FREQDET_SLEEP_US,
				       EN7528_USB_PHY_FREQDET_TIMEOUT_US);

	clrbits_le32(base + EN7528_USB_PHY_FMCR0, EN7528_USB_PHY_FREQDET_EN);
	clrbits_le32(base + EN7528_USB_PHY_FMMONR1, EN7528_USB_PHY_FRCK_EN);
	clrbits_le32(acr0, EN7528_USB_PHY_HSTX_SRCAL_EN);
	udelay(1000);

	if (!fm_out) {
		srctrl = EN7528_USB_PHY_DEFAULT_SR;
		dev_err(dev, "port%u: frequency not detected, using default SR calibration\n",
			port_id);
	} else {
		srctrl = EN7528_USB_PHY_REF_CK * EN7528_USB_PHY_SR_COEF;
		srctrl = srctrl * EN7528_USB_PHY_FM_DET_CYCLE_CNT / fm_out;
		srctrl = DIV_ROUND_CLOSEST(srctrl,
					   EN7528_USB_PHY_SR_COEF_DIVISOR);
		dev_dbg(dev, "port%u: SR calibration applied: %x\n", port_id,
			srctrl);
	}

	clrsetbits_le32(acr0, EN7528_USB_PHY_HSTX_SRCTRL,
			FIELD_PREP(EN7528_USB_PHY_HSTX_SRCTRL, srctrl));
}

static int en7528_usb_phy_init(struct phy *phy)
{
	struct en7528_usb_phy_priv *priv = dev_get_priv(phy->dev);
	unsigned int i;

	switch (phy->id) {
	case PHY_TYPE_USB2:
		/* 校准前先打开两个 U2 口 */
		for (i = 0; i < EN7528_USB_PHY_NUM_U2_PORTS; i++)
			writel(EN7528_USB_PHY_ACR3_ENABLE,
			       priv->base + en7528_u2_port_bases[i] +
			       EN7528_USB_PHY_ACR3);
		return 0;
	case PHY_TYPE_USB3:
		/* 组合 PHY 的接收端阻抗均值偏高，调低 5 Ω */
		clrsetbits_le32(priv->base + EN7528_USB_PHY_U3_PHYA_REG11,
				EN7528_USB_PHY_RX_IMPSEL,
				FIELD_PREP(EN7528_USB_PHY_RX_IMPSEL, 0x1));
		return 0;
	default:
		return -EINVAL;
	}
}

static int en7528_usb_phy_power_on(struct phy *phy)
{
	struct en7528_usb_phy_priv *priv = dev_get_priv(phy->dev);
	unsigned int i;

	if (phy->id != PHY_TYPE_USB2)
		return 0;

	for (i = 0; i < EN7528_USB_PHY_NUM_U2_PORTS; i++)
		en7528_usb_phy_slew_rate_calibration(phy->dev, priv->base, i,
						     en7528_u2_port_bases[i]);

	return 0;
}

static int en7528_usb_phy_of_xlate(struct phy *phy,
				   struct ofnode_phandle_args *args)
{
	if (args->args_count != 1) {
		dev_err(phy->dev, "invalid number of cells in 'phy' property\n");
		return -EINVAL;
	}

	if (args->args[0] != PHY_TYPE_USB2 && args->args[0] != PHY_TYPE_USB3) {
		dev_err(phy->dev, "unsupported device type: %u\n",
			args->args[0]);
		return -EINVAL;
	}

	phy->id = args->args[0];

	return 0;
}

static int en7528_usb_phy_probe(struct udevice *dev)
{
	struct en7528_usb_phy_priv *priv = dev_get_priv(dev);

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;

	return 0;
}

static const struct phy_ops en7528_usb_phy_ops = {
	.of_xlate	= en7528_usb_phy_of_xlate,
	.init		= en7528_usb_phy_init,
	.power_on	= en7528_usb_phy_power_on,
};

static const struct udevice_id en7528_usb_phy_ids[] = {
	{ .compatible = "econet,en7528-usb-phy" },
	{ }
};

U_BOOT_DRIVER(en7528_usb_phy) = {
	.name		= "en7528-usb-phy",
	.id		= UCLASS_PHY,
	.of_match	= en7528_usb_phy_ids,
	.ops		= &en7528_usb_phy_ops,
	.probe		= en7528_usb_phy_probe,
	.priv_auto	= sizeof(struct en7528_usb_phy_priv),
};
