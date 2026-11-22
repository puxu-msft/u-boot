// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 BayLibre, SAS
 */

#include <asm/io.h>
#include <dm.h>
#include <phy.h>
#include "designware.h"
#include <dm/device_compat.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/ethtool.h>

#define ETH_REG_0		0x0
#define ETH_REG_1		0x4
#define ETH_REG_2		0x18
#define ETH_REG_3		0x1c

#define GX_ETH_REG_0_PHY_INTF		BIT(0)
#define GX_ETH_REG_0_TX_PHASE(x)	(((x) & 3) << 5)
#define GX_ETH_REG_0_TX_RATIO(x)	(((x) & 7) << 7)
#define GX_ETH_REG_0_PHY_CLK_EN	BIT(10)
#define GX_ETH_REG_0_INVERT_RMII_CLK	BIT(11)
#define GX_ETH_REG_0_CLK_EN		BIT(12)

#define AXG_ETH_REG_0_PHY_INTF_RGMII	BIT(0)
#define AXG_ETH_REG_0_PHY_INTF_RMII	BIT(2)
/* 厂商 4.9 驱动（meson6_dwmac_fix_mac_speed）在 DT 的 mc_val 带这一位时千兆置位、10/100 清零；主线 Linux 不用它，含义未公开 */
#define AXG_ETH_REG_0_GBIT_BIT3		BIT(3)
#define AXG_ETH_REG_0_TX_PHASE(x)	(((x) & 3) << 5)
#define AXG_ETH_REG_0_TX_RATIO(x)	(((x) & 7) << 7)
#define AXG_ETH_REG_0_PHY_CLK_EN	BIT(10)
#define AXG_ETH_REG_0_INVERT_RMII_CLK	BIT(11)
#define AXG_ETH_REG_0_CLK_EN		BIT(12)
/* RGMII 路径写满 [24:0]（PHY 接口选择 [2:0]、bit 3、M250 选择、TX 延时、分频、时钟使能、RMII 时钟反相、RX 时序调整 [24:13]）：前一级引导程序可能按内置 PHY 的 RMII 留下这些位，只 setbits 会叠出非法的接口选择并让 RX 时序调整开着 */
#define AXG_ETH_REG_0_RGMII_MASK	GENMASK(24, 0)

/* G12A 起 PRG_ETH1 [19:16] 给输入的 RX_CLK 加延时，每级 200 ps（同 Linux dwmac-meson8b 的 PRG_ETH1_CFG_RXCLK_DLY） */
#define G12A_ETH_REG_1_RXCLK_DLY	GENMASK(19, 16)
#define G12A_ETH_RXCLK_DLY_STEP_PS	200

struct dwmac_meson8b_plat {
	struct dw_eth_pdata dw_eth_pdata;
	int (*dwmac_setup)(struct udevice *dev, struct eth_pdata *edata);
	void *regs;
	bool gbit_bit3;
};

static int dwmac_meson8b_of_to_plat(struct udevice *dev)
{
	struct dwmac_meson8b_plat *pdata = dev_get_plat(dev);

	pdata->regs = dev_read_addr_index_ptr(dev, 1);
	if (!pdata->regs)
		return -EINVAL;

	pdata->dwmac_setup = (void *)dev_get_driver_data(dev);
	if (!pdata->dwmac_setup)
		return -EINVAL;

	pdata->gbit_bit3 = dev_read_bool(dev, "amlogic,gbit-prg-eth0-bit3");

	return designware_eth_of_to_plat(dev);
}

/* PRG_ETH1 的 RX 时钟延时：rx-internal-delay-ps 按 Linux 的语义总要合法，只在 MAC 负责 RX 延时的 rgmii、rgmii-txid 下写进去，其余模式（含 rmii）写 0 */
static int dwmac_setup_g12a_rx_delay(struct udevice *dev, struct eth_pdata *edata)
{
	struct dwmac_meson8b_plat *plat = dev_get_plat(dev);
	u32 ps = dev_read_u32_default(dev, "rx-internal-delay-ps", 0);

	if (ps % G12A_ETH_RXCLK_DLY_STEP_PS ||
	    ps / G12A_ETH_RXCLK_DLY_STEP_PS > FIELD_MAX(G12A_ETH_REG_1_RXCLK_DLY)) {
		dev_err(dev, "rx-internal-delay-ps %u: must be a multiple of %d up to %lu\n",
			ps, G12A_ETH_RXCLK_DLY_STEP_PS,
			FIELD_MAX(G12A_ETH_REG_1_RXCLK_DLY) * G12A_ETH_RXCLK_DLY_STEP_PS);
		return -EINVAL;
	}
	if (edata->phy_interface != PHY_INTERFACE_MODE_RGMII &&
	    edata->phy_interface != PHY_INTERFACE_MODE_RGMII_TXID)
		ps = 0;

	clrsetbits_le32(plat->regs + ETH_REG_1, G12A_ETH_REG_1_RXCLK_DLY,
			FIELD_PREP(G12A_ETH_REG_1_RXCLK_DLY, ps / G12A_ETH_RXCLK_DLY_STEP_PS));

	return 0;
}

static int dwmac_setup_axg(struct udevice *dev, struct eth_pdata *edata)
{
	struct dwmac_meson8b_plat *plat = dev_get_plat(dev);

	switch (edata->phy_interface) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
		/* Set RGMII mode */
		clrsetbits_le32(plat->regs + ETH_REG_0, AXG_ETH_REG_0_RGMII_MASK,
				AXG_ETH_REG_0_PHY_INTF_RGMII |
				AXG_ETH_REG_0_TX_PHASE(1) |
				AXG_ETH_REG_0_TX_RATIO(4) |
				AXG_ETH_REG_0_PHY_CLK_EN |
				AXG_ETH_REG_0_CLK_EN);
		break;

	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		/* TOFIX：不读 amlogic,tx-delay-ns（各 RGMII 分支固定 1 档即 2 ns 或 0）；rgmii-id 与 rgmii-rxid 的 MAC TX 延时与 Linux 相反（Linux 是 rxid 加、id 不加）；axg 不读 rx-internal-delay-ps（Linux 在 axg 上支持 0 与 2000） */
		clrsetbits_le32(plat->regs + ETH_REG_0, AXG_ETH_REG_0_RGMII_MASK,
				AXG_ETH_REG_0_PHY_INTF_RGMII |
				AXG_ETH_REG_0_TX_RATIO(4) |
				AXG_ETH_REG_0_PHY_CLK_EN |
				AXG_ETH_REG_0_CLK_EN);
		break;

	case PHY_INTERFACE_MODE_RMII:
		/* Set RMII mode */
		out_le32(plat->regs + ETH_REG_0, AXG_ETH_REG_0_PHY_INTF_RMII |
						 AXG_ETH_REG_0_INVERT_RMII_CLK |
						 AXG_ETH_REG_0_CLK_EN);
		break;
	default:
		dev_err(dev, "Unsupported PHY mode\n");
		return -EINVAL;
	}

	return 0;
}

static int dwmac_setup_g12a(struct udevice *dev, struct eth_pdata *edata)
{
	int ret;

	ret = dwmac_setup_axg(dev, edata);
	if (ret)
		return ret;

	return dwmac_setup_g12a_rx_delay(dev, edata);
}

static int dwmac_setup_gx(struct udevice *dev, struct eth_pdata *edata)
{
	struct dwmac_meson8b_plat *plat = dev_get_plat(dev);

	switch (edata->phy_interface) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
		/* Set RGMII mode */
		setbits_le32(plat->regs + ETH_REG_0, GX_ETH_REG_0_PHY_INTF |
						     GX_ETH_REG_0_TX_PHASE(1) |
						     GX_ETH_REG_0_TX_RATIO(4) |
						     GX_ETH_REG_0_PHY_CLK_EN |
						     GX_ETH_REG_0_CLK_EN);

		break;

	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		/* TOFIX: handle amlogic,tx-delay-ns & rx-internal-delay-ps from DT */
		setbits_le32(plat->regs + ETH_REG_0, GX_ETH_REG_0_PHY_INTF |
						     GX_ETH_REG_0_TX_RATIO(4) |
						     GX_ETH_REG_0_PHY_CLK_EN |
						     GX_ETH_REG_0_CLK_EN);

		break;

	case PHY_INTERFACE_MODE_RMII:
		/* Set RMII mode */
		out_le32(plat->regs + ETH_REG_0, GX_ETH_REG_0_INVERT_RMII_CLK |
						 GX_ETH_REG_0_CLK_EN);

		if (!IS_ENABLED(CONFIG_MESON_GXBB))
			writel(0x10110181, plat->regs + ETH_REG_2);

		break;
	default:
		dev_err(dev, "Unsupported PHY mode\n");
		return -EINVAL;
	}

	return 0;
}

static int dwmac_meson8b_probe(struct udevice *dev)
{
	struct dwmac_meson8b_plat *pdata = dev_get_plat(dev);
	struct eth_pdata *edata = &pdata->dw_eth_pdata.eth_pdata;
	int ret;

	ret = pdata->dwmac_setup(dev, edata);
	if (ret)
		return ret;

	return designware_eth_probe(dev);
}

/* 与 designware_eth_start 相同，只在协商出速率之后、打开收发之前按速率改写 bit 3（DT 带 amlogic,gbit-prg-eth0-bit3 时千兆置位、10/100 清零，同厂商 4.9 驱动） */
static int dwmac_meson8b_start(struct udevice *dev)
{
	struct dwmac_meson8b_plat *pdata = dev_get_plat(dev);
	struct dw_eth_dev *priv = dev_get_priv(dev);
	int ret;

	ret = designware_eth_init(priv, pdata->dw_eth_pdata.eth_pdata.enetaddr);
	if (ret)
		return ret;

	if (pdata->gbit_bit3)
		clrsetbits_le32(pdata->regs + ETH_REG_0, AXG_ETH_REG_0_GBIT_BIT3,
				priv->phydev->speed == SPEED_1000 ? AXG_ETH_REG_0_GBIT_BIT3 : 0);

	return designware_eth_enable(priv);
}

static const struct eth_ops dwmac_meson8b_eth_ops = {
	.start			= dwmac_meson8b_start,
	.send			= designware_eth_send,
	.recv			= designware_eth_recv,
	.free_pkt		= designware_eth_free_pkt,
	.stop			= designware_eth_stop,
	.write_hwaddr		= designware_eth_write_hwaddr,
};

static const struct udevice_id dwmac_meson8b_ids[] = {
	{ .compatible = "amlogic,meson-gxbb-dwmac", .data = (ulong)dwmac_setup_gx },
	{ .compatible = "amlogic,meson-g12a-dwmac", .data = (ulong)dwmac_setup_g12a },
	{ .compatible = "amlogic,meson-axg-dwmac", .data = (ulong)dwmac_setup_axg },
	{ }
};

U_BOOT_DRIVER(dwmac_meson8b) = {
	.name		= "dwmac_meson8b",
	.id		= UCLASS_ETH,
	.of_match	= dwmac_meson8b_ids,
	.of_to_plat = dwmac_meson8b_of_to_plat,
	.probe		= dwmac_meson8b_probe,
	.ops		= &dwmac_meson8b_eth_ops,
	.priv_auto	= sizeof(struct dw_eth_dev),
	.plat_auto	= sizeof(struct dwmac_meson8b_plat),
	.flags		= DM_FLAG_ALLOC_PRIV_DMA,
};
