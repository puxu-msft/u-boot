// SPDX-License-Identifier: GPL-2.0+
/*
 * DAPU Telecom DAP8211R(I) 千兆以太网 PHY（PHY ID 0x0008011b）。睿普康 RPC8211FP(I) 的 PHY ID 相同，Linux 驱动用到的扩展寄存器字段（0xa001 bit 15 软复位、0xa003 的 RX [13:10] 与 TX [3:0] 延时）与它的数据手册 V1.0 表 14、15 一致。
 *
 * RGMII 延时（扩展寄存器 0xa003）照 Linux drivers/net/phy/dap8211r.c 与绑定 dptel,dap8211r.yaml 的语义：按 phy-mode 与 PHY 节点的 rx-internal-delay-ps、tx-internal-delay-ps（每档 150 ps）设，设之前按 0xa001 bit 15 软复位。不是 150 ps 整数倍或超过 2250 ps 的值直接报错（Linux 取最近一档或当作没给，绑定本身不允许这样的值）。
 * RGMII 接收方向的焊盘驱动强度（扩展寄存器 0xa010，RPC8211FP(I) 数据手册 V1.0 表 28）由 PHY 节点的 dptel,rx-clk-drive-strength、dptel,rx-data-drive-strength 给出（0–7，Linux 绑定里没有）；不给就不写。
 * 属性不合法时不写任何寄存器就报错返回；软复位等不到自清时报错返回，延时与驱动强度都不写；两种情况都不再调 genphy_config（designware 不看 phy_config 的返回值，打印是唯一的线索）。
 */

#include <phy.h>
#include <dm/ofnode.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#define DAP8211R_PHY_ID			0x0008011b
#define DAP8211R_PHY_ID_MASK		0xffffffff

#define DAP8211R_EXT_ADDR		0x1e
#define DAP8211R_EXT_DATA		0x1f

#define DAP8211R_CHIP_CFG		0xa001
#define DAP8211R_CHIP_CFG_SW_RST_N	BIT(15)

#define DAP8211R_RGMII_CFG1		0xa003
#define DAP8211R_RGMII_RX_DELAY		GENMASK(13, 10)
#define DAP8211R_RGMII_TX_DELAY		GENMASK(3, 0)
#define DAP8211R_DELAY_STEP_PS		150
/* 复位值：RX 0 档、TX 1 档（rgmii 不给属性时用）；rgmii-id 一类由 PHY 负责延时又不给属性时取 13 档（1.95 ns），同 Linux */
#define DAP8211R_RX_DELAY_RESET		0
#define DAP8211R_TX_DELAY_RESET		1
#define DAP8211R_DELAY_DEFAULT		13

#define DAP8211R_PAD_DRIVE		0xa010
#define DAP8211R_PAD_DRIVE_RXC		GENMASK(15, 13)
#define DAP8211R_PAD_DRIVE_RXD_HI	BIT(12)
#define DAP8211R_PAD_DRIVE_RXD_LO	GENMASK(5, 4)
#define DAP8211R_DRIVE_MAX		7

static int dap8211r_read_ext(struct phy_device *phydev, u16 reg)
{
	int ret;

	ret = phy_write(phydev, MDIO_DEVAD_NONE, DAP8211R_EXT_ADDR, reg);
	if (ret < 0)
		return ret;

	return phy_read(phydev, MDIO_DEVAD_NONE, DAP8211R_EXT_DATA);
}

static int dap8211r_modify_ext(struct phy_device *phydev, u16 reg, u16 mask, u16 set)
{
	int ret;

	ret = phy_write(phydev, MDIO_DEVAD_NONE, DAP8211R_EXT_ADDR, reg);
	if (ret < 0)
		return ret;

	return phy_modify(phydev, MDIO_DEVAD_NONE, DAP8211R_EXT_DATA, mask, set);
}

/* 返回档位；属性不在时返回 -ENOENT，值不是 150 ps 的整数倍或超出 15 档时返回 -EINVAL */
static int dap8211r_read_delay(struct phy_device *phydev, const char *prop)
{
	u32 ps;

	if (ofnode_read_u32(phydev->node, prop, &ps))
		return -ENOENT;
	if (ps % DAP8211R_DELAY_STEP_PS ||
	    ps / DAP8211R_DELAY_STEP_PS > FIELD_MAX(DAP8211R_RGMII_RX_DELAY)) {
		pr_err("%s: %s %u: must be a multiple of %d up to %lu\n", phydev->drv->name, prop, ps,
		       DAP8211R_DELAY_STEP_PS, FIELD_MAX(DAP8211R_RGMII_RX_DELAY) * DAP8211R_DELAY_STEP_PS);
		return -EINVAL;
	}

	return ps / DAP8211R_DELAY_STEP_PS;
}

static int dap8211r_config_delay(struct phy_device *phydev)
{
	int rx = DAP8211R_RX_DELAY_RESET, tx = DAP8211R_TX_DELAY_RESET;
	int ret, val;

	if (phydev->interface != PHY_INTERFACE_MODE_RGMII_TXID)
		rx = dap8211r_read_delay(phydev, "rx-internal-delay-ps");
	if (phydev->interface != PHY_INTERFACE_MODE_RGMII_RXID)
		tx = dap8211r_read_delay(phydev, "tx-internal-delay-ps");
	if (rx == -EINVAL || tx == -EINVAL)
		return -EINVAL;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII:
		if (rx < 0)
			rx = DAP8211R_RX_DELAY_RESET;
		if (tx < 0)
			tx = DAP8211R_TX_DELAY_RESET;
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		if (rx < 0)
			rx = DAP8211R_DELAY_DEFAULT;
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		if (rx < 0)
			rx = DAP8211R_DELAY_DEFAULT;
		fallthrough;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		if (tx < 0)
			tx = DAP8211R_DELAY_DEFAULT;
		break;
	default:
		return -EINVAL;
	}

	/* 软复位（低有效、自清），等它回到 1 */
	ret = dap8211r_modify_ext(phydev, DAP8211R_CHIP_CFG, DAP8211R_CHIP_CFG_SW_RST_N, 0);
	if (ret < 0)
		return ret;
	ret = read_poll_timeout(dap8211r_read_ext, val, val < 0 || (val & DAP8211R_CHIP_CFG_SW_RST_N),
				20, 200, phydev, DAP8211R_CHIP_CFG);
	if (!ret && val < 0)
		ret = val;
	if (ret) {
		pr_err("%s: soft reset did not complete (%d)\n", phydev->drv->name, ret);
		return ret;
	}

	return dap8211r_modify_ext(phydev, DAP8211R_RGMII_CFG1,
				   DAP8211R_RGMII_RX_DELAY | DAP8211R_RGMII_TX_DELAY,
				   FIELD_PREP(DAP8211R_RGMII_RX_DELAY, rx) |
				   FIELD_PREP(DAP8211R_RGMII_TX_DELAY, tx));
}

/* 解析 0xa010 要改的位：mask 为 0 表示两个属性都没给 */
static int dap8211r_parse_drive(struct phy_device *phydev, u16 *mask, u16 *set)
{
	u32 clk, data;

	*mask = 0;
	*set = 0;
	if (!ofnode_read_u32(phydev->node, "dptel,rx-clk-drive-strength", &clk)) {
		if (clk > DAP8211R_DRIVE_MAX)
			goto bad;
		*mask |= DAP8211R_PAD_DRIVE_RXC;
		*set |= FIELD_PREP(DAP8211R_PAD_DRIVE_RXC, clk);
	}
	if (!ofnode_read_u32(phydev->node, "dptel,rx-data-drive-strength", &data)) {
		if (data > DAP8211R_DRIVE_MAX)
			goto bad;
		*mask |= DAP8211R_PAD_DRIVE_RXD_HI | DAP8211R_PAD_DRIVE_RXD_LO;
		*set |= FIELD_PREP(DAP8211R_PAD_DRIVE_RXD_HI, data >> 2) |
			FIELD_PREP(DAP8211R_PAD_DRIVE_RXD_LO, data & 3);
	}

	return 0;

bad:
	pr_err("%s: dptel,rx-*-drive-strength must be 0..%d\n", phydev->drv->name, DAP8211R_DRIVE_MAX);
	return -EINVAL;
}

static int dap8211r_config(struct phy_device *phydev)
{
	u16 mask, set;
	int ret;

	if (phy_interface_is_rgmii(phydev)) {
		ret = dap8211r_parse_drive(phydev, &mask, &set);
		if (ret < 0)
			return ret;
		ret = dap8211r_config_delay(phydev);
		if (ret < 0)
			return ret;
		if (mask) {
			ret = dap8211r_modify_ext(phydev, DAP8211R_PAD_DRIVE, mask, set);
			if (ret < 0)
				return ret;
		}
	}

	return genphy_config(phydev);
}

U_BOOT_PHY_DRIVER(dap8211r) = {
	.name		= "DAP8211R Gigabit Ethernet",
	.uid		= DAP8211R_PHY_ID,
	.mask		= DAP8211R_PHY_ID_MASK,
	.features	= PHY_GBIT_FEATURES,
	.config		= dap8211r_config,
	.startup	= genphy_startup,
	.shutdown	= genphy_shutdown,
};
