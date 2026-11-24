// SPDX-License-Identifier: GPL-2.0+
/*
 * CM201-2（Hi3798MV300）的板级初始化。
 *
 * U-Boot 没有这个 SoC 的 CRG 时钟驱动，因此设备树给 eMMC 与 FEMAC 配固定时钟，由本文件让 CRG 与之一致。在 TF-A 之前运行的厂商 fastboot 会让 eMMC 控制器停在它最后用过的时序模式（其 CIU mux 会切到 50、100 或 200 MHz，HS400 还会设置控制器侧的 DDR 与 DLL 状态），核心热复位不触及其中任何一项，所以这里把控制器恢复到已知状态：CIU 设为 50 MHz 并用 fastboot 自己在该速率下用的相位，采样 DLL 退出 tuning 模式，再给控制器一个软复位脉冲，把厂商寄存器（DDR_REG、ENABLE_SHIFT、TUNING_CTRL）恢复为默认值。
 *
 * 寄存器布局：PERI_CRG40（0xa0）bit 0 BIU 门控，bit 1 CIU 门控，bit 4 软复位，bits 10:8 CIU mux（0 100M，1 50M，2 25M，3 200M，6 150M），bits 14:12 采样相位，bits 18:16 驱动相位（步进 45 度）；PERI_CRG231（0x39c）bit 16 SAP DLL 模式（1 = 不在 tuning）；PERI_CRG52（0xd0）bit 0 FEMAC 接口门控，bit 1 FEMAC 核心门控；PERI_CRG226（0x388）bit 0 FE PHY 门控。出处：Linux hi3798mv200 CRG 驱动与厂商 fastboot 自己的 eMMC 时钟例程。
 *
 * USB2 host（generic-ehci，0xf9890000）同样没有时钟与 PHY 驱动可用，由本文件在 board_init() 里打开：控制器的时钟与复位在 PERI_CRG46（0xb8），INNO PHY 的参考时钟与复位在 PERI_CRG47（0xbc），PHY 自己的寄存器直接映射在 0xf9865000（两个端口各占 0x400，每个 8 位寄存器占一个 32 位字）。顺序与写入值照搬已在本板验证的 Linux 驱动（ehci-platform 与 phy-hisi-inno-usb2，后者加内核本地补丁 0002 的 MV300 序列，值与延时出自厂商 bootloader 的 USB2 初始化例程）：释放控制器复位 → 开控制器时钟 → 开 PHY 参考时钟 → 释放 PHY 上电复位 → 逐端口写 PHY 寄存器 → 释放 PHY 端口复位。厂商 fastboot（现网 p2 的 bootcmd 不用 USB）不碰这些位，所以 board_init() 先记下两个寄存器的原值，交给内核前（board_quiesce_devices()）停掉 USB 再原样写回，内核接手时 USB 的状态与不经 U-Boot 时相同；同时也保证 EHCI 不会带着运行中的调度进内核（generic-ehci 没有 DM_FLAG_OS_PREPARE，bootm 的 dm_remove_devices_active() 不会移除它）。
 */

#include <bootm.h>
#include <fdt_support.h>
#include <fdtdec.h>
#include <init.h>
#include <lmb.h>
#include <usb.h>
#include <asm/global_data.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

#define CRG_BASE		0xf8a22000UL

#define PERI_CRG40_EMMC		(CRG_BASE + 0xa0)
#define  EMMC_BIU_CKEN		BIT(0)
#define  EMMC_CIU_CKEN		BIT(1)
#define  EMMC_SRST_REQ		BIT(4)
#define  EMMC_CLK_SEL		GENMASK(10, 8)
#define  EMMC_CLK_SEL_50M	1
#define  EMMC_SAMPLE_PHASE	GENMASK(14, 12)
#define  EMMC_DRV_PHASE		GENMASK(18, 16)
#define  EMMC_PHASE_0		0
#define  EMMC_PHASE_180		4

#define PERI_CRG231_SAP_DLL	(CRG_BASE + 0x39c)
#define  SAP_DLL_DLLMODE	BIT(16)

#define PERI_CRG52_FEMAC	(CRG_BASE + 0xd0)
#define  FEMAC_BUS_CKEN		BIT(0)
#define  FEMAC_CKEN		BIT(1)

#define PERI_CRG226_FEPHY	(CRG_BASE + 0x388)
#define  FEPHY_CKEN		BIT(0)

/* 位定义出处：Linux crg-hi3798mv200.c 的 USB2 门控与 hi3798mv200.dtsi、本板内核 DTS 的 resets（复位位置 1 为复位）。 */
#define PERI_CRG46_USB2		(CRG_BASE + 0xb8)
#define  USB2_BUS_CKEN		BIT(0)
#define  USB2_HST_PHY_CKEN	BIT(4)
#define  USB2_UTMI0_CKEN	BIT(5)
#define  USB2_UTMI1_CKEN	BIT(6)
#define  USB2_BUS_SRST		BIT(12)
#define  USB2_UTMI0_SRST	BIT(13)
#define  USB2_UTMI1_SRST	BIT(14)
#define  USB2_HST_PHY_SRST	BIT(16)

#define PERI_CRG47_USB2_PHY	(CRG_BASE + 0xbc)
#define  USB2_PHY1_REF_CKEN	BIT(0)
#define  USB2_PHY1_POR_SRST	BIT(4)
#define  USB2_PHY1_PORT0_SRST	BIT(8)
#define  USB2_PHY1_PORT1_SRST	BIT(9)
#define  USB2_PHY1_TPOR_SRST	BIT(15)

#define USB2_PHY1_BASE		0xf9865000UL
#define USB2_PHY1_PORTS		2
#define USB2_PHY1_PORT_STRIDE	0x400

/* PHY 端口寄存器 0x06 bit 2 打开 PHY 时钟；其后五个写入是 MV300 额外要的（不写时 EHCI 复位端口超时），每组写完后的延时同 Linux 驱动。 */
static const struct {
	u8 reg;
	u8 val;
	u16 delay_us;
} cm201_usb2_phy_setup[] = {
	{ 0x06, 0x04, 2000 },
	{ 0x02, 0x6c, 2000 },
	{ 0x00, 0x18, 2000 },
	{ 0x04, 0xd7, 20 },
	{ 0x1d, 0x1e, 20 },
	{ 0x1f, 0x6e, 20 },
};

static u32 cm201_usb2_saved_crg46, cm201_usb2_saved_crg47;

/*
 * 开机 logo：厂商 fastboot 把 logo 解码进 DDR 顶部附近的一块帧缓冲（实测 0x3f4b9000 起、1920x1080x4 B），VDP 的图形层 G0 一直从那里扫描，直到 Linux 的 histb-vdp 接管、换上自己的缓冲。U-Boot 的重定位区（含 32 MiB 堆）和内核都会把这块当普通内存覆盖，画面就花了（docs/07 #168）。
 *
 * G0 开着时：U-Boot 的可用内存顶压到帧缓冲之下（board_get_usable_ram_top），lmb 保留它（board_late_init，bootm 搬设备树与 ramdisk 时避开），交给内核的设备树里加 no-map 的 reserved-memory 节点（ft_board_setup），logo 原样显示到 histb-vdp 接管为止。
 *
 * fastboot 不是每次都初始化显示：p2 env 设了 logo=0、显示参数无效、logo 参数读不到或标志不为 1 时它不碰 VDP（反汇编 p1 的 fastboot-hi3798mv3dmw-64.bin：入口 0xc29f60 读 env logo，Logo_Main 0xc29d48 两处提前返回，HI_ADP_DispInit 0xc2a03c 只在 0xc29c54 一处调用），那时 VDP 可能没开时钟或还在复位。所以先看 CRG54（VDP 配置时钟开、VOU 复位放开）与 CRG211（VDP 块复位已释放），不满足就不读 VDP 的任何寄存器、什么都不做（docs/14-VDP-HDMI.md 的硬约束：CRG54 bit31 = 0 时不访问 VDP 寄存器文件）。寄存器布局见同一文档的图形层 G0 一行。
 */
#define PERI_CRG54_VO		(CRG_BASE + 0xd8)
#define  VO_VOU_SRST_REQ	BIT(30)
#define  VO_VDP_CFG_CKEN	BIT(31)
#define PERI_CRG211		(CRG_BASE + 0x34c)
#define  CRG211_VDP_SRST	BIT(29)
#define VDP_BASE		0xf8cc0000UL
#define VDP_G0_CTRL		(VDP_BASE + 0x7000)
#define  VDP_G0_EN		BIT(31)
#define VDP_G0_STRIDE		(VDP_BASE + 0x701c)	/* 行距，16 字节为单位 */
#define  VDP_G0_STRIDE_MASK	GENMASK(15, 0)
#define VDP_G0_IRESO		(VDP_BASE + 0x7020)	/* [11:0] 宽 - 1，[23:12] 高 - 1 */
#define  VDP_G0_IRESO_H		GENMASK(23, 12)
#define VDP_G0_ADDR_WORKED	(VDP_BASE + 0x70e4)	/* 已锁存、正在扫描的地址 */
#define CM201_LOGO_FB_MAX	SZ_64M

/*
 * fastboot 留下的 logo 帧缓冲：VDP 上了电、G0 开着、区间整个落在 DDR 上半部且大小合理时填 base/size（按 4K 页对齐、含页内偏移）并返回 true。只认上半部：fastboot 把帧缓冲放在 DDR 顶部附近（实测 0x3f4b9000）；放在低处会压到 BL31（0x04400000）、正在运行的 U-Boot 或 FIT 的载入区，那时保护它反而让 lmb 挡住各级启动源，不如不管。
 */
static bool cm201_logo_fb(phys_addr_t *base, phys_size_t *size)
{
	u32 crg54 = readl(PERI_CRG54_VO), stride, height;
	phys_addr_t addr;
	phys_size_t len;

	if (!(crg54 & VO_VDP_CFG_CKEN) || (crg54 & VO_VOU_SRST_REQ) || (readl(PERI_CRG211) & CRG211_VDP_SRST))
		return false;
	if (!(readl(VDP_G0_CTRL) & VDP_G0_EN))
		return false;
	stride = FIELD_GET(VDP_G0_STRIDE_MASK, readl(VDP_G0_STRIDE)) * 16;
	height = FIELD_GET(VDP_G0_IRESO_H, readl(VDP_G0_IRESO)) + 1;
	addr = readl(VDP_G0_ADDR_WORKED);
	if (!addr || !stride)
		return false;
	len = ALIGN((addr & (SZ_4K - 1)) + (phys_size_t)stride * height, SZ_4K);
	addr = ALIGN_DOWN(addr, SZ_4K);
	if (len > CM201_LOGO_FB_MAX || addr < gd->ram_base + gd->ram_size / 2 || addr + len > gd->ram_base + gd->ram_size)
		return false;
	*base = addr;
	*size = len;
	return true;
}

phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
	phys_addr_t base;
	phys_size_t size;

	if (cm201_logo_fb(&base, &size) && base < gd->ram_top)
		return base;
	return gd->ram_top;
}

int board_late_init(void)
{
	phys_addr_t base, addr;
	phys_size_t size;

	if (!cm201_logo_fb(&base, &size))
		return 0;
	addr = base;
	if (lmb_alloc_mem(LMB_MEM_ALLOC_ADDR, 0, &addr, size, LMB_NOOVERWRITE))
		printf("CM201-2: logo framebuffer 0x%08llx+0x%llx: lmb reserve failed, U-Boot may overwrite it\n",
		       (unsigned long long)base, (unsigned long long)size);
	else
		printf("CM201-2: logo framebuffer 0x%08llx-0x%08llx reserved from U-Boot allocations\n",
		       (unsigned long long)base, (unsigned long long)(base + size - 1));
	return 0;
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	struct fdt_memory fb;
	phys_addr_t base;
	phys_size_t size;
	int ret;

	if (!cm201_logo_fb(&base, &size))
		return 0;
	fb.start = base;
	fb.end = base + size - 1;
	ret = fdtdec_add_reserved_memory(blob, "framebuffer", &fb, NULL, 0, NULL, FDTDEC_RESERVED_MEMORY_NO_MAP);
	if (ret)
		printf("CM201-2: cannot reserve logo framebuffer in the kernel device tree (%d); the logo may be overwritten\n", ret);
	/* 加不上只影响 logo 显示，不拦启动 */
	return 0;
}

static void cm201_emmc_clock_init(void)
{
	u32 val = readl(PERI_CRG40_EMMC);

	val &= ~(EMMC_CLK_SEL | EMMC_SAMPLE_PHASE | EMMC_DRV_PHASE);
	val |= FIELD_PREP(EMMC_CLK_SEL, EMMC_CLK_SEL_50M) |
	       FIELD_PREP(EMMC_SAMPLE_PHASE, EMMC_PHASE_0) |
	       FIELD_PREP(EMMC_DRV_PHASE, EMMC_PHASE_180);

	setbits_le32(PERI_CRG231_SAP_DLL, SAP_DLL_DLLMODE);

	/* 与厂商 fastboot 初始化时给该模块的脉冲相同。 */
	writel(val | EMMC_SRST_REQ, PERI_CRG40_EMMC);
	udelay(1000);
	writel((val & ~EMMC_SRST_REQ) | EMMC_BIU_CKEN | EMMC_CIU_CKEN,
	       PERI_CRG40_EMMC);
	udelay(1000);
}

static void cm201_femac_clock_init(void)
{
	setbits_le32(PERI_CRG52_FEMAC, FEMAC_BUS_CKEN | FEMAC_CKEN);
	setbits_le32(PERI_CRG226_FEPHY, FEPHY_CKEN);
}

static void cm201_usb2_init(void)
{
	unsigned int i, port;

	cm201_usb2_saved_crg46 = readl(PERI_CRG46_USB2);
	cm201_usb2_saved_crg47 = readl(PERI_CRG47_USB2_PHY);

	clrbits_le32(PERI_CRG46_USB2, USB2_BUS_SRST | USB2_UTMI0_SRST |
		     USB2_UTMI1_SRST | USB2_HST_PHY_SRST);
	setbits_le32(PERI_CRG46_USB2, USB2_BUS_CKEN | USB2_HST_PHY_CKEN |
		     USB2_UTMI0_CKEN | USB2_UTMI1_CKEN);

	setbits_le32(PERI_CRG47_USB2_PHY, USB2_PHY1_REF_CKEN);
	udelay(100);
	clrbits_le32(PERI_CRG47_USB2_PHY, USB2_PHY1_POR_SRST | USB2_PHY1_TPOR_SRST);
	udelay(300);

	for (i = 0; i < ARRAY_SIZE(cm201_usb2_phy_setup); i++) {
		for (port = 0; port < USB2_PHY1_PORTS; port++)
			writel(cm201_usb2_phy_setup[i].val,
			       USB2_PHY1_BASE + port * USB2_PHY1_PORT_STRIDE +
			       cm201_usb2_phy_setup[i].reg * 4);
		udelay(cm201_usb2_phy_setup[i].delay_us);
	}

	clrbits_le32(PERI_CRG47_USB2_PHY, USB2_PHY1_PORT0_SRST | USB2_PHY1_PORT1_SRST);
	udelay(2);
}

int board_init(void)
{
	cm201_emmc_clock_init();
	cm201_femac_clock_init();
	cm201_usb2_init();

	return 0;
}

void board_quiesce_devices(void)
{
	usb_stop();
	writel(cm201_usb2_saved_crg46, PERI_CRG46_USB2);
	writel(cm201_usb2_saved_crg47, PERI_CRG47_USB2_PHY);
}
