// SPDX-License-Identifier: GPL-2.0+
/*
 * EcoNet EN7528 硬件看门狗：SoC 定时器块（0x1fbf0100）的 timer 5。
 *
 * 寄存器模型与 Linux 侧的 en7528_wdt 相同：CTRL（+0x00）的 bit5 开 timer 5、bit25 开看门狗复位、bit21 是 timer 5 的中断使能（不开，没有谁处理这个中断）；LOAD（+0x2c）是 32 位重装值，CUR（+0x30）是当前值，RELOAD（+0x38）写 1 立即重装。计数时钟取自 DT 的 clocks（GS2107 实测 112.5 MHz），32 位重装值把超时限制在约 38 s。
 *
 * 跳内核时计时器不停：Linux 的 en7528_wdt 在初始化时见 CTRL 的 bit5 与 bit25 都置位，就按自己的超时重装并代喂（WDOG_HW_RUNNING），直到用户态打开 /dev/watchdog，所以从 U-Boot 到内核早期的卡死都会复位。
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <wdt.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/math64.h>

#define EN7528_WDT_CTRL		0x00
#define EN7528_WDT_LOAD		0x2c
#define EN7528_WDT_RELOAD	0x38

#define EN7528_WDT_TIMER_ENABLE	BIT(5)
#define EN7528_WDT_IRQ_ENABLE	BIT(21)
#define EN7528_WDT_RESET_ENABLE	BIT(25)

struct en7528_wdt_priv {
	void __iomem *base;
	ulong rate;
};

static int en7528_wdt_reset(struct udevice *dev)
{
	struct en7528_wdt_priv *priv = dev_get_priv(dev);

	writel(1, priv->base + EN7528_WDT_RELOAD);

	return 0;
}

static int en7528_wdt_start(struct udevice *dev, u64 timeout_ms, ulong flags)
{
	struct en7528_wdt_priv *priv = dev_get_priv(dev);
	u64 count = div_u64(timeout_ms * priv->rate, 1000);
	u32 ctrl;

	/* 超出 32 位重装值时取最长（约 38 s），同 Linux 侧按 max_timeout 夹住；uclass 打印的是请求值，这里另报实际值 */
	if (count > U32_MAX)
		dev_warn(dev, "timeout %llu ms exceeds the 32-bit counter, using %lu ms\n",
			 timeout_ms, (ulong)div_u64((u64)U32_MAX * 1000, priv->rate));
	count = clamp_t(u64, count, 1, U32_MAX);
	writel(count, priv->base + EN7528_WDT_LOAD);

	ctrl = readl(priv->base + EN7528_WDT_CTRL);
	ctrl &= ~EN7528_WDT_IRQ_ENABLE;
	ctrl |= EN7528_WDT_TIMER_ENABLE | EN7528_WDT_RESET_ENABLE;
	writel(ctrl, priv->base + EN7528_WDT_CTRL);

	/* 计时器原本就在跑（上一级引导打开过）时，新的 LOAD 要重装一次才装入 */
	return en7528_wdt_reset(dev);
}

static int en7528_wdt_stop(struct udevice *dev)
{
	struct en7528_wdt_priv *priv = dev_get_priv(dev);

	clrbits_le32(priv->base + EN7528_WDT_CTRL, EN7528_WDT_TIMER_ENABLE |
		     EN7528_WDT_IRQ_ENABLE | EN7528_WDT_RESET_ENABLE);

	return 0;
}

static int en7528_wdt_expire_now(struct udevice *dev, ulong flags)
{
	return en7528_wdt_start(dev, 1, flags);
}

static int en7528_wdt_probe(struct udevice *dev)
{
	struct en7528_wdt_priv *priv = dev_get_priv(dev);
	struct clk clk;
	int ret;

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret)
		return ret;
	priv->rate = clk_get_rate(&clk);
	if (!priv->rate || IS_ERR_VALUE(priv->rate))
		return -EINVAL;

	return 0;
}

static const struct wdt_ops en7528_wdt_ops = {
	.start = en7528_wdt_start,
	.stop = en7528_wdt_stop,
	.reset = en7528_wdt_reset,
	.expire_now = en7528_wdt_expire_now,
};

static const struct udevice_id en7528_wdt_ids[] = {
	{ .compatible = "econet,en7528-wdt" },
	{ }
};

U_BOOT_DRIVER(en7528_wdt) = {
	.name = "en7528_wdt",
	.id = UCLASS_WDT,
	.of_match = en7528_wdt_ids,
	.probe = en7528_wdt_probe,
	.priv_auto = sizeof(struct en7528_wdt_priv),
	.ops = &en7528_wdt_ops,
};
