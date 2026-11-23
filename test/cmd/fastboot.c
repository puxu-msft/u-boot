// SPDX-License-Identifier: GPL-2.0+
/*
 * fastboot 命令：重启标记（补丁 0508：Amlogic 的原因映射 asm/arch-meson/reboot.h，板上由 mach-meson 的 fastboot_set_reboot_flag() 使用；会话没有复位就结束时撤销标记的钩子）与 -t 探测窗口的参数（补丁 0509）
 */

#include <command.h>
#include <fastboot.h>
#include <console.h>
#include <test/cmd.h>
#include <test/ut.h>
#include "../../arch/arm/include/asm/arch-meson/reboot.h"

static int fb_cleared;

/* sandbox 没有板级覆盖：这里只计数，核对 fastboot 命令结束时调了它 */
void fastboot_clear_reboot_flag(void)
{
	fb_cleared++;
}

/* reboot-bootloader、reboot-fastboot 记 fastboot（4）；reboot-recovery 在厂商那边是 factory_reset（2），不支持 */
static int cmd_test_fastboot_reboot_reason(struct unit_test_state *uts)
{
	ut_asserteq(REBOOT_REASON_FASTBOOT,
		    meson_fastboot_reboot_reason(FASTBOOT_REBOOT_REASON_BOOTLOADER));
	ut_asserteq(REBOOT_REASON_FASTBOOT,
		    meson_fastboot_reboot_reason(FASTBOOT_REBOOT_REASON_FASTBOOTD));
	ut_asserteq(-EINVAL, meson_fastboot_reboot_reason(FASTBOOT_REBOOT_REASON_RECOVERY));
	ut_asserteq(-EINVAL, meson_fastboot_reboot_reason(FASTBOOT_REBOOT_REASONS_COUNT));

	return 0;
}
CMD_TEST(cmd_test_fastboot_reboot_reason, 0);

/* 会话没起来（没有这个 USB 控制器）也算结束：标记作废 */
static int cmd_test_fastboot_reboot_clear(struct unit_test_state *uts)
{
	fb_cleared = 0;
	ut_asserteq(1, run_command("fastboot usb 99", 0));
	ut_asserteq(1, fb_cleared);

	return 0;
}
CMD_TEST(cmd_test_fastboot_reboot_clear, 0);

/*
 * -t 的毫秒数与 optimus -t 同样按 simple_strtoul(…, 10) 解析：十进制，另照 U-Boot 的惯例认 0x 前缀的 16 进制（lib/strto.c 的 _parse_integer_fixup_radix）；0、带杂字符、缺值都是用法错误；只用于 usb。
 * sandbox 没开 USB_FUNCTION_FASTBOOT，合格的参数走到“没开 USB fastboot”就返回
 */
static int cmd_test_fastboot_probe(struct unit_test_state *uts)
{
	static const char *const bad[] = {
		"fastboot -t 0 usb 0",
		"fastboot -t 3000x usb 0",
		"fastboot -t 0o10 usb 0",
		"fastboot -t",
	};
	static const char *const good[] = {
		"fastboot -t 3000 usb 0",
		"fastboot -t 0xbb8 usb 0",
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(bad); i++) {
		console_record_reset();
		ut_asserteq(1, run_command(bad[i], 0));
		ut_assert_nextline("fastboot - run as a fastboot usb or udp device");
	}
	if (IS_ENABLED(CONFIG_NET_LEGACY)) {
		console_record_reset();
		ut_asserteq(1, run_command("fastboot -t 3000 udp", 0));
		ut_assert_nextline("fastboot - run as a fastboot usb or udp device");
	}
	for (i = 0; i < ARRAY_SIZE(good); i++) {
		console_record_reset();
		ut_asserteq(1, run_command(good[i], 0));
		ut_assert_skip_to_line("     do_fastboot_usb() Fastboot USB not enabled");
	}

	return 0;
}
CMD_TEST(cmd_test_fastboot_probe, UTF_CONSOLE);
