// SPDX-License-Identifier: GPL-2.0+
/*
 * cli_simple_process_macros()：${变量} 展开、转义，以及只在输出缓冲放不下时才返回 -ENOSPC（extlinux/pxe 的 append 靠它判超长，补丁 0309、0310）
 */

#include <cli.h>
#include <env.h>
#include <test/common.h>
#include <test/test.h>
#include <test/ut.h>

static int cli_simple_macros(struct unit_test_state *uts)
{
	char out[16];

	ut_assertok(env_set("clim_v", "abc"));
	ut_assertok(cli_simple_process_macros("x=${clim_v}", out, sizeof(out)));
	ut_asserteq_str("x=abc", out);
	/* 结尾单独的反斜杠被丢掉，不算超长 */
	ut_assertok(cli_simple_process_macros("short\\", out, sizeof(out)));
	ut_asserteq_str("short", out);
	ut_assertok(cli_simple_process_macros("a\\$b", out, sizeof(out)));
	ut_asserteq_str("a$b", out);
	/* 正好放满（15 个字符加 NUL）可以，多一个就 -ENOSPC，输出截在缓冲末尾 */
	ut_assertok(cli_simple_process_macros("0123456789abcde", out, sizeof(out)));
	ut_asserteq_str("0123456789abcde", out);
	ut_asserteq(-ENOSPC, cli_simple_process_macros("0123456789abcdef", out, sizeof(out)));
	ut_asserteq(-ENOSPC, cli_simple_process_macros("${clim_v}${clim_v}${clim_v}${clim_v}${clim_v}x", out, sizeof(out)));
	env_set("clim_v", NULL);

	return 0;
}
COMMON_TEST(cli_simple_macros, 0);
