// SPDX-License-Identifier: GPL-2.0+
/*
 * image_probe_kernel() 的测试：按内容识别 32 位 Arm zImage、arm64 Image、压缩数据与外套的传统 uImage；头里声明的长度超出给定数据长度、uImage 头 CRC 或数据 CRC 不对、类型不是内核时拒绝
 */

#include <image.h>
#include <mapmem.h>
#include <asm/unaligned.h>
#include <linux/string.h>
#include <test/test.h>
#include <test/ut.h>
#include <u-boot/crc.h>

#define KERNEL_PROBE_TEST(_name, _flags)	UNIT_TEST(_name, _flags, bootm)

/* 构造映像用的 sandbox 内存 */
#define PROBE_ADDR	0x100000
#define PROBE_SIZE	0x1000
#define UIMAGE_LOAD	0x00108000

/* 在 @buf 写一个 @size 字节的 zImage：0x24 处魔数，0x28/0x2c 是起止 */
static void make_zimage(void *buf, ulong size)
{
	memset(buf, 0xa5, size);
	put_unaligned_le32(LINUX_ARM_ZIMAGE_MAGIC, buf + 0x24);
	put_unaligned_le32(0, buf + 0x28);
	put_unaligned_le32(size, buf + 0x2c);
}

/* 在 @buf 写一个 @size 字节的 arm64 Image：0x38 处魔数 */
static void make_arm64_image(void *buf, ulong size)
{
	memset(buf, 0, size);
	put_unaligned_le64(0x400000, buf + 0x10);	/* image_size */
	put_unaligned_le64(BIT(3), buf + 0x18);		/* flags：可放在任意 2 MiB 对齐处 */
	put_unaligned_le32(LINUX_ARM64_IMAGE_MAGIC, buf + 0x38);
}

/* 重算 uImage 头 CRC */
static void legacy_set_hcrc(struct legacy_img_hdr *hdr)
{
	image_set_hcrc(hdr, 0);
	image_set_hcrc(hdr, crc32(0, (const u8 *)hdr, sizeof(*hdr)));
}

/* 给 @hdr 之后已就位的 @size 字节数据写 uImage 头 */
static void make_legacy(struct legacy_img_hdr *hdr, ulong size, u8 arch, u8 type,
			u8 comp)
{
	memset(hdr, 0, sizeof(*hdr));
	image_set_magic(hdr, IH_MAGIC);
	image_set_size(hdr, size);
	image_set_load(hdr, UIMAGE_LOAD);
	image_set_ep(hdr, UIMAGE_LOAD);
	image_set_dcrc(hdr, crc32(0, (const u8 *)(hdr + 1), size));
	image_set_os(hdr, IH_OS_LINUX);
	image_set_arch(hdr, arch);
	image_set_type(hdr, type);
	image_set_comp(hdr, comp);
	strlcpy((char *)hdr->ih_name, "probe-test", IH_NMLEN);
	legacy_set_hcrc(hdr);
}

/* 32 位 Arm zImage：认出架构；声明的长度超出数据或起止颠倒都拒绝 */
static int kernel_probe_test_zimage(struct unit_test_state *uts)
{
	struct image_kernel_info info;
	void *buf = map_sysmem(PROBE_ADDR, PROBE_SIZE);

	make_zimage(buf, 0x200);
	ut_assertok(image_probe_kernel(PROBE_ADDR, 0x200, false, &info));
	ut_asserteq(IH_ARCH_ARM, info.arch);
	ut_asserteq(IH_COMP_NONE, info.comp);
	ut_asserteq(IH_TYPE_KERNEL, info.type);
	ut_assert(!info.legacy);
	ut_asserteq(PROBE_ADDR, info.data);
	ut_asserteq(0x200, info.len);

	/* 附加的 DTB 让数据比 zImage 声明的长：仍认出 */
	ut_assertok(image_probe_kernel(PROBE_ADDR, 0x300, false, &info));
	ut_asserteq(IH_ARCH_ARM, info.arch);

	/* 截断：声明 0x200，只给 0x1ff */
	ut_asserteq(-E2BIG, image_probe_kernel(PROBE_ADDR, 0x1ff, false, &info));
	ut_asserteq(IH_ARCH_INVALID, info.arch);

	/* 伪造：终点不大于起点 */
	put_unaligned_le32(0x200, buf + 0x28);
	ut_asserteq(-E2BIG, image_probe_kernel(PROBE_ADDR, 0x200, false, &info));

	unmap_sysmem(buf);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_zimage, 0);

/* arm64 Image：认出架构 */
static int kernel_probe_test_arm64(struct unit_test_state *uts)
{
	struct image_kernel_info info;
	void *buf = map_sysmem(PROBE_ADDR, PROBE_SIZE);

	make_arm64_image(buf, 0x200);
	ut_assertok(image_probe_kernel(PROBE_ADDR, 0x200, false, &info));
	ut_asserteq(IH_ARCH_ARM64, info.arch);
	ut_asserteq(IH_COMP_NONE, info.comp);
	ut_assert(!info.legacy);

	/* 头不完整时不认 */
	ut_asserteq(-ENOEXEC, image_probe_kernel(PROBE_ADDR, 0x3c, false, &info));
	ut_asserteq(IH_ARCH_INVALID, info.arch);

	unmap_sysmem(buf);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_arm64, 0);

/* 压缩数据：认出压缩方式，架构留待解压后；认不出的原样数据返回 -ENOEXEC */
static int kernel_probe_test_comp(struct unit_test_state *uts)
{
	static const u8 gz[] = { 0x1f, 0x8b, 0x08, 0x00 };
	static const u8 lz4[] = { 0x04, 0x22, 0x4d, 0x18 };
	static const u8 zstd[] = { 0x28, 0xb5, 0x2f, 0xfd };
	struct image_kernel_info info;
	u8 *buf = map_sysmem(PROBE_ADDR, PROBE_SIZE);

	memset(buf, 0, 0x100);
	memcpy(buf, gz, sizeof(gz));
	ut_assertok(image_probe_kernel(PROBE_ADDR, 0x100, false, &info));
	ut_asserteq(IH_COMP_GZIP, info.comp);
	ut_asserteq(IH_ARCH_INVALID, info.arch);

	memcpy(buf, lz4, sizeof(lz4));
	ut_assertok(image_probe_kernel(PROBE_ADDR, 0x100, false, &info));
	ut_asserteq(IH_COMP_LZ4, info.comp);

	memcpy(buf, zstd, sizeof(zstd));
	ut_assertok(image_probe_kernel(PROBE_ADDR, 0x100, false, &info));
	ut_asserteq(IH_COMP_ZSTD, info.comp);

	memset(buf, 0, 0x100);
	ut_asserteq(-ENOEXEC, image_probe_kernel(PROBE_ADDR, 0x100, false, &info));
	ut_asserteq(IH_ARCH_INVALID, info.arch);
	ut_asserteq(IH_COMP_NONE, info.comp);
	ut_asserteq(PROBE_ADDR, info.data);
	ut_asserteq(0x100, info.len);

	unmap_sysmem(buf);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_comp, 0);

/* 外套传统 uImage：取头里的 load/ep/arch/comp/type，剥掉头；头 CRC、类型、长度、数据 CRC 不对都拒绝 */
static int kernel_probe_test_legacy(struct unit_test_state *uts)
{
	struct legacy_img_hdr *hdr = map_sysmem(PROBE_ADDR, PROBE_SIZE);
	ulong total = sizeof(*hdr) + 0x200;
	struct image_kernel_info info;

	make_zimage(hdr + 1, 0x200);
	make_legacy(hdr, 0x200, IH_ARCH_ARM, IH_TYPE_KERNEL, IH_COMP_NONE);
	ut_assertok(image_probe_kernel(PROBE_ADDR, total, true, &info));
	ut_assert(info.legacy);
	ut_asserteq(IH_ARCH_ARM, info.arch);
	ut_asserteq(IH_COMP_NONE, info.comp);
	ut_asserteq(IH_TYPE_KERNEL, info.type);
	ut_asserteq(PROBE_ADDR + sizeof(*hdr), info.data);
	ut_asserteq(0x200, info.len);
	ut_asserteq(UIMAGE_LOAD, info.load);
	ut_asserteq(UIMAGE_LOAD, info.ep);

	/* 头里声明的数据比给的多一个字节 */
	ut_asserteq(-E2BIG, image_probe_kernel(PROBE_ADDR, total - 1, true, &info));
	ut_assert(!info.legacy);

	/* 伪造声明长度并重算头 CRC，仍按实际给的长度拒绝 */
	image_set_size(hdr, 0x10000);
	legacy_set_hcrc(hdr);
	ut_asserteq(-E2BIG, image_probe_kernel(PROBE_ADDR, total, false, &info));

	/* 改了头却没重算头 CRC */
	make_legacy(hdr, 0x200, IH_ARCH_ARM, IH_TYPE_KERNEL, IH_COMP_NONE);
	image_set_load(hdr, 0x8000);
	ut_asserteq(-EINVAL, image_probe_kernel(PROBE_ADDR, total, false, &info));

	/* 数据坏了：校验时拒绝，不校验时照常认出 */
	make_legacy(hdr, 0x200, IH_ARCH_ARM, IH_TYPE_KERNEL, IH_COMP_NONE);
	((u8 *)(hdr + 1))[0x100] ^= 0xff;
	ut_asserteq(-EBADMSG, image_probe_kernel(PROBE_ADDR, total, true, &info));
	ut_assertok(image_probe_kernel(PROBE_ADDR, total, false, &info));

	/* 不是内核类型 */
	make_legacy(hdr, 0x200, IH_ARCH_ARM, IH_TYPE_SCRIPT, IH_COMP_NONE);
	ut_asserteq(-EINVAL, image_probe_kernel(PROBE_ADDR, total, false, &info));

	/* KERNEL_NOLOAD 与 arm64、压缩方式按头里的声明 */
	make_arm64_image(hdr + 1, 0x200);
	make_legacy(hdr, 0x200, IH_ARCH_ARM64, IH_TYPE_KERNEL_NOLOAD, IH_COMP_GZIP);
	ut_assertok(image_probe_kernel(PROBE_ADDR, total, true, &info));
	ut_asserteq(IH_ARCH_ARM64, info.arch);
	ut_asserteq(IH_TYPE_KERNEL_NOLOAD, info.type);
	ut_asserteq(IH_COMP_GZIP, info.comp);

	unmap_sysmem(hdr);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_legacy, 0);
