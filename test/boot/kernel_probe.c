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

#if CONFIG_IS_ENABLED(ANDROID_BOOT_IMAGE)
#include <android_image.h>
#include <command.h>
#include <gzip.h>
#include <malloc.h>
#include <linux/libfdt.h>

/* Android 启动映像（头版本 1）放在这里；布局取联通 M12 的 boot.img：kernel_addr 0x01080000、ramdisk_addr 0x01000000、页 2 KiB */
#define BOOTIMG_ADDR	0x2000000
#define BOOTIMG_PAGE	0x800
#define M12_KERNEL_ADDR	0x01080000
#define M12_RD_ADDR	0x01000000
#define M12_RD_SIZE	0x180000
#define FAR_KERNEL_ADDR	0x03000000
#define ZIMAGE_SIZE	0x100000
#define FDT_ADDR	0x04000000
/* Android 映像函数收强转成指针的地址 */
#define BOOTIMG		((const void *)BOOTIMG_ADDR)
/* bootm 命令：启动映像兼作 ramdisk 来源，FDT 显式给出 */
#define BOOTM_START	"bootm start 2000000 2000000 4000000"

/* 在 FDT_ADDR 放一个空 FDT */
static int make_fdt(struct unit_test_state *uts)
{
	ut_assertok(fdt_create_empty_tree(map_sysmem(FDT_ADDR, 0x1000), 0x1000));

	return 0;
}

/* 写启动映像头与 ramdisk（内容 0x5a），内核由调用方写在头后一页 */
static u8 *make_bootimg(ulong kernel_size, ulong kernel_addr)
{
	struct andr_boot_img_hdr_v0 *hdr;
	u8 *img;

	img = map_sysmem(BOOTIMG_ADDR, BOOTIMG_PAGE + ALIGN(kernel_size, BOOTIMG_PAGE) + M12_RD_SIZE);
	hdr = (struct andr_boot_img_hdr_v0 *)img;
	memset(hdr, 0, BOOTIMG_PAGE);
	memcpy(hdr->magic, ANDR_BOOT_MAGIC, ANDR_BOOT_MAGIC_SIZE);
	hdr->kernel_size = kernel_size;
	hdr->kernel_addr = kernel_addr;
	hdr->ramdisk_size = M12_RD_SIZE;
	hdr->ramdisk_addr = M12_RD_ADDR;
	hdr->page_size = BOOTIMG_PAGE;
	hdr->header_version = 1;
	hdr->header_size = sizeof(*hdr);
	memset(img + BOOTIMG_PAGE + ALIGN(kernel_size, BOOTIMG_PAGE), 0x5a, M12_RD_SIZE);

	return img;
}

/* 内核外套 uImage（M12 厂商格式）时按 uImage 头放置；裸内核按启动映像头；截断的 uImage 被拒 */
static int kernel_probe_test_android_info(struct unit_test_state *uts)
{
	struct image_kernel_info info;
	struct legacy_img_hdr *uhdr;
	u8 *img;

	img = make_bootimg(sizeof(*uhdr) + 0x400, M12_KERNEL_ADDR);
	uhdr = (struct legacy_img_hdr *)(img + BOOTIMG_PAGE);
	make_zimage(uhdr + 1, 0x400);
	make_legacy(uhdr, 0x400, IH_ARCH_ARM, IH_TYPE_KERNEL, IH_COMP_NONE);
	ut_assertok(android_image_get_kernel_info(BOOTIMG, NULL, true, &info));
	ut_assert(info.legacy);
	ut_asserteq(IH_ARCH_ARM, info.arch);
	ut_asserteq(UIMAGE_LOAD, info.load);
	ut_asserteq(UIMAGE_LOAD, info.ep);
	ut_asserteq(BOOTIMG_ADDR + BOOTIMG_PAGE + sizeof(*uhdr), info.data);
	ut_asserteq(0x400, info.len);

	/* 启动映像头的 kernel_size 比 uImage 声明的少一个字节 */
	img = make_bootimg(sizeof(*uhdr) + 0x400 - 1, M12_KERNEL_ADDR);
	ut_asserteq(-E2BIG, android_image_get_kernel_info(BOOTIMG, NULL, true, &info));

	img = make_bootimg(0x400, M12_KERNEL_ADDR);
	make_arm64_image(img + BOOTIMG_PAGE, 0x400);
	ut_assertok(android_image_get_kernel_info(BOOTIMG, NULL, true, &info));
	ut_assert(!info.legacy);
	ut_asserteq(IH_ARCH_ARM64, info.arch);
	ut_asserteq(M12_KERNEL_ADDR, info.load);
	ut_asserteq(M12_KERNEL_ADDR, info.ep);

	/* 认不出不算错，架构留给加载后判定 */
	img = make_bootimg(0x400, M12_KERNEL_ADDR);
	memset(img + BOOTIMG_PAGE, 0, 0x400);
	ut_assertok(android_image_get_kernel_info(BOOTIMG, NULL, true, &info));
	ut_asserteq(IH_ARCH_INVALID, info.arch);
	ut_asserteq(IH_COMP_NONE, info.comp);

	unmap_sysmem(img);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_android_info, 0);

/*
 * bootm 引导 M12 布局的启动映像：内核按 uImage 放到 0x00108000、架构为 32 位 Arm，暂存在 ramdisk_addr 的 ramdisk 不被碰。abootimg addr 指向别处，确认 bootm 读的是参数所指的启动映像
 */
static int kernel_probe_test_bootm_m12(struct unit_test_state *uts)
{
	struct legacy_img_hdr *uhdr;
	u8 *img, *rd;

	img = make_bootimg(sizeof(*uhdr) + ZIMAGE_SIZE, M12_KERNEL_ADDR);
	uhdr = (struct legacy_img_hdr *)(img + BOOTIMG_PAGE);
	make_zimage(uhdr + 1, ZIMAGE_SIZE);
	make_legacy(uhdr, ZIMAGE_SIZE, IH_ARCH_ARM, IH_TYPE_KERNEL, IH_COMP_NONE);

	ut_assertok(make_fdt(uts));
	ut_assertok(run_command("abootimg addr 0", 0));
	ut_assertok(run_command(BOOTM_START, 0));
	ut_assert_skip_to_line("Kernel uImage: uncompressed, arch ARM");
	ut_assert_nextline("Kernel load addr 0x00108000 size 1024 KiB");
	ut_assertok(run_command("bootm loados", 0));
	ut_asserteq(IH_ARCH_ARM, images.os.arch);
	ut_asserteq(UIMAGE_LOAD, images.ep);
	ut_asserteq_mem(uhdr + 1, map_sysmem(UIMAGE_LOAD, ZIMAGE_SIZE), ZIMAGE_SIZE);

	rd = map_sysmem(M12_RD_ADDR, M12_RD_SIZE);
	ut_asserteq(M12_RD_ADDR, images.rd_start);
	ut_asserteq(0x5a, rd[0]);
	ut_asserteq(0x5a, rd[M12_RD_SIZE - 1]);

	/* 复原成没设过 abootimg addr，不影响后面的用例 */
	set_abootimg_addr(-1);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_bootm_m12, UTF_CONSOLE);

/* ramdisk_addr 落在启动映像自身上：拷 ramdisk 之前就报错，映像里的内核不被写坏 */
static int kernel_probe_test_bootm_rd_on_image(struct unit_test_state *uts)
{
	struct andr_boot_img_hdr_v0 *hdr;
	struct legacy_img_hdr *uhdr;
	u8 *img;

	img = make_bootimg(sizeof(*uhdr) + ZIMAGE_SIZE, M12_KERNEL_ADDR);
	uhdr = (struct legacy_img_hdr *)(img + BOOTIMG_PAGE);
	make_zimage(uhdr + 1, ZIMAGE_SIZE);
	make_legacy(uhdr, ZIMAGE_SIZE, IH_ARCH_ARM, IH_TYPE_KERNEL, IH_COMP_NONE);
	hdr = (struct andr_boot_img_hdr_v0 *)img;
	hdr->ramdisk_addr = BOOTIMG_ADDR + BOOTIMG_PAGE;

	ut_assertok(make_fdt(uts));
	ut_asserteq(1, run_command(BOOTM_START, 0));
	ut_assert_skip_to_line("ERROR: ramdisk_addr 0x2000800..0x2180800 overlaps the boot image at 0x2000000..0x2281000");
	ut_asserteq(IH_MAGIC, image_get_magic(uhdr));
	ut_assert(image_check_dcrc(uhdr));

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_bootm_rd_on_image, UTF_CONSOLE);

/* 裸内核照启动映像头放到 0x01080000 会压住暂存在 0x01000000 的 1.5 MiB ramdisk：报错，且写之前就停 */
static int kernel_probe_test_bootm_overlap(struct unit_test_state *uts)
{
	u8 *img, *rd;

	img = make_bootimg(ZIMAGE_SIZE, M12_KERNEL_ADDR);
	make_zimage(img + BOOTIMG_PAGE, ZIMAGE_SIZE);

	ut_assertok(make_fdt(uts));
	ut_assertok(run_command(BOOTM_START, 0));
	ut_assert_skip_to_line("Kernel data: uncompressed, arch ARM");
	ut_asserteq(1, run_command("bootm loados", 0));
	ut_assert_skip_to_line("ERROR: RD image overlaps OS image (OS=1080000..1180000)");

	rd = map_sysmem(M12_KERNEL_ADDR, ZIMAGE_SIZE);
	ut_asserteq(0x5a, rd[0]);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_bootm_overlap, UTF_CONSOLE);

/* 把 uImage 头的 load/ep 改成 @load 并重算头 CRC */
static void legacy_set_load(struct legacy_img_hdr *hdr, ulong load)
{
	image_set_load(hdr, load);
	image_set_ep(hdr, load);
	legacy_set_hcrc(hdr);
}

/* uImage 声明的 load 落在启动映像里（gzip 与不压缩各一次）：写之前报错，启动映像一个字节都不变 */
static int kernel_probe_test_bootm_load_in_image(struct unit_test_state *uts)
{
	const ulong load = BOOTIMG_ADDR + 0x100000;
	struct legacy_img_hdr *uhdr;
	ulong zlen = 0x40000, total;
	u8 *img, *zimage;
	u32 crc;

	zimage = malloc(ZIMAGE_SIZE);
	ut_assertnonnull(zimage);
	make_zimage(zimage, ZIMAGE_SIZE);
	img = make_bootimg(sizeof(*uhdr) + zlen, M12_KERNEL_ADDR);
	uhdr = (struct legacy_img_hdr *)(img + BOOTIMG_PAGE);
	ut_assertok(gzip(uhdr + 1, &zlen, zimage, ZIMAGE_SIZE));
	img = make_bootimg(sizeof(*uhdr) + zlen, M12_KERNEL_ADDR);
	make_legacy(uhdr, zlen, IH_ARCH_ARM, IH_TYPE_KERNEL, IH_COMP_GZIP);
	legacy_set_load(uhdr, load);
	total = BOOTIMG_PAGE + ALIGN(sizeof(*uhdr) + zlen, BOOTIMG_PAGE) + M12_RD_SIZE;
	crc = crc32(0, img, total);

	ut_assertok(make_fdt(uts));
	ut_assertok(run_command(BOOTM_START, 0));
	ut_assert_skip_to_line("Kernel uImage: gzip compressed, arch ARM");
	ut_asserteq(1, run_command("bootm loados", 0));
	ut_assert_skip_to_line("ERROR: kernel at 0x%lx would overwrite the image at 0x%x..0x%lx",
			       load, BOOTIMG_ADDR, BOOTIMG_ADDR + total);
	ut_asserteq(crc, crc32(0, img, total));

	img = make_bootimg(sizeof(*uhdr) + ZIMAGE_SIZE, M12_KERNEL_ADDR);
	memcpy(uhdr + 1, zimage, ZIMAGE_SIZE);
	make_legacy(uhdr, ZIMAGE_SIZE, IH_ARCH_ARM, IH_TYPE_KERNEL, IH_COMP_NONE);
	legacy_set_load(uhdr, load);
	total = BOOTIMG_PAGE + ALIGN(sizeof(*uhdr) + ZIMAGE_SIZE, BOOTIMG_PAGE) + M12_RD_SIZE;
	crc = crc32(0, img, total);

	ut_assertok(run_command(BOOTM_START, 0));
	ut_assert_skip_to_line("Kernel uImage: uncompressed, arch ARM");
	ut_asserteq(1, run_command("bootm loados", 0));
	ut_assert_skip_to_line("ERROR: kernel at 0x%lx would overwrite the image at 0x%x..0x%lx",
			       load, BOOTIMG_ADDR, BOOTIMG_ADDR + total);
	ut_asserteq(crc, crc32(0, img, total));
	free(zimage);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_bootm_load_in_image, UTF_CONSOLE);

/* 压缩的内核解压后再认架构；认不出时按本机架构 */
static int kernel_probe_test_bootm_decomp(struct unit_test_state *uts)
{
	ulong zlen = 0x40000;
	u8 *img, *zimage;

	zimage = malloc(ZIMAGE_SIZE);
	ut_assertnonnull(zimage);
	make_zimage(zimage, ZIMAGE_SIZE);
	img = make_bootimg(zlen, FAR_KERNEL_ADDR);
	ut_assertok(gzip(img + BOOTIMG_PAGE, &zlen, zimage, ZIMAGE_SIZE));
	make_bootimg(zlen, FAR_KERNEL_ADDR);

	ut_assertok(make_fdt(uts));
	ut_assertok(run_command(BOOTM_START, 0));
	ut_assert_skip_to_line("Kernel data: gzip compressed, arch probed after decompression");
	ut_asserteq(IH_ARCH_INVALID, images.os.arch);
	ut_assertok(run_command("bootm loados", 0));
	ut_asserteq(IH_ARCH_ARM, images.os.arch);
	ut_asserteq(FAR_KERNEL_ADDR, images.ep);
	ut_asserteq_mem(zimage, map_sysmem(FAR_KERNEL_ADDR, ZIMAGE_SIZE), ZIMAGE_SIZE);

	/* 解压出的 zImage 声明的长度超出实际：拒绝，不按本机架构硬跳 */
	put_unaligned_le32(ZIMAGE_SIZE * 2, zimage + 0x2c);
	zlen = 0x40000;
	img = make_bootimg(zlen, FAR_KERNEL_ADDR);
	ut_assertok(gzip(img + BOOTIMG_PAGE, &zlen, zimage, ZIMAGE_SIZE));
	make_bootimg(zlen, FAR_KERNEL_ADDR);
	ut_assertok(run_command(BOOTM_START, 0));
	ut_asserteq(1, run_command("bootm loados", 0));
	ut_assert_skip_to_line("Kernel zImage: size 0x200000 exceeds the 0x100000 bytes present");
	ut_asserteq(IH_ARCH_INVALID, images.os.arch);
	free(zimage);

	img = make_bootimg(0x1000, FAR_KERNEL_ADDR);
	memset(img + BOOTIMG_PAGE, 0, 0x1000);
	ut_assertok(run_command(BOOTM_START, 0));
	ut_assert_skip_to_line("Kernel data: uncompressed, arch unrecognised");
	ut_assertok(run_command("bootm loados", 0));
	ut_asserteq(IH_ARCH_DEFAULT, images.os.arch);

	return 0;
}
KERNEL_PROBE_TEST(kernel_probe_test_bootm_decomp, UTF_CONSOLE);
#endif
