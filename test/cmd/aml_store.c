// SPDX-License-Identifier: GPL-2.0+
/*
 * store 命令词与 store 后端的测试
 *
 * 用 blkmap 把一块内存盘映射成块设备，按厂商格式手写一张 Amlogic MPT（bootloader、reserved、boot、data），store_dev 指向它。核对：store init 认出设备、拒绝会擦除的 flag；store read 按分区名与分区内偏移读出、k 后缀、越过分区末尾与未知分区与非数字参数都失败且不写内存；store rom_read 读的是物理偏移 + 0x200，跨块的非对齐范围逐字节对、越过盘尾失败；aml_store_write 的非对齐头尾读-改-写只改目标字节。
 */

#include <aml_store.h>
#include <blk.h>
#include <blkmap.h>
#include <command.h>
#include <env.h>
#include <malloc.h>
#include <mapmem.h>
#include <memalign.h>
#include <part.h>
#include <asm/unaligned.h>
#include <dm/device.h>
#include <linux/sizes.h>
#include <test/cmd.h>
#include <test/test.h>
#include <test/ut.h>

#define BLKSZ		512
#define MPT_DISK	SZ_16M
#define MPT_OFF		0xc00000
#define MPT_ENT(i)	(MPT_OFF + 24 + (i) * 40)
#define BOOT_OFF	(MPT_OFF + SZ_1M)

struct store_test {
	struct udevice *dev;
	struct blk_desc *desc;
	u8 *disk;
	u8 *dst;
	ulong dsta;
};

/* Amlogic MPT（与补丁 0201、0401 的测试同一种手写法） */
static void mpt_write(u8 *disk)
{
	static const struct {
		const char *name;
		u64 offset, size;
	} ents[] = {
		{ "bootloader", 0, SZ_4M },
		{ "reserved", MPT_OFF, SZ_1M },
		{ "boot", BOOT_OFF, SZ_1M },
		{ "data", MPT_OFF + SZ_2M, MPT_DISK - MPT_OFF - SZ_2M },
	};
	u32 sum = 0;
	uint i;

	memcpy(disk + MPT_OFF, "MPT", 4);
	strcpy((char *)disk + MPT_OFF + 4, "01.00.00");
	put_unaligned_le32(ARRAY_SIZE(ents), disk + MPT_OFF + 16);
	for (i = 0; i < ARRAY_SIZE(ents); i++) {
		strncpy((char *)disk + MPT_ENT(i), ents[i].name, 16);
		put_unaligned_le64(ents[i].size, disk + MPT_ENT(i) + 16);
		put_unaligned_le64(ents[i].offset, disk + MPT_ENT(i) + 24);
	}
	for (i = 0; i < 10; i++)
		sum += get_unaligned_le32(disk + MPT_ENT(0) + i * 4);
	put_unaligned_le32(ARRAY_SIZE(ents) * sum, disk + MPT_OFF + 20);
}

static int store_setup(struct unit_test_state *uts, struct store_test *t)
{
	struct udevice *blk;
	uint i;

	t->disk = calloc(1, MPT_DISK);
	ut_assertnonnull(t->disk);
	/* 每个字节都可区分：按位置算出的伪随机图样，MPT 表区之后写表 */
	for (i = 0; i < MPT_DISK; i += 4)
		put_unaligned_le32(i * 2654435761u, t->disk + i);
	memset(t->disk + MPT_OFF, 0, SZ_16K);
	mpt_write(t->disk);
	ut_assertok(blkmap_create("store", &t->dev));
	ut_assertok(blkmap_map_mem(t->dev, 0, MPT_DISK / BLKSZ, t->disk));
	ut_assertok(blk_get_from_parent(t->dev, &blk));
	t->desc = dev_get_uclass_plat(blk);
	ut_assertok(run_commandf("setenv store_dev blkmap %d", t->desc->devnum));
	t->dst = memalign(ARCH_DMA_MINALIGN, SZ_64K);
	ut_assertnonnull(t->dst);
	t->dsta = map_to_sysmem(t->dst);

	return 0;
}

static int store_teardown(struct unit_test_state *uts, struct store_test *t)
{
	env_set("store_dev", NULL);
	ut_assertok(blkmap_destroy(t->dev));
	free(t->disk);
	free(t->dst);

	return 0;
}

static int cmd_test_aml_store_init(struct unit_test_state *uts)
{
	struct store_test t;

	if (CONFIG_AML_PARTITION_OFFSET != MPT_OFF)
		return -EAGAIN;
	ut_assertok(store_setup(uts, &t));
	ut_assertok(run_command("store init", 0));
	ut_assertok(run_command("store init 0", 0));
	ut_assertok(run_command("store init 1", 0));
	ut_asserteq(PART_TYPE_AML, t.desc->part_type);
	/* 厂商 2、3 会擦除整个存储 */
	ut_asserteq(1, run_command("store init 2", 0));
	ut_asserteq(1, run_command("store init 3", 0));
	ut_assertok(env_set("store_dev", "blkmap 99"));
	ut_asserteq(1, run_command("store init", 0));
	ut_assertok(env_set("store_dev", "blkmap"));
	ut_asserteq(1, run_command("store init", 0));

	return store_teardown(uts, &t);
}
CMD_TEST(cmd_test_aml_store_init, 0);

static int cmd_test_aml_store_read(struct unit_test_state *uts)
{
	struct store_test t;

	if (CONFIG_AML_PARTITION_OFFSET != MPT_OFF)
		return -EAGAIN;
	ut_assertok(store_setup(uts, &t));

	/* 分区内偏移 0x1234、长度 0x3000：非对齐的头尾 */
	memset(t.dst, 0xee, SZ_64K);
	ut_assertok(run_commandf("store read boot %lx 1234 3000", t.dsta));
	ut_asserteq_mem(t.disk + BOOT_OFF + 0x1234, t.dst, 0x3000);
	ut_asserteq(0xee, t.dst[0x3000]);

	/* 0x 前缀与 k 后缀（厂商 str2longlong）：偏移 1 KiB */
	memset(t.dst, 0xee, SZ_64K);
	ut_assertok(run_commandf("store read boot 0x%lx 1k 0x200", t.dsta));
	ut_asserteq_mem(t.disk + BOOT_OFF + SZ_1K, t.dst, 0x200);

	/* 正好读到分区末尾可以，越过一字节不行，也不写内存 */
	ut_assertok(run_commandf("store read boot %lx ffe00 200", t.dsta));
	ut_asserteq_mem(t.disk + BOOT_OFF + SZ_1M - 0x200, t.dst, 0x200);
	memset(t.dst, 0xee, SZ_64K);
	ut_asserteq(1, run_commandf("store read boot %lx ffe00 201", t.dsta));
	ut_asserteq(1, run_commandf("store read boot %lx 100000 1", t.dsta));
	ut_asserteq(1, run_commandf("store read boot %lx 1m 1", t.dsta));
	ut_asserteq(1, run_commandf("store read nosuch %lx 0 200", t.dsta));
	ut_asserteq(1, run_commandf("store read boot %lx 12z 200", t.dsta));
	ut_asserteq(1, run_commandf("store read boot %lx 0 ", t.dsta));
	ut_asserteq(0xee, t.dst[0]);

	return store_teardown(uts, &t);
}
CMD_TEST(cmd_test_aml_store_read, 0);

static int cmd_test_aml_store_rom_read(struct unit_test_state *uts)
{
	struct store_test t;

	if (CONFIG_AML_PARTITION_OFFSET != MPT_OFF)
		return -EAGAIN;
	ut_assertok(store_setup(uts, &t));

	/* store 坐标 0 是物理 0x200 */
	ut_assertok(run_commandf("store rom_read %lx 0 400", t.dsta));
	ut_asserteq_mem(t.disk + 0x200, t.dst, 0x400);
	ut_assertok(run_commandf("store rom_read %lx 123 777", t.dsta));
	ut_asserteq_mem(t.disk + 0x323, t.dst, 0x777);
	/* 盘尾：store 坐标 盘长 - 0x400 起 0x200 字节正好到盘尾，再多一字节就越过 */
	ut_assertok(run_commandf("store rom_read %lx %x 200", t.dsta, MPT_DISK - 0x400));
	ut_asserteq_mem(t.disk + MPT_DISK - 0x200, t.dst, 0x200);
	memset(t.dst, 0xee, SZ_64K);
	ut_asserteq(1, run_commandf("store rom_read %lx %x 201", t.dsta, MPT_DISK - 0x400));
	ut_asserteq(1, run_commandf("store rom_read %lx ffffffffffffffff 1", t.dsta));
	ut_asserteq(0xee, t.dst[0]);

	return store_teardown(uts, &t);
}
CMD_TEST(cmd_test_aml_store_rom_read, 0);

/* aml_store_write：Optimus 协议的 download store 用它写分区，非对齐头尾读-改-写 */
static int cmd_test_aml_store_write(struct unit_test_state *uts)
{
	struct store_test t;
	struct disk_partition info;
	struct blk_desc *desc;
	u8 *before;
	u64 off;
	uint i;

	if (CONFIG_AML_PARTITION_OFFSET != MPT_OFF)
		return -EAGAIN;
	ut_assertok(store_setup(uts, &t));
	ut_assertok(aml_store_part("boot", &desc, &info));
	ut_asserteq_ptr(t.desc, desc);
	ut_asserteq(BOOT_OFF / BLKSZ, info.start);
	ut_asserteq(-ENOENT, aml_store_part("nosuch", &desc, &info));

	before = malloc(MPT_DISK);
	ut_assertnonnull(before);
	memcpy(before, t.disk, MPT_DISK);
	for (i = 0; i < SZ_64K; i++)
		t.dst[i] = i * 7 + 1;
	/* 头尾都不对齐、中段跨多块：0x1ff 起 0x8003 字节 */
	off = BOOT_OFF + 0x1ff;
	ut_assertok(aml_store_write(desc, off, 0x8003, t.dst));
	ut_asserteq_mem(t.dst, t.disk + off, 0x8003);
	ut_asserteq_mem(before, t.disk, off);
	ut_asserteq_mem(before + off + 0x8003, t.disk + off + 0x8003,
			MPT_DISK - off - 0x8003);
	/* 越过盘尾整次拒绝，盘不变 */
	memcpy(before, t.disk, MPT_DISK);
	ut_asserteq(-EINVAL, aml_store_write(desc, MPT_DISK - 0x100, 0x101, t.dst));
	ut_asserteq_mem(before, t.disk, MPT_DISK);
	free(before);

	return store_teardown(uts, &t);
}
CMD_TEST(cmd_test_aml_store_write, 0);
