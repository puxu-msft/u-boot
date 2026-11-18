// SPDX-License-Identifier: GPL-2.0+
/*
 * 经 Amlogic 厂商 BL31 的安全存储读 unifykey，见 include/meson/unifykey.h
 *
 * 协议依据 M12（SM1）厂商 BL31 v1.3:079690f10 的反汇编（基址 0x05100000）：存储类调用的参数与应答都经 BL31 给的共享缓冲，输入缓冲里的整数是小端 u32，名字不带 NUL；STATUS/TELL 的名字在输入 +4，READ 在名字前多一个 buflen、名字在 +8（0x51213b4 与 0x51213e8 的取址不同）。
 * 共享缓冲由 BL31 映射为非安全普通可缓存内存，与主线页表属性一致；每次调用前后仍对用到的范围做 flush_dcache_range（clean + invalidate），不用只 invalidate 的操作：它会丢掉 BL31 刚写、还在缓存里的应答。
 */

#define LOG_CATEGORY UCLASS_SM

#include <blk.h>
#include <cpu_func.h>
#include <dm.h>
#include <malloc.h>
#include <mapmem.h>
#include <memalign.h>
#include <part.h>
#include <sm.h>
#include <asm/cache.h>
#include <asm/ptrace.h>
#include <asm/unaligned.h>
#include <linux/sizes.h>
#include <meson/sm.h>
#include <meson/unifykey.h>
#include <u-boot/sha256.h>

/* 四块共享缓冲的基址间距，BL31 页表把 0x05000000 起 1 MiB 映射给普通世界 */
#define UK_SHM_SIZE		SZ_256K
/* READ 的应答是 u32 长度 + 数据，都在输出缓冲里 */
#define UK_DATA_MAX		(UK_SHM_SIZE - 4)
/* BL31 对未知调用与部分前置检查失败返回 0xffffffff；meson_sm_call() 截成 u32 后交出 */
#define UK_SMC_UNK		0xffffffffU
#define UK_INFO_SIZE		512
#define UK_HDR_SIZE		0x200
#define UK_HDR_MAGIC		"AMLSECURITY"	/* 连同结尾的 NUL 比 12 字节：BL31 用 strcmp */
#define UK_HDR_VERSION		0x10
#define UK_HDR_ENCTYPE		0x14
#define UK_HDR_SHA		0x18

struct uk {
	struct udevice *sm;
	u8 *in;
	u8 *out;
	u8 *block;
};

/* 讲 Amlogic 存储调用的安全监控：实机是 meson_sm，sandbox 是厂商 BL31 的存储模型 */
static int uk_sm(struct udevice **devp)
{
#if IS_ENABLED(CONFIG_MESON_SM)
	return uclass_get_device_by_driver(UCLASS_SM, DM_DRIVER_GET(meson_sm), devp);
#elif IS_ENABLED(CONFIG_SANDBOX_MESON_SM)
	return uclass_get_device_by_driver(UCLASS_SM, DM_DRIVER_GET(sandbox_meson_sm), devp);
#else
	return -ENODEV;
#endif
}

static void uk_flush(const void *p, size_t len)
{
	ulong start = (ulong)p;

	flush_dcache_range(ALIGN_DOWN(start, ARCH_DMA_MINALIGN),
			   ALIGN(start + len, ARCH_DMA_MINALIGN));
}

/* 发一次调用，x1 为唯一参数；安全监控驱动不认这个索引时（-ENOENT 等）当作没有存储调用 */
static int uk_call(struct uk *uk, u32 cmd, ulong x1, u32 *ret)
{
	struct pt_regs regs = { };
	s32 r;

	regs.regs[1] = x1;
	if (sm_call(uk->sm, cmd, &r, &regs))
		return -ENOSYS;
	*ret = r;

	return 0;
}

/* 取址类调用：BL31 返回 SMC_UNK 即没有这项服务（上游 TF-A 的 G12A 平台就是这样） */
static int uk_addr(struct uk *uk, u32 cmd, u8 **p)
{
	u32 v;
	int ret;

	ret = uk_call(uk, cmd, 0, &v);
	if (ret)
		return ret;
	if (v == UK_SMC_UNK || !v)
		return -ENOSYS;
	*p = map_sysmem(v, UK_SHM_SIZE);

	return 0;
}

static int uk_open(struct uk *uk)
{
	int ret;

	if (uk_sm(&uk->sm))
		return -ENODEV;
	ret = uk_addr(uk, MESON_SMC_CMD_STORAGE_IN_BASE, &uk->in);
	if (!ret)
		ret = uk_addr(uk, MESON_SMC_CMD_STORAGE_OUT_BASE, &uk->out);
	if (!ret)
		ret = uk_addr(uk, MESON_SMC_CMD_STORAGE_BLOCK_BASE, &uk->block);

	return ret;
}

/*
 * 按名字的调用（STATUS/TELL）：输入 {namelen, name}，应答首个 u32 交给 @val。
 * BL31 返回 1 = 不存在（TELL 还包括数据为空），2 = 名字过长（调用前已挡掉）；协议之外的应答记为 -EPROTO
 */
static int uk_name_call(struct uk *uk, u32 cmd, const char *name, u32 *val)
{
	size_t n = strlen(name);
	u32 r;
	int ret;

	put_unaligned_le32(n, uk->in);
	memcpy(uk->in + 4, name, n);
	uk_flush(uk->in, 4 + n);
	ret = uk_call(uk, cmd, 0, &r);
	if (ret)
		return ret;
	uk_flush(uk->out, 4);
	if (r == 1)
		return -ENOENT;
	if (r)
		return -EPROTO;
	*val = get_unaligned_le32(uk->out);

	return 0;
}

static int uk_check_name(const char *name)
{
	size_t n = strlen(name);

	return n && n <= MESON_UNIFYKEY_NAME_MAX ? 0 : -EINVAL;
}

/* LIST：应答 {total, name0\0name1\0…}，total = Σ(namelen + 1)；BL31 不查输出长度，表最多 64 项、名字最长 79，远小于输出缓冲 */
static int uk_list(struct uk *uk, u32 *total)
{
	u32 r;
	int ret;

	ret = uk_call(uk, MESON_SMC_CMD_STORAGE_LIST, 0, &r);
	if (ret)
		return ret;
	if (r)
		return -EPROTO;
	uk_flush(uk->out, 4);
	*total = get_unaligned_le32(uk->out);
	if (*total > UK_DATA_MAX)
		return -EPROTO;
	uk_flush(uk->out, 4 + *total);

	return 0;
}

int meson_unifykey_list(char *buf, size_t size, size_t *len)
{
	struct uk uk;
	u32 total;
	int ret;

	ret = uk_open(&uk);
	if (!ret)
		ret = uk_list(&uk, &total);
	if (ret)
		return ret;
	if (total > size)
		return -E2BIG;
	memcpy(buf, uk.out + 4, total);
	*len = total;

	return 0;
}

int meson_unifykey_stat(const char *name, u32 *len, u32 *attr)
{
	struct uk uk;
	int ret;

	ret = uk_check_name(name);
	if (!ret)
		ret = uk_open(&uk);
	if (!ret)
		ret = uk_name_call(&uk, MESON_SMC_CMD_STORAGE_STATUS, name, attr);
	if (!ret)
		ret = uk_name_call(&uk, MESON_SMC_CMD_STORAGE_TELL, name, len);

	return ret;
}

int meson_unifykey_read(const char *name, void *buf, size_t size, u32 *len)
{
	size_t n = strlen(name);
	u32 attr, r, got;
	struct uk uk;
	int ret;

	ret = uk_check_name(name);
	if (!ret)
		ret = uk_open(&uk);
	if (!ret)
		ret = uk_name_call(&uk, MESON_SMC_CMD_STORAGE_STATUS, name, &attr);
	if (ret)
		return ret;
	/* BL31 对安全属性的 key 一律拒绝普通世界的 READ（返回 1，与不存在同码），先查属性才能说清原因 */
	if (attr & MESON_UNIFYKEY_ATTR_SECURE)
		return -EACCES;
	ret = uk_name_call(&uk, MESON_SMC_CMD_STORAGE_TELL, name, len);
	if (ret)
		return ret;
	if (*len > size || *len > UK_DATA_MAX)
		return -E2BIG;

	put_unaligned_le32(n, uk.in);
	put_unaligned_le32(*len, uk.in + 4);
	memcpy(uk.in + 8, name, n);
	uk_flush(uk.in, 8 + n);
	ret = uk_call(&uk, MESON_SMC_CMD_STORAGE_READ, 0, &r);
	if (ret)
		return ret;
	if (r == 1)
		return -ENOENT;
	if (r)
		return -EPROTO;
	uk_flush(uk.out, 4 + *len);
	got = get_unaligned_le32(uk.out);
	if (got != *len)
		return -EPROTO;
	memcpy(buf, uk.out + 4, got);

	return 0;
}

static u64 uk_sum(const u8 *p, size_t len)
{
	u64 sum = 0;
	size_t i;

	for (i = 0; i < len; i += 4)
		sum += get_unaligned_le32(p + i);

	return sum;
}

/* 选中副本的块头预检（BL31 装载 0x5120994 的前几步），只用于说清 BL31 拒绝的原因 */
static void uk_check_header(const u8 *blk, size_t size, struct meson_unifykey_load *res)
{
	u8 sha[SHA256_SUM_LEN];

	res->hdr_magic = !memcmp(blk, UK_HDR_MAGIC, sizeof(UK_HDR_MAGIC));
	if (!res->hdr_magic)
		return;
	res->hdr_version = get_unaligned_le32(blk + UK_HDR_VERSION);
	res->hdr_enctype = get_unaligned_le32(blk + UK_HDR_ENCTYPE);
	sha256_csum_wd(blk + UK_HDR_SIZE, size - UK_HDR_SIZE, sha, CHUNKSZ_SHA256);
	res->hdr_sha_ok = !memcmp(sha, blk + UK_HDR_SHA, sizeof(sha));
}

int meson_unifykey_load(struct blk_desc *desc, const struct disk_partition *part,
			struct meson_unifykey_load *res)
{
	const ulong off = CONFIG_MESON_UNIFYKEY_OFFSET;
	const ulong size = CONFIG_MESON_UNIFYKEY_SIZE;
	ulong blksz = part->blksz, info_len, nblk;
	u8 *copy[2] = { }, *info = NULL;
	struct uk uk;
	u32 r;
	int i, ret;

	memset(res, 0, sizeof(*res));
	ret = uk_open(&uk);
	if (ret)
		return ret;

	/* 两块 info 紧接在第二份副本之后，按块读出覆盖它们的整块 */
	info_len = ALIGN(2 * UK_INFO_SIZE, blksz);
	nblk = size / blksz;
	if (size > UK_SHM_SIZE || size <= UK_HDR_SIZE || off % blksz ||
	    size % blksz || off + 2 * size + info_len > (u64)part->size * blksz)
		return -ERANGE;

	copy[0] = malloc_cache_aligned(size);
	copy[1] = malloc_cache_aligned(size);
	info = malloc_cache_aligned(info_len);
	if (!copy[0] || !copy[1] || !info) {
		ret = -ENOMEM;
		goto out;
	}
	for (i = 0; i < 2; i++) {
		if (blk_dread(desc, part->start + (off + i * size) / blksz, nblk,
			      copy[i]) != nblk) {
			ret = -EIO;
			goto out;
		}
	}
	if (blk_dread(desc, part->start + (off + 2 * size) / blksz,
		      info_len / blksz, info) != info_len / blksz) {
		ret = -EIO;
		goto out;
	}

	/* info = {u64 checksum; u32 stamp; u32 magic}，在每块 512 B 的开头 */
	for (i = 0; i < 2; i++) {
		struct meson_unifykey_copy *c = &res->copy[i];
		const u8 *p = info + i * UK_INFO_SIZE;

		c->checksum = get_unaligned_le64(p);
		c->stamp = get_unaligned_le32(p + 8);
		c->magic = get_unaligned_le32(p + 12);
		c->sum = uk_sum(copy[i], size);
		c->valid = c->magic && c->sum == c->checksum;
	}
	res->same = !memcmp(copy[0], copy[1], size) &&
		    !memcmp(info, info + UK_INFO_SIZE, 16);
	/*
	 * 与厂商 mmc_key_read 相同：两份都有效时 (s32)(stamp0 - stamp1) < 0 取副本 1、否则取 0（厂商 0x10752dc 的 sub + tbz #31；差恰为 0x80000000 时取 1）；只有一份有效取它；都无效取 0
	 */
	if (res->copy[0].valid && res->copy[1].valid)
		res->pick = (s32)(res->copy[0].stamp - res->copy[1].stamp) < 0;
	else
		res->pick = res->copy[1].valid && !res->copy[0].valid;
	uk_check_header(copy[res->pick], size, res);

	memcpy(uk.block, copy[res->pick], size);
	uk_flush(uk.block, size);
	/* NOTIFY_EX 恒返回 0（x1 超过 0x40000 时只打印、不装载），x2 BL31 不读 */
	ret = uk_call(&uk, MESON_SMC_CMD_STORAGE_NOTIFY_EX, size, &r);
	if (!ret)
		ret = uk_call(&uk, MESON_SMC_CMD_STORAGE_BLOCK_SIZE, 0, &res->block_size);
	if (!ret)
		ret = uk_call(&uk, MESON_SMC_CMD_STORAGE_VERSION, 0, &r);
	if (ret)
		goto out;
	res->version = r;
	ret = uk_list(&uk, &r);
	if (ret)
		goto out;
	for (i = 0; i < r; i++)
		res->keys += !uk.out[4 + i];
	/*
	 * 内层头声明的块大小与交给它的不同时，BL31 改记该大小并静默清表；魔数没对上、SHA-256 不符、解密或记录解析失败同样只是表空。VERSION 只区分出“魔数没对上”（-1），不能作为成功判据：块头版本可以是 0（此时 BL31 不验 SHA-256）
	 */
	if (res->block_size != size || !res->keys)
		ret = -ENODATA;
out:
	free(info);
	free(copy[1]);
	free(copy[0]);

	return ret;
}
