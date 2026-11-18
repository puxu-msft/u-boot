// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic 厂商 BL31 安全存储的 sandbox 模型
 *
 * 按 M12（SM1）厂商 BL31 v1.3:079690f10 的反汇编（基址 0x05100000）实现 include/meson/sm.h 里存储索引的语义，供 meson-unifykey 客户端与 unifykey 命令在 sandbox 里测试；派生密钥用的 eFuse 字节缺省是 include/sandbox-meson-sm.h 的测试值，[4,16) 可经命令行 --meson_sm_efuse_4_16 或 sandbox_meson_sm_set_efuse_4_16() 换成真机的值，用来装载真机备份里的块。
 * 装载（NOTIFY_EX，0x5121480 → 0x5120994）：块大小超过 0x40000 只打印、不动状态；否则清表、复位堆，块类型与 VERSION 置 -1；魔数（strcmp "AMLSECURITY"）不对 → 块大小 := 给的大小，返回；块头版本 > 0 时 sha256(块[0x200, 大小)) 与块头 +0x18 比，不等同样返回；AES-256-CBC 解密 [0x200, 0x400) 得内层头（tag 1 外壳内 tag 2 = total、tag 0xb = 声明的块大小），解析失败或 total 为 0 → 块大小 := 给的大小，返回；声明的块大小非 0 时块大小 := 它，与给的不同 → 静默返回；再从同一 IV 重新解密 TLV 区 [0x400, 0x400 + pad16(total))，逐个对象解析（0x51215f4：tag 4/6/8/9 的 u32 值固定前进 12 B，tag 5 按 namelen 拷名字，tag 7 按 datalen 从堆分配，tag 0xa 长度必须是 0x20，其余按长度跳过），任一失败整表作废；对象数据的 sha256 与记录相符才计入（状态 1），否则状态 2、该槽给下一个对象用（与真 BL31 一样不先清空，下一个对象没写到的字段沿用上一个的；name 在 tag 5 处先清 80 字节）。
 * 查询：TELL/STATUS 的名字在输入 +4、READ 在 +8；名字超过 79 返回 2；READ 对安全属性的 key 返回 1（调用者固定视为非安全世界）；LIST 遍历全部 64 个槽。
 * 真 BL31 在越界的记录上会读写越界（例如 namelen 大于 80、对象长度超过剩余字节）；模型在这些地方判失败、整表作废，不模拟越界。
 */

#include <dm.h>
#include <hexdump.h>
#include <malloc.h>
#include <mapmem.h>
#include <sandbox-meson-sm.h>
#include <sm-uclass.h>
#include <uboot_aes.h>
#include <asm/cache.h>
#include <asm/getopt.h>
#include <asm/ptrace.h>
#include <asm/unaligned.h>
#include <linux/sizes.h>
#include <meson/sm.h>
#include <u-boot/sha256.h>

#define SB_SHM_SIZE		SZ_256K
#define SB_HEAP_SIZE		SZ_256K
#define SB_KEYS			64
#define SB_NAME_MAX		79
#define SB_NOTIFY_MAX		0x40000
#define SB_SMC_UNK		0xffffffffU
#define SB_HDR_SIZE		0x200
#define SB_TLV_OFF		0x400

struct sb_key {
	u8 name[80];
	u32 namelen;
	u32 attr;
	u32 type;
	u32 datalen;
	u8 *data;
	u8 sha[SHA256_SUM_LEN];
	u32 status;		/* bit0 有效；2 = 数据哈希不符 */
};

struct sandbox_meson_sm_priv {
	u8 *in;
	u8 *out;
	u8 *block;
	u8 *heap;
	u32 heap_used;
	u32 block_size;		/* 0x512d310，冷启动 0x40000 */
	s32 version;		/* 0x513db50，冷启动 0 */
	s32 enctype;		/* 0x512d318：最近一次装载的块类型 */
	u32 count;
	struct sb_key keys[SB_KEYS];
	u8 plain[SB_SHM_SIZE];	/* 解密出的内层头与 TLV 区 */
	bool no_storage;
	uint unknown_calls;
};

/* eFuse 字节 [4,16)：与芯片一样全进程一份，装载时取当前值 */
static u8 sb_efuse_4_16[SANDBOX_MESON_SM_EFUSE_4_16_LEN] __nonstring = SANDBOX_MESON_SM_EFUSE_4_16;

const u8 *sandbox_meson_sm_efuse_4_16(void)
{
	return sb_efuse_4_16;
}

void sandbox_meson_sm_set_efuse_4_16(const u8 efuse[SANDBOX_MESON_SM_EFUSE_4_16_LEN])
{
	memcpy(sb_efuse_4_16, efuse, sizeof(sb_efuse_4_16));
}

static int sandbox_cmdline_cb_meson_sm_efuse_4_16(struct sandbox_state *state,
						  const char *arg)
{
	u8 e[SANDBOX_MESON_SM_EFUSE_4_16_LEN];

	if (strlen(arg) != 2 * sizeof(e) || hex2bin(e, arg, sizeof(e)))
		return -EINVAL;
	sandbox_meson_sm_set_efuse_4_16(e);

	return 0;
}
SANDBOX_CMDLINE_OPT(meson_sm_efuse_4_16, 1,
		    "Amlogic BL31 secure storage model: eFuse bytes [4,16) as 24 hex digits (default: test value)");

void sandbox_meson_sm_key(u32 enctype, u8 key[32], u8 iv[16])
{
	/* 默认类型的密钥与 IV 尾部（0x512d2e0、0x512d308），类型 1 的（0x512c000、0x512c018），类型 2 的固定密钥与 IV（0x512d2c0、0x512d2f8） */
	static const u8 k0[20] = {
		0xad, 0x93, 0x00, 0xc4, 0x2f, 0x7d, 0x49, 0xd9, 0x15, 0x8b,
		0x7f, 0x04, 0x2c, 0x80, 0xb0, 0x62, 0x78, 0x25, 0x8d, 0x9c,
	};
	static const u8 i0[4] = { 0x78, 0x25, 0x8d, 0x9c };
	static const u8 k1[12] = {
		0xad, 0x93, 0x00, 0xc4, 0x8e, 0x50, 0x20, 0xc5, 0x3f, 0xbf, 0x23, 0x32,
	};
	static const u8 i1[4] = { 0x80, 0x5a, 0xc6, 0xdf };
	static const u8 k2[32] = {
		0x13, 0x22, 0x02, 0x4a, 0x55, 0x23, 0xbb, 0xcb, 0xf1, 0xfb, 0x2a,
		0xcc, 0xbb, 0x95, 0xf4, 0x50, 0xae, 0x08, 0xd7, 0xfb, 0x80, 0xf2,
		0x64, 0x72, 0xe3, 0x3c, 0xc4, 0xb4, 0xa3, 0x50, 0xd9, 0xf1,
	};
	static const u8 i2[16] = {
		0x2a, 0xde, 0xfc, 0xd7, 0x67, 0xc8, 0xde, 0xd0,
		0xf0, 0x1e, 0xe8, 0x12, 0xf9, 0x57, 0x25, 0x36,
	};
	const u8 *e = sb_efuse_4_16;

	switch (enctype) {
	case 1:
		memcpy(key, SANDBOX_MESON_SM_EFUSE_40_48, 8);
		memcpy(key + 8, e, 12);
		memcpy(key + 20, k1, sizeof(k1));
		memcpy(iv, e, 12);
		memcpy(iv + 12, i1, sizeof(i1));
		break;
	case 2:
		memcpy(key, k2, sizeof(k2));
		memcpy(iv, i2, sizeof(i2));
		break;
	default:	/* 0 以及 1、2 以外的任何值 */
		memcpy(key, e, 12);
		memcpy(key + 12, k0, sizeof(k0));
		memcpy(iv, e, 12);
		memcpy(iv + 12, i0, sizeof(i0));
		break;
	}
}

/* 0x5120718：每次调用都从 IV 重新起一条 CBC 链 */
static void sb_decrypt(struct sandbox_meson_sm_priv *priv, const u8 *src, u8 *dst, u32 len)
{
	u8 key[AES256_KEY_LENGTH], iv[AES_BLOCK_LENGTH];
	u8 exp[AES256_EXPAND_KEY_LENGTH];

	sandbox_meson_sm_key(priv->enctype, key, iv);
	aes_expand_key(key, AES256_KEY_LENGTH, exp);
	aes_cbc_decrypt_blocks(AES256_KEY_LENGTH, exp, iv, (u8 *)src, dst,
			       len / AES_BLOCK_LENGTH);
}

static void sb_reset(struct sandbox_meson_sm_priv *priv)
{
	priv->heap_used = 0;
	priv->count = 0;
	memset(priv->keys, '\0', sizeof(priv->keys));
}

/* 0x51214dc：堆上 4 字节对齐的递增分配，0 字节与用尽都返回 NULL */
static u8 *sb_alloc(struct sandbox_meson_sm_priv *priv, u32 len)
{
	u32 n = ALIGN(len, 4);
	u8 *p;

	if (!len || n > SB_HEAP_SIZE - priv->heap_used)
		return NULL;
	p = priv->heap + priv->heap_used;
	priv->heap_used += n;

	return p;
}

/* 0x5121590：内层头 {1, len, {2, 4, total}{0xb, 4, size}…}；整段必须恰好落在 0x200 字节里 */
static int sb_inner_header(const u8 *p, u32 *total, u32 *size)
{
	u32 left, len;
	const u8 *q;

	if (get_unaligned_le32(p) != 1)
		return -1;
	left = get_unaligned_le32(p + 4);
	if (left > SB_HDR_SIZE - 8)
		return -1;
	for (q = p + 8; left; q += 8 + len, left -= 8 + len) {
		if (left < 8)
			return -1;
		len = get_unaligned_le32(q + 4);
		if (len > left - 8)
			return -1;
		if (get_unaligned_le32(q) == 2)
			*total = get_unaligned_le32(q + 8);
		else if (get_unaligned_le32(q) == 0xb)
			*size = get_unaligned_le32(q + 8);
	}

	return 0;
}

/* 0x51215f4：解析一个对象，@used 为它占的字节（外层 8 + len） */
static int sb_object(struct sandbox_meson_sm_priv *priv, const u8 *p, const u8 *end,
		     struct sb_key *k, u32 *used)
{
	const u8 *q, *v;
	u32 left, tag, len, adv;

	if (end - p < 8 || get_unaligned_le32(p) != 3)
		return -1;
	left = get_unaligned_le32(p + 4);
	if (left > end - p - 8)
		return -1;
	*used = left + 8;
	for (q = p + 8; left; q += adv, left -= adv) {
		if (left < 8)
			return -1;
		tag = get_unaligned_le32(q);
		len = get_unaligned_le32(q + 4);
		v = q + 8;
		switch (tag) {
		case 4:
		case 6:
		case 8:
		case 9:
			adv = 12;
			if (adv > left)
				return -1;
			if (tag == 4)
				k->namelen = get_unaligned_le32(v);
			else if (tag == 6)
				k->datalen = get_unaligned_le32(v);
			else if (tag == 8)
				k->type = get_unaligned_le32(v);
			else
				k->attr = get_unaligned_le32(v);
			break;
		case 5:
			adv = 8 + len;
			if (len > left - 8 || k->namelen > sizeof(k->name) ||
			    k->namelen > end - v)
				return -1;
			memset(k->name, '\0', sizeof(k->name));
			memcpy(k->name, v, k->namelen);
			break;
		case 7:
			adv = 8 + len;
			if (len > left - 8 || k->datalen > end - v)
				return -1;
			k->data = sb_alloc(priv, k->datalen);
			if (!k->data)
				return -1;
			memcpy(k->data, v, k->datalen);
			break;
		case 0xa:
			adv = 8 + SHA256_SUM_LEN;
			if (len != SHA256_SUM_LEN || adv > left)
				return -1;
			memcpy(k->sha, v, SHA256_SUM_LEN);
			break;
		default:
			adv = 8 + len;
			if (len > left - 8)
				return -1;
			break;
		}
	}

	return 0;
}

static void sb_load(struct sandbox_meson_sm_priv *priv, u32 size)
{
	u32 total = 0, declared = 0, left, used, pad;
	u8 sha[SHA256_SUM_LEN];
	const u8 *p, *end;

	sb_reset(priv);
	priv->enctype = -1;
	priv->version = -1;
	if (memcmp(priv->block, "AMLSECURITY", sizeof("AMLSECURITY"))) {
		priv->block_size = size;
		return;
	}
	priv->enctype = get_unaligned_le32(priv->block + 0x14);
	priv->version = get_unaligned_le32(priv->block + 0x10);
	if (size <= SB_HDR_SIZE) {	/* 真 BL31 会按 size - 0x200 回绕后的长度去算哈希 */
		priv->block_size = size;
		return;
	}
	if (priv->version > 0) {
		sha256_csum_wd(priv->block + SB_HDR_SIZE, size - SB_HDR_SIZE, sha,
			       CHUNKSZ_SHA256);
		if (memcmp(sha, priv->block + 0x18, sizeof(sha))) {
			printf("ERROR:   !!!!!storage data is corrupt!\n");
			priv->block_size = size;
			return;
		}
	}
	sb_decrypt(priv, priv->block + SB_HDR_SIZE, priv->plain, SB_HDR_SIZE);
	if (sb_inner_header(priv->plain, &total, &declared) || !total) {
		priv->block_size = size;
		return;
	}
	if (declared)
		priv->block_size = declared;
	if (priv->block_size != size)
		return;
	pad = ALIGN(total, AES_BLOCK_LENGTH);
	if (pad < total || pad > SB_SHM_SIZE - SB_TLV_OFF)
		goto bad;
	sb_decrypt(priv, priv->block + SB_TLV_OFF, priv->plain, pad);
	end = priv->plain + pad;
	for (p = priv->plain, left = total; left; p += used, left -= used) {
		struct sb_key *k;

		if (priv->count == SB_KEYS)
			goto bad;
		k = &priv->keys[priv->count];
		if (sb_object(priv, p, end, k, &used) || used > left)
			goto bad;
		sha256_csum_wd(k->data, k->datalen, sha, CHUNKSZ_SHA256);
		if (memcmp(sha, k->sha, sizeof(sha))) {
			k->status = 2;
			continue;
		}
		k->status = 1;
		priv->count++;
	}

	return;
bad:
	sb_reset(priv);
}

/* 0x5120878：在 0..count 个槽里找有效、等长、同名的 */
static struct sb_key *sb_find(struct sandbox_meson_sm_priv *priv, const u8 *name, u32 n)
{
	u32 i;

	for (i = 0; i <= priv->count && i < SB_KEYS; i++) {
		struct sb_key *k = &priv->keys[i];

		if ((k->status & 1) && k->namelen == n && !memcmp(k->name, name, n))
			return k;
	}

	return NULL;
}

static u32 sb_by_name(struct sandbox_meson_sm_priv *priv, u32 cmd)
{
	u32 n = get_unaligned_le32(priv->in);
	const u8 *name = priv->in + (cmd == MESON_SMC_CMD_STORAGE_READ ? 8 : 4);
	struct sb_key *k;

	if (n > SB_NAME_MAX)
		return 2;
	k = sb_find(priv, name, n);
	switch (cmd) {
	case MESON_SMC_CMD_STORAGE_STATUS:
		if (!k)
			return 1;
		put_unaligned_le32(k->attr & 0x101, priv->out);
		return 0;
	case MESON_SMC_CMD_STORAGE_TELL:
		if (!k || !k->data || !k->datalen)
			return 1;
		put_unaligned_le32(k->datalen, priv->out);
		return 0;
	default: {	/* READ */
		u32 buflen = get_unaligned_le32(priv->in + 4), len;

		if (!k || (k->attr & 1) || !k->data || !k->datalen)
			return 1;
		len = min(k->datalen, buflen);
		put_unaligned_le32(len, priv->out);
		memcpy(priv->out + 4, k->data, len);
		return 0;
	}
	}
}

/* 0x5120e2c：遍历全部 64 个槽，输出 {total, name\0…} */
static void sb_list(struct sandbox_meson_sm_priv *priv)
{
	u8 *p = priv->out + 4;
	u32 total = 0, i;

	for (i = 0; i < SB_KEYS; i++) {
		struct sb_key *k = &priv->keys[i];

		if (!(k->status & 1) || !k->data)
			continue;
		memcpy(p, k->name, k->namelen);
		p[k->namelen] = '\0';
		p += k->namelen + 1;
		total += k->namelen + 1;
	}
	put_unaligned_le32(total, priv->out);
}

static int sandbox_meson_sm_call(struct udevice *dev, u32 cmd, s32 *smc_ret,
				 struct pt_regs *args)
{
	struct sandbox_meson_sm_priv *priv = dev_get_priv(dev);
	u32 ret = 0;

	if (priv->no_storage && cmd >= MESON_SMC_CMD_STORAGE_IN_BASE &&
	    cmd < MESON_SMC_CMD_COUNT) {
		ret = SB_SMC_UNK;
		goto out;
	}
	switch (cmd) {
	case MESON_SMC_CMD_STORAGE_IN_BASE:
		ret = map_to_sysmem(priv->in);
		break;
	case MESON_SMC_CMD_STORAGE_OUT_BASE:
		ret = map_to_sysmem(priv->out);
		break;
	case MESON_SMC_CMD_STORAGE_BLOCK_BASE:
		ret = map_to_sysmem(priv->block);
		break;
	case MESON_SMC_CMD_STORAGE_BLOCK_SIZE:
		ret = priv->block_size;
		break;
	case MESON_SMC_CMD_STORAGE_NOTIFY_EX:
		if ((u32)args->regs[1] > SB_NOTIFY_MAX)
			printf("ERROR:   flash size is too large!\n");
		else
			sb_load(priv, args->regs[1]);
		break;
	case MESON_SMC_CMD_STORAGE_READ:
	case MESON_SMC_CMD_STORAGE_TELL:
	case MESON_SMC_CMD_STORAGE_STATUS:
		ret = sb_by_name(priv, cmd);
		break;
	case MESON_SMC_CMD_STORAGE_LIST:
		sb_list(priv);
		break;
	case MESON_SMC_CMD_STORAGE_VERSION:
		ret = priv->version;
		break;
	default:
		/* 写、删、eFuse 等：模型不实现，记次数 */
		priv->unknown_calls++;
		ret = SB_SMC_UNK;
		break;
	}
out:
	if (smc_ret)
		*smc_ret = ret;

	return 0;
}

uint sandbox_meson_sm_unknown_calls(struct udevice *dev)
{
	struct sandbox_meson_sm_priv *priv = dev_get_priv(dev);

	return priv->unknown_calls;
}

void sandbox_meson_sm_set_storage(struct udevice *dev, bool present)
{
	struct sandbox_meson_sm_priv *priv = dev_get_priv(dev);

	priv->no_storage = !present;
}

static int sandbox_meson_sm_probe(struct udevice *dev)
{
	struct sandbox_meson_sm_priv *priv = dev_get_priv(dev);

	priv->in = memalign(ARCH_DMA_MINALIGN, SB_SHM_SIZE);
	priv->out = memalign(ARCH_DMA_MINALIGN, SB_SHM_SIZE);
	priv->block = memalign(ARCH_DMA_MINALIGN, SB_SHM_SIZE);
	priv->heap = malloc(SB_HEAP_SIZE);
	if (!priv->in || !priv->out || !priv->block || !priv->heap)
		return -ENOMEM;
	memset(priv->block, '\0', SB_SHM_SIZE);
	priv->block_size = SB_NOTIFY_MAX;

	return 0;
}

static int sandbox_meson_sm_remove(struct udevice *dev)
{
	struct sandbox_meson_sm_priv *priv = dev_get_priv(dev);

	free(priv->heap);
	free(priv->block);
	free(priv->out);
	free(priv->in);

	return 0;
}

static const struct udevice_id sandbox_meson_sm_ids[] = {
	{ .compatible = "sandbox,meson-sm" },
	{ }
};

static const struct sm_ops sandbox_meson_sm_ops = {
	.sm_call = sandbox_meson_sm_call,
};

U_BOOT_DRIVER(sandbox_meson_sm) = {
	.name = "sandbox_meson_sm",
	.id = UCLASS_SM,
	.of_match = sandbox_meson_sm_ids,
	.probe = sandbox_meson_sm_probe,
	.remove = sandbox_meson_sm_remove,
	.priv_auto = sizeof(struct sandbox_meson_sm_priv),
	.ops = &sandbox_meson_sm_ops,
};
