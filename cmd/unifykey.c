// SPDX-License-Identifier: GPL-2.0+
/*
 * unifykey：经 Amlogic 厂商 BL31 的安全存储读出厂写入的 key（mac、deviceid、authkey 等），只读
 *
 * load 把厂商 reserved 分区上的加密块按厂商规则选出一份交给 BL31 装载，只读盘；list、read 按名字查询；read … str 与厂商 keyman read … str 同一规则（任一字节 ≥ 0x80 即拒绝）设同名环境变量，供 env 脚本拼 androidboot.* 参数；ethaddr 把 MAC 类 key 设为 ethaddr，由网卡驱动与 bootm 的 FDT 修补（/aliases 有 ethernetN 时）交给内核。
 */

#include <command.h>
#include <env.h>
#include <env_internal.h>
#include <hexdump.h>
#include <malloc.h>
#include <mapmem.h>
#include <net.h>
#include <part.h>
#include <search.h>
#include <vsprintf.h>
#include <linux/ctype.h>
#include <meson/unifykey.h>

/* 名字列表的上限：BL31 的表 64 项、名字最长 79 */
#define UK_LIST_MAX	(64 * (MESON_UNIFYKEY_NAME_MAX + 1))
/* MAC 类 key 的文本形式 xx:xx:xx:xx:xx:xx（本机 mac 就是这样存的 17 字节） */
#define UK_MAC_TEXT	17

static int uk_error(const char *name, int ret)
{
	switch (ret) {
	case -ENODEV:
		printf("unifykey: no secure monitor device\n");
		break;
	case -ENOSYS:
		printf("unifykey: the secure monitor has no Amlogic secure storage calls (not the vendor BL31?)\n");
		break;
	case -ENOENT:
		printf("unifykey: %s: no such key\n", name);
		break;
	case -EACCES:
		printf("unifykey: %s: secure key, not readable from the normal world\n", name);
		break;
	case -EINVAL:
		printf("unifykey: %s: name must be 1..%d bytes\n", name, MESON_UNIFYKEY_NAME_MAX);
		break;
	case -EPROTO:
		printf("unifykey: %s: unexpected answer from the secure monitor\n", name);
		break;
	default:
		printf("unifykey: %s: error %d\n", name, ret);
		break;
	}

	return CMD_RET_FAILURE;
}

static void uk_print_copy(int i, const struct meson_unifykey_copy *c)
{
	printf("unifykey: copy %d: stamp 0x%x, magic %u, checksum 0x%llx, ", i,
	       c->stamp, c->magic, c->checksum);
	if (c->valid)
		printf("ok\n");
	else if (!c->magic)
		printf("bad (magic 0)\n");
	else
		printf("bad (computed 0x%llx)\n", c->sum);
}

static int do_unifykey_load(struct cmd_tbl *cmdtp, int flag, int argc,
			    char *const argv[])
{
	const struct meson_unifykey_copy *c0, *c1;
	struct meson_unifykey_load res;
	struct disk_partition part;
	struct blk_desc *desc;
	const char *sep;
	int ret;

	if (argc != 3)
		return CMD_RET_USAGE;
	/* 布局相对分区起点；不带分区、`<dev>:` 与 `<dev>:auto` 都会让 part 层取第 1 个或第一个可启动分区（厂商 eMMC 的第 1 个是 bootloader），只认写明的分区名或分区号 */
	sep = strpbrk(argv[2], ":#");
	if (!sep || !sep[1] || !strcmp(sep + 1, "auto")) {
		printf("unifykey: name the partition, e.g. mmc 0#reserved\n");
		return CMD_RET_USAGE;
	}
	if (part_get_info_by_dev_and_name_or_num(argv[1], argv[2], &desc, &part, 0) < 0)
		return CMD_RET_FAILURE;

	ret = meson_unifykey_load(desc, &part, &res);
	switch (ret) {
	case 0:
	case -ENODATA:
		break;
	case -ERANGE:
		printf("unifykey: the partition cannot hold the key area at 0x%x (2 copies of 0x%x and their info) in whole blocks\n",
		       CONFIG_MESON_UNIFYKEY_OFFSET, CONFIG_MESON_UNIFYKEY_SIZE);
		return CMD_RET_FAILURE;
	case -EIO:
		printf("unifykey: read error on %s %s\n", argv[1], argv[2]);
		return CMD_RET_FAILURE;
	default:
		return uk_error(argv[2], ret);
	}

	c0 = &res.copy[0];
	c1 = &res.copy[1];
	uk_print_copy(0, c0);
	uk_print_copy(1, c1);
	if (!res.same)
		printf("unifykey: the two copies differ (left as they are: nothing is written)\n");
	if (!c0->valid && !c1->valid)
		printf("unifykey: no valid copy, handing copy 0 to BL31 as the vendor u-boot does\n");
	printf("unifykey: loading copy %d\n", res.pick);
	if (!res.hdr_magic)
		printf("unifykey: block header: bad magic\n");
	else
		printf("unifykey: block header: version %d, encryption type %u, SHA-256 %s\n",
		       res.hdr_version, res.hdr_enctype,
		       res.hdr_version <= 0 ? "not checked by BL31 (version <= 0)" :
		       res.hdr_sha_ok ? "ok" : "mismatch");
	if (!ret) {
		printf("unifykey: BL31 loaded %u keys\n", res.keys);
		return CMD_RET_SUCCESS;
	}

	if (res.block_size != CONFIG_MESON_UNIFYKEY_SIZE)
		printf("unifykey: BL31 block size is 0x%x after loading, not 0x%x: the inner header declares another size and BL31 dropped every key\n",
		       res.block_size, CONFIG_MESON_UNIFYKEY_SIZE);
	else if (res.version == -1)
		printf("unifykey: BL31 did not recognise the block (storage version -1)\n");
	else if (res.hdr_version > 0 && !res.hdr_sha_ok)
		printf("unifykey: BL31 rejected the block: SHA-256 mismatch\n");
	else
		printf("unifykey: BL31 has no keys after loading (decryption with this chip's key or the key records failed)\n");

	return CMD_RET_FAILURE;
}

static int do_unifykey_list(struct cmd_tbl *cmdtp, int flag, int argc,
			    char *const argv[])
{
	u32 len, attr;
	size_t total;
	char *names, *p;
	int ret, n = 0;

	names = malloc(UK_LIST_MAX);
	if (!names)
		return CMD_RET_FAILURE;
	ret = meson_unifykey_list(names, UK_LIST_MAX, &total);
	if (ret) {
		free(names);
		return uk_error("list", ret);
	}
	for (p = names; p < names + total; p += strlen(p) + 1, n++) {
		ret = meson_unifykey_stat(p, &len, &attr);
		if (ret) {
			printf("%-24s  (error %d)\n", p, ret);
			continue;
		}
		printf("%-24s %6u%s%s\n", p, len,
		       attr & MESON_UNIFYKEY_ATTR_SECURE ? "  secure" : "",
		       attr & MESON_UNIFYKEY_ATTR_ENCRYPT ? "  encrypted" : "");
	}
	free(names);
	printf("unifykey: %d keys\n", n);

	return CMD_RET_SUCCESS;
}

/* 读出整个 key 到新分配的缓冲（多留 1 字节给 str 的结尾 NUL） */
static int uk_fetch(const char *name, u8 **bufp, u32 *len)
{
	int ret;

	ret = meson_unifykey_read(name, NULL, 0, len);
	if (ret != -E2BIG)
		return ret ? ret : -EIO;
	*bufp = malloc(*len + 1);
	if (!*bufp)
		return -ENOMEM;
	ret = meson_unifykey_read(name, *bufp, *len, len);
	if (ret) {
		free(*bufp);
		*bufp = NULL;
	}

	return ret;
}

static int do_unifykey_read(struct cmd_tbl *cmdtp, int flag, int argc,
			    char *const argv[])
{
	const char *name;
	bool str = false;
	char *end;
	ulong addr;
	u8 *buf;
	void *dst;
	u32 len, i;
	int ret;

	if (argc < 3 || argc > 4)
		return CMD_RET_USAGE;
	if (argc == 4) {
		if (strcmp(argv[3], "str"))
			return CMD_RET_USAGE;
		str = true;
	}
	name = argv[1];
	/* 地址按十六进制严格解析：脚本里地址变量为空时 str 会顶到地址的位置，hextoul 把它当成 0 */
	addr = hextoul(argv[2], &end);
	if (end == argv[2] || *end) {
		printf("unifykey: %s is not an address\n", argv[2]);
		return CMD_RET_USAGE;
	}

	ret = uk_fetch(name, &buf, &len);
	if (ret)
		return uk_error(name, ret);
	/* str：先校验再落内存与环境，拒绝时不留任何副作用（厂商是先拷到 addr 再校验） */
	if (str) {
		for (i = 0; i < len; i++) {
			if (buf[i] >= 0x80) {
				printf("unifykey: %s: byte 0x%02x at offset %u is not ASCII, not set\n",
				       name, buf[i], i);
				free(buf);
				return CMD_RET_FAILURE;
			}
		}
		buf[len] = '\0';
	}
	dst = map_sysmem(addr, len);
	memcpy(dst, buf, len);
	unmap_sysmem(dst);
	env_set_hex("filesize", len);
	/* 与厂商相同：数据里有 NUL 时变量值截在那里 */
	if (str && env_set(name, (char *)buf)) {
		printf("unifykey: %s: cannot set the environment variable\n", name);
		ret = CMD_RET_FAILURE;
	}
	free(buf);

	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

/* xx:xx:xx:xx:xx:xx，大小写都认 */
static bool uk_parse_mac(const u8 *s, u8 mac[ARP_HLEN])
{
	int i;

	for (i = 0; i < ARP_HLEN; i++) {
		if (!isxdigit(s[3 * i]) || !isxdigit(s[3 * i + 1]) ||
		    (i < ARP_HLEN - 1 && s[3 * i + 2] != ':'))
			return false;
		mac[i] = hex_to_bin(s[3 * i]) << 4 | hex_to_bin(s[3 * i + 1]);
	}

	return true;
}

static int do_unifykey_ethaddr(struct cmd_tbl *cmdtp, int flag, int argc,
			       char *const argv[])
{
	const char *name = argc > 1 ? argv[1] : "mac";
	char text[UK_MAC_TEXT + 1];
	u8 buf[UK_MAC_TEXT], mac[ARP_HLEN];
	char *set[] = { "setenv", "-f", "ethaddr", text, NULL };
	u32 len;
	int ret;

	ret = meson_unifykey_read(name, buf, sizeof(buf), &len);
	if (ret == -E2BIG || (!ret && len != ARP_HLEN && len != UK_MAC_TEXT)) {
		printf("unifykey: %s: %u bytes, not a MAC address\n", name, len);
		return CMD_RET_FAILURE;
	}
	if (ret)
		return uk_error(name, ret);
	if (len == ARP_HLEN) {
		memcpy(mac, buf, ARP_HLEN);
	} else if (!uk_parse_mac(buf, mac)) {
		printf("unifykey: %s: not a MAC address of the form xx:xx:xx:xx:xx:xx\n", name);
		return CMD_RET_FAILURE;
	}
	if (!is_valid_ethaddr(mac)) {
		printf("unifykey: %s: %pM is not a valid unicast address\n", name, mac);
		return CMD_RET_FAILURE;
	}
	snprintf(text, sizeof(text), "%pM", mac);
	/*
	 * ethaddr 默认只许写一次（ENV_OVERWRITE 关时 env_flags 记为 mo），meson 板的 misc_init_r 已经按芯片序列号设了一个本地管理地址；出厂 MAC 要取代它，所以强制写（H_FORCE，等同 env set -f）
	 */
	if (env_do_env_set(0, 4, set, H_PROGRAMMATIC)) {
		printf("unifykey: cannot set ethaddr\n");
		return CMD_RET_FAILURE;
	}
	printf("unifykey: ethaddr %s (from key %s)\n", text, name);

	return CMD_RET_SUCCESS;
}

U_BOOT_LONGHELP(unifykey,
	"load <interface> <dev[:part|#partname]> - hand the key block in this partition (vendor eMMC: mmc 0#reserved) to BL31; only reads the disk\n"
	"unifykey list - list the keys BL31 holds, with their lengths\n"
	"unifykey read <name> <addr> [str] - copy key <name> to <addr> and set filesize; str: also set env <name> to it (every byte must be below 0x80)\n"
	"unifykey ethaddr [<name>] - set ethaddr from MAC key <name> (default mac; 17 characters xx:xx:xx:xx:xx:xx or 6 bytes)");

U_BOOT_CMD_WITH_SUBCMDS(unifykey, "Amlogic unifykey (secure storage) through the vendor BL31",
	unifykey_help_text,
	U_BOOT_SUBCMD_MKENT(load, 3, 0, do_unifykey_load),
	U_BOOT_SUBCMD_MKENT(list, 1, 0, do_unifykey_list),
	U_BOOT_SUBCMD_MKENT(read, 4, 0, do_unifykey_read),
	U_BOOT_SUBCMD_MKENT(ethaddr, 2, 0, do_unifykey_ethaddr));
