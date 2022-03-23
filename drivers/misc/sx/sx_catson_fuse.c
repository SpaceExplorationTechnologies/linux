/**
 * sx_catson_fuse.c
 *
 * This driver interacts with ATF to modify and manipulate fuses.
 *
 * @author Kevin Bosien <kbosien@spacex.com>
 */

#include <linux/arm-smccc.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/stat.h>

/**
 * This is our module name.
 */
#define SX_CATSON_FUSE_NAME "sx_catson_fuse"

#define CATSON_SIP_SVC_RESET			0xC2000010
#define    CATSON_RESET_MAGIC_STRING	"reboot"
#define CATSON_SIP_SVC_GET_SOFT_RESET_COUNT	0xC2000012

#define CATSON_SIP_SVC_GET_ROM_ROTPK		0xC2000020
#define CATSON_SIP_SVC_GET_SKEY			0xC2000021
#define CATSON_SIP_SVC_GET_SKEY_IDX		0xC2000022
#define CATSON_SIP_SVC_GET_FUSE_MASK		0xC2000024
#define CATSON_SIP_SVC_GET_BOARD_TYPE		0xC2000027
#define CATSON_SIP_SVC_GET_BFIP_CERT_CTR	0xC2000030
#define CATSON_SIP_SVC_GET_CERT_CTR		0xC2000031
#define CATSON_SIP_SVC_GET_EKEY			0xC2000032
#define CATSON_SIP_SVC_GET_CKEY			0xC2000033

#define CATSON_SIP_SVC_LOAD_ROTPK_CERT		0xC2000040
#define CATSON_SIP_SVC_BLOW_SKEY		0xC2000041
#define CATSON_SIP_SVC_BLOW_RKEY		0xC2000042
#define CATSON_SIP_SVC_BLOW_PKEY		0xC2000043
#define CATSON_SIP_SVC_REVOKE_SKEY		0xC2000045
#define CATSON_SIP_SVC_BLOW_STATE_CLOSED	0xC2000046
#define CATSON_SIP_SVC_SET_BOARD_TYPE		0xC2000047
#define CATSON_SIP_SVC_SET_BFIP_NVCTR		0xC2000048
#define CATSON_SIP_SVC_SET_NVCTR		0xC2000049

#define CATSON_SIP_SVC_SET_BOOT_KEY		0xC2000050
#define CATSON_SIP_SVC_SET_NVAL			0xC2000051
#define CATSON_SIP_SVC_SET_KEYS			0xC2000052

#define CATSON_SIP_SVC_LOAD_RMA_CERT		0xC2000060
#define CATSON_SIP_SVC_BLOW_RMA			0xC2000061

#define SMC_UNK					0xffffffff

#define CATSON_NUM_SKEY		6
#define CATSON_FUSE_SIZE	4
#define CATSON_KEY_FUSE_CNT	8
#define CATSON_KEY_SIZE		(CATSON_FUSE_SIZE * CATSON_KEY_FUSE_CNT)

/* eMMC and dish config keys are 128 bits. */
#define CATSON_EKEY_SIZE	16
#define CATSON_CKEY_SIZE	16

#define SKEY_IDX_DEV		0x80

#define CATSON_BSECX_SEC_STATE_OFFSET		0x00
/* Defined "closed" to really mean JTAG killed */
#define     CATSON_BSECX_SEC_STATE_CLOSED		0xffffffff
#define CATSON_BSECX_RMA_OFFSET			0x04
#define CATSON_BSECX_OPTIONS_OFFSET		0x08
#define CATSON_BSECX_BFIP_NVCOUNTER_OFFSET	0x0c
#define CATSON_BSECX_DEVID0_OFFSET		0x10
#define CATSON_BSECX_DEVID1_OFFSET		0x14
#define CATSON_BSECX_DEVID2_OFFSET		0x18
#define CATSON_BSECX_AVS_CODE_OFFSET		0x1c
#define CATSON_BSECX_SKEY_NVAL_OFFSET		0x34
#define CATSON_BSECX_DATALOG_6_OFFSET		0x50
#define CATSON_BSECX_NVCOUNTER_OFFSET		0x74

struct catson_fuses {
	struct device *dev;
	void __iomem *bsecx;

	struct {
		u32 rom_rotpk[CATSON_KEY_FUSE_CNT];
		u32 skey[CATSON_NUM_SKEY][CATSON_KEY_FUSE_CNT];
		u64 ekey[CATSON_EKEY_SIZE / 8];
		u64 ckey[CATSON_CKEY_SIZE / 8];
		u32 board_type;
		u8 skey_idx;
		u8 soft_resets;

		struct {
			u32 bl20_trusted_boot_fw;

			u32 trusted_boot_fw;
			u32 trusted_key;

			u32 soc_fw_key;
			u32 soc_fw_content;

			u32 non_trusted_fw_key;
			u32 non_trusted_fw_content;
		} cert_ctr;
	} cache;
};

/**
 * sx_catson_fuse_cache() - Cache the values in the ATF
 *
 * @param cfuse The catson fuses structure to cache into
 *
 * Return: 0 on success, <0 on error
 */
static int sx_catson_fuse_cache(struct catson_fuses *cfuse)
{
	int i;
	struct device *dev = cfuse->dev;
	struct arm_smccc_res res = {};

	arm_smccc_smc(CATSON_SIP_SVC_GET_ROM_ROTPK, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == SMC_UNK) {
		dev_err(dev, "Failed to read bootrom key with error %lx\n",
			res.a0);
		return -EIO;
	}

	memcpy(cfuse->cache.rom_rotpk, &res, sizeof(res));

	memset(&res, 0, sizeof(res));
	arm_smccc_smc(CATSON_SIP_SVC_GET_SKEY_IDX, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(dev, "Failed to read skey idx with error %lx\n",
			res.a0);
		return -EIO;
	}
	cfuse->cache.skey_idx = res.a1 & 0xff;

	memset(&res, 0, sizeof(res));
	arm_smccc_smc(CATSON_SIP_SVC_GET_SOFT_RESET_COUNT, 0, 0, 0, 0, 0, 0, 0,
		      &res);
	if (res.a0) {
		dev_err(dev, "Failed to read soft reset count with error %lx\n",
			res.a0);
		return -EIO;
	}
	cfuse->cache.soft_resets = res.a1 & 0xff;

	for (i = 0; i < CATSON_NUM_SKEY; i++) {
		memset(&res, 0, sizeof(res));

		arm_smccc_smc(CATSON_SIP_SVC_GET_SKEY,
			      i, 0, 0, 0, 0, 0, 0, &res);

		if (res.a0 == SMC_UNK) {
			memset(cfuse->cache.skey[i], 0xff,
			       sizeof(cfuse->cache.skey[i]));
		} else {
			memcpy(cfuse->cache.skey[i], &res,
			       min(sizeof(cfuse->cache.skey[i]), sizeof(res)));
		}
	}

	memset(&res, 0, sizeof(res));
	arm_smccc_smc(CATSON_SIP_SVC_GET_BOARD_TYPE, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(dev, "Failed to read the fuse board type\n");
		return -EIO;
	}
	cfuse->cache.board_type = res.a1;

	memset(&res, 0, sizeof(res));
	arm_smccc_smc(CATSON_SIP_SVC_GET_BFIP_CERT_CTR,
		      0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(dev, "Failed to read the bootfip cert counter\n");
		return -EIO;
	}
	cfuse->cache.cert_ctr.bl20_trusted_boot_fw = res.a1;

	memset(&res, 0, sizeof(res));
	arm_smccc_smc(CATSON_SIP_SVC_GET_CERT_CTR, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(dev, "Failed to read the fip cert counters\n");
		return -EIO;
	}

	cfuse->cache.cert_ctr.trusted_boot_fw = res.a1 & 0xffffffff;
	cfuse->cache.cert_ctr.trusted_key =  res.a1 >> 32;

	cfuse->cache.cert_ctr.soc_fw_key = res.a2 >> 32;
	cfuse->cache.cert_ctr.soc_fw_content = res.a2 & 0xffffffff;

	cfuse->cache.cert_ctr.non_trusted_fw_key = res.a3 >> 32;
	cfuse->cache.cert_ctr.non_trusted_fw_content = res.a3 & 0xffffffff;

	memset(&res, 0, sizeof(res));
	arm_smccc_smc(CATSON_SIP_SVC_GET_EKEY, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == SMC_UNK) {
		memset(cfuse->cache.ekey, 0xff, sizeof(cfuse->cache.ekey));
	} else {
		memcpy(cfuse->cache.ekey, &res, min(sizeof(res),
					sizeof(cfuse->cache.ekey)));
	}

	memset(&res, 0, sizeof(res));
	arm_smccc_smc(CATSON_SIP_SVC_GET_CKEY, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == SMC_UNK) {
		memset(cfuse->cache.ckey, 0xff, sizeof(cfuse->cache.ckey));
	} else {
		memcpy(cfuse->cache.ckey, &res, min(sizeof(res),
					sizeof(cfuse->cache.ckey)));
	}

	return 0;
}

/**
 * catson_fuse_sysfs_read_fuse() - Generic function for printing a fuse
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 * @offset the address offset to read
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t catson_fuse_sysfs_read_fuse(struct device *dev,
					   struct device_attribute *attr,
					   char *buf,
					   uintptr_t offset)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	u32 data;

	data = ioread32(cfuse->bsecx + offset);

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", data);
}

/**
 * show_key_raw() - displays key from fuse cache.
 *
 * @dev the ID core device (ignored).
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 * @fuse Catson fuse cache to read from.
 * @key_size Length in bytes to read from cache.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_key_raw(struct device *dev, struct device_attribute *attr,
			    char *buf, void *fuse, ssize_t key_size)
{
	memcpy(buf, fuse, key_size);

	return key_size;
}

static ssize_t show_key_fuse(struct device *dev, struct device_attribute *attr,
			     char *buf, void *fuse)
{
	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", *((u32 *)fuse));
}

#define CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, __idx)		       \
static ssize_t show_key##__name##__idx(struct device *dev,		       \
				  struct device_attribute *attr, char *buf)    \
{									       \
	struct catson_fuses *cfuse = dev_get_drvdata(dev);		       \
	return show_key_fuse(dev, attr, buf, &cfuse->cache.__data[__idx]);     \
}									       \
static struct device_attribute dev_attr_show_##__name##_##__idx =	       \
				__ATTR(fuse##__idx, 0444,		       \
				       show_key##__name##__idx,  NULL)

#define CATSON_SYSFS_SHOW_KEY(__name, __data)				       \
static ssize_t show_##__name(struct device *dev, struct device_attribute *attr,\
			     char *buf)					       \
{									       \
	struct catson_fuses *cfuse = dev_get_drvdata(dev);		       \
	return show_key_raw(dev, attr, buf, cfuse->cache.__data,               \
			CATSON_KEY_SIZE);	                               \
}									       \
static struct device_attribute dev_attr_show_##__name##_raw =		       \
				__ATTR(raw, 0444, show_##__name, NULL);	       \
CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, 0);			       \
CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, 1);			       \
CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, 2);			       \
CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, 3);			       \
CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, 4);			       \
CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, 5);			       \
CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, 6);			       \
CATSON_SYSFS_SHOW_KEY_OFFSET(__name, __data, 7);			       \
static struct attribute *sx_catson_fuse_##__name##_device_attrs[] = {	       \
	&dev_attr_show_##__name##_raw.attr,				       \
	&dev_attr_show_##__name##_0.attr,				       \
	&dev_attr_show_##__name##_1.attr,				       \
	&dev_attr_show_##__name##_2.attr,				       \
	&dev_attr_show_##__name##_3.attr,				       \
	&dev_attr_show_##__name##_4.attr,				       \
	&dev_attr_show_##__name##_5.attr,				       \
	&dev_attr_show_##__name##_6.attr,				       \
	&dev_attr_show_##__name##_7.attr,				       \
	NULL,								       \
};									       \
static const struct attribute_group sx_catson_fuse_##__name##_sysfs_regs_group = { \
	.name = #__name, .attrs = sx_catson_fuse_##__name##_device_attrs,     \
}

CATSON_SYSFS_SHOW_KEY(skey0, skey[0]);
CATSON_SYSFS_SHOW_KEY(skey1, skey[1]);
CATSON_SYSFS_SHOW_KEY(skey2, skey[2]);
CATSON_SYSFS_SHOW_KEY(skey3, skey[3]);
CATSON_SYSFS_SHOW_KEY(skey4, skey[4]);
CATSON_SYSFS_SHOW_KEY(skey5, skey[5]);
CATSON_SYSFS_SHOW_KEY(rkey,  rom_rotpk);

/**
 * show_ekey() - returns the derived eMMC key.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_ekey(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	if (!cfuse) {
		dev_err(dev, "Failed to get device data\n");
		return -ENODATA;
	}

	return show_key_raw(dev, attr, buf, cfuse->cache.ekey,
			CATSON_EKEY_SIZE);
}
static DEVICE_ATTR(ekey, 0400, show_ekey, NULL);

/**
 * show_ckey() - returns the derived dish config key.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_ckey(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	if (!cfuse) {
		dev_err(dev, "Failed to get device data\n");
		return -ENODATA;
	}

	return show_key_raw(dev, attr, buf, cfuse->cache.ckey,
			CATSON_CKEY_SIZE);
}
static DEVICE_ATTR(ckey, 0400, show_ckey, NULL);

/**
 * show_skey_idx() - displays the key index used for current boot.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_skey_idx(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", cfuse->cache.skey_idx);
}

/**
 * This sysfs attribute returns data about what skey was used.
 */
static DEVICE_ATTR(skey_idx, 0444, show_skey_idx, NULL);

/**
 * show_board_type() - Shows the fuse based board type.
 *
 * @dev the fuse device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
const char *board_num_to_type[] = {
	"UNSET",
	"utdev",
	"mmut",
};

static ssize_t show_board_type(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	if (cfuse->cache.board_type >= ARRAY_SIZE(board_num_to_type))
	{
		dev_err(dev, "Unknown board type %d\n",
			cfuse->cache.board_type);
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 board_num_to_type[cfuse->cache.board_type]);
}

/**
 * set_board_type() - Sets the board type.
 *
 * @dev the fuse device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t set_board_type(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int i;
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	struct arm_smccc_res res = {};

	if (ioread32(cfuse->bsecx + CATSON_BSECX_SEC_STATE_OFFSET) ==
			CATSON_BSECX_SEC_STATE_CLOSED) {
		dev_err(dev, "Unable to set board type on secure boards\n");
		return -EIO;
	}

	/* Allow strings to be blanked for retired boards. */
	if (!strlen(buf)) {
		dev_err(dev, "Invalid board type ''\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(board_num_to_type); i++) {
		if (!strncmp(buf, board_num_to_type[i], count)) {
			arm_smccc_smc(CATSON_SIP_SVC_SET_BOARD_TYPE, i, 0, 0, 0, 0, 0, 0, &res);

			if (res.a0) {
				dev_err(dev, "Failed to blow type fuse to '%s'\n", buf);
				return -EIO;
			} else {
				sx_catson_fuse_cache(cfuse);
				dev_info(dev, "Board type fuse blown to '%s'\n", buf);
				return count;
			}
		}
	}

	dev_err(dev, "Unknown board type '%s'\n", buf);

	return -EIO;
}

/**
 * This sysfs attribute returns data about what the board type is.
 */
static DEVICE_ATTR(board_type, 0644, show_board_type, set_board_type);

/**
 * show_soft_resets() - displays number of soft resets that have occurred.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_soft_resets(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", cfuse->cache.soft_resets);
}

/**
 * This sysfs attribute returns the number of soft resets experienced.
 */
static DEVICE_ATTR(soft_resets, 0444, show_soft_resets, NULL);

/**
 * show_fuse_map() - displays the catson fuse map.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_fuse_map(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	u32 data;

	data = ioread32(cfuse->bsecx + CATSON_BSECX_SKEY_NVAL_OFFSET);

	data = (~data) & 0xff;

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", data);
}
static DEVICE_ATTR(fuse_map, 0444, show_fuse_map, NULL);

/**
 * show_avs_code() - displays the catson fuse map.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_avs_code(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return catson_fuse_sysfs_read_fuse(dev, attr, buf,
					   CATSON_BSECX_AVS_CODE_OFFSET);
}
static DEVICE_ATTR(avs_code, 0444, show_avs_code, NULL);

/**
 * show_avs_core_code() - displays the catson avs core code.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_avs_code_core(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	u32 data;

	data = ioread32(cfuse->bsecx + CATSON_BSECX_AVS_CODE_OFFSET);
	data &= 0x1F;

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", data);
}
static DEVICE_ATTR(avs_code_core, 0444, show_avs_code_core, NULL);

/**
 * show_avs_cpu_code() - displays the catson avs cpu code.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_avs_code_cpu(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	u32 data;

	data = ioread32(cfuse->bsecx + CATSON_BSECX_AVS_CODE_OFFSET);
	data = (data & 0x1F00) >> 8;

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", data);
}
static DEVICE_ATTR(avs_code_cpu, 0444, show_avs_code_cpu, NULL);

/**
 * show_device_id0() - displays the catson device id 0 fuse.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_device_id0(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return catson_fuse_sysfs_read_fuse(dev, attr, buf,
					   CATSON_BSECX_DEVID0_OFFSET);
}
static DEVICE_ATTR(devid0, 0444, show_device_id0, NULL);

/**
 * show_device_id1() - displays the catson device id 1 fuse.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_device_id1(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return catson_fuse_sysfs_read_fuse(dev, attr, buf,
					   CATSON_BSECX_DEVID1_OFFSET);
}
static DEVICE_ATTR(devid1, 0444, show_device_id1, NULL);

/**
 * show_device_id2() - displays the catson device id 2 fuse.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_device_id2(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return catson_fuse_sysfs_read_fuse(dev, attr, buf,
					   CATSON_BSECX_DEVID2_OFFSET);
}
static DEVICE_ATTR(devid2, 0444, show_device_id2, NULL);

/**
 * show_device_id3() - displays the catson device id 3 (datalog 6) fuse.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_device_id3(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return catson_fuse_sysfs_read_fuse(dev, attr, buf,
					   CATSON_BSECX_DATALOG_6_OFFSET);
}
static DEVICE_ATTR(devid3, 0444, show_device_id3, NULL);

/**
 * run_load_cert() - loads the ceritificate for upcoming key burns.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t run_load_cert(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct arm_smccc_res res = {};
	dma_addr_t handle;

	if (count > PAGE_SIZE) {
		return -EINVAL;
	}

	handle = dma_map_single(dev, (void *)buf, PAGE_SIZE, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, handle)) {
		return dma_mapping_error(dev, handle);
	}

	arm_smccc_smc(CATSON_SIP_SVC_LOAD_ROTPK_CERT, handle, count, 0, 0, 0, 0,
		      0, &res);

	dma_unmap_single(dev, handle, PAGE_SIZE, DMA_BIDIRECTIONAL);

	if (res.a0) {
		dev_err(dev, "Failed to load cert with error %lx\n", res.a0);
		return -EIO;
	}

	return count;
}
static DEVICE_ATTR(load_cert, 0200, NULL, run_load_cert);

/**
 * burn_key() - burns a new key into fuses.
 *
 * @command the SMC command to use to trigger the fuse burn.
 * @dev the ID core device.
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t burn_key(unsigned long command, struct device *dev,
			const char *buf, size_t count)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	struct arm_smccc_res res = {};
	dma_addr_t handle;

	if (count > PAGE_SIZE) {
		return -EINVAL;
	}

	handle = dma_map_single(dev, (void *)buf, PAGE_SIZE, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, handle)) {
		return dma_mapping_error(dev, handle);
	}

	arm_smccc_smc(command, handle, count, 0, 0, 0, 0, 0, &res);
	dma_unmap_single(dev, handle, PAGE_SIZE, DMA_BIDIRECTIONAL);

	if (res.a0) {
		dev_err(dev, "Failed to burn key with error %lx\n", res.a0);
		return -EIO;
	}

	sx_catson_fuse_cache(cfuse);

	dev_info(dev, "Successfully burned new key %ld\n", res.a1);
	return count;
}

/**
 * burn_skey() - burns a new skey into fuses.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t run_burn_skey(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	return burn_key(CATSON_SIP_SVC_BLOW_SKEY, dev, buf, count);
}
static DEVICE_ATTR(burn_skey, 0200, NULL, run_burn_skey);

/**
 * burn_rkey() - burns a new rkey into fuses.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t run_burn_rkey(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	return burn_key(CATSON_SIP_SVC_BLOW_RKEY, dev, buf, count);
}
static DEVICE_ATTR(burn_rkey, 0200, NULL, run_burn_rkey);

/**
 * run_burn_pkey() - burns a random 128 bit private key into fuses.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write (ignored).
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t run_burn_pkey(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	struct arm_smccc_res res = {};

	if (!buf || buf[0] != '1') {
		dev_err(dev, "Failed to burn pkey from bad user input\n");
		return -EINVAL;
	}

	arm_smccc_smc(CATSON_SIP_SVC_BLOW_PKEY, 0, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0) {
		dev_err(dev, "Failed to burn pkey with error %lx\n", res.a0);
		return -EIO;
	}

	sx_catson_fuse_cache(cfuse);

	return count;
}
static DEVICE_ATTR(burn_pkey, 0200, NULL, run_burn_pkey);

/**
 * run_revoke_skeys() - revokes all skeys except the current skey.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input (ignored).
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t run_revoke_skeys(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct arm_smccc_res res = {};

	if (strncmp(buf, "revoke", count)) {
		dev_err(dev, "Failed to revoke keys from bad user input\n");
		return -EINVAL;
	}

	arm_smccc_smc(CATSON_SIP_SVC_REVOKE_SKEY, 0, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0) {
		dev_err(dev, "Failed to revoke keys with error %lx\n", res.a0);
		return -EIO;
	}

	dev_err(dev, "Revoked skeys to new value %lx\n", res.a1);

	return count;
}
static DEVICE_ATTR(revoke_skeys, 0200, NULL, run_revoke_skeys);

/**
 * run_load_rma_cert() - loads the ceritificate for upcoming rma operation.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t load_rma_cert_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct arm_smccc_res res = {};
	dma_addr_t handle;

	if (count > PAGE_SIZE) {
		return -EINVAL;
	}

	handle = dma_map_single(dev, (void *)buf, PAGE_SIZE, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, handle)) {
		return dma_mapping_error(dev, handle);
	}

	arm_smccc_smc(CATSON_SIP_SVC_LOAD_RMA_CERT, handle, count, 0, 0, 0, 0,
		      0, &res);

	dma_unmap_single(dev, handle, PAGE_SIZE, DMA_BIDIRECTIONAL);

	if (res.a0) {
		dev_err(dev, "Failed to load cert with error %lx\n", res.a0);
		return -EIO;
	}

	return count;
}
static DEVICE_ATTR_WO(load_rma_cert);

/**
 * burn_rma() - burns the magic value into the RMA fuse
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t burn_rma_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	return burn_key(CATSON_SIP_SVC_BLOW_RMA, dev, buf, count);
}
static DEVICE_ATTR_WO(burn_rma);

/**
 * show_security_state() - prints whether the security state is closed
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_security_state(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	u32 data;

	data = ioread32(cfuse->bsecx + CATSON_BSECX_SEC_STATE_OFFSET);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 data == CATSON_BSECX_SEC_STATE_CLOSED);
}

/**
 * set_security_state() - Sets the security state.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t set_security_state(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	struct arm_smccc_res res = {};
	int rom_rotpk_zero = 1;
	int i;

	if (strncmp(buf, "close", count)) {
		dev_err(dev, "Failed to set state to closed from bad user input\n");
		return -EINVAL;
	}

	if (cfuse->cache.skey_idx == SKEY_IDX_DEV) {
		dev_err(dev, "Cannot close secuirty state from dev key\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(cfuse->cache.rom_rotpk); i++) {
		if (cfuse->cache.rom_rotpk[i]) {
			rom_rotpk_zero = 0;
			break;
		}
	}
	if (rom_rotpk_zero) {
		dev_err(dev, "Cannot close secuirty state with unsigned bootfip\n");
		return -EINVAL;
	}

	arm_smccc_smc(CATSON_SIP_SVC_BLOW_STATE_CLOSED, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(dev, "Failed to revoke keys with error %lx\n", res.a0);
		return -EIO;
	}

	dev_alert(dev, "Board security state is now set to closed\n");

	return count;
}
static DEVICE_ATTR(sec_state, 0644, show_security_state, set_security_state);

/**
 * rma_state_show() - prints whether the rma fuse has been blown.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t rma_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	u32 data;

	data = ioread32(cfuse->bsecx + CATSON_BSECX_RMA_OFFSET);

	/* Do not return the fuse value, only whether fuse value is set. */
	return scnprintf(buf, PAGE_SIZE, "%d\n", !!data);
}
static DEVICE_ATTR_RO(rma_state);

/**
 * options_show() - prints the bsecx options fuse
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t options_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	u32 data;

	data = ioread32(cfuse->bsecx + CATSON_BSECX_OPTIONS_OFFSET);

	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", data);
}
static DEVICE_ATTR_RO(options);

/**
 * watchdog_reboot() - Sets the security state.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t watchdog_reboot(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct arm_smccc_res res = {};

	if (strncmp(buf, CATSON_RESET_MAGIC_STRING, count)) {
		dev_err(dev,
			"User input did not match magic string '"CATSON_RESET_MAGIC_STRING"'\n");
		return -EINVAL;
	}

	arm_smccc_smc(CATSON_SIP_SVC_RESET, 0, 0, 0, 0, 0, 0, 0, &res);

	dev_err(dev, "Failed to reset the Catson (0x%lx)\n", res.a0);

	return -EIO;
}
static DEVICE_ATTR(reboot, 0200, NULL, watchdog_reboot);

/**
 * All of our sysfs attributes in one place.
 */
static struct attribute *sx_catson_fuse_device_attrs[] = {
	/* Cached Attributes */
	&dev_attr_skey_idx.attr,
	&dev_attr_soft_resets.attr,
	&dev_attr_board_type.attr,

	/* IO Mem Attributes */
	&dev_attr_fuse_map.attr,
	&dev_attr_avs_code.attr,
	&dev_attr_avs_code_core.attr,
	&dev_attr_avs_code_cpu.attr,
	&dev_attr_devid0.attr,
	&dev_attr_devid1.attr,
	&dev_attr_devid2.attr,
	&dev_attr_devid3.attr,
	&dev_attr_rma_state.attr,
	&dev_attr_options.attr,

	/* SMC Attributes */
	&dev_attr_load_cert.attr,
	&dev_attr_burn_skey.attr,
	&dev_attr_burn_rkey.attr,
	&dev_attr_load_rma_cert.attr,
	&dev_attr_burn_rma.attr,
	&dev_attr_revoke_skeys.attr,
	&dev_attr_sec_state.attr,
	&dev_attr_burn_pkey.attr,
	&dev_attr_ekey.attr,
	&dev_attr_ckey.attr,

	/* SMC Commands */
	&dev_attr_reboot.attr,

	/* Sentinel */
	NULL,
};

static const struct attribute_group sx_catson_fuse_sysfs_regs_group = {
	.attrs = sx_catson_fuse_device_attrs,
};

/**
 * show_cert_ctr_XXXX() - displays the fips certificate nvctr used for anti-rollback.
 *
 * @dev the ID core device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_cert_ctr_bl20(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 cfuse->cache.cert_ctr.bl20_trusted_boot_fw);
}
static DEVICE_ATTR(cert_ctr_bl20, 0444, show_cert_ctr_bl20, NULL);

static ssize_t show_cert_ctr_bl2(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 cfuse->cache.cert_ctr.trusted_boot_fw);

}
static DEVICE_ATTR(cert_ctr_bl2, 0444, show_cert_ctr_bl2, NULL);

static ssize_t show_cert_ctr_trusted_key(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 cfuse->cache.cert_ctr.trusted_key);

}
static DEVICE_ATTR(cert_ctr_trusted_key, 0444, show_cert_ctr_trusted_key, NULL);

static ssize_t show_cert_ctr_bl31_key(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 cfuse->cache.cert_ctr.soc_fw_key);
}
static DEVICE_ATTR(cert_ctr_bl31_key, 0444, show_cert_ctr_bl31_key, NULL);

static ssize_t show_cert_ctr_bl31_content(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 cfuse->cache.cert_ctr.soc_fw_content);
}
static DEVICE_ATTR(cert_ctr_bl31_content, 0444, show_cert_ctr_bl31_content, NULL);

static ssize_t show_cert_ctr_bl33_key(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 cfuse->cache.cert_ctr.non_trusted_fw_key);
}
static DEVICE_ATTR(cert_ctr_bl33_key, 0444, show_cert_ctr_bl33_key, NULL);

static ssize_t show_cert_ctr_bl33_content(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 cfuse->cache.cert_ctr.non_trusted_fw_content);
}
static DEVICE_ATTR(cert_ctr_bl33_content, 0444, show_cert_ctr_bl33_content, NULL);

/**
 * show_bfip_nvctr() - displays the catson bootfip nvcounter.
 *
 * @dev the fuse device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_bfip_nvctr(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return catson_fuse_sysfs_read_fuse(dev, attr, buf,
					   CATSON_BSECX_BFIP_NVCOUNTER_OFFSET);
}

/**
 * set_bfip_nvctr() - Sets the bootfip nvctr.
 *
 * @dev the fuse device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t set_bfip_nvctr(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	struct arm_smccc_res res = {};
	u32 current_bfip_nvctr = ioread32(cfuse->bsecx + \
					  CATSON_BSECX_BFIP_NVCOUNTER_OFFSET);
	u32 blank_fuse[CATSON_KEY_FUSE_CNT] = {};

	if (strncmp(buf, "norollback", count)) {
		dev_err(dev, "Failed to set bootfip nvcounter from bad user input\n");
		return -EINVAL;
	}

	if (cfuse->cache.cert_ctr.bl20_trusted_boot_fw == 0) {
		dev_err(dev, "Unable to set bootfip nvcounter as current counter is 0\n");
		return -EINVAL;
	}

	if (!memcmp(blank_fuse, cfuse->cache.rom_rotpk, sizeof(blank_fuse))) {
		dev_err(dev, "Unable to set bootfip nvcounter as production bootfip not loaded\n");
		return -EINVAL;
	}

	if (current_bfip_nvctr == cfuse->cache.cert_ctr.bl20_trusted_boot_fw) {
		dev_info(dev, "Bootfip nvcounter already up to date\n");
		return count;
	}

	arm_smccc_smc(CATSON_SIP_SVC_SET_BFIP_NVCTR,
		      cfuse->cache.cert_ctr.bl20_trusted_boot_fw,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(dev, "Failed to set bootfip nvcounter (0x%lx)\n",
			res.a0);
		return -EIO;
	}

	return count;
}

/**
 * This sysfs attribute returns data about the bootfip nvctr.
 */
static DEVICE_ATTR(bfip_nvctr, 0644, show_bfip_nvctr, set_bfip_nvctr);

/**
 * show_fip_nvctr() - displays the catson fip nvcounter.
 *
 * @dev the fuse device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer to fill with output.
 *
 * Return: the number of bytes written to buf, or a negative value on error.
 */
static ssize_t show_fip_nvctr(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return catson_fuse_sysfs_read_fuse(dev, attr, buf,
					   CATSON_BSECX_NVCOUNTER_OFFSET);
}

/**
 * set_bfip_nvctr() - Sets the fip nvctr.
 *
 * @dev the fuse device.
 * @attr the device_attribute we are using (ignored).
 * @buf the kernel-buffer filled with input.
 * @count the requested number of bytes to write.
 *
 * Return: count on success, or a negative value on error.
 */
static ssize_t set_fip_nvctr(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct catson_fuses *cfuse = dev_get_drvdata(dev);
	struct arm_smccc_res res = {};
	u32 current_bfip_nvctr = ioread32(cfuse->bsecx + \
					  CATSON_BSECX_NVCOUNTER_OFFSET);
	u32 tb_fw_cert = cfuse->cache.cert_ctr.trusted_boot_fw;

	if (strncmp(buf, "norollback", count)) {
		dev_err(dev, "Failed to set fip nvcounter from bad user input\n");
		return -EINVAL;
	}

	if (cfuse->cache.skey_idx == SKEY_IDX_DEV) {
		dev_err(dev, "Unable to set fip nvcounter as production fip not loaded\n");
		return -EINVAL;
	}

	if (tb_fw_cert == 0) {
		dev_err(dev, "Unable to set fip nvcounter as current counter is 0\n");
		return -EINVAL;
	}

	if ((tb_fw_cert != cfuse->cache.cert_ctr.trusted_key) ||
	    (tb_fw_cert != cfuse->cache.cert_ctr.soc_fw_key) ||
	    (tb_fw_cert != cfuse->cache.cert_ctr.soc_fw_content) ||
	    (tb_fw_cert != cfuse->cache.cert_ctr.non_trusted_fw_key) ||
	    (tb_fw_cert != cfuse->cache.cert_ctr.non_trusted_fw_content)) {
		dev_err(dev, "Certificates are in inconsistent state.  Unable to blow nv counter\n");
		return -EIO;
	}

	if (current_bfip_nvctr == tb_fw_cert) {
		dev_info(dev, "Bootfip nvcounter already up to date\n");
		return count;
	}

	arm_smccc_smc(CATSON_SIP_SVC_SET_NVCTR,
		      tb_fw_cert,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == SMC_UNK) {
		dev_err(dev, "Failed to set fip nvcounter (0x%lx)\n",
			res.a0);
		return -EIO;
	}

	return count;
}

/**
 * This sysfs attribute returns data about the fip nvctr.
 */
static DEVICE_ATTR(fip_nvctr, 0644, show_fip_nvctr, set_fip_nvctr);

static struct attribute *sx_catson_nvctr_device_attrs[] = {
	/* Cached Attributes */
	&dev_attr_cert_ctr_bl20.attr,
	&dev_attr_cert_ctr_bl2.attr,
	&dev_attr_cert_ctr_trusted_key.attr,
	&dev_attr_cert_ctr_bl31_key.attr,
	&dev_attr_cert_ctr_bl31_content.attr,
	&dev_attr_cert_ctr_bl33_key.attr,
	&dev_attr_cert_ctr_bl33_content.attr,

	/* Fuse Attributes */
	&dev_attr_bfip_nvctr.attr,
	&dev_attr_fip_nvctr.attr,

	/* Sentinel */
	NULL,
};

static const struct attribute_group sx_catson_fuse_nvctr_regs_group = {
	.name = "nvctr",
	.attrs = sx_catson_nvctr_device_attrs,
};

static const struct attribute_group *sx_catson_fuse_sysfs_groups[] = {
	&sx_catson_fuse_sysfs_regs_group,
	&sx_catson_fuse_nvctr_regs_group,
	&sx_catson_fuse_rkey_sysfs_regs_group,
	&sx_catson_fuse_skey0_sysfs_regs_group,
	&sx_catson_fuse_skey1_sysfs_regs_group,
	&sx_catson_fuse_skey2_sysfs_regs_group,
	&sx_catson_fuse_skey3_sysfs_regs_group,
	&sx_catson_fuse_skey4_sysfs_regs_group,
	&sx_catson_fuse_skey5_sysfs_regs_group,
	NULL,
};

/**
 * sx_catson_fuse_probe() - probe function for SpaceX Catson fuse interaction.
 *
 * This is called when the kernel wants us to attach to a device.
 *
 * @param dev Pointer into the platform device.
 *
 *
 * Return: 0 if the device was claimed.
 */
static int sx_catson_fuse_probe(struct platform_device *pdev)
{
	struct catson_fuses *cfuse;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int rc = 0;

	BUILD_BUG_ON(sizeof(struct arm_smccc_res) != CATSON_KEY_SIZE);
	if (WARN_ON(!pdev))
		return -1;

	dev_info(dev, "Probing %s\n", dev->of_node->name);

	cfuse = devm_kzalloc(dev, sizeof(*cfuse), GFP_KERNEL);
	if (!cfuse)
		return -ENOMEM;

	dev_set_drvdata(dev, cfuse);
	cfuse->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cfuse->bsecx = devm_ioremap_resource(dev, res);
	if (IS_ERR(cfuse->bsecx)) {
		dev_err(cfuse->dev, "Failed to remap BSECX region\n");
		return PTR_ERR(cfuse->bsecx);
	}

	rc = sx_catson_fuse_cache(cfuse);
	if (rc < 0) {
		dev_err(dev, "Failed to cache key values (%d)\n", rc);
		return rc;
	}

	rc = sysfs_create_groups(&dev->kobj, sx_catson_fuse_sysfs_groups);
	if (rc) {
		dev_err(dev, "Failed creating reg sysfs (%d)\n", rc);
		return rc;
	}

	return 0;
};

/**
 * sx_catson_fuse_remove() - kernel call to unbind from the device.
 *
 * @param dev the ID core device to unbind from
 *
 * Return: 0 on success
 */
static int sx_catson_fuse_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (WARN_ON(!pdev))
		return -1;

	sysfs_remove_groups(&dev->kobj, sx_catson_fuse_sysfs_groups);

	dev_info(dev, "Removed Catson fuse driver\n");

	return 0;
}

/**
 * Here we list which devices we are compatible with.
 */
static const struct of_device_id sx_catson_fuse_ids[] = {
	{
		.compatible = "sx,catson-fuse-1.00.a",
	},
	{},
};

/**
 * Now we register the compatibility information with the kernel.
 */
MODULE_DEVICE_TABLE(of, sx_catson_fuse_ids);

/**
 * Here we define the driver itself.
 */
static struct platform_driver sx_catson_fuse_driver = {
	.driver = {
		.name       = "sx-catson-fuse",
		.owner      = THIS_MODULE,
		.of_match_table = sx_catson_fuse_ids,
	},
	.probe  = sx_catson_fuse_probe,
	.remove = sx_catson_fuse_remove,
};

/**
 * And then we register the driver with the kernel.
 */
module_platform_driver(sx_catson_fuse_driver);

/**
 * Here is our stylish module documentation.
 * @{
 */
MODULE_AUTHOR("Kevin Bosien <kbosien@spacex.com>");
MODULE_DESCRIPTION("SpaceX Catson Fuse Driver");
MODULE_ALIAS("platform:" SX_CATSON_FUSE_NAME);
MODULE_LICENSE("GPL");
/**
 * @}
 */
