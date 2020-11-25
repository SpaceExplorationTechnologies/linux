/*
 * I2C Driver for the TI DS125 retimer.
 * http://www.ti.com/lit/ds/symlink/ds125df410.pdf
 *
 * Copyright (C) 2020 Space Exploration Technologies
 * Kevin Bosien <kevin.bosien@spacex.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/module.h>

#define DS125_I2C_DRIVER_NAME "ds125_i2c"

#define DS125_CHANNEL_COUNT	4

#define DS125_REG_DEVID			0x01
#define		DS125_REG_DEVID_DEV_MASK	0x1F
#define		DS125_REG_DEVID_DEV_VAL		0x11
#define DS125_CHAN_REG_CDR_STATUS	0x02
#define DS125_CHAN_REG_EQ_BST		0x03
#define DS125_CHAN_REG_HEO		0x27
#define DS125_CHAN_REG_VEO		0x28
#define DS125_CHAN_REG_EOM_VRANGE	0x29
#define DS125_REG_CHAN_SELECT		0xFF
#define		DS125_REG_CHAN_SELECT_GLOB	0
#define		DS125_REG_CHAN_SELECT_BASE	4
#define		DS125_CHAN(__raw_chan)  ((__raw_chan) + DS125_REG_CHAN_SELECT_BASE)

struct i2c_table {
	u8 address;
	u8 data;
};

const struct i2c_table setup_9p96g[] = {
	{0x2f, 0x74},
	{0x61, 0xb1},
	{0x63, 0xb1},
	{0x60, 0xcc},
	{0x62, 0xcc},
	{0x64, 0x66},
	/* Power off the DFE */
	{0x1e, 0xe9},
	/* Set swing to 1.3 Vpkpk */
	{0x2d, 0x87},
};

const struct i2c_table setup_6g[] = {
	{0x2f, 0xa4},
	{0x61, 0xbc},
	{0x63, 0xbc},
	{0x60, 0x00},
	{0x62, 0x00},
	{0x64, 0x77},
	/* Power off the DFE */
	{0x1e, 0xe9},
	/* Set swing to .6 Vpkpk */
	{0x2d, 0x80},
};

const struct i2c_table setup_11p04g[] = {
	{0x2f, 0x74},
	{0x61, 0xb7},
	{0x63, 0xb7},
	{0x60, 0x33},
	{0x62, 0x33},
	{0x64, 0x77},
	/* Power off the DFE and bypass the CDR */
	{0x09, 0x20},
	{0x1e, 0x09},
	/* Disabling fast capacitor search per recommendation in 7.5.10, rev G*/
	{0x3f, 0x80},
	/* Set swing to 0.8 Vpkpk */
	{0x2d, 0x82},
	/* Freeze retimer CTLE settings to 0 per 7.5.8 */
	{0x31, 0x00},
	{0x3a, 0x00},
	{0x03, 0x00},
	{0x40, 0x00},
};

const struct i2c_table setup_11p16g[] = {
	{0x2f, 0x74},
	{0x61, 0xb7},
	{0x63, 0xb7},
	{0x60, 0xcc},
	{0x62, 0xcc},
	{0x64, 0x77},
	/* Power off the DFE and bypass the CDR */
	{0x09, 0x20},
	{0x1e, 0x09},
	/* Disabling fast capacitor search per recommendation in 7.5.10, rev G*/
	{0x3f, 0x80},
	/* Set swing to 0.8 Vpkpk */
	{0x2d, 0x82},
	/* Freeze retimer CTLE settings to 0 per 7.5.8 */
	{0x31, 0x00},
	{0x3a, 0x00},
	{0x03, 0x00},
	{0x40, 0x00},
};

struct i2c_freq_setting {
	const char *name;
	const struct i2c_table *sequence;
	size_t seq_size;
};

const struct i2c_freq_setting i2c_freq_settings[] = {
	{
		.name = "6Gbps",
		.sequence = setup_6g,
		.seq_size = ARRAY_SIZE(setup_6g),
	},
	{
		.name = "9.96Gbps",
		.sequence = setup_9p96g,
		.seq_size = ARRAY_SIZE(setup_9p96g),
	},
	{
		.name = "11.04Gbps",
		.sequence = setup_11p04g,
		.seq_size = ARRAY_SIZE(setup_11p04g),
	},
	{
		.name = "11.16Gbps",
		.sequence = setup_11p16g,
		.seq_size = ARRAY_SIZE(setup_11p16g),
	}
};

/**
 * struct ds125_retimer- The driver tracking structure.
 * @i2c_dev:	The backing i2c device.
 * @dev:	The backing Linux device.
 * @mutex:	Mutex to allow setting of channel followed by access.
 */
struct ds125_retimer {
	struct i2c_client *i2c_dev;
	struct device *dev;

	struct mutex mutex;
};

/**
 * ds125_write_register() - Perform a register write.
 * NOTE: Lock must be held.
 * @ds125_dev:	Device to write to.
 * @addr:	Register address to write.
 * @data:	Register value to write.
 *
 * @return	0 on success, <0 on failure.
 */
static int ds125_write_register(struct ds125_retimer *ds125_dev,
				u8 addr, uint8_t data)
{
	return i2c_smbus_write_byte_data(ds125_dev->i2c_dev, addr, data);
}

/**
 * ds125_read_chan_register() - Perform a register read.
 * @ds125_dev:	Device to read from.
 * @chan:	Channel to read from.
 * @addr:	Register address to read.
 * @data[out]:	Buffer to read into.
 *
 * @return	0 on success, <0 on failure.
 */
static int ds125_read_chan_register(struct ds125_retimer *ds125_dev, u8 chan,
				    u8 addr, uint8_t *data)
{
	s32 ret = 0;

	mutex_lock(&ds125_dev->mutex);
	ret = ds125_write_register(ds125_dev, DS125_REG_CHAN_SELECT,
				   chan);
	if (ret == 0) {
		ret = i2c_smbus_read_byte_data(ds125_dev->i2c_dev, addr);
	}

	mutex_unlock(&ds125_dev->mutex);

	*data = (uint8_t)ret;

	return ret;
}

/**
 * ds125_i2c_query_device() - Perform a register read of the device register and
 *	compare it to known good value.
 * @ds125_dev:	Device to query.
 *
 * @return	0 on success, <0 on failure.
 */
static int ds125_i2c_query_device(struct ds125_retimer *ds125_dev)
{
	u8 device_id;
	int ret;

	ret = ds125_read_chan_register(ds125_dev, DS125_REG_CHAN_SELECT_GLOB,
				       DS125_REG_DEVID, &device_id);
	if (ret < 0)
		return ret;

	if ((device_id & DS125_REG_DEVID_DEV_MASK) != DS125_REG_DEVID_DEV_VAL)
		return -ENODEV;

	return 0;
}

/**
 * ds125_configure_channel() - Configures a ds125 retimer channel.
 * @ds125_dev:	Device to configure.
 * @channel:	Channel number to configure.
 * @settings:	Channel settings to apply.
 *
 * @return	0 on success, <0 on failure.
 */
static int ds125_configure_channel(struct ds125_retimer *ds125_dev, int channel,
				   const struct i2c_freq_setting *settings)
{
	int ret;
	int i;

	dev_dbg(ds125_dev->dev, "Configuring channel %d to %s\n",
		channel, settings->name);

	mutex_lock(&ds125_dev->mutex);

	/* Set to write to single channel */
	ret = ds125_write_register(ds125_dev, 0xff, 0x04 + channel);
	if (ret < 0) {
		goto exit;
	}

	/* Reset channel registers to default */
	ret = ds125_write_register(ds125_dev, 0x00, 0x7f);
	if (ret < 0) {
		goto exit;
	}
	ret = ds125_write_register(ds125_dev, 0x00, 0x00);
	if (ret < 0) {
		goto exit;
	}

	/* Assert CDR reset */
	ret = ds125_write_register(ds125_dev, 0x0a, 0x1c);
	if (ret < 0) {
		goto exit;
	}

	/* Use 25MHz ref clock for coarse VCO calibration */
	ret = ds125_write_register(ds125_dev, 0x36, 0x31);
	if (ret < 0) {
		goto exit;
	}

	for (i = 0; i < settings->seq_size; i++) {
		ret = ds125_write_register(ds125_dev,
					   settings->sequence[i].address,
			settings->sequence[i].data);
		if (ret < 0) {
			goto exit;
		}
	}

	/* Deassert CDR Reset */
	ret = ds125_write_register(ds125_dev, 0x0a, 0x10);
	if (ret < 0) {
		goto exit;
	}

exit:
	mutex_unlock(&ds125_dev->mutex);
	return ret;
}

/**
 * ds125_sysfs_configure_chan() - sysfs interface to configure chan.
 * @dev:	Linux object write is performed against.
 * @chan:	Channel this was applied against.
 * @buf:	User buf containing configuration information
 * @count:	Size of user data.
 *
 * @return	count on success, <0 on failure.
 */
static ssize_t ds125_sysfs_configure_chan(struct device *dev,
					  u8 chan,
					   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds125_retimer *ds125_dev = i2c_get_clientdata(client);
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(i2c_freq_settings); i++) {
		if (!strcmp(buf, i2c_freq_settings[i].name)) {
			ret = ds125_configure_channel(ds125_dev, chan,
						      &i2c_freq_settings[i]);
			return ret ? ret : count;
		}
	}

	dev_err(ds125_dev->dev, "Invalid configuration '%s'\n", buf);

	return -EINVAL;
}

/**
 * ds125_sysfs_read_reg() - Perform a register read and return to user.
 * @dev:	Linux object write is performed against.
 * @chan:	Channel select (register 0xff) value to apply against.
 * @reg:	Register to read.
 * @buf:	User buf to write to.
 *
 * @return	Number of bytes written on success, <0 on failure.
 */
static ssize_t ds125_sysfs_read_reg(struct device *dev,
					 u8 chan,
					 u8 reg,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds125_retimer *ds125_dev = i2c_get_clientdata(client);
	int ret;
	u8 reg_val;

	ret = ds125_read_chan_register(ds125_dev, chan, reg, &reg_val);
	if (ret < 0) {
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", reg_val);
}

/**
 * channel sysfs macros and attributes for DS125.
 */
#define DS125_CHAN_REG_SYSFS_ENTRY(__chan, __name, __reg) \
static ssize_t ds125_sysfs_read_chan_##__chan##_##__name(struct device *dev, \
						struct device_attribute *attr, \
						char *buf) \
{ \
	return ds125_sysfs_read_reg(dev, DS125_CHAN(__chan), __reg, buf); \
} \
static struct device_attribute dev_attr_chan_##__chan##_##__name = \
			__ATTR(__name, 0444, \
			       ds125_sysfs_read_chan_##__chan##_##__name, \
			       NULL)

#define DS125_CHAN_CONFIG(__chan) \
static ssize_t ds125_configure_chan_##__chan(struct device *dev, \
					       struct device_attribute *attr, \
					       const char *buf, size_t count) \
{ \
	return ds125_sysfs_configure_chan(dev, __chan, buf, count); \
} \
static struct device_attribute dev_attr_chan_##__chan##_configure = \
			__ATTR(configure, 0220, \
			       NULL, \
			       ds125_configure_chan_##__chan)

#define DS125_CHAN_SYSFS_GROUP(__chan) \
DS125_CHAN_CONFIG(__chan); \
DS125_CHAN_REG_SYSFS_ENTRY(__chan, cdr_status, DS125_CHAN_REG_CDR_STATUS); \
DS125_CHAN_REG_SYSFS_ENTRY(__chan, horizontal_eye, DS125_CHAN_REG_HEO); \
DS125_CHAN_REG_SYSFS_ENTRY(__chan, vertical_eye, DS125_CHAN_REG_VEO); \
DS125_CHAN_REG_SYSFS_ENTRY(__chan, vertical_eye_range, DS125_CHAN_REG_EOM_VRANGE); \
DS125_CHAN_REG_SYSFS_ENTRY(__chan, eq_boost, DS125_CHAN_REG_EQ_BST); \
static struct attribute *ds125_chan_##__chan##_device_attrs[] = { \
	&dev_attr_chan_##__chan##_cdr_status.attr, \
	&dev_attr_chan_##__chan##_configure.attr, \
	&dev_attr_chan_##__chan##_horizontal_eye.attr, \
	&dev_attr_chan_##__chan##_vertical_eye.attr, \
	&dev_attr_chan_##__chan##_vertical_eye_range.attr, \
	&dev_attr_chan_##__chan##_eq_boost.attr, \
	/* Sentinel */ \
	NULL, \
}; \
static const struct attribute_group ds125_chan_##__chan##_sysfs_regs_group = { \
	.name = "channel" #__chan, .attrs = ds125_chan_##__chan##_device_attrs, \
}

DS125_CHAN_SYSFS_GROUP(0);
DS125_CHAN_SYSFS_GROUP(1);
DS125_CHAN_SYSFS_GROUP(2);
DS125_CHAN_SYSFS_GROUP(3);

/**
 * global sysfs macros and attributes for DS125.
 */
#define DS125_GLOBAL_REG_SYSFS_ENTRY(__name, __reg) \
static ssize_t ds125_sysfs_read_global_##__name(struct device *dev, \
						struct device_attribute *attr, \
						char *buf) \
{ \
	return ds125_sysfs_read_reg(dev, DS125_REG_CHAN_SELECT_GLOB, __reg, buf); \
} \
static struct device_attribute dev_attr_global_##__name = __ATTR(name, 0444, \
					ds125_sysfs_read_global_##__name, \
					NULL)

DS125_GLOBAL_REG_SYSFS_ENTRY(deviceid, DS125_REG_DEVID);

static struct attribute *ds125_global_device_attrs[] = {
	&dev_attr_global_deviceid.attr,
	/* Sentinel */
	NULL,
};

static const struct attribute_group ds125_global_sysfs_regs_group = {
	.name = "global", .attrs = ds125_global_device_attrs,
};

static const struct attribute_group *ds125_global_sysfs_groups[] = {
	&ds125_global_sysfs_regs_group,
	&ds125_chan_0_sysfs_regs_group,
	&ds125_chan_1_sysfs_regs_group,
	&ds125_chan_2_sysfs_regs_group,
	&ds125_chan_3_sysfs_regs_group,
	NULL,
};

/**
 * ds125_i2c_probe() - Drive probe function.
 * @client:	The i2c client being probed.
 * @id:		The i2c device id being probed.
 *
 * @return	0 on success, <0 on failure.
 */
static int ds125_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ds125_retimer *ds125_dev;
	int ret = 0;

	ds125_dev = devm_kzalloc(&client->dev, sizeof(*ds125_dev), GFP_KERNEL);
	if (!ds125_dev)
		return -ENOMEM;

	ds125_dev->i2c_dev = client;
	ds125_dev->dev = &ds125_dev->i2c_dev->dev;

	i2c_set_clientdata(client, ds125_dev);

	mutex_init(&ds125_dev->mutex);

	ret = ds125_i2c_query_device(ds125_dev);
	if (ret < 0)
		goto cleanup;

	ret = sysfs_create_groups(&ds125_dev->dev->kobj,
				  ds125_global_sysfs_groups);
	if (ret) {
		dev_err(ds125_dev->dev, "Failed creating reg sysfs (%d)\n",
			ret);
		goto cleanup;
	}

	dev_info(ds125_dev->dev, "DS125 device successfully queried\n");

	return 0;

cleanup:
	mutex_destroy(&ds125_dev->mutex);

	return ret;
}

/**
 * ds125_i2c_remove() - Drive removal function.
 * @client:	The i2c client being removed.
 *
 * @return	0 on success, <0 on failure.
 */
static int ds125_i2c_remove(struct i2c_client *client)
{
	struct ds125_retimer *ds125_dev = i2c_get_clientdata(client);

	sysfs_remove_groups(&ds125_dev->dev->kobj, ds125_global_sysfs_groups);

	mutex_destroy(&ds125_dev->mutex);

	return 0;
}

/**
 * i2c device table
 */
static struct i2c_device_id ds125_i2c_id_table[] = {
	{DS125_I2C_DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ds125_i2c_id_table);

/**
 * ds125 i2c compatible strings.
 */
static const struct of_device_id of_ds125_i2c_match[] = {
	{ .compatible = "sx,ti-ds125-i2c", },
	{}
};
MODULE_DEVICE_TABLE(of, of_ds125_i2c_match);

/**
 * ds125 i2c driver configuration.
 */
static struct i2c_driver ds125_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DS125_I2C_DRIVER_NAME,
		.of_match_table = of_match_ptr(of_ds125_i2c_match),
	},
	.probe = ds125_i2c_probe,
	.remove = ds125_i2c_remove,
	.id_table = ds125_i2c_id_table,
};

module_i2c_driver(ds125_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C driver for DS125 Retimers");
MODULE_AUTHOR("Kevin Bosien <kevin.bosien@spacex.com>");
