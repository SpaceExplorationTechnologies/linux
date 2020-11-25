/**
 * sx_catson_mdml_monitor.c
 *
 * This driver monitors the state of the modemlink and attempts a recovery
 * by strobing miphy if necessary.  It also aggregates statistical information.
 *
 * @author Kevin Bosien <kbosien@spacex.com>
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <linux/time.h>

#define MODEMLINK_POLL_RATE	HZ  /* Poll at 1HZ */
#define NAME_LEN 32
#define MEMORY_REGION_NAME "mdml_monitor"

/**
 * struct modemlink_registers - A structure based representation of Aurora
 * registers.
 */
struct modemlink_registers {
	u32 ctrl;
#define AURORA_STATUS_LANE_UP		BIT(0)
#define AURORA_STATUS_CHANNEL_UP	BIT(1)
#define AURORA_STATUS_UP		(AURORA_STATUS_LANE_UP | AURORA_STATUS_CHANNEL_UP)
#define AURORA_STATUS_SYS_RST_OUT	BIT(2)
#define AURORA_STATUS_SOFT_ERR		BIT(4)
#define AURORA_STATUS_HARD_ERR		BIT(5)
#define AURORA_STATUS_CRC_ERR		BIT(6)
	u32 status;
	u32 hard_err_cnt;
	u32 soft_err_cnt;
	u32 crc_err_cnt;
} __packed__;

/**
 * struct modemlink_dev - Control structure for this driver.
 * @dev:		The backing platform device..
 * @regs:		The registers associated with this modemlink.
 * @resource:		The resource associated with this modemlink.
 * @status:		Shadow copy of the aurora status register.
 * @link_up:		Whether the modemlink is currently up.
 * @soft_err_events:	The number of times a soft error was detected.
 * @hard_err_events:	The number of times a hard error was detected.
 * @crc_err_events:	The number of times a crc error was detected.
 * @link_down_events:	The number of times a modemlmink link_up transitioned to 0.
 * @miphy_resets:	The number of times this driver has attempted to reset the miphy.
 * @control_resets:   The number of times reset was attempted on the miphy
 *                    through the sysfs interface.
 * @heartbeats:		The number of times work has been run.
 * @miphy:		The miphy associated with this modemlink.
 * @work:		The work which periodically runs.
 */
struct modemlink_dev {
	struct platform_device *dev;

	struct modemlink_registers __iomem *regs;
	struct resource *resource;

	u32 status;
	unsigned long link_up;

	unsigned long soft_err_events;
	unsigned long hard_err_events;
	unsigned long crc_err_events;
	unsigned long link_down_events;
	unsigned long miphy_resets;
	unsigned long control_resets;

	unsigned long heartbeats;

	struct phy *miphy;

	struct delayed_work work;
};

static ssize_t sx_catson_miphy_reset(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	int err;
	u32 value;
	struct modemlink_dev *mdev = dev_get_drvdata(dev);

	if (sscanf(buf, "%u\n", &value) != 1)
		return -EINVAL;

	/* Writing zero does nothing. Any other value triggers a reset. */
	if (value == 0)
		return count;

	err = phy_reset(mdev->miphy);
	mdev->miphy_resets++;
	mdev->control_resets++;

	if (mdev->link_up) {
		mdev->link_down_events++;
		mdev->link_up = 0;
	}

	if (err)
		return err;

	return count;
}

static struct device_attribute dev_attr_control_reset =
	__ATTR(reset, 0220, NULL, sx_catson_miphy_reset);

/**
 * catson_mdml_hb() - The periodic function that monitors modemlink.
 * @work:	The work context being run.
 */
static void catson_mdml_hb(struct work_struct *work)
{
	struct modemlink_dev *mdev = container_of(work, struct modemlink_dev,
						  work.work);
	mdev->status = readl(&mdev->regs->status);

	if ((mdev->status & AURORA_STATUS_UP) != AURORA_STATUS_UP) {
		phy_reset(mdev->miphy);
		mdev->miphy_resets++;

		if (mdev->link_up) {
			mdev->link_down_events++;
			mdev->link_up = 0;
		}
	} else {
		mdev->link_up = 1;
	}

	if (mdev->status & AURORA_STATUS_SOFT_ERR)
		mdev->soft_err_events++;
	if (mdev->status & AURORA_STATUS_HARD_ERR)
		mdev->hard_err_events++;
	if (mdev->status & AURORA_STATUS_CRC_ERR)
		mdev->crc_err_events++;

	mdev->heartbeats++;

	schedule_delayed_work(&mdev->work, MODEMLINK_POLL_RATE);
}

/* The sysfs functions for displaying stats */
#define MDML_MONITOR_SHOW_STAT_SYSFS(name) \
static ssize_t mdml_monitor_show_##name(struct device *dev, \
					struct device_attribute *attr, \
					char *buf) \
{ \
	struct modemlink_dev *mdev = dev_get_drvdata(dev); \
	return scnprintf(buf, PAGE_SIZE, "%lu\n", (unsigned long)mdev->name); \
} \
static DEVICE_ATTR(name, 0444, mdml_monitor_show_##name, NULL)

MDML_MONITOR_SHOW_STAT_SYSFS(status);
MDML_MONITOR_SHOW_STAT_SYSFS(link_up);
MDML_MONITOR_SHOW_STAT_SYSFS(soft_err_events);
MDML_MONITOR_SHOW_STAT_SYSFS(hard_err_events);
MDML_MONITOR_SHOW_STAT_SYSFS(crc_err_events);
MDML_MONITOR_SHOW_STAT_SYSFS(link_down_events);
MDML_MONITOR_SHOW_STAT_SYSFS(miphy_resets);
MDML_MONITOR_SHOW_STAT_SYSFS(control_resets);
MDML_MONITOR_SHOW_STAT_SYSFS(heartbeats);

/**
 * All of our sysfs attributes in one place.
 */
static struct attribute *mdml_monitor_device_attrs[] = {
	/* Status */
	&dev_attr_status.attr,
	&dev_attr_link_up.attr,

	/* Stats */
	&dev_attr_soft_err_events.attr,
	&dev_attr_hard_err_events.attr,
	&dev_attr_crc_err_events.attr,
	&dev_attr_link_down_events.attr,

	/* Driver State */
	&dev_attr_miphy_resets.attr,
	&dev_attr_control_resets.attr,
	&dev_attr_heartbeats.attr,

	/* Control */
	&dev_attr_control_reset.attr,

	/* Sentinel */
	NULL,
};

static const struct attribute_group mdml_monitor_sysfs_regs_group = {
	.attrs = mdml_monitor_device_attrs,
};

/**
 * sx_catson_modemlink_probe() - The probe function for this driver.
 * @dev:	The backing platform device.
 *
 * @return	0 on success, <0 on failure.
 */
static int sx_catson_modemlink_probe(struct platform_device *dev)
{
	struct modemlink_dev *mdev = NULL;
	int rc;

	mdev = devm_kzalloc(&dev->dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev) {
		return -ENOMEM;
	}

	platform_set_drvdata(dev, mdev);
	mdev->dev = dev;

	mdev->resource = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!mdev->resource) {
		dev_err(&dev->dev, "No memory region specified for %s!\n",
			MEMORY_REGION_NAME);
		return -ENODEV;
	}

	if (!devm_request_mem_region(&dev->dev, mdev->resource->start,
				     resource_size(mdev->resource),
				     MEMORY_REGION_NAME)) {
		dev_err(&dev->dev, "Could not map memory region %s!\n",
			MEMORY_REGION_NAME);
		return -ENODEV;
	}

	mdev->regs = devm_ioremap(&dev->dev, mdev->resource->start,
				  resource_size(mdev->resource));
	if (!mdev->regs) {
		dev_err(&dev->dev, "IO Remap failed for region %s!\n",
			MEMORY_REGION_NAME);
		return -EACCES;
	}

	mdev->miphy = devm_phy_get(&dev->dev, "miphy");
	if (!IS_ERR(mdev->miphy)) {
		phy_init(mdev->miphy);
	} else {
		/* Includes -EPROBE_DEFER */
		return PTR_ERR(mdev->miphy);
	}

	rc = sysfs_create_group(&dev->dev.kobj, &mdml_monitor_sysfs_regs_group);
	if (rc) {
		dev_err(&dev->dev, "Failed creating reg sysfs (%d)\n", rc);
		return rc;
	}

	INIT_DELAYED_WORK(&mdev->work, catson_mdml_hb);

	schedule_delayed_work(&mdev->work, 0);

	return 0;
}

/**
 * sx_catson_modemlink_remove() - The probe function for this driver.
 * @dev:	The backing platform device.
 *
 * @return	0 on success, <0 on failure.
 */
static int sx_catson_modemlink_remove(struct platform_device *dev)
{
	struct modemlink_dev *mdev = platform_get_drvdata(dev);

	cancel_delayed_work_sync(&mdev->work);

	sysfs_remove_group(&dev->dev.kobj, &mdml_monitor_sysfs_regs_group);

	release_mem_region(mdev->resource->start,
			   resource_size(mdev->resource));

	return 0;
}

/*
 * The compatible ids associated with this driver.
 */
static const struct of_device_id sx_catson_mdml_monitor_ids[] = {
	{
		.compatible = "sx,catson-mdml-monitor-1.00",
	},
	{},
};

/**
 * Now we register the compatibility information with the kernel.
 */
MODULE_DEVICE_TABLE(of, sx_catson_mdml_monitor_ids);

/**
 * Here we define the driver itself.
 */
static struct platform_driver sx_catson_mdml_monitor_driver = {
	.driver = {
		.name       = "sx-catson-mdml-monitor-1.00",
		.owner      = THIS_MODULE,
		.of_match_table = sx_catson_mdml_monitor_ids,
	},
	.probe  = sx_catson_modemlink_probe,
	.remove = sx_catson_modemlink_remove,
};

/**
 * And then we register the driver with the kernel.
 */
module_platform_driver(sx_catson_mdml_monitor_driver);

MODULE_AUTHOR("Kevin Bosien <kbosien@spacex.com>");
MODULE_DESCRIPTION("SpaceX Catson Modemlink Monitor");
MODULE_ALIAS("platform: sx_catson_mdml_monitor");
MODULE_LICENSE("Proprietary");
