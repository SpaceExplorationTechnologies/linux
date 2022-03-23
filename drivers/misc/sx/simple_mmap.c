/**
 * This is the main source file for the SpaceX simple mmap driver.
 *
 * This adds support for the local and remote Personality Module,
 * or PM, cores present on many SpaceX FPGA builds.  Such cores provide
 * Remote Input/Output (RIO) functionality.
 *
 * This driver exports the memory regions from those cores as sysfs files.
 */

#include "simple_mmap.h"

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>

/**
 * This is our driver name.
 */
#define DRIVER_NAME "sx-simple-mmap"

/**
 * The "compatible" identifier in the device tree for this driver.
 */
#define COMPATIBLE_ID "sx,simple-mmap"

/**
 * smmap_init_region() - initializes a struct smmap_region from the device tree
 *
 * @priv:	private data structure; may not be fully initialized, though
 *		struct platform_device and struct device back-pointers should
 *		be set
 * @region:	pointer to the struct smmap_region to be initialized
 * @node:	pointer to device tree node
 */
static int smmap_init_region(struct smmap_priv *priv,
		struct smmap_region *region, struct device_node *node)
{
	const __be32 *reg = NULL;
	int len = 0;
	int rc = 0;

	region->label = of_get_property(node, "label", &len);
	if (!region->label)
		region->label = of_get_property(node, "name", &len);

	reg = of_get_property(node, "reg", &len);
	if (!reg || (len != sizeof(*reg) * 2)) {
		dev_warn(priv->dev, "No 'reg' property specified, or wrong "
			 "format, for '%s'!\n", region->label);
		return -EINVAL;
	}

	rc = of_address_to_resource(node, 0, &region->resource);
	if (rc != 0) {
		dev_warn(priv->dev, "Could not parse I/O memory "
			 "resource for '%s'\n", region->label);
		return rc;
	}
	if (resource_type(&region->resource) != IORESOURCE_MEM) {
		dev_warn(priv->dev, "Did not parse memory as the first "
			 "resource in the device tree for '%s'\n",
			 region->label);
		return -EINVAL;
	}
	dev_dbg(priv->dev, "%s\n", region->label);

	/* We can't use sx_init_mem_regions() here because the regs
	 * fields are split up. So we do the mapping ourselves. */
	/* First, we build a unique identifier for this memory region. */
	scnprintf(region->name, sizeof(region->name),
		  "%s_%s", region->label, priv->pdev->name);
	if (!request_mem_region(region->resource.start,
				resource_size(&region->resource),
				region->name)) {
		dev_err(priv->dev, "Could not map memory region '%s'!",
			region->label);
		return -ENOMEM;
	}
	region->remap_start = round_down(region->resource.start, PAGE_SIZE);
	region->remap_size = round_up(resource_size(&region->resource) +
					(region->resource.start -
						region->remap_start),
				      PAGE_SIZE);

	/* Start and end the remapped region on page boundaries. */
	region->regs = ioremap(region->remap_start, region->remap_size);
	if (NULL == region->regs) {
		dev_err(priv->dev, "ioremap() failed for '%s' "
			"region!\n", region->label);
		release_mem_region(region->resource.start,
				   resource_size(&region->resource));
		return -EACCES;
	}
	dev_dbg(priv->dev, "Successfully mapped '%s' region to %p\n",
		region->label, region->regs);

	/* The bin attributes are initialized and created in
         * smmap_create_device_files(). */

	return 0;
}

/**
 * smmap_deinit_region() - deinitializes a struct smmap_region
 *
 * @priv:	private data structure; may not be fully initialized, though
 *		struct platform_device and struct device back-pointers should
 *		be set
 * @region:	pointer to the struct smmap_region to be deinitialized
 */
static void
smmap_deinit_region(struct smmap_priv *priv, struct smmap_region *region)
{
	iounmap(region->regs);
	release_mem_region(region->resource.start,
			   resource_size(&region->resource));
}

/**
 * smmap_deinit_regions() - deinitializes the first @n struct smmap_regions
 *
 * @priv:	private data structure; may not be fully initialized, though
 *		struct platform_device and struct device back-pointers should
 *		be set
 * @n:		number of @priv's struct smmap_regions to deinitialize
 */
static void
smmap_deinit_regions(struct smmap_priv *priv, unsigned n)
{
	unsigned i = 0;

	for (i = 0; i < n; i++)
		smmap_deinit_region(priv, &priv->regions[i]);
}

/**
 * smmap_init_data() - allocates and inits memory for mmap()ing core
 *
 * @dev:	our platform_device
 *
 * This function returns the newly-allocated and initialized data, or an
 * ERR_PTR on error.
 *
 * The format of the device tree (parsed herein) resembles the MTD partition
 * layout.  This is by design; see parse_ofpart_partitions() for the
 * inspiration.
 */
static struct smmap_priv *smmap_init_data(struct platform_device *dev)
{
	struct smmap_priv *priv = NULL;
	void *err = ERR_PTR(-EIO);
	unsigned i = 0;
	struct device_node *pp = NULL;

	BUG_ON(!dev);

	dev_dbg(&dev->dev, "Initializing data...\n");
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (NULL == priv) {
		dev_err(&dev->dev, "kzalloc failed\n");
		err = ERR_PTR(-ENOMEM);
		goto out;
	}

	/* set the back-references */
	priv->pdev = dev;
	priv->dev = &dev->dev;

	/* get the memory type (cached/uncached) of the node */
	priv->is_cached = of_property_read_bool(dev->dev.of_node,
						"is-cached");

	/* get the memory type (cached/uncached) of the node */
	priv->no_dma = of_property_read_bool(dev->dev.of_node, "no-dma");

	/* count the number of regions to export, based on the number of
	 * subnodes */
	pp = NULL;
	priv->num_regions = 0;
	while ((pp = of_get_next_child(dev->dev.of_node, pp)))
		priv->num_regions++;

	dev_dbg(&dev->dev, "There are %u regions to export for mmap:\n",
		priv->num_regions);
	/* allocate memory for the regions array */
	priv->regions = kzalloc(sizeof(*priv->regions) * priv->num_regions,
				GFP_KERNEL);
	if (NULL == priv->regions) {
		dev_err(&dev->dev,
			"kzalloc failed for mmap region array memory\n");
		err = ERR_PTR(-ENOMEM);
		goto out_kfree;
	}

	/* place string pointers into 'names' array and the offsets+sizes
	 * into 'mem' array */
	pp = NULL;
	i = 0;
	while ((pp = of_get_next_child(dev->dev.of_node, pp))) {
		int rc = smmap_init_region(priv, &priv->regions[i], pp);
		if (rc != 0)
			break;
		i++;
	}

	if (i < priv->num_regions) {
		dev_info(&dev->dev, "Cleaning up after error...\n");
		smmap_deinit_regions(priv, i);
		err = ERR_PTR(-EIO);
		goto out_kfree;
	}

	/* link the platform_device to this private data */
	platform_set_drvdata(dev, priv);

	return priv;

out_kfree:
	/* because priv was kzalloc()ed, it is OK to always kfree its internal
	 * pointers to dynamically-allocated memory here (kfree(NULL) returns
	 * immediately) */
	kfree(priv->regions);
	kfree(priv);
out:
	return err;
}

/**
 * smmap_deinit_data() - frees the memory for the mmap()able device
 *
 * @dev:	our platform device with private data
 */
static void smmap_deinit_data(struct platform_device *dev)
{
	struct smmap_priv *priv = NULL;

	BUG_ON(!dev);
	priv = platform_get_drvdata(dev);
	BUG_ON(!priv);

	platform_set_drvdata(dev, NULL);

	dev_dbg(&dev->dev, "Deinitializing data...\n");
	smmap_deinit_regions(priv, priv->num_regions);

	kfree(priv->regions);
	kfree(priv);
}

/**
 * smmap_probe() - probe function for the SpaceX simple mmap driver
 *
 * @dev:	pointer to the platform device
 *
 * This is called when the kernel wants us to attach to a device.
 * It returns 0 if the device was claimed.
 */
static int smmap_probe(struct platform_device *dev)
{
	int rc = 0;
	struct smmap_priv *priv = NULL;

	BUG_ON(!dev);

	dev_info(&dev->dev, "Probing\n");

	if (!of_device_is_compatible(dev->dev.of_node, COMPATIBLE_ID)) {
		/* This should not happen; the kernel should call us because the
		 * device tree says the device is compatible. */
		dev_warn(&dev->dev, "Incompatible device!\n");
		return -ENODEV;
	}

	/* Configure from the device tree. */
	priv = smmap_init_data(dev);
	if (IS_ERR(priv)) {
		return PTR_ERR(priv);
	}

	/* Register with the DMA engine. */
	if (!priv->no_dma)
		dmaengine_get();

	/* Create sysfs interfaces. */
	rc = smmap_create_device_files(priv);
	if (rc != 0) {
		smmap_deinit_data(dev);
		return rc;
        }

	return 0;
};

/**
 * smmap_remove() - called when kernel wants us to unbind from the device
 *
 * @dev:	the device to unbind from
 *
 * This function returns 0 on success.
 */
static int smmap_remove(struct platform_device *dev)
{
	struct smmap_priv *priv = NULL;

	BUG_ON(!dev);
	priv = platform_get_drvdata(dev);
	BUG_ON(!priv);

	smmap_remove_device_files(priv);
	smmap_deinit_data(dev);

	/* Unregister with the DMA engine. */
	if (!priv->no_dma)
		dmaengine_put();

	dev_info(&dev->dev, "Removed\n");

	return 0;
}

/*
 * Here we list which devices we are compatible with.
 */
static const struct of_device_id smmap_ids[] = {
	{
		.compatible = COMPATIBLE_ID,
	},
	{},
};
/*
 * Now we register the compatibility information with the kernel.
 */
MODULE_DEVICE_TABLE(of, smmap_ids);

/*
 * Here we define the driver itself.
 */
static struct platform_driver smmap_driver = {
	.driver	= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= smmap_ids,
	},
	.probe	= smmap_probe,
	.remove = smmap_remove,
};

/**
 * smmap_init() - initializes the driver
 *
 * This function returns 0 on success or an error code on failure.
 */
static int __init smmap_init(void)
{
	int rc = 0;

	/* This makes the driver real and eligible for devices to attach. */
	rc = platform_driver_register(&smmap_driver);

	return rc;
}
module_init(smmap_init);

/**
 * smmap_exit() - removes the driver
 */
static void __exit smmap_exit(void)
{
	platform_driver_unregister(&smmap_driver);
}
module_exit(smmap_exit);

/*
 * Stylish module documentation
 */
MODULE_AUTHOR("Jim Gruen <jgruen@spacex.com>");
MODULE_DESCRIPTION("SpaceX Simple mmap() Driver");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("Proprietary");
