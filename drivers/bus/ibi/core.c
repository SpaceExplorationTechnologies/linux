/*
 * Copyright (C) 2015 STMicroelectronics - All Rights Reserved
 * Author: Maxime Coquelin <maxime.coquelin@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "core.h"

#define CFG_NAME_MAX_LEN 32
#define STATE_NAME_MAX_LEN 32

static LIST_HEAD(ibidev_list);
DEFINE_MUTEX(ibidev_list_mutex);

static int ibi_apply_config(struct ibi_config *config);

/* Consumer functions */

/**
 * ibi_set_state - apply an IBI state
 * @ibi: a pointer to the ibi reference
 * @state: the IBI state to set
 */
int ibi_set_state(struct ibi *ibi, struct ibi_state *state)
{
	int i, ret = 0;

	if (IS_ERR_OR_NULL(ibi) || IS_ERR_OR_NULL(state))
		return 0;

	mutex_lock(&ibi->lock);
	for (i = 0; i < state->nconfigs; i++) {
		struct ibi_config *config = state->configs[i];

		mutex_lock(&config->ibidev->lock);
		ret = ibi_apply_config(config);
		mutex_unlock(&config->ibidev->lock);
		if (ret) {
			dev_err(ibi->dev,
				"failed to apply IBI config %s (%d)\n",
				config->name, ret);
			break;
		}
	}
	mutex_unlock(&ibi->lock);

	return ret;
}
EXPORT_SYMBOL(ibi_set_state);

/**
 * ibi_lookup_state - lookup IBI state reference by its name
 * @ibi: a pointer to the ibi reference
 * @name: the name of the state
 *
 * This function should be used to retrieve IBI state from its associated name
 * in DT node.
 */
struct ibi_state *ibi_lookup_state(struct ibi *ibi, const char *name)
{
	struct ibi_state *state;

	if (IS_ERR_OR_NULL(ibi))
		return NULL;

	mutex_lock(&ibi->lock);
	list_for_each_entry(state, &ibi->states, node) {
		if (!strncmp(state->name, name, STATE_NAME_MAX_LEN)) {
			mutex_unlock(&ibi->lock);
			return state;
		}
	}
	mutex_unlock(&ibi->lock);

	dev_err(ibi->dev, "no IBI state named %s found\n", name);

	return NULL;
}
EXPORT_SYMBOL(ibi_lookup_state);

static struct ibi_dev *get_ibi_dev_from_node(struct device_node *np)
{
	struct ibi_dev *ibidev;

	mutex_lock(&ibidev_list_mutex);
	list_for_each_entry(ibidev, &ibidev_list, node) {
		if (ibidev->dev->of_node == np) {
			mutex_unlock(&ibidev_list_mutex);
			return ibidev;
		}
	}
	mutex_unlock(&ibidev_list_mutex);

	return NULL;
}

static struct ibi_config *of_ibi_get_config_by_phandle(struct device_node *np)
{
	struct device_node *parent;
	struct ibi_dev *ibidev;
	struct ibi_config *config;

	parent = of_get_next_parent(np);
	if (!parent || of_node_is_root(parent))
		return NULL;

	ibidev = get_ibi_dev_from_node(parent);
	if (!ibidev)
		return NULL;

	mutex_lock(&ibidev->lock);
	list_for_each_entry(config, &ibidev->configs, node) {
		if (config->np == np) {
			mutex_unlock(&ibidev->lock);
			return config;
		}
	}
	mutex_unlock(&ibidev->lock);

	return NULL;
}

static struct ibi_state *of_ibi_state_parse_one(struct ibi *ibi, int state_nr)
{
	struct device *dev = ibi->dev;
	struct device_node *np = dev->of_node, *np_config;
	struct ibi_state *state;
	struct ibi_config *config, **cfgs;
	char propname[16];
	const char *statename;
	struct property *prop;
	phandle phandle;
	const __be32 *list;
	int size, i, ret;

	ret = of_property_read_string_index(np, "ibi-names",
					    state_nr, &statename);
	if (ret < 0) {
		dev_err(dev, "No associated name to ibi-%d\n", state_nr);
		return ERR_PTR(ret);
	}

	snprintf(propname, sizeof(propname), "ibi-%d", state_nr);
	prop = of_find_property(np, propname, &size);
	if (!prop) {
		dev_err(dev, "ibi-name=%s without a proper state definition\n",
			statename);
		return ERR_PTR(-ENODEV);
	}

	list = prop->value;
	size /= sizeof(*list);

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return ERR_PTR(-ENOMEM);

	state->ibi = ibi;
	state->name = statename;

	cfgs = kcalloc(size, sizeof(*cfgs), GFP_KERNEL);
	if (!cfgs) {
		ret = -ENOMEM;
		goto free_state;
	}

	/* For each config in state */
	for (i = 0; i < size; i++) {
		phandle = be32_to_cpup(list++);

		np_config = of_find_node_by_phandle(phandle);
		if (!np_config) {
			dev_err(dev, "IBI name %s index %d invalid phandle",
				statename, i);
			ret = -EINVAL;
			goto free_cfgs;
		}

		config = of_ibi_get_config_by_phandle(np_config);
		if (!config) {
			dev_err(dev, "Failed to get config from node %p\n",
				np_config);
			ret = -EINVAL;
			goto free_cfgs;
		}

		cfgs[i] = config;
	}

	for (i = 0; i < size; i++) {
		mutex_lock(&cfgs[i]->ibidev->lock);
		cfgs[i]->refcount++;
		mutex_unlock(&cfgs[i]->ibidev->lock);
	}

	state->configs = cfgs;
	state->nconfigs = size;

	return state;

free_cfgs:
	kfree(cfgs);
free_state:
	kfree(state);

	return ERR_PTR(ret);
}

static void ibi_state_release_one(struct ibi_state *state)
{
	int i;

	for (i = 0; i < state->nconfigs; i++) {
		struct ibi_config *config = state->configs[i];

		mutex_lock(&config->ibidev->lock);
		config->refcount--;
		mutex_unlock(&config->ibidev->lock);
	}

	kfree(state->configs);
	kfree(state);
}

static void ibi_state_release_all(struct ibi *ibi)
{
	struct ibi_state *state, *temp_state;

	mutex_lock(&ibi->lock);
	list_for_each_entry_safe(state, temp_state, &ibi->states, node) {
		list_del(&state->node);
		ibi_state_release_one(state);
	}
	mutex_unlock(&ibi->lock);
}

static int of_ibi_state_parse_all(struct ibi *ibi)
{
	struct device *dev = ibi->dev;
	struct device_node *np = dev->of_node;
	struct ibi_state *state;
	int i, nstates, ret = 0;

	nstates = of_property_count_strings(np, "ibi-names");
	if (nstates < 0)
		return nstates;

	for (i = 0; i < nstates; i++) {
		state = of_ibi_state_parse_one(ibi, i);
		if (IS_ERR(state)) {
			ret = PTR_ERR(state);
			goto fail;
		}

		mutex_lock(&ibi->lock);
		list_add_tail(&state->node, &ibi->states);
		mutex_unlock(&ibi->lock);
	}

	return 0;

fail:
	dev_err(dev, "%s FAILED in parsing nstates = %d\n", __func__, i);

	ibi_state_release_all(ibi);

	return ret;
}

/**
 * ibi_get - get the IBI reference
 * @dev: a pointer to the device
 */
struct ibi *ibi_get(struct device *dev)
{
	struct ibi *ibi;
	int ret;

	ibi = kzalloc(sizeof(*ibi), GFP_KERNEL);
	if (!ibi)
		return ERR_PTR(-ENOMEM);

	ibi->dev = dev;
	of_node_get(dev->of_node);
	INIT_LIST_HEAD(&ibi->states);
	mutex_init(&ibi->lock);

	ret = of_ibi_state_parse_all(ibi);
	if (ret) {
		dev_err(dev, "failed to parse IBI states (%d)\n", ret);
		of_node_put(dev->of_node);
		kfree(ibi);
		return ERR_PTR(ret);
	}

	return ibi;
}
EXPORT_SYMBOL(ibi_get);

/**
 * ibi_put - put the IBI reference
 * ibi - a pointer to the IBI reference
 */
void ibi_put(struct ibi *ibi)
{
	ibi_state_release_all(ibi);
	of_node_put(ibi->dev->of_node);
	kfree(ibi);
}
EXPORT_SYMBOL(ibi_put);

static void devm_ibi_release(struct device *dev, void *res)
{
	ibi_put(*(struct ibi **)res);
}

/**
 * devm_ibi_get - managed version of ibi_get
 * @dev: a pointer to the device
 */
struct ibi *devm_ibi_get(struct device *dev)
{
	struct ibi **ptr, *ibi;

	ptr = devres_alloc(devm_ibi_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	ibi = ibi_get(dev);
	if (!IS_ERR(ibi)) {
		*ptr = ibi;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return ibi;
}
EXPORT_SYMBOL(devm_ibi_get);

static int devm_ibi_match(struct device *dev, void *res, void *data)
{
	struct ibi **ibi = res;

	if (!ibi || !*ibi) {
		WARN_ON(!ibi || !*ibi);
		return 0;
	}
	return *ibi == data;
}

/**
 * devm_ibi_put - managed version of ibi_put, should normally not be called
 * @dev: a pointer to the device
 * @ibi: a pointer to the IBI reference
 */
void devm_ibi_put(struct device *dev, struct ibi *ibi)
{
	int ret;

	ret = devres_release(dev, devm_ibi_release, devm_ibi_match, ibi);

	WARN_ON(ret);
}
EXPORT_SYMBOL(devm_ibi_put);

/* Provider functions */
static int ibi_apply_config(struct ibi_config *config)
{
	struct ibi_dev *ibidev = config->ibidev;
	struct ibi_ops *ops = ibidev->ops;
	int ret = -1;

	if (!WARN_ON_ONCE(!ops->apply_config))
		ret = ops->apply_config(config);

	return ret;
}

static struct ibi_config *of_ibi_config_parse_one(struct ibi_dev *ibidev,
						  struct device_node *np)
{
	struct ibi_config *cfg;
	struct ibi_ops *ops = ibidev->ops;
	int ret;

	cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return ERR_PTR(-ENOMEM);

	cfg->ibidev = ibidev;
	cfg->name = np->full_name;
	cfg->np = of_node_get(np);

	ret = ops->of_parse_config(np, cfg);
	if (ret) {
		of_node_put(cfg->np);
		kfree(cfg);
		return ERR_PTR(ret);
	}

	return cfg;
}

static void ibi_config_release_all(struct ibi_dev *ibidev)
{
	struct ibi_config *config, *temp_cfg;

	mutex_lock(&ibidev->lock);
	list_for_each_entry_safe(config, temp_cfg, &ibidev->configs, node) {
		/*
		 * Reliability of refcount is uncertain so let we
		 * keep a system warning here (not necessary to panic)
		 */
		of_node_put(config->np);
		WARN_ON(config->refcount);
		list_del(&config->node);
		kfree(config);
	}
	mutex_unlock(&ibidev->lock);
}

static int of_ibi_config_parse_all(struct ibi_dev *ibidev)
{
	struct device *dev = ibidev->dev;
	struct device_node *child, *np = dev->of_node;
	struct ibi_config *cfg;
	int ret = 0;

	for_each_child_of_node(np, child) {
		cfg = of_ibi_config_parse_one(ibidev, child);
		if (IS_ERR(cfg)) {
			dev_err(dev, "failed to parse config\n");
			ret = PTR_ERR(cfg);
			of_node_put(child);
			goto fail;
		}
		mutex_lock(&ibidev->lock);
		list_add_tail(&cfg->node, &ibidev->configs);
		mutex_unlock(&ibidev->lock);
	}

	if (list_empty(&ibidev->configs)) {
		dev_err(dev, "No configs provided in IBI device node\n");
		ret = -ENODEV;
	}

	return ret;
fail:
	ibi_config_release_all(ibidev);

	return ret;
}

/**
 * ibi_register - register an IBI device
 * @ibidesc: IBI driver registering information
 * @dev: the device entry for this IP Bus Interface
 * @driver_data: IBI driver's private data to be stored in ibi_dev
 */
struct ibi_dev *ibi_register(struct ibi_desc *ibidesc,
			     struct device *dev,
			     void *driver_data)
{
	struct ibi_dev *ibidev;
	int ret;

	if (!ibidesc || !ibidesc->ops || !ibidesc->ops->of_parse_config ||
	    !ibidesc->ops->apply_config)
		return ERR_PTR(-EINVAL);

	ibidev = kzalloc(sizeof(*ibidev), GFP_KERNEL);
	if (!ibidev)
		return ERR_PTR(-ENOMEM);

	ibidev->ops = ibidesc->ops;
	ibidev->dev = dev;
	of_node_get(dev->of_node);
	ibidev->driver_data = driver_data;
	mutex_init(&ibidev->lock);
	INIT_LIST_HEAD(&ibidev->configs);

	ret = of_ibi_config_parse_all(ibidev);
	if (ret) {
		dev_err(dev, "failed to parse configs (%d)\n", ret);
		goto err;
	}

	mutex_lock(&ibidev_list_mutex);
	list_add_tail(&ibidev->node, &ibidev_list);
	mutex_unlock(&ibidev_list_mutex);

#ifndef CONFIG_SPACEX
	dev_info(dev, "IP Bus Interface registered\n");
#else
	dev_dbg(dev, "IP Bus Interface registered\n");
#endif

	return ibidev;
err:
	of_node_put(dev->of_node);
	kfree(ibidev);
	return ERR_PTR(ret);
}

/**
 * ibi_unregister - unregister an IBI device
 * @ibidev - the IBI device instance
 */
void ibi_unregister(struct ibi_dev *ibidev)
{
	ibi_config_release_all(ibidev);
	of_node_put(ibidev->dev->of_node);
	kfree(ibidev);
}

#ifdef CONFIG_DEBUG_FS

static int ibi_configs_show(struct seq_file *s, void *what)
{
	struct ibi_dev *ibidev;
	struct ibi_config *cfg;

	seq_puts(s, "List of IBI configs\n");

	mutex_lock(&ibidev_list_mutex);
	list_for_each_entry(ibidev, &ibidev_list, node) {
		mutex_lock(&ibidev->lock);
		list_for_each_entry(cfg, &ibidev->configs, node) {
			seq_printf(s, "%s\n", cfg->name);
		}
		mutex_unlock(&ibidev->lock);
	}
	mutex_unlock(&ibidev_list_mutex);

	return 0;
}

static int ibi_configs_apply_help(struct seq_file *s, void *what)
{
	seq_puts(s, "echo all or the config name to apply it\n");

	return 0;
}

static ssize_t ibi_configs_apply(struct file *filp, const char __user *ubuf,
				 size_t size, loff_t *off)
{
	struct ibi_dev *ibidev;
	struct ibi_config *cfg;
	int filter_on;
	char buf[128] = {};

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, size)))
		return -EFAULT;

	/* If "all" is entered, then blindly apply all configs w/o filter */
	filter_on = strncmp(buf, "all", 3);

	mutex_lock(&ibidev_list_mutex);
	list_for_each_entry(ibidev, &ibidev_list, node) {
		mutex_lock(&ibidev->lock);
		list_for_each_entry(cfg, &ibidev->configs, node) {
			if (!filter_on || !strncmp(buf, cfg->name,
						   strlen(cfg->name))) {
				ibi_apply_config(cfg);
				pr_info("Apply:%s\n", cfg->name);
			}
		}
		mutex_unlock(&ibidev->lock);
	}
	mutex_unlock(&ibidev_list_mutex);

	return size;
}

static int ibi_configs_open(struct inode *inode, struct file *file)
{
	return single_open(file, ibi_configs_show, NULL);
}

static int ibi_configs_apply_open(struct inode *inode, struct file *file)
{
	return single_open(file, ibi_configs_apply_help, NULL);
}

static const struct file_operations ibi_configs_ops = {
	.open		= ibi_configs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations ibi_apply_ops = {
	.open		= ibi_configs_apply_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ibi_configs_apply,
};

static struct dentry *debugfs_root;

static void ibi_init_debugfs(void)
{
	debugfs_root = debugfs_create_dir("ibi", NULL);
	if (IS_ERR(debugfs_root) || !debugfs_root) {
		pr_warn("failed to create ibi debugfs directory\n");
		debugfs_root = NULL;
		return;
	}

	debugfs_create_file("configs", S_IFREG | 0444,
			    debugfs_root, NULL, &ibi_configs_ops);
	debugfs_create_file("apply-configs", 0644,
			    debugfs_root, NULL, &ibi_apply_ops);
}

#else /* CONFIG_DEBUG_FS */

static void ibi_init_debugfs(void)
{
}

#endif

static int __init ibi_init(void)
{
	ibi_init_debugfs();
	return 0;
}
postcore_initcall(ibi_init);
