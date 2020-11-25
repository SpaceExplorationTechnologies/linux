/*
 * Copyright (C) 2015 STMicroelectronics - All Rights Reserved
 * Author: Maxime Coquelin <maxime.coquelin@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _IBI_CORE_H_
#define _IBI_CORE_H_

#include <linux/ibi.h>

/**
 * struct ibi_dev - IP bus interface device
 * @node: node to include this IBI device in the global list
 * @dev: the device entry for this IP bus interface
 * @driver_data: driver data associated to the device
 * @configs: the list of available configs
 * @ops: the IBI device callabcks
 */
struct ibi_dev {
	struct list_head node;
	struct device *dev;
	void *driver_data;
	struct list_head configs;
	struct ibi_ops *ops;
	struct mutex lock;
};

/**
 * struct ibi_config - IBI config information
 * @node: node to include this config in the ibi device
 * @name: the name of the config
 * @ibidev: the ibi device the config belongs to
 * @np: the config node pointer
 * @config_priv: private data for the config
 * @refcount: reference counter
 */
struct ibi_config {
	struct list_head node;
	const char *name;
	struct ibi_dev *ibidev;
	struct device_node *np;
	void *config_priv;
	unsigned int refcount;
};

/**
 * struct ibi_ops - IBI driver operations
 * @of_parse_config: parse the config node passed in argument (MANDATORY).
 *		     private data for the config can be saved in the
 *		     config_priv field of the struct ibi_config element.
 * @apply_config: apply a configuration to the interface (MANDATORY).
 */
struct ibi_ops {
	int (*of_parse_config)(struct device_node *np, struct ibi_config *cfg);
	int (*apply_config)(struct ibi_config *cfg);
};

/**
 * struct ibi_desc - IBI driver registering information
 * @ops: the IBI driver callabcks
 */
struct ibi_desc {
	struct ibi_ops *ops;
};

struct ibi_dev *ibi_register(struct ibi_desc *ibidesc,
			     struct device *dev,
			     void *driver_data);
void ibi_unregister(struct ibi_dev *ibidev);

#endif
