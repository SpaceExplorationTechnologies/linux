/*
 * Copyright (C) 2015 STMicroelectronics - All Rights Reserved
 * Author: Maxime Coquelin <maxime.coquelin@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __LINUX_IBI_H
#define __LINUX_IBI_H

/**
 * struct ibi_state - IBI state
 * @node: node to include this state in the ibi list
 * @ibi: the IBI it belongs to
 * @name: the state name
 * @configs: reference table to the configs associated to this state
 * @nconfigs: number of configs
 */
struct ibi_state {
	struct list_head node;
	struct ibi *ibi;
	const char *name;
	struct ibi_config **configs;
	int nconfigs;
};

/**
 * struct ibi - IBI element
 * @dev: the consumer device reference
 * @states: list of available IBI states
 * @lock: mutex to protect the states
 */
struct ibi {
	struct device *dev;
	struct list_head states;
	struct mutex lock;
};

#ifdef CONFIG_IBI

struct ibi *ibi_get(struct device *dev);
void ibi_put(struct ibi *ibi);
struct ibi *devm_ibi_get(struct device *dev);
void devm_ibi_put(struct device *dev, struct ibi *ibi);
struct ibi_state *ibi_lookup_state(struct ibi *ibi, const char *name);
int ibi_set_state(struct ibi *ibi, struct ibi_state *state);

#else /* CONFIG_IBI */
#ifndef CONFIG_SPACEX  /* Compile instead of runtime error */

#include <linux/err.h>

/* Stubs if we're not using IBI framework */

struct ibi *ibi_get(struct device *dev)
{
	WARN_ONCE(1, "IBI framework is disabled");
	return NULL;
}

void ibi_put(struct ibi *ibi) {}

struct ibi *devm_ibi_get(struct device *dev)
{
	WARN_ONCE(1, "IBI framework is disabled");
	return NULL;
}

void devm_ibi_put(struct device *dev, struct ibi *ibi) {}

struct ibi_state *ibi_lookup_state(struct ibi *ibi, const char *name)
{
	return NULL;
}

int ibi_set_state(struct ibi *ibi, struct ibi_state *state)
{
	return 0;
}

#endif /* CONFIG_SPACEX */
#endif /* CONFIG_IBI */

#endif /* __LINUX_IBI_H */

