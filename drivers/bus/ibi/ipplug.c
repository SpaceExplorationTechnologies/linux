/*
 * IPPlug bridge driver using the IPI framework
 * Author: Pankaj Dev <pankaj.dev@st.com>
 *
 *
 * Copyright (C) 2015 STMicroelectronics (R&D) Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include "core.h"

#define MAX_CLIENTS 16
#define MAX_VCHANS 16

struct ibi_ipplug_client_cfg {
	unsigned char chunk_max_size;
	unsigned char opcode_max_size;
	unsigned char burst_max_size;
	unsigned char ivc, svc, wlru_ratio;
	unsigned int otr_pkts, dpreg;
	unsigned int dpreg_size;
};

struct ibi_ipplug_cfg {
	unsigned char page_size, node_enable;
	unsigned int wr_post;
	unsigned int bus_size, dpreg_total_size;
	struct ibi_ipplug_client_cfg *clients[MAX_CLIENTS];
};

enum regfield_g {
	MEM_PG_SIZE,
	NODE_EN,
	WR_POST,
	PROG_W,
	PROG_R,
	MAX_G_REGFIELDS,
};

enum regfield_client {
	MAX_CHK,
	MAX_OPC,
	MAX_BURST,
	OST_PKT,
	IVC,
	SVC,
	WLRU_RAT,
	DPREG,
	MAX_CLIENT_REGFIELDS,
};

/* G config members of ibi_ipplug_cfg */
#define MEM_PG_SIZE_CONFIG page_size
#define WR_POST_CONFIG wr_post
#define NODE_EN_CONFIG node_enable

/* C config members of ibi_ipplug_client_cfg */
#define MAX_CHK_CONFIG chunk_max_size
#define MAX_OPC_CONFIG opcode_max_size
#define MAX_BURST_CONFIG burst_max_size
#define OST_PKT_CONFIG otr_pkts
#define IVC_CONFIG ivc
#define SVC_CONFIG svc
#define WLRU_RAT_CONFIG wlru_ratio
#define DPREG_CONFIG dpreg

/* G config members 2^start index */
#define MEM_PG_SIZE_PWR2_START_INDEX 6

/* C config members 2^start index */
#define MAX_CHK_PWR2_START_INDEX 5
#define MAX_OPC_PWR2_START_INDEX 3
#define MAX_BURST_PWR2_START_INDEX 3
#define BWE_SAT_PWR2_START_INDEX 8
#define RATE_PWR2_START_INDEX 5
#define MAX_BWE_PWR2_START_INDEX 0

const char *dt_g_prop[MAX_G_REGFIELDS] = {
	"st,memory-page-size", /* MEM_PG_SIZE */
	"st,node-enable", /* NODE_EN */
	"st,write-posting", /* WR_POST */
};

const char *dt_client_prop[MAX_CLIENT_REGFIELDS] = {
	"st,chunk-max-size", /* MAX_CHK */
	"st,opcode-max-size", /* MAX_OPC */
	"st,burst-max-size", /* MAX_BURST */
	"st,outstanding-packets", /* OST_PKT */
	"st,ivc", /* IVC */
	"st,svc", /* SVC */
	"st,wlru-ratio", /* WLRU_RAT */
	"st,dpreg-size", /* DPREG */
};

struct ibi_ipplug {
#define IPPLUG_BUS_AXI	0
#define IPPLUG_BUS_ST	1
	unsigned int bus_mode;
	struct regmap *regmap;
	struct regmap_field *regf_g_arr[MAX_G_REGFIELDS];
	struct regmap_field *regf_client_arr[MAX_CLIENTS][MAX_CLIENT_REGFIELDS];
};

static const struct reg_field ibi_ipplug_g_regfields[MAX_G_REGFIELDS] = {
	[MEM_PG_SIZE] = REG_FIELD(0x0, 0, 2),
	[NODE_EN] = REG_FIELD(0x0, 8, 8),
	[WR_POST] = REG_FIELD(0x0, 16, 17),
	[PROG_W] = REG_FIELD(0x4, 0, 0),
	[PROG_R] = REG_FIELD(0x8, 0, 0),
};

static const struct reg_field
ibi_ipplug_client_regfields[MAX_CLIENT_REGFIELDS] = {
	[MAX_CHK] = REG_FIELD(0x20, 0, 1),
	[MAX_OPC] = REG_FIELD(0x20, 2, 3),
	[MAX_BURST] = REG_FIELD(0x20, 0, 2),
	[OST_PKT] = REG_FIELD(0x20, 8, 23),
	[IVC] = REG_FIELD(0x24, 0, 7),
	[SVC] = REG_FIELD(0x24, 8, 11),
	[WLRU_RAT] = REG_FIELD(0x24, 16, 19),
	[DPREG] = REG_FIELD(0x28, 0, 31),
};

static int ibi_check_param_common(struct ibi_config *cfg, unsigned int param,
				  struct reg_field regf)
{
	struct device *dev = cfg->ibidev->dev;
	if (param > GENMASK(regf.msb - regf.lsb, 0)) {
		dev_err(dev, "invalid argument for reg %d %d-%d\n",
			regf.reg, regf.lsb, regf.msb);
		return -EINVAL;
	}

	return 0;
}

static int ibi_check_g_param(struct ibi_config *cfg, unsigned int param,
			     enum regfield_g field)
{
	return ibi_check_param_common(cfg, param,
				      ibi_ipplug_g_regfields[field]);
}

static int ibi_check_client_param(struct ibi_config *cfg, unsigned int param,
				  enum regfield_client field)
{
	return ibi_check_param_common(cfg, param,
				      ibi_ipplug_client_regfields[field]);
}

static int ibi_alloc_g_regmapf(struct ibi_config *cfg, enum regfield_g field)
{
	struct device *dev = cfg->ibidev->dev;
	struct ibi_ipplug *ibi_data = cfg->ibidev->driver_data;

	ibi_data->regf_g_arr[field] =
		devm_regmap_field_alloc(dev,
					ibi_data->regmap,
					ibi_ipplug_g_regfields[field]);
	if (IS_ERR(ibi_data->regf_g_arr[field])) {
		dev_err(dev, "failed to alloc reg_g field %d\n", field);
		return PTR_ERR(ibi_data->regf_g_arr[field]);
	}

	return 0;
}

static int ibi_alloc_client_regmapf(struct ibi_config *cfg, unsigned int client,
				    enum regfield_client field)
{
	struct device *dev = cfg->ibidev->dev;
	struct ibi_ipplug *ibi_data = cfg->ibidev->driver_data;
	struct reg_field regf = ibi_ipplug_client_regfields[field];

	regf.reg += 0x10 * client;

	ibi_data->regf_client_arr[client][field] =
		devm_regmap_field_alloc(dev,
					ibi_data->regmap, regf);
	if (IS_ERR(ibi_data->regf_client_arr[client][field])) {
		dev_err(dev, "failed to alloc reg_client field %d\n", field);
		return PTR_ERR(ibi_data->regf_client_arr[client][field]);
	}

	return 0;
}

static int ibi_parse_g_prop_u8(struct device_node *np, struct ibi_config *cfg,
			       enum regfield_g field, unsigned char *config)
{
	struct device *dev = cfg->ibidev->dev;
	int ret;

	*config = 0;

	ret = of_property_read_u8(np, dt_g_prop[field],
				  config);
	if (ret) {
		dev_dbg(dev, "ibi config without %s\n", dt_g_prop[field]);
		return 0;
	}

	ret = ibi_check_g_param(cfg, *config, field);
	if (ret)
		return ret;

	ret = ibi_alloc_g_regmapf(cfg, field);

	return ret;
}

static int ibi_parse_g_prop_u32(struct device_node *np, struct ibi_config *cfg,
				enum regfield_g field, unsigned int *config)
{
	struct device *dev = cfg->ibidev->dev;
	int ret;

	*config = 0;

	ret = of_property_read_u32(np, dt_g_prop[field],
				   config);
	if (ret) {
		dev_dbg(dev, "ibi config without %s\n", dt_g_prop[field]);
		return 0;
	}

	ret = ibi_check_g_param(cfg, *config, field);
	if (ret)
		return ret;

	ret = ibi_alloc_g_regmapf(cfg, field);

	return ret;
}

static int ibi_parse_g_prop_pwr2(struct device_node *np, struct ibi_config *cfg,
				 enum regfield_g field, unsigned char *config,
				 u8 start_index)
{
	struct device *dev = cfg->ibidev->dev;
	int ret;
	unsigned int dt_val;

	*config = 0;

	ret = of_property_read_u32(np, dt_g_prop[field],
				   &dt_val);
	if (ret) {
		dev_dbg(dev, "ibi config without %s\n", dt_g_prop[field]);
		return 0;
	}

	if (dt_val)
		*config = order_base_2(dt_val) - start_index;

	ret = ibi_check_g_param(cfg, *config, field);
	if (ret)
		return ret;

	ret = ibi_alloc_g_regmapf(cfg, field);

	return ret;
}

static int ibi_parse_client_prop_u8(struct device_node *np,
				    struct ibi_config *cfg, unsigned int client,
				    enum regfield_client field,
				    unsigned char *config)
{
	struct device *dev = cfg->ibidev->dev;
	int ret;

	*config = 0;

	ret = of_property_read_u8(np, dt_client_prop[field],
				  config);
	if (ret) {
		dev_dbg(dev, "ibi config without %s\n", dt_client_prop[field]);
		return 0;
	}

	ret = ibi_check_client_param(cfg, *config, field);
	if (ret)
		return ret;

	ret = ibi_alloc_client_regmapf(cfg, client, field);

	return ret;
}

static int ibi_parse_client_prop_u32(struct device_node *np,
				     struct ibi_config *cfg,
				     unsigned int client,
				     enum regfield_client field,
				     unsigned int *config)
{
	struct device *dev = cfg->ibidev->dev;
	int ret;

	*config = 0;

	ret = of_property_read_u32(np, dt_client_prop[field],
				   config);
	if (ret) {
		dev_dbg(dev, "ibi config without %s\n", dt_client_prop[field]);
		return 0;
	}

	ret = ibi_check_client_param(cfg, *config, field);
	if (ret)
		return ret;

	ret = ibi_alloc_client_regmapf(cfg, client, field);

	return ret;
}

static int ibi_parse_client_prop_pwr2(struct device_node *np,
				      struct ibi_config *cfg,
				      unsigned int client,
				      enum regfield_client field,
				      unsigned char *config, u8 start_index)
{
	struct device *dev = cfg->ibidev->dev;
	int ret;
	unsigned int dt_val;

	*config = 0;

	ret = of_property_read_u32(np, dt_client_prop[field],
				   &dt_val);

	if (ret) {
		dev_dbg(dev, "ibi config without %s\n", dt_client_prop[field]);
		return 0;
	}

	if (dt_val)
		*config = order_base_2(dt_val) - start_index;

	ret = ibi_check_client_param(cfg, *config, field);
	if (ret)
		return ret;

	ret = ibi_alloc_client_regmapf(cfg, client, field);

	return ret;
}

static int ibi_parse_client_dpreg(struct ibi_config *cfg,
				  struct ibi_ipplug_cfg *cfg_priv)
{
	struct device *dev = cfg->ibidev->dev;
	unsigned int dpreg_start, dpreg_end, dpreg_size;
	int i;
	int ret;

	if (!cfg_priv->bus_size || !cfg_priv->dpreg_total_size) {
		dev_dbg(dev, "missing global settings, DPREG skipped\n");
		return 0;
	}

	dpreg_end = 0;

	for (i = 0; i < MAX_CLIENTS; i++) {
		if (!cfg_priv->clients[i])
			continue;

		if (dpreg_end) {
			dpreg_start = dpreg_end + 1;
		} else {
			dpreg_start = 0;
		}

		dpreg_size = cfg_priv->clients[i]->dpreg_size;
		if (dpreg_start > 0x10000) {
			dev_err(dev, "invalid dpreg start 0x%x on client %d\n",
				dpreg_start, i);
			return -EINVAL;
		}
		/* Compute the dpreg_end for this client */
		dpreg_end = dpreg_start +
				(dpreg_size * 8 / cfg_priv->bus_size) - 1;
		if (dpreg_end >=
			(cfg_priv->dpreg_total_size * 8 / cfg_priv->bus_size) ||
		    dpreg_end > 0x10000) {
			dev_err(dev, "dpreg size on client %d exceed total\n",
				i);
			return -EINVAL;
		}

		ret = ibi_alloc_client_regmapf(cfg, i, DPREG);
		if (ret)
			return ret;

		cfg_priv->clients[i]->dpreg = (dpreg_end << 16) | dpreg_start;
	}

	return 0;
}

#define PARSE_G_PROP_U8(np, cfg, field, cfg_st) \
		ibi_parse_g_prop_u8(np, cfg, field, &cfg_st->field##_CONFIG)
#define PARSE_G_PROP_U32(np, cfg, field, cfg_st) \
		ibi_parse_g_prop_u32(np, cfg, field, &cfg_st->field##_CONFIG)
#define PARSE_G_PROP_PWR2(np, cfg, field, cfg_st) \
		ibi_parse_g_prop_pwr2(np, cfg, field, &cfg_st->field##_CONFIG, \
				      field##_PWR2_START_INDEX)
#define PARSE_CLIENT_PROP_U8(np, cfg, client, field, cfg_st) \
		ibi_parse_client_prop_u8(np, cfg, client, field, \
				    &cfg_st->clients[client]->field##_CONFIG)
#define PARSE_CLIENT_PROP_U32(np, cfg, client, field, cfg_st) \
		ibi_parse_client_prop_u32(np, cfg, client, field, \
				     &cfg_st->clients[client]->field##_CONFIG)
#define PARSE_CLIENT_PROP_PWR2(np, cfg, client, field, cfg_st) \
		ibi_parse_client_prop_pwr2(np, cfg, client, field, \
				     &cfg_st->clients[client]->field##_CONFIG, \
				     field##_PWR2_START_INDEX)

static int ibi_apply_g_config(struct ibi_config *cfg, enum regfield_g field,
			      unsigned char config)
{
	struct ibi_ipplug *ibi_data = cfg->ibidev->driver_data;
	int ret = 0;
	struct regmap_field *regmapf = ibi_data->regf_g_arr[field];

	if (regmapf)
		ret = regmap_field_write(regmapf, config);

	return ret;
}

static int ibi_apply_client_config(struct ibi_config *cfg,
				   unsigned int client,
				   enum regfield_client field,
				   unsigned int config)
{
	struct ibi_ipplug *ibi_data = cfg->ibidev->driver_data;
	int ret = 0;
	struct regmap_field *regmapf = ibi_data->regf_client_arr[client][field];

	if (regmapf)
		ret = regmap_field_write(regmapf, config);

	return ret;
}

#define APPLY_G_CONFIG(cfg, field, cfg_st) \
		ibi_apply_g_config(cfg, field, cfg_st->field##_CONFIG)
#define APPLY_G_CONFIG_U8(cfg, field, cfg_st) \
		APPLY_G_CONFIG(cfg, field, cfg_st)
#define APPLY_G_CONFIG_U32(cfg, field, cfg_st) \
		APPLY_G_CONFIG(cfg, field, cfg_st)
#define APPLY_CLIENT_CONFIG(cfg, client, field, cfg_st) \
		ibi_apply_client_config(cfg, client, field, \
				      cfg_st->clients[client]->field##_CONFIG)
#define APPLY_CLIENT_CONFIG_U8(cfg, client, field, cfg_st) \
		APPLY_CLIENT_CONFIG(cfg, client, field, cfg_st)
#define APPLY_CLIENT_CONFIG_U32(cfg, client, field, cfg_st) \
		APPLY_CLIENT_CONFIG(cfg, client, field, cfg_st)

static int ibi_ipplug_parse(struct device_node *np, struct ibi_config *cfg)
{
	struct device *dev = cfg->ibidev->dev;
	struct ibi_ipplug *ibi_data = cfg->ibidev->driver_data;
	struct ibi_ipplug_cfg *cfg_priv;
	int ret, cid;
	struct device_node *node;

	cfg_priv = devm_kzalloc(dev, sizeof(*cfg_priv), GFP_KERNEL);
	if (!cfg_priv)
		return -ENOMEM;

	/* Allocate regmap_field only for the prop mentioned in DT-entry */
	ret = PARSE_G_PROP_PWR2(np, cfg, MEM_PG_SIZE, cfg_priv);
	if (ret)
		return ret;

	ret = PARSE_G_PROP_U32(np, cfg, WR_POST, cfg_priv);
	if (ret)
		return ret;

	ret = PARSE_G_PROP_U8(np, cfg, NODE_EN, cfg_priv);
	if (ret)
		return ret;

	ret = ibi_alloc_g_regmapf(cfg, PROG_W);
	if (ret)
		return ret;

	ret = ibi_alloc_g_regmapf(cfg, PROG_R);
	if (ret)
		return ret;

	/* Try to read bus-size & dpreg-total-size, necessary for st,dpreg-size
	 * of each clients
	 */
	ret = of_property_read_u32(np, "st,bus-size", &cfg_priv->bus_size);
	if (ret)
		dev_dbg(dev, "No st,bus-size, no dpreg settings\n");

	ret = of_property_read_u32(np, "st,dpreg-total-size",
				   &cfg_priv->dpreg_total_size);
	if (ret)
		dev_dbg(dev, "No st,dpreg-total-size, no dpreg settings\n");

	for_each_child_of_node(np, node) {
		cid = MAX_CLIENTS;
		if (sscanf(node->name, "client%d", &cid) == 1 &&
		    cid < MAX_CLIENTS) {
			/*
			 * Allocate ibi_ipplug_client_cfg only for the clients
			 * present in DT
			 */
			cfg_priv->clients[cid] =
				devm_kzalloc(dev,
					     sizeof(*cfg_priv->clients[cid]),
					     GFP_KERNEL);
			if (!cfg_priv->clients[cid])
				return -ENOMEM;

			if (ibi_data->bus_mode == IPPLUG_BUS_ST) {
				ret = PARSE_CLIENT_PROP_PWR2(node, cfg, cid,
							     MAX_CHK,
							     cfg_priv);
				if (ret)
					return ret;

				ret = PARSE_CLIENT_PROP_PWR2(node, cfg, cid,
							     MAX_OPC,
							     cfg_priv);
				if (ret)
					return ret;
			} else if (ibi_data->bus_mode == IPPLUG_BUS_AXI) {
				ret = PARSE_CLIENT_PROP_PWR2(node, cfg, cid,
							     MAX_BURST,
							     cfg_priv);
				if (ret)
					return ret;
			}

			ret = PARSE_CLIENT_PROP_U32(node, cfg, cid, OST_PKT,
						    cfg_priv);
			if (ret)
				return ret;

			ret = PARSE_CLIENT_PROP_U8(node, cfg, cid, IVC,
						   cfg_priv);
			if (ret)
				return ret;

			ret = PARSE_CLIENT_PROP_U8(node, cfg, cid, SVC,
						   cfg_priv);
			if (ret)
				return ret;

			ret = PARSE_CLIENT_PROP_U8(node, cfg, cid, WLRU_RAT,
						   cfg_priv);
			if (ret)
				return ret;

			/* For the time being only store the dpreg-size value */
			ret = of_property_read_u32(node, dt_client_prop[DPREG],
					&cfg_priv->clients[cid]->dpreg_size);
			if (ret)
				dev_dbg(dev, "ibi config without %s\n",
					dt_client_prop[DPREG]);
			if (!cfg_priv->clients[cid]->dpreg_size) {
				dev_err(dev, "invalid dpreg-size value\n");
				return -EINVAL;
			}
		}
	}

	/* Process the dpreg value by looking at each client entry */
	ret = ibi_parse_client_dpreg(cfg, cfg_priv);

	cfg->config_priv = cfg_priv;

	return 0;
}

static int ibi_ipplug_apply_config(struct ibi_config *cfg)
{
	struct ibi_ipplug_cfg *cfg_priv = cfg->config_priv;
	struct ibi_ipplug *ibi_data = cfg->ibidev->driver_data;
	struct device *dev = cfg->ibidev->dev;
	int ret, i;
	unsigned int lock_status;

	dev_dbg(dev, "Applyling ibi_config: %s", cfg->name);

	/* Lock Request */
	ret = regmap_field_write(ibi_data->regf_g_arr[PROG_W],
				 0x1);
	if (ret)
		goto err;

	/* Status Polling */
	do {
		ret = regmap_field_read(ibi_data->regf_g_arr[PROG_R],
					&lock_status);
		if (ret)
			goto err;

	} while (lock_status != 0x1);

	/*
	 * Apply the config only for the allocated regmap_fields, which got
	 * allocated during parsing only for the prop present in DT-entry
	 */
	ret = APPLY_G_CONFIG_U8(cfg, MEM_PG_SIZE, cfg_priv);
	if (ret)
		goto err;

	ret = APPLY_G_CONFIG_U8(cfg, NODE_EN, cfg_priv);
	if (ret)
		goto err;

	ret = APPLY_G_CONFIG_U32(cfg, WR_POST, cfg_priv);
	if (ret)
		goto err;

	for (i = 0; i < MAX_CLIENTS; i++) {
		if (!cfg_priv->clients[i])
			continue;

		if (ibi_data->bus_mode == IPPLUG_BUS_ST) {
			ret = APPLY_CLIENT_CONFIG_U8(cfg, i, MAX_CHK, cfg_priv);
			if (ret)
				goto err;

			ret = APPLY_CLIENT_CONFIG_U8(cfg, i, MAX_OPC, cfg_priv);
			if (ret)
				goto err;
		} else if (ibi_data->bus_mode == IPPLUG_BUS_AXI) {
			ret = APPLY_CLIENT_CONFIG_U8(cfg, i, MAX_BURST,
						     cfg_priv);
			if (ret)
				goto err;
		}

		ret = APPLY_CLIENT_CONFIG_U32(cfg, i, OST_PKT, cfg_priv);
		if (ret)
			goto err;

		ret = APPLY_CLIENT_CONFIG_U8(cfg, i, IVC, cfg_priv);
		if (ret)
			goto err;

		ret = APPLY_CLIENT_CONFIG_U8(cfg, i, SVC, cfg_priv);
		if (ret)
			goto err;

		ret = APPLY_CLIENT_CONFIG_U8(cfg, i, WLRU_RAT, cfg_priv);
		if (ret)
			goto err;

		ret = APPLY_CLIENT_CONFIG_U32(cfg, i, DPREG, cfg_priv);
		if (ret)
			goto err;
	}

	/* Unlock Request */
	ret = regmap_field_write(ibi_data->regf_g_arr[PROG_W],
				 0x0);
	if (ret)
		goto err;

	return 0;

err:
	dev_warn(dev, "Error in applyling ibi_config: %s", cfg->name);
	return ret;
}

struct ibi_ops ibi_ipplug_ops = {
	.of_parse_config = ibi_ipplug_parse,
	.apply_config = ibi_ipplug_apply_config, };

struct ibi_desc ibi_ipplug_desc = {
	.ops = &ibi_ipplug_ops,
};

static const struct regmap_config ibi_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static int ibi_ipplug_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct ibi_dev *ibi_dev = NULL;
	struct ibi_ipplug *ibi_data = NULL;
	void __iomem *mmio;
	struct resource *res;

	ibi_data = devm_kzalloc(dev, sizeof(*ibi_data), GFP_KERNEL);
	if (!ibi_data)
		return -ENOMEM;

	if (of_device_is_compatible(np, "st,ibi-ipplug"))
		ibi_data->bus_mode = IPPLUG_BUS_ST;
	else if (of_device_is_compatible(np, "st,ibi-ipplug-axi"))
		ibi_data->bus_mode = IPPLUG_BUS_AXI;
	else
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmio = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(mmio))
		return PTR_ERR(mmio);

	ibi_data->regmap = devm_regmap_init_mmio(dev, mmio,
						 &ibi_regmap_cfg);
	if (IS_ERR(ibi_data->regmap))
		return PTR_ERR(ibi_data->regmap);

	platform_set_drvdata(pdev, ibi_dev);

	ibi_dev = ibi_register(&ibi_ipplug_desc, dev, ibi_data);
	if (IS_ERR(ibi_dev))
		return PTR_ERR(ibi_dev);

	return 0;
}

static int ibi_ipplug_remove(struct platform_device *pdev)
{
	struct ibi_dev *ibidev = platform_get_drvdata(pdev);

	ibi_unregister(ibidev);

	return 0;
}

static const struct of_device_id ibi_ipplug_of_match[] = {
	{ .compatible = "st,ibi-ipplug" },
	{ .compatible = "st,ibi-ipplug-axi" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ibi_ipplug);
static struct platform_driver ibi_ipplug = {
	.driver = {
		.name = "ibi_ipplug",
		.owner = THIS_MODULE,
		.of_match_table = ibi_ipplug_of_match,
	},
	.probe = ibi_ipplug_probe,
	.remove = ibi_ipplug_remove,
};

module_platform_driver(ibi_ipplug);

MODULE_AUTHOR("<pankaj.dev@st.com>");
MODULE_DESCRIPTION("IPPlug bridge driver using the IBI framework");
MODULE_LICENSE("GPL v2");
