/*
 * Copyright (C) 2018 STMicroelectronics
 *
 * STMicroelectronics Embedded PHY driver
 *
 * Author: Jerome Audu <jerome.audu@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <dt-bindings/phy/phy.h>

#define SYSCFG_2035 0x8c
#define EPHY_CLKEN_BIT	BIT(11)

struct ephy_phy {
	struct phy *phy;
	struct device *dev;
	struct reset_control *reset;
	struct clk *clock;
	struct regmap *regmap;
};

static int ephy_init(struct phy *phy)
{
	struct ephy_phy *ephy_phy = phy_get_drvdata(phy);
	int ret;

	ret = regmap_update_bits(ephy_phy->regmap, SYSCFG_2035,
				 (unsigned int)EPHY_CLKEN_BIT, false);
	if (ret)
		return ret;

	if (ephy_phy->clock)
		clk_prepare_enable(ephy_phy->clock);

	reset_control_assert(ephy_phy->reset);
	reset_control_deassert(ephy_phy->reset);
	usleep_range(10000, 15000);

	dev_info(ephy_phy->dev, "Embedded PHY init done\n");

	return 0;
}

static const struct phy_ops ephy_ops = {
	.init = ephy_init,
	.owner = THIS_MODULE,
};

static int ephy_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct ephy_phy *ephy_phy;
	struct phy_provider *provider;
	int ret;

	ephy_phy = devm_kzalloc(&pdev->dev, sizeof(*ephy_phy), GFP_KERNEL);
	if (!ephy_phy)
		return -ENOMEM;

	ephy_phy->dev = &pdev->dev;

	dev_set_drvdata(ephy_phy->dev, ephy_phy);

	dev_info(ephy_phy->dev, "Embedded PHY probed\n");

	ephy_phy->phy = devm_phy_create(ephy_phy->dev, np, &ephy_ops);
	if (IS_ERR(ephy_phy->phy)) {
		dev_err(ephy_phy->dev, "failed to create PHY\n");
		ret = PTR_ERR(ephy_phy->phy);
		goto error;
	}

	ephy_phy->reset = devm_reset_control_get(ephy_phy->dev, NULL);
	if (IS_ERR(ephy_phy->reset)) {
		dev_err(ephy_phy->dev, "no ephy-rst declared\n");
		ret = PTR_ERR(ephy_phy->reset);
		goto error;
	}

	ephy_phy->clock = devm_clk_get(ephy_phy->dev, NULL);
	if (IS_ERR(ephy_phy->clock))
		ephy_phy->clock = NULL;

	ephy_phy->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(ephy_phy->regmap)) {
		dev_err(ephy_phy->dev, "No syscfg phandle specified\n");
		ret = PTR_ERR(ephy_phy->regmap);
		goto error;
	}

	phy_set_drvdata(ephy_phy->phy, ephy_phy);

	provider = devm_of_phy_provider_register(ephy_phy->dev,
						 of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);

error:
	of_node_put(np);
	return ret;
}

static const struct of_device_id ephy_of_match[] = {
	{
		.compatible = "st,gllcff-ephy-phy",
	},
	{},
};

MODULE_DEVICE_TABLE(of, ephy_of_match);

static struct platform_driver ephy_driver = {
	.probe = ephy_probe,
	.driver = {
		.name = "ephy-phy",
		.of_match_table = ephy_of_match,
	}
};

module_platform_driver(ephy_driver);

MODULE_AUTHOR("Jerome Audu <jerome.audu@st.com>");
MODULE_DESCRIPTION("STMicroelectronics ephy driver");
MODULE_LICENSE("GPL v2");
