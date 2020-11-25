/*
 * Copyright (C) 2015 STMicroelectronics Limited
 * Authors: Pankaj Dev <pankaj.dev@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

/*
 * ST specific wrapper around the generic power domain
 */
struct st_pm_domain {
	struct device *dev;
	struct regmap_field *regfield;
	struct generic_pm_domain pd;
};

static int st_gate_power(struct generic_pm_domain *domain, bool power_on)
{
	struct st_pm_domain *pd = container_of(domain, struct st_pm_domain, pd);
	int err;

	err = regmap_field_write(pd->regfield, power_on);
	if (err)
		dev_err(pd->dev, "IO register not accessible\n");

	return err;
}

static int st_gate_power_status(struct generic_pm_domain *domain,
				bool *power_on)
{
	struct st_pm_domain *pd = container_of(domain, struct st_pm_domain, pd);
	unsigned int val;
	int err;

	err = regmap_field_read(pd->regfield, &val);
	if (err) {
		dev_err(pd->dev, "IO register not accessible\n");
		return err;
	}

	*power_on = !!val;

	return 0;
}

static int st_gate_power_on(struct generic_pm_domain *domain)
{
	return st_gate_power(domain, true);
}

static int st_gate_power_off(struct generic_pm_domain *domain)
{
	return st_gate_power(domain, false);
}

int sti_pd_gate_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct st_pm_domain *pd;
	struct regmap *regmap;
	struct reg_field regfield;
	u32 offset, bitpos;
	bool power_on;
	int err;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to lookup regmap\n");
		return PTR_ERR(regmap);
	}

	err = of_property_read_u32_index(np, "st,syscfg", 1, &offset);
	if (err) {
		dev_err(dev, "No syscfg offset specified\n");
		return err;
	}

	bitpos = 0;
	err = of_property_read_u32_index(np, "st,syscfg", 2, &bitpos);
	if (err && err != -EOVERFLOW) {
		dev_err(dev, "Failed to parse syscfg bitpos\n");
		return err;
	}

	pd->pd.name = kstrdup(np->name, GFP_KERNEL);
	pd->pd.power_off = st_gate_power_off;
	pd->pd.power_on = st_gate_power_on;

	regfield.reg = offset;
	regfield.lsb = bitpos;
	regfield.msb = bitpos;
	pd->regfield = devm_regmap_field_alloc(dev, regmap, regfield);
	if (IS_ERR(pd->regfield)) {
		dev_err(dev, "Failed to create reg_field\n");
		return PTR_ERR(pd->regfield);
	}

	/* Check the clock status at boot */
	err = st_gate_power_status(&pd->pd, &power_on);
	if (err)
		return err;

	/* Clock status at boot, is_off="!power_on" passed */
	pm_genpd_init(&pd->pd, NULL, !power_on);
	of_genpd_add_provider_simple(np, &pd->pd);

	pd->dev = &pdev->dev;

	dev_info(dev, "registered (TP-status:%d)\n", power_on);

	return 0;
}

static const struct of_device_id sti_pd_gate_match[] = {
	{ .compatible = "st,fcgate-pd" },
	{},
};

static struct platform_driver st_pd_gate_driver = {
	.probe = sti_pd_gate_probe,
	.driver = {
		   .name = "sti-pd-gate",
		   .owner = THIS_MODULE,
		   .of_match_table = sti_pd_gate_match,
	},
};

static int __init st_pm_init_power_domain(void)
{
	return platform_driver_register(&st_pd_gate_driver);
}
arch_initcall(st_pm_init_power_domain);
