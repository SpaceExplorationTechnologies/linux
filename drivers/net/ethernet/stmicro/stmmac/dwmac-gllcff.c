/*
 * Copyright (C) 2016 STMicroelectronics
 *
 * Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>

#include "stmmac_platform.h"

#define	GMAC_SS_ADDR		0x2000
#define	GMAC_SS_CONF		0
#define	GMAC_SS_STATUS		0x4
#define	EMBEDDED_PHY		0
#define	SGMII_PHY		BIT(0)
#define	SGMII_SPEED_MASK	GENMASK(2, 1)
#define	SGMII_SPEED_SHIFT	1

struct gllcff_dwmac {
	struct device *dev;
	void __iomem *reg;	/* GMAC Sub-System configuration */
};

struct gllcff_dwmac_phy_config {
	unsigned long sgmii;
	unsigned long ephy;
};

const struct gllcff_dwmac_phy_config gllcff_gmac0_phy_config = {
	.sgmii = BIT(0),
	.ephy = 0,
};

const struct gllcff_dwmac_phy_config gllcff_gmac1_phy_config = {
	.sgmii = 0,
	.ephy = BIT(0),
};

static const struct of_device_id gllcff_dwmac_match[];

/* To check the real speed from status register */
static void gllcff_dwmac_speed(void *priv, u32 spd)
{
	struct gllcff_dwmac *dwmac = priv;
	u32 sgmii_speed;
	int speed = -EINVAL;

	sgmii_speed = (readl_relaxed(dwmac->reg + GMAC_SS_STATUS) &
		       SGMII_SPEED_MASK) >> SGMII_SPEED_SHIFT;

	switch (sgmii_speed) {
	case 2:
		speed = SPEED_1000;
		break;
	case 1:
		speed = SPEED_100;
		break;
	case 0:
		speed = SPEED_10;
		break;
	default:
		break;
	};

	if (speed != -EINVAL)
		pr_debug("SGMII Speed %d Mb/s\n", speed);
}

static int gllcff_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct gllcff_dwmac *dwmac;
	int ret;
	struct reset_control *gmac_global;
	struct phy *phy;
	bool ephy;
	const struct gllcff_dwmac_phy_config *phy_config;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

#ifdef CONFIG_SPACEX
	plat_dat->disable_pcs = of_property_read_bool(pdev->dev.of_node,
						      "disable-pcs");
	if (of_property_read_bool(pdev->dev.of_node, "mdiobus-scan-of")) {
		plat_dat->mdio_node = pdev->dev.of_node;
	}
	plat_dat->disable_fcs = of_property_read_bool(pdev->dev.of_node,
						      "disable-fcs");
	plat_dat->always_enable_mac_tx = of_property_read_bool(pdev->dev.of_node,
						      "always-enable-mac-tx");
#endif

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;

	plat_dat->bsp_priv = dwmac;
	dwmac->reg = stmmac_res.addr + GMAC_SS_ADDR;

	/* Get the GMAC out of reset */
	gmac_global = devm_reset_control_get(&pdev->dev, "gmac-global");
	if (IS_ERR(gmac_global)) {
		dev_err(&pdev->dev, "couldn't get the global reset\n");
		return PTR_ERR(gmac_global);
	}
	reset_control_assert(gmac_global);

	/* Release the GMAC reset */
	reset_control_deassert(gmac_global);

	phy_config = (const struct gllcff_dwmac_phy_config *)
		of_match_node(gllcff_dwmac_match, pdev->dev.of_node)->data;
	if (!phy_config) {
		dev_err(&pdev->dev, "unable to get phy config\n");
		return -EIO;
	}

	/* Get the gmac-phy, ie an external phy, otherwise try EPHY */
	phy = devm_phy_get(&pdev->dev, "gmac-phy");
	if (IS_ERR(phy)) {
		if (PTR_ERR(phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_err(&pdev->dev, "Couldn't find gmac-phy\n");
		return -EINVAL;
	}

	/* Select PHY connected to the GMAC */
	ephy = strstr(dev_name(&phy->dev), "ephy") ? true : false;

	if (ephy) {
		writel_relaxed(phy_config->ephy, dwmac->reg + GMAC_SS_CONF);
		dev_info(&pdev->dev, "attached to Embedded PHY\n");
	} else {
		writel_relaxed(phy_config->sgmii, dwmac->reg + GMAC_SS_CONF);
		plat_dat->fix_mac_speed = gllcff_dwmac_speed;
		dev_info(&pdev->dev, "attached to SGMII\n");
	}

	phy_init(phy);

	return stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
}

static const struct of_device_id gllcff_dwmac_match[] = {
	{
		.compatible = "st,gllcff-dwmac0",
		.data = &gllcff_gmac0_phy_config,
	},
	{
		.compatible = "st,gllcff-dwmac1",
		.data = &gllcff_gmac1_phy_config,
	},
	{},
};

MODULE_DEVICE_TABLE(of, gllcff_dwmac_match);

static struct platform_driver gllcff_dwmac_driver = {
	.probe = gllcff_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		   .name = "gllcff-dwmac",
		   .pm = &stmmac_pltfr_pm_ops,
		   .of_match_table = gllcff_dwmac_match,
	},
};

module_platform_driver(gllcff_dwmac_driver);

MODULE_AUTHOR("Giuseppe Cavallaro <peppe.cavallaro@st.com>");
MODULE_DESCRIPTION("STi DWMAC glue layer for GLLCFF");
MODULE_LICENSE("GPL v2");
