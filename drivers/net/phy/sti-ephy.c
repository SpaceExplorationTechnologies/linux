/*
 * Copyright (C) 2019 ST Microelectronics
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/* ST Microelectronics STi Platform Embedded PHY support */
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>

struct sti_ephy_priv {
	struct mutex lock;	/* lock for suspend/resume */
};

static inline int sti_ephy_update_reg(struct phy_device *phydev,
				      int addr, int mask, int value)
{
	int ctl;

	ctl = phy_read(phydev, addr);
	if (ctl < 0)
		return ctl;

	return phy_write(phydev, addr, (ctl & ~mask) | (value & mask));
}

static int sti_ephy_suspend(struct phy_device *phydev)
{
	struct sti_ephy_priv *priv = phydev->priv;

	mutex_lock(&priv->lock);
	sti_ephy_update_reg(phydev, MII_BMCR, BMCR_PDOWN, BMCR_PDOWN);
	mutex_unlock(&priv->lock);

	return 0;
}

static int sti_ephy_resume(struct phy_device *phydev)
{
	struct sti_ephy_priv *priv = phydev->priv;

	mutex_lock(&priv->lock);
	sti_ephy_update_reg(phydev, MII_BMCR, BMCR_PDOWN, 0);
	mutex_unlock(&priv->lock);

	return 0;
}

/* Perform the CUSTOM configuration of the PHY upon reset of the IP */
#define MII_EPHY_SHADOW_PAGE_REG	0x10
#define MII_EPHY_TX_OPAMP_BIAS		0x17
#define MII_EPHY_AFE_SPARE_AB		0x18
#define MII_EPHY_AFE_SPARE_CD		0x19
static int sti_ephy_config_init(struct phy_device *phydev)
{
	struct sti_ephy_priv *priv;
	int ret;

	priv = devm_kzalloc(&phydev->mdio.dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);
	phydev->priv = priv;

	/* Put the PHY in Low Power mode */
	ret = sti_ephy_suspend(phydev);
	if (ret)
		return ret;

	/* Get access to shadow registers */
	ret = sti_ephy_update_reg(phydev, MII_EPHY_SHADOW_PAGE_REG,
				  (0x3f << 8), (0x0f << 8));
	if (ret)
		return ret;

	/* Set OPAMP */
	ret = sti_ephy_update_reg(phydev, MII_EPHY_TX_OPAMP_BIAS,
				  0xff00, 0xaa00);
	if (ret)
		return ret;

	/* Set Control Capacitance */
	ret = sti_ephy_update_reg(phydev, MII_EPHY_AFE_SPARE_AB,
				  0xffff, 0x4040);
	if (ret)
		return ret;

	ret = sti_ephy_update_reg(phydev, MII_EPHY_AFE_SPARE_CD,
				  0xffff, 0x4040);
	if (ret)
		return ret;

	/* Close access to shadow registers */
	ret = sti_ephy_update_reg(phydev, MII_EPHY_SHADOW_PAGE_REG,
				  (0x3f << 8), 0);
	if (ret)
		return ret;

	return 0;
}

static int sti_ephy_config_aneg(struct phy_device *phydev)
{
	int ret, ret_aneg;

	/* need power-down mode to update AN registers */
	ret = sti_ephy_suspend(phydev);
	if (ret)
		return ret;

	ret_aneg = genphy_config_aneg(phydev);

	/* exit power down */
	ret = sti_ephy_resume(phydev);

	return ret_aneg ? ret_aneg : ret;
}

static int sti_ephy_read_status(struct phy_device *phydev)
{
	int status, tmp, ret;

	/* Read the Link status bit [2] of BMSR
	 * Taken from genphy_update_link
	 */
	/* Do a fake read */
	status = phy_read(phydev, MII_BMSR);
	if (status < 0)
		return status;

	/* Read link status */
	status = phy_read(phydev, MII_BMSR);
	if (status < 0)
		return status;

	if ((status & BMSR_LSTATUS) == 0) {
		/* Link status is down - check MII_EXPANSION */
		status = phy_read(phydev, MII_EXPANSION);
		if (status < 0)
			return status;

		if ((status & EXPANSION_MFAULTS) == 0)
			goto genphy_read_status;

		/* Power down the PHY (enable low power state) */
		tmp = phy_read(phydev, MII_BMCR);
		if (tmp < 0)
			return tmp;
		ret = phy_write(phydev, MII_BMCR, tmp | BMCR_PDOWN);
		if (ret < 0)
			return ret;

		/* Power up the PHY (disable low power state */
		ret = phy_write(phydev, MII_BMCR, tmp & ~BMCR_PDOWN);
		if (ret < 0)
			return ret;

		/* Wait for 5 seconds */
		mdelay(5000);
	}

genphy_read_status:
	return genphy_read_status(phydev);
}

#define PHY_ID_STI_EPHY		0x00061cf0
static struct phy_driver sti_ephy_phy_driver[] = {
{
	.phy_id		= PHY_ID_STI_EPHY,
	.phy_id_mask	= 0xffffffff,
	.name		= "ST Microelectronics Embedded PHY",
	.features	= PHY_GBIT_FEATURES,
	.soft_reset	= genphy_soft_reset,
	.config_init	= sti_ephy_config_init,
	.config_aneg	= sti_ephy_config_aneg,
	.read_status	= sti_ephy_read_status,
	.suspend	= sti_ephy_suspend,
	.resume		= sti_ephy_resume,
} };

static struct mdio_device_id __maybe_unused sti_ephy_phy_tbl[] = {
	{ PHY_ID_STI_EPHY, 0xffffffff, },
	{ }
};

MODULE_DEVICE_TABLE(mdio, sti_ephy_phy_tbl);

module_phy_driver(sti_ephy_phy_driver);

MODULE_DESCRIPTION("ST Microelectronics STi Embedded PHY driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Alain Volmat <alain.volmat@st.com>");
