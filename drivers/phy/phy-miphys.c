/*
 * Copyright (C) 2017 STMicroelectronics
 *
 * STMicroelectronics PHY driver MiPHY-S
 *
 * Author: Alain Volmat <alain.volmat@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/st_vtsens_kernel.h>

#include <dt-bindings/phy/phy.h>

/* TODO - PLL / Pipe Lock wait time to be tuned */
#define MIPHYS_PLL_WAIT_S	5
#define MIPHYS_PIPE_WAIT_S	5

#define MIPHY_EYE_POLL_RATE	HZ  /* Poll at 1HZ */

/* MiPHY-S DS register access macro */
#define MIPHYS_DS(n, r)	(4 * (((n) * 0x100) + ((r) & 0xff)))

#define MIPHYS_CONF_RESET	0x00
#define RST_APPLI_SW	BIT(0)
#define RST_CONF_SW	BIT(1)
#define RST_MACRO_SW	BIT(2)

#define MIPHYS_RESET		0x01
#define RST_PLL_SW	BIT(0)
#define RST_PLL_CAL_SW	BIT(1)
#define RST_COMP_SW	BIT(2)
#define RST_RX_CLK_SW	BIT(3)
#define RST_RX_DIG_SW	BIT(4)
#define RST_TX_SW	BIT(5)
#define RST_RX_CAL_SW	BIT(6)

#define MIPHYS_STATUS_1		0x02
#define HFC_READY	BIT(1)
#define PLL_READY	BIT(2)

#define MIPHYS_CONTROL		0x04
#define AUTO_RST	BIT(4)
#define RX_POL		BIT(5)
#define TX_POL		BIT(6)

#define MIPHYS_BOUNDARY_SEL	0x0a
#define MIPHYS_BOUNDARY_2	0x0c

#define MIPHYS_PLL_CLKREF_FREQ	0x0d
#define MIPHYS_SPEED		0x0e

#define MIPHYS_CONF		0x0f

#define MIPHYS_SYNCHAR_CONTROL	0x2f

#define MIPHYS_TX_SWING		0x49
#define MIPHYS_TX_SUPPLY_CAL_MAN	0x4a

#define MIPHYS_RX_LOCK_HOR_THRES_MAX	0x5c
#define MIPHYS_RX_STAT_LENGTH	0x5f
#define MIPHYS_RX_RBL		0x60
#define MIPHYS_RXCLKGEN_BIAS_CTRL	0x61
#define MIPHYS_RXBUF_LC_CTRL		0x62

#define MIPHYS_RX_MODSEL	0x74
#define MIPHYS_RX_K_GAIN	0x75

#define MIPHYS_RX_BUFFER_CTRL	0x78

#define MIPHYS_RX_GAIN_IDX_MAN	0x79

#define MIPHYS_RX_CAL_VGA_STEP	0x93
#define MIPHYS_RX_STAT_LOCK_CTRL	0x97
#define MIPHYS_RX_LOCK_HOR_THRES	0x98

#define MIPHYS_RX_OFFSET_CTRL	0xa9
#define MIPHYS_RX_LOCK_CTRL_1	0xb0
#define MIPHYS_RX_LOCK_SETTING_OPT	0xb1
#define MIPHYS_RX_SIGDET_SLEEP_SEL	0xb9
#define MIPHYS_RX_SIGDET_WAIT_SEL	0xba
#define MIPHYS_RX_SIGDET_DATA_SEL	0xbb

#define MIPHYS_RX_POWER_CTRL_1	0xbc
#define MIPHYS_RX_POWER_CTRL_2	0xbd
#define MIPHYS_RX_POWER_CTRL_4	0xbf
#define MIPHYS_RX_POWER_CTRL_5	0xc0
#define MIPHYS_RX_POWER_CTRL_6	0xc1

#define MIPHYS_PLL_RATIO_0	0xc4
#define MIPHYS_PLL_RATIO_1	0xc5
#define MIPHYS_PLL_RATIO_2	0xc6
#define MIPHYS_PLL_RATIO_3	0xc7

#define MIPHYS_PLL_CFG_0	0xc8
#define MIPHYS_PLL_CFG_3	0xcb
#define MIPHYS_PLL_RFCLKPATH_0	0xcc
#define MIPHYS_PLL_RFCLKPATH_1	0xcd

#define MIPHYS_PLL_CALFREQ_0	0xd3
#define MIPHYS_PLL_CALFREQ_1	0xd4
#define MIPHYS_TIME2CALFREQ_0(t) ((t) >> 5)
#define MIPHYS_TIME2CALFREQ_1(t) ((t) & 0x1f)
#define MIPHYS_PLL_CALFREQ_2	0xd5
#define MIPHYS_PLL_CALFREQ_3	0xd6
#define MIPHYS_TARGET2CALFREQ_2(t) ((t) >> 5)
#define MIPHYS_TARGET2CALFREQ_3(t) ((t) & 0x1f)

#define MIPHYS_PLL_ANA_0	0xd8
#define MIPHYS_PLL_ANA_1	0xd9
#define MIPHYS_PLL_ANA_3	0xda
#define MIPHYS_PLL_ANA_4	0xdb
#define MIPHYS_PLL_ANA_6	0xdd

#define MIPHYS_PLL_FDB_4	0xe6
#define PLL_LOCKED		BIT(3)

/* Miphy Release definitions */
#define MIPHY_VERSION		0xfe
#define MIPHY_REVISION		0xff
#define MIPHY_VER(n)	(((n) & 0xf) << 8)

/* SYSCFG offsets */

#define SYSCFG_MIPHY0_CLK      0x98000
#define SYSCFG_MIPHY1_CLK      0xA8000
#define SYSCFG_MIPHY2_CLK      0xB8000
#define MIPHY_CLKEN		BIT(0)
#define MIPHY_CS_CLK_REF_INT_EN        BIT(1)

#define SYSCFG_MIPHY0_CS_CONF	0x130040
#define SYSCFG_MIPHY1_CS_CONF	0x130070
#define SYSCFG_MIPHY2_CS_CONF	0x130094
#define MIPHY_PLL_VCODIV_RATIO_MASK	0x00000007
#define MIPHY_PLL_VCODIV_RATIO_4	0
#define MIPHY_PLL_VCODIV_RATIO_16	2
#define MIPHY_PLL_VCODIV_RATIO_20	6
#define MIPHY_PLL_VCODIV_PUP	BIT(3)
#define MIPHY_PLL_SSC_EN	BIT(4)
#define MIPHY_CS_CLK_FREF_DIG_EN	BIT(5)
#define MIPHY_CS_REFDIV_RATIO_MASK	0x000000c0
#define MIPHY_CS_REFDIV_RATIO_1	0
#define MIPHY_CS_PCIREGGEN_EN	BIT(8)
#define MIPHY_RC_IBIAS_PUP	BIT(9)

#define SYSCFG_MIPHY0_DS_0_TX_CONF	0x130044
#define SYSCFG_MIPHY1_SGMII_TX_CONF	0x130074
#define SYSCFG_MIPHY2_SGMII_TX_CONF	0x130098
#define MIPHY_SGMII_TX_OUT_EN	BIT(0)
#define MIPHY_SGMII_POWER_MODE_MASK	0x00000038
#define MIPHY_SGMII_POWER_MODE_1	(0x0 << 3)
#define MIPHY_SGMII_TX_C0_MASK	0x00003f00
#define MIPHY_SGMII_TX_C0_32	(0x20 << 8)

#define SYSCFG_MIPHY0_PRG_0_CONFIGURATION	0x13005c
#define MIPHY_PRG1_PCI_PUP		BIT(0)
#define MIPHY_PRG1_PCI_VREG_CTRL_MASK	0x06
#define MIPHY_PRG1_PCI_VREG_CTRL_08v	BIT(1)

#define SYSCFG_MIPHY_0_PIPE_CONFIGURATION	0x130060
#define SYSCFG_MIPHY_1_LINK_CONFIGURATION	0x13008C
#define SYSCFG_MIPHY_2_LINK_CONFIGURATION	0x1300B0
#define MIPHY0_LANE_EN		BIT(1)
#define MIPHY_EXT_MIPHY_CTRL	(BIT(0) | BIT(1))
#define MIPHY_EXT_MIPHY_CTRL_SGMII	0
#define MIPHY_EXT_MIPHY_CTRL_PCIE	0
#define MIPHY_EXT_MIPHY_CTRL_MDML	1

#define SYSCFG_MIPHY1_DIV_RATIO	0x130090
#define SYSCFG_MIPHY2_DIV_RATIO	0x1300B4
#define MIPHY_DIV_RATIO	(BIT(1) | BIT(0))


/* MIPHY0 only */
#define SYSCFG_MIPHY_0_STATUS	0x130118
#define MIPHY_0_P1_INIT_REG_DONE	BIT(0)

/* PCIE Pipe register */
#define PIPEW_TX_DETECT_PCIE_DELAY_COMMON_MODE_RISE_1 0x05
#define PIPEW_TX_DETECT_PCIE_DELAY_COMMON_MODE_FALL_1 0x09
#define PIPEW_TX_DETECT_PCIE_RISE_VALUE_THRESHOLD_0 0x0C
#define PIPEW_TX_DETECT_PCIE_RISE_VALUE_THRESHOLD_1 0x0D
#define PIPEW_TX_MIPHY_DETECT_CTRL 0x13
#define PIPEW_TX_EBUF_CTRL 0x74

/* Data slice 1 only */
/* SGMII Pipe registers */
#define MIPHYS_PIPE_RX_CAL_CTRL_1 0x01
#define MIPHYS_PIPE_RX_CAL_CTRL_2 0x02
#define RST_RX_CAL_N		BIT(7)
#define MIPHYS_PIPE_RX_CAL_OFFSET_CTRL 0x03
#define MIPHYS_PIPE_RX_CAL_BYPASS_FINE 0x04
#define MIPHY_PIPE_RX_CAL_VER_EYE_LEFT 0x0f
#define MIPHY_PIPE_RX_CAL_VER_EYE_RIGHT 0x10
#define    MIPHY_RX_CAL_VER_EYE_LEFT_BOUND_COMPL BIT(5)
#define MIPHYS_PIPE_RX_CAL_VGA_OFFSET 0x31

#define MAX_DS_NB	3
struct miphys_phy {
	struct phy *phy;

	struct device *dev;
	struct regmap *regmap;

	void __iomem *base;

	/* Miphy configuration */
	struct miphys_cfg *cfg;

	/* Miphy global reset */
	struct reset_control *miphy_rst;
	/* Miphy OSC reset */
	struct reset_control *miphy_osc_rst;
	/* Miphy Lane resets */
	struct reset_control *miphy_ln_rst[MAX_DS_NB];

	u8 type;

	u8 frequency;
	u8 polarity;

	bool externally_initialized;

	struct {
		int left;
		int right;
		struct delayed_work work;
	} eye;
};

/*
 * Table defining the configuration of each MiPhyS
 */
struct calib_target {
	int temperature;
	unsigned int target;
};

struct miphys_cfg {
	bool valid;
	u32 nb_ds;
	u32 nb_rst;
	u32 syscfg_lnk_conf;
	u32 syscfg_clk;
	u32 syscfg_cs_conf;
	u32 syscfg_tx_conf;
	u32 syscfg_vco_div;
	const struct calib_target *calib_target;
	u32 calib_target_nb;
};

#define PHY_MODES_NUM	(PHY_TYPE_MDML + 1)

static const struct of_device_id miphys_of_match[];

/*
 * Reset all DS necessary for the requested mode of the Miphy-S
 * Only keeping the register accessible once in reset
 */
static inline void miphys_reset_ds(struct miphys_phy *miphy_phy)
{
	void __iomem *base = miphy_phy->base;
	int i;

	/* Putting all DS in reset */
	for (i = 0; i < miphy_phy->cfg[miphy_phy->type].nb_ds; i++) {
		writel_relaxed(RST_APPLI_SW | RST_CONF_SW | RST_MACRO_SW,
			       base + MIPHYS_DS(i, MIPHYS_CONF_RESET));
		writel_relaxed(RST_APPLI_SW, base +
			       MIPHYS_DS(i, MIPHYS_CONF_RESET));
	}
}

/*
 * Unreset all DS necessary for the requested more of the Miphy-S
 */
static inline void miphys_unreset_ds(struct miphys_phy *miphy_phy)
{
	void __iomem *base = miphy_phy->base;
	int i;

	for (i = 0; i < miphy_phy->cfg[miphy_phy->type].nb_ds; i++)
		writel_relaxed(0x00, base + MIPHYS_DS(i, MIPHYS_CONF_RESET));
}

static inline int miphy_is_ready(struct miphys_phy *miphy_phy)
{
	unsigned long finish = jiffies + MIPHYS_PLL_WAIT_S * HZ;
	/* For Modemlink check that PLL is ready */
	const u8 mask = PLL_READY;
	u8 val;

	do {
		val = readl_relaxed(miphy_phy->base +
					MIPHYS_DS(0, MIPHYS_STATUS_1));
		if ((val & mask) == mask)
			return 0;

		cpu_relax();
	} while (!time_after_eq(jiffies, finish));

	return -EBUSY;
}

static int get_calfreq_target(struct miphys_phy *miphy_phy,
			      unsigned int *target)
{
	int ret;
	unsigned int i;
	unsigned int calib_nb = miphy_phy->cfg[miphy_phy->type].calib_target_nb;
	const struct calib_target *calib =
				miphy_phy->cfg[miphy_phy->type].calib_target;
	int temperature;

	if (!calib_nb) {
		dev_err(miphy_phy->dev,
			"No calibration temperatures defined\n");
		return -EINVAL;
	}

	/* Read temperature from the VTSens */
	ret = vtsens_read_temperature(0, &temperature);
	if (ret) {
		dev_err(miphy_phy->dev, "failed to get temperature\n");
		return ret;
	}

	/* Check that we are not out of range */
	if ((temperature < calib[0].temperature) ||
	    (temperature > calib[calib_nb - 1].temperature)) {
		dev_err(miphy_phy->dev,
			"temperature %d outside of range [%d - %d\n",
			temperature, calib[0].temperature,
			calib[calib_nb - 1].temperature);
		return -EINVAL;
	}

	for (i = 0; i < calib_nb; i++) {
		if (temperature > calib[i].temperature)
			continue;
		dev_info(miphy_phy->dev,
			 "Calibrate using temperature: %d, actual %d\n",
			 calib[i].temperature, temperature);
		*target = calib[i].target;
		break;
	}

	return 0;
}

/*
 * Initialize the Miphy-S for SGMII on 1 DS
 */
static int miphys_init_sgmii(struct miphys_phy *miphy_phy)
{
	struct miphys_cfg *cfg = &miphy_phy->cfg[miphy_phy->type];
	void __iomem *base = miphy_phy->base;
	unsigned long finish;
	u8 val = 0;
	int ret, i;
	unsigned int calfreq_target = 0;
	u32 reg;

	/* SYSCFG settings */
	/*
	 * Set CS_CONFIGURATION
	 * SGMII_TX_CLK = CLK_VCO divided by this value : 6 = div20
	 * -> 250mhx but there is internal div2 for sgmii path
	 */
	regmap_update_bits(miphy_phy->regmap, cfg->syscfg_cs_conf,
			   MIPHY_PLL_VCODIV_RATIO_MASK | MIPHY_PLL_VCODIV_PUP |
			   MIPHY_PLL_SSC_EN | MIPHY_CS_CLK_FREF_DIG_EN |
			   MIPHY_CS_REFDIV_RATIO_MASK | MIPHY_RC_IBIAS_PUP,
			   MIPHY_PLL_VCODIV_RATIO_20 | MIPHY_CS_REFDIV_RATIO_1 |
			   MIPHY_CS_CLK_FREF_DIG_EN | MIPHY_RC_IBIAS_PUP);

	/* allow all slices to output data */
	regmap_update_bits(miphy_phy->regmap, cfg->syscfg_tx_conf,
			   MIPHY_SGMII_TX_OUT_EN | MIPHY_SGMII_POWER_MODE_MASK |
			   MIPHY_SGMII_TX_C0_MASK,
			   MIPHY_SGMII_TX_OUT_EN | MIPHY_SGMII_POWER_MODE_1 |
			   MIPHY_SGMII_TX_C0_32);

	/* MIPHY registers */
	/* Reset the macro except the CPU registers */
	miphys_reset_ds(miphy_phy);

	/* Programming of the CS */
	writel_relaxed(0x3c, base + MIPHYS_DS(0, MIPHYS_PLL_CLKREF_FREQ));
	writel_relaxed(0x10, base + MIPHYS_DS(0, MIPHYS_TX_SUPPLY_CAL_MAN));
	writel_relaxed(0x0d, base + MIPHYS_DS(0, MIPHYS_RX_BUFFER_CTRL));
	writel_relaxed(0x1d, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_6));
	writel_relaxed(0x20, base + MIPHYS_DS(0, MIPHYS_PLL_CFG_0));
	writel_relaxed(0x6c, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_0));
	writel_relaxed(0x7f, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_1));
	writel_relaxed(0x1b, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_3));
	writel_relaxed(0x06, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_4));
	writel_relaxed(0x08, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_6));

	/* BIT(2) is not understood and comes from ST implementation. */
	reg = AUTO_RST | BIT(2);
	if (miphy_phy->polarity & PHY_POL_SWAP_RX)
		reg |= RX_POL;
	if (miphy_phy->polarity & PHY_POL_SWAP_TX)
		reg |= TX_POL;
	writel_relaxed(reg, base + MIPHYS_DS(0, MIPHYS_CONTROL));

	writel_relaxed(0x10, base + MIPHYS_DS(0, MIPHYS_BOUNDARY_2));

	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_CONF));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_SPEED));
	writel_relaxed(0x0b, base + MIPHYS_DS(0, MIPHYS_SYNCHAR_CONTROL));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_RX_RBL));
	writel_relaxed(0x41, base + MIPHYS_DS(0, MIPHYS_BOUNDARY_SEL));
	writel_relaxed(0x15, base + MIPHYS_DS(0, MIPHYS_TX_SWING));
	writel_relaxed(0xdf, base + MIPHYS_DS(0, MIPHYS_RXCLKGEN_BIAS_CTRL));
	writel_relaxed(0x03, base + MIPHYS_DS(0, MIPHYS_RXBUF_LC_CTRL));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_RX_MODSEL));
	writel_relaxed(0xa7, base + MIPHYS_DS(0, MIPHYS_RX_K_GAIN));
	writel_relaxed(0x9a, base + MIPHYS_DS(0, MIPHYS_RX_GAIN_IDX_MAN));
	writel_relaxed(0x22, base + MIPHYS_DS(0, MIPHYS_RX_SIGDET_SLEEP_SEL));
	writel_relaxed(0x22, base + MIPHYS_DS(0, MIPHYS_RX_SIGDET_WAIT_SEL));
	writel_relaxed(0x2a, base + MIPHYS_DS(0, MIPHYS_RX_SIGDET_DATA_SEL));
	writel_relaxed(0x01, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_1));
	writel_relaxed(0x0b, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_4));
	writel_relaxed(0x01, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_5));
	writel_relaxed(0x08, base + MIPHYS_DS(0, MIPHYS_PLL_CFG_3));
	writel_relaxed(0x70, base + MIPHYS_DS(0, MIPHYS_PLL_RFCLKPATH_0));
	writel_relaxed(0x6c, base + MIPHYS_DS(0, MIPHYS_PLL_RFCLKPATH_1));

	writel_relaxed(0x01, base + MIPHYS_DS(0, MIPHYS_RX_LOCK_CTRL_1));
	writel_relaxed(0x30, base + MIPHYS_DS(0, MIPHYS_RX_OFFSET_CTRL));
	writel_relaxed(0x7f, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_2));
	writel_relaxed(0x0a, base + MIPHYS_DS(0, MIPHYS_RX_STAT_LENGTH));
	writel_relaxed(0x07, base + MIPHYS_DS(0, MIPHYS_RX_LOCK_HOR_THRES_MAX));
	writel_relaxed(0x11, base + MIPHYS_DS(0, MIPHYS_RX_CAL_VGA_STEP));
	writel_relaxed(0xdf, base + MIPHYS_DS(0, MIPHYS_RX_STAT_LOCK_CTRL));
	writel_relaxed(0x03, base + MIPHYS_DS(0, MIPHYS_RX_LOCK_HOR_THRES));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_RX_LOCK_SETTING_OPT));

	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_CONF));
	writel_relaxed(0x29, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_0));
	writel_relaxed(0xaa, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_1));
	writel_relaxed(0xaa, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_2));
	writel_relaxed(0xaa, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_3));
	writel_relaxed(MIPHYS_TIME2CALFREQ_0(0x1f40),
		       base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_0));
	writel_relaxed(MIPHYS_TIME2CALFREQ_1(0x1f40),
		       base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_1));

	/* Get calfreq target value */
	ret = get_calfreq_target(miphy_phy, &calfreq_target);
	if (!ret) {
		writel_relaxed(MIPHYS_TARGET2CALFREQ_2(calfreq_target),
			       base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_2));
		writel_relaxed(MIPHYS_TARGET2CALFREQ_3(calfreq_target),
			       base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_3));
	} else {
		dev_err(miphy_phy->dev, "failed to get calreq target - use default value\n");
		writel_relaxed(MIPHYS_TARGET2CALFREQ_2(0x1f40),
				base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_2));
		writel_relaxed(MIPHYS_TARGET2CALFREQ_3(0x1f40),
				base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_3));
	}

	/* Miphy-S Pipe wrapper registers */
	writel_relaxed(0x40, base + 0x400 +
			     MIPHYS_DS(0, MIPHYS_PIPE_RX_CAL_CTRL_2));
	writel_relaxed(0xc8, base + 0x400 +
			     MIPHYS_DS(0, MIPHYS_PIPE_RX_CAL_CTRL_1));
	writel_relaxed(0x63, base + 0x400 +
			     MIPHYS_DS(0, MIPHYS_PIPE_RX_CAL_OFFSET_CTRL));
	writel_relaxed(0x0f, base + 0x400 +
			     MIPHYS_DS(0, MIPHYS_PIPE_RX_CAL_BYPASS_FINE));
	writel_relaxed(0x00, base + 0x400 +
			     MIPHYS_DS(0, MIPHYS_PIPE_RX_CAL_VGA_OFFSET));
	writel_relaxed(0xc4, base + 0x400 +
			     MIPHYS_DS(0, MIPHYS_PIPE_RX_CAL_CTRL_2));

	/* Releasing the reset of the DS */
	miphys_unreset_ds(miphy_phy);

	/* Put out of reset the data lane */
	for (i = 0; i < miphy_phy->cfg[miphy_phy->type].nb_rst; i++) {
		ret = reset_control_deassert(miphy_phy->miphy_ln_rst[i]);
		if (ret) {
			dev_err(miphy_phy->dev, "can't deassert lane%d reset\n",
				i);
			return ret;
		}
	}

	/* waiting pll lock */
	val = 0;
	finish = jiffies + MIPHYS_PLL_WAIT_S * HZ;
	do {
		val = readl_relaxed(base + MIPHYS_DS(0, MIPHYS_PLL_FDB_4));

		if (time_after_eq(jiffies, finish))
			return -EBUSY;
		cpu_relax();
	} while (!(val & PLL_LOCKED));

	/* enable pll_vco_pull_up (to have vco_clk_div toggling */
	regmap_update_bits(miphy_phy->regmap, cfg->syscfg_cs_conf,
			   MIPHY_PLL_VCODIV_PUP, MIPHY_PLL_VCODIV_PUP);

	return 0;
}

/*
 * Initialize the Miphy-S for Modmelink on 1 DS
 */
static int miphys_init_mdml(struct miphys_phy *miphy_phy)
{
	void __iomem *base = miphy_phy->base;
	struct miphys_cfg *cfg = &miphy_phy->cfg[miphy_phy->type];
	int ret;

	regmap_update_bits(miphy_phy->regmap, cfg->syscfg_cs_conf,
			   MIPHY_PLL_VCODIV_RATIO_MASK | MIPHY_PLL_VCODIV_PUP |
			   MIPHY_PLL_SSC_EN | MIPHY_CS_CLK_FREF_DIG_EN |
			   MIPHY_CS_REFDIV_RATIO_MASK | MIPHY_RC_IBIAS_PUP,
			   MIPHY_PLL_VCODIV_RATIO_16 | MIPHY_RC_IBIAS_PUP |
			   MIPHY_CS_CLK_FREF_DIG_EN);

	/* VCO DIV needs to be doubled when the doubler (below) is disabled */
	if (miphy_phy->frequency== PHY_MDML_FREQ_6Gbps) {
		regmap_update_bits(miphy_phy->regmap, cfg->syscfg_vco_div,
				   MIPHY_DIV_RATIO, 2);
	} else {
		regmap_update_bits(miphy_phy->regmap, cfg->syscfg_vco_div,
				   MIPHY_DIV_RATIO, 1);
	}

	writel_relaxed(0x07, base + MIPHYS_DS(0, MIPHYS_CONF_RESET));
	writel_relaxed(0x01, base + MIPHYS_DS(0, MIPHYS_CONF_RESET));
	writel_relaxed(0x3C, base + MIPHYS_DS(0, MIPHYS_PLL_CLKREF_FREQ));
	writel_relaxed(0x10, base + MIPHYS_DS(0, MIPHYS_TX_SUPPLY_CAL_MAN));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_RX_BUFFER_CTRL));
	writel_relaxed(0x1D, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_6));
	writel_relaxed(0x20, base + MIPHYS_DS(0, MIPHYS_PLL_CFG_0));
	writel_relaxed(0x6C, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_0));
	writel_relaxed(0x7F, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_1));
	writel_relaxed(0x1B, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_3));
	writel_relaxed(0x06, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_4));
	writel_relaxed(0x08, base + MIPHYS_DS(0, MIPHYS_PLL_ANA_6));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_CONF));
	writel_relaxed(0x0A, base + MIPHYS_DS(0, MIPHYS_SPEED));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_SYNCHAR_CONTROL));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_RX_RBL));
	writel_relaxed(0x41, base + MIPHYS_DS(0, MIPHYS_BOUNDARY_SEL));
	writel_relaxed(0x15, base + MIPHYS_DS(0, MIPHYS_TX_SWING));
	writel_relaxed(0xDF, base + MIPHYS_DS(0, MIPHYS_RXCLKGEN_BIAS_CTRL));
	writel_relaxed(0x03, base + MIPHYS_DS(0, MIPHYS_RXBUF_LC_CTRL));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_RX_MODSEL));
	writel_relaxed(0xA7, base + MIPHYS_DS(0, MIPHYS_RX_K_GAIN));
	writel_relaxed(0x55, base + MIPHYS_DS(0, MIPHYS_RX_GAIN_IDX_MAN));
	writel_relaxed(0x22, base + MIPHYS_DS(0, MIPHYS_RX_SIGDET_SLEEP_SEL));
	writel_relaxed(0x22, base + MIPHYS_DS(0, MIPHYS_RX_SIGDET_WAIT_SEL));
	writel_relaxed(0x2A, base + MIPHYS_DS(0, MIPHYS_RX_SIGDET_DATA_SEL));
	writel_relaxed(0x01, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_1));
	writel_relaxed(0x0B, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_4));
	writel_relaxed(0x01, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_5));
	writel_relaxed(0x09, base + MIPHYS_DS(0, MIPHYS_PLL_CFG_3));
	/* Disable the doubler and rmin_sel when running below 8 Gbps */
	if (miphy_phy->frequency== PHY_MDML_FREQ_6Gbps)
		writel_relaxed(0xB0, base + MIPHYS_DS(0, MIPHYS_PLL_RFCLKPATH_0));
	else
		writel_relaxed(0xB9, base + MIPHYS_DS(0, MIPHYS_PLL_RFCLKPATH_0));
	writel_relaxed(0x6C, base + MIPHYS_DS(0, MIPHYS_PLL_RFCLKPATH_1));

	writel_relaxed(0x02, base + MIPHYS_DS(0, MIPHYS_RX_LOCK_CTRL_1));
	writel_relaxed(0x30, base + MIPHYS_DS(0, MIPHYS_RX_OFFSET_CTRL));
	writel_relaxed(0x7F, base + MIPHYS_DS(0, MIPHYS_RX_POWER_CTRL_2));
	writel_relaxed(0x0A, base + MIPHYS_DS(0, MIPHYS_RX_STAT_LENGTH));
	writel_relaxed(0x11, base + MIPHYS_DS(0, MIPHYS_RX_CAL_VGA_STEP));
	writel_relaxed(0xDF, base + MIPHYS_DS(0, MIPHYS_RX_STAT_LOCK_CTRL));
	writel_relaxed(0x03, base + MIPHYS_DS(0, MIPHYS_RX_LOCK_HOR_THRES));

	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_CONF));

	switch (miphy_phy->frequency) {
	case PHY_MDML_FREQ_6Gbps:
		dev_info(miphy_phy->dev, "Running modemlink at 6Gbps\n");
		writel_relaxed(0x32, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_0));
		writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_1));
		writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_2));
		writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_3));
		break;
	case PHY_MDML_FREQ_8Gbps:
		dev_info(miphy_phy->dev, "Running modemlink at 8Gbps\n");
		writel_relaxed(0x21, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_0));
		writel_relaxed(0x55, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_1));
		writel_relaxed(0x55, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_2));
		writel_relaxed(0x55, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_3));
		break;
	case PHY_MDML_FREQ_9p96Gbps:
		dev_info(miphy_phy->dev, "Running modemlink at 9.96Gbps\n");
		writel_relaxed(0x29, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_0));
		writel_relaxed(0x80, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_1));
		writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_2));
		writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_3));
		break;
	case PHY_MDML_FREQ_10p3125Gbps:
		dev_info(miphy_phy->dev, "Running modemlink at 10.03125Gbps\n");
		writel_relaxed(0x2a, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_0));
		writel_relaxed(0xf8, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_1));
		writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_2));
		writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_RATIO_3));
		break;
	default:
		dev_err(miphy_phy->dev, "Invalid MDML Frequency %d\n",
			miphy_phy->frequency);
		return -EINVAL;
	}

	writel_relaxed(0xFA, base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_0));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_1));
	writel_relaxed(0xFA, base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_2));
	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_PLL_CALFREQ_3));

	writel_relaxed(0x40, base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_CTRL_2));
	writel_relaxed(0xC8, base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_CTRL_1));

	writel_relaxed(0x63,
		       base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_OFFSET_CTRL));

	writel_relaxed(0x0F,
		       base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_BYPASS_FINE));

	writel_relaxed(0x00,
		       base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_VGA_OFFSET));

	writel_relaxed(0xC4, base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_CTRL_2));

	writel_relaxed(0x00, base + MIPHYS_DS(0, MIPHYS_CONF_RESET));

	/* Wait for phy_ready */
	ret = miphy_is_ready(miphy_phy);
	if (!ret) {
		/* After PLL is locked, power up VCODIV. */
		regmap_update_bits(miphy_phy->regmap, cfg->syscfg_cs_conf,
				   MIPHY_PLL_VCODIV_PUP,
				   MIPHY_PLL_VCODIV_PUP);
	}

	return ret;
}

static int miphys_init(struct phy *phy)
{
	struct miphys_phy *miphy_phy = phy_get_drvdata(phy);
	struct miphys_cfg *cfg = &miphy_phy->cfg[miphy_phy->type];
	void __iomem *base = miphy_phy->base;
	int ret;
	u16 release;

	if (miphy_phy->externally_initialized) {
		dev_info(miphy_phy->dev,
			 "Skipping initializing as miphy previously initialized\n");
		schedule_delayed_work(&miphy_phy->eye.work, MIPHY_EYE_POLL_RATE);
		miphy_phy->externally_initialized = false;

		return 0;
	}

	/* Select the Mode for the MIPHY */
	switch (miphy_phy->type) {
	case PHY_TYPE_SGMII:
		/* Select SGMII over the MIPHY */
		regmap_update_bits(miphy_phy->regmap,
				   cfg->syscfg_lnk_conf,
				   MIPHY_EXT_MIPHY_CTRL,
				   MIPHY_EXT_MIPHY_CTRL_SGMII);
		break;
	case PHY_TYPE_MDML:
		/* Select Modemlink over the MIPHY */
		regmap_update_bits(miphy_phy->regmap,
				   cfg->syscfg_lnk_conf,
				   MIPHY_EXT_MIPHY_CTRL,
				   MIPHY_EXT_MIPHY_CTRL_MDML);
		break;
	default:
		dev_err(miphy_phy->dev, "Unsupported type (0x%x)\n",
			miphy_phy->type);
		return -EINVAL;
	}

	/* Enabling Miphy clocks */
	regmap_update_bits(miphy_phy->regmap, cfg->syscfg_clk,
			   MIPHY_CS_CLK_REF_INT_EN | MIPHY_CLKEN,
			   MIPHY_CS_CLK_REF_INT_EN | MIPHY_CLKEN);

	/* Put the miphy out of reset */
	ret = reset_control_deassert(miphy_phy->miphy_rst);
	if (ret) {
		dev_err(miphy_phy->dev, "can't deassert reset\n");
		return ret;
	}

	ret = reset_control_deassert(miphy_phy->miphy_osc_rst);
	if (ret) {
		dev_err(miphy_phy->dev, "can't deassert osc reset\n");
		return ret;
	}

	switch (miphy_phy->type) {
	case PHY_TYPE_SGMII:
		dev_info(miphy_phy->dev, "initialize Miphy-S in SGMII mode\n");
		ret = miphys_init_sgmii(miphy_phy);
		break;
	case PHY_TYPE_MDML:
		dev_info(miphy_phy->dev, "initialize Miphy-S in MDML mode\n");
		ret = miphys_init_mdml(miphy_phy);
		break;
	default:
		dev_info(miphy_phy->dev, "un-supported Miphy-S mode (0x%x)\n",
			 miphy_phy->type);
		ret = -EINVAL;
		break;
	}

	/* read MIPHY release */
	release = readl_relaxed(base + MIPHYS_DS(0, MIPHY_REVISION));
	release |= MIPHY_VER(readl_relaxed(base + MIPHYS_DS(0, MIPHY_VERSION)));
	dev_info(miphy_phy->dev, "MiPHY-S release %#x init done\n", release);

	schedule_delayed_work(&miphy_phy->eye.work, MIPHY_EYE_POLL_RATE);

	return ret;
}

/*
 * Get reset information from the DT
 */
static int miphys_probe_resets(struct miphys_phy *miphy_phy)
{
	char reset_name[20];
	int i;

	/*
	 * Global Miphy reset.  Only perform once per phy, even if the phy is
	 * shared from multiple drivers.
	 */
	if (IS_ERR_OR_NULL(miphy_phy->miphy_rst)) {
		miphy_phy->miphy_rst = devm_reset_control_get(miphy_phy->dev,
							      "miphy-sw-rst");
	}
	if (IS_ERR(miphy_phy->miphy_rst)) {
		dev_err(miphy_phy->dev,
			"miphy soft reset control not defined\n");
		return PTR_ERR(miphy_phy->miphy_rst);
	}

	/* Oscillator reset */
	if (IS_ERR_OR_NULL(miphy_phy->miphy_osc_rst)) {
		miphy_phy->miphy_osc_rst = devm_reset_control_get(miphy_phy->dev,
								  "miphy-osc-rst");
	}
	if (IS_ERR(miphy_phy->miphy_osc_rst)) {
		dev_err(miphy_phy->dev,
			"miphy osc reset control not defined\n");
		return PTR_ERR(miphy_phy->miphy_osc_rst);
	}

	for (i = 0; i < miphy_phy->cfg[miphy_phy->type].nb_rst; i++) {
		snprintf(reset_name, sizeof(reset_name), "miphy-ln%d-rst", i);

		if (IS_ERR_OR_NULL(miphy_phy->miphy_ln_rst[i])) {
			miphy_phy->miphy_ln_rst[i] = devm_reset_control_get(
						miphy_phy->dev, reset_name);
		}
		if (IS_ERR(miphy_phy->miphy_ln_rst[i])) {
			dev_err(miphy_phy->dev,
				"%s reset control not defined\n",
				reset_name);
			return PTR_ERR(miphy_phy->miphy_ln_rst[i]);
		}
	}

	return 0;
}

static struct phy *miphys_xlate(struct device *dev,
				struct of_phandle_args *args)
{
	struct miphys_phy *miphy_phy = dev_get_drvdata(dev);
	int ret;
	int polarity = PHY_POL_SWAP_NONE;
	int frequency = PHY_MDML_FREQ_6Gbps;

	if (args->args_count < 1 || args->args_count > 3) {
		dev_err(dev, "Invalid number of cells in 'phy' property\n");
		return ERR_PTR(-EINVAL);
	}

	if ((args->args[0] >= PHY_MODES_NUM) ||
	    (!miphy_phy->cfg[args->args[0]].valid)) {
		dev_err(dev, "Unsupported phy mode %d\n", args->args[0]);
		return ERR_PTR(-EINVAL);
	}

	if (args->args_count >= 2)
		frequency = args->args[1];
	if (args->args_count >= 3)
		polarity = args->args[2];

	/*
	 * Make sure that if the phy was previously configured, it was
	 * configured for the same scenario.
	 */
	if (miphy_phy->type != PHY_NONE) {
		if (miphy_phy->frequency != frequency ||
		    miphy_phy->polarity != polarity) {
			dev_err(dev, "Conflicting PHY request type\n");
			return ERR_PTR(-EINVAL);
		}
	}

	miphy_phy->type = args->args[0];
	miphy_phy->frequency = frequency;
	miphy_phy->polarity = polarity;

	ret = miphys_probe_resets(miphy_phy);
	if (ret) {
		dev_err(miphy_phy->dev, "Failed to probe reset information\n");
		return NULL;
	}

	return miphy_phy->phy;
}

static int miphys_reset(struct phy *phy)
{
	struct miphys_phy *miphy_phy = phy_get_drvdata(phy);
	u32 reset_reg = readl_relaxed(miphy_phy->base + MIPHYS_DS(0, MIPHYS_RESET));
	u32 cal_ctrl = readl(miphy_phy->base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_CTRL_2));

	/* Don't allow soft resets if the device isn't already initialized */
	if (phy->init_count == 0) {
		return -EPERM;
	}

	reset_reg |= RST_RX_CAL_SW | RST_RX_DIG_SW | RST_RX_CLK_SW;
	writel(reset_reg, miphy_phy->base + MIPHYS_DS(0, MIPHYS_RESET));

	writel(cal_ctrl & ~RST_RX_CAL_N, miphy_phy->base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_CTRL_2));
	writel(cal_ctrl | RST_RX_CAL_N, miphy_phy->base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_CTRL_2));

	reset_reg &= ~RST_RX_CLK_SW;
	writel(reset_reg, miphy_phy->base + MIPHYS_DS(0, MIPHYS_RESET));

	reset_reg &= ~RST_RX_CAL_SW;
	writel(reset_reg, miphy_phy->base + MIPHYS_DS(0, MIPHYS_RESET));

	reset_reg &= ~RST_RX_DIG_SW;
	writel(reset_reg, miphy_phy->base + MIPHYS_DS(0, MIPHYS_RESET));

	return 0;
}

static const struct phy_ops miphys_ops = {
	.init = miphys_init,
	.reset = miphys_reset,
	.owner = THIS_MODULE,
};

static void miphy_phy_scan_eye(struct work_struct *work)
{
	struct miphys_phy *miphy_phy = container_of(work, struct miphys_phy,
						    eye.work.work);

	if (readl(miphy_phy->base + MIPHYS_DS(1, MIPHY_PIPE_RX_CAL_VER_EYE_LEFT)) &
	    MIPHY_RX_CAL_VER_EYE_LEFT_BOUND_COMPL) {
		miphy_phy->eye.left = readl(miphy_phy->base +
			       MIPHYS_DS(1, MIPHY_PIPE_RX_CAL_VER_EYE_LEFT)) &
			 GENMASK(4, 0);
		miphy_phy->eye.right = readl(miphy_phy->base +
			       MIPHYS_DS(1, MIPHY_PIPE_RX_CAL_VER_EYE_RIGHT)) &
			 GENMASK(4, 0);
	} else {
		miphy_phy->eye.left = 0;
		miphy_phy->eye.right = 0;
	}

	/* Restart the scan */
	writel_relaxed(0xC0,
		       miphy_phy->base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_CTRL_2));
	writel_relaxed(0xC4,
		       miphy_phy->base + MIPHYS_DS(1, MIPHYS_PIPE_RX_CAL_CTRL_2));

	schedule_delayed_work(&miphy_phy->eye.work, MIPHY_EYE_POLL_RATE);
}

static ssize_t show_left_horiz_eye_opening(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct miphys_phy *miphy_phy = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", miphy_phy->eye.left);
}
static DEVICE_ATTR(horiz_eye_opening_left, 0444, show_left_horiz_eye_opening, NULL);

static ssize_t show_right_horiz_eye_opening(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct miphys_phy *miphy_phy = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", miphy_phy->eye.right);
}
static DEVICE_ATTR(horiz_eye_opening_right, 0444, show_right_horiz_eye_opening, NULL);

static struct attribute *miphys_sysfs_device_attrs[] = {
	&dev_attr_horiz_eye_opening_left.attr,
	&dev_attr_horiz_eye_opening_right.attr,

	/* Sentinel */
	NULL,
};

static const struct attribute_group miphys_sysfs_regs_group = {
	.attrs = miphys_sysfs_device_attrs,
};

static int miphys_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct miphys_phy *miphy_phy;
	struct phy_provider *provider;
	struct resource *res;
	int ret;
	int temperature;

	/* vtsens is required due to temperature specific calibration */
	ret = vtsens_read_temperature(0, &temperature);
	if (ret != 0) {
		return ret;
	}

	miphy_phy = devm_kzalloc(&pdev->dev, sizeof(*miphy_phy), GFP_KERNEL);
	if (!miphy_phy)
		return -ENOMEM;

	miphy_phy->dev = &pdev->dev;
	dev_set_drvdata(miphy_phy->dev, miphy_phy);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	miphy_phy->base = devm_ioremap_resource(miphy_phy->dev, res);
	if (IS_ERR(miphy_phy->base)) {
		dev_err(miphy_phy->dev, "failed to remap IO\n");
		return PTR_ERR(miphy_phy->base);
	}

	miphy_phy->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(miphy_phy->regmap)) {
		dev_err(miphy_phy->dev, "No syscfg phandle specified\n");
		return PTR_ERR(miphy_phy->regmap);
	}

	/* Get the miphys_cfg */
	miphy_phy->cfg = (struct miphys_cfg *)
		of_match_node(miphys_of_match, np)->data;
	if (!miphy_phy->cfg) {
		dev_err(miphy_phy->dev, "unable to get cfg table\n");
		ret = -EIO;
		goto error;
	}

	miphy_phy->externally_initialized = of_property_read_bool(np,
						"externally-initialized");

	miphy_phy->phy = devm_phy_create(miphy_phy->dev, np, &miphys_ops);
	if (IS_ERR(miphy_phy->phy)) {
		dev_err(miphy_phy->dev, "failed to create PHY\n");
		ret = PTR_ERR(miphy_phy->phy);
		goto error;
	}
	phy_set_drvdata(miphy_phy->phy, miphy_phy);

	provider = devm_of_phy_provider_register(miphy_phy->dev, miphys_xlate);
	if (IS_ERR(provider)) {
		dev_err(miphy_phy->dev, "failed to register PHY\n");
		ret = PTR_ERR(provider);
		goto error;
	}

	miphy_phy->eye.left = 0;
	miphy_phy->eye.right = 0;
	INIT_DELAYED_WORK(&miphy_phy->eye.work, miphy_phy_scan_eye);

	ret = sysfs_create_group(&pdev->dev.kobj, &miphys_sysfs_regs_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed creating sysfs (%d)\n", ret);
		goto error;
	}

	dev_info(miphy_phy->dev, "Miphy-S probed\n");
	return 0;

error:
	/* of_node registered as part of syscon_regmap_lookup_by_phandle */
	of_node_put(np);
	return ret;
}

static int miphys_remove(struct platform_device *pdev)
{
	struct miphys_phy *miphy_phy = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&miphy_phy->eye.work);

	sysfs_remove_group(&pdev->dev.kobj, &miphys_sysfs_regs_group);

	return 0;
}

/* Temperature dependent calibration is currently unknown */
const struct calib_target sgmii_target[] = {
	{ -50, 0x1f40 },
	{ 125, 0x1f40 },
};

static const struct miphys_cfg gllcff_miphys1_cfg[PHY_MODES_NUM] = {
	/* SGMII - 1 DS */
	[PHY_TYPE_SGMII] = {1, 1, 1, SYSCFG_MIPHY_1_LINK_CONFIGURATION,
			    SYSCFG_MIPHY1_CLK, SYSCFG_MIPHY1_CS_CONF,
			    SYSCFG_MIPHY1_SGMII_TX_CONF,
			    SYSCFG_MIPHY1_DIV_RATIO,
			    sgmii_target, ARRAY_SIZE(sgmii_target)},
	[PHY_TYPE_MDML] = {1, 1, 1, SYSCFG_MIPHY_1_LINK_CONFIGURATION,
			   SYSCFG_MIPHY1_CLK, SYSCFG_MIPHY1_CS_CONF,
			   SYSCFG_MIPHY1_SGMII_TX_CONF,
			   SYSCFG_MIPHY1_DIV_RATIO,
			   sgmii_target, ARRAY_SIZE(sgmii_target)},
};

static const struct miphys_cfg gllcff_miphys2_cfg[PHY_MODES_NUM] = {
	/* SGMII - 1 DS */
	[PHY_TYPE_SGMII] = {1, 1, 1, SYSCFG_MIPHY_2_LINK_CONFIGURATION,
			    SYSCFG_MIPHY2_CLK, SYSCFG_MIPHY2_CS_CONF,
			    SYSCFG_MIPHY2_SGMII_TX_CONF,
			    SYSCFG_MIPHY2_DIV_RATIO,
			    sgmii_target, ARRAY_SIZE(sgmii_target)},
	[PHY_TYPE_MDML] =  {1, 1, 1, SYSCFG_MIPHY_2_LINK_CONFIGURATION,
			    SYSCFG_MIPHY2_CLK, SYSCFG_MIPHY2_CS_CONF,
			    SYSCFG_MIPHY2_SGMII_TX_CONF,
			    SYSCFG_MIPHY2_DIV_RATIO,
			    sgmii_target, ARRAY_SIZE(sgmii_target)},
};

static const struct of_device_id miphys_of_match[] = {
	{
		.compatible = "st,gllcff-miphys1-phy",
		.data = gllcff_miphys1_cfg,
	},
	{
		.compatible = "st,gllcff-miphys2-phy",
		.data = gllcff_miphys2_cfg,
	},
	{},
};
MODULE_DEVICE_TABLE(of, miphys_of_match);

static struct platform_driver miphys_driver = {
	.probe = miphys_probe,
	.remove = miphys_remove,
	.driver = {
		.name = "miphys-phy",
		.of_match_table = miphys_of_match,
	}
};
module_platform_driver(miphys_driver);

MODULE_AUTHOR("Alain Volmat <alain.volmat@st.com>");
MODULE_DESCRIPTION("STMicroelectronics miphys driver");
MODULE_LICENSE("GPL v2");
