/*
 * Copyright (C) 2016 STMicroelectronics (R&D) Limited
 * Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <dt-bindings/reset/gllcff-resets.h>
#include <dt-bindings/soc/gllcff-syscfg.h>
#include "reset-syscfg.h"

#define DRIVER_NAME "reset-gllcff"

static const char syscfg_fc0_backbone[] = "st,gllcff-fc0-backbone-syscfg";
static const char syscfg_fc11_bootdev[] = "st,gllcff-fc11-bootdev-syscfg";
static const char syscfg_fc7_hsif_bank1[] = "st,gllcff-fc7-hsif-bank1-syscfg";
static const char syscfg_fc2_modem[] = "st,gllcff-fc2-modem-syscfg";

static const struct syscfg_reset_channel_data gllcff_softresets[] = {
	/* Backbone DMA device */
	[GLLCFF_FC0_DMA0_RESET] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc0_backbone, SYSCFG_1, 0),
	/* CAN devices */
	[GLLCFF_FC7_RST_CAN_0_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7000, 0),
	[GLLCFF_FC7_RST_CAN_1_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7010, 0),
	/* GPS/IMU Cut2 Only */
	[GLLCFF_FC7_RST_PPSGEN_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7020, 0),
	[GLLCFF_FC7_RST_TSP_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7020, 1),
	/* PCIe only cut1 */
	[GLLCFF_FC7_RST_PCIE_0_CONTROLLER_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7050, 0),
	/* GMAC0 */
	[GLLCFF_FC7_RST_GMAC_0_CTRL_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7070, 1),
	[GLLCFF_FC7_RST_GMAC_0_APP_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7070, 0),
	/* EPHY */
	[GLLCFF_FC7_RST_EPHY_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7409, 0),
	/* GMAC1 */
	[GLLCFF_FC7_RST_GMAC_1_CTRL_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7080, 1),
	[GLLCFF_FC7_RST_GMAC_1_APP_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7080, 0),
	/* MIPHY0 only cut 1*/
	[GLLCFF_FC7_RST_CPL_L2_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7090, 6),
	[GLLCFF_FC7_RST_CPL_L1_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7090, 5),
	[GLLCFF_FC7_RST_CPL_L0_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7090, 4),
	[GLLCFF_FC7_RST_PCIE_0_PIPE_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7090, 2),
	[GLLCFF_FC7_RST_MIPHY_0_OSC_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7090, 1),
	[GLLCFF_FC7_RST_MIPHY_0_GLOBAL_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7090, 0),
	/* MIPHY1 */
	[GLLCFF_FC7_RST_MODEM_LINK_L0_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7100, 3),
	[GLLCFF_FC7_RST_SGMII_0_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7100, 2),
	[GLLCFF_FC7_RST_MIPHY_1_OSC_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7100, 1),
	[GLLCFF_FC7_RST_MIPHY_1_GLOBAL_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7100, 0),
	/* MIPHY2 */
	[GLLCFF_FC7_RST_MODEM_LINK_L1_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7110, 3),
	[GLLCFF_FC7_RST_SGMII_1_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7110, 2),
	[GLLCFF_FC7_RST_MIPHY_2_OSC_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7110, 1),
	[GLLCFF_FC7_RST_MIPHY_2_GLOBAL_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc7_hsif_bank1, SYSCFG_7110, 0),
	/* MMC */
	[GLLCFF_FC11_FLASH_SUBSYSTEM_RESET] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc11_bootdev, SYSCFG_11003, 0),
	[GLLCFF_FC11_FLASH_SUBSYSTEM_PHY_RESET] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc11_bootdev, SYSCFG_11003, 1),
	/* Modem */
	[GLLCFF_FC2_RST_SCP_MASTER_N] =
	    _SYSCFG_RST_CH_NO_ACK(syscfg_fc2_modem, SYSCFG_2017, 16),
};

static const struct syscfg_reset_controller_data gllcff_softreset = {
	.wait_for_ack = false,
	.active_low = true,
	.nr_channels = ARRAY_SIZE(gllcff_softresets),
	.channels = gllcff_softresets,
};

static const struct of_device_id gllcff_reset_match[] = {
	{
		.compatible = "st,gllcff-softreset",
		.data = &gllcff_softreset,
	},
	{},
};

static struct platform_driver gllcff_reset_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = gllcff_reset_match,
	},
	.probe = syscfg_reset_probe,
};

static int __init gllcff_reset_init(void)
{
	return platform_driver_register(&gllcff_reset_driver);
}

arch_initcall(gllcff_reset_init);
