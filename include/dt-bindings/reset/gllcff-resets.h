/*
 * Copyright (C) 2016 STMicroelectronics (R&D) Limited
 * Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _DT_BINDINGS_RESET_CONTROLLER_GLLCFF
#define _DT_BINDINGS_RESET_CONTROLLER_GLLCFF

/* Backbone */
#define GLLCFF_FC0_DMA0_RESET			0

/* Bootdev */
#define GLLCFF_FC11_FLASH_SUBSYSTEM_RESET	1
#define GLLCFF_FC11_FLASH_SUBSYSTEM_PHY_RESET	2

/* Connectivity */
#define	GLLCFF_FC7_RST_CAN_0_N			3
#define	GLLCFF_FC7_RST_CAN_1_N			4
#define	GLLCFF_FC7_RST_PPSGEN_N			5
#define	GLLCFF_FC7_RST_TSP_N			6
#define	GLLCFF_FC7_RST_PCIE_0_CONTROLLER_N	7 /* only cut 1 */
#define	GLLCFF_FC7_RST_GMAC_1_CTRL_N		8
#define	GLLCFF_FC7_RST_GMAC_1_APP_N		9

/* MiPHY0 */
#define	GLLCFF_FC7_RST_CPL_L2_N			10 /* only cut 1*/
#define	GLLCFF_FC7_RST_CPL_L1_N			11 /* only cut 1*/
#define	GLLCFF_FC7_RST_CPL_L0_N			12 /* only cut 1*/
#define	GLLCFF_FC7_RST_PCIE_0_PIPE_N		13 /* only cut 1*/
#define	GLLCFF_FC7_RST_MIPHY_0_OSC_N		14 /* only cut1 */
#define	GLLCFF_FC7_RST_MIPHY_0_GLOBAL_N		15 /* only cut1 */

/* MiPHY1 */
#define	GLLCFF_FC7_RST_MODEM_LINK_L0_N		16
#define	GLLCFF_FC7_RST_SGMII_0_N		17
#define	GLLCFF_FC7_RST_MIPHY_1_OSC_N		18
#define	GLLCFF_FC7_RST_MIPHY_1_GLOBAL_N		19

/* MiPHY2 */
#define	GLLCFF_FC7_RST_MODEM_LINK_L1_N		20
#define	GLLCFF_FC7_RST_SGMII_1_N		21
#define	GLLCFF_FC7_RST_MIPHY_2_OSC_N		22
#define	GLLCFF_FC7_RST_MIPHY_2_GLOBAL_N		23

/* EPhy */
#define GLLCFF_FC7_RST_EPHY_N			24

/* GMAC 0 */
#define	GLLCFF_FC7_RST_GMAC_0_CTRL_N		25
#define	GLLCFF_FC7_RST_GMAC_0_APP_N		26

/* Modem */
#define GLLCFF_FC2_RST_SCP_MASTER_N		27

#endif /* _DT_BINDINGS_RESET_CONTROLLER_GLLCFF */
