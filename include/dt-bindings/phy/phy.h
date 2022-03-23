/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * This header provides constants for the phy framework
 *
 * Copyright (C) 2014 STMicroelectronics
 * Author: Gabriel Fernandez <gabriel.fernandez@st.com>
 */

#ifndef _DT_BINDINGS_PHY
#define _DT_BINDINGS_PHY

#define PHY_NONE		0
#define PHY_TYPE_SATA		1
#define PHY_TYPE_PCIE		2
#define PHY_TYPE_USB2		3
#define PHY_TYPE_USB3		4
#define PHY_TYPE_UFS		5
#define PHY_TYPE_DP		6
#define PHY_TYPE_XPCS		7
#define PHY_TYPE_SGMII		8
#define PHY_TYPE_QSGMII		9
#define PHY_TYPE_MDML		10


#define PHY_FREQ_DEFAULT		0
#define PHY_MDML_FREQ_6Gbps		1
#define PHY_MDML_FREQ_8Gbps		2
#define PHY_MDML_FREQ_9p96Gbps		3
#define PHY_MDML_FREQ_10p3125Gbps	4

#define PHY_POL_SWAP_NONE       0
#define PHY_POL_SWAP_RX         1
#define PHY_POL_SWAP_TX         2
#define PHY_POL_SWAP_BOTH       3

#endif /* _DT_BINDINGS_PHY */
