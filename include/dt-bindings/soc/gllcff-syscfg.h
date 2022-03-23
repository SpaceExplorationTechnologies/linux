/*
 * Copyright (C) 2016 STMicroelectronics (R&D) Limited
 * Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _DT_BINDINGS_STI_GLLCFF
#define _DT_BINDINGS_STI_GLLCFF

/* BACKBONE */
#define SYSCFG_1	0x4	/* DMA0_RST */
#define SYSCFG_5	0x14	/* DMA0_CLK */
#define SYSCFG_419	0x13004C/* RESET_GEN_CONTROL */
#define SYSCFG_420	0x130050/* RESET_GEN_STATUS */

/* BOOTDEV */
#define SYSCFG_11003	0x20000	/* FLASHSS_BOOT_0_CLK_RST */
#define SYSCFG_11009	0x80000	/* FLASHSS_BOOT_0_SPI_CLK */
#define SYSCFG_11010	0x90000	/* FLASHSS_BOOT_0_EMMC_CLK */
#define SYSCFG_11011	0xA0000	/* FLASHSS_BOOT_0_SPI2_CLK */

/* CONNECTIVITY */
#define	SYSCFG_7000	0x0	/* CAN_0_RST */
#define	SYSCFG_7005	0x8000	/* CAN_CLK */
#define	SYSCFG_7010	0x10000	/* CAN_1_RST */
#define	SYSCFG_7020	0x20000	/* ONEPPS_0_RST */
#define	SYSCFG_7050	0x50000	/* PCIE_0_RST */
#define	SYSCFG_7055	0x58000	/* PCIE_0_CLK */
#define	SYSCFG_7070	0x70000	/* GMAC_0_RST */
#define	SYSCFG_7075	0x78000	/* GMAC_0_CLK */
#define	SYSCFG_7080	0x80000	/* GMAC_1_RST */
#define	SYSCFG_7085	0x88000	/* GMAC_1_CLK */
#define	SYSCFG_7090	0x90000	/* MIPHY_0_RST */
#define	SYSCFG_7095	0x98000	/* MIPHY_0_CLK */
#define	SYSCFG_7100	0xA0000	/* MIPHY_1_RST */
#define	SYSCFG_7105	0xA8000	/* MIPHY_1_CLK */
#define	SYSCFG_7110	0xB0000	/* MIPHY_2_RST */
#define	SYSCFG_7115	0xB8000	/* MIPHY_2_CLK */
#define	SYSCFG_7409	0x130024	/* EPHY_0_CONTROL */

#define SYSCFG_2017	0x44 /* SCP_MASTER_0_CONTROL_RST_CLK */

#endif /* _DT_BINDINGS_STI_GLLCFF */
