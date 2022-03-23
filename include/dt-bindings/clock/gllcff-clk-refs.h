/*
 * This header for constants clk index STMicroelectronics GLLCFF SoC.
 */
#ifndef _DT_BINDINGS_CLK_GLLCFF
#define _DT_BINDINGS_CLK_GLLCFF

/* CLOCKGEN C0 */
/* Output clocks */
#define CLK_EXT2F		0
/* div_1 spare */
#define CLK_ICN_CPU		2
#define CLK_ICN_ESRAM		3
#define CLK_ICN_ROM		3
/* div_4 spare */
#define CLK_PERIPH_0		5
#define CLK_SPI			6
#define CLK_XIN_EMMC		7
#define CLK_ICN_DMA_CONN	8
#define CLK_ICN_MAIN		9
/* div_10 spare */
#define CLK_ICN_MDM_NTW		11
#define CLK_ICN_BOOTDEV		12
#define CLK_PERIPH_1		13
#define CLK_PERIPH_2		14
#define CLK_ATB			15
#define CLK_LPC			16 /* Cut 2 Only */
#define CLK_1PPS		17 /* Cut 2 Only */
/* div_18 spare */
#define CLK_PHY_GMAC_0		19
/* div_20 spare */
#define CLK_PROC_NTW		21
#define CLK_PROC_DMA		22
#define CLK_EXT_DDR		23
#define CLK_TPIU		24
#define CLK_AXIM_MDMTX		25
#define CLK_AXIM_MDMRX		26
#define CLK_PTP_REF_GMAC_0	27
#define CLK_PTP_REF_GMAC_1	28
#define CLK_TSP			29 /* Cut 2 Only */
/* div_30, div_31 spare */

/* GFG0 (PLL_0) outputs */
#define CLK_S_C0_PLL0_ODF_0	0
/* GFG1 (PLL_1) outputs */
#define CLK_S_C0_PLL1_ODF_0	0
/* GFG2 (4FS600) outputs */
#define CLK_S_C0_4FS_CH0	0
#define CLK_S_C0_4FS_CH1	1
#define CLK_S_C0_4FS_CH2	2
#define CLK_S_C0_4FS_CH3	3

#endif /* _DT_BINDINGS_CLK_GLLCFF */
