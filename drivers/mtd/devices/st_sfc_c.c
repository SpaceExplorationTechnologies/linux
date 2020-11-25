/*
 * ST Serial Flash Controller Rev C
 *
 * Author: Christophe Kerello <christophe.kerello@st.com>
 *
 * Copyright (C) 2010-2015 STMicroelectronics Limited
 *
 * JEDEC probe based on drivers/mtd/devices/m25p80.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/sched.h>
#include <linux/sort.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>
#include <linux/ibi.h>

#include "serial_flash_cmds.h"

#define DMA_BUFFER_SIZE			8192

/* SFC Controller Registers */
#define SFC_MODE_SELECT			0x0000
#define SFC_CLOCK_CONTROL		0x0004
#define SFC_INTERRUPT_ENABLE		0x000c
#define SFC_INTERRUPT_STATUS		0x0010
#define SFC_INTERRUPT_CLEAR		0x0014
#define SFC_TX_STEP_DELAY		0x001c
#define SFC_RX_STEP_DELAY		0x0020
#define SFC_IP_SW_RESET			0x0024
#define SFC_IP_STATUS			0x0028
#define SFC_FLASH_ADDRESS_CONFIG	0x0030
#define SFC_READ_FLASH_STATUS_CONFIG	0x0034
#define SFC_CHECK_FLASH_STATUS_CONFIG	0x0038
#define SFC_FLASH_STATUS_DATA		0x003c
#define SFC_POLL_SCHEDULER_0		0x0040
#define SFC_CLOCK_DIVISION		0x0050
#define SFC_TRANSMIT_CONFIG_CDM		0x0060
#define SFC_CONFIG_CDM			0x0064
#define SFC_DATA_CDM			0x0068
#define SFC_OPCODE_0			0x0100
#define SFC_OPCODE_1			0x0104
#define SFC_OPCODE_2			0x0108
#define SFC_OPCODE_3			0x010c
#define SFC_OPCODE_4			0x0110
#define SFC_OPCODE_5			0x0114
#define SFC_FLASH_ADDRESS		0x0118
#define SFC_SEQUENCE_0			0x011c
#define SFC_SEQUENCE_1			0x0120
#define SFC_SEQUENCE_2			0x0124
#define SFC_SEQUENCE_3			0x0128
#define SFC_DATA_INST_SIZE		0x012c
#define SFC_SYSTEM_MEMORY_ADDRESS	0x0130
#define SFC_SEQUENCE_OTP_0		0x0134
#define SFC_SEQUENCE_OTP_1		0x0138
#define SFC_SEQUENCE_CONFIG		0x013c

/* Register: SFC_MODE_SELECT */
#define SFC_BOOT_ENABLE			BIT(0)
#define SFC_CPU_MODE			BIT(1)
#define SFC_DMA_MODE			BIT(2)
#define SFC_WR_HS_NOT_DS		BIT(3)
#define SFC_RD_HS_NOT_DS		BIT(4)
#define SFC_CS_SEL_1_NOT_0		BIT(5)
#define SFC_T1_ENB_R_OPC		BIT(6)
#define SFC_T3_ENB_R_OPC		BIT(7)

/* Register: SFC_CLOCK_CONTROL */
#define SFC_START_DLL			BIT(4)
#define SFC_TX_DLL_SEL			BIT(5)
#define SFC_RX_DLL_SEL			BIT(6)
#define SFC_RX_CLK_SEL			BIT(7)
#define SFC_DLL_FUNC_CLK_SEL		BIT(9)

/* Register: SFC_INTERRUPT_ENABLE */
#define SFC_INT_ENABLE			BIT(0)

/* Register: SFC_INTERRUPT_STATUS */
#define SFC_INT_PENDING			BIT(0)

/* Register: SFC_INTERRUPT_ENABLE/SFC_INTERRUPT_STATUS/SFC_INTERRUPT_CLEAR */
#define SFC_INT_SEQUENCE_COMPLETION	BIT(2)
#define SFC_INT_CHECK_ERROR		BIT(4)
#define SFC_INT_TIMEOUT_ERROR		BIT(7)
#define SFC_INT_FIFO_UNDERFLOW_ERROR	BIT(12)
#define SFC_INT_FIFO_OVERFLOW_ERROR	BIT(13)
#define SFC_INT_BOOT_ROGUE_ACCESS_ERROR	BIT(14)
#define SFC_INT_ERRORS	(SFC_INT_CHECK_ERROR | SFC_INT_TIMEOUT_ERROR |\
			 SFC_INT_FIFO_UNDERFLOW_ERROR |\
			 SFC_INT_FIFO_OVERFLOW_ERROR |\
			 SFC_INT_BOOT_ROGUE_ACCESS_ERROR)

/* Register: SFC_IP_SW_RESET */
#define SFC_SW_RESET_IP			BIT(0)
#define SFC_SW_RESET_RD_PLUG		BIT(4)
#define SFC_SW_RESET_WR_PLUG		BIT(8)

/* Register: SFC_IP_STATUS */
#define SFC_BDM_NOT_IDLE		BIT(16)
#define SFC_DLL_LOCKED			BIT(19)
#define SFC_RD_PLUG_SW_RESET_END	BIT(24)
#define SFC_WR_PLUG_SW_RESET_END	BIT(25)

/* Register: SFC_FLASH_ADDRESS_CONFIG */
#define SFC_ADDR_4_BYTE_MODE		BIT(0)
#define SFC_ADDR_CS_ASSERT		BIT(23)
#define SFC_ADDR_DDR_ENABLE		BIT(24)
#define SFC_ADDR_PADS(x)		(((x) & GENMASK(1, 0)) << 25)
#define SFC_ADDR_PADS_1			BIT(25)
#define SFC_ADDR_PADS_2			BIT(26)
#define SFC_ADDR_PADS_4			GENMASK(26, 25)
#define SFC_ADDR_NO_OF_CYC(x)		(((u32)(x) & GENMASK(4, 0)) << 27)

/* Register: SFC_READ_FLASH_STATUS_CONFIG */
#define SFC_RFS_INST(x)			(((x) & GENMASK(7, 0)))
#define SFC_RFS_OP_WTR(x)		(((x) & GENMASK(1, 0)) << 8)
#define SFC_RFS_SCHED_SEL		BIT(12)
#define SFC_RFS_CS_ASSERT		BIT(23)
#define SFC_RFS_DDR_ENABLE		BIT(24)
#define SFC_RFS_PADS(x)			(((x) & GENMASK(1, 0)) << 25)
#define SFC_RFS_PADS_1			BIT(25)
#define SFC_RFS_PADS_2			BIT(26)
#define SFC_RFS_PADS_4			GENMASK(26, 25)
#define SFC_RFS_NO_OF_CYC(x)		(((x) & GENMASK(4, 0)) << 27)

/* Register: SFC_CHECK_FLASH_STATUS_CONFIG */
#define SFC_CFS_INST(x)			(((x) & GENMASK(7, 0)))
#define SFC_CFS_OP_WTR(x)		(((x) & GENMASK(1, 0)) << 8)
#define SFC_CFS_CS_ASSERT		BIT(23)
#define SFC_CFS_DDR_ENABLE		BIT(24)
#define SFC_CFS_PADS(x)			(((x) & GENMASK(1, 0)) << 25)
#define SFC_CFS_PADS_1			BIT(25)
#define SFC_CFS_PADS_2			BIT(26)
#define SFC_CFS_PADS_4			GENMASK(26, 25)
#define SFC_CFS_NO_OF_CYC(x)		(((x) & GENMASK(4, 0)) << 27)

/* Register: SFC_POLL_SCHEDULER_0 */
#define SFC_SCHED_TIMER(x)		(((x) & GENMASK(21, 0)))
#define SFC_SCHED_COUNTER(x)		(((x) & GENMASK(9, 0) << 22))

/* Register: SFC_CLOCK_DIVISION */
#define SFC_CLK_DIVISION(x)		(((x) & GENMASK(2, 0)))
#define SFC_CLK_DIV			GENMASK(2, 0)
#define SFC_CLK_DIV_BYPASS		BIT(3)

/* Register: SFC_TRANSMIT_CONFIG_CDM */
#define SFC_TCFG_CDM_TRANSMIT_DATA(x)	(((x) & GENMASK(15, 0)))
#define SFC_TCFG_CDM_CS_ASSERT		BIT(23)
#define SFC_TCFG_CDM_DDR_ENABLE		BIT(24)
#define SFC_TCFG_CDM_PADS(x)		(((x) & GENMASK(1, 0)) << 25)
#define SFC_TCFG_CDM_PADS_1		BIT(25)
#define SFC_TCFG_CDM_PADS_2		BIT(26)
#define SFC_TCFG_CDM_PADS_4		GENMASK(26, 25)
#define SFC_TCFG_CDM_NO_OF_CYC(x)	(((x) & GENMASK(4, 0)) << 27)

/* Register: SFC_CONFIG_CDM */
#define SFC_CFG_CDM_CS_ASSERT		BIT(23)
#define SFC_CFG_CDM_DDR_ENABLE		BIT(24)
#define SFC_CFG_CDM_PADS(x)		(((x) & GENMASK(1, 0)) << 25)
#define SFC_CFG_CDM_PADS_1		BIT(25)
#define SFC_CFG_CDM_PADS_2		BIT(26)
#define SFC_CFG_CDM_PADS_4		GENMASK(26, 25)
#define SFC_CFG_CDM_NO_OF_CYC(x)	(((u32)(x) & GENMASK(4, 0)) << 27)

/* Register: SFC_OPCODE_n */
#define SFC_OPC_TRANSMIT_DATA(x)	(((x) & GENMASK(15, 0)))
#define SFC_OPC_CS_ASSERT		BIT(23)
#define SFC_OPC_DDR_ENABLE		BIT(24)
#define SFC_OPC_PADS(x)			(((x) & GENMASK(1, 0)) << 25)
#define SFC_OPC_PADS_1			BIT(25)
#define SFC_OPC_PADS_2			BIT(26)
#define SFC_OPC_PADS_4			GENMASK(26, 25)
#define SFC_OPC_NO_OF_CYC(x)		(((x) & GENMASK(4, 0)) << 27)

/* Register: SFC_DATA_INST_SIZE */
#define SFC_DIS_SIZE(x)			((x) & GENMASK(24, 0))

/* Register: SFC_SEQUENCE_CONFIG */
#define SFC_SC_REPEAT_COUNT(x)		(((x) & GENMASK(15, 0)))
#define SFC_SC_BOOT_OTP_ON		BIT(22)
#define SFC_SC_CS_ASSERT		BIT(23)
#define SFC_SC_DDR_ENABLE		BIT(24)
#define SFC_SC_PADS(x)			(((x) & GENMASK(1, 0)) << 25)
#define SFC_SC_PADS_1			BIT(25)
#define SFC_SC_PADS_2			BIT(26)
#define SFC_SC_PADS_4			GENMASK(26, 25)
#define SFC_SC_WRITE_NOT_READ		BIT(27)
#define SFC_SC_DMA_ON			BIT(28)
#define SFC_SC_START_SEQ		BIT(31)

/* DMA registers */
#define SFC_REG_G3_RD			0x0808

/* Freq used */
#define SFC_FLASH_SAFE_FREQ		25000000UL

/* Command address size */
#define SFC_32BIT_ADDR			32
#define SFC_24BIT_ADDR			24

/* Clk div min and max */
#define CLK_DIV_MAX			14
#define CLK_DIV_MIN			2

/* Maximum READID length */
#define SFC_MAX_READID_LEN		6
#define SFC_MAX_READID_LEN_ALIGNED	((SFC_MAX_READID_LEN + 0x3) & ~0x3)

/* Cycles number for an opcode */
#define SFC_NB_OPCODE_CYCLES		8

/* Page size */
#define SFC_FLASH_PAGESIZE		256
#define SFC_DMA_ALIGNMENT		64

/* Min data transfer size */
#define SFC_MIN_DATA_TRANSFER		4

/* bytes lost by RD plug in case HW issue happens */
#define SFC_MAX_LOST_BYTES		4

/* pm runtime autosuspend delay */
#define SFC_AUTOSUSPEND_DELAY		100

/* Pad opc/address/data/ */
#define SFC_PADS_1			1
#define SFC_PADS_2			2
#define SFC_PADS_4			3

/* RSR operand WTR/CHECK */
#define SFC_RSR_OPERAND_0		0
#define SFC_RSR_OPERAND_1		1
#define SFC_RSR_OPERAND_2		2
#define SFC_RSR_OPERAND_3		3

/* Maximum operation times (in ms) */
#define SFC_FLASH_MAX_CHIP_ERASE_MS	500000 /* Chip Erase time */
#define SFC_FLASH_MAX_SEC_ERASE_MS	30000 /* Sector Erase time */
#define SFC_FLASH_MAX_PAGE_WRITE_MS	100 /* Write Page time */
#define SFC_FLASH_MAX_STA_WRITE_MS	4000 /* Write status reg time */
#define SFC_MAX_WAIT_SEQ_MS		1000 /* Sequence execution time */

/* Block Protect Bits (BPx) */
#define SFC_BPX_MAX_BITS		4
#define SFC_BPX_MAX_N_BNDS		BIT(SFC_BPX_MAX_BITS)

enum sfc_block_mask_idx {
	SFC_BLOCK_UNLOCKED,
	SFC_BLOCK_LOCKED,
	SFC_BLOCK_MASK_CNT,
};

/* Invert address */
#define SFC_3_BYTES_ADDR(x)		((((x) & 0xff) << 16) | \
					((x) & 0xff00) | \
					(((x) & 0xff0000) >> 16))
#define SFC_4_BYTES_ADDR(x)		((((x) & 0xff) << 24) | \
					(((x) & 0xff00) << 8) | \
					(((x) & 0xff0000) >> 8) | \
					(((x) & 0xff000000) >> 24))

/* Flags operation of default read/write/erase/lock/unlock routines */
#define SFC_CFG_READ_TOGGLE_32BIT_ADDR	0x00000001
#define SFC_CFG_WRITE_TOGGLE_32BIT_ADDR	0x00000002
#define SFC_CFG_LOCK_TOGGLE_32BIT_ADDR	0x00000004
#define SFC_CFG_ERASE_TOGGLE_32BIT_ADDR	0x00000008
#define SFC_CFG_S25FL_CHECK_ERROR_FLAGS	0x00000010
#define SFC_CFG_N25Q_CHECK_ERROR_FLAGS	0x00000020
#define SFC_CFG_WRSR_FORCE_16BITS	0x00000040
#define SFC_CFG_RD_WR_LOCK_REG		0x00000080
#define SFC_CFG_MX25_TOGGLE_QE_BIT	0x00000100
#define CFG_S25FS_4K_ERASE		0x00000200

/* Device flags */
#define SFC_FLAG_SINGLE			0x000000ff
#define SFC_FLAG_READ_WRITE		0x00000001
#define SFC_FLAG_READ_FAST		0x00000002
#define SFC_FLAG_32BIT_ADDR		0x00000004
#define SFC_FLAG_RESET			0x00000008
#define SFC_FLAG_BLK_LOCKING		0x00000010
#define SFC_FLAG_BPX_LOCKING		0x00000020

#define SFC_FLAG_DUAL			0x0000ff00
#define SFC_FLAG_READ_1_1_2		0x00000100
#define SFC_FLAG_READ_1_2_2		0x00000200
#define SFC_FLAG_READ_2_2_2		0x00000400
#define SFC_FLAG_WRITE_1_1_2		0x00001000
#define SFC_FLAG_WRITE_1_2_2		0x00002000
#define SFC_FLAG_WRITE_2_2_2		0x00004000

#define SFC_FLAG_QUAD			0x00ff0000
#define SFC_FLAG_READ_1_1_4		0x00010000
#define SFC_FLAG_READ_1_4_4		0x00020000
#define SFC_FLAG_READ_4_4_4		0x00040000
#define SFC_FLAG_WRITE_1_1_4		0x00100000
#define SFC_FLAG_WRITE_1_4_4		0x00200000
#define SFC_FLAG_WRITE_4_4_4		0x00400000

/* SFC BDM Instruction Opcodes */
#define STSFC_OPC_WRITE			0x1
#define STSFC_OPC_ADDRESS		0x2
#define STSFC_OPC_DATA			0x3
#define STSFC_OPC_RSR			0x4
#define STSFC_OPC_WTR			0x5
#define STSFC_OPC_CHECK			0x6
#define STSFC_OPC_LOOP			0x7
#define STSFC_OPC_STOP			0xF

/* SFC BDM Instructions (== opcode + operand) */
#define STSFC_INSTR(cmd, op)		((cmd) | ((op) << 4))

#define STSFC_INST_WR_OPC_0		STSFC_INSTR(STSFC_OPC_WRITE, 0)
#define STSFC_INST_WR_OPC_1		STSFC_INSTR(STSFC_OPC_WRITE, 1)
#define STSFC_INST_WR_OPC_2		STSFC_INSTR(STSFC_OPC_WRITE, 2)
#define STSFC_INST_WR_OPC_3		STSFC_INSTR(STSFC_OPC_WRITE, 3)
#define STSFC_INST_WR_OPC_4		STSFC_INSTR(STSFC_OPC_WRITE, 4)
#define STSFC_INST_WR_OPC_5		STSFC_INSTR(STSFC_OPC_WRITE, 5)

#define STSFC_INST_ADDR			STSFC_INSTR(STSFC_OPC_ADDRESS, 0)

#define STSFC_INST_DATA_BDM		STSFC_INSTR(STSFC_OPC_DATA, 0)
#define STSFC_INST_DATA_BOOT		STSFC_INSTR(STSFC_OPC_DATA, 8)

#define STSFC_INST_RSR_0		STSFC_INSTR(STSFC_OPC_RSR, 0)
#define STSFC_INST_RSR_1		STSFC_INSTR(STSFC_OPC_RSR, 1)
#define STSFC_INST_RSR_2		STSFC_INSTR(STSFC_OPC_RSR, 2)
#define STSFC_INST_RSR_3		STSFC_INSTR(STSFC_OPC_RSR, 3)

#define STSFC_INST_WTR_0		STSFC_INSTR(STSFC_OPC_WTR, 0)
#define STSFC_INST_WTR_1		STSFC_INSTR(STSFC_OPC_WTR, 1)
#define STSFC_INST_WTR_2		STSFC_INSTR(STSFC_OPC_WTR, 2)
#define STSFC_INST_WTR_3		STSFC_INSTR(STSFC_OPC_WTR, 3)
#define STSFC_INST_WTR_4		STSFC_INSTR(STSFC_OPC_WTR, 4)
#define STSFC_INST_WTR_5		STSFC_INSTR(STSFC_OPC_WTR, 5)
#define STSFC_INST_WTR_6		STSFC_INSTR(STSFC_OPC_WTR, 6)
#define STSFC_INST_WTR_7		STSFC_INSTR(STSFC_OPC_WTR, 7)
#define STSFC_INST_WTR_8		STSFC_INSTR(STSFC_OPC_WTR, 8)
#define STSFC_INST_WTR_9		STSFC_INSTR(STSFC_OPC_WTR, 9)
#define STSFC_INST_WTR_10		STSFC_INSTR(STSFC_OPC_WTR, 10)
#define STSFC_INST_WTR_11		STSFC_INSTR(STSFC_OPC_WTR, 11)
#define STSFC_INST_WTR_12		STSFC_INSTR(STSFC_OPC_WTR, 12)
#define STSFC_INST_WTR_13		STSFC_INSTR(STSFC_OPC_WTR, 13)
#define STSFC_INST_WTR_14		STSFC_INSTR(STSFC_OPC_WTR, 14)
#define STSFC_INST_WTR_15		STSFC_INSTR(STSFC_OPC_WTR, 15)

#define STSFC_INST_CHECK_0		STSFC_INSTR(STSFC_OPC_CHECK, 0)
#define STSFC_INST_CHECK_1		STSFC_INSTR(STSFC_OPC_CHECK, 1)
#define STSFC_INST_CHECK_2		STSFC_INSTR(STSFC_OPC_CHECK, 2)
#define STSFC_INST_CHECK_3		STSFC_INSTR(STSFC_OPC_CHECK, 3)
#define STSFC_INST_CHECK_4		STSFC_INSTR(STSFC_OPC_CHECK, 4)
#define STSFC_INST_CHECK_5		STSFC_INSTR(STSFC_OPC_CHECK, 5)
#define STSFC_INST_CHECK_6		STSFC_INSTR(STSFC_OPC_CHECK, 6)
#define STSFC_INST_CHECK_7		STSFC_INSTR(STSFC_OPC_CHECK, 7)
#define STSFC_INST_CHECK_8		STSFC_INSTR(STSFC_OPC_CHECK, 8)
#define STSFC_INST_CHECK_9		STSFC_INSTR(STSFC_OPC_CHECK, 9)
#define STSFC_INST_CHECK_10		STSFC_INSTR(STSFC_OPC_CHECK, 10)
#define STSFC_INST_CHECK_11		STSFC_INSTR(STSFC_OPC_CHECK, 11)
#define STSFC_INST_CHECK_12		STSFC_INSTR(STSFC_OPC_CHECK, 12)
#define STSFC_INST_CHECK_13		STSFC_INSTR(STSFC_OPC_CHECK, 13)
#define STSFC_INST_CHECK_14		STSFC_INSTR(STSFC_OPC_CHECK, 14)
#define STSFC_INST_CHECK_15		STSFC_INSTR(STSFC_OPC_CHECK, 15)

#define STSFC_INST_LOOP_0		STSFC_INSTR(STSFC_OPC_LOOP, 0)
#define STSFC_INST_LOOP_1		STSFC_INSTR(STSFC_OPC_LOOP, 1)
#define STSFC_INST_LOOP_2		STSFC_INSTR(STSFC_OPC_LOOP, 2)
#define STSFC_INST_LOOP_3		STSFC_INSTR(STSFC_OPC_LOOP, 3)
#define STSFC_INST_LOOP_4		STSFC_INSTR(STSFC_OPC_LOOP, 4)
#define STSFC_INST_LOOP_5		STSFC_INSTR(STSFC_OPC_LOOP, 5)
#define STSFC_INST_LOOP_6		STSFC_INSTR(STSFC_OPC_LOOP, 6)
#define STSFC_INST_LOOP_7		STSFC_INSTR(STSFC_OPC_LOOP, 7)
#define STSFC_INST_LOOP_8		STSFC_INSTR(STSFC_OPC_LOOP, 8)
#define STSFC_INST_LOOP_9		STSFC_INSTR(STSFC_OPC_LOOP, 9)
#define STSFC_INST_LOOP_10		STSFC_INSTR(STSFC_OPC_LOOP, 10)
#define STSFC_INST_LOOP_11		STSFC_INSTR(STSFC_OPC_LOOP, 11)
#define STSFC_INST_LOOP_12		STSFC_INSTR(STSFC_OPC_LOOP, 12)
#define STSFC_INST_LOOP_13		STSFC_INSTR(STSFC_OPC_LOOP, 13)
#define STSFC_INST_LOOP_14		STSFC_INSTR(STSFC_OPC_LOOP, 14)
#define STSFC_INST_LOOP_15		STSFC_INSTR(STSFC_OPC_LOOP, 15)

#define STSFC_INST_STOP			STSFC_INSTR(STSFC_OPC_STOP, 0)

/* SFC BDM sequence node - 64 bytes aligned */
/* Note that this is actually a hardware mmap.  Do not change this structure. */
struct stsfc_seq {
	u32 opc[6];
	u32 addr;
	u8  seq[16];
	u32 data_size;
	u32 dma_addr;
	u8  seq_otp[8];
	u32 seq_cfg;
} __packed __aligned(4);

struct stsfc {
	struct device *dev;
	void __iomem *base;
	void __iomem *dma;

	u8 *dma_buf_raw;
	dma_addr_t dma_addr_raw;

	u8 *dma_buf;
	size_t dma_buf_size;
	dma_addr_t dma_addr;

	struct resource *region;
	struct mtd_info mtd;
	struct mutex lock;
	struct flash_info *info;
	struct clk *sfc_clk;
	struct clk *emi_clk;
	struct completion seq_completed;
	u32 seq_status;

	struct ibi *ibi;
	struct ibi_state *ibi_rd_plug;
	struct ibi_state *ibi_wr_plug;

	u32 configuration;
	u32 max_freq;
	bool booted_from_spi;
	bool reset_signal;
	bool reset_por;
	bool cs1_used;
	bool polling_mode;
	bool check_modepins;

	/* DLL mode */
	bool dll_mode;
	u8 dll_tx_delay;
	u8 dll_rx_delay;

	struct stsfc_seq stsfc_seq_read;
	u32 seq_read_addr_cfg;
	struct stsfc_seq stsfc_seq_write;
	u32 seq_write_addr_cfg;
	struct stsfc_seq stsfc_seq_erase_sector;
	u32 seq_erase_addr_cfg;

	/* Block locking support */
	u8 lock_mask;
	u8 lock_val[SFC_BLOCK_MASK_CNT];

	/* 4KiB Parameter Sector boundaries */
	u64 p4k_bot_end;
	u64 p4k_top_start;

	/* Section address of 4-kB physical sectors at top/bottom */
	u32 p4k_sector_address;
	u8 p4k_erase_cmd;

	/* 'BPx' Block Protection scheme */
	int bpx_tb;
	int bpx_n_bnds;
	int bpx_n_bits;
	u8 bpx_sr_masks[SFC_BPX_MAX_BITS];
	u64 bpx_bnds[SFC_BPX_MAX_N_BNDS];

	/* individual block lock/unlock opcode */
	u8	op_lock;
	u8	op_unlock;
	u8	op_wr_lock;
	u8	op_rd_lock;
	u8 lock_cycles_addr;

	void (*enter_32bit_addr)(struct stsfc *, bool);
};

/* Parameters to configure a READ or WRITE FSM sequence */
struct seq_rw_config {
	u32 flags;
	u8 cmd;
	bool write;
	u8 addr_pads;
	u8 data_pads;
	u16 mode_data;
	u8 mode_cycles;
	u8 dummy_cycles;
};

/* SPI Flash Device Table */
struct flash_info {
	char *name;
	u8 readid[SFC_MAX_READID_LEN];
	int readid_len;
	unsigned int sector_size;
	u16 n_sectors;
	u32 flags;
	u32 max_freq;

	int (*config)(struct stsfc *);
	int (*resume)(struct stsfc *);
};

/* Device with standard 3-byte JEDEC ID */
#define JEDEC_INFO(_name, _jedec_id, _sector_size, _n_sectors,	\
		   _flags, _max_freq, _config, _resume)		\
	{							\
		.name = (_name),				\
		.readid[0] = ((_jedec_id) >> 16 & 0xff),	\
		.readid[1] = ((_jedec_id) >>  8 & 0xff),	\
		.readid[2] = ((_jedec_id) >>  0 & 0xff),	\
		.readid_len = 3,				\
		.sector_size = (_sector_size),			\
		.n_sectors = (_n_sectors),			\
		.flags = (_flags),				\
		.max_freq = (_max_freq),			\
		.config = (_config),				\
		.resume = (_resume)				\
	}

/* Device with arbitrary-length READID */
#define RDID(...) __VA_ARGS__  /* Dummy macro to protect array argument. */
#define RDID_INFO(_name, _readid, _readid_len, _sector_size,	\
		  _n_sectors, _flags, _max_freq, _config,	\
		  _resume)					\
	{							\
		.name = (_name),				\
		.readid = _readid,				\
		.readid_len = _readid_len,			\
		.sector_size = (_sector_size),			\
		.n_sectors = (_n_sectors),			\
		.flags = (_flags),				\
		.max_freq = (_max_freq),			\
		.config = (_config),				\
		.resume = (_resume)				\
	}

static int stsfc_n25q_config(struct stsfc *sfc);
static int stsfc_mx25_config(struct stsfc *sfc);
static int stsfc_s25fl_config(struct stsfc *sfc);
static int stsfc_w25q_config(struct stsfc *sfc);

static int stsfc_n25q_resume(struct stsfc *sfc);
static int stsfc_mx25_resume(struct stsfc *sfc);

static struct flash_info flash_types[] = {
	/* Macronix MX25xxx */
#define MX25_FLAG (SFC_FLAG_READ_WRITE | \
		   SFC_FLAG_READ_FAST | \
		   SFC_FLAG_READ_1_1_2 | \
		   SFC_FLAG_READ_1_2_2 | \
		   SFC_FLAG_READ_1_1_4)
	JEDEC_INFO("mx25l3255e",  0xc29e16, 64 * 1024, 64,
		   MX25_FLAG | SFC_FLAG_WRITE_1_4_4, 86000000,
		   stsfc_mx25_config, stsfc_mx25_resume),
	JEDEC_INFO("mx25l12835f", 0xc22018, 64 * 1024, 256,
		   (MX25_FLAG | SFC_FLAG_RESET), 104000000,
		   stsfc_mx25_config, stsfc_mx25_resume),
	JEDEC_INFO("mx25l25635f", 0xc22019, 64 * 1024, 512,
		   (MX25_FLAG | SFC_FLAG_RESET), 104000000,
		   stsfc_mx25_config, stsfc_mx25_resume),
	JEDEC_INFO("mx25l25655f", 0xc22619, 64 * 1024, 512,
		   (MX25_FLAG | SFC_FLAG_RESET), 104000000,
		   stsfc_mx25_config, stsfc_mx25_resume),

	/* Micron N25Qxxx */
#define N25Q_FLAG (SFC_FLAG_READ_WRITE | \
		   SFC_FLAG_READ_FAST | \
		   SFC_FLAG_READ_1_1_2 | \
		   SFC_FLAG_READ_1_2_2 | \
		   SFC_FLAG_READ_1_1_4 | \
		   SFC_FLAG_WRITE_1_1_2 | \
		   SFC_FLAG_WRITE_1_2_2 | \
		   SFC_FLAG_WRITE_1_1_4 | \
		   SFC_FLAG_WRITE_1_4_4 | \
		   SFC_FLAG_BLK_LOCKING)
	JEDEC_INFO("n25q128", 0x20ba18, 64 * 1024,  256,
		   N25Q_FLAG, 108000000, stsfc_n25q_config, stsfc_n25q_resume),

	/*
	 * Micron N25Q256/N25Q512/N25Q00A (32-bit ADDR devices)
	 *
	 * Versions are available with or without a dedicated RESET# pin
	 * (e.g. N25Q512A83GSF40G vs. N25Q512A13GSF40G). To complicate matters,
	 * the versions that include a RESET# pin (Feature Set = 8) require a
	 * different opcode for the FLASH_CMD_WRITE_1_4_4 command.
	 * Unfortunately it is not possible to determine easily at run-time
	 * which version is being used.  We therefore remove support for
	 * FLASH_FLAG_WRITE_1_4_4 (falling back to FLASH_FLAG_WRITE_1_1_4), and
	 * defer overall support for RESET# to the board-level platform/Device
	 * Tree property "reset-signal".
	 */
#define N25Q_32BIT_ADDR_FLAG  ((N25Q_FLAG | \
				SFC_FLAG_RESET) & \
			       ~SFC_FLAG_WRITE_1_4_4)
	JEDEC_INFO("n25q256", 0x20ba19, 64 * 1024,   512,
		   N25Q_32BIT_ADDR_FLAG, 108000000,
		   stsfc_n25q_config, stsfc_n25q_resume),
	RDID_INFO("n25q512", RDID({0x20, 0xba, 0x20}), 3,
		  64 * 1024, 1024, N25Q_32BIT_ADDR_FLAG, 108000000,
		  stsfc_n25q_config, stsfc_n25q_resume),
	RDID_INFO("n25q512", RDID({0x20, 0xbb, 0x20}), 3,
		  64 * 1024, 1024, N25Q_32BIT_ADDR_FLAG, 108000000,
		  stsfc_n25q_config, stsfc_n25q_resume),
	RDID_INFO("n25q00a", RDID({0x20, 0xba, 0x21, 0x10, 0x00}), 5,
		  64 * 1024, 2048, N25Q_32BIT_ADDR_FLAG, 108000000,
		  stsfc_n25q_config, stsfc_n25q_resume),
	RDID_INFO("mt25qu01g", RDID({0x20, 0xbb, 0x21}), 3,
		  64 * 1024, 2048, N25Q_32BIT_ADDR_FLAG,
		  108000000, stsfc_n25q_config, stsfc_n25q_resume),
	RDID_INFO("mt25qu02g", RDID({0x20, 0xbb, 0x22}), 3,
		  64 * 1024, 4096, N25Q_32BIT_ADDR_FLAG | SFC_FLAG_WRITE_1_4_4,
		  166000000, stsfc_n25q_config, stsfc_n25q_resume),

	/*
	 * Spansion S25FLxxxP
	 *     - 256KiB and 64KiB sector variants (identified by ext. JEDEC)
	 *     - S25FL128Px devices do not support DUAL or QUAD I/O
	 */
#define S25FLXXXP_FLAG (SFC_FLAG_READ_WRITE | \
			SFC_FLAG_READ_1_1_2 | \
			SFC_FLAG_READ_1_2_2 | \
			SFC_FLAG_READ_1_1_4 | \
			SFC_FLAG_READ_1_4_4 | \
			SFC_FLAG_WRITE_1_1_4 | \
			SFC_FLAG_READ_FAST | \
			SFC_FLAG_BPX_LOCKING)
	RDID_INFO("s25fl032p", RDID({0x01, 0x02, 0x15, 0x4d, 0x00}), 5,
		  64 * 1024,  64, S25FLXXXP_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fl064p", RDID({0x01, 0x02, 0x16, 0x4d, 0x00}), 5,
		  64 * 1024,  128, S25FLXXXP_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fl128p1", RDID({0x01, 0x20, 0x18, 0x03, 0x00}), 5,
		  256 * 1024, 64,
		  (SFC_FLAG_READ_WRITE | SFC_FLAG_READ_FAST), 104000000,
		  NULL, NULL),
	RDID_INFO("s25fl128p0", RDID({0x01, 0x20, 0x18, 0x03, 0x01}), 5,
		  64 * 1024, 256,
		  (SFC_FLAG_READ_WRITE | SFC_FLAG_READ_FAST), 104000000,
		  NULL, NULL),
	RDID_INFO("s25fl129p0", RDID({0x01, 0x20, 0x18, 0x4d, 0x00}), 5,
		  256 * 1024,  64, S25FLXXXP_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fl129p1", RDID({0x01, 0x20, 0x18, 0x4d, 0x01}), 5,
		  64 * 1024, 256, S25FLXXXP_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),

	/*
	 * Spansion S25FLxxxS
	 *     - 256KiB and 64KiB sector variants (identified by ext. JEDEC)
	 *     - RESET# signal supported by die but not bristled out on all
	 *       package types.  The package type is a function of board design,
	 *       so this information is captured in the board's flags.
	 *     - Supports 'DYB' sector protection. Depending on variant, sectors
	 *       may default to locked state on power-on.
	 *     - S25FL127Sx handled as S25FL128Sx
	 */
#define S25FLXXXS_FLAG (S25FLXXXP_FLAG | \
			SFC_FLAG_RESET | \
			SFC_FLAG_BLK_LOCKING)
	RDID_INFO("s25fl128s0", RDID({0x01, 0x20, 0x18, 0x4d, 0x00, 0x80}), 6,
		  256 * 1024, 64, S25FLXXXS_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fl128s1", RDID({0x01, 0x20, 0x18, 0x4d, 0x01, 0x80}), 6,
		  64 * 1024, 256, S25FLXXXS_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fl256s0", RDID({0x01, 0x02, 0x19, 0x4d, 0x00, 0x80}), 6,
		  256 * 1024, 128, S25FLXXXS_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fl256s1", RDID({0x01, 0x02, 0x19, 0x4d, 0x01, 0x80}), 6,
		  64 * 1024, 512, S25FLXXXS_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fl512s", RDID({0x01, 0x02, 0x20, 0x4d, 0x00, 0x80}), 6,
		  256 * 1024, 256, S25FLXXXS_FLAG, 80000000,
		  stsfc_s25fl_config, NULL),

/* Spansion S25FSxxxS */
#define S25FSXXXS_CAPS (SFC_FLAG_READ_WRITE	| \
			SFC_FLAG_READ_FAST	| \
			SFC_FLAG_READ_1_2_2	| \
			SFC_FLAG_READ_1_4_4	| \
			SFC_FLAG_RESET		| \
			SFC_FLAG_BLK_LOCKING	| \
			SFC_FLAG_BPX_LOCKING)
	RDID_INFO("s25fs128s0", RDID({0x01, 0x20, 0x18, 0x4d, 0x00, 0x81}), 6,
		  256 * 1024, 64, S25FSXXXS_CAPS, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fs128s1", RDID({0x01, 0x20, 0x18, 0x4d, 0x01, 0x81}), 6,
		  64 * 1024, 256, S25FSXXXS_CAPS, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fs256s0", RDID({0x01, 0x02, 0x19, 0x4d, 0x00, 0x81}), 6,
		  256 * 1024, 128, S25FSXXXS_CAPS, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fs256s1", RDID({0x01, 0x02, 0x19, 0x4d, 0x01, 0x81}), 6,
		  64 * 1024, 512, S25FSXXXS_CAPS, 80000000,
		  stsfc_s25fl_config, NULL),
	RDID_INFO("s25fs512s", RDID({0x01, 0x02, 0x20, 0x4d, 0x00, 0x81}), 6,
		  256 * 1024, 256, S25FSXXXS_CAPS, 80000000,
		  stsfc_s25fl_config, NULL),

#define W25X_FLAG (SFC_FLAG_READ_WRITE | \
		   SFC_FLAG_READ_FAST | \
		   SFC_FLAG_READ_1_1_2 | \
		   SFC_FLAG_WRITE_1_1_2)
	JEDEC_INFO("w25x40", 0xef3013, 64 * 1024,   8, W25X_FLAG, 75000000,
		   NULL, NULL),
	JEDEC_INFO("w25x80", 0xef3014, 64 * 1024,  16, W25X_FLAG, 75000000,
		   NULL, NULL),
	JEDEC_INFO("w25x16", 0xef3015, 64 * 1024,  32, W25X_FLAG, 75000000,
		   NULL, NULL),
	JEDEC_INFO("w25x32", 0xef3016, 64 * 1024,  64, W25X_FLAG, 75000000,
		   NULL, NULL),
	JEDEC_INFO("w25x64", 0xef3017, 64 * 1024, 128, W25X_FLAG, 75000000,
		   NULL, NULL),

	/* Winbond -- w25q "blocks" are 64K, "sectors" are 4KiB */
#define W25Q_FLAG (SFC_FLAG_READ_WRITE | \
		   SFC_FLAG_READ_FAST | \
		   SFC_FLAG_READ_1_1_2 | \
		   SFC_FLAG_READ_1_2_2 | \
		   SFC_FLAG_READ_1_1_4 | \
		   SFC_FLAG_READ_1_4_4 | \
		   SFC_FLAG_WRITE_1_1_4 | \
		   SFC_FLAG_BPX_LOCKING)
	JEDEC_INFO("w25q80", 0xef4014, 64 * 1024,  16,
		   W25Q_FLAG, 80000000, stsfc_w25q_config, NULL),
	JEDEC_INFO("w25q16", 0xef4015, 64 * 1024,  32,
		   W25Q_FLAG, 80000000, stsfc_w25q_config, NULL),
	JEDEC_INFO("w25q32", 0xef4016, 64 * 1024,  64,
		   W25Q_FLAG, 80000000, stsfc_w25q_config, NULL),
	JEDEC_INFO("w25q64", 0xef4017, 64 * 1024, 128,
		   W25Q_FLAG, 80000000, stsfc_w25q_config, NULL),

	/* Spansion S25FL1xxK - same init as Winbond W25Q family devices */
#define S25FL1XXK_FLAG (SFC_FLAG_READ_WRITE | \
			SFC_FLAG_READ_FAST | \
			SFC_FLAG_READ_1_1_2 | \
			SFC_FLAG_READ_1_2_2 | \
			SFC_FLAG_READ_1_1_4 | \
			SFC_FLAG_READ_1_4_4 | \
			SFC_FLAG_BPX_LOCKING)
	JEDEC_INFO("s25fl116k", 0x014015, 64 * 1024,  32, S25FL1XXK_FLAG,
		   108000000, stsfc_w25q_config, NULL),
	JEDEC_INFO("s25fl132k", 0x014016, 64 * 1024,  64, S25FL1XXK_FLAG,
		   108000000, stsfc_w25q_config, NULL),
	JEDEC_INFO("s25fl164k", 0x014017, 64 * 1024, 128, S25FL1XXK_FLAG,
		   108000000, stsfc_w25q_config, NULL),

	{},
};

/*
 * GLLCFF provides NOR1 and NOR1/NOR2 boot modes,
 * so we include all combinations, based on selected
 * NOR and clock divider ratio.
 * GLLCFF_SYSCON_BOOT_DEV_SPIXY_Z
 * X = 1, Y = VOID stands for boot from NOR1
 * X = 1, Y = 2 stands for boot from NOR1 or NOR2
 * Z = 1,2,4,6 are the clock divider ratios
 */
#define GLLCFF_SYSCON_BOOT_DEV_REG	0x130048
#define GLLCFF_SYSCON_BOOT_DEV_SPI1_1	0x0
#define GLLCFF_SYSCON_BOOT_DEV_SPI1_2	0x4
#define GLLCFF_SYSCON_BOOT_DEV_SPI1_4	0x8
#define GLLCFF_SYSCON_BOOT_DEV_SPI1_6	0xc
#define GLLCFF_SYSCON_BOOT_DEV_SPI12_1	0x10
#define GLLCFF_SYSCON_BOOT_DEV_SPI12_2	0x14
#define GLLCFF_SYSCON_BOOT_DEV_SPI12_4	0x18
#define GLLCFF_SYSCON_BOOT_DEV_SPI12_6	0x1c
#define GLLCFF_SYSCON_BOOT_DEV_MASK	0x03c

#define SFC_MAX_MODE_PINS		8

struct boot_dev {
	u32 reg;
	u32 spi[SFC_MAX_MODE_PINS];
	u32 mask;
};

static struct boot_dev stsfc_gllcff_data = {
	.reg = GLLCFF_SYSCON_BOOT_DEV_REG,
	.spi = {GLLCFF_SYSCON_BOOT_DEV_SPI1_1, GLLCFF_SYSCON_BOOT_DEV_SPI1_2,
		GLLCFF_SYSCON_BOOT_DEV_SPI1_4, GLLCFF_SYSCON_BOOT_DEV_SPI1_6,
		GLLCFF_SYSCON_BOOT_DEV_SPI12_1, GLLCFF_SYSCON_BOOT_DEV_SPI12_2,
		GLLCFF_SYSCON_BOOT_DEV_SPI12_4, GLLCFF_SYSCON_BOOT_DEV_SPI12_6},
	.mask = GLLCFF_SYSCON_BOOT_DEV_MASK,
};

/*
 *
 * SFC sequence engine
 *
 */
static inline void stsfc_enable_interrupts(struct stsfc *sfc, u32 reg)
{
	writel_relaxed(reg, sfc->base + SFC_INTERRUPT_ENABLE);
}

static inline u32 stsfc_read_interrupts_status(struct stsfc *sfc)
{
	return readl_relaxed(sfc->base + SFC_INTERRUPT_STATUS);
}

static inline void stsfc_clear_interrupts(struct stsfc *sfc, u32 reg)
{
	writel_relaxed(reg, sfc->base + SFC_INTERRUPT_CLEAR);
}

static inline int stsfc_is_busy(struct stsfc *sfc)
{
	return readl_relaxed(sfc->base + SFC_IP_STATUS) & SFC_BDM_NOT_IDLE;
}

static inline int stsfc_load_seq(struct stsfc *sfc,
				 const struct stsfc_seq *seq)
{
	void __iomem *dst = sfc->base + SFC_OPCODE_0;
	const u32 *src = (const u32 *)seq;
	int words = sizeof(*seq) / sizeof(*src);

	if (stsfc_is_busy(sfc)) {
		dev_err(sfc->dev, "SFC sequence in progress\n");
		return -EBUSY;
	}

	reinit_completion(&sfc->seq_completed);
	sfc->seq_status = 0;

	for ( ; words > 0; words--, src++, dst += 4)
		writel_relaxed(*src, dst);

	return 0;
}

static inline void stsfc_load_addr_cfg(struct stsfc *sfc,
				       u32 addr_cfg)
{
	writel_relaxed(addr_cfg, sfc->base + SFC_FLASH_ADDRESS_CONFIG);
}

static inline int stsfc_wait_seq(struct stsfc *sfc, unsigned int max_time_ms)
{
	int ret;

#ifdef CONFIG_SPACEX
	/* Can't use jiffies during a panic_write */
	if (oops_in_progress) {
		u32 status;
		u64 timeout = __arch_counter_get_cntvct() +
			      (arch_timer_get_cntfrq() * max_time_ms) / 1000;

		while (__arch_counter_get_cntvct() < timeout) {
			status = stsfc_read_interrupts_status(sfc);
			if (status & SFC_INT_ERRORS)
				sfc->seq_status |= status & SFC_INT_ERRORS;

			stsfc_clear_interrupts(sfc, status);

			if (status & SFC_INT_SEQUENCE_COMPLETION)
				return 0;
		}
		return -ETIMEDOUT;
	}
#endif

	if (sfc->polling_mode) {
		unsigned long deadline = jiffies +
					 msecs_to_jiffies(max_time_ms);
		u32 status;

		while (time_after_eq(jiffies, deadline)) {
			status = stsfc_read_interrupts_status(sfc);
			if (status & SFC_INT_ERRORS)
				sfc->seq_status |= status & SFC_INT_ERRORS;

			stsfc_clear_interrupts(sfc, status);

			if (status & SFC_INT_SEQUENCE_COMPLETION)
				return 0;

			cond_resched();
		}

		dev_err(sfc->dev, "SFC sequence timeout\n");
		return -ETIMEDOUT;
	}

	ret = wait_for_completion_timeout(&sfc->seq_completed,
					  msecs_to_jiffies(max_time_ms));
	if (!ret) {
		dev_err(sfc->dev, "SFC sequence timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static inline void stsfc_restart_seq(struct stsfc *sfc,
				     const struct stsfc_seq *seq)
{
	writel_relaxed(seq->seq_cfg, sfc->base + SFC_SEQUENCE_CONFIG);
}

static inline void stsfc_load_transmit_config_cdm(struct stsfc *sfc,
						  u32 reg)
{
	writel_relaxed(reg, sfc->base + SFC_TRANSMIT_CONFIG_CDM);
}

static inline void stsfc_load_config_cdm(struct stsfc *sfc,
					 u32 reg)
{
	writel_relaxed(reg, sfc->base + SFC_CONFIG_CDM);
}

static inline void stsfc_load_data_cdm(struct stsfc *sfc,
				       u32 reg)
{
	writel_relaxed(reg, sfc->base + SFC_DATA_CDM);
}

static inline u32 stsfc_read_data_cdm(struct stsfc *sfc)
{
	return readl_relaxed(sfc->base + SFC_DATA_CDM);
}

/*
 * SFC message sequence configurations:
 *
 * All configs are presented in order of preference
 */
static void stsfc_read_status(struct stsfc *sfc, u8 cmd, u8 *data);
static int stsfc_write_status(struct stsfc *sfc, u8 cmd,
			      u16 data, u8 bytes, bool wait_busy);
static int stsfc_erase_sector(struct stsfc *sfc, u32 offset);

/* Default READ configurations, in order of preference */
static struct seq_rw_config default_read_configs[] = {
	{SFC_FLAG_READ_1_4_4, SPINOR_OP_READ_1_4_4, 0,
		SFC_PADS_4, SFC_PADS_4, 0x00, 2, 4},
	{SFC_FLAG_READ_1_1_4, SPINOR_OP_READ_1_1_4, 0,
		SFC_PADS_1, SFC_PADS_4, 0x00, 0, 8},
	{SFC_FLAG_READ_1_2_2, SPINOR_OP_READ_1_2_2, 0,
		SFC_PADS_2, SFC_PADS_2, 0x00, 4, 0},
	{SFC_FLAG_READ_1_1_2, SPINOR_OP_READ_1_1_2, 0,
		SFC_PADS_1, SFC_PADS_2, 0x00, 0, 8},
	{SFC_FLAG_READ_FAST,  SPINOR_OP_READ_FAST,  0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 8},
	{SFC_FLAG_READ_WRITE, SPINOR_OP_READ,       0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

/* Default WRITE configurations */
static struct seq_rw_config default_write_configs[] = {
	{SFC_FLAG_WRITE_1_4_4, SPINOR_OP_WRITE_1_4_4, 1,
		SFC_PADS_4, SFC_PADS_4, 0x00, 0, 0},
	{SFC_FLAG_WRITE_1_1_4, SPINOR_OP_WRITE_1_1_4, 1,
		SFC_PADS_1, SFC_PADS_4, 0x00, 0, 0},
	{SFC_FLAG_WRITE_1_2_2, SPINOR_OP_WRITE_1_2_2, 1,
		SFC_PADS_2, SFC_PADS_2, 0x00, 0, 0},
	{SFC_FLAG_WRITE_1_1_2, SPINOR_OP_WRITE_1_1_2, 1,
		SFC_PADS_1, SFC_PADS_2, 0x00, 0, 0},
	{SFC_FLAG_READ_WRITE,  SPINOR_OP_WRITE,       1,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

/*
 * SoC reset on 'boot-from-spi' systems
 *
 * Certain modes of operation cause the Flash device to enter a particular state
 * for a period of time (e.g. 'Erase Sector', 'Quad Enable', and 'Enter 32-bit
 * Addr' commands).  On boot-from-spi systems, it is important to consider what
 * happens if a warm reset occurs during this period.  The SPIBoot controller
 * assumes that Flash device is in its default reset state, 24-bit address mode,
 * and ready to accept commands.  This can be achieved using some form of
 * on-board logic/controller to force a device POR in response to a SoC-level
 * reset or by making use of the device reset signal if available (limited
 * number of devices only).
 *
 * Failure to take such precautions can cause problems following a warm reset.
 * For some operations (e.g. ERASE), there is little that can be done.  For
 * other modes of operation (e.g. 32-bit addressing), options are often
 * available that can help minimise the window in which a reset could cause a
 * problem.
 *
 */
static bool stsfc_can_handle_soc_reset(struct stsfc *sfc)
{
	/* Reset signal is available on the board and supported by the device */
	if (sfc->reset_signal && (sfc->info->flags & SFC_FLAG_RESET))
		return true;

	/* Board-level logic forces a power-on-reset */
	if (sfc->reset_por)
		return true;

	/* Reset is not properly handled and may result in failure to reboot */
	return false;
}

/* Prepare a ERASE sequence */
static void stsfc_prepare_erasesec_seq(struct stsfc *sfc)
{
	struct stsfc_seq *seq = &sfc->stsfc_seq_erase_sector;
	u32 *addr_cfg = &sfc->seq_erase_addr_cfg;
	u8 cycles = sfc->info->flags & SFC_FLAG_32BIT_ADDR ?
			 SFC_32BIT_ADDR : SFC_24BIT_ADDR;
	u8 i = 0;

	seq->opc[0] = SFC_OPC_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		      SFC_OPC_PADS_1 |
		      SFC_OPC_TRANSMIT_DATA(SPINOR_OP_WREN);
	seq->opc[1] = SFC_OPC_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		      SFC_OPC_PADS_1 |
		      SFC_OPC_CS_ASSERT |
		      SFC_OPC_TRANSMIT_DATA(SPINOR_OP_SE);

	seq->seq[i++] = STSFC_INST_WR_OPC_0;
	seq->seq[i++] = STSFC_INST_WR_OPC_1;
	seq->seq[i++] = STSFC_INST_ADDR;
	seq->seq[i++] = STSFC_INST_STOP;

	seq->seq_cfg = SFC_SC_START_SEQ |
		       SFC_SC_WRITE_NOT_READ;

	*addr_cfg = SFC_ADDR_NO_OF_CYC(cycles - 1) |
		    SFC_ADDR_PADS_1;
	if (sfc->info->flags & SFC_FLAG_32BIT_ADDR)
		*addr_cfg |= SFC_ADDR_4_BYTE_MODE;
}

/* Search for preferred configuration based on available flags */
static struct seq_rw_config *
stsfc_search_seq_rw_configs(struct stsfc *sfc,
			    struct seq_rw_config cfgs[])
{
	struct seq_rw_config *config;
	int flags = sfc->info->flags;

	for (config = cfgs; config->cmd != 0; config++)
		if ((config->flags & flags) == config->flags)
			return config;

	return NULL;
}

/* Prepare a READ/WRITE sequence according to configuration parameters */
static int stsfc_prepare_rw_seq(struct stsfc *sfc, struct seq_rw_config *cfg)
{
	struct stsfc_seq *seq = &sfc->stsfc_seq_read;
	u32 *addr_cfg = &sfc->seq_read_addr_cfg;
	u8 cycles;
	u8 i = 0;

	if (cfg->write) {
		seq = &sfc->stsfc_seq_write;
		addr_cfg = &sfc->seq_write_addr_cfg;
	}

	/* Add READ/WRITE OPC  */
	seq->opc[i++] = SFC_OPC_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
			SFC_OPC_PADS_1 |
			SFC_OPC_CS_ASSERT |
			SFC_OPC_TRANSMIT_DATA(cfg->cmd);

	/* Add WREN OPC for a WRITE sequence */
	if (cfg->write)
		seq->opc[i] = SFC_OPC_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
			      SFC_OPC_PADS_1 |
			      SFC_OPC_TRANSMIT_DATA(SPINOR_OP_WREN);
	i++;

	/* Add mode data bits (no. of pads taken from addr cfg) */
	if (cfg->mode_cycles)
		seq->opc[i] = SFC_OPC_NO_OF_CYC(cfg->mode_cycles - 1) |
			      SFC_OPC_PADS(cfg->addr_pads) |
			      SFC_OPC_CS_ASSERT |
			      SFC_OPC_TRANSMIT_DATA(cfg->mode_data);
	i++;

	/* Add dummy data bits (no. of pads taken from addr cfg) */
	if (cfg->dummy_cycles)
		seq->opc[i] = SFC_OPC_NO_OF_CYC(cfg->dummy_cycles - 1) |
			      SFC_OPC_PADS(cfg->addr_pads) |
			      SFC_OPC_CS_ASSERT;
	i++;

	/* RDSR OPC #4 for a WRITE sequence (for WTR) */
	if (cfg->write)
		seq->opc[i] = SFC_OPC_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
			      SFC_OPC_PADS_1 |
			      SFC_OPC_CS_ASSERT |
			      SFC_OPC_TRANSMIT_DATA(SPINOR_OP_RDSR);

	/* Address configuration (24 or 32-bit addresses) */
	cycles = sfc->info->flags & SFC_FLAG_32BIT_ADDR ?
		 SFC_32BIT_ADDR : SFC_24BIT_ADDR;
	switch (cfg->addr_pads) {
	case SFC_PADS_1:
		break;
	case SFC_PADS_2:
		cycles /= 2;
		break;
	case SFC_PADS_4:
		cycles /= 4;
		break;
	default:
		WARN_ON(1);
		return -1;
	}
	*addr_cfg = SFC_ADDR_NO_OF_CYC(cycles - 1) |
		    SFC_ADDR_PADS(cfg->addr_pads) |
		    SFC_ADDR_CS_ASSERT;
	if (sfc->info->flags & SFC_FLAG_32BIT_ADDR)
		*addr_cfg |= SFC_ADDR_4_BYTE_MODE;

	/* Data/Sequence configuration */
	seq->seq_cfg = SFC_SC_START_SEQ |
		       SFC_SC_DMA_ON |
		       SFC_SC_PADS(cfg->data_pads);
	if (cfg->write)
		seq->seq_cfg |= SFC_SC_WRITE_NOT_READ;

	/* Instruction sequence */
	i = 0;
	if (cfg->write)
		seq->seq[i++] = STSFC_INST_WR_OPC_1;

	seq->seq[i++] = STSFC_INST_WR_OPC_0;

	seq->seq[i++] = STSFC_INST_ADDR;

	if (cfg->mode_cycles)
		seq->seq[i++] = STSFC_INST_WR_OPC_2;

	if (cfg->dummy_cycles)
		seq->seq[i++] = STSFC_INST_WR_OPC_3;

	seq->seq[i++] = STSFC_INST_DATA_BDM;

	if (cfg->write)
		seq->seq[i++] = STSFC_INST_WTR_0;

	if (cfg->write)
		seq->seq[i++] = STSFC_INST_LOOP_0;

	seq->seq[i++] = STSFC_INST_STOP;

	return 0;
}

static int stsfc_search_prepare_rw_seq(struct stsfc *sfc,
				       struct seq_rw_config *cfgs)
{
	struct seq_rw_config *config;

	config = stsfc_search_seq_rw_configs(sfc, cfgs);
	if (!config) {
		dev_err(sfc->dev, "failed to find suitable config\n");
		return -EINVAL;
	}

	return stsfc_prepare_rw_seq(sfc, config);
}

/* Prepare a READ/WRITE/ERASE 'default' sequences */
static int stsfc_prepare_rwe_seqs_default(struct stsfc *sfc)
{
	u32 flags = sfc->info->flags;
	int ret;

	/* Configure 'READ' sequence */
	ret = stsfc_search_prepare_rw_seq(sfc, default_read_configs);
	if (ret) {
		dev_err(sfc->dev,
			"failed to prep READ sequence with flags [0x%08x]\n",
			flags);
		return ret;
	}

	/* Configure 'WRITE' sequence */
	ret = stsfc_search_prepare_rw_seq(sfc, default_write_configs);
	if (ret) {
		dev_err(sfc->dev,
			"failed to prep WRITE sequence with flags [0x%08x]\n",
			flags);
		return ret;
	}

	/* Configure 'ERASE_SECTOR' sequence */
	stsfc_prepare_erasesec_seq(sfc);

	return 0;
}

/*
 * Configure the "BPx" Block Protection Boundaries
 *
 * The BPx Block Protection scheme allows an area of Flash to be protected from
 * Write and Erase operations.  The protected area is determined by the BPx bits
 * which define an address boundary, and the TB bit which selects whether
 * protection is active below or above the boundary.  The permissible boundary
 * addresses are determined by the size of the device, the state of TB, and
 * the number of boundaries supported by the device.
 *
 * Example: BP[2-0], 8 boundaries, TB = 1:
 *                   __________________________________________________________
 *          Device: | |  |   |     |        |             |                    |
 *                  |_|__|___|_____|________|_____________|____________________|
 *      Protection
 *     from bottom: |--....->
 *                NONE                                                       ALL
 *                  ^ ^  ^   ^     ^        ^             ^                    ^
 * BP[2-0]:         | |  |   |     |        |             |                    |
 *   0 0 0 ( 0/64) -  |  |   |     |        |             |                    |
 *   0 0 1 ( 1/64) ---   |   |     |        |             |                    |
 *   0 1 0 ( 2/64) ------    |     |        |             |                    |
 *   0 1 1 ( 4/64) ----------      |        |             |                    |
 *   1 0 0 ( 8/64) ----------------         |             |                    |
 *   1 0 1 (16/64) -------------------------              |                    |
 *   1 1 0 (32/64) ---------------------------------------                     |
 *   1 1 1 (64/64) ------------------------------------------------------------
 *
 */
static void stsfc_configure_bpx_boundaries(struct stsfc *sfc, int tb,
					   u64 size, int n_bnds)
{
	int i;
	u64 bpx_size = size;

	WARN_ON(n_bnds > SFC_BPX_MAX_N_BNDS);

	sfc->bpx_tb = tb;
	sfc->bpx_n_bnds = n_bnds;

	sfc->bpx_bnds[0] = tb ? 0 : size;
	do_div(bpx_size, BIT(n_bnds - 2));

	for (i = 1; i < n_bnds; i++) {
		sfc->bpx_bnds[i] = tb ? bpx_size : size - bpx_size;
		bpx_size *= 2;
	}
}

/*
 * [N25Qxxx] Configuration
 */

/* N25Q - READ/WRITE/CLEAR NON/VOLATILE STATUS/CONFIG Registers */
#define N25Q_CMD_WREN			0x06
#define N25Q_CMD_WRVCR			0x81
#define N25Q_CMD_RDVCR			0x85
#define N25Q_CMD_RFSR			0x70
#define N25Q_CMD_CLFSR			0x50
#define N25Q_CMD_RDLOCK			0xe8
#define N25Q_CMD_WRLOCK			0xe5

/* N25Q Flags Status Register: Error Flags */
#define N25Q_FLAGS_ERR_ERASE		BIT(5)
#define N25Q_FLAGS_ERR_PROG		BIT(4)
#define N25Q_FLAGS_ERR_VPP		BIT(3)
#define N25Q_FLAGS_ERR_PROT		BIT(1)
#define N25Q_FLAGS_ERROR		(N25Q_FLAGS_ERR_ERASE | \
					 N25Q_FLAGS_ERR_PROG | \
					 N25Q_FLAGS_ERR_VPP | \
					 N25Q_FLAGS_ERR_PROT)

/*
 * N25Q 3-byte Address READ configurations
 *	- 'FAST' variants configured for 8 dummy cycles.
 *
 * Note, the number of dummy cycles used for 'FAST' READ operations is
 * configurable and would normally be tuned according to the READ command and
 * operating frequency.  However, this applies universally to all 'FAST' READ
 * commands, including those used by the SPIBoot controller, and remains in
 * force until the device is power-cycled.  Since the SPIBoot controller is
 * hard-wired to use 8 dummy cycles, we must configure the device to also use 8
 * cycles.
 */
static struct seq_rw_config n25q_read3_configs[] = {
	{SFC_FLAG_READ_1_4_4, SPINOR_OP_READ_1_4_4, 0,
		SFC_PADS_4, SFC_PADS_4, 0x00, 0, 8},
	{SFC_FLAG_READ_1_1_4, SPINOR_OP_READ_1_1_4, 0,
		SFC_PADS_1, SFC_PADS_4, 0x00, 0, 8},
	{SFC_FLAG_READ_1_2_2, SPINOR_OP_READ_1_2_2, 0,
		SFC_PADS_2, SFC_PADS_2, 0x00, 0, 8},
	{SFC_FLAG_READ_1_1_2, SPINOR_OP_READ_1_1_2, 0,
		SFC_PADS_1, SFC_PADS_2, 0x00, 0, 8},
	{SFC_FLAG_READ_FAST,  SPINOR_OP_READ_FAST,  0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 8},
	{SFC_FLAG_READ_WRITE, SPINOR_OP_READ,       0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

/*
 * N25Q 4-byte Address READ configurations
 *	- use special 4-byte address READ commands (reduces overheads, and
 *        reduces risk of hitting watchdog reset issues).
 *	- 'FAST' variants configured for 8 dummy cycles (see note above.)
 */
static struct seq_rw_config n25q_read4_configs[] = {
	{SFC_FLAG_READ_1_4_4, SPINOR_OP_READ_1_4_4_4B, 0,
		SFC_PADS_4, SFC_PADS_4, 0x00, 0, 8},
	{SFC_FLAG_READ_1_1_4, SPINOR_OP_READ4_1_1_4, 0,
		SFC_PADS_1, SFC_PADS_4, 0x00, 0, 8},
	{SFC_FLAG_READ_1_2_2, SPINOR_OP_READ_1_2_2_4B, 0,
		SFC_PADS_2, SFC_PADS_2, 0x00, 0, 8},
	{SFC_FLAG_READ_1_1_2, SPINOR_OP_READ4_1_1_2, 0,
		SFC_PADS_1, SFC_PADS_2, 0x00, 0, 8},
	{SFC_FLAG_READ_FAST,  SPINOR_OP_READ4_FAST,  0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 8},
	{SFC_FLAG_READ_WRITE, SPINOR_OP_READ4,       0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

static void stsfc_n25q_enter_32bit_addr(struct stsfc *sfc, bool enter)
{
	u32 cmd = enter ? SPINOR_OP_EN4B : SPINOR_OP_EX4B;
	u32 tcfg_reg;

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(SPINOR_OP_WREN);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(cmd);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);
}

static void stsfc_n25q_clear_flag(struct stsfc *sfc)
{
	u32 tcfg_reg;

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(N25Q_CMD_CLFSR);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);
}

static void stsfc_n25q_set_dummy_cycles(struct stsfc *sfc, u8 cycles)
{
	u32 tcfg_reg, cfg_reg;
	u8 vcr;

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(N25Q_CMD_RDVCR);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		  SFC_CFG_CDM_PADS_1;

	stsfc_load_config_cdm(sfc, cfg_reg);

	vcr = stsfc_read_data_cdm(sfc) & 0xff;

	vcr = (vcr & 0x0f) | (cycles << 4);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(N25Q_CMD_WREN);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(N25Q_CMD_WRVCR);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(vcr);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);
}

static int stsfc_n25q_config(struct stsfc *sfc)
{
	struct flash_info *info = sfc->info;
	struct seq_rw_config *read_cfg;
	struct seq_rw_config *config;
	u32 flags = info->flags;
	u8 sta;
	int ret;
	bool soc_reset;

	/* Configure 'READ' sequence */
	if (flags & SFC_FLAG_32BIT_ADDR)
		read_cfg = n25q_read4_configs;
	else
		read_cfg = n25q_read3_configs;

	config = stsfc_search_seq_rw_configs(sfc, read_cfg);
	if (!config) {
		dev_err(sfc->dev, "failed to find suitable config\n");
		return -EINVAL;
	}

	stsfc_prepare_rw_seq(sfc, config);

	/* Configure 'WRITE' sequence (default configs) */
	ret = stsfc_search_prepare_rw_seq(sfc, default_write_configs);
	if (ret) {
		dev_err(sfc->dev,
			"preparing WRITE sequence using flags [0x%08x] failed\n",
			flags);
		return ret;
	}

	/* Configure 'ERASE_SECTOR' sequence */
	stsfc_prepare_erasesec_seq(sfc);

	/* Configure block locking scheme */
	if (flags & SFC_FLAG_BLK_LOCKING) {
		sfc->lock_cycles_addr = flags & SFC_FLAG_32BIT_ADDR ?
					SFC_32BIT_ADDR : SFC_24BIT_ADDR;
		sfc->op_rd_lock = N25Q_CMD_RDLOCK;
		sfc->op_wr_lock = N25Q_CMD_WRLOCK;

		sfc->configuration |= SFC_CFG_RD_WR_LOCK_REG;

		sfc->p4k_bot_end = 0;
		sfc->p4k_top_start = info->sector_size * info->n_sectors;

		sfc->lock_mask = 0x1;
		sfc->lock_val[SFC_BLOCK_UNLOCKED] = 0x0;
		sfc->lock_val[SFC_BLOCK_LOCKED] = 0x1;
	}

	/* Check/Clear Error Flags */
	sfc->configuration |= SFC_CFG_N25Q_CHECK_ERROR_FLAGS;
	stsfc_read_status(sfc, N25Q_CMD_RFSR, &sta);
	if (sta & N25Q_FLAGS_ERROR)
		stsfc_n25q_clear_flag(sfc);

	/* Configure read dummy cycles */
	stsfc_n25q_set_dummy_cycles(sfc, config->dummy_cycles);

	/* Configure 32-bit address support */
	if (flags & SFC_FLAG_32BIT_ADDR) {
		sfc->enter_32bit_addr = stsfc_n25q_enter_32bit_addr;

		soc_reset = stsfc_can_handle_soc_reset(sfc);
		if (soc_reset || !sfc->booted_from_spi)
			/*
			 * If we can handle SoC resets, we enable 32-bit
			 * address mode pervasively
			 */
			sfc->enter_32bit_addr(sfc, true);
		else
			/*
			 * If not, enable/disable for WRITE and ERASE
			 * operations (READ uses special commands)
			 */
			sfc->configuration |= SFC_CFG_WRITE_TOGGLE_32BIT_ADDR |
					      SFC_CFG_ERASE_TOGGLE_32BIT_ADDR |
					      SFC_CFG_LOCK_TOGGLE_32BIT_ADDR;
	}

	return 0;
}

static int stsfc_n25q_resume(struct stsfc *sfc)
{
	struct flash_info *info = sfc->info;
	u32 flags = info->flags;
	u8 sta;

	/* Check/Clear Error Flags */
	stsfc_read_status(sfc, N25Q_CMD_RFSR, &sta);
	if (sta & N25Q_FLAGS_ERROR)
		stsfc_n25q_clear_flag(sfc);

	/*
	 * If we can handle SoC resets, we enable 32-bit
	 * address mode pervasively
	 */
	if (flags & SFC_FLAG_32BIT_ADDR &&
	    (stsfc_can_handle_soc_reset(sfc) || !sfc->booted_from_spi)) {
		sfc->enter_32bit_addr(sfc, true);
	}

	return 0;
}

/*
 * [MX25xxx] Configuration
 */
#define MX25_CMD_WRITE_1_4_4		0x38
#define MX25_CMD_RDCR			0x15
#define MX25_CMD_RDSCUR			0x2b
#define MX25_CMD_RDSFDP			0x5a
#define MX25_CMD_SBLK			0x36
#define MX25_CMD_SBULK			0x39
#define MX25_CMD_RDBLOCK		0x3c
#define MX25_CMD_RDDPB			0xe0
#define MX25_CMD_WRDPB			0xe1

#define MX25_SR_QE			BIT(6)
#define MX25_SCUR_WPSEL			BIT(7)
#define MX25_CR_TB			BIT(3)

#define MX25_RDSFDP_DUMMY_CYCLES	8
#define MX25_RDSFDP_DATA_CYCLES		32

#define MX25L32_DEVICE_ID		0x16
#define MX25L128_DEVICE_ID		0x18

#define MX25_LOCK_OPCODE_ADDR		0x68
#define MX25_LOCK_OPCODE_MASK		0x3fc

/* Mx25 WRITE configurations, in order of preference */
static struct seq_rw_config mx25_write_configs[] = {
	{SFC_FLAG_WRITE_1_4_4, MX25_CMD_WRITE_1_4_4, 1,
		SFC_PADS_4, SFC_PADS_4, 0x00, 0, 0},
	{SFC_FLAG_READ_WRITE,  SPINOR_OP_WRITE,      1,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

static void stsfc_mx25_enter_32bit_addr(struct stsfc *sfc, bool enter)
{
	u32 cmd = enter ? SPINOR_OP_EN4B : SPINOR_OP_EX4B;
	u32 tcfg_reg;

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(cmd);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);
}

static void stsfc_mx25_enter_quad_mode(struct stsfc *sfc, bool enter)
{
	u8 sr;

	stsfc_read_status(sfc, SPINOR_OP_RDSR, &sr);
	sr = (enter ? sr | MX25_SR_QE : sr & ~MX25_SR_QE);
	stsfc_write_status(sfc, SPINOR_OP_WRSR, sr, 1, true);
}

/*
 * Read SFDP mode at @0x68, 8 dummy cycles required
 * bits 2-9 => opcode used
 */
static int stsfc_mx25_read_lock_opcode(struct stsfc *sfc,
				       u8 *lock_opcode)
{
	u32 tcfg_reg;
	u32 cfg_reg;
	u32 data_reg;

	/* RDSFDP command sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(MX25_CMD_RDSFDP);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	/* Address sent */
	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(SFC_24BIT_ADDR - 1) |
		  SFC_CFG_CDM_PADS_1 |
		  SFC_CFG_CDM_CS_ASSERT;

	stsfc_load_config_cdm(sfc, cfg_reg);

	stsfc_load_data_cdm(sfc, SFC_3_BYTES_ADDR(MX25_LOCK_OPCODE_ADDR));

	/* Dummy cycles sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(MX25_RDSFDP_DUMMY_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT;

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(MX25_RDSFDP_DATA_CYCLES - 1) |
		  SFC_CFG_CDM_PADS_1;

	stsfc_load_config_cdm(sfc, cfg_reg);

	/* Data read */
	data_reg = stsfc_read_data_cdm(sfc);

	*lock_opcode = (data_reg & MX25_LOCK_OPCODE_MASK) >> 2;

	return 0;
}

static int stsfc_mx25_config(struct stsfc *sfc)
{
	struct flash_info *info = sfc->info;
	u32 *flags = &info->flags;
	u64 size = info->sector_size * info->n_sectors;
	u32 data_pads;
	u8 sta, lock_opcode, i;
	int ret, tb, n_bnds;
	bool soc_reset;

	/* Configure 'READ' sequence */
	ret = stsfc_search_prepare_rw_seq(sfc, default_read_configs);
	if (ret) {
		dev_err(sfc->dev,
			"failed to prep READ sequence with flags [0x%08x]\n",
			*flags);
		return ret;
	}

	/* Configure 'WRITE' sequence */
	ret = stsfc_search_prepare_rw_seq(sfc, mx25_write_configs);
	if (ret) {
		dev_err(sfc->dev,
			"failed to prep WRITE sequence with flags [0x%08x]\n",
			*flags);
		return ret;
	}

	/* Configure 'ERASE_SECTOR' sequence */
	stsfc_prepare_erasesec_seq(sfc);

	/* Configure 32-bit Address Support */
	if (*flags & SFC_FLAG_32BIT_ADDR) {
		/* Configure 'enter_32bitaddr' FSM sequence */
		sfc->enter_32bit_addr = stsfc_mx25_enter_32bit_addr;

		soc_reset = stsfc_can_handle_soc_reset(sfc);
		if (soc_reset || !sfc->booted_from_spi)
			/*
			 * If we can handle SoC resets, we enable 32-bit address
			 * mode pervasively
			 */
			sfc->enter_32bit_addr(sfc, true);
		else
			/*
			 * Else, enable/disable 32-bit addressing before/after
			 * each operation
			 */
			sfc->configuration |= SFC_CFG_READ_TOGGLE_32BIT_ADDR |
					      SFC_CFG_WRITE_TOGGLE_32BIT_ADDR |
					      SFC_CFG_ERASE_TOGGLE_32BIT_ADDR;
	}

	/*
	 * Check WPSEL
	 * WPSEL = 0 => Block Lock protection mode
	 * WPSEL = 1 => Individual block lock protection mode
	 */
	stsfc_read_status(sfc, MX25_CMD_RDSCUR, &sta);
	if (sta & MX25_SCUR_WPSEL) {
		/* Individual block lock protection mode detected */

		/* Read opcode used to lock */
		lock_opcode = 0;
		stsfc_mx25_read_lock_opcode(sfc, &lock_opcode);
		if (lock_opcode == MX25_CMD_SBLK) {
			*flags |= SFC_FLAG_BLK_LOCKING;

			sfc->lock_cycles_addr = SFC_24BIT_ADDR;
			sfc->op_rd_lock = MX25_CMD_RDBLOCK;
			sfc->op_lock = MX25_CMD_SBLK;
			sfc->op_unlock = MX25_CMD_SBULK;

			/*
			 * Handle 4KiB parameter sectors
			 * at the top and the bottom
			 */
			sfc->p4k_bot_end = 16 * SZ_4K;
			sfc->p4k_top_start = size - (16 * SZ_4K);

			sfc->lock_mask = 0xff;
			sfc->lock_val[SFC_BLOCK_UNLOCKED] = 0x00;
			sfc->lock_val[SFC_BLOCK_LOCKED] = 0xff;
		} else if (lock_opcode == MX25_CMD_WRDPB) {
			*flags |= SFC_FLAG_BLK_LOCKING;

			sfc->lock_cycles_addr = SFC_32BIT_ADDR;
			sfc->op_rd_lock = MX25_CMD_RDDPB;
			sfc->op_wr_lock = MX25_CMD_WRDPB;

			sfc->configuration |= SFC_CFG_RD_WR_LOCK_REG;

			/*
			 *Handle 4KiB parameter sectors
			 * at the top and the bottom
			 */
			sfc->p4k_bot_end = 16 * SZ_4K;
			sfc->p4k_top_start = size - (16 * SZ_4K);

			sfc->lock_mask = 0xff;
			sfc->lock_val[SFC_BLOCK_UNLOCKED] = 0x00;
			sfc->lock_val[SFC_BLOCK_LOCKED] = 0xff;
		} else
			/* Lock opcode is not supported */
			dev_warn(sfc->dev,
				 "Lock/unlock command %02x not supported.\n",
				 lock_opcode);
	} else {
		u8 cr;

		/* BP lock lock protection mode detected */
		*flags |= SFC_FLAG_BPX_LOCKING;

		/* Get 'tb' bit */
		stsfc_read_status(sfc, MX25_CMD_RDCR, &cr);
		tb = (cr & MX25_CR_TB) ? 1 : 0;

		/* Configure BPx bits */
		sfc->bpx_n_bits = 4;
		sfc->bpx_sr_masks[0] = SR_BP0;
		sfc->bpx_sr_masks[1] = SR_BP1;
		sfc->bpx_sr_masks[2] = SR_BP2;
		sfc->bpx_sr_masks[3] = SR_BP3;

		/* Configure BPx boundaries: 16 levels from top/bottom */
		if (info->readid[2] == MX25L32_DEVICE_ID)
			n_bnds = 8;
		else if (info->readid[2] == MX25L128_DEVICE_ID)
			n_bnds = 10;
		else
			n_bnds = 11;

		stsfc_configure_bpx_boundaries(sfc, tb, size, n_bnds);

		for (i = n_bnds; i < 16; i++)
			sfc->bpx_bnds[i] = sfc->bpx_bnds[n_bnds - 1];
	}

	/* Check status of 'QE' bit, update if required. */
	stsfc_read_status(sfc, SPINOR_OP_RDSR, &sta);
	data_pads = (sfc->stsfc_seq_read.seq_cfg >> 25) & 0x3;
	if (data_pads == SFC_PADS_4) {
		if (!(sta & MX25_SR_QE)) {
			/* Set 'QE' */
			sta |= MX25_SR_QE;

			stsfc_write_status(sfc, SPINOR_OP_WRSR, sta, 1, true);
		}

		sfc->configuration |= SFC_CFG_MX25_TOGGLE_QE_BIT;
	} else if (sta & MX25_SR_QE) {
		/* Clear 'QE' */
		sta &= ~MX25_SR_QE;

		stsfc_write_status(sfc, SPINOR_OP_WRSR, sta, 1, true);
	}

	return 0;
}

static int stsfc_mx25_resume(struct stsfc *sfc)
{
	struct flash_info *info = sfc->info;
	u32 flags = info->flags;
	bool soc_reset;

	/* Configure 32-bit Address Support */
	if (flags & SFC_FLAG_32BIT_ADDR) {
		soc_reset = stsfc_can_handle_soc_reset(sfc);
		if (soc_reset || !sfc->booted_from_spi)
			/*
			 * If we can handle SoC resets, we enable 32-bit address
			 * mode pervasively
			 */
			sfc->enter_32bit_addr(sfc, true);
	}

	return 0;
}

/*
 * [S25FSxxx] Configuration
 */
#define S25FS_CMD_RDAR			0x65
#define S25FS_CMD_4P4E			0x21
#define S25FS_CMD_P4E			0x20

#define S25FS_ANYREG_DUMMY_CYCLES	8
#define S25FS_ANYREG_DATA_CYCLES	8

#define S25FS_4K_SECTOR			8
#define S25FS_FAMILY			0x81
#define S25FS_CR3_4K_ERASE		BIT(3)
#define S25FS_CR1_TBARM			BIT(2)

/*
 * S25FSxxxS devices provide three ways of supporting 32-bit addressing: Bank
 * Register, Extended Address Modes, and a 32-bit address command set.  The
 * 32-bit address command set is used here, since it avoids any problems with
 * entering a state that is incompatible with the SPIBoot Controller.
 */

static struct seq_rw_config stsfc_s25fs_read4_configs[] = {
	{SFC_FLAG_READ_1_4_4,  SPINOR_OP_READ_1_4_4_4B,  0,
		SFC_PADS_4, SFC_PADS_4, 0x00, 2, 8},
	{SFC_FLAG_READ_1_2_2,  SPINOR_OP_READ_1_2_2_4B,  0,
		SFC_PADS_2, SFC_PADS_2, 0x00, 4, 8},
	{SFC_FLAG_READ_FAST,   SPINOR_OP_READ4_FAST,   0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 8},
	{SFC_FLAG_READ_WRITE,  SPINOR_OP_READ4,        0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

static struct seq_rw_config stsfc_s25fs_read3_configs[] = {
	{SFC_FLAG_READ_1_4_4,  SPINOR_OP_READ_1_4_4,  0,
		SFC_PADS_4, SFC_PADS_4, 0x00, 2, 8},
	{SFC_FLAG_READ_1_2_2,  SPINOR_OP_READ_1_2_2,  0,
		SFC_PADS_2, SFC_PADS_2, 0x00, 4, 8},
	{SFC_FLAG_READ_FAST,   SPINOR_OP_READ_FAST,   0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 8},
	{SFC_FLAG_READ_WRITE,  SPINOR_OP_READ,        0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

static u8 stsfc_s25fs_read_any_reg(struct stsfc *sfc, u32 offs)
{
	u32 tcfg_reg;
	u32 cfg_reg;

	/* RDSFDP command sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(S25FS_CMD_RDAR);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	/* Address sent */
	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(SFC_24BIT_ADDR - 1) |
		  SFC_CFG_CDM_PADS_1 |
		  SFC_CFG_CDM_CS_ASSERT;

	stsfc_load_config_cdm(sfc, cfg_reg);

	stsfc_load_data_cdm(sfc, SFC_3_BYTES_ADDR(offs));

	/* Dummy cycles sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(S25FS_ANYREG_DUMMY_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT;

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(S25FS_ANYREG_DATA_CYCLES - 1) |
		  SFC_CFG_CDM_PADS_1;

	stsfc_load_config_cdm(sfc, cfg_reg);

	return stsfc_read_data_cdm(sfc) & 0xff;
}

static int stsfc_s25fs_erase_4k_sector(struct stsfc *sfc, u32 offset)
{
	struct stsfc_seq *seq = &sfc->stsfc_seq_erase_sector;
	u32 offs = offset;
	u32 offs_end = offset + S25FS_4K_SECTOR * SZ_4K;
	u32 seq_opc = seq->opc[1];
	int ret = 0;

	seq->opc[1] = SFC_OPC_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		      SFC_OPC_PADS_1 |
		      SFC_OPC_CS_ASSERT |
		      SFC_OPC_TRANSMIT_DATA(sfc->p4k_erase_cmd);

	while (offs < offs_end) {
		ret = stsfc_erase_sector(sfc, offs);
		if (ret < 0)
			break;

		/* Parameter sectors have 4KiB erase granularity */
		offs += SZ_4K;
	}

	/* restore initial erase command */
	seq->opc[1] = seq_opc;

	return ret;
}

/*
 * [S25FLxxx] Configuration
 */
#define S25FL_CMD_WRITE4_1_1_4		0x34
#define S25FL_CMD_SE4			0xdc
#define S25FL_CMD_CLSR			0x30
#define S25FL_CMD_WRITE4		0x12
#define S25FL_CMD_DYBWR			0xe1
#define S25FL_CMD_DYBRD			0xe0

#define S25FL_SR1_E_ERR			BIT(5)
#define S25FL_SR1_P_ERR			BIT(6)
#define S25FL_SR2_QE			BIT(1)
#define S25FL_CR1_TBPROT		BIT(5)

/*
 * S25FLxxxS devices provide three ways of supporting 32-bit addressing: Bank
 * Register, Extended Address Modes, and a 32-bit address command set.  The
 * 32-bit address command set is used here, since it avoids any problems with
 * entering a state that is incompatible with the SPIBoot Controller.
 */
static struct seq_rw_config stsfc_s25fl_read4_configs[] = {
	{SFC_FLAG_READ_1_4_4,  SPINOR_OP_READ_1_4_4_4B,  0,
		SFC_PADS_4, SFC_PADS_4, 0x00, 2, 4},
	{SFC_FLAG_READ_1_1_4,  SPINOR_OP_READ4_1_1_4,  0,
		SFC_PADS_1, SFC_PADS_4, 0x00, 0, 8},
	{SFC_FLAG_READ_1_2_2,  SPINOR_OP_READ_1_2_2_4B,  0,
		SFC_PADS_2, SFC_PADS_2, 0x00, 4, 0},
	{SFC_FLAG_READ_1_1_2,  SPINOR_OP_READ4_1_1_2,  0,
		SFC_PADS_1, SFC_PADS_2, 0x00, 0, 8},
	{SFC_FLAG_READ_FAST,   SPINOR_OP_READ4_FAST,   0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 8},
	{SFC_FLAG_READ_WRITE,  SPINOR_OP_READ4,        0,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

static struct seq_rw_config stsfc_s25fl_write4_configs[] = {
	{SFC_FLAG_WRITE_1_1_4, S25FL_CMD_WRITE4_1_1_4, 1,
		SFC_PADS_1, SFC_PADS_4, 0x00, 0, 0},
	{SFC_FLAG_READ_WRITE,  S25FL_CMD_WRITE4,       1,
		SFC_PADS_1, SFC_PADS_1, 0x00, 0, 0},
	{},
};

static void stsfc_s25fl_prepare_erasesec_seq_32(struct stsfc *sfc)
{
	struct stsfc_seq *seq = &sfc->stsfc_seq_erase_sector;
	u32 *addr_cfg = &sfc->seq_erase_addr_cfg;

	seq->opc[1] = SFC_OPC_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		      SFC_OPC_PADS_1 |
		      SFC_OPC_CS_ASSERT |
		      SFC_OPC_TRANSMIT_DATA(S25FL_CMD_SE4);

	*addr_cfg = SFC_ADDR_NO_OF_CYC(SFC_32BIT_ADDR - 1) |
		    SFC_ADDR_PADS_1 |
		    SFC_ADDR_4_BYTE_MODE;
}

static void stsfc_s25fl_clear_flag(struct stsfc *sfc)
{
	u32 tcfg_reg;

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(S25FL_CMD_CLSR);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(SPINOR_OP_WRDI);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);
}

static int stsfc_s25fl_config(struct stsfc *sfc)
{
	struct flash_info *info = sfc->info;
	u64 size = info->sector_size * info->n_sectors;
	u32 flags = info->flags;
	u32 data_pads;
	u16 sta_wr;
	u8 sr1, cr1;
	bool update_sr = false;
	int ret, tbprot;
	struct seq_rw_config *read_cfg;
	struct seq_rw_config *write_cfg;

	/*
	 * WRSR must always cover CONFIG register to prevent loss of QUAD bit
	 * state
	 */
	sfc->configuration |= SFC_CFG_WRSR_FORCE_16BITS;

	/*
	 * S25FLxxx devices support Program and Error error flags
	 * Configure driver to check flags and clear if necessary.
	 */
	sfc->configuration |= SFC_CFG_S25FL_CHECK_ERROR_FLAGS;

	/*
	 * Prepare Read/Write/Erase sequences according to S25FLxxx
	 * 32-bit address command set
	 */
	if (flags & SFC_FLAG_32BIT_ADDR) {
		if (info->readid[5] == S25FS_FAMILY)
			read_cfg = stsfc_s25fs_read4_configs;
		else
			read_cfg = stsfc_s25fl_read4_configs;

		write_cfg = stsfc_s25fl_write4_configs;

		stsfc_prepare_erasesec_seq(sfc);
		stsfc_s25fl_prepare_erasesec_seq_32(sfc);
	} else {
		if (info->readid[5] == S25FS_FAMILY)
			read_cfg = stsfc_s25fs_read3_configs;
		else
			read_cfg = default_read_configs;

		write_cfg = default_write_configs;

		stsfc_prepare_erasesec_seq(sfc);
	}

	ret = stsfc_search_prepare_rw_seq(sfc, read_cfg);
	if (ret)
		return ret;

	ret = stsfc_search_prepare_rw_seq(sfc, write_cfg);
	if (ret)
		return ret;

	/*
	 * In case of S25FS family, erase functionality has a different
	 * behavior as other Spansion family if hybrid architecture is defined.
	 * Check in CR3 if hybrid architecture is defined
	 * Read CR3 at @0x04
	 * Check in CR1 tbparm to distinguish memory organization
	 * For S25FS family, 8 sections of 4K has to be erased with a 4K_ERASE
	 * command. Other commands are not allowed.
	 * Identify the 64K/256K section start address of these 8 4K sections
	 */
	if (info->readid[5] == S25FS_FAMILY) {
		u8 cr3 = stsfc_s25fs_read_any_reg(sfc, 0x04);
		if (!(cr3 & S25FS_CR3_4K_ERASE)) {
			/* Get 'TBPARM' bit */
			stsfc_read_status(sfc, SPINOR_OP_RDSR2, &cr1);
			if (cr1 & S25FS_CR1_TBARM)
				/* 4-kB physical sectors at top */
				sfc->p4k_sector_address = size -
							  info->sector_size;
			else
				/* 4-kB physical sectors at bottom */
				sfc->p4k_sector_address = 0;

			/* Set 4k erase command */
			if (flags & SFC_FLAG_32BIT_ADDR)
				sfc->p4k_erase_cmd = S25FS_CMD_4P4E;
			else
				sfc->p4k_erase_cmd = S25FS_CMD_P4E;

			sfc->configuration |= CFG_S25FS_4K_ERASE;
		}
	}

	/* Configure block locking support */
	if (flags & SFC_FLAG_BLK_LOCKING) {
		sfc->lock_cycles_addr = SFC_32BIT_ADDR;
		sfc->op_rd_lock = S25FL_CMD_DYBRD;
		sfc->op_wr_lock = S25FL_CMD_DYBWR;

		sfc->configuration |= SFC_CFG_RD_WR_LOCK_REG;

		/*
		 * Handle 4KiB parameter sectors: catch-all approach to
		 * accommodate all variants (e.g. top vs. bottom and 16 vs. 32
		 * parameter sectors).
		 */
		sfc->p4k_bot_end = 32 * SZ_4K;
		sfc->p4k_top_start = size - (32 * SZ_4K);

		sfc->lock_mask = 0xff;
		sfc->lock_val[SFC_BLOCK_UNLOCKED] = 0xff;
		sfc->lock_val[SFC_BLOCK_LOCKED] = 0x00;
	} else if (flags & SFC_FLAG_BPX_LOCKING) {
		/* Get 'TBPROT' bit */
		stsfc_read_status(sfc, SPINOR_OP_RDSR2, &cr1);
		tbprot = (cr1 & S25FL_CR1_TBPROT) ? 1 : 0;

		/* Configure BPx bits */
		sfc->bpx_n_bits = 3;
		sfc->bpx_sr_masks[0] = SR_BP0;
		sfc->bpx_sr_masks[1] = SR_BP1;
		sfc->bpx_sr_masks[2] = SR_BP2;

		/* Configure BPx boundaries: 8 levels from top/bottom */
		stsfc_configure_bpx_boundaries(sfc, tbprot, size, 8);
	}

	/* Check status of 'QE' bit, update if required. */
	stsfc_read_status(sfc, SPINOR_OP_RDSR2, &cr1);
	data_pads = (sfc->stsfc_seq_read.seq_cfg >> 25) & 0x3;
	if (data_pads == SFC_PADS_4) {
		if (!(cr1 & S25FL_SR2_QE)) {
			/* Set 'QE' */
			cr1 |= S25FL_SR2_QE;

			update_sr = true;
		}
	} else {
		if (cr1 & S25FL_SR2_QE) {
			/* Clear 'QE' */
			cr1 &= ~S25FL_SR2_QE;

			update_sr = true;
		}
	}
	if (update_sr) {
		stsfc_read_status(sfc, SPINOR_OP_RDSR, &sr1);
		sta_wr = ((u16)cr1  << 8) | sr1;
		stsfc_write_status(sfc, SPINOR_OP_WRSR, sta_wr, 2, true);
	}

	return 0;
}

/*
 * [W25Qxxx] Configuration
 */
#define W25Q_SR1_TB			BIT(5)
#define W25Q_SR1_SEC			BIT(6)
#define W25Q_SR2_QE			BIT(1)
#define W25Q_SR2_CMP			BIT(6)

#define W25Q16_DEVICE_ID		0x15
#define W25Q80_DEVICE_ID		0x14

static int stsfc_w25q_config(struct stsfc *sfc)
{
	struct flash_info *info = sfc->info;
	u32 *flags = &info->flags;
	u64 size = info->sector_size * info->n_sectors;
	u32 data_pads;
	u16 sr_wr;
	u8 sr1, sr2, i;
	bool update_sr = false;
	int ret, tb, sec, cmp, n_bnds;

	/*
	 * WRSR must always cover STATUS register 2 to prevent loss of QUAD bit
	 * and CMP bit state
	 */
	sfc->configuration |= SFC_CFG_WRSR_FORCE_16BITS;

	/* Use default READ/WRITE sequences */
	ret = stsfc_prepare_rwe_seqs_default(sfc);
	if (ret)
		return ret;

	if (*flags & SFC_FLAG_BPX_LOCKING) {
		/* Get 'TB' and 'SEC' bits */
		stsfc_read_status(sfc, SPINOR_OP_RDSR, &sr1);
		tb = (sr1 & W25Q_SR1_TB) ? 1 : 0;
		sec = (sr1 & W25Q_SR1_SEC) ? 1 : 0;

		/* Get 'CMP' bit */
		stsfc_read_status(sfc, SPINOR_OP_RDSR2, &sr2);
		cmp = (sr2 & W25Q_SR2_CMP) ? 1 : 0;

		if (cmp || sec) {
			/* This scheme is not supported */
			dev_warn(sfc->dev, "Lock/unlock scheme not supported. Only schemes whith CMP=0 and SEC=0 is supported.\n");

			/* disable BPx locking support */
			*flags &= ~SFC_FLAG_BPX_LOCKING;
		} else {
			/* Configure BPx bits */
			sfc->bpx_n_bits = 3;
			sfc->bpx_sr_masks[0] = SR_BP0;
			sfc->bpx_sr_masks[1] = SR_BP1;
			sfc->bpx_sr_masks[2] = SR_BP2;

			/* Configure BPx boundaries: 8 levels from top/bottom */
			if (info->readid[2] == W25Q16_DEVICE_ID)
				n_bnds = 7;
			else if (info->readid[2] == W25Q80_DEVICE_ID)
				n_bnds = 6;
			else
				n_bnds = 8;

			stsfc_configure_bpx_boundaries(sfc, tb, size, n_bnds);

			for (i = n_bnds; i < 8; i++)
				sfc->bpx_bnds[i] = sfc->bpx_bnds[n_bnds - 1];
		}
	}

	/* Check status of 'QE' bit, update if required. */
	stsfc_read_status(sfc, SPINOR_OP_RDSR2, &sr2);
	data_pads = (sfc->stsfc_seq_read.seq_cfg >> 25) & 0x3;
	if (data_pads == SFC_PADS_4) {
		if (!(sr2 & W25Q_SR2_QE)) {
			/* Set 'QE' */
			sr2 |= W25Q_SR2_QE;
			update_sr = true;
		}
	} else {
		if (sr2 & W25Q_SR2_QE) {
			/* Clear 'QE' */
			sr2 &= ~W25Q_SR2_QE;
			update_sr = true;
		}
	}
	if (update_sr) {
		/* Write status register */
		stsfc_read_status(sfc, SPINOR_OP_RDSR, &sr1);
		sr_wr = ((u16)sr2 << 8) | sr1;
		stsfc_write_status(sfc, SPINOR_OP_WRSR, sr_wr, 2, true);
	}

	return 0;
}

/*
 *
 * SFC command sequence
 *
 */
static int stsfc_wait_busy(struct stsfc *sfc, unsigned int max_time_ms)
{
	u32 tcfg_reg;
	u32 cfg_reg;
	u32 status;
	unsigned long deadline;
	int timeout = 0;

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(SPINOR_OP_RDSR);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		  SFC_CFG_CDM_PADS_1;

	stsfc_load_config_cdm(sfc, cfg_reg);

#ifdef CONFIG_SPACEX
	/* Can't use jiffies during a panic_write */
	if (oops_in_progress) {
		u32 status;
		u64 timeout = __arch_counter_get_cntvct() +
			      (arch_timer_get_cntfrq() * max_time_ms) / 1000;

		while (__arch_counter_get_cntvct() < timeout) {
			/* Read flash status data */
			status = stsfc_read_data_cdm(sfc) & 0xff;

			if (!(status & SR_WIP))
				return 0;

			/* S25FL: Check/Clear Error Flags */
			if ((sfc->configuration & SFC_CFG_S25FL_CHECK_ERROR_FLAGS) &&
			    ((status & S25FL_SR1_P_ERR) ||
			     (status & S25FL_SR1_E_ERR))) {
				stsfc_s25fl_clear_flag(sfc);
				return -EPROTO;
			}

			if (!timeout)
				/* Restart */
				stsfc_load_transmit_config_cdm(sfc, tcfg_reg);
		}
		return -ETIMEDOUT;
	}
#endif

	/* Repeat until busy bit is deasserted or timeout */
	deadline = jiffies + msecs_to_jiffies(max_time_ms);
	while (!timeout) {
		cond_resched();

		if (time_after_eq(jiffies, deadline))
			timeout = 1;

		/* Read flash status data */
		status = stsfc_read_data_cdm(sfc) & 0xff;

		if (!(status & SR_WIP))
			return 0;

		/* S25FL: Check/Clear Error Flags */
		if ((sfc->configuration & SFC_CFG_S25FL_CHECK_ERROR_FLAGS) &&
		    ((status & S25FL_SR1_P_ERR) ||
		     (status & S25FL_SR1_E_ERR))) {
			stsfc_s25fl_clear_flag(sfc);
			return -EPROTO;
		}

		if (!timeout)
			/* Restart */
			stsfc_load_transmit_config_cdm(sfc, tcfg_reg);
	}

	dev_err(sfc->dev, "Timeout on wait_busy\n");

	return -ETIMEDOUT;
}

static void stsfc_read_status(struct stsfc *sfc, u8 cmd, u8 *data)
{
	u32 tcfg_reg;
	u32 cfg_reg;

	/* Command sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(cmd);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	/* Read data */
	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		  SFC_CFG_CDM_PADS_1;

	stsfc_load_config_cdm(sfc, cfg_reg);

	*data = stsfc_read_data_cdm(sfc) & 0xff;
}

static int stsfc_write_status(struct stsfc *sfc, u8 cmd,
			      u16 data, u8 bytes, bool wait_busy)
{
	u32 tcfg_reg;
	u16 data_to_send = data;
	u8 cycles;

	if (WARN_ON(bytes != 1 && bytes != 2))
		return -1;

	if (cmd == SPINOR_OP_WRSR &&
	    bytes == 1 &&
	    (sfc->configuration & SFC_CFG_WRSR_FORCE_16BITS)) {
		u8 cr;

		stsfc_read_status(sfc, SPINOR_OP_RDSR2, &cr);

		data_to_send = (data & 0xff) | ((u16)cr << 8);
		bytes = 2;
	}

	cycles = bytes * 8;

	/* Commands sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(SPINOR_OP_WREN);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(cmd);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	/* Data sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(cycles - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(data_to_send);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	/*
	 * No timeout error returned, depending of the device
	 * we have to wait that the command is ended
	 */
	if (wait_busy)
		stsfc_wait_busy(sfc, SFC_FLASH_MAX_STA_WRITE_MS);

	return 0;
}

static int stsfc_read(struct stsfc *sfc, u8 *buf, u32 size,
		      u32 offset)
{
	struct stsfc_seq *seq = &sfc->stsfc_seq_read;
	int ret;

	seq->dma_addr = sfc->dma_addr;

	/* Enter 32-bit address mode, if required */
	if (sfc->configuration & SFC_CFG_READ_TOGGLE_32BIT_ADDR)
		sfc->enter_32bit_addr(sfc, true);

	while (size > 0) {
		u32 data_to_read = min_t(size_t, SFC_DIS_SIZE(size),
					 sfc->dma_buf_size);

		/* Set data size to read */
		seq->data_size = roundup(data_to_read, SFC_MIN_DATA_TRANSFER);

		/* Set sector address to read in */
		seq->addr = offset;

		/* Start sequence */
		ret = stsfc_load_seq(sfc, seq);
		if (ret < 0)
			goto out;

		/* Wait for sequence completion */
		ret = stsfc_wait_seq(sfc, SFC_MAX_WAIT_SEQ_MS);
		if (ret < 0)
			goto out;

		/* Check seq_status to see if an error occurred */
		if (sfc->seq_status) {
			dev_err(sfc->dev, "Failed to read %d bytes from offset 0x%x error irqs (0x%x)\n",
				seq->data_size, seq->addr, sfc->seq_status);
			ret = -EIO;
			goto out;
		}

		memcpy(buf, sfc->dma_buf, data_to_read);

		buf += data_to_read;
		offset += data_to_read;
		size -= data_to_read;
	}

out:
	/* Exit 32-bit address mode, if required */
	if (sfc->configuration & SFC_CFG_READ_TOGGLE_32BIT_ADDR)
		sfc->enter_32bit_addr(sfc, false);

	return ret;
}

static int stsfc_write(struct stsfc *sfc, const u8 *buf, u32 size, u32 offset)
{
	struct stsfc_seq *seq = &sfc->stsfc_seq_write;
	u8 *dma_buf = sfc->dma_buf;
	u8 sta;
	u32 pages_to_write, prepadding, postpadding;
	int ret = 0;
	size_t data_to_write;

	seq->dma_addr = sfc->dma_addr;

	/* Enter 32-bit address mode, if required */
	if (sfc->configuration & SFC_CFG_WRITE_TOGGLE_32BIT_ADDR)
		sfc->enter_32bit_addr(sfc, true);

	while (size) {
		dma_buf = sfc->dma_buf;
		pages_to_write = 1;
		prepadding = 0;
		postpadding = 0;

		/*
		 * Data to write is the minimum of:
		 *  - Requested size
		 *  - Buffer size
		 *  - End of next page if offset not page aligned
		 *  - End of last page if offset but not size page aligned
		 *
		 * Additionally, a transfer size must be aligned to
		 * SFC_MIN_DATA_TRANFSER.  Pad before buffer if we would if
		 * offset is in the middle of a page, and after buffer
		 * regardless.
		 */
		data_to_write = min_t(size_t, sfc->dma_buf_size,
				      SFC_DIS_SIZE(size));

		if (offset % SFC_FLASH_PAGESIZE) {
			data_to_write = min_t(size_t, data_to_write,
					      SFC_FLASH_PAGESIZE -
						 (offset % SFC_FLASH_PAGESIZE));
			prepadding = offset % SFC_MIN_DATA_TRANSFER;
			memset(dma_buf, 0xff, prepadding);
			dma_buf += prepadding;
		} else if (data_to_write > SFC_FLASH_PAGESIZE) {
			data_to_write = min_t(size_t, data_to_write,
					      rounddown(data_to_write,
							SFC_FLASH_PAGESIZE));
			pages_to_write = data_to_write / SFC_FLASH_PAGESIZE;
		}

		memcpy(dma_buf, buf, data_to_write);

		/* Apply post padding if needed */
		if ((data_to_write + prepadding) % SFC_MIN_DATA_TRANSFER) {
			postpadding = SFC_MIN_DATA_TRANSFER -
				((data_to_write + prepadding) % SFC_MIN_DATA_TRANSFER);
			memset(dma_buf + data_to_write, 0xff, postpadding);
		}

		/*
		 * Set data size to write.  Note that either padding will be
		 * zero or pages_to_write will be 1.
		 */
		seq->data_size = (data_to_write + prepadding + postpadding) /
				 pages_to_write;

		/* Set sector address to write in */
		seq->addr = offset - prepadding;

		seq->seq_cfg &= ~SFC_SC_REPEAT_COUNT(U32_MAX);
		seq->seq_cfg |= SFC_SC_REPEAT_COUNT(pages_to_write - 1);

		/* Start sequence */
		ret = stsfc_load_seq(sfc, seq);
		if (ret < 0)
			goto out;

		/* Wait for sequence completion */
		ret = stsfc_wait_seq(sfc, SFC_MAX_WAIT_SEQ_MS);
		if (ret < 0)
			goto out;

		/* Check seq_status to see if an error occurred */
		if (sfc->seq_status) {
			dev_err(sfc->dev, "Failed to write %d bytes to offset 0x%x error irqs (0x%x)\n",
				seq->data_size, seq->addr, sfc->seq_status);
			ret = -EIO;
			goto out;
		}

		/* Wait for device idle state */
		ret = stsfc_wait_busy(sfc, SFC_FLASH_MAX_PAGE_WRITE_MS);

		/* N25Q: Check/Clear Error Flags */
		if (sfc->configuration & SFC_CFG_N25Q_CHECK_ERROR_FLAGS) {
			stsfc_read_status(sfc, N25Q_CMD_RFSR, &sta);
			if (sta & N25Q_FLAGS_ERROR) {
				stsfc_n25q_clear_flag(sfc);
				ret = -EPROTO;
			}
		}

		buf += data_to_write;
		size -= data_to_write;
		offset += data_to_write;
	}

out:
	/* Exit 32-bit address mode, if required */
	if (sfc->configuration & SFC_CFG_WRITE_TOGGLE_32BIT_ADDR)
		sfc->enter_32bit_addr(sfc, false);

	return ret;
}

static int stsfc_erase_sector(struct stsfc *sfc, u32 offset)
{
	struct stsfc_seq *seq = &sfc->stsfc_seq_erase_sector;
	u8 sta;
	int ret;

	/* Set sector address to erase */
	seq->addr = offset;

	/* Enter 32-bit address mode, if required */
	if (sfc->configuration & SFC_CFG_ERASE_TOGGLE_32BIT_ADDR)
		sfc->enter_32bit_addr(sfc, true);

	/* Start sequence */
	ret = stsfc_load_seq(sfc, seq);
	if (ret < 0)
		return ret;

	/* Wait for sequence completion */
	ret = stsfc_wait_seq(sfc, SFC_MAX_WAIT_SEQ_MS);
	if (ret < 0)
		return ret;

	/* Wait for device idle state */
	ret = stsfc_wait_busy(sfc, SFC_FLASH_MAX_SEC_ERASE_MS);

	/* Exit 32-bit address mode, if required */
	if (sfc->configuration & SFC_CFG_ERASE_TOGGLE_32BIT_ADDR)
		sfc->enter_32bit_addr(sfc, false);

	/* N25Q: Check/Clear Error Flags */
	if (sfc->configuration & SFC_CFG_N25Q_CHECK_ERROR_FLAGS) {
		stsfc_read_status(sfc, N25Q_CMD_RFSR, &sta);
		if (sta & N25Q_FLAGS_ERROR) {
			stsfc_n25q_clear_flag(sfc);
			ret = -EPROTO;
		}
	}

	return ret;
}

static int stsfc_erase_chip(struct stsfc *sfc)
{
	u32 tcfg_reg;

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(SPINOR_OP_WREN);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(SPINOR_OP_CHIP_ERASE);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	return stsfc_wait_busy(sfc, SFC_FLASH_MAX_CHIP_ERASE_MS);
}

static int stsfc_read_jedec(struct stsfc *sfc, u8 *jedec)
{
	int ret;
	struct stsfc_seq seq = {
		.data_size = SFC_DIS_SIZE(SFC_MAX_READID_LEN_ALIGNED),

		.opc[0] = (SFC_OPC_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
			   SFC_OPC_PADS_1 |
			   SFC_OPC_CS_ASSERT |
			   SFC_OPC_TRANSMIT_DATA(SPINOR_OP_RDID)),

		.seq[0] = STSFC_INST_WR_OPC_0,
		.seq[1] = STSFC_INST_DATA_BDM,
		.seq[2] = STSFC_INST_STOP,

		.seq_cfg = (SFC_SC_START_SEQ |
			    SFC_SC_DMA_ON |
			    SFC_SC_PADS_1),
	};

	WARN_ON(sfc->dma_buf_size < SFC_MAX_READID_LEN_ALIGNED);

	/* Set physical address to write in */
	seq.dma_addr = sfc->dma_addr;

	/* Start sequence */
	ret = stsfc_load_seq(sfc, &seq);
	if (ret < 0)
		return ret;

	/* Wait for sequence completion */
	ret = stsfc_wait_seq(sfc, SFC_MAX_WAIT_SEQ_MS);
	if (ret < 0)
		return ret;

	memcpy(jedec, sfc->dma_buf, SFC_MAX_READID_LEN);

	return 0;
}

static u8 stsfc_read_lock(struct stsfc *sfc, u32 addr)
{
	u32 tcfg_reg;
	u32 cfg_reg;

	/* Command sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(sfc->op_rd_lock);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	/* Address sent */
	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(sfc->lock_cycles_addr - 1) |
		  SFC_CFG_CDM_PADS_1 |
		  SFC_CFG_CDM_CS_ASSERT;

	stsfc_load_config_cdm(sfc, cfg_reg);

	stsfc_load_data_cdm(sfc, addr);

	/* Data read */
	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		  SFC_CFG_CDM_PADS_1;

	stsfc_load_config_cdm(sfc, cfg_reg);

	return stsfc_read_data_cdm(sfc) & 0xff;
}

static int stsfc_write_lock(struct stsfc *sfc, u32 addr, u8 data)
{
	u32 tcfg_reg;
	u32 cfg_reg;

	/* Commands sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(SPINOR_OP_WREN);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(sfc->op_wr_lock);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	/* Address sent */
	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(sfc->lock_cycles_addr - 1) |
		  SFC_CFG_CDM_PADS_1 |
		  SFC_CFG_CDM_CS_ASSERT;

	stsfc_load_config_cdm(sfc, cfg_reg);

	stsfc_load_data_cdm(sfc, addr);

	/* Data sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(data);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	return stsfc_wait_busy(sfc, SFC_FLASH_MAX_STA_WRITE_MS);
}

static int stsfc_lock(struct stsfc *sfc, u32 addr, bool lock)
{
	u8 opcode = (lock ? sfc->op_lock : sfc->op_unlock);
	u32 tcfg_reg;
	u32 cfg_reg;

	/* Commands sent */
	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_TRANSMIT_DATA(SPINOR_OP_WREN);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	tcfg_reg = SFC_TCFG_CDM_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		   SFC_TCFG_CDM_PADS_1 |
		   SFC_TCFG_CDM_CS_ASSERT |
		   SFC_TCFG_CDM_TRANSMIT_DATA(opcode);

	stsfc_load_transmit_config_cdm(sfc, tcfg_reg);

	/* Address sent */
	cfg_reg = SFC_CFG_CDM_NO_OF_CYC(sfc->lock_cycles_addr - 1) |
		  SFC_CFG_CDM_PADS_1;

	stsfc_load_config_cdm(sfc, cfg_reg);

	stsfc_load_data_cdm(sfc, addr);

	return stsfc_wait_busy(sfc, SFC_FLASH_MAX_STA_WRITE_MS);
}

static int stsfc_blk_xxlock_oneblock(struct stsfc *sfc, loff_t offs, bool lock)
{
	u32 addr = ((sfc->lock_cycles_addr == SFC_32BIT_ADDR) ?
			SFC_4_BYTES_ADDR(offs) : SFC_3_BYTES_ADDR(offs));
	u8 msk, val, reg;

	msk = sfc->lock_mask;
	val = lock ? sfc->lock_val[SFC_BLOCK_LOCKED] :
		     sfc->lock_val[SFC_BLOCK_UNLOCKED];

	reg = stsfc_read_lock(sfc, addr);
	if ((reg & msk) != val) {
		if (sfc->configuration & SFC_CFG_RD_WR_LOCK_REG) {
			reg = (reg & ~msk) | (val & msk);
			stsfc_write_lock(sfc, addr, reg);
		} else {
			stsfc_lock(sfc, addr, lock);
		}

		/* Check that the lock/unlock command has been executed */
		reg = stsfc_read_lock(sfc, addr);
		if ((reg & msk) != val) {
			dev_err(sfc->dev, "Failed to %s sector at 0x%012llx\n",
				lock ? "lock" : "unlock", offs);
			return -EIO;
		}
	}

	return 0;
}

/* Individual block locking scheme: Lock/Unlock */
static int stsfc_blk_xxlock(struct stsfc *sfc, loff_t offs, u64 len,
			    bool lock)
{
	struct mtd_info *mtd = &sfc->mtd;
	u64 offs_end = offs + len;
	u64 p4k_bot_end = sfc->p4k_bot_end;
	u64 p4k_top_start = sfc->p4k_top_start;
	int ret;

	if (sfc->configuration & SFC_CFG_MX25_TOGGLE_QE_BIT)
		stsfc_mx25_enter_quad_mode(sfc, false);

	if (sfc->configuration & SFC_CFG_LOCK_TOGGLE_32BIT_ADDR)
		sfc->enter_32bit_addr(sfc, true);

	while (offs < offs_end) {
		ret = stsfc_blk_xxlock_oneblock(sfc, offs, lock);
		if (ret)
			break;

		/* Parameter sectors have 4KiB locking granularity */
		if (offs < p4k_bot_end || offs >= p4k_top_start)
			offs += SZ_4K;
		else
			offs += mtd->erasesize;
	}

	if (sfc->configuration & SFC_CFG_LOCK_TOGGLE_32BIT_ADDR)
		sfc->enter_32bit_addr(sfc, false);

	if (sfc->configuration & SFC_CFG_MX25_TOGGLE_QE_BIT)
		stsfc_mx25_enter_quad_mode(sfc, true);

	return ret;
}

/* Print the protected region for a particular value of BPx */
static void stsfc_dump_bpx_region(struct stsfc *sfc, u8 bpx)
{
	char *p;
	char bpx_str[SFC_BPX_MAX_BITS * 2 + 1];
	u8 mask;

	p = bpx_str;
	for (mask = BIT(sfc->bpx_n_bits - 1); mask; mask >>= 1) {
		*p++ = ' ';
		*p++ = (bpx & mask) ? '1' : '0';
	}
	*p = '\0';

	dev_info(sfc->dev, "\t%s: 0x%08llx -> 0x%08llx\n", bpx_str,
		 sfc->bpx_tb ? 0UL : sfc->bpx_bnds[bpx],
		 sfc->bpx_tb ? sfc->bpx_bnds[bpx] : sfc->mtd.size);
}

/* Extract BPx from the value of the status register */
static u8 stsfc_status_to_bpx(struct stsfc *sfc, u8 status)
{
	u8 bpx = 0;
	u8 mask = 1;
	u8 i;

	for (i = 0; i < sfc->bpx_n_bits; i++, mask <<= 1) {
		if (status & sfc->bpx_sr_masks[i])
			bpx |= mask;
	}

	return bpx;
}

static void stsfc_dump_bpx_state(struct stsfc *sfc)
{
	u8 sr, bpx;

	dev_info(sfc->dev, "BP[%d-0] Locked Region:\n", sfc->bpx_n_bits - 1);

	stsfc_read_status(sfc, SPINOR_OP_RDSR, &sr);
	bpx = stsfc_status_to_bpx(sfc, sr);

	stsfc_dump_bpx_region(sfc, bpx);
}

/*
 * BPx Block Protection scheme: Lock/Unlock
 *
 * Lock/Unlock requests involve moving the BPx boundary up or down:
 *
 *	Lock (TB = 1): Move boundary down
 *    Unlock (TB = 0)            <-------|
 *                   ____________________|____________________
 *          Device: |                    |                    |
 *                  |____________________|____________________|
 *                                       |
 *    Unlock (TB = 1)                    |------>
 *      Lock (TB = 0): Move boundary up
 *
 * Returns an error if the new boundary is not supported by the BPx scheme, or
 * if the request is not contiguous with the existing BPx state.
 */
static int stsfc_bpx_xxlock(struct stsfc *sfc, loff_t offs, u64 len,
			    bool lock)
{
	const char *const opstr = lock ? "lock" : "unlock";
	u64 old_boundary, new_boundary;
	u8 sr, bpx, i;

	/* Get current BPx state */
	stsfc_read_status(sfc, SPINOR_OP_RDSR, &sr);
	bpx = stsfc_status_to_bpx(sfc, sr);
	old_boundary = sfc->bpx_bnds[bpx];

	if ((lock && sfc->bpx_tb) || (!lock && !sfc->bpx_tb)) {
		/*
		 * Lock/TB=1 and Unlock/TB=0: Move lock boundary up
		 */

		/* Set new boundary according to request */
		new_boundary = offs + len;

		/* Boundary already higher than new request */
		if (new_boundary <= old_boundary)
			goto out1;

		/* Request is non-contiguous with current boundary */
		if (offs > old_boundary)
			goto err1;
	} else {
		/*
		 * Unlock/TB=1 and Lock/TB=0: Move lock boundary down
		 */

		/* Set new boundary according to request */
		new_boundary = offs;

		/* Boundary already lower than new request */
		if (new_boundary >= old_boundary)
			goto out1;

		/* Request is non-contiguous with current boundary */
		if (offs + len < old_boundary)
			goto err1;
	}

	/* Find a valid BPx boundary */
	for (i = 0; i < sfc->bpx_n_bnds; i++) {
		if (new_boundary == sfc->bpx_bnds[i]) {
			bpx = i;
			break;
		}
	}

	/* Request does not match a BPx boundary */
	if (i == sfc->bpx_n_bnds)
		goto err2;

	/* Update Status register with new BPx */
	for (i = 0; i < sfc->bpx_n_bits; i++) {
		if (bpx & BIT(i))
			sr |= sfc->bpx_sr_masks[i];
		else
			sr &= ~sfc->bpx_sr_masks[i];
	}

	return stsfc_write_status(sfc, SPINOR_OP_WRSR, sr, 1, true);

 err2:
	dev_err(sfc->dev, "%s 0x%08llx -> 0x%08llx: request not compatible with BPx boundaries\n",
		opstr, offs, offs + len);
	return -EOPNOTSUPP;

 err1:
	dev_err(sfc->dev, "%s 0x%08llx -> 0x%08llx: request not compatible BPx state\n",
		opstr, offs, offs + len);
	return -EOPNOTSUPP;

 out1:
	dev_info(sfc->dev, "%s 0x%08llx -> 0x%08llx: already within %sed region\n",
		 opstr, offs, offs + len, opstr);
	return 0;
}

/*
 * SFC Interrupts
 */
static irqreturn_t sfc_irq_handler(int irq, void *dev)
{
	struct stsfc *sfc = dev;
	u32 status;

	status = stsfc_read_interrupts_status(sfc);
	if (!(status & SFC_INT_PENDING))
		return IRQ_NONE;

	if (status & SFC_INT_ERRORS)
		sfc->seq_status |= (status & SFC_INT_ERRORS);

	stsfc_clear_interrupts(sfc, status);

	if (status & SFC_INT_SEQUENCE_COMPLETION)
		complete(&sfc->seq_completed);

	return IRQ_HANDLED;
}

/*
 *
 * SFC mtd entries
 *
 */

/*
 * Read an address range from the flash chip. The address range
 * may be any size provided it is within the physical boundaries.
 */
static int stsfc_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
			  size_t *retlen, u_char *buf)
{
	struct stsfc *sfc = dev_get_drvdata(mtd->dev.parent);
	int ret;

	*retlen = 0;

	mutex_lock(&sfc->lock);

	ret = pm_runtime_get_sync(sfc->dev);
	if (ret < 0) {
		mutex_unlock(&sfc->lock);
		return ret;
	}

	/* Load address configuration */
	stsfc_load_addr_cfg(sfc, sfc->seq_read_addr_cfg);

	ret = stsfc_read(sfc, buf, len, from);
	if (ret == 0) {
		*retlen = len;
	}

	pm_runtime_mark_last_busy(sfc->dev);
	pm_runtime_put_autosuspend(sfc->dev);
	mutex_unlock(&sfc->lock);

	return ret;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * SFC_FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int stsfc_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
			   size_t *retlen, const u_char *buf)
{
	struct stsfc *sfc = dev_get_drvdata(mtd->dev.parent);
	int ret = 0;

	*retlen = 0;

	mutex_lock(&sfc->lock);

	ret = pm_runtime_get_sync(sfc->dev);
	if (ret < 0) {
		mutex_unlock(&sfc->lock);
		return ret;
	}

	/* Load address configuration */
	stsfc_load_addr_cfg(sfc, sfc->seq_write_addr_cfg);

	ret = stsfc_write(sfc, buf, len, to);
	if (ret == 0) {
		*retlen = len;
	}

	pm_runtime_mark_last_busy(sfc->dev);
	pm_runtime_put_autosuspend(sfc->dev);
	mutex_unlock(&sfc->lock);

	return ret;
}

static int stsfc_mtd_panic_write(struct mtd_info *mtd, loff_t to, size_t len,
				 size_t *retlen, const u_char *buf)
{
	struct stsfc *sfc = dev_get_drvdata(mtd->dev.parent);
	int ret = 0;

	*retlen = 0;

	pm_runtime_get_sync(sfc->dev);
	/* Continue anyways */

	/* Load address configuration */
	stsfc_load_addr_cfg(sfc, sfc->seq_write_addr_cfg);

	ret = stsfc_write(sfc, buf, len, to);
	if (ret == 0) {
		*retlen = len;
	}

	pm_runtime_mark_last_busy(sfc->dev);
	pm_runtime_put_autosuspend(sfc->dev);

	return ret;
}

/*
 * Erase an address range on the flash chip. The address range may extend
 * one or more erase sectors.  Return an error if there is a problem erasing.
 */
static int stsfc_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct stsfc *sfc = dev_get_drvdata(mtd->dev.parent);
	u32 addr, len;
	int ret;

	if (instr->len & (mtd->erasesize - 1))
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	mutex_lock(&sfc->lock);

	ret = pm_runtime_get_sync(sfc->dev);
	if (ret < 0) {
		instr->fail_addr = MTD_FAIL_ADDR_UNKNOWN;
		mutex_unlock(&sfc->lock);
		return ret;
	}

	/* Whole-chip erase? */
	if (len == mtd->size) {
		ret = stsfc_erase_chip(sfc);
		if (ret < 0)
			goto out;
	} else {
		/* Load address configuration */
		stsfc_load_addr_cfg(sfc, sfc->seq_erase_addr_cfg);

		while (len) {
			if ((sfc->configuration & CFG_S25FS_4K_ERASE) &&
			    (sfc->p4k_sector_address == addr)) {
				u64 p4k_addr = addr;

				if (sfc->p4k_sector_address)
					p4k_addr += mtd->erasesize -
						   (S25FS_4K_SECTOR * SZ_4K);

				ret = stsfc_s25fs_erase_4k_sector(sfc,
								  p4k_addr);
				if (ret < 0)
					goto out;
			}

			ret = stsfc_erase_sector(sfc, addr);
			if (ret < 0)
				goto out;

			addr += mtd->erasesize;
			len -= mtd->erasesize;
		}
	}

	pm_runtime_mark_last_busy(sfc->dev);
	pm_runtime_put_autosuspend(sfc->dev);
	mutex_unlock(&sfc->lock);

	return 0;

out:
	pm_runtime_mark_last_busy(sfc->dev);
	pm_runtime_put_autosuspend(sfc->dev);
	mutex_unlock(&sfc->lock);
	instr->fail_addr = addr;

	return ret;
}

/*
 * Lock an address range on the flash chip. The address range may extend
 * one or more lock sectors. Return an error if there is a problem locking.
 */
static int stsfc_mtd_lock(struct mtd_info *mtd, loff_t ofs, u64 len)
{
	struct stsfc *sfc = dev_get_drvdata(mtd->dev.parent);
	u32 block_mask = mtd->erasesize - 1;
	u32 flags = sfc->info->flags;
	int ret;

	if (ofs & block_mask || len & block_mask)
		return -EOPNOTSUPP;

	mutex_lock(&sfc->lock);

	ret = pm_runtime_get_sync(sfc->dev);
	if (ret < 0) {
		mutex_unlock(&sfc->lock);
		return ret;
	}

	if (flags & SFC_FLAG_BLK_LOCKING) {
		ret = stsfc_blk_xxlock(sfc, ofs, len, true);
	} else {
		ret = stsfc_bpx_xxlock(sfc, ofs, len, true);

		stsfc_dump_bpx_state(sfc);
	}

	pm_runtime_mark_last_busy(sfc->dev);
	pm_runtime_put_autosuspend(sfc->dev);
	mutex_unlock(&sfc->lock);

	return ret;
}

/*
 * Unlock an address range on the flash chip. The address range may extend
 * one or more unlock sectors. Return an error if there is a problem unlocking.
 */
static int stsfc_mtd_unlock(struct mtd_info *mtd, loff_t ofs, u64 len)
{
	struct stsfc *sfc = dev_get_drvdata(mtd->dev.parent);
	u32 block_mask = mtd->erasesize - 1;
	u32 flags = sfc->info->flags;
	int ret;

	if (ofs & block_mask || len & block_mask)
		return -EOPNOTSUPP;

	mutex_lock(&sfc->lock);

	ret = pm_runtime_get_sync(sfc->dev);
	if (ret < 0) {
		mutex_unlock(&sfc->lock);
		return ret;
	}

	if (flags & SFC_FLAG_BLK_LOCKING) {
		ret = stsfc_blk_xxlock(sfc, ofs, len, false);
	} else {
		ret = stsfc_bpx_xxlock(sfc, ofs, len, false);

		stsfc_dump_bpx_state(sfc);
	}

	pm_runtime_mark_last_busy(sfc->dev);
	pm_runtime_put_autosuspend(sfc->dev);
	mutex_unlock(&sfc->lock);

	return ret;
}

/*
 *
 * SFC initialization
 *
 */
static struct flash_info *stsfc_jedec_probe(struct stsfc *sfc)
{
	struct flash_info *info;
	struct flash_info *match = NULL;
	u8 readid[SFC_MAX_READID_LEN];
	int match_len = 0;

	if (stsfc_read_jedec(sfc, readid) < 0)
		return NULL;

	dev_dbg(sfc->dev, "READID = %s\n", readid);

	/*
	 * The 'readid' may match multiple entries in the table.  To ensure we
	 * retrieve the most specific match, use the match with the longest len.
	 */
	for (info = flash_types; info->name; info++) {
		if (memcmp(info->readid, readid, info->readid_len) == 0 &&
		    info->readid_len > match_len) {
			match = info;
			match_len = info->readid_len;
		}
	}

	if (!match) {
		char readid_str[SFC_MAX_READID_LEN * 3 + 1];

		hex_dump_to_buffer(readid, SFC_MAX_READID_LEN, 16, 1,
				   readid_str, sizeof(readid_str), 0);
		dev_err(sfc->dev, "Unrecognized READID [%s]\n", readid_str);
	}

	return match;
}

static void stsfc_disable_boot_mode(struct stsfc *sfc)
{
	u32 sfc_mode_select;

	sfc_mode_select = readl_relaxed(sfc->base + SFC_MODE_SELECT);

	/* Disable boot mode */
	sfc_mode_select &= ~SFC_BOOT_ENABLE;

	/* Set mode */
	writel_relaxed(sfc_mode_select, sfc->base + SFC_MODE_SELECT);
}

static void stsfc_set_mode(struct stsfc *sfc)
{
	u32 sfc_mode_select;

	sfc_mode_select = readl_relaxed(sfc->base + SFC_MODE_SELECT);

	/* Only BDM-DMA mode is supported */
	sfc_mode_select &= ~SFC_CPU_MODE;
	sfc_mode_select |= SFC_DMA_MODE;

	/* Disable high speed */
	sfc_mode_select &= ~SFC_WR_HS_NOT_DS;
	sfc_mode_select &= ~SFC_RD_HS_NOT_DS;

	/* CS active */
	if (sfc->cs1_used)
		sfc_mode_select |= SFC_CS_SEL_1_NOT_0;
	else
		sfc_mode_select &= ~SFC_CS_SEL_1_NOT_0;

	/* Set mode */
	writel_relaxed(sfc_mode_select, sfc->base + SFC_MODE_SELECT);

	/* Configure the SFC_POLL_SCHEDULER_0 register in charge of
	 * controlling the TIMER and RETRY mechanism in use by the
	 * SFC controller during the WTR command.
	 * COUNTER and TIMER values are computed as below and are
	 * flash device dependent, hence it would be better to have
	 * those information part of a table for each probed device
	 *
	 * TIMER = (Flash Typical Program Timing / SFC Controller input
	 *	    clock / 2) * 1.05
	 * COUNTER = (Flash Maximum Program Timing / (Flash Typical Program
	 *	      timing * 1.05)) + 1
	 *
	 * Ex: in case of a 133Mhz input clock and a flash with the following
	 * characteristics: Typical = 120 us, Maximum = 1800 us
	 * TIMER = 8379 and COUNTER = 16
	 */
	writel_relaxed(SFC_SCHED_TIMER(8379) | SFC_SCHED_COUNTER(16),
		       sfc->base + SFC_POLL_SCHEDULER_0);

	/* Configure the READ_FLASH_STATUS_CONFIG */
	writel_relaxed(SFC_RFS_NO_OF_CYC(SFC_NB_OPCODE_CYCLES - 1) |
		       SFC_RFS_PADS_1 | SFC_RFS_INST(STSFC_INST_WR_OPC_4),
		       sfc->base + SFC_READ_FLASH_STATUS_CONFIG);
}

static void stsfc_set_dll_mode(struct stsfc *sfc)
{
	u32 clk_control = 0x00;

	if (sfc->dll_mode) {
		dev_info(sfc->dev, "DLL mode: TX delay (%d), RX delay (%d)\n",
			 (sfc->dll_tx_delay) & 0x0f,
			 (sfc->dll_rx_delay) & 0x0f);
		writel_relaxed((sfc->dll_tx_delay) & 0x0f,
			       sfc->base + SFC_TX_STEP_DELAY);
		writel_relaxed((sfc->dll_rx_delay) & 0x0f,
			       sfc->base + SFC_RX_STEP_DELAY);
		clk_control = SFC_START_DLL | SFC_TX_DLL_SEL | SFC_RX_DLL_SEL;
	}
	writel_relaxed(clk_control, sfc->base + SFC_CLOCK_CONTROL);
}

static void stsfc_set_freq(struct stsfc *sfc, u32 spi_freq)
{
	u32 sfc_freq;
	u32 sfc_clock_division;
	int clk_div;

	sfc_freq = clk_get_rate(sfc->sfc_clk);

	/* In case of the SPI can handle freq >= SFC Freq, then no division
	 * should be done
	 */
	if (spi_freq >= sfc_freq) {
		writel_relaxed(0x00, sfc->base + SFC_CLOCK_DIVISION);
		dev_dbg(sfc->dev, "sfc_clk = %uHZ, spi_freq = %uHZ, no div\n",
			sfc_freq, spi_freq);
		return;
	}

	sfc_clock_division = readl_relaxed(sfc->base + SFC_CLOCK_DIVISION);

	/* Set clk_div on */
	sfc_clock_division &= ~SFC_CLK_DIV_BYPASS;

	/* Calculate clk_div - values 2, 4, 6, 8, 10, 12 or 14 */
	sfc_clock_division &= ~SFC_CLK_DIV;
	clk_div = DIV_ROUND_UP(sfc_freq, spi_freq);
	clk_div = max(min(clk_div, CLK_DIV_MAX), CLK_DIV_MIN);
	clk_div = DIV_ROUND_UP(clk_div, 2);
	sfc_clock_division |= SFC_CLK_DIVISION(clk_div);

	dev_dbg(sfc->dev, "sfc_clk = %uHZ, spi_freq = %uHZ, clk_div = %u\n",
		sfc_freq, spi_freq, clk_div * 2);

	writel_relaxed(sfc_clock_division, sfc->base + SFC_CLOCK_DIVISION);
}

static int stsfc_clk_enable(struct stsfc *sfc)
{
	int ret;

	ret = clk_prepare_enable(sfc->emi_clk);
	if (ret) {
		dev_err(sfc->dev, "Failed to enable emi_clk clock\n");
		return ret;
	}

	ret = clk_prepare_enable(sfc->sfc_clk);
	if (ret) {
		dev_err(sfc->dev, "Failed to enable sfc_clk clock\n");
		clk_disable_unprepare(sfc->emi_clk);
	}

	return ret;
}

static void stsfc_clk_disable(struct stsfc *sfc)
{
	clk_disable_unprepare(sfc->emi_clk);
	clk_disable_unprepare(sfc->sfc_clk);
}

static void stsfc_soft_reset(struct stsfc *sfc)
{
	u32 sfc_ip_sw_reset;

	sfc_ip_sw_reset = readl_relaxed(sfc->base + SFC_IP_SW_RESET);

	/* Reset full IP */
	sfc_ip_sw_reset |= SFC_SW_RESET_IP;
	writel_relaxed(sfc_ip_sw_reset, sfc->base + SFC_IP_SW_RESET);
	udelay(1);
	sfc_ip_sw_reset &= ~SFC_SW_RESET_IP;
	writel_relaxed(sfc_ip_sw_reset, sfc->base + SFC_IP_SW_RESET);
}

static void stsfc_init(struct stsfc *sfc)
{
	u32 reg;

	/* Disable boot mode */
	stsfc_disable_boot_mode(sfc);

	/* Perform a soft reset of the SFC */
	stsfc_soft_reset(sfc);

	/* Disable the DLL in case it is enabled */
	writel_relaxed(0, sfc->base + SFC_CLOCK_CONTROL);

	/* Set clock to 'safe' frequency initially */
	stsfc_set_freq(sfc, SFC_FLASH_SAFE_FREQ);

	/* Switch to BDM DMA mode */
	stsfc_set_mode(sfc);

	/* Enable interruptions */
	reg = SFC_INT_SEQUENCE_COMPLETION | SFC_INT_ERRORS;
	if (!sfc->polling_mode)
		reg |= SFC_INT_ENABLE;
	stsfc_enable_interrupts(sfc, reg);
}

static int stsfc_init_ip_plug(struct stsfc *sfc)
{
	int ret;

	/*
	 * Get the IP Interface consumer cookie.
	 * If the IP Interface is not defined, we use default configuration
	 */
	sfc->ibi = devm_ibi_get(sfc->dev);
	if (IS_ERR(sfc->ibi)) {
		sfc->ibi = NULL;
		return 0;
	}

	/* Get the IP Interface states */
	sfc->ibi_rd_plug = ibi_lookup_state(sfc->ibi, "rd_ip_plug");
	if (!sfc->ibi_rd_plug) {
		dev_err(sfc->dev, "rd_ip_plug not found\n");
		return -EINVAL;
	}

	sfc->ibi_wr_plug = ibi_lookup_state(sfc->ibi, "wr_ip_plug");
	if (!sfc->ibi_wr_plug) {
		dev_err(sfc->dev, "wr_ip_plug not found\n");
		return -EINVAL;
	}

	/* Set RD/WR plug states */
	ret = ibi_set_state(sfc->ibi, sfc->ibi_rd_plug);
	if (ret < 0) {
		dev_err(sfc->dev, "rd_ip_plug state not set\n");
		return ret;
	}

	ret = ibi_set_state(sfc->ibi, sfc->ibi_wr_plug);
	if (ret < 0) {
		dev_err(sfc->dev, "wr_ip_plug state not set\n");
		return ret;
	}

	return 0;
}

static int stsfc_runtime_suspend(struct device *dev)
{
	struct stsfc *sfc = dev_get_drvdata(dev);

	stsfc_clk_disable(sfc);

	return 0;
}

static int stsfc_runtime_resume(struct device *dev)
{
	struct stsfc *sfc = dev_get_drvdata(dev);

	return stsfc_clk_enable(sfc);
}

static const struct of_device_id stsfc_match[] = {
	{ .compatible = "st,gllcff-sfc-revc", .data = &stsfc_gllcff_data},
	{},
};
MODULE_DEVICE_TABLE(of, stsfc_match);

static int stsfc_fetch_platform_configs(struct platform_device *pdev)
{
	struct stsfc *sfc = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	const struct boot_dev *boot_dev;
	const struct of_device_id *match;
	struct regmap *regmap;
	u32 boot_device;
	int i, ret;

	regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(regmap))
		goto boot_device_fail;

	sfc->reset_signal = of_property_read_bool(np, "st,reset-signal");

	sfc->reset_por = of_property_read_bool(np, "st,reset-por");

	sfc->cs1_used = of_property_read_bool(np, "st,cs1_used");

	sfc->polling_mode = of_property_read_bool(np, "st,polling_mode");

	sfc->check_modepins = of_property_read_bool(np, "st,check_modepins");

	of_property_read_u32(np, "max-freq", &sfc->max_freq);

	sfc->dll_mode = of_property_read_bool(np, "st,dll_mode");
	if (sfc->dll_mode) {
		ret = of_property_read_u8(np, "st,dll_tx_delay",
					  &sfc->dll_tx_delay);
		if (ret) {
			dev_err(&pdev->dev, "st,dll_mode setted but no st,dll_tx_delay found\n");
			goto boot_device_fail;
		}

		ret = of_property_read_u8(np, "st,dll_rx_delay",
					  &sfc->dll_rx_delay);
		if (ret) {
			dev_err(&pdev->dev, "st,dll_mode setted but no st,dll_rx_delay found\n");
			goto boot_device_fail;
		}
	}

	match = of_match_node(stsfc_match, np);
	if (!match)
		goto boot_device_fail;
	boot_dev = match->data;

	ret = regmap_read(regmap, boot_dev->reg, &boot_device);
	if (ret)
		goto boot_device_fail;

	boot_device &= boot_dev->mask;

	sfc->booted_from_spi = false;
	for (i = 0; i < SFC_MAX_MODE_PINS; i++) {
		if (boot_device == boot_dev->spi[i]) {
			sfc->booted_from_spi = true;
			break;
		}
	}

	return 0;

boot_device_fail:
	return -EINVAL;
}

static int stsfc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mtd_part_parser_data ppdata;
	struct flash_info *info;
	struct resource *res;
	struct stsfc *sfc;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	sfc = devm_kzalloc(&pdev->dev, sizeof(*sfc), GFP_KERNEL);
	if (!sfc)
		return -ENOMEM;

	sfc->dev = &pdev->dev;

	platform_set_drvdata(pdev, sfc);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sfc_mem");
	if (!res) {
		dev_err(&pdev->dev, "sfc_mem resource not found\n");
		return -ENODEV;
	}

	sfc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sfc->base)) {
		dev_err(&pdev->dev,
			"Failed to reserve memory region %p\n", res);
		return PTR_ERR(sfc->base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sfc_dma");
	if (!res) {
		sfc->dma = NULL;
		dev_info(&pdev->dev, "sfc_dma resource not found\n");
	} else {
		sfc->dma = devm_ioremap_resource(&pdev->dev, res);

		if (IS_ERR(sfc->dma)) {
			dev_err(&pdev->dev,
				"Failed to reserve memory region %p\n", res);
			return PTR_ERR(sfc->dma);
		}
	}

	ret = platform_get_irq_byname(pdev, "sfc_irq");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to find IRQ resource\n");
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, ret, sfc_irq_handler, 0,
			       dev_name(&pdev->dev), sfc);
	if (ret) {
		dev_err(&pdev->dev, "irq request failed\n");
		return ret;
	}

	sfc->emi_clk = devm_clk_get(&pdev->dev, "emi_clk");
	if (IS_ERR(sfc->emi_clk)) {
		dev_err(&pdev->dev, "Failed to get emi_clk clock\n");
		return PTR_ERR(sfc->emi_clk);
	}

	sfc->sfc_clk = devm_clk_get(&pdev->dev, "sfc_clk");
	if (IS_ERR(sfc->sfc_clk)) {
		dev_err(&pdev->dev, "Failed to get sfc_clk clock\n");
		return PTR_ERR(sfc->sfc_clk);
	}

	pm_runtime_set_active(&pdev->dev);
	ret = stsfc_runtime_resume(&pdev->dev);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(&pdev->dev, SFC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	mutex_init(&sfc->lock);
	init_completion(&sfc->seq_completed);

	ret = stsfc_fetch_platform_configs(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to fetch platform configuration\n");
		goto out;
	}

	/*
	 * In case of property "st,check_modepins" is enabled, the driver will
	 * be probed only if mode pins is set to boot from spi nor
	 */
	if (sfc->check_modepins && !sfc->booted_from_spi)
		goto out;

	stsfc_init(sfc);

	ret = stsfc_init_ip_plug(sfc);
	if (ret)
		goto out;

	sfc->dma_buf_raw = dma_alloc_coherent(sfc->dev, DMA_BUFFER_SIZE,
					      &sfc->dma_addr_raw, GFP_KERNEL);
	if (sfc->dma_buf_raw == NULL) {
		return -ENOMEM;
	}

	/* Theoretically this is possible.  Practically it should not happen. */
	if (sfc->dma_addr_raw % SFC_DMA_ALIGNMENT) {
		dev_warn(&pdev->dev, "Allocated unaligned DMA buffer\n");
	}
	sfc->dma_addr = roundup(sfc->dma_addr_raw, SFC_DMA_ALIGNMENT);
	sfc->dma_buf_size = DMA_BUFFER_SIZE -
					(sfc->dma_addr - sfc->dma_addr_raw);
	sfc->dma_buf = sfc->dma_buf_raw + (sfc->dma_addr - sfc->dma_addr_raw);

	/* Detect SPI FLASH device */
	info = stsfc_jedec_probe(sfc);
	if (!info) {
		ret = -ENODEV;
		goto out;
	}
	sfc->info = info;

	/* Use device size to determine address width */
	if (info->sector_size * info->n_sectors > SZ_16M)
		info->flags |= SFC_FLAG_32BIT_ADDR;

	/*
	 * Configure READ/WRITE/ERASE sequences according to platform and
	 * device flags.
	 */
	if (info->config) {
		ret = info->config(sfc);
		if (ret)
			goto out;
	} else {
		ret = stsfc_prepare_rwe_seqs_default(sfc);
		if (ret)
			goto out;
	}

	/* Set operating frequency, from table or overridden by platform data */
	if (sfc->max_freq) {
		stsfc_set_freq(sfc, sfc->max_freq);
	} else if (info->max_freq) {
		stsfc_set_freq(sfc, info->max_freq);
		sfc->max_freq = info->max_freq;
	}

	/* Configure the DLL mode */
	stsfc_set_dll_mode(sfc);

	sfc->mtd.name = info->name;
	sfc->mtd.dev.parent = &pdev->dev;
	sfc->mtd.type = MTD_NORFLASH;
	sfc->mtd.writesize = 1;
	sfc->mtd.writebufsize = sfc->mtd.writesize;
	sfc->mtd.flags = MTD_CAP_NORFLASH;
	sfc->mtd.size = info->sector_size * info->n_sectors;
	sfc->mtd.erasesize = info->sector_size;
	mtd_set_of_node(&sfc->mtd, pdev->dev.of_node);

	sfc->mtd._read = stsfc_mtd_read;
	sfc->mtd._write = stsfc_mtd_write;
	sfc->mtd._panic_write = stsfc_mtd_panic_write;
	sfc->mtd._erase = stsfc_mtd_erase;

	/* Block Locking Support */
	if (info->flags & SFC_FLAG_BLK_LOCKING) {
		/* - Individual Block Locking scheme */
		sfc->mtd._lock = stsfc_mtd_lock;
		sfc->mtd._unlock = stsfc_mtd_unlock;

		dev_info(&pdev->dev, "Individual block locking scheme enabled\n");
	} else if (info->flags & SFC_FLAG_BPX_LOCKING) {
		/* - BPx Block Protection Scheme */
		sfc->mtd._lock = stsfc_mtd_lock;
		sfc->mtd._unlock = stsfc_mtd_unlock;

		dev_info(&pdev->dev, "BPx block locking scheme enabled\n");

		stsfc_dump_bpx_state(sfc);
	}

	dev_info(&pdev->dev, "Found serial flash device: %s\n",	info->name);
	dev_info(&pdev->dev,
		 " size = %llx (%lldMiB) erasesize = 0x%08x (%uKiB)\n",
		 (long long)sfc->mtd.size, (long long)(sfc->mtd.size >> 20),
		 sfc->mtd.erasesize, (sfc->mtd.erasesize >> 10));

	ret = mtd_device_parse_register(&sfc->mtd, NULL, &ppdata, NULL, 0);
	if (!ret) {
		pm_runtime_put(&pdev->dev);
		return 0;
	}

out:
	pm_runtime_put(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		stsfc_runtime_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int stsfc_remove(struct platform_device *pdev)
{
	struct stsfc *sfc = platform_get_drvdata(pdev);

	if (!pm_runtime_enabled(&pdev->dev))
		stsfc_runtime_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	dma_free_coherent(sfc->dev, DMA_BUFFER_SIZE, sfc->dma_buf_raw,
			  sfc->dma_addr_raw);

	return mtd_device_unregister(&sfc->mtd);
}

#ifdef CONFIG_PM_SLEEP
static int stsfc_suspend(struct device *dev)
{
	if ((!pm_runtime_enabled(dev)) ||
	    pm_runtime_autosuspend_expiration(dev))
		stsfc_runtime_suspend(dev);

	return 0;
}

static int stsfc_resume(struct device *dev)
{
	struct stsfc *sfc = dev_get_drvdata(dev);
	struct flash_info *info = sfc->info;
	int ret;

	ret = stsfc_runtime_resume(dev);
	if (ret)
		return ret;

	stsfc_init(sfc);

	if (sfc->ibi) {
		ret = ibi_set_state(sfc->ibi, sfc->ibi_rd_plug);
		if (ret < 0)
			return ret;

		ret = ibi_set_state(sfc->ibi, sfc->ibi_wr_plug);
		if (ret < 0)
			return ret;
	}

	if (info->resume)
		info->resume(sfc);

	if (sfc->max_freq)
		stsfc_set_freq(sfc, sfc->max_freq);

	if (!pm_runtime_enabled(dev))
		stsfc_runtime_suspend(dev);

	return 0;
}
#endif

static const struct dev_pm_ops stsfc_pm_ops = {
	SET_RUNTIME_PM_OPS(stsfc_runtime_suspend,
			   stsfc_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(stsfc_suspend,
				stsfc_resume)
};

static struct platform_driver stsfc_driver = {
	.probe = stsfc_probe,
	.remove = stsfc_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-sfc-revc",
		.of_match_table = stsfc_match,
		.pm = &stsfc_pm_ops,
	},
};
module_platform_driver(stsfc_driver);

MODULE_AUTHOR("Christophe Kerello <christophe.kerello@st.com>");
MODULE_DESCRIPTION("ST SFC Rev C driver");
MODULE_LICENSE("GPL v2");
