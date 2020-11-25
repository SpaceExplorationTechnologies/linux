/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Generic/SFDP Flash Commands and Device Capabilities
 *
 * Copyright (C) 2013 Lee Jones <lee.jones@lianro.org>
 */

#ifndef _MTD_SERIAL_FLASH_CMDS_H
#define _MTD_SERIAL_FLASH_CMDS_H

/* Flash opcodes. */
#define SPINOR_OP_WREN		0x06	/* Write enable */
#define SPINOR_OP_RDSR		0x05	/* Read status register */
#define SPINOR_OP_WRSR		0x01	/* Write status register 1 byte */
#define SPINOR_OP_READ		0x03	/* Read data bytes (low frequency) */
#define SPINOR_OP_READ_FAST	0x0b	/* Read data bytes (high frequency) */
#define SPINOR_OP_READ_1_1_2	0x3b	/* Read data bytes (Dual SPI) */
#define SPINOR_OP_READ_1_1_4	0x6b	/* Read data bytes (Quad SPI) */
#define SPINOR_OP_BE_4K		0x20	/* Erase 4KiB block */
#define SPINOR_OP_BE_4K_PMC	0xd7	/* Erase 4KiB block on PMC chips */
#define SPINOR_OP_BE_32K	0x52	/* Erase 32KiB block */
#define SPINOR_OP_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define SPINOR_OP_SE		0xd8	/* Sector erase (usually 64KiB) */
#define SPINOR_OP_RDID		0x9f	/* Read JEDEC ID */
#define SPINOR_OP_RDFSR		0x70	/* Read flag status register */

/* 4-byte address opcodes - used on Spansion and some Macronix flashes. */
#define SPINOR_OP_READ4		0x13	/* Read data bytes (low frequency) */
#define SPINOR_OP_READ4_FAST	0x0c	/* Read data bytes (high frequency) */
#define SPINOR_OP_READ4_1_1_2	0x3c	/* Read data bytes (Dual SPI) */
#define SPINOR_OP_READ4_1_1_4	0x6c	/* Read data bytes (Quad SPI) */
#define SPINOR_OP_PP_4B		0x12	/* Page program (up to 256 bytes) */
#define SPINOR_OP_SE_4B		0xdc	/* Sector erase (usually 64KiB) */

/* Used for SST flashes only. */
#define SPINOR_OP_BP		0x02	/* Byte program */
#define SPINOR_OP_WRDI		0x04	/* Write disable */
#define SPINOR_OP_AAI_WP	0xad	/* Auto address increment */

/* Used for Macronix and Winbond flashes. */
#define SPINOR_OP_EN4B		0xb7	/* Enter 4-byte mode */
#define SPINOR_OP_EX4B		0xe9	/* Exit 4-byte mode */

/* Used for Spansion flashes only. */
#define SPINOR_OP_BRWR		0x17	/* Bank register write */

/* Generic Flash Commands/OPCODEs */
#define SPINOR_OP_WRVCR		0x81
#define SPINOR_OP_RDVCR		0x85

/* JEDEC Standard - Serial Flash Discoverable Parmeters (SFDP) Commands */
#define SPINOR_OP_WRITE		0x02	/* PAGE PROGRAM */
#define SPINOR_OP_WRITE_1_1_2	0xa2	/* DUAL INPUT PROGRAM */
#define SPINOR_OP_WRITE_1_2_2	0xd2	/* DUAL INPUT EXT PROGRAM */
#define SPINOR_OP_WRITE_1_1_4	0x32	/* QUAD INPUT PROGRAM */
#define SPINOR_OP_WRITE_1_4_4	0x38	/* QUAD INPUT EXT PROGRAM */

/* Configuration flags */
#define FLASH_FLAG_SINGLE	0x000000ff
#define FLASH_FLAG_READ_WRITE	0x00000001
#define FLASH_FLAG_READ_FAST	0x00000002
#define FLASH_FLAG_SE_4K	0x00000004
#define FLASH_FLAG_SE_32K	0x00000008
#define FLASH_FLAG_CE		0x00000010
#define FLASH_FLAG_32BIT_ADDR	0x00000020
#define FLASH_FLAG_RESET	0x00000040
#define FLASH_FLAG_DYB_LOCKING	0x00000080

#define FLASH_FLAG_DUAL		0x0000ff00
#define FLASH_FLAG_READ_1_1_2	0x00000100
#define FLASH_FLAG_READ_1_2_2	0x00000200
#define FLASH_FLAG_READ_2_2_2	0x00000400
#define FLASH_FLAG_WRITE_1_1_2	0x00001000
#define FLASH_FLAG_WRITE_1_2_2	0x00002000
#define FLASH_FLAG_WRITE_2_2_2	0x00004000

#define FLASH_FLAG_QUAD		0x00ff0000
#define FLASH_FLAG_READ_1_1_4	0x00010000
#define FLASH_FLAG_READ_1_4_4	0x00020000
#define FLASH_FLAG_READ_4_4_4	0x00040000
#define FLASH_FLAG_WRITE_1_1_4	0x00100000
#define FLASH_FLAG_WRITE_1_4_4	0x00200000
#define FLASH_FLAG_WRITE_4_4_4	0x00400000

#endif /* _MTD_SERIAL_FLASH_CMDS_H */
