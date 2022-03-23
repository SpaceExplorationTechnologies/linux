/*
 * Marvell 88e6xxx Ethernet switch single-chip support
 *
 * Copyright (c) 2019 Space Exploration Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_bridge.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/netdevice.h>
#include <linux/gpio/consumer.h>
#include <linux/phy.h>

#define SMI_CMD         0x00
#define SMI_CMD_BUSY        BIT(15)
#define SMI_CMD_CLAUSE_22   BIT(12)
#define SMI_CMD_OP_22_WRITE ((1 << 10) | SMI_CMD_BUSY | SMI_CMD_CLAUSE_22)
#define SMI_CMD_OP_22_READ  ((2 << 10) | SMI_CMD_BUSY | SMI_CMD_CLAUSE_22)
#define SMI_CMD_OP_45_WRITE_ADDR    ((0 << 10) | SMI_CMD_BUSY)
#define SMI_CMD_OP_45_WRITE_DATA    ((1 << 10) | SMI_CMD_BUSY)
#define SMI_CMD_OP_45_READ_DATA     ((2 << 10) | SMI_CMD_BUSY)
#define SMI_CMD_OP_45_READ_DATA_INC ((3 << 10) | SMI_CMD_BUSY)
#define SMI_DATA        0x01

/* PHY Registers */
#define PHY_PAGE        0x16
#define PHY_PAGE_COPPER     0x00
#define PHY_PAGE_MAC        0x02
#define PHY_PAGE_PRBS       0x04
#define PHY_PAGE_VCT        0x05
#define PHY_PAGE_PKTGEN     0x06
#define PHY_PAGE_CABLE      0x07

#define ADDR_SERDES     0x0f
#define SERDES_PAGE_FIBER   0x01

#define PORT_STATUS     0x00
#define PORT_STATUS_PAUSE_EN    BIT(15)
#define PORT_STATUS_MY_PAUSE    BIT(14)
#define PORT_STATUS_HD_FLOW BIT(13)
#define PORT_STATUS_PHY_DETECT  BIT(12)
#define PORT_STATUS_LINK    BIT(11)
#define PORT_STATUS_DUPLEX  BIT(10)
#define PORT_STATUS_SPEED_MASK  0x0300
#define PORT_STATUS_SPEED_10    0x0000
#define PORT_STATUS_SPEED_100   0x0100
#define PORT_STATUS_SPEED_1000  0x0200
#define PORT_STATUS_EEE     BIT(6) /* 6352 */
#define PORT_STATUS_AM_DIS  BIT(6) /* 6165 */
#define PORT_STATUS_MGMII   BIT(6) /* 6185 */
#define PORT_STATUS_TX_PAUSED   BIT(5)
#define PORT_STATUS_FLOW_CTRL   BIT(4)
#define PORT_STATUS_CMODE_MASK  0x0f
#define PORT_STATUS_CMODE_RGMII		0x7
#define PORT_STATUS_CMODE_100BASE_X 0x8
#define PORT_STATUS_CMODE_1000BASE_X    0x9
#define PORT_STATUS_CMODE_SGMII     0xa
#define PORT_PCS_CTRL       0x01
#define PORT_PCS_CTRL_RGMII_DELAY_RXCLK BIT(15)
#define PORT_PCS_CTRL_RGMII_DELAY_TXCLK BIT(14)
#define PORT_PCS_CTRL_FC        BIT(7)
#define PORT_PCS_CTRL_FORCE_FC      BIT(6)
#define PORT_PCS_CTRL_LINK_UP       BIT(5)
#define PORT_PCS_CTRL_FORCE_LINK    BIT(4)
#define PORT_PCS_CTRL_DUPLEX_FULL   BIT(3)
#define PORT_PCS_CTRL_FORCE_DUPLEX  BIT(2)
#define PORT_PCS_CTRL_10        0x00
#define PORT_PCS_CTRL_100       0x01
#define PORT_PCS_CTRL_1000      0x02
#define PORT_PCS_CTRL_UNFORCED      0x03
#define PORT_PAUSE_CTRL     0x02
#define PORT_SWITCH_ID      0x03
#define PORT_SWITCH_ID_PROD_NUM_6085    0x04a
#define PORT_SWITCH_ID_PROD_NUM_6095    0x095
#define PORT_SWITCH_ID_PROD_NUM_6131    0x106
#define PORT_SWITCH_ID_PROD_NUM_6320    0x115
#define PORT_SWITCH_ID_PROD_NUM_6123    0x121
#define PORT_SWITCH_ID_PROD_NUM_6161    0x161
#define PORT_SWITCH_ID_PROD_NUM_6165    0x165
#define PORT_SWITCH_ID_PROD_NUM_6171    0x171
#define PORT_SWITCH_ID_PROD_NUM_6172    0x172
#define PORT_SWITCH_ID_PROD_NUM_6175    0x175
#define PORT_SWITCH_ID_PROD_NUM_6176    0x176
#define PORT_SWITCH_ID_PROD_NUM_6185    0x1a7
#define PORT_SWITCH_ID_PROD_NUM_6240    0x240
#define PORT_SWITCH_ID_PROD_NUM_6321    0x310
#define PORT_SWITCH_ID_PROD_NUM_6352    0x352
#define PORT_SWITCH_ID_PROD_NUM_6350    0x371
#define PORT_SWITCH_ID_PROD_NUM_6351    0x375
#define PORT_CONTROL        0x04
#define PORT_CONTROL_USE_CORE_TAG   BIT(15)
#define PORT_CONTROL_DROP_ON_LOCK   BIT(14)
#define PORT_CONTROL_EGRESS_UNMODIFIED  (0x0 << 12)
#define PORT_CONTROL_EGRESS_UNTAGGED    (0x1 << 12)
#define PORT_CONTROL_EGRESS_TAGGED  (0x2 << 12)
#define PORT_CONTROL_EGRESS_ADD_TAG (0x3 << 12)
#define PORT_CONTROL_HEADER     BIT(11)
#define PORT_CONTROL_IGMP_MLD_SNOOP BIT(10)
#define PORT_CONTROL_DOUBLE_TAG     BIT(9)
#define PORT_CONTROL_FRAME_MODE_NORMAL      (0x0 << 8)
#define PORT_CONTROL_FRAME_MODE_DSA     (0x1 << 8)
#define PORT_CONTROL_FRAME_MODE_PROVIDER    (0x2 << 8)
#define PORT_CONTROL_FRAME_ETHER_TYPE_DSA   (0x3 << 8)
#define PORT_CONTROL_DSA_TAG        BIT(8)
#define PORT_CONTROL_VLAN_TUNNEL    BIT(7)
#define PORT_CONTROL_TAG_IF_BOTH    BIT(6)
#define PORT_CONTROL_USE_IP     BIT(5)
#define PORT_CONTROL_USE_TAG        BIT(4)
#define PORT_CONTROL_FORWARD_UNKNOWN_MC BIT(3)
#define PORT_CONTROL_FORWARD_UNKNOWN    BIT(2)
#define PORT_CONTROL_STATE_MASK     0x03
#define PORT_CONTROL_STATE_DISABLED 0x00
#define PORT_CONTROL_STATE_BLOCKING 0x01
#define PORT_CONTROL_STATE_LEARNING 0x02
#define PORT_CONTROL_STATE_FORWARDING   0x03
#define PORT_CONTROL_1      0x05
#define PORT_CONTROL_1_FID_11_4_MASK    (0xff << 0)
#define PORT_BASE_VLAN      0x06
#define PORT_BASE_VLAN_FID_3_0_MASK (0xf << 12)
#define PORT_DEFAULT_VLAN   0x07
#define PORT_DEFAULT_VLAN_MASK  0xfff
#define PORT_CONTROL_2      0x08
#define PORT_CONTROL_2_IGNORE_FCS   BIT(15)
#define PORT_CONTROL_2_VTU_PRI_OVERRIDE BIT(14)
#define PORT_CONTROL_2_SA_PRIO_OVERRIDE BIT(13)
#define PORT_CONTROL_2_DA_PRIO_OVERRIDE BIT(12)
#define PORT_CONTROL_2_JUMBO_1522   (0x00 << 12)
#define PORT_CONTROL_2_JUMBO_2048   (0x01 << 12)
#define PORT_CONTROL_2_JUMBO_10240  (0x02 << 12)
#define PORT_CONTROL_2_8021Q_MASK   (0x03 << 10)
#define PORT_CONTROL_2_8021Q_DISABLED   (0x00 << 10)
#define PORT_CONTROL_2_8021Q_FALLBACK   (0x01 << 10)
#define PORT_CONTROL_2_8021Q_CHECK  (0x02 << 10)
#define PORT_CONTROL_2_8021Q_SECURE (0x03 << 10)
#define PORT_CONTROL_2_DISCARD_TAGGED   BIT(9)
#define PORT_CONTROL_2_DISCARD_UNTAGGED BIT(8)
#define PORT_CONTROL_2_MAP_DA       BIT(7)
#define PORT_CONTROL_2_DEFAULT_FORWARD  BIT(6)
#define PORT_CONTROL_2_FORWARD_UNKNOWN  BIT(6)
#define PORT_CONTROL_2_EGRESS_MONITOR   BIT(5)
#define PORT_CONTROL_2_INGRESS_MONITOR  BIT(4)
#define PORT_RATE_CONTROL   0x09
#define PORT_RATE_CONTROL_2 0x0a
#define PORT_ASSOC_VECTOR   0x0b
#define PORT_ASSOC_VECTOR_HOLD_AT_1     BIT(15)
#define PORT_ASSOC_VECTOR_INT_AGE_OUT       BIT(14)
#define PORT_ASSOC_VECTOR_LOCKED_PORT       BIT(13)
#define PORT_ASSOC_VECTOR_IGNORE_WRONG      BIT(12)
#define PORT_ASSOC_VECTOR_REFRESH_LOCKED    BIT(11)
#define PORT_ATU_CONTROL    0x0c
#define PORT_PRI_OVERRIDE   0x0d
#define PORT_ETH_TYPE       0x0f
#define PORT_IN_DISCARD_LO  0x10
#define PORT_IN_DISCARD_HI  0x11
#define PORT_IN_FILTERED    0x12
#define PORT_OUT_FILTERED   0x13
#define PORT_TAG_REGMAP_0123    0x18
#define PORT_TAG_REGMAP_4567    0x19

#define GLOBAL_STATUS       0x00
#define GLOBAL_STATUS_PPU_STATE BIT(15) /* 6351 and 6171 */
/* Two bits for 6165, 6185 etc */
#define GLOBAL_STATUS_PPU_MASK      (0x3 << 14)
#define GLOBAL_STATUS_PPU_DISABLED_RST  (0x0 << 14)
#define GLOBAL_STATUS_PPU_INITIALIZING  (0x1 << 14)
#define GLOBAL_STATUS_PPU_DISABLED  (0x2 << 14)
#define GLOBAL_STATUS_PPU_POLLING   (0x3 << 14)
#define GLOBAL_MAC_01       0x01
#define GLOBAL_MAC_23       0x02
#define GLOBAL_MAC_45       0x03
#define GLOBAL_ATU_FID      0x01
#define GLOBAL_VTU_FID      0x02
#define GLOBAL_VTU_FID_MASK 0xfff
#define GLOBAL_VTU_SID      0x03    /* 6097 6165 6351 6352 */
#define GLOBAL_VTU_SID_MASK 0x3f
#define GLOBAL_CONTROL      0x04
#define GLOBAL_CONTROL_SW_RESET     BIT(15)
#define GLOBAL_CONTROL_PPU_ENABLE   BIT(14)
#define GLOBAL_CONTROL_DISCARD_EXCESS   BIT(13) /* 6352 */
#define GLOBAL_CONTROL_SCHED_PRIO   BIT(11) /* 6152 */
#define GLOBAL_CONTROL_MAX_FRAME_1632   BIT(10) /* 6152 */
#define GLOBAL_CONTROL_RELOAD_EEPROM    BIT(9)  /* 6152 */
#define GLOBAL_CONTROL_DEVICE_EN    BIT(7)
#define GLOBAL_CONTROL_STATS_DONE_EN    BIT(6)
#define GLOBAL_CONTROL_VTU_PROBLEM_EN   BIT(5)
#define GLOBAL_CONTROL_VTU_DONE_EN  BIT(4)
#define GLOBAL_CONTROL_ATU_PROBLEM_EN   BIT(3)
#define GLOBAL_CONTROL_ATU_DONE_EN  BIT(2)
#define GLOBAL_CONTROL_TCAM_EN      BIT(1)
#define GLOBAL_CONTROL_EEPROM_DONE_EN   BIT(0)
#define GLOBAL_VTU_OP       0x05
#define GLOBAL_VTU_OP_BUSY  BIT(15)
#define GLOBAL_VTU_OP_FLUSH_ALL     ((0x01 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_OP_VTU_LOAD_PURGE    ((0x03 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_OP_VTU_GET_NEXT  ((0x04 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_OP_STU_LOAD_PURGE    ((0x05 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_OP_STU_GET_NEXT  ((0x06 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_VID      0x06
#define GLOBAL_VTU_VID_MASK 0xfff
#define GLOBAL_VTU_VID_VALID    BIT(12)
#define GLOBAL_VTU_DATA_0_3 0x07
#define GLOBAL_VTU_DATA_4_7 0x08
#define GLOBAL_VTU_DATA_8_11    0x09
#define GLOBAL_VTU_STU_DATA_MASK        0x03
#define GLOBAL_VTU_DATA_MEMBER_TAG_UNMODIFIED   0x00
#define GLOBAL_VTU_DATA_MEMBER_TAG_UNTAGGED 0x01
#define GLOBAL_VTU_DATA_MEMBER_TAG_TAGGED   0x02
#define GLOBAL_VTU_DATA_MEMBER_TAG_NON_MEMBER   0x03
#define GLOBAL_STU_DATA_PORT_STATE_DISABLED 0x00
#define GLOBAL_STU_DATA_PORT_STATE_BLOCKING 0x01
#define GLOBAL_STU_DATA_PORT_STATE_LEARNING 0x02
#define GLOBAL_STU_DATA_PORT_STATE_FORWARDING   0x03
#define GLOBAL_ATU_CONTROL  0x0a
#define GLOBAL_ATU_CONTROL_LEARN2ALL    BIT(3)
#define GLOBAL_ATU_OP       0x0b
#define GLOBAL_ATU_OP_BUSY  BIT(15)
#define GLOBAL_ATU_OP_NOP       (0 << 12)
#define GLOBAL_ATU_OP_FLUSH_MOVE_ALL        ((1 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_FLUSH_MOVE_NON_STATIC ((2 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_LOAD_DB       ((3 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_GET_NEXT_DB   ((4 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_FLUSH_MOVE_ALL_DB     ((5 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_FLUSH_MOVE_NON_STATIC_DB ((6 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_GET_CLR_VIOLATION   ((7 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_DATA     0x0c
#define GLOBAL_ATU_DATA_TRUNK           BIT(15)
#define GLOBAL_ATU_DATA_TRUNK_ID_MASK       0x00f0
#define GLOBAL_ATU_DATA_TRUNK_ID_SHIFT      4
#define GLOBAL_ATU_DATA_PORT_VECTOR_MASK    0x3ff0
#define GLOBAL_ATU_DATA_PORT_VECTOR_SHIFT   4
#define GLOBAL_ATU_DATA_STATE_MASK      0x0f
#define GLOBAL_ATU_DATA_STATE_UNUSED        0x00
#define GLOBAL_ATU_DATA_STATE_UC_MGMT       0x0d
#define GLOBAL_ATU_DATA_STATE_UC_STATIC     0x0e
#define GLOBAL_ATU_DATA_STATE_UC_PRIO_OVER  0x0f
#define GLOBAL_ATU_DATA_STATE_MC_NONE_RATE  0x05
#define GLOBAL_ATU_DATA_STATE_MC_STATIC     0x07
#define GLOBAL_ATU_DATA_STATE_MC_MGMT       0x0e
#define GLOBAL_ATU_DATA_STATE_MC_PRIO_OVER  0x0f
#define GLOBAL_ATU_MAC_01   0x0d
#define GLOBAL_ATU_MAC_23   0x0e
#define GLOBAL_ATU_MAC_45   0x0f
#define GLOBAL_IP_PRI_0     0x10
#define GLOBAL_IP_PRI_1     0x11
#define GLOBAL_IP_PRI_2     0x12
#define GLOBAL_IP_PRI_3     0x13
#define GLOBAL_IP_PRI_4     0x14
#define GLOBAL_IP_PRI_5     0x15
#define GLOBAL_IP_PRI_6     0x16
#define GLOBAL_IP_PRI_7     0x17
#define GLOBAL_IEEE_PRI     0x18
#define GLOBAL_CORE_TAG_TYPE    0x19
#define GLOBAL_MONITOR_CONTROL  0x1a
#define GLOBAL_MONITOR_CONTROL_INGRESS_SHIFT    12
#define GLOBAL_MONITOR_CONTROL_EGRESS_SHIFT 8
#define GLOBAL_MONITOR_CONTROL_ARP_SHIFT    4
#define GLOBAL_MONITOR_CONTROL_MIRROR_SHIFT 0
#define GLOBAL_MONITOR_CONTROL_ARP_DISABLED (0xf0)
#define GLOBAL_CONTROL_2    0x1c
#define GLOBAL_CONTROL_2_NO_CASCADE     0xe000
#define GLOBAL_CONTROL_2_MULTIPLE_CASCADE   0xf000

#define GLOBAL_STATS_OP     0x1d
#define GLOBAL_STATS_OP_BUSY    BIT(15)
#define GLOBAL_STATS_OP_NOP     (0 << 12)
#define GLOBAL_STATS_OP_FLUSH_ALL   ((1 << 12) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_FLUSH_PORT  ((2 << 12) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_READ_CAPTURED   ((4 << 12) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_CAPTURE_PORT    ((5 << 12) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_HIST_RX     ((1 << 10) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_HIST_TX     ((2 << 10) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_HIST_RX_TX  ((3 << 10) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_BANK_1  BIT(9)
#define GLOBAL_STATS_COUNTER_32 0x1e
#define GLOBAL_STATS_COUNTER_01 0x1f

#define ADDR_GLOBAL2	0x1c

#define GLOBAL2_INT_SOURCE  0x00
#define GLOBAL2_INT_MASK    0x01
#define GLOBAL2_MGMT_EN_2X  0x02
#define GLOBAL2_MGMT_EN_0X  0x03
#define GLOBAL2_FLOW_CONTROL    0x04
#define GLOBAL2_SWITCH_MGMT 0x05
#define GLOBAL2_SWITCH_MGMT_USE_DOUBLE_TAG_DATA BIT(15)
#define GLOBAL2_SWITCH_MGMT_PREVENT_LOOPS   BIT(14)
#define GLOBAL2_SWITCH_MGMT_FLOW_CONTROL_MSG    BIT(13)
#define GLOBAL2_SWITCH_MGMT_FORCE_FLOW_CTRL_PRI BIT(7)
#define GLOBAL2_SWITCH_MGMT_RSVD2CPU        BIT(3)
#define GLOBAL2_DEVICE_MAPPING  0x06
#define GLOBAL2_DEVICE_MAPPING_UPDATE       BIT(15)
#define GLOBAL2_DEVICE_MAPPING_TARGET_SHIFT 8
#define GLOBAL2_DEVICE_MAPPING_PORT_MASK    0x0f
#define GLOBAL2_TRUNK_MASK  0x07
#define GLOBAL2_TRUNK_MASK_UPDATE       BIT(15)
#define GLOBAL2_TRUNK_MASK_NUM_SHIFT        12
#define GLOBAL2_TRUNK_MASK_HASK         BIT(11)
#define GLOBAL2_TRUNK_MAPPING   0x08
#define GLOBAL2_TRUNK_MAPPING_UPDATE        BIT(15)
#define GLOBAL2_TRUNK_MAPPING_ID_SHIFT      11
#define GLOBAL2_IRL_CMD     0x09
#define GLOBAL2_IRL_CMD_BUSY    BIT(15)
#define GLOBAL2_IRL_CMD_OP_INIT_ALL ((0x001 << 12) | GLOBAL2_IRL_CMD_BUSY)
#define GLOBAL2_IRL_CMD_OP_INIT_SEL ((0x010 << 12) | GLOBAL2_IRL_CMD_BUSY)
#define GLOBAL2_IRL_CMD_OP_WRITE_SEL    ((0x011 << 12) | GLOBAL2_IRL_CMD_BUSY)
#define GLOBAL2_IRL_CMD_OP_READ_SEL ((0x100 << 12) | GLOBAL2_IRL_CMD_BUSY)
#define GLOBAL2_IRL_DATA    0x0a
#define GLOBAL2_PVT_ADDR    0x0b
#define GLOBAL2_PVT_ADDR_BUSY   BIT(15)
#define GLOBAL2_PVT_ADDR_OP_INIT_ONES   ((0x01 << 12) | GLOBAL2_PVT_ADDR_BUSY)
#define GLOBAL2_PVT_ADDR_OP_WRITE_PVLAN ((0x03 << 12) | GLOBAL2_PVT_ADDR_BUSY)
#define GLOBAL2_PVT_ADDR_OP_READ    ((0x04 << 12) | GLOBAL2_PVT_ADDR_BUSY)
#define GLOBAL2_PVT_DATA    0x0c
#define GLOBAL2_SWITCH_MAC  0x0d
#define GLOBAL2_ATU_STATS   0x0e
#define GLOBAL2_PRIO_OVERRIDE   0x0f
#define GLOBAL2_PRIO_OVERRIDE_FORCE_SNOOP   BIT(7)
#define GLOBAL2_PRIO_OVERRIDE_SNOOP_SHIFT   4
#define GLOBAL2_PRIO_OVERRIDE_FORCE_ARP     BIT(3)
#define GLOBAL2_PRIO_OVERRIDE_ARP_SHIFT     0
#define GLOBAL2_EEPROM_CMD      0x14
#define GLOBAL2_EEPROM_CMD_BUSY     BIT(15)
#define GLOBAL2_EEPROM_CMD_OP_MASK  (0x7 << 12)
#define GLOBAL2_EEPROM_CMD_OP_WRITE (0x3 << 12)
#define GLOBAL2_EEPROM_CMD_OP_READ  (0x4 << 12)
#define GLOBAL2_EEPROM_CMD_OP_LOAD  (0x6 << 12)
#define GLOBAL2_EEPROM_CMD_RUNNING  BIT(11)
#define GLOBAL2_EEPROM_CMD_WRITE_EN BIT(10)
#define GLOBAL2_EEPROM_CMD_ADDR_MASK    0xff
#define GLOBAL2_EEPROM_DATA 0x15
#define GLOBAL2_PTP_AVB_OP  0x16
#define GLOBAL2_PTP_AVB_DATA    0x17
#define GLOBAL2_SMI_PHY_CMD         0x18
#define GLOBAL2_SMI_PHY_CMD_BUSY        BIT(15)
#define GLOBAL2_SMI_PHY_CMD_MODE_22     BIT(12)
#define GLOBAL2_SMI_PHY_CMD_OP_22_WRITE_DATA    ((0x1 << 10) | \
						 GLOBAL2_SMI_PHY_CMD_MODE_22 | \
						 GLOBAL2_SMI_PHY_CMD_BUSY)
#define GLOBAL2_SMI_PHY_CMD_OP_22_READ_DATA ((0x2 << 10) | \
						 GLOBAL2_SMI_PHY_CMD_MODE_22 | \
						 GLOBAL2_SMI_PHY_CMD_BUSY)
#define GLOBAL2_SMI_PHY_DATA            0x19
#define GLOBAL2_SCRATCH_MISC    0x1a
#define GLOBAL2_SCRATCH_BUSY        BIT(15)
#define GLOBAL2_SCRATCH_REGISTER_SHIFT  8
#define GLOBAL2_SCRATCH_VALUE_MASK  0xff
#define GLOBAL2_WDOG_CONTROL    0x1b
#define GLOBAL2_QOS_WEIGHT  0x1c
#define GLOBAL2_MISC        0x1d

/* Defines for EEPROM contents.
 * See "EEPROM Programming Format" in the datasheet for more.
 */
#define EEPROM_CMD_HALT_EEPROM_PROCESSING 0xFFFF
#define EEPROM_CMD_RELEASE_INTERNAL_RESET 0x7D01
#define EEPROM_CMD_NIBBLE_SHIFT 12
#define EEPROM_CMD_NIBBLE_WRITE_PHY 0x0
#define EEPROM_CMD_NIBBLE_WRITE_GLOBAL 0x7
#define EEPROM_CMD_NIBBLE_WRITE_MAC 0x8
#define EEPROM_CMD_REG_SIZE 5
#define EEPROM_CMD_MAC_VECTOR_SIZE 7
#define EEPROM_CMD_PHY_VECTOR_SIZE 5
#define EEPROM_CMD_WRITE_GLOBAL_G2_BIT BIT(6)

struct mv88e6xxx_chip;

struct mv88e6xxx_ops {
	int (*get_eeprom)(struct mv88e6xxx_chip *chip,
			  struct ethtool_eeprom *eeprom, u8 *data);
	int (*set_eeprom)(struct mv88e6xxx_chip *chip,
			  struct ethtool_eeprom *eeprom, u8 *data);
	int (*set_switch_mac)(struct mv88e6xxx_chip *chip, u8 *addr);

	int (*phy_read)(struct mv88e6xxx_chip *chip, int addr, int reg,
			u16 *val);
	int (*phy_write)(struct mv88e6xxx_chip *chip, int addr, int reg,
			 u16 val);
};

static int mv88e6xxx_g2_get_eeprom16(struct mv88e6xxx_chip *chip,
				     struct ethtool_eeprom *eeprom, u8 *data);
static int mv88e6xxx_g2_set_eeprom16(struct mv88e6xxx_chip *chip,
				     struct ethtool_eeprom *eeprom, u8 *data);
static int mv88e6xxx_g2_set_switch_mac(struct mv88e6xxx_chip *chip, u8 *addr);
static int mv88e6xxx_g2_smi_phy_read(struct mv88e6xxx_chip *chip, int addr,
				     int reg, u16 *val);
static int mv88e6xxx_g2_smi_phy_write(struct mv88e6xxx_chip *chip, int addr,
				      int reg, u16 val);

static const struct mv88e6xxx_ops mv88e6240_ops = {
	.get_eeprom = mv88e6xxx_g2_get_eeprom16,
	.set_eeprom = mv88e6xxx_g2_set_eeprom16,
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

struct mv88e6xxx_info {
	u16 prod_num;
	const char *name;
	unsigned int num_ports;
	unsigned int port_base_addr;
	unsigned int global1_addr;
	unsigned long long flags;
	const struct mv88e6xxx_ops *ops;
};

#define PORT_SWITCH_ID_PROD_NUM_6240    0x240
static const struct mv88e6xxx_info mv88e6240_info = {
	.prod_num = PORT_SWITCH_ID_PROD_NUM_6240,
	.name = "Marvell 88E6240",
	.num_ports = 7,
	.port_base_addr = 0x10,
	.global1_addr = 0x1b,
	.ops = &mv88e6240_ops,
};

struct mv88e6xxx_bus_ops {
	int (*read)(struct mv88e6xxx_chip *chip, int addr, int reg, u16 *val);
	int (*write)(struct mv88e6xxx_chip *chip, int addr, int reg, u16 val);
};

struct mv88e6xxx_chip {
	const struct mv88e6xxx_info *info;

	/* The device this structure is associated to */
	struct device *dev;

	/* This mutex protects the access to the switch registers */
	struct mutex reg_lock;

	/* The MII bus and the address on the bus that is used to
	 * communication with the switch
	 */
	const struct mv88e6xxx_bus_ops *smi_ops;
	struct mii_bus *bus;
	int sw_addr;

	int port_count;
	int serdes_powered;

	struct device_node *np;

	/* Optional reset GPIO */
	struct gpio_desc *reset;

	struct {
		u32 len;
		struct bin_attribute attr;
		const __be16 *dt_words;
		int dt_len;
		union {
			__le16 words[0];
			char bytes[0];
		} *cache;
		bool mismatch;
		bool write_failed;
	} eeprom;
};

static void assert_reg_lock(struct mv88e6xxx_chip *chip)
{
	if (unlikely(!mutex_is_locked(&chip->reg_lock))) {
		dev_err(chip->dev, "Switch registers lock not held!\n");
		dump_stack();
	}
}

/* The switch ADDR[4:1] configuration pins define the chip SMI device address
 * (ADDR[0] is always zero, thus only even SMI addresses can be strapped).
 *
 * When ADDR is all zero, the chip uses Single-chip Addressing Mode, assuming it
 * is the only device connected to the SMI master. In this mode it responds to
 * all 32 possible SMI addresses, and thus maps directly the internal devices.
 *
 * When ADDR is non-zero, the chip uses Multi-chip Addressing Mode, allowing
 * multiple devices to share the SMI interface. In this mode it responds to only
 * 2 registers, used to indirectly access the internal SMI devices.
 */

static int mv88e6xxx_smi_read(struct mv88e6xxx_chip *chip,
			      int addr, int reg, u16 *val)
{
	if (!chip->smi_ops)
		return -EOPNOTSUPP;

	return chip->smi_ops->read(chip, addr, reg, val);
}

static int mv88e6xxx_smi_write(struct mv88e6xxx_chip *chip,
			       int addr, int reg, u16 val)
{
	if (!chip->smi_ops)
		return -EOPNOTSUPP;

	return chip->smi_ops->write(chip, addr, reg, val);
}

static int mv88e6xxx_smi_single_chip_read(struct mv88e6xxx_chip *chip,
					  int addr, int reg, u16 *val)
{
	int ret;

	ret = mdiobus_read_nested(chip->bus, addr, reg);
	if (ret < 0)
		return ret;

	*val = ret & 0xffff;

	return 0;
}

static int mv88e6xxx_smi_single_chip_write(struct mv88e6xxx_chip *chip,
					   int addr, int reg, u16 val)
{
	int ret;

	ret = mdiobus_write_nested(chip->bus, addr, reg, val);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct mv88e6xxx_bus_ops mv88e6xxx_smi_single_chip_ops = {
	.read = mv88e6xxx_smi_single_chip_read,
	.write = mv88e6xxx_smi_single_chip_write,
};

static int mv88e6xxx_smi_multi_chip_wait(struct mv88e6xxx_chip *chip)
{
	int ret;
	int i;

	for (i = 0; i < 16; i++) {
		ret = mdiobus_read_nested(chip->bus, chip->sw_addr, SMI_CMD);
		if (ret < 0)
			return ret;

		if ((ret & SMI_CMD_BUSY) == 0)
			return 0;

		usleep_range(10, 20);
	}

	return -ETIMEDOUT;
}

static int mv88e6xxx_smi_multi_chip_read(struct mv88e6xxx_chip *chip,
					 int addr, int reg, u16 *val)
{
	int ret;

	/* Wait for the bus to become free. */
	ret = mv88e6xxx_smi_multi_chip_wait(chip);
	if (ret < 0)
		return ret;

	/* Transmit the read command. */
	ret = mdiobus_write_nested(chip->bus, chip->sw_addr, SMI_CMD,
				   SMI_CMD_OP_22_READ | (addr << 5) | reg);
	if (ret < 0)
		return ret;

	/* Wait for the read command to complete. */
	ret = mv88e6xxx_smi_multi_chip_wait(chip);
	if (ret < 0)
		return ret;

	/* Read the data. */
	ret = mdiobus_read_nested(chip->bus, chip->sw_addr, SMI_DATA);
	if (ret < 0)
		return ret;

	*val = ret & 0xffff;

	return 0;
}

static int mv88e6xxx_smi_multi_chip_write(struct mv88e6xxx_chip *chip,
					  int addr, int reg, u16 val)
{
	int ret;

	/* Wait for the bus to become free. */
	ret = mv88e6xxx_smi_multi_chip_wait(chip);
	if (ret < 0)
		return ret;

	/* Transmit the data to write. */
	ret = mdiobus_write_nested(chip->bus, chip->sw_addr, SMI_DATA, val);
	if (ret < 0)
		return ret;

	/* Transmit the write command. */
	ret = mdiobus_write_nested(chip->bus, chip->sw_addr, SMI_CMD,
				   SMI_CMD_OP_22_WRITE | (addr << 5) | reg);
	if (ret < 0)
		return ret;

	/* Wait for the write command to complete. */
	ret = mv88e6xxx_smi_multi_chip_wait(chip);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct mv88e6xxx_bus_ops mv88e6xxx_smi_multi_chip_ops = {
	.read = mv88e6xxx_smi_multi_chip_read,
	.write = mv88e6xxx_smi_multi_chip_write,
};

static int mv88e6xxx_read(struct mv88e6xxx_chip *chip, int addr, int reg,
			  u16 *val)
{
	int err;

	assert_reg_lock(chip);

	err = mv88e6xxx_smi_read(chip, addr, reg, val);
	if (err)
		return err;

	dev_dbg(chip->dev, "<- addr: 0x%.2x reg: 0x%.2x val: 0x%.4x\n",
		addr, reg, *val);

	return 0;
}

static int mv88e6xxx_write(struct mv88e6xxx_chip *chip, int addr, int reg,
			   u16 val)
{
	int err;

	assert_reg_lock(chip);

	err = mv88e6xxx_smi_write(chip, addr, reg, val);
	if (err)
		return err;

	dev_dbg(chip->dev, "-> addr: 0x%.2x reg: 0x%.2x val: 0x%.4x\n",
		addr, reg, val);

	return 0;
}

static int mv88e6xxx_port_read(struct mv88e6xxx_chip *chip, int port, int reg,
			       u16 *val)
{
	int addr = chip->info->port_base_addr + port;

	return mv88e6xxx_read(chip, addr, reg, val);
}

static int mv88e6xxx_port_write(struct mv88e6xxx_chip *chip, int port, int reg,
				u16 val)
{
	int addr = chip->info->port_base_addr + port;

	return mv88e6xxx_write(chip, addr, reg, val);
}

static int mv88e6xxx_phy_read(struct mv88e6xxx_chip *chip, int phy,
			      int reg, u16 *val)
{
	int addr = phy; /* PHY devices addresses start at 0x0 */

	if (!chip->info->ops->phy_read)
		return -EOPNOTSUPP;

	return chip->info->ops->phy_read(chip, addr, reg, val);
}

static int mv88e6xxx_phy_write(struct mv88e6xxx_chip *chip, int phy,
			       int reg, u16 val)
{
	int addr = phy; /* PHY devices addresses start at 0x0 */

	if (!chip->info->ops->phy_write)
		return -EOPNOTSUPP;

	return chip->info->ops->phy_write(chip, addr, reg, val);
}

static int mv88e6xxx_phy_set_page(struct mv88e6xxx_chip *chip, int phy, u8 page)
{
	return mv88e6xxx_phy_write(chip, phy, PHY_PAGE, page);
}

static void mv88e6xxx_phy_set_page_copper(struct mv88e6xxx_chip *chip, int phy)
{
	int err;

	/* Restore PHY page Copper 0x0 for access via the registered MDIO bus */
	err = mv88e6xxx_phy_set_page(chip, phy, PHY_PAGE_COPPER);
	if (unlikely(err)) {
		dev_err(chip->dev, "failed to restore PHY %d page Copper (%d)\n",
			phy, err);
	}
}

static int mv88e6xxx_phy_page_read(struct mv88e6xxx_chip *chip, int phy,
				   u8 page, int reg, u16 *val)
{
	int err;

	/* There is no paging for registers 22 */
	if (reg == PHY_PAGE)
		return -EINVAL;

	err = mv88e6xxx_phy_set_page(chip, phy, page);
	if (!err) {
		err = mv88e6xxx_phy_read(chip, phy, reg, val);
		mv88e6xxx_phy_set_page_copper(chip, phy);
	}

	return err;
}

static int mv88e6xxx_phy_page_write(struct mv88e6xxx_chip *chip, int phy,
				    u8 page, int reg, u16 val)
{
	int err;

	/* There is no paging for registers 22 */
	if (reg == PHY_PAGE)
		return -EINVAL;

	err = mv88e6xxx_phy_set_page(chip, phy, page);
	if (!err) {
		err = mv88e6xxx_phy_write(chip, phy, reg, val);
		mv88e6xxx_phy_set_page_copper(chip, phy);
	}

	return err;
}

static int mv88e6xxx_g1_write(struct mv88e6xxx_chip *chip, int reg, u16 val)
{
	int addr = chip->info->global1_addr;

	return mv88e6xxx_write(chip, addr, reg, val);
}

static int mv88e6xxx_g1_read(struct mv88e6xxx_chip *chip, int reg, u16 *val)
{
	int addr = chip->info->global1_addr;

	return mv88e6xxx_read(chip, addr, reg, val);
}

static int mv88e6xxx_serdes_read(struct mv88e6xxx_chip *chip, int reg, u16 *val)
{
	return mv88e6xxx_phy_page_read(chip, ADDR_SERDES, SERDES_PAGE_FIBER,
					   reg, val);
}

static int mv88e6xxx_serdes_write(struct mv88e6xxx_chip *chip, int reg, u16 val)
{
	return mv88e6xxx_phy_page_write(chip, ADDR_SERDES, SERDES_PAGE_FIBER,
					reg, val);
}

static int mv88e6xxx_g2_read(struct mv88e6xxx_chip *chip, int reg, u16 *val)
{
	return mv88e6xxx_read(chip, ADDR_GLOBAL2, reg, val);
}

static int mv88e6xxx_g2_write(struct mv88e6xxx_chip *chip, int reg, u16 val)
{
	return mv88e6xxx_write(chip, ADDR_GLOBAL2, reg, val);
}

static int mv88e6xxx_wait_mask(struct mv88e6xxx_chip *chip, int addr, int reg,
			       u16 mask, u16 val)
{
	int i;
	u16 data;

	for (i = 0; i < 500; i++) {
		int err;

		err = mv88e6xxx_read(chip, addr, reg, &data);
		if (err)
			return err;

		if ((data & mask) == val)
			return 0;

		usleep_range(1000, 2000);
	}

	dev_err(chip->dev, "Timeout waiting for addr 0x%02x reg 0x%02x\n",
		addr, reg);
	dev_err(chip->dev, "data 0x%04x & mask 0x%04x != val 0x%04x\n",
		data, mask, val);
	return -ETIMEDOUT;
}

static int mv88e6xxx_wait(struct mv88e6xxx_chip *chip, int addr, int reg,
			  u16 mask)
{
	return mv88e6xxx_wait_mask(chip, addr, reg, mask, 0);
}

static int mv88e6xxx_update(struct mv88e6xxx_chip *chip, int addr, int reg,
			    u16 update)
{
	u16 val;
	int err;

	/* Wait until the previous operation is completed */
	err = mv88e6xxx_wait(chip, addr, reg, BIT(15));
	if (err)
		return err;

	/* Set the Update bit to trigger a write operation */
	val = BIT(15) | update;

	return mv88e6xxx_write(chip, addr, reg, val);
}

static int mv88e6xxx_g2_update(struct mv88e6xxx_chip *chip, int reg, u16 update)
{
	return mv88e6xxx_update(chip, ADDR_GLOBAL2, reg, update);
}

static int mv88e6xxx_g2_wait_mask(struct mv88e6xxx_chip *chip, int reg,
				  u16 mask, u16 val)
{
	return mv88e6xxx_wait_mask(chip, ADDR_GLOBAL2, reg, mask, val);
}

static int mv88e6xxx_g2_wait(struct mv88e6xxx_chip *chip, int reg, u16 mask)
{
	return mv88e6xxx_wait(chip, ADDR_GLOBAL2, reg, mask);
}

/* Offset 0x0D: Switch MAC/WoL/WoF register */

static int mv88e6xxx_g2_switch_mac_write(struct mv88e6xxx_chip *chip,
					 unsigned int pointer, u8 data)
{
	u16 val = (pointer << 8) | data;

	return mv88e6xxx_g2_update(chip, GLOBAL2_SWITCH_MAC, val);
}

static int mv88e6xxx_g2_set_switch_mac(struct mv88e6xxx_chip *chip, u8 *addr)
{
	int i, err;

	for (i = 0; i < 6; i++) {
		err = mv88e6xxx_g2_switch_mac_write(chip, i, addr[i]);
		if (err)
			break;
	}

	return err;
}

/* Offset 0x14: EEPROM Command
 * Offset 0x15: EEPROM Data
 */

static int mv88e6xxx_g2_eeprom_wait_mask(struct mv88e6xxx_chip *chip, u16 cmd)
{
	return mv88e6xxx_g2_wait_mask(chip, GLOBAL2_EEPROM_CMD,
				      GLOBAL2_EEPROM_CMD_OP_MASK |
				      GLOBAL2_EEPROM_CMD_BUSY |
				      GLOBAL2_EEPROM_CMD_RUNNING |
				      GLOBAL2_EEPROM_CMD_ADDR_MASK,
				      cmd);
}

static int mv88e6xxx_g2_eeprom_wait(struct mv88e6xxx_chip *chip)
{
	return mv88e6xxx_g2_wait(chip, GLOBAL2_EEPROM_CMD,
				 GLOBAL2_EEPROM_CMD_BUSY |
				 GLOBAL2_EEPROM_CMD_RUNNING);
}

static int mv88e6xxx_g2_eeprom_cmd(struct mv88e6xxx_chip *chip, u16 cmd)
{
	int err;

	err = mv88e6xxx_g2_write(chip, GLOBAL2_EEPROM_CMD,
				 GLOBAL2_EEPROM_CMD_BUSY | cmd);
	if (err)
		return err;

	return mv88e6xxx_g2_eeprom_wait_mask(chip, cmd);
}

static int mv88e6xxx_g2_eeprom_read16(struct mv88e6xxx_chip *chip,
				      u8 addr, u16 *data)
{
	u16 cmd = GLOBAL2_EEPROM_CMD_OP_READ | addr;
	int err;

	err = mv88e6xxx_g2_eeprom_wait(chip);
	if (err)
		return err;

	err = mv88e6xxx_g2_eeprom_cmd(chip, cmd);
	if (err)
		return err;

	return mv88e6xxx_g2_read(chip, GLOBAL2_EEPROM_DATA, data);
}

static int mv88e6xxx_g2_eeprom_write16(struct mv88e6xxx_chip *chip,
				       u8 addr, u16 data)
{
	u16 cmd = GLOBAL2_EEPROM_CMD_OP_WRITE | addr;
	int err;

	err = mv88e6xxx_g2_eeprom_wait(chip);
	if (err)
		return err;

	err = mv88e6xxx_g2_write(chip, GLOBAL2_EEPROM_DATA, data);
	if (err)
		return err;

	return mv88e6xxx_g2_eeprom_cmd(chip, cmd);
}

static int mv88e6xxx_g2_get_eeprom16(struct mv88e6xxx_chip *chip,
				     struct ethtool_eeprom *eeprom, u8 *data)
{
	unsigned int offset = eeprom->offset;
	unsigned int len = eeprom->len;
	u16 val;
	int err;

	eeprom->len = 0;

	if (offset & 1) {
		err = mv88e6xxx_g2_eeprom_read16(chip, offset >> 1, &val);
		if (err)
			return err;

		*data++ = (val >> 8) & 0xff;

		offset++;
		len--;
		eeprom->len++;
	}

	while (len >= 2) {
		err = mv88e6xxx_g2_eeprom_read16(chip, offset >> 1, &val);
		if (err)
			return err;

		*data++ = val & 0xff;
		*data++ = (val >> 8) & 0xff;

		offset += 2;
		len -= 2;
		eeprom->len += 2;
	}

	if (len) {
		err = mv88e6xxx_g2_eeprom_read16(chip, offset >> 1, &val);
		if (err)
			return err;

		*data++ = val & 0xff;

		offset++;
		len--;
		eeprom->len++;
	}

	return 0;
}

static int mv88e6xxx_g2_set_eeprom16(struct mv88e6xxx_chip *chip,
				     struct ethtool_eeprom *eeprom, u8 *data)
{
	unsigned int offset = eeprom->offset;
	unsigned int len = eeprom->len;
	u16 val;
	int err;

	/* Ensure the RO WriteEn bit is set */
	err = mv88e6xxx_g2_read(chip, GLOBAL2_EEPROM_CMD, &val);
	if (err)
		return err;

	if (!(val & GLOBAL2_EEPROM_CMD_WRITE_EN))
		return -EROFS;

	eeprom->len = 0;

	if (offset & 1) {
		err = mv88e6xxx_g2_eeprom_read16(chip, offset >> 1, &val);
		if (err)
			return err;

		val = (*data++ << 8) | (val & 0xff);

		err = mv88e6xxx_g2_eeprom_write16(chip, offset >> 1, val);
		if (err)
			return err;

		offset++;
		len--;
		eeprom->len++;
	}

	while (len >= 2) {
		val = *data++;
		val |= *data++ << 8;

		err = mv88e6xxx_g2_eeprom_write16(chip, offset >> 1, val);
		if (err)
			return err;

		offset += 2;
		len -= 2;
		eeprom->len += 2;
	}

	if (len) {
		err = mv88e6xxx_g2_eeprom_read16(chip, offset >> 1, &val);
		if (err)
			return err;

		val = (val & 0xff00) | *data++;

		err = mv88e6xxx_g2_eeprom_write16(chip, offset >> 1, val);
		if (err)
			return err;

		offset++;
		len--;
		eeprom->len++;
	}

	return 0;
}

static ssize_t mv88e6xxx_eeprom_read_locked(struct mv88e6xxx_chip *chip,
					    char *buf,
					    loff_t offset,
					    size_t len)
{
	struct ethtool_eeprom pass_through = {
		.offset = offset,
		.len = len,
	};
	ssize_t rc;

	rc = chip->info->ops->get_eeprom(chip, &pass_through, buf);
	if (rc)
		return rc;

	return pass_through.len;
}

static ssize_t mv88e6xxx_eeprom_read(struct file *unused_1,
				     struct kobject *unused_2,
				     struct bin_attribute *bin_attr,
				     char *buf,
				     loff_t offset,
				     size_t len)
{
	struct mv88e6xxx_chip *chip = bin_attr->private;
	ssize_t rc;

	mutex_lock(&chip->reg_lock);
	rc = mv88e6xxx_eeprom_read_locked(chip, buf, offset, len);
	mutex_unlock(&chip->reg_lock);

	return rc;
}

static ssize_t mv88e6xxx_eeprom_write_locked(
	struct mv88e6xxx_chip *chip,
	char *buf,
	loff_t offset,
	size_t len)
{
	struct ethtool_eeprom pass_through;
	ssize_t rc;

	memset(&pass_through, 0, sizeof(pass_through));
	pass_through.offset = offset;
	pass_through.len = len;

	rc = chip->info->ops->set_eeprom(chip, &pass_through, buf);
	if (rc)
		return rc;

	return pass_through.len;
}

static ssize_t mv88e6xxx_eeprom_write(
	struct file *dontcare,
	struct kobject *notgonnausethiseither,
	struct bin_attribute *bin_attr,
	char *buf,
	loff_t offset,
	size_t len)
{
	struct mv88e6xxx_chip *chip = bin_attr->private;
	ssize_t rc;

	mutex_lock(&chip->reg_lock);
	rc = mv88e6xxx_eeprom_write_locked(chip, buf, offset, len);
	mutex_unlock(&chip->reg_lock);

	return rc;
}

/* Offset 0x18: SMI PHY Command Register
 * Offset 0x19: SMI PHY Data Register
 */
static int mv88e6xxx_g2_smi_phy_wait(struct mv88e6xxx_chip *chip)
{
	return mv88e6xxx_g2_wait(chip, GLOBAL2_SMI_PHY_CMD,
				 GLOBAL2_SMI_PHY_CMD_BUSY);
}

static int mv88e6xxx_g2_smi_phy_cmd(struct mv88e6xxx_chip *chip, u16 cmd)
{
	int err;

	err = mv88e6xxx_g2_write(chip, GLOBAL2_SMI_PHY_CMD, cmd);
	if (err)
		return err;

	return mv88e6xxx_g2_smi_phy_wait(chip);
}

static int mv88e6xxx_g2_smi_phy_read(struct mv88e6xxx_chip *chip, int addr,
				     int reg, u16 *val)
{
	u16 cmd = GLOBAL2_SMI_PHY_CMD_OP_22_READ_DATA | (addr << 5) | reg;
	int err;

	err = mv88e6xxx_g2_smi_phy_wait(chip);
	if (err)
		return err;

	err = mv88e6xxx_g2_smi_phy_cmd(chip, cmd);
	if (err)
		return err;

	return mv88e6xxx_g2_read(chip, GLOBAL2_SMI_PHY_DATA, val);
}

static int mv88e6xxx_g2_smi_phy_write(struct mv88e6xxx_chip *chip, int addr,
				      int reg, u16 val)
{
	u16 cmd = GLOBAL2_SMI_PHY_CMD_OP_22_WRITE_DATA | (addr << 5) | reg;
	int err;

	err = mv88e6xxx_g2_smi_phy_wait(chip);
	if (err)
		return err;

	err = mv88e6xxx_g2_write(chip, GLOBAL2_SMI_PHY_DATA, val);
	if (err)
		return err;

	return mv88e6xxx_g2_smi_phy_cmd(chip, cmd);
}

static int mv88e6xxx_serdes_power_on(struct mv88e6xxx_chip *chip)
{
	u16 val;
	int err;

	err = mv88e6xxx_serdes_read(chip, MII_BMCR, &val);
	if (err)
		return err;

	/* Clear Power Down bit */
	if (val & BMCR_PDOWN) {
		val &= ~BMCR_PDOWN;
		err = mv88e6xxx_serdes_write(chip, MII_BMCR,
					     val);
		return err;
	}

	chip->serdes_powered = 1;

	return 0;
}

static int mv88e6xxx_verify_active_port(struct mv88e6xxx_chip *chip, int port,
					struct device_node *node)
{
	int err;
	u16 cmode;

	phy_interface_t phy_mode = PHY_INTERFACE_MODE_NA;

	of_get_phy_mode(node, &phy_mode);

	if (of_phy_is_fixed_link(node))
		of_phy_register_fixed_link(node);

	err = mv88e6xxx_port_read(chip, port, PORT_STATUS, &cmode);
	if (err)
		return err;

	cmode &= PORT_STATUS_CMODE_MASK;

	switch (phy_mode) {
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_ID:
		if (cmode != PORT_STATUS_CMODE_RGMII) {
			dev_info(chip->dev, "Port %d CMODE mismatch.  CMODE: %d OF: %s\n",
				 port, cmode, phy_modes(phy_mode));
		}
		break;
	case PHY_INTERFACE_MODE_SGMII:
		/* 1000BASE_X is supported in 4.12 but not 4.9. From a driver
		 * perspective it is the same anyways.
		 */
		if ((cmode != PORT_STATUS_CMODE_SGMII) &&
		    (cmode != PORT_STATUS_CMODE_1000BASE_X) &&
		    (cmode != PORT_STATUS_CMODE_100BASE_X)) {
			dev_info(chip->dev, "Port %d CMODE mismatch.  CMODE: %d OF: %s\n",
				 port, cmode, phy_modes(phy_mode));
		}

		if (!chip->serdes_powered) {
			err = mv88e6xxx_serdes_power_on(chip);
			if (err)
				dev_warn(chip->dev, "Failed to power on serdes\n");
		}
	default:
		break;
	}
	return 0;
}

static int mv88e6xxx_verify_ports(struct mv88e6xxx_chip *chip)
{
	int ret;
	u32 port;
	struct device_node *port_node, *child;

	port_node = of_find_node_by_name(chip->np, "ports");
	if (!port_node) {
		dev_err(chip->dev, "Failed to find ports node\n");
		return -ENODEV;
	}

	for_each_available_child_of_node(port_node, child) {
		ret = of_property_read_u32(child, "reg", &port);
		if (ret < 0 || port > chip->info->num_ports) {
			dev_err(chip->dev, "%s has invalid Port number\n",
				child->full_name);
			continue;
		}

		ret = mv88e6xxx_verify_active_port(chip, port, child);
		if (ret) {
			dev_err(chip->dev, "Failed to verify port %d\n", port);
			/* Continue */
		}
	}

	of_node_put(port_node);

	return 0;
}

static int mv88e6xxx_set_all_port_state(struct mv88e6xxx_chip *chip, u32 state)
{
	int err = 0;
	u16 reg;
	int i;

	if (state & ~PORT_CONTROL_STATE_MASK) {
		dev_err(chip->dev, "Invalid port state 0x%x requested\n",
			state);
	}

	mutex_lock(&chip->reg_lock);
	for (i = 0; i < chip->port_count; i++) {
		err = mv88e6xxx_port_read(chip, i, PORT_CONTROL, &reg);
		if (err)
			break;

		reg = (reg & ~PORT_CONTROL_STATE_MASK) | state;

		err = mv88e6xxx_port_write(chip, i, PORT_CONTROL, reg);
		if (err)
			break;
	}
	mutex_unlock(&chip->reg_lock);

	/* Wait for transmit queues to drain.  Per datasheet, this is the
	 * maximum time for a frame to be transmitted at 10Mbps
	 */
	if (state == PORT_CONTROL_STATE_DISABLED) {
		usleep_range(2000, 4000);
	}

	return err;
}

static int mv88e6xxx_reset_wait(struct mv88e6xxx_chip *chip)
{
	/* Wait up to one second for reset to complete. */
	unsigned long timeout = jiffies + 1 * HZ;

	while (time_before(jiffies, timeout)) {
		u16 reg;
		int err;

		mutex_lock(&chip->reg_lock);
		err = mv88e6xxx_g1_read(chip, 0x00, &reg);
		mutex_unlock(&chip->reg_lock);
		if (err) {
			/* Register read timeouts are expected while chip is
			 * unavailable.
			 */
			if (err == -ETIMEDOUT)
				continue;
			return err;
		}

		if ((reg & 0x8800) == 0x8800)
			break;
		usleep_range(1000, 2000);
	}
	if (time_after(jiffies, timeout)) {
		dev_err(chip->dev, "Timed out waiting for reset\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int mv88e6xxx_reset(struct mv88e6xxx_chip *chip)
{
	int err = 0;

	/* Set all ports to the disabled state. */
	err = mv88e6xxx_set_all_port_state(chip, PORT_CONTROL_STATE_DISABLED);
	if (err) {
		return err;
	}

	/* If there is a gpio connected to the reset pin, toggle it */
	if (chip->reset) {
		gpiod_set_value_cansleep(chip->reset, 1);
		/* 88E6240 data sheet, "Reset and Configuration Timing" */
		usleep_range(10000, 15000);
		gpiod_set_value_cansleep(chip->reset, 0);
		/* 88E6240 data sheet, Table 5 lists up to 14ms of reset
		 * being asserted by the chip itself following external release.
		 */
		usleep_range(15000, 20000);
	} else {
		/* Otherwise soft reset the switch. Keep the PPU active. The PPU
		 * needs to be active to support indirect phy register access
		 * through global registers 0x18 and 0x19.
		 */
		mutex_lock(&chip->reg_lock);
		err = mv88e6xxx_g1_write(chip, 0x04, 0xc000);
		mutex_unlock(&chip->reg_lock);
		if (err < 0) {
			return err;
		}
	}

	err = mv88e6xxx_reset_wait(chip);
	if (err)
		return err;

	/* This is the post-hard reset configuration, but in the event of a
	 * soft reset, reenable the port above
	 */
	err = mv88e6xxx_set_all_port_state(chip, PORT_CONTROL_STATE_FORWARDING);

	return err;
}

static int mv88e6xxx_verify_eeprom(struct mv88e6xxx_chip *chip)
{
	int i;
	ssize_t len;

	chip->eeprom.dt_words = of_get_property(chip->np, "eeprom",
						&chip->eeprom.dt_len);
	if (!chip->eeprom.dt_words) {
		dev_err(chip->dev, "No 'eeprom' provided in device tree!\n");
		return -EINVAL;
	}
	if (chip->eeprom.dt_len > chip->eeprom.len) {
		dev_err(chip->dev, "'eeprom' len %d > 'eeprom-length' %lu!\n",
			chip->eeprom.dt_len, (unsigned long)chip->eeprom.len);
		return -EINVAL;
	}
	/* Each EEPROM entry is 16-bit command + 16-bit data.
	 * Check to make sure last command is terminator.
	 */
	if (chip->eeprom.dt_len % 4 != 0) {
		dev_err(chip->dev,
			"Invalid 'eeprom' len %d (2x16-bits per command)\n",
			chip->eeprom.dt_len);
		return -EINVAL;
	}
	if (chip->eeprom.dt_words[(chip->eeprom.dt_len /
				   sizeof(*chip->eeprom.dt_words)) - 2] !=
	    EEPROM_CMD_HALT_EEPROM_PROCESSING) {
		dev_err(chip->dev,
			"'eeprom' does not end with halt command!\n");
		return -EINVAL;
	}

	chip->eeprom.cache = devm_kzalloc(chip->dev, chip->eeprom.dt_len,
					  GFP_KERNEL);
	if (!chip->eeprom.cache)
		return -ENOMEM;

	len = mv88e6xxx_eeprom_read_locked(chip, chip->eeprom.cache->bytes,
					   0, chip->eeprom.dt_len);
	if (len != chip->eeprom.dt_len) {
		dev_err(chip->dev,
			"Failed to read EEPROM: returned %zd != desired %d\n",
			len, chip->eeprom.dt_len);
		chip->eeprom.mismatch = true;
		return 0;
	}

	/* We used kzalloc() for `chip`, but for clarity, set to false. */
	chip->eeprom.mismatch = false;
	for (i = 0; i < (chip->eeprom.dt_len / sizeof(u16)); i++) {
		/* EEPROM is little-endian, but device tree is big-endian. */
		if (chip->eeprom.cache->words[i] !=
		    swab16(chip->eeprom.dt_words[i])) {
			dev_notice(chip->dev,
				   "EEPROM/device-tree mismatch at word %d\n",
				   i);
			chip->eeprom.mismatch = true;
			break;
		}
	}

	return 0;
}

static int mv88e6xxx_configure_from_dt_eeprom(struct mv88e6xxx_chip *chip)
{
	int i;
	u16 command;

	for (i = 0;
	     i < (chip->eeprom.dt_len / sizeof(*chip->eeprom.dt_words)) - 1;
	     i += 2) {
		int err;
		int reg;
		u16 reg_data;
		unsigned long vector;
		int bit;

		command = be16_to_cpu(chip->eeprom.dt_words[i]);
		if (command == EEPROM_CMD_HALT_EEPROM_PROCESSING) {
			dev_info(chip->dev,
				 "Successfully applied configuration.\n");
			/* By using break instead of continue here, we
			 * allow for a second "program" to be specified in
			 * the device tree to be written to the EEPROM.
			 */
			break;
		}
		if (command == EEPROM_CMD_RELEASE_INTERNAL_RESET) {
			dev_warn(chip->dev,
				 "Skipping 'release internal reset'\n");
			continue;
		}
		/* For GENMASK(start, end), start and end are inclusive. */
		reg = command & GENMASK(EEPROM_CMD_REG_SIZE - 1, 0);
		reg_data = be16_to_cpu(chip->eeprom.dt_words[i + 1]);
		switch (command >> EEPROM_CMD_NIBBLE_SHIFT) {
		case EEPROM_CMD_NIBBLE_WRITE_GLOBAL:
			if (command & EEPROM_CMD_WRITE_GLOBAL_G2_BIT)
				err = mv88e6xxx_g2_write(chip, reg, reg_data);
			else
				err = mv88e6xxx_g1_write(chip, reg, reg_data);
			break;
		case EEPROM_CMD_NIBBLE_WRITE_MAC:
			vector = ((command &
				   GENMASK((EEPROM_CMD_REG_SIZE +
					    EEPROM_CMD_MAC_VECTOR_SIZE - 1),
					   EEPROM_CMD_REG_SIZE)) >>
				  EEPROM_CMD_REG_SIZE);
			for_each_set_bit(bit,
					 &vector,
					 EEPROM_CMD_MAC_VECTOR_SIZE) {
				err = mv88e6xxx_port_write(chip, bit, reg,
							   reg_data);
				if (err)
					break;
			}
			break;
		case EEPROM_CMD_NIBBLE_WRITE_PHY:
			vector = ((command &
				   GENMASK((EEPROM_CMD_REG_SIZE +
					    EEPROM_CMD_PHY_VECTOR_SIZE - 1),
					   EEPROM_CMD_REG_SIZE)) >>
				  EEPROM_CMD_REG_SIZE);
			for_each_set_bit(bit,
					 &vector,
					 EEPROM_CMD_PHY_VECTOR_SIZE) {
				err = mv88e6xxx_phy_write(chip, bit, reg,
							  reg_data);
				if (err)
					break;
			}
			break;
		default:
			dev_err(chip->dev, "Unknown command 0x%04X!\n",
				command);
			err = -EINVAL;
		}
		if (err) {
			dev_err(chip->dev,
				"Encountered error %d while writing config\n",
				err);
			return err;
		}
	}
	/* Should not happen - we checked for terminator in verify_eeprom. */
	WARN_ON_ONCE(command != EEPROM_CMD_HALT_EEPROM_PROCESSING);

	return 0;
}

static void mv88e6xxx_write_dt_eeprom(struct mv88e6xxx_chip *chip)
{
	ssize_t len;
	int i;

	/* copy the (big-endian) device tree EEPROM to the cache */
	memcpy(chip->eeprom.cache->bytes,
	       chip->eeprom.dt_words,
	       chip->eeprom.dt_len);
	/* Byte-swap the buffer to make little endian to match the physical
	 * EEPROM.
	 */
	for (i = 0; i < (chip->eeprom.dt_len / sizeof(u16)); i++) {
		chip->eeprom.cache->words[i] =
			swab16(chip->eeprom.cache->words[i]);
	}

	/* Attempt to write out the EEPROM. */
	len = mv88e6xxx_eeprom_write_locked(chip, chip->eeprom.cache->bytes,
					    0, chip->eeprom.dt_len);
	if (len != chip->eeprom.dt_len) {
		dev_err(chip->dev,
			"Failed to write EEPROM: returned %zd != desired %d\n",
			len, chip->eeprom.dt_len);
		chip->eeprom.write_failed = true;
		/* Even if EEPROM failed, try to apply configuration. */
	} else {
		dev_info(chip->dev, "EEPROM updated\n");
	}
}

static int mv88e6xxx_apply_dt_eeprom(struct mv88e6xxx_chip *chip)
{
	mv88e6xxx_write_dt_eeprom(chip);

	return mv88e6xxx_configure_from_dt_eeprom(chip);
}

static int mv88e6xxx_setup(struct mv88e6xxx_chip *chip)
{
	int err = 0;

	err = mv88e6xxx_verify_ports(chip);
	if (err)
		return err;

	err = mv88e6xxx_verify_eeprom(chip);
	if (err)
		return err;

	if (chip->eeprom.mismatch)
		return mv88e6xxx_apply_dt_eeprom(chip);
	return 0;
}

static int mv88e6xxx_teardown(struct mv88e6xxx_chip *chip)
{
	struct device_node *port_node, *child;

	port_node = of_find_node_by_name(chip->np, "ports");
	if (port_node) {
		for_each_available_child_of_node(port_node, child) {
			if (of_phy_is_fixed_link(child)) {
				of_phy_deregister_fixed_link(child);
			}
		}
	}

	return 0;
}

static int mv88e6xxx_detect(struct mv88e6xxx_chip *chip)
{
	unsigned int prod_num, rev;
	u16 id;
	int err;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_port_read(chip, 0, PORT_SWITCH_ID, &id);
	mutex_unlock(&chip->reg_lock);
	if (err)
		return err;

	prod_num = (id & 0xfff0) >> 4;
	rev = id & 0x000f;

	if (prod_num != chip->info->prod_num) {
		dev_info(chip->dev, "Unable to query switch\n",
			 prod_num);
		return -ENODEV;
	}

	dev_info(chip->dev, "switch 0x%x detected: %s, revision %u\n",
		 chip->info->prod_num, chip->info->name, rev);

	return 0;
}

static int mv88e6xxx_stats_wait(struct mv88e6xxx_chip *chip)
{
	u16 val;
	int i, err;

	for (i = 0; i < 10; i++) {
		err = mv88e6xxx_g1_read(chip, GLOBAL_STATS_OP, &val);
		if ((val & GLOBAL_STATS_OP_BUSY) == 0)
			return 0;

		usleep_range(10, 20);
	}

	return -ETIMEDOUT;
}

static int mv88e6xxx_stats_read(struct mv88e6xxx_chip *chip, int port, int stat,
				u64 *val, bool is_64bit, bool capture)
{
	u64 tmp_val;
	u16 reg;
	int err;

	*val = 0;

	if (capture) {
		/* Snapshot the hardware statistics counters for this port. */
		err = mv88e6xxx_g1_write(chip, GLOBAL_STATS_OP,
					 GLOBAL_STATS_OP_CAPTURE_PORT |
					 GLOBAL_STATS_OP_HIST_RX_TX |
					 ((port + 1) << 5));
		if (err)
			return err;

		/* Wait for the snapshotting to complete. */
		err = mv88e6xxx_stats_wait(chip);
		if (err)
			return err;
	};

	err = mv88e6xxx_g1_write(chip, GLOBAL_STATS_OP,
				 GLOBAL_STATS_OP_READ_CAPTURED |
				 GLOBAL_STATS_OP_HIST_RX_TX | stat);
	if (err)
		return err;

	err = mv88e6xxx_stats_wait(chip);
	if (err)
		return err;

	err = mv88e6xxx_g1_read(chip, GLOBAL_STATS_COUNTER_32, &reg);
	if (err)
		return err;

	tmp_val = (u64)reg << 16;

	err = mv88e6xxx_g1_read(chip, GLOBAL_STATS_COUNTER_01, &reg);
	if (err)
		return err;

	tmp_val |= (u64)reg;

	/* Revisit: 90% sure dsa driver is flat broken for this */
	if (is_64bit) {
		err = mv88e6xxx_g1_write(chip, GLOBAL_STATS_OP,
					 GLOBAL_STATS_OP_READ_CAPTURED |
					 GLOBAL_STATS_OP_HIST_RX_TX |
					 (stat + 1));
		if (err)
			return err;

		err = mv88e6xxx_stats_wait(chip);
		if (err)
			return err;

		err = mv88e6xxx_g1_read(chip, GLOBAL_STATS_COUNTER_32, &reg);
		if (err)
			return err;

		tmp_val |= ((u64)reg << 48);

		err = mv88e6xxx_g1_read(chip, GLOBAL_STATS_COUNTER_01, &reg);
		if (err)
			return err;

		tmp_val |= ((u64)reg << 32);
	}

	*val = tmp_val;

	return 0;
}

static ssize_t mv88e6xxx_stat_read(struct device *dev,
				   struct device_attribute *attr,
				   char *buf, int portnum, int reg,
				   bool is_64bit)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	u64 val;
	int status;

	mutex_lock(&chip->reg_lock);
	status = mv88e6xxx_stats_read(chip, portnum, reg, &val, is_64bit, true);
	mutex_unlock(&chip->reg_lock);
	if (status < 0) {
		return status;
	}

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t mv88e6xxx_aggregate_stat_read(struct device *dev,
					     struct device_attribute *attr,
					     char *buf, int portnum,
					     const u16 *regs, u16 regs_count)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	u64 val;
	int status;
	u64 total = 0;
	int i;

	mutex_lock(&chip->reg_lock);
	for (i = 0; i < regs_count; i++) {
		status = mv88e6xxx_stats_read(chip, portnum, regs[i], &val,
					      false, i == 0);
		if (status < 0)
			break;

		total += val;
	}
	mutex_unlock(&chip->reg_lock);
	if (status < 0) {
		return status;
	}

	return scnprintf(buf, PAGE_SIZE, "%llu\n", total);
}

static ssize_t mv88e6xxx_port_stat_read(struct device *dev,
					struct device_attribute *attr,
					char *buf, int portnum, int reg,
					uint first_bit, uint last_bit)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	u32 val;
	u16 regval;
	int status;

	if (portnum > chip->info->num_ports || reg >= 32 ||
	    first_bit > last_bit || last_bit >= 32)
		return -EINVAL;

	mutex_lock(&chip->reg_lock);
	status = mv88e6xxx_port_read(chip, portnum, reg, &regval);
	if (status) {
		mutex_unlock(&chip->reg_lock);
		return status;
	}

	val = regval;

	if (last_bit >= 16) {
		status = mv88e6xxx_port_read(chip, portnum, reg + 1, &regval);
		if (status) {
			mutex_unlock(&chip->reg_lock);
			return status;
		}
		val |= ((u32)regval << 16);
	}
	mutex_unlock(&chip->reg_lock);

	val = (val & GENMASK(last_bit, first_bit)) >> first_bit;

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

#define MV88E6XXX_PORT_GSTAT_ATTR(statname, portnum, regnum, is_64bit) \
static ssize_t mv88e6xx_stat_##portnum##_##statname(struct device *dev, \
					       struct device_attribute *attr, \
					       char *buf) \
{ \
	return mv88e6xxx_stat_read(dev, attr, buf, portnum, regnum, is_64bit); \
} \
static struct device_attribute dev_attr_stat##portnum##_##statname = \
				__ATTR(statname, 0444, \
				mv88e6xx_stat_##portnum##_##statname, \
				NULL)

#define MV88E6XXX_PORT_GSTAT_AGG_ATTR(statname, portnum, reglist) \
static ssize_t mv88e6xx_stat_##portnum##_##statname(struct device *dev, \
					       struct device_attribute *attr, \
					       char *buf) \
{ \
	return mv88e6xxx_aggregate_stat_read(dev, attr, buf, portnum, reglist, ARRAY_SIZE(reglist)); \
} \
static struct device_attribute dev_attr_stat##portnum##_##statname = \
				__ATTR(statname, 0444, \
				mv88e6xx_stat_##portnum##_##statname, \
				NULL)

#define MV88E6XXX_PORT_PSTAT_ATTR(statname, portnum, regnum, start_bit, stop_bit) \
static ssize_t mv88e6xx_stat_##portnum##_##statname(struct device *dev, \
					       struct device_attribute *attr, \
					       char *buf) \
{ \
	return mv88e6xxx_port_stat_read(dev, attr, buf, portnum, regnum, start_bit, stop_bit); \
} \
static struct device_attribute dev_attr_stat##portnum##_##statname = \
				__ATTR(statname, 0444, \
				mv88e6xx_stat_##portnum##_##statname, \
				NULL)

/* Table 3 set 2 */
static const u16 rx_packets[] = { 0x04, 0x06, 0x07, 0x16 };
/* Table 3 set 3 */
static const u16 rx_errors[] = { 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d };
/* Table 4 set 6 */
static const u16 tx_packets[] = { 0x10, 0x13, 0x12, 0x15 };
/* Table 4 set 7: Most are only applicable in half-duplex */
static const u16 tx_errors[] = { 0x03 };

#define MV88E6XXX_PORT_STAT(portnum) \
MV88E6XXX_PORT_GSTAT_ATTR(in_good_octets,	portnum, 0x00, true);  \
MV88E6XXX_PORT_GSTAT_ATTR(in_bad_octets,	portnum, 0x02, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_unicast,		portnum, 0x04, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_broadcasts,	portnum, 0x06, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_multicasts,	portnum, 0x07, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_pause,		portnum, 0x16, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_undersize,		portnum, 0x18, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_fragments,		portnum, 0x19, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_oversize,		portnum, 0x1a, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_jabber,		portnum, 0x1b, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_rx_error,		portnum, 0x1c, false); \
MV88E6XXX_PORT_GSTAT_ATTR(in_fcs_error,		portnum, 0x1d, false); \
MV88E6XXX_PORT_GSTAT_ATTR(out_octets,		portnum, 0x0e, true);  \
MV88E6XXX_PORT_GSTAT_ATTR(out_unicast,		portnum, 0x10, false); \
MV88E6XXX_PORT_GSTAT_ATTR(out_broadcasts,	portnum, 0x13, false); \
MV88E6XXX_PORT_GSTAT_ATTR(out_multicasts,	portnum, 0x12, false); \
MV88E6XXX_PORT_GSTAT_ATTR(out_pause,		portnum, 0x15, false); \
MV88E6XXX_PORT_GSTAT_ATTR(excessive,		portnum, 0x11, false); \
MV88E6XXX_PORT_GSTAT_ATTR(collisions,		portnum, 0x1e, false); \
MV88E6XXX_PORT_GSTAT_ATTR(deferred,		portnum, 0x05, false); \
MV88E6XXX_PORT_GSTAT_ATTR(single,		portnum, 0x14, false); \
MV88E6XXX_PORT_GSTAT_ATTR(multiple,		portnum, 0x17, false); \
MV88E6XXX_PORT_GSTAT_ATTR(out_fcs_error,	portnum, 0x03, false); \
MV88E6XXX_PORT_GSTAT_ATTR(late,			portnum, 0x1f, false); \
MV88E6XXX_PORT_GSTAT_ATTR(hist_64bytes,		portnum, 0x08, false); \
MV88E6XXX_PORT_GSTAT_ATTR(hist_65_127bytes,	portnum, 0x09, false); \
MV88E6XXX_PORT_GSTAT_ATTR(hist_128_255bytes,	portnum, 0x0a, false); \
MV88E6XXX_PORT_GSTAT_ATTR(hist_256_511bytes,	portnum, 0x0b, false); \
MV88E6XXX_PORT_GSTAT_ATTR(hist_512_1023bytes,	portnum, 0x0c, false); \
MV88E6XXX_PORT_GSTAT_ATTR(hist_1024_max_bytes,	portnum, 0x0d, false); \
MV88E6XXX_PORT_GSTAT_AGG_ATTR(rx_packets,	portnum, rx_packets);  \
MV88E6XXX_PORT_GSTAT_AGG_ATTR(rx_errors,	portnum, rx_errors);   \
MV88E6XXX_PORT_GSTAT_AGG_ATTR(tx_packets,	portnum, tx_packets);  \
MV88E6XXX_PORT_GSTAT_AGG_ATTR(tx_errors,	portnum, tx_errors);   \
MV88E6XXX_PORT_PSTAT_ATTR(sw_in_discards,	portnum, 0x10, 0, 31); \
MV88E6XXX_PORT_PSTAT_ATTR(sw_in_filtered,	portnum, 0x12, 0, 16); \
MV88E6XXX_PORT_PSTAT_ATTR(sw_out_filtered,	portnum, 0x13, 0, 16); \
static struct attribute *mv88e6xx_port##portnum##_stats_sysfs_attrs[] = { \
	&dev_attr_stat##portnum##_in_good_octets.attr, \
	&dev_attr_stat##portnum##_in_bad_octets.attr, \
	&dev_attr_stat##portnum##_in_unicast.attr, \
	&dev_attr_stat##portnum##_in_broadcasts.attr, \
	&dev_attr_stat##portnum##_in_multicasts.attr, \
	&dev_attr_stat##portnum##_in_pause.attr, \
	&dev_attr_stat##portnum##_in_undersize.attr, \
	&dev_attr_stat##portnum##_in_fragments.attr, \
	&dev_attr_stat##portnum##_in_oversize.attr, \
	&dev_attr_stat##portnum##_in_jabber.attr, \
	&dev_attr_stat##portnum##_in_rx_error.attr, \
	&dev_attr_stat##portnum##_in_fcs_error.attr, \
	&dev_attr_stat##portnum##_out_octets.attr, \
	&dev_attr_stat##portnum##_out_unicast.attr, \
	&dev_attr_stat##portnum##_out_broadcasts.attr, \
	&dev_attr_stat##portnum##_out_multicasts.attr, \
	&dev_attr_stat##portnum##_out_pause.attr, \
	&dev_attr_stat##portnum##_excessive.attr, \
	&dev_attr_stat##portnum##_collisions.attr, \
	&dev_attr_stat##portnum##_deferred.attr, \
	&dev_attr_stat##portnum##_single.attr, \
	&dev_attr_stat##portnum##_multiple.attr, \
	&dev_attr_stat##portnum##_out_fcs_error.attr, \
	&dev_attr_stat##portnum##_late.attr, \
	&dev_attr_stat##portnum##_hist_64bytes.attr, \
	&dev_attr_stat##portnum##_hist_65_127bytes.attr, \
	&dev_attr_stat##portnum##_hist_128_255bytes.attr, \
	&dev_attr_stat##portnum##_hist_256_511bytes.attr, \
	&dev_attr_stat##portnum##_hist_512_1023bytes.attr, \
	&dev_attr_stat##portnum##_hist_1024_max_bytes.attr, \
	&dev_attr_stat##portnum##_rx_packets.attr, \
	&dev_attr_stat##portnum##_rx_errors.attr, \
	&dev_attr_stat##portnum##_tx_packets.attr, \
	&dev_attr_stat##portnum##_tx_errors.attr, \
	&dev_attr_stat##portnum##_sw_in_discards.attr, \
	&dev_attr_stat##portnum##_sw_in_filtered.attr, \
	&dev_attr_stat##portnum##_sw_out_filtered.attr, \
	NULL, \
}; \
static const struct attribute_group port##portnum##_stats_group = { \
	.name = "port" #portnum "_stats", .attrs = mv88e6xx_port##portnum##_stats_sysfs_attrs, \
}

MV88E6XXX_PORT_STAT(0);
MV88E6XXX_PORT_STAT(1);
MV88E6XXX_PORT_STAT(2);
MV88E6XXX_PORT_STAT(3);
MV88E6XXX_PORT_STAT(4);
MV88E6XXX_PORT_STAT(5);
MV88E6XXX_PORT_STAT(6);

#define MV88E6XXX_PORT_STATE(portnum) \
MV88E6XXX_PORT_PSTAT_ATTR(speed,	portnum, 0x00, 8, 9); \
MV88E6XXX_PORT_PSTAT_ATTR(duplex,	portnum, 0x00, 10, 10); \
MV88E6XXX_PORT_PSTAT_ATTR(link,		portnum, 0x00, 11, 11); \
MV88E6XXX_PORT_PSTAT_ATTR(phy_detected,	portnum, 0x00, 12, 12); \
static struct attribute *mv88e6xx_port##portnum##_state_sysfs_attrs[] = { \
	&dev_attr_stat##portnum##_speed.attr, \
	&dev_attr_stat##portnum##_duplex.attr, \
	&dev_attr_stat##portnum##_link.attr, \
	&dev_attr_stat##portnum##_phy_detected.attr, \
	NULL, \
}; \
static const struct attribute_group port##portnum##_state_group = { \
	.name = "port" #portnum "_state", .attrs = mv88e6xx_port##portnum##_state_sysfs_attrs, \
}

MV88E6XXX_PORT_STATE(0);
MV88E6XXX_PORT_STATE(1);
MV88E6XXX_PORT_STATE(2);
MV88E6XXX_PORT_STATE(3);
MV88E6XXX_PORT_STATE(4);
MV88E6XXX_PORT_STATE(5);
MV88E6XXX_PORT_STATE(6);

/**
 * struct mv88e6xxx_raw_port_attribute - wrapper for raw port reads
 *
 * @dev_attr:	the &struct device_sttribute
 * @port:	port number
 * @reg:	register number
 */
struct mv88e6xxx_raw_port_attribute {
	struct device_attribute dev_attr;
	int port;
	int reg;
};

static ssize_t mv88e6xxx_raw_port_read(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	const struct mv88e6xxx_raw_port_attribute *port_attr =
		container_of(attr,
			     struct mv88e6xxx_raw_port_attribute,
			     dev_attr);
	int port = port_attr->port;
	int reg = port_attr->reg;
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	u16 val;
	int status;

	if (port >= chip->info->num_ports) {
		return -EINVAL;
	}

	mutex_lock(&chip->reg_lock);
	status = mv88e6xxx_port_read(chip, port, reg, &val);
	mutex_unlock(&chip->reg_lock);
	if (status < 0) {
		return status;
	}

	return scnprintf(buf, PAGE_SIZE, "0x%04x\n", val);
}

static ssize_t mv88e6xxx_raw_port_write(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	const struct mv88e6xxx_raw_port_attribute *port_attr =
		container_of(attr,
			     struct mv88e6xxx_raw_port_attribute,
			     dev_attr);
	int port = port_attr->port;
	int reg = port_attr->reg;
	u32 value;
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);

	if (port >= chip->info->num_ports) {
		return -EINVAL;
	}

	if (sscanf(buf, "%i\n", &value) != 1) {
		return -EINVAL;
	}

	mutex_lock(&chip->reg_lock);
	mv88e6xxx_port_write(chip, port, reg, value);
	mutex_unlock(&chip->reg_lock);

	return count;
}

/**
 * struct mv88e6xxx_raw_phy_attribute - wrapper for raw PHY reads
 *
 * @dev_attr:	the &struct device_sttribute
 * @port:	port number
 * @page:	PHY page number
 * @reg:	register number
 */
struct mv88e6xxx_raw_phy_attribute {
	struct device_attribute dev_attr;
	int port;
	int page;
	int reg;
};

static ssize_t mv88e6xxx_raw_phy_read(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	const struct mv88e6xxx_raw_phy_attribute *phy_attr =
		container_of(attr,
			     struct mv88e6xxx_raw_phy_attribute,
			     dev_attr);
	int port = phy_attr->port;
	int page = phy_attr->page;
	int reg = phy_attr->reg;
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	u16 val;
	int status;

	if (port >= chip->info->num_ports) {
		return -EINVAL;
	}

	mutex_lock(&chip->reg_lock);
	status = mv88e6xxx_phy_page_read(chip, port, page, reg, &val);
	mutex_unlock(&chip->reg_lock);
	if (status < 0) {
		return status;
	}

	return scnprintf(buf, PAGE_SIZE, "0x%04x\n", val);
}

static ssize_t mv88e6xxx_raw_phy_write(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	const struct mv88e6xxx_raw_phy_attribute *phy_attr =
		container_of(attr,
			     struct mv88e6xxx_raw_phy_attribute,
			     dev_attr);
	int port = phy_attr->port;
	int page = phy_attr->page;
	int reg = phy_attr->reg;
	u32 value;
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);

	if (port >= chip->info->num_ports) {
		return -EINVAL;
	}

	if (sscanf(buf, "%i\n", &value) != 1) {
		return -EINVAL;
	}

	mutex_lock(&chip->reg_lock);
	mv88e6xxx_phy_page_write(chip, port, page, reg, value);
	mutex_unlock(&chip->reg_lock);

	return count;
}

#define MV88E6XXX_RAW_PHY_ACCESS_REG(portnum, regnum, prefix, pagenum)	\
static struct mv88e6xxx_raw_phy_attribute \
attr_raw_phy_##prefix##portnum##_##regnum = { \
	.dev_attr = __ATTR(phy_##prefix##regnum, 0664, \
			   mv88e6xxx_raw_phy_read, \
			   mv88e6xxx_raw_phy_write), \
	.port = portnum, \
	.page = pagenum, \
	.reg = regnum, \
}

#define MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, regnum)	\
static struct mv88e6xxx_raw_port_attribute \
attr_raw_port_##portnum##_##regnum = { \
	.dev_attr = __ATTR(reg##regnum, 0664, \
			   mv88e6xxx_raw_port_read, \
			   mv88e6xxx_raw_port_write), \
	.port = portnum, \
	.reg = regnum, \
}; \
MV88E6XXX_RAW_PHY_ACCESS_REG(portnum, regnum, copper, PHY_PAGE_COPPER); \
MV88E6XXX_RAW_PHY_ACCESS_REG(portnum, regnum, mac, PHY_PAGE_MAC); \
MV88E6XXX_RAW_PHY_ACCESS_REG(portnum, regnum, prbs, PHY_PAGE_PRBS); \
MV88E6XXX_RAW_PHY_ACCESS_REG(portnum, regnum, vct, PHY_PAGE_VCT); \
MV88E6XXX_RAW_PHY_ACCESS_REG(portnum, regnum, pktgen, PHY_PAGE_PKTGEN); \
MV88E6XXX_RAW_PHY_ACCESS_REG(portnum, regnum, cable, PHY_PAGE_CABLE)

#define MV88E6XXX_RAW_PORT_ACCESS(portnum) \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  0); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  1); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  2); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  3); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  4); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  5); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  6); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  7); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  8); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum,  9); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 10); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 11); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 12); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 13); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 14); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 15); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 16); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 17); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 18); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 19); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 20); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 21); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 22); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 23); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 24); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 25); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 26); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 27); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 28); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 29); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 30); \
MV88E6XXX_RAW_PORT_ACCESS_REG(portnum, 31); \
static struct attribute *mv88e6xx_raw_port##portnum##_sysfs_reg_attrs[] = { \
	&attr_raw_port_##portnum##_0.dev_attr.attr, \
	&attr_raw_port_##portnum##_1.dev_attr.attr, \
	&attr_raw_port_##portnum##_2.dev_attr.attr, \
	&attr_raw_port_##portnum##_3.dev_attr.attr, \
	&attr_raw_port_##portnum##_4.dev_attr.attr, \
	&attr_raw_port_##portnum##_5.dev_attr.attr, \
	&attr_raw_port_##portnum##_6.dev_attr.attr, \
	&attr_raw_port_##portnum##_7.dev_attr.attr, \
	&attr_raw_port_##portnum##_8.dev_attr.attr, \
	&attr_raw_port_##portnum##_9.dev_attr.attr, \
	&attr_raw_port_##portnum##_10.dev_attr.attr, \
	&attr_raw_port_##portnum##_11.dev_attr.attr, \
	&attr_raw_port_##portnum##_12.dev_attr.attr, \
	&attr_raw_port_##portnum##_13.dev_attr.attr, \
	&attr_raw_port_##portnum##_14.dev_attr.attr, \
	&attr_raw_port_##portnum##_15.dev_attr.attr, \
	&attr_raw_port_##portnum##_16.dev_attr.attr, \
	&attr_raw_port_##portnum##_17.dev_attr.attr, \
	&attr_raw_port_##portnum##_18.dev_attr.attr, \
	&attr_raw_port_##portnum##_19.dev_attr.attr, \
	&attr_raw_port_##portnum##_20.dev_attr.attr, \
	&attr_raw_port_##portnum##_21.dev_attr.attr, \
	&attr_raw_port_##portnum##_22.dev_attr.attr, \
	&attr_raw_port_##portnum##_23.dev_attr.attr, \
	&attr_raw_port_##portnum##_24.dev_attr.attr, \
	&attr_raw_port_##portnum##_25.dev_attr.attr, \
	&attr_raw_port_##portnum##_26.dev_attr.attr, \
	&attr_raw_port_##portnum##_27.dev_attr.attr, \
	&attr_raw_port_##portnum##_28.dev_attr.attr, \
	&attr_raw_port_##portnum##_29.dev_attr.attr, \
	&attr_raw_port_##portnum##_30.dev_attr.attr, \
	&attr_raw_port_##portnum##_31.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_0.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_1.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_2.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_3.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_4.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_5.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_6.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_7.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_8.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_9.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_10.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_11.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_12.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_13.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_14.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_15.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_16.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_17.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_18.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_19.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_20.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_21.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_22.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_23.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_24.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_25.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_26.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_27.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_28.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_29.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_30.dev_attr.attr, \
	&attr_raw_phy_copper##portnum##_31.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_0.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_1.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_2.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_3.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_4.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_5.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_6.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_7.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_8.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_9.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_10.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_11.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_12.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_13.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_14.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_15.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_16.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_17.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_18.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_19.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_20.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_21.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_22.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_23.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_24.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_25.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_26.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_27.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_28.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_29.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_30.dev_attr.attr, \
	&attr_raw_phy_mac##portnum##_31.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_0.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_1.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_2.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_3.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_4.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_5.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_6.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_7.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_8.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_9.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_10.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_11.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_12.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_13.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_14.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_15.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_16.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_17.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_18.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_19.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_20.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_21.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_22.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_23.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_24.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_25.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_26.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_27.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_28.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_29.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_30.dev_attr.attr, \
	&attr_raw_phy_prbs##portnum##_31.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_0.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_1.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_2.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_3.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_4.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_5.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_6.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_7.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_8.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_9.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_10.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_11.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_12.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_13.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_14.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_15.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_16.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_17.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_18.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_19.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_20.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_21.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_22.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_23.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_24.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_25.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_26.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_27.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_28.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_29.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_30.dev_attr.attr, \
	&attr_raw_phy_vct##portnum##_31.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_0.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_1.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_2.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_3.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_4.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_5.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_6.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_7.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_8.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_9.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_10.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_11.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_12.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_13.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_14.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_15.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_16.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_17.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_18.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_19.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_20.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_21.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_22.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_23.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_24.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_25.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_26.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_27.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_28.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_29.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_30.dev_attr.attr, \
	&attr_raw_phy_pktgen##portnum##_31.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_0.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_1.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_2.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_3.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_4.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_5.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_6.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_7.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_8.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_9.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_10.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_11.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_12.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_13.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_14.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_15.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_16.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_17.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_18.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_19.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_20.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_21.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_22.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_23.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_24.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_25.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_26.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_27.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_28.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_29.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_30.dev_attr.attr, \
	&attr_raw_phy_cable##portnum##_31.dev_attr.attr, \
	NULL, \
}; \
static const struct attribute_group raw_port##portnum##_regs_group = { \
	.name = "port" #portnum "_raw", .attrs = mv88e6xx_raw_port##portnum##_sysfs_reg_attrs, \
}

MV88E6XXX_RAW_PORT_ACCESS(0);
MV88E6XXX_RAW_PORT_ACCESS(1);
MV88E6XXX_RAW_PORT_ACCESS(2);
MV88E6XXX_RAW_PORT_ACCESS(3);
MV88E6XXX_RAW_PORT_ACCESS(4);
MV88E6XXX_RAW_PORT_ACCESS(5);
MV88E6XXX_RAW_PORT_ACCESS(6);

static ssize_t mv88e6xxx_raw_g1_read(struct device *dev,
				     struct device_attribute *attr,
				     char *buf, int reg)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	u16 val;
	int status;

	mutex_lock(&chip->reg_lock);
	status = mv88e6xxx_g1_read(chip, reg, &val);
	mutex_unlock(&chip->reg_lock);
	if (status < 0) {
		return status;
	}

	return scnprintf(buf, PAGE_SIZE, "0x%04x\n", val);
}

static ssize_t mv88e6xxx_raw_g1_write(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count,
				      int reg)
{
	u32 value;
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);

	if (sscanf(buf, "%i\n", &value) != 1) {
		return -EINVAL;
	}

	mutex_lock(&chip->reg_lock);
	mv88e6xxx_g1_write(chip, reg, value);
	mutex_unlock(&chip->reg_lock);

	return count;
}

#define MV88E6XXX_RAW_G1_ACCESS_REG(regnum) \
static ssize_t mv88e6xx_raw_g1_##regnum##_read(struct device *dev, \
					       struct device_attribute *attr, \
					       char *buf) \
{ \
	return mv88e6xxx_raw_g1_read(dev, attr, buf, regnum); \
} \
static ssize_t mv88e6xx_raw_g1_##regnum##_write(struct device *dev, \
					       struct device_attribute *attr, \
					       const char *buf, size_t count) \
{ \
	return mv88e6xxx_raw_g1_write(dev, attr, buf, count, regnum); \
} \
static struct device_attribute dev_attr_raw_g1_##regnum = \
				__ATTR(reg##regnum, 0664, \
				mv88e6xx_raw_g1_##regnum##_read, \
				mv88e6xx_raw_g1_##regnum##_write)

MV88E6XXX_RAW_G1_ACCESS_REG(0);
MV88E6XXX_RAW_G1_ACCESS_REG(1);
MV88E6XXX_RAW_G1_ACCESS_REG(2);
MV88E6XXX_RAW_G1_ACCESS_REG(3);
MV88E6XXX_RAW_G1_ACCESS_REG(4);
MV88E6XXX_RAW_G1_ACCESS_REG(5);
MV88E6XXX_RAW_G1_ACCESS_REG(6);
MV88E6XXX_RAW_G1_ACCESS_REG(7);
MV88E6XXX_RAW_G1_ACCESS_REG(8);
MV88E6XXX_RAW_G1_ACCESS_REG(9);
MV88E6XXX_RAW_G1_ACCESS_REG(10);
MV88E6XXX_RAW_G1_ACCESS_REG(11);
MV88E6XXX_RAW_G1_ACCESS_REG(12);
MV88E6XXX_RAW_G1_ACCESS_REG(13);
MV88E6XXX_RAW_G1_ACCESS_REG(14);
MV88E6XXX_RAW_G1_ACCESS_REG(15);
MV88E6XXX_RAW_G1_ACCESS_REG(16);
MV88E6XXX_RAW_G1_ACCESS_REG(17);
MV88E6XXX_RAW_G1_ACCESS_REG(18);
MV88E6XXX_RAW_G1_ACCESS_REG(19);
MV88E6XXX_RAW_G1_ACCESS_REG(20);
MV88E6XXX_RAW_G1_ACCESS_REG(21);
MV88E6XXX_RAW_G1_ACCESS_REG(22);
MV88E6XXX_RAW_G1_ACCESS_REG(23);
MV88E6XXX_RAW_G1_ACCESS_REG(24);
MV88E6XXX_RAW_G1_ACCESS_REG(25);
MV88E6XXX_RAW_G1_ACCESS_REG(26);
MV88E6XXX_RAW_G1_ACCESS_REG(27);
MV88E6XXX_RAW_G1_ACCESS_REG(28);
MV88E6XXX_RAW_G1_ACCESS_REG(29);
MV88E6XXX_RAW_G1_ACCESS_REG(30);
MV88E6XXX_RAW_G1_ACCESS_REG(31);

static struct attribute *mv88e6xx_raw_g1_sysfs_reg_attrs[] = {
	&dev_attr_raw_g1_0.attr,
	&dev_attr_raw_g1_1.attr,
	&dev_attr_raw_g1_2.attr,
	&dev_attr_raw_g1_3.attr,
	&dev_attr_raw_g1_4.attr,
	&dev_attr_raw_g1_5.attr,
	&dev_attr_raw_g1_6.attr,
	&dev_attr_raw_g1_7.attr,
	&dev_attr_raw_g1_8.attr,
	&dev_attr_raw_g1_9.attr,
	&dev_attr_raw_g1_10.attr,
	&dev_attr_raw_g1_11.attr,
	&dev_attr_raw_g1_12.attr,
	&dev_attr_raw_g1_13.attr,
	&dev_attr_raw_g1_14.attr,
	&dev_attr_raw_g1_15.attr,
	&dev_attr_raw_g1_16.attr,
	&dev_attr_raw_g1_17.attr,
	&dev_attr_raw_g1_18.attr,
	&dev_attr_raw_g1_19.attr,
	&dev_attr_raw_g1_20.attr,
	&dev_attr_raw_g1_21.attr,
	&dev_attr_raw_g1_22.attr,
	&dev_attr_raw_g1_23.attr,
	&dev_attr_raw_g1_24.attr,
	&dev_attr_raw_g1_25.attr,
	&dev_attr_raw_g1_26.attr,
	&dev_attr_raw_g1_27.attr,
	&dev_attr_raw_g1_28.attr,
	&dev_attr_raw_g1_29.attr,
	&dev_attr_raw_g1_30.attr,
	&dev_attr_raw_g1_31.attr,
	NULL,
};

static const struct attribute_group raw_g1_regs_group = {
	.name = "raw_g1", .attrs = mv88e6xx_raw_g1_sysfs_reg_attrs,
};

static ssize_t mv88e6xxx_raw_g2_read(struct device *dev,
				     struct device_attribute *attr,
				     char *buf, int reg)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	u16 val;
	int status;

	mutex_lock(&chip->reg_lock);
	status = mv88e6xxx_g2_read(chip, reg, &val);
	mutex_unlock(&chip->reg_lock);
	if (status < 0) {
		return status;
	}

	return scnprintf(buf, PAGE_SIZE, "0x%04x\n", val);
}

static ssize_t mv88e6xxx_raw_g2_write(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count,
				      int reg)
{
	u32 value;
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);

	if (sscanf(buf, "%i\n", &value) != 1) {
		return -EINVAL;
	}

	mutex_lock(&chip->reg_lock);
	mv88e6xxx_g2_write(chip, reg, value);
	mutex_unlock(&chip->reg_lock);

	return count;
}

#define MV88E6XXX_RAW_G2_ACCESS_REG(regnum) \
static ssize_t mv88e6xx_raw_g2_##regnum##_read(struct device *dev, \
					       struct device_attribute *attr, \
					       char *buf) \
{ \
	return mv88e6xxx_raw_g2_read(dev, attr, buf, regnum); \
} \
static ssize_t mv88e6xx_raw_g2_##regnum##_write(struct device *dev, \
					       struct device_attribute *attr, \
					       const char *buf, size_t count) \
{ \
	return mv88e6xxx_raw_g2_write(dev, attr, buf, count, regnum); \
} \
static struct device_attribute dev_attr_raw_g2_##regnum = \
				__ATTR(reg##regnum, 0664, \
				mv88e6xx_raw_g2_##regnum##_read, \
				mv88e6xx_raw_g2_##regnum##_write)

MV88E6XXX_RAW_G2_ACCESS_REG(0);
MV88E6XXX_RAW_G2_ACCESS_REG(1);
MV88E6XXX_RAW_G2_ACCESS_REG(2);
MV88E6XXX_RAW_G2_ACCESS_REG(3);
MV88E6XXX_RAW_G2_ACCESS_REG(4);
MV88E6XXX_RAW_G2_ACCESS_REG(5);
MV88E6XXX_RAW_G2_ACCESS_REG(6);
MV88E6XXX_RAW_G2_ACCESS_REG(7);
MV88E6XXX_RAW_G2_ACCESS_REG(8);
MV88E6XXX_RAW_G2_ACCESS_REG(9);
MV88E6XXX_RAW_G2_ACCESS_REG(10);
MV88E6XXX_RAW_G2_ACCESS_REG(11);
MV88E6XXX_RAW_G2_ACCESS_REG(12);
MV88E6XXX_RAW_G2_ACCESS_REG(13);
MV88E6XXX_RAW_G2_ACCESS_REG(14);
MV88E6XXX_RAW_G2_ACCESS_REG(15);
MV88E6XXX_RAW_G2_ACCESS_REG(16);
MV88E6XXX_RAW_G2_ACCESS_REG(17);
MV88E6XXX_RAW_G2_ACCESS_REG(18);
MV88E6XXX_RAW_G2_ACCESS_REG(19);
MV88E6XXX_RAW_G2_ACCESS_REG(20);
MV88E6XXX_RAW_G2_ACCESS_REG(21);
MV88E6XXX_RAW_G2_ACCESS_REG(22);
MV88E6XXX_RAW_G2_ACCESS_REG(23);
MV88E6XXX_RAW_G2_ACCESS_REG(24);
MV88E6XXX_RAW_G2_ACCESS_REG(25);
MV88E6XXX_RAW_G2_ACCESS_REG(26);
MV88E6XXX_RAW_G2_ACCESS_REG(27);
MV88E6XXX_RAW_G2_ACCESS_REG(28);
MV88E6XXX_RAW_G2_ACCESS_REG(29);
MV88E6XXX_RAW_G2_ACCESS_REG(30);
MV88E6XXX_RAW_G2_ACCESS_REG(31);

static struct attribute *mv88e6xx_raw_g2_sysfs_reg_attrs[] = {
	&dev_attr_raw_g2_0.attr,
	&dev_attr_raw_g2_1.attr,
	&dev_attr_raw_g2_2.attr,
	&dev_attr_raw_g2_3.attr,
	&dev_attr_raw_g2_4.attr,
	&dev_attr_raw_g2_5.attr,
	&dev_attr_raw_g2_6.attr,
	&dev_attr_raw_g2_7.attr,
	&dev_attr_raw_g2_8.attr,
	&dev_attr_raw_g2_9.attr,
	&dev_attr_raw_g2_10.attr,
	&dev_attr_raw_g2_11.attr,
	&dev_attr_raw_g2_12.attr,
	&dev_attr_raw_g2_13.attr,
	&dev_attr_raw_g2_14.attr,
	&dev_attr_raw_g2_15.attr,
	&dev_attr_raw_g2_16.attr,
	&dev_attr_raw_g2_17.attr,
	&dev_attr_raw_g2_18.attr,
	&dev_attr_raw_g2_19.attr,
	&dev_attr_raw_g2_20.attr,
	&dev_attr_raw_g2_21.attr,
	&dev_attr_raw_g2_22.attr,
	&dev_attr_raw_g2_23.attr,
	&dev_attr_raw_g2_24.attr,
	&dev_attr_raw_g2_25.attr,
	&dev_attr_raw_g2_26.attr,
	&dev_attr_raw_g2_27.attr,
	&dev_attr_raw_g2_28.attr,
	&dev_attr_raw_g2_29.attr,
	&dev_attr_raw_g2_30.attr,
	&dev_attr_raw_g2_31.attr,
	NULL,
};

static const struct attribute_group raw_g2_regs_group = {
	.name = "raw_g2", .attrs = mv88e6xx_raw_g2_sysfs_reg_attrs,
};

static ssize_t mv88e6xxx_raw_serdes_read(struct device *dev,
				     struct device_attribute *attr,
				     char *buf, int reg)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	u16 val;
	int status;

	mutex_lock(&chip->reg_lock);
	status = mv88e6xxx_serdes_read(chip, reg, &val);
	mutex_unlock(&chip->reg_lock);
	if (status < 0) {
		return status;
	}

	return scnprintf(buf, PAGE_SIZE, "0x%04x\n", val);
}

static ssize_t mv88e6xxx_raw_serdes_write(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count,
				      int reg)
{
	u32 value;
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);

	if (sscanf(buf, "%i\n", &value) != 1) {
		return -EINVAL;
	}

	mutex_lock(&chip->reg_lock);
	mv88e6xxx_serdes_write(chip, reg, value);
	mutex_unlock(&chip->reg_lock);

	return count;
}

#define MV88E6XXX_RAW_SERDES_ACCESS_REG(regnum) \
static ssize_t mv88e6xx_raw_serdes_##regnum##_read(struct device *dev, \
					       struct device_attribute *attr, \
					       char *buf) \
{ \
	return mv88e6xxx_raw_serdes_read(dev, attr, buf, regnum); \
} \
static ssize_t mv88e6xx_raw_serdes_##regnum##_write(struct device *dev, \
					       struct device_attribute *attr, \
					       const char *buf, size_t count) \
{ \
	return mv88e6xxx_raw_serdes_write(dev, attr, buf, count, regnum); \
} \
static struct device_attribute dev_attr_raw_serdes_##regnum = \
				__ATTR(reg##regnum, 0664, \
				mv88e6xx_raw_serdes_##regnum##_read, \
				mv88e6xx_raw_serdes_##regnum##_write)

MV88E6XXX_RAW_SERDES_ACCESS_REG(0);
MV88E6XXX_RAW_SERDES_ACCESS_REG(1);
MV88E6XXX_RAW_SERDES_ACCESS_REG(2);
MV88E6XXX_RAW_SERDES_ACCESS_REG(3);
MV88E6XXX_RAW_SERDES_ACCESS_REG(4);
MV88E6XXX_RAW_SERDES_ACCESS_REG(5);
MV88E6XXX_RAW_SERDES_ACCESS_REG(6);
MV88E6XXX_RAW_SERDES_ACCESS_REG(7);
MV88E6XXX_RAW_SERDES_ACCESS_REG(8);
MV88E6XXX_RAW_SERDES_ACCESS_REG(9);
MV88E6XXX_RAW_SERDES_ACCESS_REG(10);
MV88E6XXX_RAW_SERDES_ACCESS_REG(11);
MV88E6XXX_RAW_SERDES_ACCESS_REG(12);
MV88E6XXX_RAW_SERDES_ACCESS_REG(13);
MV88E6XXX_RAW_SERDES_ACCESS_REG(14);
MV88E6XXX_RAW_SERDES_ACCESS_REG(15);
MV88E6XXX_RAW_SERDES_ACCESS_REG(16);
MV88E6XXX_RAW_SERDES_ACCESS_REG(17);
MV88E6XXX_RAW_SERDES_ACCESS_REG(18);
MV88E6XXX_RAW_SERDES_ACCESS_REG(19);
MV88E6XXX_RAW_SERDES_ACCESS_REG(20);
MV88E6XXX_RAW_SERDES_ACCESS_REG(21);
MV88E6XXX_RAW_SERDES_ACCESS_REG(22);
MV88E6XXX_RAW_SERDES_ACCESS_REG(23);
MV88E6XXX_RAW_SERDES_ACCESS_REG(24);
MV88E6XXX_RAW_SERDES_ACCESS_REG(25);
MV88E6XXX_RAW_SERDES_ACCESS_REG(26);
MV88E6XXX_RAW_SERDES_ACCESS_REG(27);
MV88E6XXX_RAW_SERDES_ACCESS_REG(28);
MV88E6XXX_RAW_SERDES_ACCESS_REG(29);
MV88E6XXX_RAW_SERDES_ACCESS_REG(30);
MV88E6XXX_RAW_SERDES_ACCESS_REG(31);

static struct attribute *mv88e6xx_raw_serdes_sysfs_reg_attrs[] = {
	&dev_attr_raw_serdes_0.attr,
	&dev_attr_raw_serdes_1.attr,
	&dev_attr_raw_serdes_2.attr,
	&dev_attr_raw_serdes_3.attr,
	&dev_attr_raw_serdes_4.attr,
	&dev_attr_raw_serdes_5.attr,
	&dev_attr_raw_serdes_6.attr,
	&dev_attr_raw_serdes_7.attr,
	&dev_attr_raw_serdes_8.attr,
	&dev_attr_raw_serdes_9.attr,
	&dev_attr_raw_serdes_10.attr,
	&dev_attr_raw_serdes_11.attr,
	&dev_attr_raw_serdes_12.attr,
	&dev_attr_raw_serdes_13.attr,
	&dev_attr_raw_serdes_14.attr,
	&dev_attr_raw_serdes_15.attr,
	&dev_attr_raw_serdes_16.attr,
	&dev_attr_raw_serdes_17.attr,
	&dev_attr_raw_serdes_18.attr,
	&dev_attr_raw_serdes_19.attr,
	&dev_attr_raw_serdes_20.attr,
	&dev_attr_raw_serdes_21.attr,
	&dev_attr_raw_serdes_22.attr,
	&dev_attr_raw_serdes_23.attr,
	&dev_attr_raw_serdes_24.attr,
	&dev_attr_raw_serdes_25.attr,
	&dev_attr_raw_serdes_26.attr,
	&dev_attr_raw_serdes_27.attr,
	&dev_attr_raw_serdes_28.attr,
	&dev_attr_raw_serdes_29.attr,
	&dev_attr_raw_serdes_30.attr,
	&dev_attr_raw_serdes_31.attr,
	NULL,
};
static const struct attribute_group raw_serdes_regs_group = {
	.name = "raw_serdes", .attrs = mv88e6xx_raw_serdes_sysfs_reg_attrs,
};

static ssize_t mv88e6xxx_control_reset_write(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	int err;
	u32 value;
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);

	if (sscanf(buf, "%u\n", &value) != 1) {
		return -EINVAL;
	}

	/* Writing zero does nothing. Any other value triggers a reset. */
	if (value == 0) {
		return count;
	}

	err = mv88e6xxx_reset(chip);
	if (err) {
		return err;
	}

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_setup(chip);
	mutex_unlock(&chip->reg_lock);
	if (err) {
		return err;
	}

	return count;
}

static struct device_attribute dev_attr_control_reset =
	__ATTR(reset, 0220, NULL, mv88e6xxx_control_reset_write);

/**
 * mv88e6xxx_control_eeprom_load_write() - Tell chip to load config from EEPROM
 *
 * @dev: the &struct device for our chip
 * @attr: the &struct device_attribute we wrote to
 * @buf: must contain an integer byte offset (must be even due to 16-bit words)
 *       from which to read commands and values
 * @count: number of bytes in @buf
 *
 * This can be used to read an EEPROM configuration even if it does not start
 * at offset 0 (i.e., not read at boot). The EEPROM can therefore contain
 * multiple configurations; only the one starting at offset 0 is applied at
 * power-on.
 *
 * See "EEPROM Programming Format" in the chip datasheet for information about
 * the EEPROM format.
 *
 * Return: number of bytes read on success, or negative error value
 */
static ssize_t mv88e6xxx_control_eeprom_load_write(
	struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);
	int err = 0;
	unsigned long value;

	if ((kstrtoul(buf, 0, &value) != 0) ||
	    (value & 1) == 1 ||
	    value >= chip->eeprom.len)
		return -EINVAL;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_g2_eeprom_wait(chip);
	if (!err)
		err = mv88e6xxx_g2_eeprom_cmd(chip,
					      (GLOBAL2_EEPROM_CMD_OP_LOAD |
					       (value >> 1)));
	mutex_unlock(&chip->reg_lock);

	if (!err)
		return count;
	return err;
}

static struct device_attribute dev_attr_control_eeprom_load =
	__ATTR(eeprom_load, 0220, NULL, mv88e6xxx_control_eeprom_load_write);

static struct attribute *mv88e6xx_control_sysfs_reg_attrs[] = {
	&dev_attr_control_reset.attr,
	&dev_attr_control_eeprom_load.attr,
	NULL,
};

static const struct attribute_group control_group = {
	.name = "control", .attrs = mv88e6xx_control_sysfs_reg_attrs,
};

static ssize_t mv88e6xxx_init_eeprom_mismatch_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->eeprom.mismatch);
}

static ssize_t mv88e6xxx_init_eeprom_write_failed_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->eeprom.write_failed);
}

static struct device_attribute dev_attr_init_eeprom_mismatch =
	__ATTR(eeprom_mismatch, 0440,
	       mv88e6xxx_init_eeprom_mismatch_show, NULL);

static struct device_attribute dev_attr_init_eeprom_write_failed =
	__ATTR(eeprom_write_failed, 0440,
	       mv88e6xxx_init_eeprom_write_failed_show, NULL);

static struct attribute *mv88e6xx_init_sysfs_attrs[] = {
	&dev_attr_init_eeprom_mismatch.attr,
	&dev_attr_init_eeprom_write_failed.attr,
	NULL,
};

static const struct attribute_group init_group = {
	.name = "init", .attrs = mv88e6xx_init_sysfs_attrs,
};

static const struct attribute_group *mv88e6xxx_sysfs_groups[] = {
	&raw_port0_regs_group,
	&raw_port1_regs_group,
	&raw_port2_regs_group,
	&raw_port3_regs_group,
	&raw_port4_regs_group,
	&raw_port5_regs_group,
	&raw_port6_regs_group,
	&raw_g1_regs_group,
	&raw_g2_regs_group,
	&raw_serdes_regs_group,
	&port0_stats_group,
	&port1_stats_group,
	&port2_stats_group,
	&port3_stats_group,
	&port4_stats_group,
	&port5_stats_group,
	&port6_stats_group,
	&port0_state_group,
	&port1_state_group,
	&port2_state_group,
	&port3_state_group,
	&port4_state_group,
	&port5_state_group,
	&port6_state_group,
	&control_group,
	&init_group,
	NULL,
};

static int mv88e6xxx_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct mv88e6xxx_chip *chip;
	int err;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	dev_set_drvdata(dev, chip);
	chip->dev = dev;

	mutex_init(&chip->reg_lock);
	chip->info = &mv88e6240_info;
	chip->bus = mdiodev->bus;
	chip->sw_addr = mdiodev->addr;
	chip->np = dev->of_node;

	/* ADDR[0] pin is unavailable externally and considered zero */
	if (chip->sw_addr & 0x1)
		return -EINVAL;

	if (chip->sw_addr == 0)
		chip->smi_ops = &mv88e6xxx_smi_single_chip_ops;
	else
		chip->smi_ops = &mv88e6xxx_smi_multi_chip_ops;

	err = of_property_read_u32(chip->np, "port-count", &chip->port_count);
	if (err) {
		dev_err(dev, "Port count not specified\n");
		return err;
	}

	err = of_property_read_u32(chip->np, "eeprom-length",
				   &chip->eeprom.len);
	if (err) {
		dev_info(dev, "eeprom-length not specified in device tree\n");
		return err;
	}

	/* Release reset if board was previously in reset */
	chip->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(chip->reset))
		return PTR_ERR(chip->reset);
	if (chip->reset) {
		err = mv88e6xxx_reset_wait(chip);
		if (err)
			return err;
	}

	err = mv88e6xxx_detect(chip);
	if (err)
		return err;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_setup(chip);
	mutex_unlock(&chip->reg_lock);
	if (err)
		return err;

	err = sysfs_create_groups(&dev->kobj, mv88e6xxx_sysfs_groups);
	if (err)
		dev_err(dev, "Failed creating reg sysfs (%d)\n", err);
	sysfs_bin_attr_init(&chip->eeprom.attr);
	chip->eeprom.attr.attr.name = "eeprom";
	chip->eeprom.attr.attr.mode = 0600;
	chip->eeprom.attr.private = chip;
	chip->eeprom.attr.size = chip->eeprom.len;
	chip->eeprom.attr.read = mv88e6xxx_eeprom_read;
	chip->eeprom.attr.write = mv88e6xxx_eeprom_write;
	/* not using mmap */
	err = sysfs_create_bin_file(&dev->kobj, &chip->eeprom.attr);
	if (err)
		dev_err(dev,
			"Failed to create EEPROM sysfs file (%d)\n",
			err);

	return 0;
}

static void mv88e6xxx_remove(struct mdio_device *mdiodev)
{
	struct mv88e6xxx_chip *chip = dev_get_drvdata(&mdiodev->dev);

	sysfs_remove_bin_file(&chip->dev->kobj, &chip->eeprom.attr);
	sysfs_remove_groups(&chip->dev->kobj, mv88e6xxx_sysfs_groups);

	mv88e6xxx_teardown(chip);
}

static const struct of_device_id mv88e6xxx_of_match[] = {
	{
		.compatible = "marvell,mv88eXXXX-normal",
	},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, mv88e6xxx_of_match);

static struct mdio_driver mv88e6xxx_driver = {
	.probe  = mv88e6xxx_probe,
	.remove = mv88e6xxx_remove,
	.mdiodrv.driver = {
		.name = "mv88eXXXX-normal",
		.of_match_table = mv88e6xxx_of_match,
	},
};

static int __init mv88e6xxx_init(void)
{
	return mdio_driver_register(&mv88e6xxx_driver);
}
module_init(mv88e6xxx_init);

static void __exit mv88e6xxx_cleanup(void)
{
	mdio_driver_unregister(&mv88e6xxx_driver);
}
module_exit(mv88e6xxx_cleanup);

MODULE_AUTHOR("Kevin Bosien <kevin.bosien@spacex.com");
MODULE_DESCRIPTION("Driver for Marvell 88E6XXX ethernet switch chips");
MODULE_LICENSE("GPL");
