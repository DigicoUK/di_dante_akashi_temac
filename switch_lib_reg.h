/*
 * switch_lib_reg.h
 *
 *  Created on: 07/05/2014
 *      Author: -Jerry-
 */

#ifndef SWITCH_LIB_REG_H_
#define SWITCH_LIB_REG_H_

#include "switch_lib.h"
#ifdef SWITCH_LIB_DANTE_APP
#include "regif.h"
#endif

//----------
#ifdef __cplusplus
extern "C" {
#endif


/*******************************************
 ************ Definitions ******************
 *******************************************/
#define PHY_UNIQUE_ID				0x7
#define PHY_MICREL_UNIQUE_ID			0x22

/*
 * Switch register control/monitor registers/bits
 */
#define SWITCH_PORT_BASE_ADDR			0x10
#define SWITCH_PORT_STAT_REG			0x00	//Part2 Pg156
#define SWITCH_PORT_VLANT_REG			0x06	//Part2 Pg173, Pg37
#define SWITCH_PORT_ON_VAL			0x003f
#define SWITCH_PHY_CTRL_REG			0x00	//Part3 Pg39
#define SWITCH_PHY_ON_VAL			0x1140
#define SWITCH_PHY_ON_VAL			0x1140

#define SWITCH_PORT_STATUS_GET_6350R_CMODE(x)		((x) & 0x7)
#define SWITCH_PORT_STATUS_GET_6320_6352_CMODE(x)	((x) & 0xf)


/* Switch Registers */
#define SWITCH_GLOBAL_REGISTER_GROUP_1		0x1b
#define SWITCH_GLOBAL_REGISTER_GROUP_2		0x1c
#define SWITCH_GLOBAL_CTLR_REG			4
#define SWITCH_GLOBAL_MANAGEMENT_REG		5
#define SWITCH_PORT_NUMBER_5			(1<<5)

#define SWITCH_FLOOD_BROADCAST			(1<<12)
#define SWITCH_GLOBAL1_STATUS_PPU_STATE		(1<<14)
#define SWITCH_GLOCAL1_STATUS_INIT_READY 	(1<<11) // global1 status register.

// Physical Control Register - Part2, Pg 165, Table 39
#define SWITCH_PHYSICAL_CTRL_REG			1
#define SWITCH_PHYSICAL_CTRL_RGMII_DELAY		(3<<14) //Bit 14:15 - 1:1 // Rx & TX delay
#define SWITCH_PHYSICAL_CTRL_FORCE_SPEED_DEFAULT	(3<<0)  //Bit 0:1 - 3 //Normal speed detection occurs
#define SWITCH_PHYSICAL_CTRL_FORCE_LINK_DOWN		(1<<4) //Bit 4 - 1 // ForceLink down

// Port Control Register - Part2 Pg 174, Table 42
#define SWITCH_PORT_CTRL_REG				4
#define SWITCH_PORT_DEFAULT_VLAN_ID_N_PRIORITY		7
#define SWITCH_PORT_ETYPE_REG				15
#define SWITCH_PORT_CTRL_FORWARDING			(3<<0) 		//Bit 0:1 - 3 // Forwarding
#define SWITCH_PORT_CTRL_DISABLED			(0xfffc)   	//Bit 0:1 - 0x00 // disable port
#define SWITCH_PORT_INGRESS_EGRESS_HEADER_MODE		(1<<11)
#define SWITCH_PORT_EGRESS_ETHER_TYPE_DSA_TAG_MODE	(3<<12)		// 11 - ADDING ether type dsa tag for control packets. - egress
#define SWITCH_PORT_FRAME_ETHER_TYPE_DSA_MODE		(3<<8)		// 11 - ADDING ether type dsa tag for control packets. - Ingress
#define SWITCH_PORT_PTP_EVENTS_MASK			(0xcfff)	// IP-PRI reg 0x11 bit 12:13 - set as highest for diffserv 0x38
#define SWITCH_PORT_TAGIFBOTH_MASK			(0xffbf)	// Port control reg bit 6 - TagIfBoth bit mask
#define SWITCH_PORT_TAGIFBOTH				(0x0<<6)	// Port control reg bit 6 - set as 0 to use IP Pri mapping regs (0x10-0x17)
#define SWITCH_PORT_IP_PRIORITY_MASK			(0xffcf)	// Port control reg bit 4:5 - Initial priority bits mask
#define SWITCH_PORT_IP_PRIORITY				(0x2<<4)	// Port control reg bit 4:5 - Set as 10 to use IPv4, IPv6 port priority only
#define PHY_CONTROL_REGISTER_OFFSET			0		// Page 0, Register 0
#define PHY_COPPER_SPECIFIC_CONTROL_REGISTER_OFFSET 16			// Page 0, Register 16

#define SWITCH_IDENTIFIER_REGISTER			3		// part2 offset 3

// Port numbers
#define SWITCH_PORT_NUM_0			0
#define SWITCH_PORT_NUM_1			1
#define SWITCH_PORT_NUM_2			2
#define SWITCH_PORT_NUM_3			3
#define SWITCH_PORT_NUM_4			4
#define SWITCH_PORT_NUM_5			5
#define SWITCH_PORT_NUM_6			6

#define PHY_MODE_CTRL_REG	0
#define PHY_MODE_CTRL_ISOLATE	(1<<10)

#define PHY_CONTROL_POWER_DOWN			(1<<11)     //Part3 Pg 39, Table 9,  Bit 11    - 0
#define PHY_CONTROL_SOFTWARE_RESET		(1<<15)     //Part3 Pg 39, Table 9,  Bit 15    - 1
#define PHY_COPPER_SPECIFIC_CONTROL_POWER_UP	(3<<2)      //Part3 Pg 53, Table 21, Bit 2     - 0
#define PHY_COPPER_SPECIFIC_CONTROL_ENERGE_DETECT	(3<<8)      //Part3 Pg 52, Table 21, Bit 8:9   - 0

#define PHY_AUXILIARY_CONTROL_STATUS_REGISTER		28
#define PHY_IDENTIFIER_REGISTER_2		3	// vitesse page 41
#define PHY_IDENTIFIER_REGISTER_1		2	// vitesse page 41
#define PHY_VITESSSE_UNIQUE_ID			0x7	// vitesse page 41
#define PHY_MICREL_UNIQUE_ID			0x22	// micrel page 43

#define PHY_KSZ9031_CONTROL_STATUS_REGISTER	0x1f
#define PHY_KSZ9031_SPEED_1G			(1<<6)
#define PHY_KSZ9031_SPEED_100M			(1<<5)
#define PHY_KSZ9031_DUPLEX_STATUS		(1<<3)

#define PHY_STATUS_SPEED_STATUS_MASK		0x18	// vitesse page 54
#define PHY_STATUS_SPEED_1000BASE_T		(1<<4)	// vitesse page 54 bit 4
#define PHY_STATUS_SPEED_100BASE_TX		(1<<3)	// vitesss page 54 bit 3

#define PHY_MODE_STATUS_REGISTER		1
#define PHY_MODE_STATUS_AUTO_NEGOTIATION_COMPLETE	(1<<5)	// vitesss page 40 bit 5
#define PHY_MODE_STATUS_LINK_STATUS			(1<<2)	// vitesss page 40 bit 2

#define MICREL_PHY_MMD_ACEES_CONTROL_REG	0xd
#define MICREL_PHY_MMD_ACEES_DATA_REG		0xe

#define SET_MICREL_MMD_OPERATION_MODE_REG(x)	(x << 14)
#define MICREL_MMD_OPERATION_MODE_REG			0
#define MICREL_MMD_OPERATION_MODE_DATA			0x1
#define MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE	0x2
#define MICREL_MMD_OPERATION_MODE_POSTINC_WRITE_ONLY	0x3

#define SET_MICREL_MMD_DEVICE_ADDR(x)		(x)

#define MICREL_MMD_DEVICE_ADDR2			0x2
#define MICREL_RGMII_CTL_SIGNAL_PAD_SKEW_REG	0x4
#define MICREL_RGMII_RX_DATA_PAD_SKEW_REG	0x5
#define MICREL_RGMII_TX_DATA_PAD_SKEW_REG	0x6
#define MICREL_RGMII_CLOCK_PAD_SKEW_REG		0x8

#define MICREL_RGMII_CTL_SIGNAL_PAD_SKEW_REG_VALUE	0x70
#define MICREL_RGMII_RX_DATA_PAD_SKEW_REG_VALUE		0x7777
#define MICREL_RGMII_TX_DATA_PAD_SKEW_REG_VALUE		0x0
#define MICREL_RGMII_CLOCK_PAD_SKEW_REG_VALUE		0x3FC

#define MICREL_PHY_1G_CTR_REG	0x9
#define MICREL_PHY_1G_CTR_MS_MANUAL_CONFIG_ENABLE (1<<12)
#define MICREL_PHY_1G_CTR_MS_MANUAL_CONFIG_VAL	  (1<<11)

//MICREL specific registers
#define MICREL_PHY_INT_CTRL_STS		(0x1B)	//micrel phy interrupt control/status
#define MICREL_PHY_PHYCTRL1			(0x1E)	//micrel phy control 1
#define MICREL_PHY_PHYCTRL2			(0x1F)	//micrel phy control 2

// bit definitions and macros for MICREL_PHY_INT_CTRL_STS
//interrupts enables
#define MICREL_PHY_INT_CTRL_STS_JABBER_INT_EN					(1 << 15)
#define MICREL_PHY_INT_CTRL_STS_RX_ERROR_INT_EN					(1 << 14)
#define MICREL_PHY_INT_CTRL_STS_PAGE_RX_INT_EN					(1 << 13)
#define MICREL_PHY_INT_CTRL_STS_PARALLEL_DETECT_FAULT_INT_EN			(1 << 12)
#define MICREL_PHY_INT_CTRL_STS_LINK_PAR_ACK_INT_EN				(1 << 11)
#define MICREL_PHY_INT_CTRL_STS_LINK_DOWN_INT_EN				(1 << 10)
#define MICREL_PHY_INT_CTRL_STS_REMOTE_FAULT_INT_EN				(1 << 9)
#define MICREL_PHY_INT_CTRL_STS_LINK_UP_INT_EN					(1 << 8)
//interrupt indicators
#define MICREL_PHY_INT_CTRL_STS_JABBER_INT					(1 << 7)
#define MICREL_PHY_INT_CTRL_STS_RX_ERROR_INT					(1 << 6)
#define MICREL_PHY_INT_CTRL_STS_PAGE_RX_INT					(1 << 5)
#define MICREL_PHY_INT_CTRL_STS_PARALLEL_DETECT_FAULT_INT			(1 << 4)
#define MICREL_PHY_INT_CTRL_STS_LINK_PAR_ACK_INT				(1 << 3)
#define MICREL_PHY_INT_CTRL_STS_LINK_DOWN_INT					(1 << 2)
#define MICREL_PHY_INT_CTRL_STS_REMOTE_FAULT_INT				(1 << 1)
#define MICREL_PHY_INT_CTRL_STS_LINK_UP_INT					(1 << 0)

// bit definitions and macros for MICREL_PHYCTRL1
#define MICREL_PHYCTRL1_FLOW_CTRL_CAPABLE		(1 << 9)
#define MICREL_PHYCTRL1_LINK_UP				(1 << 8)
#define MICREL_PHYCTRL1_REVERSED_POLARITY		(1 << 7)
#define MICREL_PHYCTRL1_MDI_X				(1 << 5)
#define MICREL_PHYCTRL1_ENERGY_DETECT			(1 << 4)
#define MICREL_PHYCTRL1_PHY_ISOLATE			(1 << 3)
#define MICREL_PHYCTRL_OP_MODE_MASK			(0x0007)
#define MICREL_PHYCTRL1_10BT_HALF			(0x0001)
#define MICREL_PHYCTRL1_100BT_HALF			(0x0002)
#define MICREL_PHYCTRL1_10BT_FULL			(0x0005)
#define MICREL_PHYCTRL1_100BT_FULL			(0x0006)

// bit definitions and macros for MICREL_PHYCTRL2
#define MICREL_PHYCTRL2_HP_MDI_MDIX_MODE		(1 << 15) //default
#define MICREL_PHYCTRL2_INT_LEVEL_HIGH			(1 << 9)
#define MICREL_PHYCTRL2_INT_LEVEL_LOW			(0 << 9) //default
#define MICREL_PHYCTRL2_ENABLE_JABBER_COUNT		(1 << 8) //default
#define MICREL_PHYCTRL2_RMII_50MHZ_CLKIN		(1 << 7)
#define MICREL_PHYCTRL2_RMII_25MHZ_CLKIN		(0 << 7)

#define SWITCH_PHYINT_MASK		0x1f	// use only bit 0-4
#define SWITCH_88E6320_PHYINT_MASK	0x18	// use only bit 3-4
#define SWITCH_88E6361_PHYINT_MASK	0xf8	// use only bit 3-7
#define SWITCH_88E6161_SERDES_INT_MASK	(3<<11)	// use bit 12:11 and 0:4
#define SWITCH_88E6161_SERDES_INT_BITS	11	// start from bit 11

#define SWITCH_PORT_VLANT_DEFAULT_FID(x)	(x<<12)	// only lower 4 bits
#define SWITCH_PORT_DEFAULT_VLANID_REG		0x07
#define SWITCH_PORT_CTRL_2_REG			0x08
#define SWITCH_PORT_CTRL_2_802_1Q_MODE_MASK	~(3<<10)
#define SWITCH_PORT_CTRL_2_802_1Q_MODE_DISALBED	(0<<10)
#define SWITCH_PORT_CTRL_2_802_1Q_MODE_FALLBACK	(1<<10)
#define SWITCH_PORT_CTRL_2_802_1Q_MODE_CHECK	(2<<10)
#define SWITCH_PORT_CTRL_2_802_1Q_MODE_SECURE	(3<<10)

#define SWITCH_PORT_CTRL_REG_DEFAULT_VAL	0x3f

#define SWITCH_DROP_UNKNOWN_MULTICAST		(1<<2)	// Port control reg bit 2:3 - set 1 as drop unknown multicast
#define SWITCH_DROP_UNKNOWN_MULTICAST_MASK_BIT	(3<<2)	// Port control reg bit 2:3 mask.

#define SWITCH_PORT_EGRESS2_RATE_CONTROL_REG	0xa

#define SWITCH_PORT_EGRESS_RATE_MASK_BITS	(0xcfff)

#define SWITCH_PORT_EGRESS_RATE_WEIGHTED_ROUND_ROBIN		(0<<12)
#define SWITCH_PORT_EGRESS_RATE_STRICT_3_ROUND_ROBIN_REST	(1<<12)
#define SWITCH_PORT_EGRESS_RATE_STRICT_32_ROUND_ROBIN_REST	(2<<12)
#define SWITCH_PORT_EGRESS_RATE_STRICT_FOR_ALL			(3<<12)

#define SWITCH_88E6123_RGMII_HIDDEN_REG		0x1A	// Indirect access reg
#define SWITCH_88E6123_RGMII_RX_DELAY		(1<<4)	// Indirect access reg bit 4 - RGMII receive delay control
#define SWITCH_88E6123_RGMII_TX_DELAY		(1<<3)	// Indirect access reg bit 3 - RGMII transmit delay control

#define SWITCH_88E6123_FORWARDING_ALL_PORTS	0x3f
#define SWITCH_88E6161_FORWARDING_ALL_PORTS	0x3f
#define SWITCH_88E6350R_FORWARDING_ALL_PORTS	0x7f
#define SWITCH_88E6351_FORWARDING_ALL_PORTS	0x7f
#define SWITCH_88E6172_FORWARDING_ALL_PORTS	0x7f
#define SWITCH_88E6176_FORWARDING_ALL_PORTS	0x7f
#define SWITCH_88E6240_FORWARDING_ALL_PORTS	0x7f
#define SWITCH_88E6352_FORWARDING_ALL_PORTS	0x7f
#define SWITCH_88E6320_FORWARDING_ALL_PORTS	0x7f
#define SWITCH_88E6321_FORWARDING_ALL_PORTS	0x7f
#define SWITCH_88E6361_FORWARDING_ALL_PORTS	0x6f9
#define SWITCH_DEFAULT_FORWARDING_ALL_PORTS	0x7f

#define SWITCH_GLOBAL_1_MONITOR_CONTROL_REG	0x1A		// Moniter ctl reg
#define SWITCH_GLOBAL_1_CPU_DEST_PORT_5		(5<<4)		// Moniter ctl reg bit 4:7 - set as port 5
#define SWITCH_GLOBAL_1_CPU_DEST_PORT_6		(6<<4)		// Moniter ctl reg bit 4:7 - set as port 6
#define SWITCH_GLOBAL_1_CPU_DEST_PORT_MASK	(0xF<<4)	// Moniter ctl reg bit 4:7 mask

#define SWITCH_GLOBAL_2_SWITCH_MANAGEMENT_REG	5
#define SWITCH_GLOBAL_2_RSVD2CPU_ENABLE		(1<<3)

// VTU entry load 2,3,5,6,7,8,9
#define SWITCH_GLOBAL_VTU_FID_REG		2
#define SWITCH_GLOBAL_VTU_SID_REG		3
#define SWITCH_GLOBAL_VTU_OPERATION_REG		5
#define SWITCH_GLOBAL_VTU_OP_VTUBUSY		(1<<15)
#define SWITCH_GLOBAL_VTU_OP_FLASH_ALL		(1<<12)	// VTU/STU fluash all
#define SWITCH_GLOBAL_VTU_OP_VTUOP_LOAD		(3<<12)	// VTU load
#define SWITCH_GLOBAL_VTU_OP_VTUOP_GETNEXT	(4<<12)	// VTU get next
#define SWITCH_GLOBAL_VTU_OP_STUOP_LOAD		(5<<12)	// STU load
#define SWITCH_GLOBAL_VTU_OP_STUOP_GETNEXT	(6<<12)	// STU get next
#define SWITCH_GLOBAL_VTU_OP_GET_CLEAR_VIOLATION_DATA	(7<<12)	// get/clear violation data

#define SWITCH_GLOBAL_VTU_VID_REG		6
#define SWITCH_GLOBAL_VTU_VID_VALID		(1<<12)		// Entry is valid bit
#define SWITCH_GLOBAL_VTU_DATA_PORTS_0TO3_REG	7
#define SWITCH_GLOBAL_VTU_DATA_PORTS_4TO6_REG	8

#define SWITCH_GLOBAL_VTU_PORTS_MEMBER_EGRESS_UNMODIFIED	0
#define SWITCH_GLOBAL_VTU_PORTS_MEMBER_EGRESS_UNTAGGED		1
#define SWITCH_GLOBAL_VTU_PORTS_MEMBER_EGRESS_TAGGED		2
#define SWITCH_GLOBAL_VTU_PORTS_NOMEMBER_NOALLOW_ANYFRAMES	3

#define SWITCH_GLOBAL_VTU_DATA_VTU_OPERATION_REG	9

typedef enum
{
	SWITCH_DEFAULT_VLAN_INDEX_PRIMARY = 1,
	SWITCH_DEFAULT_VLAN_INDEX_SECONDARY,
	SWITCH_DEFAULT_VLAN_INDEX_USER_VLAN_2,
	SWITCH_DEFAULT_VLAN_INDEX_USER_VLAN_3
}switch_vlan_default_id;

#define SWITCH_GLOBAL_ATU_FID_REG		1
#define SWITCH_GLOBAL_ATU_OPERATION_REG		0xb
#define SWITCH_GLOBAL_ATU_DATA_REG		0xc
#define SWITCH_GLOBAL_ATU_MAC_REG_1		0xd
#define SWITCH_GLOBAL_ATU_MAC_REG_2		0xe
#define SWITCH_GLOBAL_ATU_MAC_REG_3		0xf

#define SWITCH_GLOBAL_ATU_ENTRY_STATE_PURGE	0x0		// ATU Entry state bits as purge
#define SWITCH_GLOBAL_ATU_ENTRY_STATE_STATIC	0x7		// ATU Entry state bits as static entry for ATU data reg - part 2. pg 220. reg index - 0xc
#define SWITCH_GLOBAL_ATU_FROM_PORT		(0x5<<4)	// ATU data reg bit 4:11 from port 5.

#define SWITCH_GLOBAL_ATUop_MASK_BIT		0x8fff		// ATU operation reg 0xB
#define SWITCH_GLOBAL_ATUop_START_BUSY_BIT	0x8000		// ATU busy check bit 15
#define SWITCH_GLOBAL_ATU_ENTRY_LOAD_PURGE	0x3000		// ATU operation command ATU entry load purge.

#define SWITCH_GLOBAL_IP_PRI_MAPPING_REG0	0x10
#define SWITCH_GLOBAL_IP_PRI_MAPPING_REG1	0x11
#define SWITCH_GLOBAL_IP_PRI_MAPPING_REG2	0x12
#define SWITCH_GLOBAL_IP_PRI_MAPPING_REG3	0x13
#define SWITCH_GLOBAL_IP_PRI_MAPPING_REG4	0x14
#define SWITCH_GLOBAL_IP_PRI_MAPPING_REG5	0x15
#define SWITCH_GLOBAL_IP_PRI_MAPPING_REG6	0x16
#define SWITCH_GLOBAL_IP_PRI_MAPPING_REG7	0x17

#define SWITCH_GLOBAL_IP_MAPPING_TABLE_REG	0x19

#define IP_MAPPING_TABLE_WRITE			0x8000		// Write enable.

#define IP_FRAME_PRIORITY_HIGHEST		(0x6<<4)	// IP frame priority
#define IP_FRAME_PRIORITY_MEDIUM		(0x4<<4)
#define IP_FRAME_PRIORITY_LOW			(0x2<<4)
#define IP_FRAME_PRIORITY_DEFAULT		(0x0<<4)

#define IP_QUEUE_PRIORITY_HIGHEST		0x3		// queue priority
#define IP_QUEUE_PRIORITY_MEDIUM		0x2
#define IP_QUEUE_PRIORITY_LOW			0x1
#define IP_QUEUE_PRIORITY_DEFAULT		0x0

#define PTP_EVENT_PRIORITY			(IP_QUEUE_PRIORITY_HIGHEST<<12)	// For 88E123 & 88E6161. Setting mapping register 0x11 with highest IP Priority for diffserv 0x38

#define DSCP_TIME_CRITICAL_PTP_EVENTS		(0x38<<8)	// diffserve values for IP mapping table. part 2. pg 227
#define DSCP_AUDIO_PTP				(0x2e<<8)
#define DSCP_RESERVED				(0x8<<8)


#define USE_IP_FRAME_PRIORITY			(1<<14)		// use ip priority providing.

#define DSCP_HIGH				( USE_IP_FRAME_PRIORITY | DSCP_TIME_CRITICAL_PTP_EVENTS | IP_QUEUE_PRIORITY_HIGHEST | IP_FRAME_PRIORITY_HIGHEST )	// IP FRAME/QUEUE Priority highest
#define DSCP_MEDUIM				( USE_IP_FRAME_PRIORITY | DSCP_AUDIO_PTP | IP_QUEUE_PRIORITY_MEDIUM | IP_FRAME_PRIORITY_MEDIUM )			// IP FRAME/QUEUE Priority medium
#define DSCP_LOW				( USE_IP_FRAME_PRIORITY | DSCP_RESERVED | IP_QUEUE_PRIORITY_LOW | IP_FRAME_PRIORITY_LOW )				// IP FRAME/QUEUE Priority low
#define DSCP_DEFAULT				0x0

// All about switch initialization based on switch mode
#define SWITCH_GLOBAL_STATUS_REG			0

#define SWITCH_MODE_BITS				(0x3<<12)	// Global register 0 : offset 0:bits 12-13
#define SWITCH_6350_6351_6350R_TEST_MODE		0		// 00 - test mode
#define SWITCH_6350_6351_6350R_RESERVED_MODE		1		// 01 - reserved
#define SWITCH_6350_6351_6350R_DEFAULT_MODE		2		// 10 - unmanaged/forward mode (default)
#define SWITCH_6350_6351_6350R_CPU_ATTACHED_MODE	3		// 11 - CPU attached mode/Disabled mode

#define SWITCH_6123_6161_CPU_ATTACHED_MODE		0		// 00 - CPU attached mode/Disabled mode
#define SWITCH_6123_6161_RESERVED_MODE			1		// 01 - reserved
#define SWITCH_6123_6161_TEST_MODE			2		// 10 - test mode
#define SWITCH_6123_6161_EEPROM_ATTACHED_MODE		3		// 11 - eeprom attached mode (default)

#define SWITCH_SCRATCH_MISC_REG			0x1A
#define SWITCH_SCRATCH_MISC_CONFIG1		(0x71<<8)	// Config1 pointer bit 8:14
#define SWITCH_SCRATCH_MISC_UPDATE_BUSY_FLAG	(1<<15)
#define SWITCH_SCRATCH_MISC_REG_MASK		0xff		// 8bit data

#define SWITCH_88E6320SERIES_88E6352SERIES_CPU_ATTACHED		(1<<2)	// Bit 2 0 : cpu attached, 1: No cpu attached

#define SWITCH_6161_SERDES_PHY_PORT4		0xc
#define ETH_SWITCH_VLAN_NONE 0

/*
 * SMI phy EEE configuration.
 */
#define SWITCH_SMI_PHY_PAGE_ADDR		22
#define SWITCH_SMI_PHY_MMD_ACCESS_CTL_REG	13
#define SWITCH_SMI_PHY_MMD_ACCESS_ADDR_DATA_REG	14

#define SWITCH_SMI_PHY_MMD_ACCESS_FUNCTION_ADDR	(0<<14)	// Address
#define SWITCH_SMI_PHY_MMD_ACCESS_FUNCTION_DATA	(1<<14)	// DATA ,no port increment.

#define SWITCH_SMI_PHY_EEE_ADV_DEV	7
#define SWITCH_SMI_PHY_EEE_ADV_REG	0x3C	// bit 1,2 should be 0. switch 6352(p.g 504)
#define SWITCH_88E6361_SMI_PHY_EEE_ADV_REG	0x68	// Table 86: EEE advertisement register

#define SWITCH_SMI_PHY_PAGE_7		7
#define SWITCH_SMI_PHY_PAGE_18		18

#define SWITCH_SMI_PHY_EEE_CTL_REG_0	0	// page 18, register 0.
#define SWITCH_SMI_PHY_EEE_MASTER_MODE	(1<<0)	// page 18, register 0.bit 0 - EEE master mode.

/* SMI phy command/data register
 * 88e123 P2 pg. 237
 * 88e8350 P2 pg. 256
 * 88e8352 P2 pg. 317
 * */
#define SWITCH_SMI_PHY_CMD_REG	0x18
#define SWITCH_SMI_PHY_DATA_REG	0x19

#define SWITCH_SMI_PHY_BUSY_BIT		(1<<15)
#define SWITCH_SMI_PHY_START_OP		(1<<15)
#define SWITCH_SMI_PHY_CMD_CLAUSE22	(1<<12)
#define SWITCH_SMI_PHY_CMD_OP_READ	(0x2<<10)
#define SWITCH_SMI_PHY_CMD_OP_WRITE	(0x1<<10)
#define SWITCH_SMI_PHY_CMD_CLAUSE45	(0<<12)
#define SWITCH_SMI_PHY_CMD_OP_WRITE_ADDR_CLAUSE45	(0x0<<10)
#define SWITCH_SMI_PHY_CMD_OP_READ_CLAUSE45		(0x3<<10)
#define SWITCH_SMI_PHY_CMD_DEV_ADDR(x)	(x<<5)
#define SWITCH_SMI_PHY_CMD_REG_ADDR(x)	(x<<0)

enum
{
	SMI_PHY_WRITE_OP = 1,
	SMI_PHY_READ_OP
};

/*
 * SMI phy indirect commands/data access flags
 */
#define SMI_PHY_COMMAND_REG		0
#define SMI_PHY_DATA_REG		1
#define SMI_PHY_MODE_45SMI_FRAME	(0<<12)
#define SMI_PHY_MODE_22SMI_FRAME	(1<<12)
#define SMI_PHY_WRITE_REG		(0x1<<10)
#define SMI_PHY_READ_REG		(0x2<<10)
#define SMI_PHY_SET_DEV_ADDR(x)		(x<<5)
#define SMI_PHY_SET_REG_ADDR(x)		(x<<0)

#define SMI_PHY_BUSY			(1<<15)
#define SMI_PHY_CMD_OP			(1<<15)
#define SMI_PHY_SCAN_REG_ADDR		0

// SERDES registers starting device offset
#define	SWITH_6352_SERDES_BASE_REG	0xF	// same for 6176 & 6320
#define	SWITH_6161_SERDES_BASE_REG	0xC
#define	SWITH_6320_SERDES_BASE_REG	0xC
#define	SWITH_6361_SERDES_BASE_REG	0x9

#define	SWITCH_SERDES_CONTROL_REG	0x0
#define	SWITCH_SERDES_CONTROL_POWER_DOWN (1<<11)// POWER DOWN BITS - should be one (normal operation)

#define SWITCH_PHY_PORT_BASE_ADDR		0x00
#define SWITCH_PHY_INT_ENABLE_REG		18        //Part3 Pg70
#define SWITCH_PHY_INT_STATUS_REG		19        //Part3 Pg70
#define SWITCH_PHY_LINK_STATUS_CHANGE 		(1<<10)	  //Part3 Pg70 - PHY link status change interrupt enable
#define SWITCH_PHY_PAGE_ADDR_REG		22        //Part3 Pg73
#define SWITCH_PHY_LED_FUNCTION_CTRL_REG	16        //Part3 Pg77
#define SWITCH_PHY_LED_POLARITY_CTRL_REG	17        //Part3 Pg79

#define PHY_KSZ9031_CONTROL_REGISTER	0x1f
#define PHY_KSZ9031_SPEED_1G		(1<<6)
#define PHY_KSZ9031_SPEED_100M		(1<<5)
#define PHY_KSZ9031_DUPLEX_STATUS	(1<<3)

#define PHY_KSZ8051_CONTROL1_REGISTER		0x1e
#define PHY_KSZ8051_OPERATION_MODE_MASK		(0x7)	// bit 0-2
#define PHY_KSZ8051_SPEED_10M_HALF_DUPLEX	(0x1)
#define PHY_KSZ8051_SPEED_100M_HALF_DUPLEX	(0x2)
#define PHY_KSZ8051_SPEED_10M_FULL_DUPLEX	(0x5)
#define PHY_KSZ8051_SPEED_100M_FULL_DUPLEX	(0x6)

#define PHY_INT_MASK_REG		25	//VC8601 Pg 51
#define PHY_INT_LINK_STATUS_CHANGE	(1<<13)	//VC8601 Pg 51
#define PHY_MDINT_STATUS_ENABLE		(1<<15)	//VC8601 Pg 51
#define PHY_INT_STATUS_REG		26	//VC8601 Pg 52 - clear interrupt after reading this register.
#define PHY_INT_STATUS_BIT		(1<<15)	//VC8601 Pg 52

#define PHY_88E1510_INT_ENABLE_REG		18	// 88E1510 Pg 130
#define PHY_88E1510_INT_LINK_CHANGE		(1<<10)	// 88E1510 Pg 130
#define PHY_88E1510_INT_STATUS_REG		19	// 88E1510 Pg 131 - clear interrupt after reading this register.
#define PHY_88E1510_INT_STATUS_LINK_CHANGE	(1<<10)	// 88E1510 Pg 131
#define PHY_88E1510_LED_FUNC_CTL_REG		16	// 88E1510 Pg 160
#define PHY_88E1510_LED_FUNC_CTL_LED0_1G_ON_ELSE_OFF 		(7<<0)	// 88E1510 Pg 160
#define PHY_88E1510_LED_FUNC_CTL_LED1_LINK_ON_BLINK_ACTIVITY	(1<<4)	// 88E1510 Pg 159
#define PHY_88E1510_LED_FUNC_CTL_LEDS_MASK			0xff00	// 88E1510 Pg 159
#define PHY_88E1510_LED_TIMER_CTRL_REG				18	// 88E1510 Pg 131
#define PHY_88E1510_LED_TIMER_CTRL_REG_INTERRUPT_ENABLE (1<<7)		// 88E1510 Pg 131

#define PHY_RGMII_RX_CLOCK_DELAY_ENABLE			(1<<15)
#define PHY_RGMII_TX_CLOCK_DELAY_ENABLE			(1<<8)

#define PHY_MICREL_INT_CTL_REG		0x1B	//MICREL Pg 50 - read on clear for interrupt status.
#define PHY_MICREL_INT_LINK_UP		(1<<0)	//MICREL Pg 50
#define PHY_MICREL_INT_LINK_DOWN	(1<<2)	//MICREL Pg 50
#define PHY_MICREL_INT_LINK_UP_ENABLE	(1<<8)	//MICREL Pg 50
#define PHY_MICREL_INT_LINK_DOWN_ENABLE	(1<<10)	//MICREL Pg 49

// SERDES registers starting device offset
#define	SWITH_6352_SERDES_BASE_REG	0xF	// same for 6176 & 6320
#define	SWITH_6161_SERDES_BASE_REG	0xC
#define	SWITH_6320_SERDES_BASE_REG	0xC
#define	SWITH_6361_SERDES_BASE_REG	0x9

// SERDES interrupt enable/clear register
#define SWITCH_SERDES_FIBER_INT_ENABLE	0x12
#define SWITCH_SERDES_FIBER_INT_STATUS	0x13

// Global register 2 interrupt enable starting bit
#define SWITCH_SERDES_6161_START_BIT	11	// 11:12
#define SWITCH_SERDES_6352_START_BIT	11
#define SWITCH_SERDES_6320_START_BIT	0
#define SWITCH_SERDES_6361_START_BIT	9

// Global register 2 interrupt source register.
#define SWITCH_SERDES_6161_INT_MASK	(1<<11)		// bits 11:12
#define SWITCH_SERDES_6352_INT_MASK	(1<<11)		// bits 11
#define SWITCH_SERDES_6320_INT_MASK	(0x3<<0)	// bits 0:1
#define SWITCH_SERDES_6361_INT_MASK	(0x3<<9)	// bits 9:10


#define SWITCH_GLOBAL_REGISTER_GROUP_1  0x1b
#define SWITCH_GLOBAL1_STATUS_REG	0x0	//Part2, Pg207
#define SWITCH_GLOBAL1_CONTROL_REG	0x4	//Part2, Pg211
#define SWITCH_DEV_INT_ENABLE		(1<<7)	//Part2, Pg211 - Device interrupt enable including phy interrupt.
#define SWITCH_EEPROM_INT_ENABLE	(1<<0)	//Part2, Pg211 - EEPROM done interrupt.

#define SWITCH_GLOBAL_REGISTER_GROUP_2  0x1c
#define SWITCH_GLOBAL2_INT_SRC_REG	0x0	//Part2, Pg237
#define SWITCH_GLOBAL2_INT_MASK_REG	0x1	//Part2, Pg237

#define SWITCH_EXTERNAL_PHY_PORTS_MASK	0xff0fffff	// except port 5

#define SWITCH_PORT_SPEED(x)   ((x>>8)&0x3)
#define SWITCH_PORT_DUPLEX     (1<<10)
#define SWITCH_PORT_LINK_UP    (1<<11)

#define SWITCH_SMI_DEVICE_INTERFACE_BOUNDARY	0x10

#define SWITCH_88E6320_START_SWITCH_PORT	0x10
#define SWITCH_88E6320_GLOBAL_REG1_OFFSET	0x1b
#define SWITCH_88E6320_GLOBAL_REG2_OFFSET	0x1c
#define SWITCH_88E6320_GLOBAL_REG3_OFFSET	0x1d

#define PHY_PORT_AUX_REG    0x1C // Auxilary Control and Status
#define PHY_PORT_DUPLEX     (1<<5)
#define PHY_PORT_SPEED(x)   ((x>>3)&0x3)

#define PHY_EXTENDED_PAGE_ACCESS_REG    0x1f // switch between main regs and extended regs groups.
#define PHY_ACCESS_MAIN_REG_SPACE       0
#define PHY_ACCESS_EXTENDED_REG_SPACE   1

#define MII_BMSR            0x01        /* Basic mode status register  */
#define BMSR_LSTATUS            0x0004  /* Link status                 */

#define GET_PHY_SPEED(x)	((PHY_PORT_SPEED(x)) == PORT_SPEED_1000 ? 1000 :	\
				(PHY_PORT_SPEED(x)== PORT_SPEED_100) ? 100 : 10 )
#define GET_PHY_DUPLEX(x)	((x) & PHY_PORT_DUPLEX ? FULL_DUPLEX : HALF_DUPLEX)
#define GET_PHY_LINK_UP(x)	(((x) & BMSR_LSTATUS) != 0)

#define SWTICH_PHY_PORT_1G_CTL_REG			0x9
#define SWTICH_PHY_PORT_1G_ADVERT_MASK			0xFCFF
#define SWTICH_PHY_PORT_1G_ADVERT_DISABLE(reg)		(reg &= 0xFCFF)	// disable 1G full/half duplex advertisement

#define SWTICH_PHY_PORT_COPPER_SPECIFIC_CTL_REG3	0x1A
#define SWTICH_PHY_PORT_COPPER_SPECIFIC_CTL_DISABLE_1G(reg)	(reg &= 0xBFFF)

#define EGRESS_TOTAL_QUEUE_COUNTER	 	 (8<<12)
#define INGRESS_TOTAL_RESERVED_QUEUE_COUNTER	 (9<<12)

#define STATS_OPERATION_REG	0x1D	// global reg 0x1b, offset 29
#define STATS_COUNT_BYTES_23	0x1E	// Stats counter_reg bit 16:32
#define STATS_COUNT_BYTES_01	0x1F	// Stats counter_reg bit 0:15

#define STATS_BUSY				(1<<15)		// Run this operation.
#define STATS_OP_CAPTURE_ALL_COUNTERS_PORT	(0x5<<12)	// Capture all counter for a port
#define STATS_OP_FLUSH_ALL_COUNTERS_ALL_PORT	(0x1<<12)	// Flush all counter for a port
#define STATS_OP_FLUSH_ALL_COUNTERS_PORT	(0x2<<12)	// Flush all counter for a port
#define STATS_OP_READ_CAPTURED_COUNTERS_PORT	(0x4<<12)	// Read all counter for a port
#define STATS_OP_PORT_5				(0x6<<5)	// Read captured port 5 counters
#define STATS_PTR_PORT_5			(0x5)		// Read captured port 5 counters
#define STATS_PTR_GOOD_IN_OCTETS_LO		(0x0)		// Read In GoodOctets Low
#define STATS_PTR_GOOD_IN_OCTETS_HI		(0x1)		// Read In GoodOctets High
#define STATS_PTR_IN_BAD_OCTETS			(0x2)		// Read in bad octets
#define STATS_PTR_GOOD_OUT_OCTETS_LO		(0xE)		// Read Out GoodOctets Low
#define STATS_PTR_GOOD_OUT_OCTETS_HI		(0xF)		// Read Out GoodOctets High
#define STATS_PTR_IN_RX_ERR			(0x1C)		// Read In rx error
#define STATS_PTR_IN_FCS_ERR			(0x1D)		// Read In FCS error
#define STATS_PTR_OUT_FCS_ERR			(0x3)		// Read Out FCS error
#define STATS_PTR_IN_UNDER_SIZE                 (0x18)           // Read Out FCS error
#define STATS_PTR_IN_FRAGMENTS                  (0x19)           // Read Out FCS error
#define STATS_PTR_IN_OVERSIZE                   (0x1A)           // Read Out FCS error

/*
 * For marvel 88E1510 status
 */
#define PHY_MARVEL_88E1510_PORT_SPEED(x) ((x>>14)&0x3)
#define PHY_MARVEL_88E1510_PORT_DUPLEX	(1<<13)
#define PHY_MARVEL_88E1510_GET_PHY_SPEED(x)	((PHY_MARVEL_88E1510_PORT_SPEED(x)) == PORT_SPEED_1000 ? 1000 :	\
					(PHY_MARVEL_88E1510_PORT_SPEED(x)== PORT_SPEED_100) ? 100 : 10 )
#define PHY_MARVEL_88E1510_GET_PHY_DUPLEX(x)	((x) & PHY_MARVEL_88E1510_PORT_DUPLEX ? FULL_DUPLEX : HALF_DUPLEX)
#define PHY_MARVEL_88E1510_GET_PHY_LINK_UP(x)	(((x) & BMSR_LSTATUS) != 0)

#define PHY_MARVEL_88E1510_COPPER_SPECIFIC_STATUS_REG	17
#define PHY_MARVEL_88E1510_COPPER_SPECIFIC_STATUS_COPPER_LINK	(1<<10)	// copper link in real time
/*
 * LED control registers 0x16 for each switch ports.
 */
#define SWITCH_LED_CTR_REG			0x16
#define SWITCH_LED_CTR_UPDATE			(1<<15)
#define SWITCH_LED_CTR_POINTER(x)		(x<<12)
#define SWITCH_LED_CTR_LED0N1			0		// LED 0&1 control
#define SWITCH_LED_CTR_DATA_MASK(x)		(x&0x7FF)	// data is bit 0:10
#define SWITCH_LED_CTR_FORCE_OFF		(0xEE)		// Force off for led 0 & 1
#define SWITCH_LED_CTR_FORCE_ON			(0xFF)		// Force on for led 0 & 1
#define SWITCH_LED_CTR_LINK_ON_ONLY_1G_ON	(0x38)		// link on (no activity) and 1G on & other speed off
#define SWITCH_LED_CTL_DEFAULT			0x33
#define SWITCH_88E6361_SERDES_LED_CTL_DEFAULT			0xe1

// VSC8541 specific registers
#define PHY_VSC8541_EXTENDED_PAGE_ACCESS_REG	31
#define PHY_VSC8541_MAIN_REG_PAGE				0x0000
#define PHY_VSC8541_EXTENDED_REG_PAGE_1			0x0001
#define PHY_VSC8541_EXTENDED_REG_PAGE_2			0x0002
#define PHY_VSC8541_EXTENDED_REG_PAGE_GPIO		0x0010

#define PHY_VSC8541_INTERRUPT_MASK_REG				25	
#define PHY_VSC8541_INTERRUPT_STATUS_REG			26
#define PHY_VSC8541_INTERRUPT_MASK_EXTENDED_REG		28
#define PHY_VSC8541_INTERRUPT_STATUS_EXTENDED_REG	29
#define PHY_VSC8541_MDINT_ENABLE					(1<<15)
#define PHY_VSC8541_LINK_STATE_CHANGE				(1<<13)
#define PHY_VSC8541_LED_MODE_SELECT_REG				29
#define PHY_VSC8541_LED_BEHAVIOUR_REG				30
#define PHY_VSC8541_LED1_COMBINE_DISABLE			(1<<1)

#define PHY_VSC8541_RGMII_CONTROL_REG				20

// MV88E6361 specific registers
// MV88E6361 SERDES interrupt enable/clear register for 88E6361
#define SWITCH_88E6361_SERDES_DEVICE_ADDR	4
#define SWITCH_88E6361_SERDES_FIBER_INT_ENABLE	0xa001
#define SWITCH_88E6361_SERDES_FIBER_INT_STATUS	0xa002
#define SWITCH_88E6361_SERDES_LINK_STATUS_UP 		(1<<10)	  // Serdes link status up interrupt enable
#define SWITCH_88E6361_SERDES_LINK_STATUS_DOWN 		(1<<9)	  // Serdes link status down interrupt enable
#define SWITCH_88E6361_SERDES_LINK_STATUS_CHANGE 	(3<<9)	  // Serdes link status change interrupt enable
#define SWITCH_88E6361_SERDES_PORT_OPERATION_CONFIG_REG		0xf002
#define SWITCH_88E6361_SERDES_CONTROL_REG		0x2000

// MV88E6361 Port Queue Control register
#define SWITCH_88E6361_PORT_QUEUE_CONTROL_REG	0x1c
#define SWITCH_88E6361_PORT_QUEUE_CONTROL_UPDATE	(1<<15)
#define SWITCH_88E6361_PORT_QUEUE_CONTROL_POINTER_MASK	(0x7f<<8)
#define SWITCH_88E6361_PORT_QUEUE_CONTROL_POINTER_PORT_SCHEDULE	(0<<8)
#define SWITCH_88E6361_PORT_QUEUE_CONTROL_DATA_MASK	0xff
#define SWITCH_88E6361_PORT_QUEUE_CONTROL_DATA_STRICT_MASK	0x7
#define SWITCH_88E6361_PORT_QUEUE_CONTROL_DATA_STRICT_FOR_ALL	0x7




#if defined(SWITCH_LIB_UBOOT) \
	|| defined(SWITCH_LIB_LINUX)

#define AUD_SYD_SMI_DATA_BASEADDR	(*(volatile uint32_t *)(AUD_SYD_BASEADDR + 0x400))
#define AUD_SYD_SMI_ADDR		(*(volatile uint32_t *)(AUD_SYD_BASEADDR + 0x480))
#define AUD_SYD_SMI_CONTROL		(*(volatile uint32_t *)(AUD_SYD_BASEADDR + 0x484))

#define AUD_SYD_SMI_CTRL_READ		0
#define AUD_SYD_SMI_CTRL_WRITE		2

#define AUD_SYD_SMI_INTR		(*(volatile uint32_t *)(AUD_SYD_BASEADDR + 0x500))
#define AUD_SYD_SMI_INT_ENABLE          (1<<0)
#define AUD_SYD_SMI_INTR_RDY		(1<<3)

#endif

#ifdef __cplusplus
}	// extern C
#endif
#endif /* SWITCH_LIB_REG_H_ */
