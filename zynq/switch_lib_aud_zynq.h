#ifndef SWITCH_LIB_AUD_ZYNQ_H_
#define SWITCH_LIB_AUD_ZYNQ_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>

#ifdef __AUD_ZYNQ_KERNEL__
// If building in kernel, then use copied one.
enum
{
	ETH_SWITCH_MODE_SWITCH			= (1 << 0),
	ETH_SWITCH_MODE_REDUNDANT		= (1 << 1),
	ETH_SWITCH_MODE_VLANS			= (1 << 2)
};

enum
{
	ETH_SWITCH_EXTERNAL_PHY_MODE	= (1 << 0)
};

enum
{
	ETH_SWITCH_VLAN_PRI				= (1 <<0),
	ETH_SWITCH_VLAN_SEC				= (1 <<1),
	ETH_SWITCH_VLAN_2				= (1 <<2),
	ETH_SWITCH_VLAN_3				= (1 <<3),
};

#define MC_MAX_MULTICAST_ADDRESS_LIST	16

// This is used in the 'control_port' field below.
enum
{
	ETH_SWITCH_CONTROL_PORT_MCAST_FILTER  = (1<<0),
	ETH_SWITCH_CONTROL_PORT_LINK_CALCULATION = (1<<1),
	ETH_SWITCH_CONTROL_PORT_MCAST_FILTER_LINK_CALCULATION = (1<<2)
};
#else
#include "module_config.h"
#endif

// MDIO interface function for akashi platform
#ifdef __AUD_ZYNQ_KERNEL__
extern volatile uint8_t * aud_syd_virtual_base_addr;
#define AUD_SYD_BASEADDR (aud_syd_virtual_base_addr)
#define AUD_SYD_HIGHADDR (aud_syd_virtual_base_addr + 0x7FFFF)
#else
#define AUD_SYD_BASEADDR 0x60000000
#endif

/************************************************************
******** Support list of network adapter IDs ****************
******** the order of this enum must not changed ************
******** Just add the end of each supported phy/switch ******
*************************************************************/
typedef enum
{
	NETWORK_CHIPSET_NONE,
	PHY_KSZ9031,
	PHY_MARVEL_88E1510,
	PHY_VSC8541,
	PHY_KSZ9131,
	PHY_UNKNOWN,
	SWITCH_88E6123,
	SWITCH_88E6161,
	SWITCH_88E6350R,
	SWITCH_88E6351,
	SWITCH_88E6172,
	SWITCH_88E6176,
	SWITCH_88E6352,
	SWITCH_88E6361,
	SWITCH_88E6320,
	SWITCH_88E6321,
	SWITCH_UNKNOWN,
	SWITCH_DEFAULT,
	ADAPTER_ENUM_MAX
} adapter_enum_t;

/* Support list of network adapter IDs */
#define PHY_MICREL_KSZ9031_PRODUCT_ID		0x1620
#define PHY_MICREL_KSZ9131_PRODUCT_ID		0x1640
#define PHY_MARVEL_88E1510_PRODUCT_ID		0x0dd0
#define PHY_MICROCHIP_VSC8541_PRODUCT_ID	0x0770

#define SWITCH_88E123_PRODUCT_ID		0x1210
#define SWITCH_88E6161_PRODUCT_ID		0x1610
#define SWITCH_88E6350R_PRODUCT_ID		0x3710
#define SWITCH_88E6351_PRODUCT_ID		0x3750
#define SWITCH_88E6172_PRODUCT_ID		0x1720
#define SWITCH_88E6176_PRODUCT_ID		0x1760
#define SWITCH_88E6352_PRODUCT_ID		0x3520

#define	SWITCH_88E6320_PRODUCT_ID		0x1150
#define	SWITCH_88E6321_PRODUCT_ID		0x3100
#define	SWITCH_88E6361_PRODUCT_ID		0x2610

#define DANTE_DEFAULT_MB_PORTS_MASK	0xFF0FFFFF
#define DANTE_DEFAULT_MB_PORTS_VLAN	0x00100000	// enable port 5 as default

#define DANTE_88E6361_DEFAULT_PORTS_MASK	0xFFFFFFF0
#define DANTE_88E6361_DEFAULT_PORTS_VLAN	0x00000001	// enable port 0 as default
/*
 * Audinate CPU port 5 bits mask :: Each bit represents each port starting from bit 0 (port 0)
 */
#define DANTE_OTHER_PLATFORM_CPU_PORT_MASK				~(0x20)
#define SWITCH_DANTE_CPU_PORTS_MASK 					DANTE_OTHER_PLATFORM_CPU_PORT_MASK


/*******************************************
 ******** structure definition *************
 *******************************************/
typedef enum
{
	PHY_PRIMARY_PORT,
	PHY_SECONDARY_PORT
} phy_ports_t;

typedef enum switch_mode_s
{
	CPU_ATTACHED_MODE,
	TEST_MODE
} switch_mode_t;

typedef enum switch_port_cmode_s
{
	SWITCH_PORT_CMODE_DISABLED,
	SWITCH_PORT_CMODE_ENABLED
} switch_port_cmode_t;

typedef enum
{
	PORT_DISABLE,
	PORT_ENABLE
} switch_port_ctl_s;

typedef enum
{
	PHY_ISOLATE_ENABLE,
	PHY_ISOLATE_DISABLE
} phy_isolate_ctl_s;

typedef enum
{
	ATU_ENTRY_LOAD,
	ATU_ENTRY_PURGE,
	ATU_ENTRY_FLUSH_ALL,
} switch_atu_ctl_t;

typedef enum
{
	ADAPTER_SWITCH = 1,
	ADAPTER_PHY
} adapter_type_t;

typedef enum
{
	SWITCH_LED_CONTROL_FORCE_OFF,
	SWITCH_LED_CONTROL_FORCE_ON,
	SWITCH_LED_CONTROL_LINK_ONLY,
	SWITCH_LED_CONTROL_DEFAULT
}switch_port_led_action;

typedef enum
{
	SWITCH_LED_CTL_REG_READ,
	SWITCH_LED_CTL_REG_WRITE,
	SWITCH_LED_CTL_REG_NONE,
}switch_port_led_reg_access;

typedef struct switch_ports_info_s
{
	uint8_t num_switch_ports;
	uint8_t num_phy_ports;			// number of internal phy ports
	uint8_t num_phy_port_start;		// phy start index number
	uint8_t num_serdes_ports;		// number of internal serdes ports
	uint8_t num_serdes_port_start;		// serdes start index number
	uint8_t base_reg_addr_offset;		// base reg addr offset - this is only for 6220 series.
} switch_ports_info_t;

typedef void (*phy_smi_read_t)(uint8_t mii_addr, uint8_t reg_addr, uint16_t *reg_data);
typedef void (*phy_smi_write_t)(uint8_t mii_addr, uint8_t reg_addr, uint16_t reg_data);
typedef int (*phy_reg_read_clause45_t)(uint8_t dev_addr, uint8_t phy_addr, uint16_t reg_addr, uint16_t *reg_data);
typedef int (*phy_reg_write_clause45_t)(uint8_t dev_addr, uint8_t phy_addr, uint16_t reg_addr, uint16_t reg_data);

typedef struct network_adapter_s
{
	adapter_type_t adapter_type;
	adapter_enum_t adapter_name;
	uint8_t phy_redundant;			// flag for phy redundant?
	uint16_t product_identifier;
	uint16_t phy_address[2];		// In case PHY redundant or multi switch, it will be 2.
	switch_ports_info_t switch_ports_info;
	uint8_t multi_chipset_addr;		// phy address in multi network chipset environment
	phy_smi_read_t phy_smi_read_fn;
	phy_smi_write_t phy_smi_write_fn;
	phy_reg_read_clause45_t phy_reg_read_clause45_fn;
	phy_reg_write_clause45_t phy_reg_write_clause45_fn;
	uint8_t cpu_port_num;
} network_adapter_t;

/*********************************************************
 ******** Zynq specific function prototypes **************
 *********************************************************/
// Filling network adapter information
void fill_network_adapter_info( network_adapter_t *ptr_adapter, adapter_enum_t adapter_enum, uint16_t phy_addresses[2]);

/*******************************************
 ***********  print function  **************
 *******************************************/
#undef SW_DEBUG_ON
// #define SW_DEBUG_ON

#ifdef SW_DEBUG_ON
#ifdef __AUD_ZYNQ_KERNEL__
// This is for linux debug purpose
#define sw_trace_print(...)	printk(__VA_ARGS__);
#else
// This is for uboot & ultimo & other platforms
#define sw_trace_print(...)	printf(__VA_ARGS__);
#endif
#else
#define sw_trace_print(...)
#endif

#endif
