
/***********************************************************
******** Audinate Zynq kernel specific header files ********
************************************************************/
#ifdef __AUD_ZYNQ_KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
#include <stdarg.h>
#else
#include <linux/stdarg.h>
#endif

#include "switch_lib_reg.h"

// build with linux driver
extern network_adapter_t network_adapter_info;
#else
#include "switch_lib.h"
#include "switch_lib_reg.h"
#include <stdarg.h>

#if defined(SWITCH_LIB_LINUX) \
	|| defined(HAS_STRING_LIB)
#include <string.h>
#endif

#ifdef LINUX_VERSION_4_10
#ifdef SWITCH_LIB_DANTE_APP
	// build with skye application
	network_adapter_t network_adapter_info;
#else
	// build with linux driver
	extern network_adapter_t network_adapter_info;
#endif
#else
#if !(defined LCCK60)
// global structure for switch/phy
network_adapter_t network_adapter_info;
#endif
#endif
#endif

void sl_lib_init(void)
{
	memset(sl_get_chipset_info(), 0, sizeof(network_adapter_t));
}

void sl_lib_smi_handler_register(phy_smi_read_t read, phy_smi_write_t write)
{
	sl_get_chipset_info()->phy_smi_read_fn = read;
	sl_get_chipset_info()->phy_smi_write_fn = write;
}

// Get adapter information
#if !(defined LCCK60)
network_adapter_t * sl_get_chipset_info(void)
{
	return (network_adapter_t *)&network_adapter_info;
}
#endif

// get phy addr
uint8_t sl_get_phy_addr(phy_ports_t phy_port)
{
	if(phy_port == PHY_PRIMARY_PORT)
		return sl_get_chipset_info()->phy_address[0];
	else return sl_get_chipset_info()->phy_address[1];
}

// switch busy check
int sl_switch_busy_check(void)
{
	return smi_phy_cmd_busy();
}

/*
 * Waiting for switch init ready
 */
int sl_switch_init_ready(void)
{
	uint16_t reg;
	int counter = 0;

	// BT8 make sure that the init_ready bit is set which means the switch is ready to accept configuration.
	while (counter < 1000)
	{
		// Get global1 status register
		sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_STATUS_REG, &reg);

		if((reg & SWITCH_GLOCAL1_STATUS_INIT_READY))
		{
			break;
		}

		sl_nop_delay_loop(10000);
		counter++;
	}

	if (counter >= 1000)
	{
		// Mostlikely this is a clock problem, the switch doesn't get a proper clock so it won't start in time.
		sw_trace_print("switch init still not set, continuing anyway\n\r");
	}

	return SWITCH_SUCCESS;
}

/*
 * Check PPU enabled for indirect phy address access
 */
void sl_switch_ppu_enable(void)
{
	uint16_t global1_reg;

	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_CTLR_REG, &global1_reg);

	if(!(global1_reg & SWITCH_GLOBAL1_STATUS_PPU_STATE))
	{
		sw_trace_print("PPU disabled, enabling\n");
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_CTLR_REG, (global1_reg | SWITCH_GLOBAL1_STATUS_PPU_STATE));
	}
}

/******* switch/phy port & speed & status monitoring apis*********/

// get switch port status register
int sl_get_switch_port_status(int port_index, uint16_t *status_reg)
{
	sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_STAT_REG, status_reg);
	return SWITCH_SUCCESS;
}

/*
 * get phy status register
 * Returning phy basic status register vlaue
 * @phy_port : primary or secondary phy?
 * @*status_reg : returning status register.
 */
int sl_get_phy_port_status(phy_ports_t phy_port, uint16_t *status_reg)
{
	adapter_type_t adapter_type;

	adapter_type = sl_get_chipset_info()->adapter_type;

	if(adapter_type != ADAPTER_PHY)
		return PHY_ONLY_USE;

	if(phy_port == PHY_PRIMARY_PORT)
		sl_read_smi(sl_get_chipset_info()->phy_address[0], PHY_MODE_STATUS_REGISTER, status_reg);
	else sl_read_smi(sl_get_chipset_info()->phy_address[1], PHY_MODE_STATUS_REGISTER, status_reg);

	return SWITCH_SUCCESS;
}

/*
 * Get phy chipset link speed & duplex
 */
int sl_get_phy_speed_duplex(phy_ports_t phy_port, int *speed, int *duplex)
{
	uint8_t phy_addr;
	uint16_t reg;

	if(phy_port == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	// Support Micrel phys chipsets
	if(sl_get_chipset_info()->adapter_name == PHY_KSZ9031
#ifdef PHY_MICREL_KSZ9131_PRODUCT_ID
		|| sl_get_chipset_info()->adapter_name == PHY_KSZ9131 
#endif 
#ifdef PHY_MICREL_KSZ8021_8031_8051_PRODUCT_ID
		|| sl_get_chipset_info()->adapter_name == PHY_KSZ8021_8031_8051
#endif
#ifdef PHY_MICREL_KSZ8081_8091_PRODUCT_ID
		|| sl_get_chipset_info()->adapter_name == PHY_KSZ8081_8091
#endif
		)
	{
		if(sl_get_chipset_info()->adapter_name == PHY_KSZ9031 
#ifdef PHY_MICREL_KSZ9131_PRODUCT_ID
		|| sl_get_chipset_info()->adapter_name == PHY_KSZ9131 
#endif  
		)
		{
			sl_read_smi(phy_addr, PHY_KSZ9031_CONTROL_REGISTER, &reg);

			if(reg & PHY_KSZ9031_SPEED_1G)
				*speed = 1000;
			else if(reg & PHY_KSZ9031_SPEED_100M)
				*speed = 100;
			else *speed = 10;

			if(reg & PHY_KSZ9031_DUPLEX_STATUS)
				*duplex = FULL_DUPLEX;
			else *duplex = HALF_DUPLEX;
		}
		else
		{
			uint16_t reg_temp;

			// This is for KSZ8051 chipset
			sl_read_smi(phy_addr, PHY_KSZ8051_CONTROL1_REGISTER, &reg);

			reg_temp = reg & PHY_KSZ8051_OPERATION_MODE_MASK;

			if( reg_temp == PHY_KSZ8051_SPEED_100M_HALF_DUPLEX)
			{
				*speed = 100;
				*duplex = HALF_DUPLEX;
			}
			else if(reg_temp == PHY_KSZ8051_SPEED_100M_FULL_DUPLEX)
			{
				*speed = 100;
				*duplex = FULL_DUPLEX;
			}
			else if(reg_temp == PHY_KSZ8051_SPEED_10M_HALF_DUPLEX)
			{
				*speed = 10;
				*duplex = HALF_DUPLEX;
			}
			else if(reg_temp == PHY_KSZ8051_SPEED_10M_FULL_DUPLEX)
			{
				*speed = 10;
				*duplex = FULL_DUPLEX;
			}
			else
			{
				*speed = 0;
				*duplex = 0;
			}
		}
	}
	else if(sl_get_chipset_info()->adapter_name == PHY_MARVEL_88E1510)
	{
		sl_read_smi(phy_addr, PHY_MARVEL_88E1510_COPPER_SPECIFIC_STATUS_REG, &reg);

		// If real time link status is on, then these speed & duplex is correct otherwise set those at 0
		if(reg & PHY_MARVEL_88E1510_COPPER_SPECIFIC_STATUS_COPPER_LINK)
		{
			*speed = PHY_MARVEL_88E1510_GET_PHY_SPEED(reg);
			*duplex = PHY_MARVEL_88E1510_GET_PHY_DUPLEX(reg);
		}
		else
		{
			*speed = 0;
			*duplex = HALF_DUPLEX;
		}
	}
	else
	{
		// This is for vitesse chipset and other phys
		uint16_t page_access_backup_reg;

		sl_read_smi(phy_addr, PHY_EXTENDED_PAGE_ACCESS_REG, &page_access_backup_reg);

		// changed extended reg to main reg space.
		if( page_access_backup_reg != PHY_ACCESS_MAIN_REG_SPACE )
			sl_write_smi(phy_addr, PHY_EXTENDED_PAGE_ACCESS_REG, PHY_ACCESS_MAIN_REG_SPACE);

		/* Read Vitesse VSC8601 PHY specific Auxilary Control and Status Register */
		sl_read_smi(phy_addr, PHY_PORT_AUX_REG, &reg);

		sw_trace_print("%s : reg - %x\n", __func__, reg);

		// recover to previous setting.
		if( page_access_backup_reg != PHY_ACCESS_MAIN_REG_SPACE )
			sl_write_smi(phy_addr, PHY_EXTENDED_PAGE_ACCESS_REG, page_access_backup_reg);

		*speed = GET_PHY_SPEED(reg);
		*duplex = GET_PHY_DUPLEX(reg);
	}

	sw_trace_print("%s : phy speed - %d, duplex - %s\n", __func__, *speed, (*duplex ? "Full duplex" : "Half duplex"));

	return SWITCH_SUCCESS;
}

/*
 * Get phy chipset link speed & duplex
 */
int sl_get_phy_link_status(phy_ports_t phy_port, int *link_status)
{
	int s;
	uint8_t phy_addr;
	uint16_t reg;

	if(phy_port == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	// Marvel real time link status is at 17_0.10 pg.128
	// AR8033 has same bit for real time link
	if(sl_get_chipset_info()->adapter_name == PHY_MARVEL_88E1510)
	{
		// Getting link up status. same for all phy chipsets
		sl_read_smi(phy_addr, PHY_MARVEL_88E1510_COPPER_SPECIFIC_STATUS_REG, &reg);

		if(reg & PHY_MARVEL_88E1510_COPPER_SPECIFIC_STATUS_COPPER_LINK)
			*link_status = 1;
		else *link_status = 0;
	}
	else
	{
		// Getting link up status. same for all phy chipsets
		s = sl_read_smi(phy_addr, MII_BMSR, &reg);
		if (s != 0)
		{
			// "%s: Could not read PHY status register; error %d\n",
			return SWITCH_FAILURE;
		}

		*link_status = GET_PHY_LINK_UP(reg);
	}

	sw_trace_print("%s : Link status - %s\n", __func__, *link_status ? "UP" : "Down");
	return SWITCH_SUCCESS;
}



/*
 * get default copper/fiber only vlan config vlaue
 */
uint32_t sl_get_default_copper_fiber_only_vlan_config(void)
{
	adapter_enum_t network_adapter;

	network_adapter = sl_get_chipset_info()->adapter_name;

	switch(network_adapter)
	{
		case SWITCH_88E6123: 	return SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6123;
		case SWITCH_88E6161:	return SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6161;

		case SWITCH_88E6350R:
		case SWITCH_88E6351:	return SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6350R;
		case SWITCH_88E6172:
		case SWITCH_88E6176:
		case SWITCH_88E6352:	return SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6352;
		case SWITCH_88E6320:
		case SWITCH_88E6321:	return SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6320;
		case SWITCH_88E6361:	return SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6361;

		default: return SWITCH_DEFAULT_COPPER_FIBER_VLAN_CONFIG;
	}
}


/******* Serdes register index conversion apis - shared by uboot & linux codes *********/
/*
 * Getting start bit num for serdes int enabling
 * serdes global for 6320 - bit 0:1
 * serdes global for 6352 - bit 11
 * serdes global for 6161 - bit 11:12
 */
uint16_t sl_get_global_reg2_serdes_int_status(adapter_enum_t switch_name, uint16_t global2_status_reg)
{
	switch(switch_name)
	{
		case SWITCH_88E6161:
			global2_status_reg &= SWITCH_SERDES_6161_INT_MASK;	// bit 11:12
			global2_status_reg >>= SWITCH_SERDES_6161_START_BIT;	// shift >> start bits
			return global2_status_reg;
		case SWITCH_88E6352:
		case SWITCH_88E6176:
			global2_status_reg &= SWITCH_SERDES_6352_INT_MASK;	// bit 11
			global2_status_reg >>= SWITCH_SERDES_6352_START_BIT;	// shift >> start bits
			return global2_status_reg;
		case SWITCH_88E6320:
		case SWITCH_88E6321:
			global2_status_reg &= SWITCH_SERDES_6320_INT_MASK;	// bit 0:1
			return global2_status_reg;
		case SWITCH_88E6361:
			global2_status_reg &= SWITCH_SERDES_6361_INT_MASK;	// bit 9:10
			global2_status_reg >>= SWITCH_SERDES_6361_START_BIT;	// shift >> start bits
			return global2_status_reg;
		default:
			return SWITCH_CANT_FIND_ANY;
	}
}


/*
 * Get serdes register index number by switch type
 */
int sl_get_serdes_reg_index(adapter_enum_t switch_name)
{
	switch(switch_name)
	{
		case SWITCH_88E6161: return SWITH_6161_SERDES_BASE_REG;
		case SWITCH_88E6352:
		case SWITCH_88E6176: return SWITH_6352_SERDES_BASE_REG;
		case SWITCH_88E6320:
		case SWITCH_88E6321: return SWITH_6320_SERDES_BASE_REG;
		case SWITCH_88E6361: return SWITH_6361_SERDES_BASE_REG;
		default : return SWITH_6320_SERDES_BASE_REG;	// default - start from 0xc
	}
}

/*
 * Getting start bit num for serdes int enabling
 * serdes global for 6320 - bit 0:1
 * serdes global for 6352 - bit 11
 * serdes global for 6161 - bit 11:12
 */
int sl_get_global_reg2_serdes_int_bit_index(adapter_enum_t switch_name)
{
	switch(switch_name)
	{
		case SWITCH_88E6161: return SWITCH_SERDES_6161_START_BIT;	// bit 11:12
		case SWITCH_88E6352:
		case SWITCH_88E6176: return SWITCH_SERDES_6352_START_BIT;	// bit 11
		case SWITCH_88E6320:
		case SWITCH_88E6321: return SWITCH_SERDES_6320_START_BIT;	// bit 0:1
		case SWITCH_88E6361: return SWITCH_SERDES_6361_START_BIT;	// bit 0:1
		default : return SWITCH_NOT_AVAILABLE_PORT;
	}
}

static void sl_port_statistic_reg_busy_check(void)
{
	uint16_t reg = 0;

	while(1)
	{
		sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, STATS_OPERATION_REG, &reg);
		if(!(reg&STATS_BUSY))
			break;
	}
}

static int sl_get_port_statistic_data_reg(uint32_t *received_counter)
{
	uint16_t reg = 0;

	*received_counter = 0;

	sl_port_statistic_reg_busy_check();
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, STATS_COUNT_BYTES_23, &reg);
	*received_counter = reg << 16;

	sl_port_statistic_reg_busy_check();
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, STATS_COUNT_BYTES_01, &reg);
	*received_counter |= (reg & 0xffff);

	return SWITCH_SUCCESS;
}

/*
 * Getting FCS & RX error counter of selected switch port
 */
int sl_get_port_statistic_error_counter(uint8_t port_num, uint32_t *sw_errors)
{
	uint16_t sw_reg;
	uint32_t received_counter;

	*sw_errors = 0;

	sl_port_statistic_reg_busy_check();
	sw_reg = (STATS_BUSY|STATS_OP_CAPTURE_ALL_COUNTERS_PORT|((port_num+1)<<5));
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, STATS_OPERATION_REG, sw_reg);

	sl_port_statistic_reg_busy_check();
	sw_reg = (STATS_BUSY|STATS_OP_READ_CAPTURED_COUNTERS_PORT|STATS_PTR_IN_RX_ERR);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, STATS_OPERATION_REG, sw_reg);
	sl_get_port_statistic_data_reg(&received_counter);
	*sw_errors += received_counter;

	sl_port_statistic_reg_busy_check();
	sw_reg = (STATS_BUSY|STATS_OP_READ_CAPTURED_COUNTERS_PORT|STATS_PTR_IN_FCS_ERR);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, STATS_OPERATION_REG, sw_reg);
	sl_get_port_statistic_data_reg(&received_counter);
	*sw_errors += received_counter;

	return SWITCH_SUCCESS;
}

// get switch port c mode value as marvell cmode
int sl_get_switch_port_marvell_cmode(int switch_port, uint8_t *port_cmode)
{
	adapter_enum_t switch_name;
	uint16_t reg_val;

	*port_cmode = SWITCH_PORT_CMODE_DISABLED;

	switch_name = sl_get_chipset_info()->adapter_name;

	sl_read_smi(SWITCH_PORT_BASE_ADDR + switch_port, SWITCH_PORT_STAT_REG, &reg_val ); // clear interrupt on SERDES level.

	if(switch_name == SWITCH_88E6350R || switch_name == SWITCH_88E6351)
	{
		reg_val = SWITCH_PORT_STATUS_GET_6350R_CMODE(reg_val);
		*port_cmode = reg_val;
	}
	else if(switch_name >= SWITCH_88E6172 && switch_name <= SWITCH_88E6321)
	{
		reg_val = SWITCH_PORT_STATUS_GET_6320_6352_CMODE(reg_val);
		*port_cmode = reg_val;
	}
	else return SWITCH_FAILURE;

	return SWITCH_SUCCESS;
}

// get physical port number according to the logical port number
int sl_get_physical_port_id(int logical_id)
{
	int physical_id = 0;

	// When accessing phy port, the input port id is 9&10 for serdes ports
	// When accessing switch port, the input port id is 6&7 for serdes ports
	int mv88e6361_physical_ids[] = {0, 3, 4, 5, 6, 7, 9, 10, -1, 9, 10};

	switch(sl_get_chipset_info()->adapter_name)
	{
		case SWITCH_88E6361:
			physical_id = mv88e6361_physical_ids[logical_id];
			if (physical_id < 0)
			{
				sw_trace_print("Illegal logical_id:%d", logical_id);
				physical_id = 0;
			}
			break;
		default:
			physical_id = logical_id;
			break;
	}

	return physical_id;
}

int sl_get_logical_port_id(int physical_id)
{
	int logical_id = 0;
	int mv88e6361_logical_ids[] = {0, -1, -1, 1, 2, 3, 4, 5, -1, 6, 7};

	switch(sl_get_chipset_info()->adapter_name)
	{
		case SWITCH_88E6361:
			logical_id = mv88e6361_logical_ids[physical_id];
			if (logical_id < 0)
			{
				sw_trace_print("Illegal physical_id:%d", physical_id);
				logical_id = 0;
			}
			break;
		default:
			logical_id = physical_id;
			break;
	}

	return logical_id;
}

int sl_get_max_switch_port_number(void)
{
	int switch_port_max = SWITCH_PORT_NUMBER_MAX;

	switch(sl_get_chipset_info()->adapter_name)
	{
		case SWITCH_88E6361:
			switch_port_max = 8;
			break;
		default:
			switch_port_max = SWITCH_PORT_NUMBER_MAX;
			break;
	}

	return switch_port_max;
}

void sl_lib_reg_clause45_handler_register(phy_reg_read_clause45_t read, phy_reg_write_clause45_t write)
{
	sl_get_chipset_info()->phy_reg_read_clause45_fn = read;
	sl_get_chipset_info()->phy_reg_write_clause45_fn = write;
}

int sl_read_reg_clause45(uint8_t dev_addr, uint8_t phy_addr, uint16_t reg_addr, uint16_t *reg_data)
{
	if (sl_get_chipset_info()->phy_reg_read_clause45_fn)
	{
		return sl_get_chipset_info()->phy_reg_read_clause45_fn(dev_addr, phy_addr, reg_addr, reg_data);
	}

	return SWITCH_DATA_INVALID;
}

int sl_write_reg_clause45(uint8_t dev_addr, uint8_t phy_addr, uint16_t reg_addr, uint16_t reg_data)
{
	if (sl_get_chipset_info()->phy_reg_write_clause45_fn)
	{
		return sl_get_chipset_info()->phy_reg_write_clause45_fn(dev_addr, phy_addr, reg_addr, reg_data);
	}

	return SWITCH_DATA_INVALID;
}

void sl_set_cpu_port(uint8_t cpu_port)
{
	sl_get_chipset_info()->cpu_port_num = cpu_port;
}

void sl_get_cpu_port(uint8_t *cpu_port)
{
	*cpu_port = sl_get_chipset_info()->cpu_port_num;
}
