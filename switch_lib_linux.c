/*
 * switch_lib_kernel.c
 *
 *  Created on: 15/05/2014
 *      Author: -Jerry-
 */
#include "switch_lib.h"
#include "switch_lib_reg.h"

/******* Interrupt enabling/clearing on switch/phy *********/
/*
 * Enable global interrupt
 */
int sl_enable_global_interrupt(void)
{
	uint16_t reg_val;

	// Global interrupt register.
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL1_CONTROL_REG, &reg_val);
	reg_val &= ~SWITCH_EEPROM_INT_ENABLE; // Disable EEPROM done interrupt
	reg_val |= SWITCH_DEV_INT_ENABLE; // interrupt mask register	& at the moment enable port 0&1
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL1_CONTROL_REG, reg_val);
	return SWITCH_SUCCESS;
}

// clear global interrupt/status register
int sl_clear_global_interrupt(void)
{
	uint16_t reg_val;
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL1_STATUS_REG, &reg_val);			// clear global interrupt 1 on switch mode.
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_SRC_REG, &reg_val);			// clear global interrupt 2 on switch mode.
	return SWITCH_SUCCESS;
}

// get global2 register & clear interrupts
int sl_get_global2_int_src_reg(uint16_t *reg_val)
{
	// get & clear global interrupt
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_SRC_REG, reg_val);
	return SWITCH_SUCCESS;
}

/*
 * set link status change interrupt on a single switch internal phy 0~4
  */
int sl_set_intr_switch_phy(int port_index)
{
	uint16_t reg_val;
	sl_read_smi(port_index, SWITCH_PHY_INT_ENABLE_REG, &reg_val);
	sl_write_smi(port_index, SWITCH_PHY_INT_ENABLE_REG, (reg_val | SWITCH_PHY_LINK_STATUS_CHANGE) );	// switch link status change int enable
	return SWITCH_SUCCESS;
}

// clear interrupt on a single phy port
int sl_clear_intr_switch_phy(int port_index)
{
	uint16_t reg_val;
	sl_read_smi(port_index, SWITCH_PHY_INT_STATUS_REG, &reg_val );		// clear interrupt on phy level
	return SWITCH_SUCCESS;
}

// get internal phys interrupt status register value from global register 2.
int sl_get_switch_phy_intr_status(void)
{
	uint16_t global2_status_reg;
	adapter_enum_t switch_type;

	switch_type = sl_get_chipset_info()->adapter_name;

	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_SRC_REG, &global2_status_reg);

	// Phy ports start from 3 for 88E6320 & 88E6321 switches
	if(switch_type == SWITCH_88E6320 || switch_type == SWITCH_88E6321)
		global2_status_reg &= SWITCH_88E6320_PHYINT_MASK;	// 88E6320 series uses port 3,4
	else if(switch_type == SWITCH_88E6361)
		global2_status_reg &= SWITCH_88E6361_PHYINT_MASK;	// 88E6361 series uses port 3~7
	else global2_status_reg &= SWITCH_PHYINT_MASK;			// all other switch chipsets - use only bit 0-4

	sw_trace_print("%s : %x\n", __func__, global2_status_reg);

	return global2_status_reg;
}

//
/*
 * get serdes interrupt status register value from global register 2.
 * return global status reg for only serdes available switch
 */

int sl_get_serdes_intr_status(void)
{
	uint16_t global2_status_reg;
	adapter_enum_t switch_type;

	switch_type = sl_get_chipset_info()->adapter_name;

	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_SRC_REG, &global2_status_reg);

	return sl_get_global_reg2_serdes_int_status(switch_type, global2_status_reg);
}

/*
 * set interrupt on single serdes port
 * port index should be calculated to absolute index.
 *
 * 6352 series - 0xF
 * 6161 series - 0xC
 * 6320 series - 0xC & 0xD
 * 6361 series - 0x9 & 0xA
 */
int sl_set_intr_serdes(int serdex_port)
{
	uint16_t reg_val;
#if (SUPPORT_SWITCH_88E6361 == 1)
	if (sl_get_chipset_info()->adapter_name == SWITCH_88E6361)
	{
		sl_read_reg_clause45(SWITCH_88E6361_SERDES_DEVICE_ADDR, serdex_port, SWITCH_88E6361_SERDES_FIBER_INT_ENABLE, &reg_val);
		sl_write_reg_clause45(SWITCH_88E6361_SERDES_DEVICE_ADDR, serdex_port, SWITCH_88E6361_SERDES_FIBER_INT_ENABLE, (reg_val | SWITCH_88E6361_SERDES_LINK_STATUS_CHANGE));
	}
	else
#endif // SUPPORT_SWITCH_88E6361 == 1
	{
		sl_read_smi(serdex_port, SWITCH_SERDES_FIBER_INT_ENABLE, &reg_val);
		sl_write_smi(serdex_port, SWITCH_SERDES_FIBER_INT_ENABLE, (reg_val | SWITCH_PHY_LINK_STATUS_CHANGE) ); // serdes link interrupt enabled.
	}
	return SWITCH_SUCCESS;
}

/*
 * clear all interupts on serdes port
 * port index should be calculated to absolute index.
 */
int sl_clear_intr_serdes(int serdex_port)
{
	uint16_t reg_val;

#if (SUPPORT_SWITCH_88E6361 == 1)
	if (sl_get_chipset_info()->adapter_name == SWITCH_88E6361)
	{
		sl_read_reg_clause45(SWITCH_88E6361_SERDES_DEVICE_ADDR, serdex_port, SWITCH_88E6361_SERDES_FIBER_INT_STATUS, &reg_val);
	}
	else
#endif // SUPPORT_SWITCH_88E6361 == 1
	{
		sl_read_smi(serdex_port, SWITCH_SERDES_FIBER_INT_STATUS, &reg_val ); // clear interrupt on SERDES level.
	}
	return SWITCH_SUCCESS;
}


/******* phy related control functions ***********/
// reset phy for primary/secondary - what is primary/secondary???
int sl_reset_phy(phy_ports_t phy_port)
{
	uint8_t phy_addr;

	if(phy_port == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	sl_write_smi(phy_addr, PHY_MODE_CTRL_REG, PHY_CONTROL_SOFTWARE_RESET);

	return SWITCH_SUCCESS;
}

// isolate phys for primary/secondary
int sl_isolate_phy(phy_ports_t phy_port, phy_isolate_ctl_s control)
{
	uint8_t phy_addr;
	uint16_t reg_val;

	if(phy_port == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	// Read out phy mode control register
	sl_read_smi(phy_addr, PHY_MODE_CTRL_REG, &reg_val);

	if(control == PHY_ISOLATE_ENABLE)
	{
		// if it is not isolated then, isolate it.
		if(!(reg_val & PHY_MODE_CTRL_ISOLATE))
		{
			reg_val |= PHY_MODE_CTRL_ISOLATE;
			sl_write_smi(phy_addr, PHY_MODE_CTRL_REG, reg_val);
		}
	}
	else
	{
		// disabling isolate option if it is isolated only.
		if(reg_val & PHY_MODE_CTRL_ISOLATE)
		{
			reg_val &= ~PHY_MODE_CTRL_ISOLATE;
			sl_write_smi(phy_addr, PHY_MODE_CTRL_REG, reg_val);
		}
	}

	return SWITCH_SUCCESS;
}

// phy led configuration
int sl_phy_led_configuration(phy_ports_t phy_index)
{
	adapter_enum_t adapter_name;
	uint8_t phy_addr;

	adapter_name = sl_get_chipset_info()->adapter_name;

	if(phy_index == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	if(adapter_name == PHY_MARVEL_88E1510)
	{
		// Configuring LED1 as link on/blink on activity
		// Configuring LED0 as on 1G and off for else
		// change page address to 3

		uint16_t page_addr_temp, reg_val;

		// change page to 3
		sl_read_smi(phy_addr, SWITCH_PHY_PAGE_ADDR_REG, &page_addr_temp);
		sl_write_smi(phy_addr, SWITCH_PHY_PAGE_ADDR_REG, 3);

		// set led0 for 1G on and off for else.
		// set led1 for link on/blink activity
		sl_read_smi(phy_addr, PHY_88E1510_LED_FUNC_CTL_REG, &reg_val);
		reg_val &= PHY_88E1510_LED_FUNC_CTL_LEDS_MASK;
		reg_val |= (PHY_88E1510_LED_FUNC_CTL_LED0_1G_ON_ELSE_OFF | PHY_88E1510_LED_FUNC_CTL_LED1_LINK_ON_BLINK_ACTIVITY);
		sl_write_smi(phy_addr, PHY_88E1510_LED_FUNC_CTL_REG, reg_val);

		// back to page to previous status.
		sl_write_smi(phy_addr, SWITCH_PHY_PAGE_ADDR_REG, page_addr_temp);

	}
#ifdef PHY_VSC8601
	else if(adapter_name == PHY_VSC8601)
	{
		sl_write_smi(phy_addr, 0x1f,0x1); // Enable extended registers
		sl_write_smi(phy_addr, 0x11,0x1c12);
		sl_write_smi(phy_addr, 0x11,0xc12);
		sl_write_smi(phy_addr, 0x10,0x10);
		sl_write_smi(phy_addr, 0x1f,0x0); // Disable extended registers
	}
#endif
	else if(adapter_name == PHY_VSC8541)
	{
		uint16_t reg_val;
		
		//change to PHY register main page
		sl_write_smi(phy_addr, PHY_VSC8541_EXTENDED_PAGE_ACCESS_REG, PHY_VSC8541_MAIN_REG_PAGE);

		//disable the combination of "Link/Activity" from LED1 so that it is on and not flashing when a gigabit link is active.
		sl_read_smi(phy_addr, PHY_VSC8541_LED_BEHAVIOUR_REG, &reg_val);
		reg_val |= (PHY_VSC8541_LED1_COMBINE_DISABLE);
		sl_write_smi(phy_addr, PHY_VSC8541_LED_BEHAVIOUR_REG, reg_val);
		
	}
	return SWITCH_SUCCESS;
}
/*
 * Set phy chipset interrupt with link status change
 */
int sl_set_intr_phy(phy_ports_t phy_index)
{
	adapter_enum_t adapter_name;
	uint8_t phy_addr;
	uint16_t reg_val;

	adapter_name = sl_get_chipset_info()->adapter_name;

	if(phy_index == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	if(adapter_name == PHY_KSZ9031
#ifdef PHY_MICREL_KSZ8021_8031_8051_PRODUCT_ID
		|| adapter_name == PHY_KSZ8021_8031_8051
#endif
#ifdef PHY_MICREL_KSZ8081_8091_PRODUCT_ID
		|| adapter_name == PHY_KSZ8081_8091
#endif
#ifdef PHY_MICREL_KSZ9131_PRODUCT_ID
		|| adapter_name == PHY_KSZ9131
#endif
		)
	{
		// This is MICREL phy chipsets
		sl_read_smi(phy_addr, PHY_MICREL_INT_CTL_REG, &reg_val);	// read interrupt control reg and clear int status
		reg_val |= (PHY_MICREL_INT_LINK_UP_ENABLE|PHY_MICREL_INT_LINK_DOWN_ENABLE);
		sl_write_smi(phy_addr, PHY_MICREL_INT_CTL_REG, reg_val);
		sl_read_smi(phy_addr, PHY_MICREL_INT_CTL_REG, &reg_val);	// clear int status
	}
	else if(adapter_name == PHY_MARVEL_88E1510)
	{
		uint16_t page_addr_temp;

		// change page to 3
		sl_read_smi(phy_addr, SWITCH_PHY_PAGE_ADDR_REG, &page_addr_temp);	// read interrupt control reg and clear int status
		sl_write_smi(phy_addr, SWITCH_PHY_PAGE_ADDR_REG, 3);

		// Enable interrupt pin on LED[2]
		sl_read_smi(phy_addr, PHY_88E1510_LED_TIMER_CTRL_REG, &reg_val);	// read interrupt control reg and clear int status
		reg_val |= PHY_88E1510_LED_TIMER_CTRL_REG_INTERRUPT_ENABLE;
		sl_write_smi(phy_addr, PHY_88E1510_LED_TIMER_CTRL_REG, reg_val);

		// back to page to previous status.
		sl_write_smi(phy_addr, SWITCH_PHY_PAGE_ADDR_REG, page_addr_temp);

		// enable link change interrupt & clear
		sl_read_smi(phy_addr, PHY_88E1510_INT_ENABLE_REG, &reg_val);	// read interrupt control reg and clear int status
		reg_val |= PHY_88E1510_INT_LINK_CHANGE;
		sl_write_smi(phy_addr, PHY_88E1510_INT_ENABLE_REG, reg_val);
		sl_read_smi(phy_addr, PHY_88E1510_INT_STATUS_REG, &reg_val);	// clear int status
	}
	else if(adapter_name == PHY_VSC8541)
	{
		sl_write_smi(phy_addr, PHY_VSC8541_EXTENDED_PAGE_ACCESS_REG, PHY_VSC8541_MAIN_REG_PAGE);		//ensure that the correct register page is selected.
		sl_read_smi(phy_addr, PHY_VSC8541_INTERRUPT_STATUS_REG, &reg_val);		//read interrupt status register to clear existing interrupt status - self-clearing register.
		sl_read_smi(phy_addr, PHY_VSC8541_INTERRUPT_MASK_REG, &reg_val);		//read interrupt mask register.
		reg_val |= (PHY_VSC8541_MDINT_ENABLE|PHY_VSC8541_LINK_STATE_CHANGE);		//enable MDINT output and link state change interrupt.
		sl_write_smi(phy_addr, PHY_VSC8541_INTERRUPT_MASK_REG, reg_val);
		sl_read_smi(phy_addr, PHY_VSC8541_INTERRUPT_STATUS_REG, &reg_val);		//read interrupt status register to clear existing interrupt status - self-clearing register.
	}
	else
	{
		// This is for Vitesse and others.
		sl_read_smi(phy_addr, PHY_INT_MASK_REG, &reg_val);	// read mask register
		sl_write_smi(phy_addr, PHY_INT_MASK_REG, (reg_val |PHY_INT_LINK_STATUS_CHANGE|PHY_MDINT_STATUS_ENABLE)); // enable link status change interrupt.
		// clear current interrupt.
		sl_read_smi(phy_addr, PHY_INT_STATUS_REG, &reg_val);
	}

	return SWITCH_SUCCESS;
}

/*
 * Set phy chipset interrupt with link status change
 */
int sl_clear_intr_phy(phy_ports_t phy_index)
{
	adapter_enum_t adapter_name;
	uint8_t phy_addr;
	uint16_t reg_val;

	adapter_name = sl_get_chipset_info()->adapter_name;

	if(phy_index == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	if(adapter_name == PHY_KSZ9031
#ifdef PHY_MICREL_KSZ8021_8031_8051_PRODUCT_ID
		|| adapter_name == PHY_KSZ8021_8031_8051
#endif
#ifdef PHY_MICREL_KSZ8081_8091_PRODUCT_ID
		|| adapter_name == PHY_KSZ8081_8091
#endif
#ifdef PHY_MICREL_KSZ9131_PRODUCT_ID
		|| adapter_name == PHY_KSZ9131
#endif
		)
	{
		// This is MICREL phy chipsets
		sl_read_smi(phy_addr, PHY_MICREL_INT_CTL_REG, &reg_val);
	}
	else if(adapter_name == PHY_MARVEL_88E1510)
	{
		sl_read_smi(phy_addr, PHY_88E1510_INT_STATUS_REG, &reg_val);
	}
	else if(adapter_name == PHY_VSC8541)
	{
		sl_read_smi(phy_addr, PHY_VSC8541_INTERRUPT_STATUS_REG, &reg_val);
	}
	else
	{
		// This is for Vitesse and others.
		sl_read_smi(phy_addr, PHY_INT_STATUS_REG, &reg_val);
	}

	return SWITCH_SUCCESS;
}

/******* Interrupt enabling/clearing on switch/phy *********/

/*
 * set interrupt on all switch internal phys based on vlan config
 * Note : it enables interrupt for only primary & secondary configuration
 * @vlan_config : vlan configuration with primary & secondary.
 */
int sl_set_intr_switch_phy_all(uint32_t vlan_config, uint32_t external_phys)
{
	uint8_t vlan_port;
	uint16_t reg_val;
	int i, phy_port_start, phy_port_mum;
	uint32_t int_mask = 0;

	// Get internal phy start index.
	phy_port_start = sl_get_chipset_info()->switch_ports_info.num_phy_port_start;
	phy_port_mum = sl_get_chipset_info()->switch_ports_info.num_phy_ports;

	// Enable phy link status change interrupt
	for(i=phy_port_start; i < (phy_port_mum+phy_port_start); i++ )
	{
		vlan_port = (vlan_config >> (i*4)) & 0xff;

#if defined(LCCK60) || defined(__ZEPHYR__)
		// For Ultomo
		if(vlan_port&ETH_SWITCH_VLAN_PRI)
		{
			// Set interrupt on phy
			sl_set_intr_switch_phy(i);
			// clear interrupt on phy level.
			sl_clear_intr_switch_phy(i);
			int_mask |= (1<<i);	// calculating int mask value.

			sw_trace_print("%s : enable int phy port - %d\n", __func__, i);
		}
#else
		if( (vlan_port & ETH_SWITCH_VLAN_PRI) || (vlan_port & ETH_SWITCH_VLAN_SEC))
		{
			// if a port is used for external phy port, then do not set up interrupt for internal phy.
			if(!(external_phys & (ETH_SWITCH_EXTERNAL_PHY_MODE << (i*4))))
			{
				// Set interrupt on phy
				sl_set_intr_switch_phy(i);
				// clear interrupt on phy level.
				sl_clear_intr_switch_phy(i);
				int_mask |= (1 << sl_get_physical_port_id(i)); // calculating int mask value.
				sw_trace_print("%s : enable int phy port - %d\n", __func__, i);
			}
		}
#endif

	}

	sw_trace_print("%s : int mask - %x\n",__func__, int_mask);

	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_MASK_REG, &reg_val); 		// interrupt mask register
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_MASK_REG, (reg_val|int_mask));	// interrupt mask register

	return SWITCH_SUCCESS;
}

/*
 * Set interrupts for all phys
 */
int sl_set_intr_phy_all(void)
{
	// enable interrupt for primary phy port
	sl_set_intr_phy(PHY_PRIMARY_PORT);
	// PHY redundant ?
	if(sl_get_chipset_info()->phy_redundant)
		sl_set_intr_phy(PHY_SECONDARY_PORT);

	return SWITCH_SUCCESS;
}

/*
 * clear interrupts for all phys
 */
int sl_clear_intr_phy_all(void)
{
	// clear interrupt for primary phy
	sl_clear_intr_phy(PHY_PRIMARY_PORT);
	// PHY redundant ?
	if(sl_get_chipset_info()->phy_redundant)
		sl_clear_intr_phy(PHY_SECONDARY_PORT);

	return SWITCH_SUCCESS;
}

/*
 * clear all internal phys interrupt from switch phys
 * Note : This is for only all phys ports
 */
int sl_clear_intr_switch_phy_all(uint16_t global2_status_reg)
{
	uint8_t phy_port_start = 0;
	int logical_port_id;
	adapter_enum_t switch_type = sl_get_chipset_info()->adapter_name;

	// Phy ports start from 3 for 88E6320 & 88E6321 switches
	if(switch_type == SWITCH_88E6320 || switch_type == SWITCH_88E6321)
		global2_status_reg &= SWITCH_88E6320_PHYINT_MASK;	// 88E6320 series uses port 3,4
	else if(switch_type == SWITCH_88E6361)
		global2_status_reg &= SWITCH_88E6361_PHYINT_MASK;	// 88E6361 series uses port 3~7
	else global2_status_reg &= SWITCH_PHYINT_MASK;		// all other switch chipsets - use only bit 0-4

	// Clear internal phy interrupts
	while(global2_status_reg)
	{
		if(global2_status_reg & 0x1)
		{
			logical_port_id = sl_get_logical_port_id(phy_port_start);
			// clear interrupt on phy level.
			sl_clear_intr_switch_phy(logical_port_id);
			sw_trace_print("%s : clear port - %d\n",__func__, logical_port_id);
		}

		phy_port_start ++;
		global2_status_reg >>= 1;
	}

	return SWITCH_SUCCESS;
}

/*
 * Check if there is any interrupt,  then return SWITCH_INT_AVAILABLE
 */
int sl_is_interrupt(uint16_t global2_status_reg)
{
	uint16_t phy_int_reg, serdes_int_reg;
	adapter_enum_t switch_type = sl_get_chipset_info()->adapter_name;

	// Phy ports start from 3 for 88E6320 & 88E6321 switches
	if(switch_type == SWITCH_88E6320 || switch_type == SWITCH_88E6321)
		phy_int_reg = global2_status_reg & SWITCH_88E6320_PHYINT_MASK;	// 88E6320 series uses port 3,4
	else if(switch_type == SWITCH_88E6361)
		phy_int_reg = global2_status_reg & SWITCH_88E6361_PHYINT_MASK;	// 88E6361 series uses port 3~7
	else phy_int_reg = global2_status_reg & SWITCH_PHYINT_MASK;		// all other switch chipsets - use only bit 0-4

	// Do we have serdes ports?
	if(sl_get_chipset_info()->switch_ports_info.num_serdes_ports)
	{
		// Extracting serdes interrupt status bits
		serdes_int_reg = sl_get_global_reg2_serdes_int_status(sl_get_chipset_info()->adapter_name, global2_status_reg);
	}
	else serdes_int_reg = 0;

	sw_trace_print("%s : phy_int_reg - %x, serdes_int_reg - %x\n",__func__, phy_int_reg, serdes_int_reg);

	if(phy_int_reg || serdes_int_reg)
		return SWITCH_INT_AVAILABLE;
	return SWITCH_INT_NOT_AVAILABLE;
}

/*
 * Clear all external phy interrupts
 */
int sl_clr_ext_phys_all(uint32_t external_phys, uint16_t *ext_phy_reg_array)
{
#if !defined(LCCK60) && !defined(__ZEPHYR__)
  	uint8_t phy_port_start;

	// Clear interrupt status for external ports
	if(external_phys)
	{
		uint16_t reg_val;
		uint32_t external_phys_temp;

		phy_port_start = 0;
		external_phys_temp = external_phys & SWITCH_EXTERNAL_PHY_PORTS_MASK;	// except port 5

		while(external_phys_temp)
		{
			if(external_phys_temp & ETH_SWITCH_EXTERNAL_PHY_MODE)
			{
				sl_read_smi(phy_port_start, *(ext_phy_reg_array+phy_port_start), &reg_val );
			}

			phy_port_start ++;
			external_phys_temp >>= 4;	// nibbled based config up to - 8 ports.
		}
	}
#endif
	return SWITCH_SUCCESS;
}

// set interrupt on serdes ports based on vlan config
int sl_set_intr_serdes_all(uint32_t vlan_config)
{
	int i;
	adapter_enum_t switch_name;
	uint16_t int_mask, reg_val;
	uint8_t serdes_port_start, serdes_page_num;
	int serdes_reg_base, global_reg2_bit_index;

	serdes_port_start = int_mask = 0;

	// Do we have serdes ports?
	if(!sl_get_chipset_info()->switch_ports_info.num_serdes_ports)
		return SWITCH_NOT_AVAILABLE_SERDES;

	switch_name = sl_get_chipset_info()->adapter_name;

	// Getting base reg addr for SERDES ports
	serdes_reg_base = sl_get_serdes_reg_index(switch_name);
	// Getting global register serdes index bits
	global_reg2_bit_index = sl_get_global_reg2_serdes_int_bit_index(switch_name);

	serdes_port_start = sl_get_chipset_info()->switch_ports_info.num_serdes_port_start;

	switch(switch_name)
	{
		case SWITCH_88E6172:
		case SWITCH_88E6176:
		case SWITCH_88E6352:
		case SWITCH_88E6320:
		case SWITCH_88E6321:
			serdes_page_num = 1;	break;
		default: serdes_page_num = 0;	break;
	}

	// Enable phy link status change interrupt
	for(i=0; i < (sl_get_chipset_info()->switch_ports_info.num_serdes_ports); i++ )
	{
		// 88E6361 doesn't use page register to access serdes register
		if (sl_get_chipset_info()->adapter_name != SWITCH_88E6361)
		{
			// Page register number is correct for handling serdes register?
			sl_read_smi((serdes_reg_base+i), SWITCH_PHY_PAGE_ADDR_REG, &reg_val);
			if(serdes_page_num != reg_val)
			{
				sl_write_smi((serdes_reg_base+i), SWITCH_PHY_PAGE_ADDR_REG, serdes_page_num); // change page register to nominated number
			}
		}
#if defined(LCCK60) || defined(__ZEPHYR__)
		// For Ultomo
		if(vlan_config & (ETH_SWITCH_VLAN_PRI<<((i+serdes_port_start)*4)))

#else
		if((vlan_config & (ETH_SWITCH_VLAN_PRI<<((i+serdes_port_start)*4))) || (vlan_config & (ETH_SWITCH_VLAN_SEC<<((i+serdes_port_start)*4))))
#endif
		{
			// enable link change interrupt
			sl_set_intr_serdes(serdes_reg_base+i);
			// clear interrupt on SERDES level.
			sl_clear_intr_serdes(serdes_reg_base+i);
			// calculating int mask value.
			int_mask |= (1<< (global_reg2_bit_index+i));
		}
	}

	sw_trace_print("%s : int mask - %x\n",__func__, int_mask);

	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_MASK_REG, &reg_val); 		// interrupt mask register
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_MASK_REG, (reg_val|int_mask));	// interrupt mask register

	return SWITCH_SUCCESS;
}

// clear all interupts on serdes ports
int sl_clear_intr_serdes_all(uint16_t global2_status_reg)
{
	uint8_t serdes_page_num;
	uint16_t serdes_status_reg;

	// Do we have serdes ports?
	if(!sl_get_chipset_info()->switch_ports_info.num_serdes_ports)
		return SWITCH_NOT_AVAILABLE_SERDES;

	// Extracting serdes interrupt status bits
	serdes_status_reg = sl_get_global_reg2_serdes_int_status(sl_get_chipset_info()->adapter_name, global2_status_reg);

	switch(sl_get_chipset_info()->adapter_name)
	{
		case SWITCH_88E6172:
		case SWITCH_88E6176:
		case SWITCH_88E6352:
		case SWITCH_88E6320:
		case SWITCH_88E6321:
			serdes_page_num = 1; 	break;
		default: serdes_page_num = 0;	break;
	}

	// clear SERDES interrupt status
	if(serdes_status_reg)
	{
		uint8_t reg_base, phy_port_start;
		uint16_t reg_val;

		phy_port_start = 0;
		reg_base = sl_get_serdes_reg_index(sl_get_chipset_info()->adapter_name);

		while(serdes_status_reg)
		{
			// 88E6361 doesn't use page register to access serdes register
			if (sl_get_chipset_info()->adapter_name != SWITCH_88E6361)
			{
				// Page register number is correct for handling serdes register?
				sl_read_smi((reg_base+phy_port_start), SWITCH_PHY_PAGE_ADDR_REG, &reg_val);
				if(serdes_page_num != reg_val)
				{
					sl_write_smi((reg_base+phy_port_start), SWITCH_PHY_PAGE_ADDR_REG, serdes_page_num); // change page register to nominated number
				}
			}

			if(serdes_status_reg & 0x1)
			{
				// clear serdes interrupt status
				sl_clear_intr_serdes(reg_base + phy_port_start);
			}
			phy_port_start ++;
			serdes_status_reg >>= 1;
		}
	}

	return SWITCH_SUCCESS;
}

/*
 * Bosch switch - configure PHY LEDs
 * 1Gbps - Green, 100Mbps - Orange.
 * Simply change polarity
 */
int sl_marvel_sw_led_config(void)
{

	uint8_t phy_port_start;

	// Phy ports start from 3 for 88E6320 & 88E6321 switches
	phy_port_start = sl_get_chipset_info()->switch_ports_info.num_phy_port_start;

	// Switch Eth0 Led configure.
	sl_write_smi(SWITCH_PHY_PORT_BASE_ADDR+phy_port_start, SWITCH_PHY_PAGE_ADDR_REG, 3);		// Change Page Address - 3
	sl_write_smi(SWITCH_PHY_PORT_BASE_ADDR+phy_port_start, SWITCH_PHY_LED_FUNCTION_CTRL_REG, 0x1077);  // Led Configure
	sl_write_smi(SWITCH_PHY_PORT_BASE_ADDR+phy_port_start, SWITCH_PHY_LED_POLARITY_CTRL_REG, 0x5);     // Change polarity
	sl_write_smi(SWITCH_PHY_PORT_BASE_ADDR+phy_port_start, SWITCH_PHY_PAGE_ADDR_REG, 0);		// Back Page Address to 0.

	phy_port_start++;

	// Switch Eth1 Led configure.
	sl_write_smi(SWITCH_PHY_PORT_BASE_ADDR+phy_port_start, SWITCH_PHY_PAGE_ADDR_REG, 3);		// Change Page Address - 3
	sl_write_smi(SWITCH_PHY_PORT_BASE_ADDR+phy_port_start, SWITCH_PHY_LED_FUNCTION_CTRL_REG, 0x1077);  // Led Configure
	sl_write_smi(SWITCH_PHY_PORT_BASE_ADDR+phy_port_start, SWITCH_PHY_LED_POLARITY_CTRL_REG, 0x5);     // Change polarity
	sl_write_smi(SWITCH_PHY_PORT_BASE_ADDR+phy_port_start, SWITCH_PHY_PAGE_ADDR_REG, 0);		// Back Page Address to 0.

	return SWITCH_SUCCESS;
}

/*
 * Control each port led control
 * @param port_num : port nubmer
 * @param action : force on, force off, default
 */
int sl_marvel_port_led_control(uint8_t port_num, switch_port_led_action action)
{
	if(sl_get_chipset_info()->adapter_type == ADAPTER_SWITCH)
	{
		uint16_t action_cmd = (SWITCH_LED_CTR_UPDATE|SWITCH_LED_CTR_POINTER(SWITCH_LED_CTR_LED0N1));

		switch(action)
		{
			case SWITCH_LED_CONTROL_FORCE_OFF :
				action_cmd |= SWITCH_LED_CTR_FORCE_OFF;
				break;
			case SWITCH_LED_CONTROL_FORCE_ON :
				action_cmd |= SWITCH_LED_CTR_FORCE_ON;
				break;
			case SWITCH_LED_CONTROL_LINK_ONLY :
				action_cmd |= SWITCH_LED_CTR_LINK_ON_ONLY_1G_ON;
				break;
			case SWITCH_LED_CONTROL_DEFAULT :
				action_cmd |= SWITCH_LED_CTL_DEFAULT;
				break;
			default :
				action_cmd |= SWITCH_LED_CTL_DEFAULT;
				break;
		}

		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_num, SWITCH_LED_CTR_REG, action_cmd);
	}

	return SWITCH_SUCCESS;
}

/*
 * Accessing each port led control
 * @param port_num : port nubmer
 * @param action : read / write
 */
int sl_marvel_copper_port_led_reg_access(uint8_t port_num, uint16_t *port_reg, switch_port_led_reg_access reg_access_action)
{
	if(sl_get_chipset_info()->adapter_type == ADAPTER_SWITCH)
	{
		// avaliable copper port?
		if(port_num >= SWITCH_COPPER_PORT_NUMBER_MAX)
			return SWITCH_NOT_AVAILABLE_PORT;

		if(reg_access_action == SWITCH_LED_CTL_REG_READ)
			sl_read_smi(SWITCH_PORT_BASE_ADDR+port_num, SWITCH_LED_CTR_REG, port_reg);
		else if(reg_access_action == SWITCH_LED_CTL_REG_WRITE)
		{
			*port_reg |= (SWITCH_LED_CTR_UPDATE|SWITCH_LED_CTR_POINTER(SWITCH_LED_CTR_LED0N1));

			sl_write_smi(SWITCH_PORT_BASE_ADDR+port_num, SWITCH_LED_CTR_REG, *port_reg);
		}
		else return SWITCH_DATA_INVALID;
	}
	else return SWITCH_ONLY_USE;

	return SWITCH_SUCCESS;
}
