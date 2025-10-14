/*
 * switch_lib_uboot.c
 *
 *  Created on: 15/05/2014
 *      Author: -Jerry-
 */
#include "switch_lib.h"
#include "switch_lib_reg.h"

/******* Configuring QoS on a port on switch *******/
int sl_clear_switch_ip_priority_table(void);

/*
 * set up vlan configuration on a single switch port
 * @port_index : switch port num 0 - 6
 */
int sl_set_switch_vlan_config(int port_index, uint16_t vlan_config)
{
	sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_VLANT_REG, vlan_config);     // switch port disable
	return SWITCH_SUCCESS;
}

/*
 * Set switch port adding marvel header
 * @port_index : switch port num 0 - 6
 * @control - enable/disable
 */
int sl_set_switch_port_marvel_header(int port_index, switch_port_ctl_s control)
{
	uint16_t reg_val;

	// port 5 configure with marvel header
	if(control == PORT_ENABLE)
	{
		sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, &reg_val);
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, reg_val | SWITCH_PORT_INGRESS_EGRESS_HEADER_MODE );
		sw_trace_print("%s : enable\n",__func__);
	}
	else
	{
		// removing port 5 marvel header
		sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, &reg_val);
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, (reg_val & ~SWITCH_PORT_INGRESS_EGRESS_HEADER_MODE) );
		sw_trace_print("%s : disable\n",__func__);
	}

	return SWITCH_SUCCESS;
}

/******* Static ATU multicast filtering entrys management *******/
/*
 * add one static multicast address filter
 * @param multicast_addr : 32 bit multicast address
 * @param atu_ctl : purge/load
 * @dpv_bitmap : applying destination port vector. Will use default if this is 0
 */
int sl_manage_atu_entry(uint32_t multicast_addr, switch_atu_ctl_t atu_ctl, uint16_t dpv_bitmap, uint8_t fid_val)
{
	int retry_count = 10;
	uint16_t dpv, reg_val;
	uint16_t multicast_mac_address[3] = { 0x100, 0, 0 }; // Multicast mac address format

	if(!multicast_addr)
		return SWITCH_DATA_INVALID;

	sw_trace_print("Multicast filter adding - %d.%d.%d.%d and dpv_bitmap:fid_val - %x:%d\n", (multicast_addr>>24) & 0xff, (multicast_addr>>16) & 0xff, (multicast_addr>>8) & 0xff, multicast_addr&0xff, dpv_bitmap, fid_val);

	/* For each multicast address
	 * add a ATU entry that enables forwarding of this multicast address to every port
	 * disabled ports are dont care
	 */
	// Calculate mac address from IP address
	multicast_mac_address[1] = 0x5e00 | ((multicast_addr >> 16) & 0x7f);
	multicast_mac_address[2] = multicast_addr & 0xffff;

	// In case we are given with other DPV info
	if(dpv_bitmap)
		dpv = dpv_bitmap;
	else dpv = sl_get_switch_default_dpv(); // Getting DPV value

	// specify ATU FID value as 0
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_ATU_FID_REG, fid_val);

	// 4 bit / 4bit / forwarding bitmap / entry state ( STATIC or 0 )
	if(atu_ctl == ATU_ENTRY_LOAD)
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_ATU_DATA_REG, ((dpv <<4) |SWITCH_GLOBAL_ATU_ENTRY_STATE_STATIC) ); // static entry
	else if(atu_ctl == ATU_ENTRY_PURGE)
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_ATU_DATA_REG, ((dpv <<4) |SWITCH_GLOBAL_ATU_ENTRY_STATE_PURGE) ); // static entry

	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_ATU_MAC_REG_1, multicast_mac_address[0]);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_ATU_MAC_REG_2, multicast_mac_address[1]);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_ATU_MAC_REG_3, multicast_mac_address[2]);

	// load or purge an entry in a particular FID DB 0
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_ATU_OPERATION_REG, (SWITCH_GLOBAL_ATU_ENTRY_LOAD_PURGE | SWITCH_GLOBAL_ATUop_START_BUSY_BIT) );

	while(retry_count--)
	{
		// Checking busy flag
		sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_ATU_OPERATION_REG, &reg_val);
		if( !(reg_val&SWITCH_GLOBAL_ATUop_START_BUSY_BIT) )
		{
			break;
		}

		sl_nop_delay_loop(1000);	// delay 1ms
	}

	if(!retry_count)
		return SWITCH_MDIO_TIMEOUT;

	return SWITCH_SUCCESS;
}

// probe one static multicast address filter
int sl_probe_atu_entry(uint32_t multicast_addr)
{
	return SWITCH_SUCCESS;
}

/******* Configuring QoS on a port on switch *******/
int sl_clear_switch_ip_priority_table(void)
{
	int i;

	// Clear all ip priority table. 0x10 ~ 0x17. Datasheet 2/3 for all marvel switch.
	for(i = SWITCH_GLOBAL_IP_PRI_MAPPING_REG0; i <= SWITCH_GLOBAL_IP_PRI_MAPPING_REG7; i++)
	{
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, i, 0);	// Clear tables.
	}
	return SWITCH_SUCCESS;
}

// clear global interrupt/status register
int sl_clear_global_interrupt1(void)
{
	uint16_t reg_val;
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL1_STATUS_REG, &reg_val);			// clear global interrupt 1 on switch mode.
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL2_INT_SRC_REG, &reg_val);			// clear global interrupt 2 on switch mode.
	return SWITCH_SUCCESS;
}

static  int sl_set_switch_88e6361_qos(int port_index)
{
	uint16_t reg_val;

	// IP priority mapping table is enough by defautl(defined in table126). It doesn't need extra configuration
	sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_88E6361_PORT_QUEUE_CONTROL_REG, SWITCH_88E6361_PORT_QUEUE_CONTROL_POINTER_PORT_SCHEDULE);
	sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_88E6361_PORT_QUEUE_CONTROL_REG, &reg_val);
	reg_val &= SWITCH_88E6361_PORT_QUEUE_CONTROL_DATA_MASK;
	reg_val &= (~SWITCH_88E6361_PORT_QUEUE_CONTROL_DATA_STRICT_MASK);
	reg_val |= (SWITCH_88E6361_PORT_QUEUE_CONTROL_UPDATE | SWITCH_88E6361_PORT_QUEUE_CONTROL_DATA_STRICT_FOR_ALL);
	sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_88E6361_PORT_QUEUE_CONTROL_REG, reg_val);

	// set initial priority value as "Use IP Priority"
	sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, &reg_val);
	reg_val &= SWITCH_PORT_IP_PRIORITY_MASK;
	reg_val &= SWITCH_PORT_TAGIFBOTH_MASK;
	sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, reg_val | SWITCH_PORT_IP_PRIORITY);
	sw_trace_print("%s : enable port %d for 88E6361\n", __func__, port_index);
	return SWITCH_SUCCESS;
}

/*
 * configure qos on a switch port
 * @port_index : switch port index 0-6
 */
int sl_set_switch_qos(int port_index)
{
	adapter_enum_t network_adapter;
	uint16_t reg_val;

	network_adapter = sl_get_chipset_info()->adapter_name;

	if( (network_adapter == SWITCH_88E6123) \
		|| (network_adapter == SWITCH_88E6161) )
	{
		/*
		 * Set PTP events as top priority on IP Priority table 1 - 0x11 (Diffserv - 0x38)
		 */
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_IP_PRI_MAPPING_REG1, PTP_EVENT_PRIORITY);

		// Set port scheduling mode & Use IP priority on all switch port
		// set up port scheduling mode on switch ports : strict priority scheme for all queues
		sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_EGRESS2_RATE_CONTROL_REG, &reg_val);
		reg_val &= SWITCH_PORT_EGRESS_RATE_MASK_BITS;
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_EGRESS2_RATE_CONTROL_REG, reg_val | SWITCH_PORT_EGRESS_RATE_STRICT_FOR_ALL);

		// set initial priority value as "Use IP Priority"
		sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, &reg_val);
		reg_val &= SWITCH_PORT_IP_PRIORITY_MASK;
		reg_val &= SWITCH_PORT_TAGIFBOTH_MASK;
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, reg_val | SWITCH_PORT_IP_PRIORITY | SWITCH_PORT_TAGIFBOTH);

		sw_trace_print("%s : enable port %d for 88E6123 & 88E6161\n",__func__, port_index);
	}
	else if (network_adapter == SWITCH_88E6361)
	{
		sl_set_switch_88e6361_qos(port_index);
	}
	else
	{
		// set up priority for 88E6350R, 88E6172, 88E6240, 88E6176, 88E6352, 88E6320, 88E6321 and default switch.
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_IP_MAPPING_TABLE_REG, (IP_MAPPING_TABLE_WRITE | DSCP_HIGH) );
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_IP_MAPPING_TABLE_REG, (IP_MAPPING_TABLE_WRITE | DSCP_MEDUIM) );
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_IP_MAPPING_TABLE_REG, (IP_MAPPING_TABLE_WRITE | DSCP_LOW) );

		// Set up All ports(0~6) as IPv4 or IPv6 IP Priority
		// QPri & FPri are from IP mapping table. (global 1 offset 0x19)
		// set up - port scheduling mode. (strict priority scheme for all queues)
		sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_EGRESS2_RATE_CONTROL_REG, &reg_val);
		reg_val &= SWITCH_PORT_EGRESS_RATE_MASK_BITS;
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_EGRESS2_RATE_CONTROL_REG, reg_val | SWITCH_PORT_EGRESS_RATE_STRICT_FOR_ALL);

		// set initial priority value as "Use IP Priority"
		sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, &reg_val);
		reg_val &= SWITCH_PORT_IP_PRIORITY_MASK;
		reg_val &= SWITCH_PORT_TAGIFBOTH_MASK;
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, reg_val | SWITCH_PORT_IP_PRIORITY);
		sw_trace_print("%s : enable port %d except 88E6123 & 88E6161\n",__func__, port_index);
	}
	return SWITCH_SUCCESS;
}

/******* Controlling switch/phy ports enabling/disabling *******/


/*
 * enable/disable a switch port
 * @port_index : switch port 0-6
 * @control : PORT_ENABLE or PORT_DISABLE
 */
int sl_ctrl_switch_port(int port_index, switch_port_ctl_s control)
{
	uint16_t reg_val;

	sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index,  SWITCH_PORT_CTRL_REG, &reg_val); // Read Switch port register

	if(control == PORT_ENABLE)
	{
		// port enabling
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index,  SWITCH_PORT_CTRL_REG, (reg_val|SWITCH_PORT_CTRL_FORWARDING));
	}
	else
	{
		// port disabling
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, (reg_val&SWITCH_PORT_CTRL_DISABLED));
	}
	return SWITCH_SUCCESS;
}

/*
 * enable a phys port
 * @port_index : switch port 0-4
 * @control : PORT_ENABLE or PORT_DISABLE
 */
int sl_ctrl_phy_port(int port_index, switch_port_ctl_s control)
{
	uint16_t reg_val;

	if(control == PORT_ENABLE)
	{
		// To power phy ports up, register 0.11 and 16._0.2 must be set to 0. Datasheet:part 3, page 40 : table 9.
		sl_read_smi(port_index, PHY_COPPER_SPECIFIC_CONTROL_REGISTER_OFFSET, &reg_val);	// Copper specific control register
		sl_write_smi(port_index, PHY_COPPER_SPECIFIC_CONTROL_REGISTER_OFFSET,(reg_val & (~PHY_COPPER_SPECIFIC_CONTROL_ENERGE_DETECT & ~PHY_COPPER_SPECIFIC_CONTROL_POWER_UP)));      // Energy detection disable, power normal operation.
		sl_read_smi(port_index, PHY_CONTROL_REGISTER_OFFSET, &reg_val);					// PHY control register
		sl_write_smi(port_index, PHY_CONTROL_REGISTER_OFFSET, (reg_val & ~PHY_CONTROL_POWER_DOWN));		// Power down to normal operation.
		sl_write_smi(port_index, PHY_CONTROL_REGISTER_OFFSET, (reg_val & ~PHY_CONTROL_POWER_DOWN )|PHY_CONTROL_SOFTWARE_RESET);	// software reset + restart copper negotiation.
	}
	else
	{
		sl_read_smi(port_index, PHY_CONTROL_REGISTER_OFFSET, &reg_val);				// Phy control register
		sl_write_smi(port_index, PHY_CONTROL_REGISTER_OFFSET,(reg_val | PHY_CONTROL_POWER_DOWN));	// PHY port power down
	}
	return SWITCH_SUCCESS;
}

/******* Disabling EEE on phy port *********/

/*
 * disable EEE on a switch phy port
 * This function is only for switch & 6352 & 6321 series
 */
int sl_disable_eee(int port_index)
{
	adapter_enum_t network_adapter;
	uint16_t eee_adv_value = SWITCH_SMI_PHY_EEE_ADV_REG;

	network_adapter = sl_get_chipset_info()->adapter_name;

	if (network_adapter == SWITCH_88E6361)
		eee_adv_value = SWITCH_88E6361_SMI_PHY_EEE_ADV_REG;

	// Support only switch 6352 & 6320 series
	if((network_adapter >= SWITCH_88E6172 && network_adapter <=SWITCH_88E6352) \
		|| (network_adapter >= SWITCH_88E6320 && network_adapter <=SWITCH_88E6321))
	{
		// Clause 22 access to clause 45MDIO MMD
		// Set address & device addr for EEE advertisement register.
		sl_write_smi(port_index, SWITCH_SMI_PHY_MMD_ACCESS_CTL_REG, SWITCH_SMI_PHY_EEE_ADV_DEV);
		// Set EEE advertisement register number
		sl_write_smi(port_index, SWITCH_SMI_PHY_MMD_ACCESS_ADDR_DATA_REG, eee_adv_value);
		// Set data cmd still
		sl_write_smi(port_index, SWITCH_SMI_PHY_MMD_ACCESS_CTL_REG, (SWITCH_SMI_PHY_MMD_ACCESS_FUNCTION_DATA|SWITCH_SMI_PHY_EEE_ADV_DEV));
		// Write 0 to disable 1G-T, 100-TX EEE advertisement option.
		sl_write_smi(port_index, SWITCH_SMI_PHY_MMD_ACCESS_ADDR_DATA_REG, 0);

		sw_trace_print("%s : disable for 6320 & 6352 series\n",__func__);
		return SWITCH_SUCCESS;
	}
	else return SWITCH_NOT_AVAILABLE_EEE;
}

/*
 * disable EEE on a PHY chipsets
 * This function is only for PHY chipsets which has EEE function
 */
int sl_phy_disable_eee(phy_ports_t phy_port)
{
	adapter_enum_t network_adapter;

	network_adapter = sl_get_chipset_info()->adapter_name;

	if(network_adapter == PHY_MARVEL_88E1510 || network_adapter == PHY_VSC8541
#ifdef PHY_MICREL_KSZ9131_PRODUCT_ID
		|| network_adapter == PHY_KSZ9131
#endif
	)
	{
		uint8_t phy_addr;

		if(phy_port == PHY_PRIMARY_PORT)
			phy_addr = sl_get_chipset_info()->phy_address[0];
		else
		{
			if(sl_get_chipset_info()->phy_redundant)
				phy_addr = sl_get_chipset_info()->phy_address[1];
			else return SWITCH_NOT_AVAILABLE_EEE;
		}

		// Clause 22 access to clause 45MDIO MMD
		// Set address & device addr for EEE advertisement register.
		sl_write_smi(phy_addr, SWITCH_SMI_PHY_MMD_ACCESS_CTL_REG, SWITCH_SMI_PHY_EEE_ADV_DEV);
		// Set EEE advertisement register number
		sl_write_smi(phy_addr, SWITCH_SMI_PHY_MMD_ACCESS_ADDR_DATA_REG, SWITCH_SMI_PHY_EEE_ADV_REG);
		// Set data cmd still
		sl_write_smi(phy_addr, SWITCH_SMI_PHY_MMD_ACCESS_CTL_REG, (SWITCH_SMI_PHY_MMD_ACCESS_FUNCTION_DATA|SWITCH_SMI_PHY_EEE_ADV_DEV));
		// Write 0 to disable 1G-T, 100-TX EEE advertisement option.
		sl_write_smi(phy_addr, SWITCH_SMI_PHY_MMD_ACCESS_ADDR_DATA_REG, 0);
		return SWITCH_SUCCESS;
	}
	else return PHY_ONLY_USE;
}

/******** Extra switch functions **********/
// set rgmi delay on internal RGMII ports
int sl_set_rgmi_delay(int port_index)
{
	uint16_t reg_val;
	adapter_enum_t switch_type;

	switch_type = sl_get_chipset_info()->adapter_name;

	// Set RGMII RX/TX delay for Port 5/6
	if( switch_type == SWITCH_88E6123 || switch_type == SWITCH_88E6161 )
	{
		// To give RGMII delay for port 5 - special approch for 88E6123 & 88E6161
		sl_write_smi(SWITCH_PORT_BASE_ADDR+4, SWITCH_88E6123_RGMII_HIDDEN_REG,0x81E7);	// open port 5 rgmii register
		sl_read_smi(SWITCH_PORT_BASE_ADDR+5, SWITCH_88E6123_RGMII_HIDDEN_REG,&reg_val);	// read port 5 rgmii register
		sl_write_smi(SWITCH_PORT_BASE_ADDR+5, SWITCH_88E6123_RGMII_HIDDEN_REG, reg_val | (SWITCH_88E6123_RGMII_RX_DELAY|SWITCH_88E6123_RGMII_TX_DELAY));   // write port 5 rgmii register
		sl_write_smi(SWITCH_PORT_BASE_ADDR+4, SWITCH_88E6123_RGMII_HIDDEN_REG,0xC1E7); // commmit rgmii delay value to register.

		sw_trace_print("%s : rgmi delay for 88E6123 & 88E6161\n",__func__);
	}
	else
	{
		// read port 5/6 rgmii register
		sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PHYSICAL_CTRL_REG,&reg_val);
		// Enable rx/tx delay of port 5/6
		sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PHYSICAL_CTRL_REG, SWITCH_PHYSICAL_CTRL_RGMII_DELAY|reg_val);
		sw_trace_print("%s : rgmi delay for port %d\n",__func__, port_index);
	}

	return SWITCH_SUCCESS;
}

//  set a port as control port - drop unknown multicast
int sl_set_switch_drop_unknown_multicast(int port_index)
{
	uint16_t reg_val;

	sl_read_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, &reg_val);

	// drop unknown multicast packets.
	reg_val &= (~SWITCH_DROP_UNKNOWN_MULTICAST_MASK_BIT);
	sl_write_smi(SWITCH_PORT_BASE_ADDR+port_index, SWITCH_PORT_CTRL_REG, reg_val | SWITCH_DROP_UNKNOWN_MULTICAST );

	// set broadcast enable
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL_MANAGEMENT_REG, &reg_val);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL_MANAGEMENT_REG, reg_val | SWITCH_FLOOD_BROADCAST );

	sw_trace_print("%s : port - %d\n",__func__, port_index);

	return SWITCH_SUCCESS;
}

/*
 * set ether type dsa tag on packets
 * @ether_type_dsa_value - ether type dsa value
 * @dst_cpu_port - destination cpu port number
 */
int sl_set_ethtype_dsa_tag(uint16_t ether_type_dsa_value, uint8_t dst_cpu_port)
{
	uint16_t reg_val, dst_port;

	if(dst_cpu_port != SWITCH_PORT_NUM_5 && dst_cpu_port != SWITCH_PORT_NUM_6 )
		return SWITCH_NOT_AVAILABLE_PORT;

	// Get cpu destination port number
	if(dst_cpu_port == SWITCH_PORT_NUM_5)
		dst_port = SWITCH_GLOBAL_1_CPU_DEST_PORT_5;
	else dst_port = SWITCH_GLOBAL_1_CPU_DEST_PORT_6;

	// set cpu destination port 5 (Global 1 register - offset 1A)
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_1_MONITOR_CONTROL_REG, &reg_val);
	reg_val &= ~SWITCH_GLOBAL_1_CPU_DEST_PORT_MASK;	// mask cpu target bits. 0xf
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_1_MONITOR_CONTROL_REG, reg_val | dst_port);	// Assign dst cpu port

	// set rsvd2CPU enable. (Global 2 register - offset 5)
	sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL_2_SWITCH_MANAGEMENT_REG, &reg_val);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL_2_SWITCH_MANAGEMENT_REG, reg_val | SWITCH_GLOBAL_2_RSVD2CPU_ENABLE);

	// Set port 5 as Ether type DSA tag mode for management packets.
	sl_read_smi(SWITCH_PORT_BASE_ADDR+dst_cpu_port, SWITCH_PORT_CTRL_REG, &reg_val);
	sl_write_smi(SWITCH_PORT_BASE_ADDR+dst_cpu_port, SWITCH_PORT_CTRL_REG, reg_val | SWITCH_PORT_FRAME_ETHER_TYPE_DSA_MODE);

	sl_read_smi(SWITCH_PORT_BASE_ADDR+dst_cpu_port, SWITCH_PORT_ETYPE_REG, &reg_val);
	sl_write_smi(SWITCH_PORT_BASE_ADDR+dst_cpu_port, SWITCH_PORT_ETYPE_REG, SWITCH_ETHER_TYPE_DSA_DEFAULT_VALUE);

	sw_trace_print("%s : enable port %d\n",__func__, dst_cpu_port);

	return SWITCH_SUCCESS;
}

/*
 * Get switch mode for a switch cpu-attached/test mode
 * @switch_cpu_attached - 1 : cpu attached mode, 0 : test mode.
 */
int sl_get_switch_mode(uint8_t *switch_cpu_attached)
{
	uint16_t reg, switch_mode_value;
	adapter_enum_t switch_type;

	switch_type = sl_get_chipset_info()->adapter_name;

	*switch_cpu_attached = 0;

	if(switch_type == SWITCH_88E6123 \
		|| switch_type == SWITCH_88E6161)
	{
		sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_STATUS_REG, &reg);  // read global status reg to get current switch mode.
		switch_mode_value = ((reg&SWITCH_MODE_BITS)>>12);

		if(switch_mode_value == SWITCH_6123_6161_CPU_ATTACHED_MODE)
			*switch_cpu_attached = 1;
	}
	else if( switch_type == SWITCH_88E6351 \
		|| switch_type == SWITCH_88E6350R )
	{
		sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_STATUS_REG, &reg);  // read global status reg to get current switch mode.
		switch_mode_value = ((reg&SWITCH_MODE_BITS)>>12);

		if(switch_mode_value == SWITCH_6350_6351_6350R_CPU_ATTACHED_MODE)
			*switch_cpu_attached = 1;
	}
	// Check NO CPU config for 88E6320 series and 6352 series
	else if(switch_type >= SWITCH_88E6172 && switch_type <= SWITCH_88E6321)
	{
		/*
		 * Get switch scratch misc register - config1 reg value
		 */
		// set config1 register pointer
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SCRATCH_MISC_REG, SWITCH_SCRATCH_MISC_CONFIG1);
		sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SCRATCH_MISC_REG, &reg);

		if((switch_type == SWITCH_88E6361) && (reg&SWITCH_88E6320SERIES_88E6352SERIES_CPU_ATTACHED))
		{
			*switch_cpu_attached = 1;
		}
		// Switch 88E6320 series and others
		else if(!(reg&SWITCH_88E6320SERIES_88E6352SERIES_CPU_ATTACHED))
		{
			// CPU attached & need to power up phy/switch ports
			*switch_cpu_attached = 1;
		}
	}
	else
	{
		// unknown switch? init anyway.
		*switch_cpu_attached = 1;
	}

	sw_trace_print("%s : %s\n",__func__, *switch_cpu_attached ? "Cpu attached mode" : "Test mode");

	return SWITCH_SUCCESS;
}

/*
 * !!Warning!! do not change switch_port_cmode_s order. These values are all identical as switch port cmode status"
 * - 88E6320/6321/6361 series has 4 bits.
 * - 88E6350R/6351 series have 3 bits
 * - 88E6352/6240/7167/6172 have 4 bits (similar to 6320 series except bit 0)
 */
typedef enum
{
	FD_MII,
	MII_PHY,
	MII_MAC_OR_TOPHY,
	GMII_OR_TOPHY,
	RMII_PHY_OR_TOPHY,
	RMII_MAC_OR_TOPHY,
	XMII_TRISTATE,
	RGMII_OR_TOPHY,
	/*
	 * 0~7 are for only port 2 or 5 or 6
	 */
	SW_100BASE_FX,
	SW_1000BASE_X,
	SGMII,
	RESERVED1,
	RESERVED2,
	RESERVED3,
	RESERVED4,
	PHY
} switch_6320_6352_port_cmode_s;

typedef enum
{
	GMII, // 88E6350R - disabled.
	RGMII,
	MII,
	MII_2,
	DISABLED1,
	DISABLED2,
	DISABLED3,
	DISABLED4
} switch_6350_port_cmode_s;

/*
 * clear all interupts on serdes port
 * port index should be calculated to absolute index.
 */
int sl_get_switch_port_cmode(int switch_port, switch_port_cmode_t *port_cmode)
{
	adapter_enum_t switch_name;
	uint16_t reg_val;

	*port_cmode = SWITCH_PORT_CMODE_DISABLED;

	switch_name = sl_get_chipset_info()->adapter_name;

	sl_read_smi(SWITCH_PORT_BASE_ADDR + switch_port, SWITCH_PORT_STAT_REG, &reg_val ); // clear interrupt on SERDES level.

	if(switch_name == SWITCH_88E6350R || switch_name == SWITCH_88E6351)
	{
		reg_val = SWITCH_PORT_STATUS_GET_6350R_CMODE(reg_val);

		if(switch_name == SWITCH_88E6350R && reg_val == GMII)
		{
			*port_cmode = SWITCH_PORT_CMODE_DISABLED;
		}
		else
		{
			switch(reg_val)
			{
				case GMII:
				case RGMII:
				case MII:
				case MII_2:
					*port_cmode = SWITCH_PORT_CMODE_ENABLED;
					break;

				default : *port_cmode = SWITCH_PORT_CMODE_DISABLED;
					break;
			}
		}
	}
	else if(switch_name >= SWITCH_88E6172 && switch_name <= SWITCH_88E6321)
	{
		reg_val = SWITCH_PORT_STATUS_GET_6320_6352_CMODE(reg_val);

		if((switch_name >= SWITCH_88E6172 && switch_name <= SWITCH_88E6352) && reg_val == FD_MII)
		{
			*port_cmode = SWITCH_PORT_CMODE_DISABLED;
		}
		else
		{
			switch(reg_val)
			{
				case FD_MII:
				case MII_PHY:
				case MII_MAC_OR_TOPHY:
				case GMII_OR_TOPHY:
				case RMII_PHY_OR_TOPHY:
				case RMII_MAC_OR_TOPHY:
				case XMII_TRISTATE:
				case RGMII_OR_TOPHY:
					*port_cmode = SWITCH_PORT_CMODE_ENABLED;
					break;

				default : *port_cmode = SWITCH_PORT_CMODE_DISABLED;
					break;
			}
		}
	}
	else return SWITCH_FAILURE;

	return SWITCH_SUCCESS;
}

/*
 * Power up all serdes ports within vlan config
 * Note - if serdes port is disabled, then it power up.
 */
int sl_serdes_ports_up_all(uint32_t vlan_config)
{
	adapter_enum_t switch_name;
	uint8_t i, serdes_base_reg, serdes_page_num;
	uint8_t num_serdes_ports, serdes_port_starts;
	uint16_t reg, value;

	switch_name = sl_get_chipset_info()->adapter_name;

	// do we have any serdes ports on detected switch ?
	if(!sl_get_chipset_info()->switch_ports_info.num_serdes_ports)
		return SWITCH_SUCCESS;

	// Get base address for serdes register
	serdes_base_reg = sl_get_serdes_reg_index(switch_name);

	num_serdes_ports = sl_get_chipset_info()->switch_ports_info.num_serdes_ports;
	serdes_port_starts = sl_get_chipset_info()->switch_ports_info.num_serdes_port_start;

	switch(switch_name)
	{
		case SWITCH_88E6172:
		case SWITCH_88E6176:
		case SWITCH_88E6352:
		case SWITCH_88E6320:
		case SWITCH_88E6321:
			serdes_page_num = 1; break;
		default: serdes_page_num = 0; break;
	}

	if(switch_name == SWITCH_88E6361)
	{
		for(i=0; i<num_serdes_ports; i++)
		{
			if(!((vlan_config >> ((i+serdes_port_starts)*4)) & 0xf))
			{
				// not being used & do not power up
				continue;
			}

			sl_read_reg_clause45(SWITCH_88E6361_SERDES_DEVICE_ADDR, serdes_base_reg + i, SWITCH_88E6361_SERDES_CONTROL_REG, &reg);

			// Is serdes port power down status ?
			if(reg & SWITCH_SERDES_CONTROL_POWER_DOWN)
			{
				sl_read_reg_clause45(SWITCH_88E6361_SERDES_DEVICE_ADDR, serdes_base_reg + i, SWITCH_88E6361_SERDES_PORT_OPERATION_CONFIG_REG, &value);
				value &= ~(1<<5);
				value |= PHY_CONTROL_SOFTWARE_RESET;
				// workaround for 6361 to clear PwrDn bit
				sl_write_reg_clause45(SWITCH_88E6361_SERDES_DEVICE_ADDR, serdes_base_reg + i, SWITCH_88E6361_SERDES_PORT_OPERATION_CONFIG_REG, value);

				reg &= ~SWITCH_SERDES_CONTROL_POWER_DOWN;
				reg |= PHY_CONTROL_SOFTWARE_RESET;
				// change to normal status
				sl_write_reg_clause45(SWITCH_88E6361_SERDES_DEVICE_ADDR, serdes_base_reg + i, SWITCH_88E6361_SERDES_CONTROL_REG, reg);
			}
		}
		return SWITCH_SUCCESS;
	}

	for(i=0; i<num_serdes_ports; i++)
	{
		if(!((vlan_config >> ((i+serdes_port_starts)*4)) & 0xf))
		{
			// not being used & do not power up
			continue;
		}

		// Except 88E6123 & 88E6161, need to check switch serdes ports page number
		if(switch_name != SWITCH_88E6123 && switch_name != SWITCH_88E6161)
		{
			// Page register number is correct for handling serdes register?
			sl_read_smi((serdes_base_reg+i), SWITCH_PHY_PAGE_ADDR_REG, &reg);
			if(serdes_page_num != reg)
			{
				// change page register to nominated number
				sl_write_smi((serdes_base_reg+i), SWITCH_PHY_PAGE_ADDR_REG, serdes_page_num);
			}
		}

		// Power up for page register
		sl_read_smi((serdes_base_reg+i),SWITCH_SERDES_CONTROL_REG, &reg);

		// Is serdes port power down status ?
		if(reg & SWITCH_SERDES_CONTROL_POWER_DOWN)
		{
			reg &= ~SWITCH_SERDES_CONTROL_POWER_DOWN;
			sl_write_smi(serdes_base_reg+i, SWITCH_SERDES_CONTROL_REG, reg); // change to normal status
			sw_trace_print("%s : port - %x up\n", __func__, (serdes_base_reg+i));
		}
	}

	return SWITCH_SUCCESS;
}

/******* Disabling EEE on phy port *********/
/*
 * Disabling EEE based on phy level
 * Note : This applys for only 6352 & 6320 platforms
 */
int sl_disable_eee_all(uint32_t vlan_config)
{
	adapter_enum_t switch_type;
	int i, phy_port_start, phy_port_mum;
	uint16_t eee_adv_value = SWITCH_SMI_PHY_EEE_ADV_REG;

	switch_type = sl_get_chipset_info()->adapter_name;
	phy_port_start = sl_get_chipset_info()->switch_ports_info.num_phy_port_start;
	phy_port_mum = sl_get_chipset_info()->switch_ports_info.num_phy_ports;

	if (switch_type == SWITCH_88E6361)
		eee_adv_value = SWITCH_88E6361_SMI_PHY_EEE_ADV_REG;

	// Disabling EEE support option for 6352 series & 6320 series
	if( (switch_type >= SWITCH_88E6172) && (switch_type <= SWITCH_88E6321) )
	{
		// disabling EEE advertisement options for phy ports 0 ~ 4
		for(i = phy_port_start; i < (phy_port_mum+phy_port_start); i++)
		{
			// Clause 22 access to clause 45MDIO MMD
			// Set address & device addr for EEE advertisement register.
			sl_write_smi(i, SWITCH_SMI_PHY_MMD_ACCESS_CTL_REG, SWITCH_SMI_PHY_EEE_ADV_DEV);
			// Set EEE advertisement register number
			sl_write_smi(i, SWITCH_SMI_PHY_MMD_ACCESS_ADDR_DATA_REG, eee_adv_value);
			// Set data cmd still
			sl_write_smi(i, SWITCH_SMI_PHY_MMD_ACCESS_CTL_REG, (SWITCH_SMI_PHY_MMD_ACCESS_FUNCTION_DATA|SWITCH_SMI_PHY_EEE_ADV_DEV));
			// Write 0 to disable 1G-T, 100-TX EEE advertisement option.
			sl_write_smi(i, SWITCH_SMI_PHY_MMD_ACCESS_ADDR_DATA_REG, 0);
			sw_trace_print("%s : port %d disabled\n", __func__, i);
		}
	}

	return SWITCH_SUCCESS;
}

int sl_set_default_vlan_id_port(uint16_t vid_value, uint8_t port_number)
{
	sl_write_smi(SWITCH_PORT_BASE_ADDR + port_number,  SWITCH_PORT_DEFAULT_VLANID_REG, vid_value);
	return SWITCH_SUCCESS;
}

int sl_set_default_vlan_id_port_all(uint16_t vid_value, uint32_t vlan_config)
{
	int i;

	for(i = 0; i < sl_get_max_switch_port_number(); i++)
	{
		if(vlan_config & (0xf << i*4))
		{
			sl_set_default_vlan_id_port(vid_value, i);
		}
	}

	return SWITCH_SUCCESS;
}

int sl_set_default_fowarding_id_port(uint16_t vid_value, uint8_t port_number)
{
	uint16_t mdio_reg;
	sl_read_smi(SWITCH_PORT_BASE_ADDR + port_number,  SWITCH_PORT_VLANT_REG, &mdio_reg);
	sl_write_smi(SWITCH_PORT_BASE_ADDR + port_number,  SWITCH_PORT_VLANT_REG, (SWITCH_PORT_VLANT_DEFAULT_FID(vid_value) | mdio_reg));
	return SWITCH_SUCCESS;
}

int sl_set_default_fowarding_id_port_all(uint16_t vid_value, uint32_t vlan_config)
{
	int i;

	for(i = 0; i< sl_get_max_switch_port_number(); i++)
	{
		if(vlan_config & (0xf << i*4))
		{
			sl_set_default_fowarding_id_port(vid_value, i);
		}
	}

	return SWITCH_SUCCESS;
}

/*
 * Set all vlan config to default
 */
int sl_set_switch_vlan_default(void)
{
	uint8_t i;
	uint32_t all_port_mask = ((1 << sl_get_max_switch_port_number()) - 1);

	// set up vlan configuration to switch.
	for (i = 0; i < sl_get_max_switch_port_number(); i++)
	{
		// default vlan value for each switch port
		sl_set_switch_vlan_config(i, (all_port_mask & ~(1<<sl_get_physical_port_id(i))));
		// Removing marvel header
		sl_set_switch_port_marvel_header(i, PORT_DISABLE);
	}

	return SWITCH_SUCCESS;
}

/*
 * Enable 802.1Q for given vlan config which including only same type
 */
int sl_set_802_1Q_enable(uint16_t vid_value, uint32_t vlan_config)
{
	uint16_t ctr2_reg;
	int i;

	for(i = 0; i< sl_get_max_switch_port_number(); i++)
	{
		if(vlan_config & (0xf << i*4))
		{
			sl_read_smi(SWITCH_PORT_BASE_ADDR + i,  SWITCH_PORT_CTRL_2_REG, &ctr2_reg);
			ctr2_reg &= SWITCH_PORT_CTRL_2_802_1Q_MODE_MASK;
			sl_write_smi(SWITCH_PORT_BASE_ADDR + i,  SWITCH_PORT_CTRL_2_REG, ctr2_reg | SWITCH_PORT_CTRL_2_802_1Q_MODE_SECURE);
		}
	}

	return SWITCH_SUCCESS;
}

/*
 * Load STU entry for ports within VTU entry
 */
int sl_load_stu_entry(uint16_t vid_value)
{
	int retry_count = 10;	// retrying mdio 10ms.
	uint16_t data_temp;

	while(retry_count--)
	{
		// Checking busy flag
		sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_VTU_OPERATION_REG, &data_temp);
		if( !(data_temp&SWITCH_GLOBAL_VTU_OP_VTUBUSY) )
		{
			break;
		}

		sl_nop_delay_loop(1000);	// delay 1ms
	}

	if(!retry_count)
	{
		sw_trace_print("%s : load stu entry timed out for vlan number - %d\n", __func__, vid_value);
		return SWITCH_MDIO_TIMEOUT;
	}

	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_DATA_PORTS_0TO3_REG, 0);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_DATA_PORTS_4TO6_REG, 0);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_DATA_VTU_OPERATION_REG, 0);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_SID_REG, vid_value);
	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_VID_REG, SWITCH_GLOBAL_VTU_VID_VALID);

	sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_OPERATION_REG, (SWITCH_GLOBAL_VTU_OP_VTUBUSY|SWITCH_GLOBAL_VTU_OP_STUOP_LOAD));

	return SWITCH_SUCCESS;
}

/*
 * Adding VTU entry with given vlan ports infor
 */
int sl_load_vtu_entry(uint16_t vid_value, uint32_t vlan_config)
{
	int i, retry_count = 10, port_id;	// retrying mdio 10ms.
	uint16_t vtu_data_port_0to3, vtu_data_port_4to6, data_temp;
	uint16_t vtu_data_port_0to7, vtu_data_port_8to10;
	adapter_enum_t switch_type = sl_get_chipset_info()->adapter_name;

	vtu_data_port_0to3 = vtu_data_port_4to6 = vtu_data_port_0to7 = vtu_data_port_8to10 = 0;

	while(retry_count--)
	{
		// Checking busy flag
		sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_VTU_OPERATION_REG, &data_temp);
		if( !(data_temp&SWITCH_GLOBAL_VTU_OP_VTUBUSY) )
		{
			break;
		}

		sl_nop_delay_loop(1000);	// delay 1ms
	}

	if(!retry_count)
	{
		sw_trace_print("%s : load vtu entry timed out for vlan number - %d\n", __func__, vid_value);
		return SWITCH_MDIO_TIMEOUT;
	}

	for(i = 0; i< sl_get_max_switch_port_number(); i++)
	{
		if (switch_type != SWITCH_88E6361)
		{
			if( i < 4 )
			{
				if(vlan_config & (0xf << (i*4)))
					vtu_data_port_0to3 |= (SWITCH_GLOBAL_VTU_PORTS_MEMBER_EGRESS_UNMODIFIED << (i*4));
				else vtu_data_port_0to3 |= (SWITCH_GLOBAL_VTU_PORTS_NOMEMBER_NOALLOW_ANYFRAMES << (i*4));
			}
			else
			{
				if(vlan_config & (0xf << (i*4)))
					vtu_data_port_4to6 |= (SWITCH_GLOBAL_VTU_PORTS_MEMBER_EGRESS_UNMODIFIED << ((i-4)*4));
				else vtu_data_port_4to6 |= (SWITCH_GLOBAL_VTU_PORTS_NOMEMBER_NOALLOW_ANYFRAMES << ((i-4)*4));
			}
		}
		else
		{
			port_id = sl_get_physical_port_id(i);
			if( i < 7 )
			{
				if(vlan_config & (0xf << (port_id*4)))
					vtu_data_port_0to7 |= (SWITCH_GLOBAL_VTU_PORTS_MEMBER_EGRESS_UNMODIFIED << (port_id*2));
				else vtu_data_port_0to7 |= (SWITCH_GLOBAL_VTU_PORTS_NOMEMBER_NOALLOW_ANYFRAMES << (port_id*2));
			}
			else
			{
				if(vlan_config & (0xf << (port_id*4)))
					vtu_data_port_8to10 |= (SWITCH_GLOBAL_VTU_PORTS_MEMBER_EGRESS_UNMODIFIED << ((port_id-7)*2));
				else vtu_data_port_8to10 |= (SWITCH_GLOBAL_VTU_PORTS_NOMEMBER_NOALLOW_ANYFRAMES << ((port_id-7)*2));
			}
		}
	}

	if (switch_type != SWITCH_88E6361)
	{
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_VID_REG, (SWITCH_GLOBAL_VTU_VID_VALID|vid_value));
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_DATA_PORTS_0TO3_REG, vtu_data_port_0to3);
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_DATA_PORTS_4TO6_REG, vtu_data_port_4to6);
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_DATA_VTU_OPERATION_REG, 0);
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_FID_REG, vid_value);
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_OPERATION_REG, (SWITCH_GLOBAL_VTU_OP_VTUBUSY|SWITCH_GLOBAL_VTU_OP_VTUOP_LOAD));
	}
	else
	{
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_VID_REG, (SWITCH_GLOBAL_VTU_VID_VALID|vid_value));
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_DATA_PORTS_0TO3_REG, vtu_data_port_0to7);
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_DATA_PORTS_4TO6_REG, vtu_data_port_8to10);
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_FID_REG, vid_value);
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_OPERATION_REG, (SWITCH_GLOBAL_VTU_OP_VTUBUSY|SWITCH_GLOBAL_VTU_OP_VTUOP_LOAD));
	}

	return SWITCH_SUCCESS;
}


/*
 * Configuring vlan configuration on ports
 * set up vlan configuration based on vlan config
 * If there is user vlan configration, then need to enable 802.1Q and VTU adding
 */
int sl_set_switch_vlan_config_all(uint32_t vlan_config)
{
	int i, j, reg_value;
	uint8_t vlan_port, pri_vlan, sec_vlan, user_vlan2, user_vlan3;

	pri_vlan = sec_vlan = user_vlan2 = user_vlan3 = 0;

	// vlan set up - get each port configuration value.
	for( i=0; i<sl_get_max_switch_port_number(); i++)
	{
		reg_value = 0;
		vlan_port =(vlan_config >> (4*i)) & 0xf;	// Get each port vlan config

		if( vlan_port )	// check if it has any vlan configuration.
		{
			if((vlan_port & ETH_SWITCH_VLAN_PRI) == ETH_SWITCH_VLAN_PRI)
				pri_vlan ++;
			if((vlan_port & ETH_SWITCH_VLAN_SEC) == ETH_SWITCH_VLAN_SEC)
				sec_vlan ++;
			if((vlan_port & ETH_SWITCH_VLAN_2) == ETH_SWITCH_VLAN_2)
				user_vlan2 ++;
			if((vlan_port & ETH_SWITCH_VLAN_3) == ETH_SWITCH_VLAN_3)
				user_vlan3 ++;

			for( j=0; j< sl_get_max_switch_port_number(); j++ )
			{
				// Don't compare same port.
				if(i == j)
					continue;

				{
					unsigned char vlan_setup = ETH_SWITCH_VLAN_PRI;
					// search ETH_SWITCH_VLAN_PRI ~ ETH_SWITCH_VLAN_3 & extract vlan configuration values.
					while(vlan_setup <= ETH_SWITCH_VLAN_3)
					{
						// check start from primary to vlan_3.
						if( (vlan_port & vlan_setup) && (((vlan_config >> (4*j)) & 0xf) & vlan_setup) )
						{
							reg_value |= (1<<sl_get_physical_port_id(j));
						}
						vlan_setup <<= 1; //check next vlan flag.
					}
				}
			}
		}

		// disable switch learning
		// reg_temp = (1<<11);

		sl_write_smi(SWITCH_PORT_BASE_ADDR + i,  SWITCH_PORT_VLANT_REG, reg_value);
		sw_trace_print("%s : port %d - vlan reg - %4x\n", __func__, i, reg_value);
	}

	// Do we have user vlan & any primary or secondary vlans?
	if(user_vlan2 || user_vlan3)
	{
		/*
		 * Handling multiple vlans
		 * 1. adding VTU entry with given vlan ports (Global register 1)
		 * 2. Enable 802.1Q modes for vlan ports
		 */
		unsigned char vlan_type;
		uint32_t vlan_config_temp;
		uint16_t vlan_vid = SWITCH_DEFAULT_VLAN_INDEX_USER_VLAN_2;	// vlan start from 2 for user vlan configuration

		for(vlan_type = ETH_SWITCH_VLAN_2; vlan_type <= ETH_SWITCH_VLAN_3; (vlan_type <<= 1), vlan_vid++)
		{
			vlan_config_temp = vlan_config;

			for( j=0; j< sl_get_max_switch_port_number(); j++ )
			{
				vlan_config_temp &= (0xFFFFFFFF & ~(0xf << (4*j))) | (vlan_type << (4*j));
			}

			if(vlan_config_temp)
			{
				if(vlan_type == ETH_SWITCH_VLAN_SEC)
				{
#if 0
					// Assign only default vlan for secondary - primary use 1 as default.
					// don't do seconday now
					if(sec_vlan)
						sl_set_default_vlan_id_port_all(vlan_vid, vlan_config_temp);
#endif
				}
				else
				{
					adapter_enum_t switch_type;
					switch_type = sl_get_chipset_info()->adapter_name;

					// need to do work around to use VTU load on this switch chipsets.
					if(switch_type == SWITCH_88E6352 \
						|| switch_type == SWITCH_88E6172 \
						|| switch_type == SWITCH_88E6176)
					{
						// STU load for VTU load ,disabled 802.1s for all ports in VTU
						sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_SID_REG, vlan_vid);
						sl_load_stu_entry(vlan_vid);
					}
					else
					{
						// STU set as 0
						sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1,  SWITCH_GLOBAL_VTU_SID_REG, 0);
					}

					sl_set_802_1Q_enable(vlan_vid, vlan_config_temp);
					sl_set_default_vlan_id_port_all(vlan_vid, vlan_config_temp);
					sl_load_vtu_entry(vlan_vid, vlan_config_temp);
				}
			}
		}
	}
	return SWITCH_SUCCESS;
}

/******* Static ATU multicast filtering entrys management *******/
// add multicast filter list
int sl_set_multicast_filtering(uint8_t num_multicast, uint32_t *multicast_addr, uint32_t vlan_config, uint8_t ctl_port_map)
{
	int i;
	uint8_t vlan_port;
	uint16_t dpv, pri_sec_bitmap, user_vlan2_bitmap, user_vlan3_bitmap;
	pri_sec_bitmap = user_vlan2_bitmap = user_vlan3_bitmap = 0;

	/* For each multicast address
	 * add a ATU entry that enables forwarding of this multicast address to every port
	 * disabled ports are dont care
	 */

	// vlan set up - get each port configuration value.
	for( i = 0; i < sl_get_max_switch_port_number(); ctl_port_map >>= 1, i++)
	{
		vlan_port = (vlan_config >> (4*i)) & 0xf;	// Get each port vlan config

		if((vlan_port & ETH_SWITCH_VLAN_2) == ETH_SWITCH_VLAN_2)
			user_vlan2_bitmap |= (1 << sl_get_physical_port_id(i));

		if((vlan_port & ETH_SWITCH_VLAN_3) == ETH_SWITCH_VLAN_3)
			user_vlan3_bitmap |= (1 << sl_get_physical_port_id(i));
	}

	// Getting DPV value
	dpv = sl_get_switch_default_dpv();

	// Calculating multicast forwarding bitmap for pri & sec
	pri_sec_bitmap = dpv & ~(user_vlan2_bitmap | user_vlan3_bitmap);

	for( i = 0; i < num_multicast; i++)
	{
		// Adding one entry to ATU
		sl_manage_atu_entry( multicast_addr[i], ATU_ENTRY_LOAD, pri_sec_bitmap, 0);
		if(user_vlan2_bitmap)
		{
			sl_manage_atu_entry( multicast_addr[i], ATU_ENTRY_LOAD, user_vlan2_bitmap, SWITCH_DEFAULT_VLAN_INDEX_USER_VLAN_2);
			sw_trace_print("%s : adding ATU for User VLAN2 FID database - %d\n", __func__, SWITCH_DEFAULT_VLAN_INDEX_USER_VLAN_2);
		}
		if(user_vlan3_bitmap)
		{
			sl_manage_atu_entry( multicast_addr[i], ATU_ENTRY_LOAD, user_vlan3_bitmap, SWITCH_DEFAULT_VLAN_INDEX_USER_VLAN_3);
			sw_trace_print("%s : adding ATU for User VLAN3 FID database - %d\n", __func__, SWITCH_DEFAULT_VLAN_INDEX_USER_VLAN_3);
		}
	}

	return SWITCH_SUCCESS;
}

/******* Getting default DPV value for switch *******/
uint16_t sl_get_switch_default_dpv(void)
{
		// Getting DPV value
	switch(sl_get_chipset_info()->adapter_name)
	{
		case SWITCH_88E6123:
			return SWITCH_88E6123_FORWARDING_ALL_PORTS;
		case SWITCH_88E6161:
			return SWITCH_88E6161_FORWARDING_ALL_PORTS;
		case SWITCH_88E6350R:
			return SWITCH_88E6350R_FORWARDING_ALL_PORTS;
		case SWITCH_88E6351:
			return SWITCH_88E6351_FORWARDING_ALL_PORTS;
		case SWITCH_88E6172:
		case SWITCH_88E6176:
		case SWITCH_88E6352: // These series have same number of ports.
			return SWITCH_88E6172_FORWARDING_ALL_PORTS;
		case SWITCH_88E6320:
		case SWITCH_88E6321:
			return SWITCH_88E6320_FORWARDING_ALL_PORTS;
		case SWITCH_88E6361:
			return SWITCH_88E6361_FORWARDING_ALL_PORTS;
		default : return SWITCH_DEFAULT_FORWARDING_ALL_PORTS; // Default is 6350R

	}
}

/******* Configuring QoS on a port on switch *******/
// configure qos for all ports on a switch
int sl_set_switch_qos_all(void)
{
	adapter_enum_t switch_type;
	uint8_t switch_port_num;
	int i;

	switch_type = sl_get_chipset_info()->adapter_name;
	switch_port_num = sl_get_chipset_info()->switch_ports_info.num_switch_ports;

	if (switch_type != SWITCH_88E6361)
	{
		// Clear all ip priority table. 0x10 ~ 0x17. Datasheet 2/3 for all marvel switch.
		for(i = SWITCH_GLOBAL_IP_PRI_MAPPING_REG0; i <= SWITCH_GLOBAL_IP_PRI_MAPPING_REG7; i++)
		{
			sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, i, 0);	// Clear tables.
		}
	}

	if( (switch_type == SWITCH_88E6123) || (switch_type == SWITCH_88E6161) )
	{
		/*
		 * Set PTP events as top priority on IP Priority table 1 - 0x11 (Diffserv - 0x38)
		 */
		sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_1, SWITCH_GLOBAL_IP_PRI_MAPPING_REG1, PTP_EVENT_PRIORITY);

		// Set port scheduling mode & Use IP priority on all switch port
		for( i= 0; i<switch_port_num; i ++ )
		{
			// In case 88E123, configure port 5 instead port 2.
			if(i == SWITCH_PORT_NUM_2 && switch_type == SWITCH_88E6123)
				i = SWITCH_PORT_NUM_5;

			sl_set_switch_qos(i);
		}
	}
	else
	{
		for( i= 0; i<switch_port_num; i ++ )
		{
			sl_set_switch_qos(i);
		}
	}
	return SWITCH_SUCCESS;
}

/******* Controlling switch/phy ports enabling/disabling *******/
// disable all switch phys ports
int sl_disable_switch_phy_port_all(void)
{
	int i;
	uint8_t phy_port_start, phy_ports_num;

	phy_port_start = sl_get_chipset_info()->switch_ports_info.num_phy_port_start;
	phy_ports_num = sl_get_chipset_info()->switch_ports_info.num_phy_ports;

	for(i = phy_port_start; i < phy_port_start + phy_ports_num; i++)
	{
		sw_trace_print("%s : switch port - %d - disabled\n", __func__, i);
		sl_ctrl_switch_port(i, PORT_DISABLE);
	}

	return SWITCH_SUCCESS;
}


// enable all switch ports based on vlan configuration
int sl_enable_switch_all(uint32_t vlan_config)
{
	int port_index = 0;

	while(vlan_config)
	{
		// Calculate avaluable phy port number
		// disable ports which are not being used.
		if(!(vlan_config & 0xf))
		{
			// not being used & do not power up
			// if the port doesn't belong to any vlan group, then disable that port.
			// port disable
			sw_trace_print("switch port - %d - disabled\n", port_index);
			sl_ctrl_switch_port(port_index, PORT_DISABLE);

		}
		else
		{
			sw_trace_print("switch port - %d - enabled\n", port_index);
			sl_ctrl_switch_port(port_index, PORT_ENABLE);
		}

		vlan_config >>= 4;
		port_index++;
	}

	return SWITCH_SUCCESS;
}

// enable all phys ports on a switch based on vlan config
int sl_enable_phy_all(uint32_t vlan_config)
{
	int i;
	uint8_t cpu_attached_mode = 0, phy_port_start, phy_port_num;

	phy_port_start  = sl_get_chipset_info()->switch_ports_info.num_phy_port_start;
	phy_port_num = sl_get_chipset_info()->switch_ports_info.num_phy_ports;

	sl_get_switch_mode(&cpu_attached_mode);

	sw_trace_print("Switch CPU attached mode - %d\n", cpu_attached_mode);

	for(i =phy_port_start; i< (phy_port_num+phy_port_start); i++)
	{
		// disable ports which are not being used.
		if(!((vlan_config >> (i*4)) & 0xf))
		{
			// not being used & do not power up
			sw_trace_print("PHY port %d - disabled\n", i);
			continue;
		}

		// If it is only cpu attached mode.
		if(cpu_attached_mode)
		{
			sw_trace_print("PHY port %d - enabled\n", i);
			sl_ctrl_phy_port(i, PORT_ENABLE);
		}
	}
	return SWITCH_SUCCESS;
}

// enable safesafe port only and disabling all other ports.
int sl_enable_failsafe(int port_index)
{
	return SWITCH_SUCCESS;
}

/*
 * Disabling PHY copper port 1G advertisement
 */
int sl_disable_copper_1G_advert(int port_index)
{
	uint16_t reg_data;

	sl_read_smi(port_index, SWTICH_PHY_PORT_1G_CTL_REG, &reg_data);
	SWTICH_PHY_PORT_1G_ADVERT_DISABLE(reg_data);
	sl_write_smi(port_index, SWTICH_PHY_PORT_1G_CTL_REG, reg_data);
	
	return SWITCH_SUCCESS;
}

/*
 * Disabling PHY copper port 1G advertisement with give vlan configuration
 */
int sl_disable_copper_1G_advert_all(uint32_t vlan_config)
{
	int i;
	uint8_t phy_port_start, phy_port_num;

	// don't change port 5
	vlan_config &= 0x000FFFFF;

	phy_port_start  = sl_get_chipset_info()->switch_ports_info.num_phy_port_start;
	phy_port_num = sl_get_chipset_info()->switch_ports_info.num_phy_ports;

	for(i =phy_port_start; i< (phy_port_num+phy_port_start); i++)
	{
		// disable ports which are not being used.
		if(((vlan_config >> (i*4)) & 0xf))
		{
			sw_trace_print("PHY port %d - 1G advert disabled\n", i);

			// disable 1G advertise for this copper port
			sl_disable_copper_1G_advert(i);
		}
	}
	
	return SWITCH_SUCCESS;
}

/*
 * This is rgmi delay for ksz9031
 * RGMII 1.38ns delay (-0.42ns for data and 0.96 for clock).
 * MMD Address 2h, Register 4h --RGMII Control signal PAD skew : 0x0
 * MMD Address 2h, Register 5h --RGMII RX Data PAD skew : 0x0
 * MMD Address 2h, Register 6h --RGMII TX Data PAD skew : 0x0
 * MMD Address 2h, Register 8h --RGMII Clock PAD skew : 0x3ff
 */
int sl_set_rgmi_delay_ksz9031(phy_ports_t phy_port)
{
	uint8_t phy_addr;

	if( sl_get_chipset_info()->adapter_name != PHY_KSZ9031 )
	{
		return SWITCH_FAILURE;
	}

	if(phy_port == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	// setup register address for MMD - device address 2h
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_REG) | MICREL_MMD_DEVICE_ADDR2);
	// setup register address for MMD - device address 2h
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_CTL_SIGNAL_PAD_SKEW_REG);
	// select register data for MMD - device address 2h, register 4
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	// write value 0x0 to MMD
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_CTL_SIGNAL_PAD_SKEW_REG_VALUE);

	// select register data for MMD - device address 2h, register 5
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	// write value 0x0 to MMD
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_RX_DATA_PAD_SKEW_REG_VALUE);

	// select register data for MMD - device address 2h, register 6
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	// write value 0x0 to MMD
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_TX_DATA_PAD_SKEW_REG_VALUE);

	// select register data for MMD - device address 2h, register 7 - write dummy
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	// write value 0x0 to MMD
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, 0);

	// select register data for MMD - device address 2h, register 8
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	// write value 0x0 to MMD
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_CLOCK_PAD_SKEW_REG_VALUE);

	// Fast Link pulse timing for auto-nego, Set Fast Link Pulse interval to 16ms
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, 0);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, 0x4);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, 0x4000);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, 0x6);

	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, 0);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, 0x3);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, 0x4000);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, 0x1A80);

#define MICREL_1G_LINKUP_TIME_CTR_REG	0x5A

	// speed up link at 1G for KSZ9031 connection
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, 0x1);	// Set up register address for MMD - Device Address 1h
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_1G_LINKUP_TIME_CTR_REG);	// Select Register 5Ah of MMD - Device Address 1h
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, 0x4001);	// Select register data for MMD - Device Address 1h, Register 5Ah
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, 0x0106);	// (0000_0001_0000_0110 Write value 0x0106 to MMD -  Device Address 1h, Register 5Ah

	return SWITCH_SUCCESS;
}

/*
 * This is rgmi delay for ksz9131
 * MMD Address 2h, Register 4h --RGMII Control signal PAD skew : 0x70 -> No delay for RX, -0.2ns to -0.51ns for TX 
 * MMD Address 2h, Register 5h --RGMII RX Data PAD skew : 0x0 gives -0.2ns to -0.51ns delay
 * MMD Address 2h, Register 6h --RGMII TX Data PAD skew : 0x0 gives -0.2ns to -0.51ns delay
 * MMD Address 2h, Register 8h --RGMII Clock PAD skew : 0x3ff gives 0.58ns to 1.39ns delay
 */
int sl_set_rgmi_delay_ksz9131(phy_ports_t phy_port)
{
	uint8_t phy_addr;

	if( sl_get_chipset_info()->adapter_name != PHY_KSZ9131 )
	{
		return SWITCH_FAILURE;
	}

	if(phy_port == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	// MMD Address 2h, Register 4h --RGMII Control signal PAD skew 
	// Data = 0x70:  RX skew -> no delay, TX skew -> -0.2ns to -0.51ns
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_REG) | MICREL_MMD_DEVICE_ADDR2);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_CTL_SIGNAL_PAD_SKEW_REG);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_CTL_SIGNAL_PAD_SKEW_REG_VALUE);

	// MMD Address 2h, Register 5h --RGMII RX Data PAD skew 
	// Data = 0x7777: No delays on RX data pad
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_RX_DATA_PAD_SKEW_REG_VALUE);

	// MMD Address 2h, Register 6h --RGMII TX Data PAD skew
	// Data = 0x0: -0.2ns to -0.51ns delay for TX data pad
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_TX_DATA_PAD_SKEW_REG_VALUE);

	// MMD Address 2h, Register 7h: Nothing here, dummy write for post increment to register 0x8
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, 0);

	// MMD Address 2h, Register 8h --RGMII Clock PAD skew
	// Data = 0x3fc: TX -> 0.58ns to 1.39ns, RX -> 0.5ns to 1.22ns
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_CONTROL_REG, SET_MICREL_MMD_OPERATION_MODE_REG(MICREL_MMD_OPERATION_MODE_POSTINC_READ_WRITE) | MICREL_MMD_DEVICE_ADDR2);
	sl_write_smi(phy_addr, MICREL_PHY_MMD_ACEES_DATA_REG, MICREL_RGMII_CLOCK_PAD_SKEW_REG_VALUE);

	return SWITCH_SUCCESS;
}

int sl_set_rgmii_delay_vsc8541(phy_ports_t phy_port)
{
	/*
	RGMII delay control settings are found in register 20E2, bits 6:4 for RX_CLK delay and bits 2:0 for TX_CLK delay.
	*/
	uint8_t phy_addr;
	uint16_t reg_data;

	if( sl_get_chipset_info()->adapter_name != PHY_VSC8541 )
	{
		return SWITCH_FAILURE;
	}

	if(phy_port == PHY_PRIMARY_PORT)
		phy_addr = sl_get_chipset_info()->phy_address[0];
	else phy_addr = sl_get_chipset_info()->phy_address[1];

	//switch to extended register page 2.		
	sl_write_smi(phy_addr, PHY_VSC8541_EXTENDED_PAGE_ACCESS_REG, PHY_VSC8541_EXTENDED_REG_PAGE_2);
	
	//read existing RGMII control register value
	sl_read_smi(phy_addr, PHY_VSC8541_RGMII_CONTROL_REG, &reg_data);
	
	//toggle appropriate bits for 2ns RX_CLK delay and 2ns TX_CLK delay. 0b100 for both.
	reg_data |= (1<<6) | (1<<2);

	sl_write_smi(phy_addr, PHY_VSC8541_RGMII_CONTROL_REG, reg_data);

	//return back to main register page.
	sl_write_smi(phy_addr, PHY_VSC8541_EXTENDED_PAGE_ACCESS_REG, PHY_VSC8541_MAIN_REG_PAGE);
	return SWITCH_SUCCESS;
}

/*
 * 88E6361 LED mode is not in default mode. Using this API to change it to the default mode
 */
int sl_set_switch_led_default_mode(void)
{
	int port_num;
	switch_ports_info_t *port_info;
	if (sl_get_chipset_info()->adapter_name == SWITCH_88E6361)
	{
		port_info = &sl_get_chipset_info()->switch_ports_info;
		for(port_num = 0; port_num < port_info->num_serdes_ports; port_num++)
		{
			sl_write_smi(SWITCH_PORT_BASE_ADDR + port_info->num_serdes_port_start + port_num, SWITCH_LED_CTR_REG, SWITCH_88E6361_SERDES_LED_CTL_DEFAULT | SWITCH_LED_CTR_UPDATE);
		}
	}

	return SWITCH_SUCCESS;
}
