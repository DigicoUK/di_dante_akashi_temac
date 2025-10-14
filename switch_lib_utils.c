#include "switch_lib.h"
#include "switch_lib_reg.h"

/*
 * get adapter name in string for debug. buffer size should be bigger than 30 bytes.
 * return Success : pointer of buffer, Unknown type : NULL
 */
char *sl_get_adapter_name_string(adapter_enum_t adapter_name, char *adapter_buf)
{
	if(!adapter_buf)
		return 0;

	switch(adapter_name)
	{
		case PHY_KSZ9031	: strcpy(adapter_buf, "MICREL KSZ9031"); 	return adapter_buf;
		case PHY_KSZ9131	: strcpy(adapter_buf, "MICREL KSZ9131"); 	return adapter_buf;
		case PHY_MARVEL_88E1510 : strcpy(adapter_buf, "MARVEL 88E1510"); 	return adapter_buf;
		case PHY_VSC8541	: strcpy(adapter_buf, "Microchip VSC8541");	return adapter_buf;
		case SWITCH_88E6123	: strcpy(adapter_buf, "MARVEL 88E123"); 	return adapter_buf;
		case SWITCH_88E6161	: strcpy(adapter_buf, "MARVEL 88E6161"); 	return adapter_buf;
		case SWITCH_88E6350R	: strcpy(adapter_buf, "MARVEL 88E6350R"); 	return adapter_buf;
		case SWITCH_88E6351	: strcpy(adapter_buf, "MARVEL 88E6351"); 	return adapter_buf;
		case SWITCH_88E6320	: strcpy(adapter_buf, "MARVEL 88E6320"); 	return adapter_buf;
		case SWITCH_88E6321	: strcpy(adapter_buf, "MARVEL 88E6321"); 	return adapter_buf;
		case SWITCH_88E6172	: strcpy(adapter_buf, "MARVEL 88E6172"); 	return adapter_buf;
		case SWITCH_88E6176	: strcpy(adapter_buf, "MARVEL 88E6176"); 	return adapter_buf;
		case SWITCH_88E6352	: strcpy(adapter_buf, "MARVEL 88E6352"); 	return adapter_buf;
		case SWITCH_88E6361	: strcpy(adapter_buf, "MARVEL 88E6361"); 	return adapter_buf;
		default: return 0;
	}
}

// get identified network chipset with product id
adapter_enum_t sl_identify_network_chipset(uint16_t product_id)
{
	switch(product_id)
	{
		case PHY_MICREL_KSZ9031_PRODUCT_ID :
			sw_trace_print("Found MICREL KSZ9031\n");
			return PHY_KSZ9031;
		case PHY_MICREL_KSZ9131_PRODUCT_ID :
			sw_trace_print("Found MICREL KSZ9131\n");
			return PHY_KSZ9131;
		case PHY_MARVEL_88E1510_PRODUCT_ID :
			sw_trace_print("Found Marvel 88E1510\n");
			return PHY_MARVEL_88E1510;
		case PHY_MICROCHIP_VSC8541_PRODUCT_ID :
			sw_trace_print("Found Microchip VSC8541\n");
			return PHY_VSC8541;
		case SWITCH_88E123_PRODUCT_ID :
			sw_trace_print("Found Switch 88E123\n");
			return SWITCH_88E6123;
		case SWITCH_88E6161_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6161\n");
			return SWITCH_88E6161;
		case SWITCH_88E6350R_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6350R\n");
			return SWITCH_88E6350R;
		case SWITCH_88E6351_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6351\n");
			return SWITCH_88E6351;
		case SWITCH_88E6320_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6320\n");
			return SWITCH_88E6320;
		case SWITCH_88E6321_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6321\n");
			return SWITCH_88E6321;
		case SWITCH_88E6172_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6172\n");
			return SWITCH_88E6172;
		case SWITCH_88E6176_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6176\n");
			return SWITCH_88E6176;
		case SWITCH_88E6352_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6352\n");
			return SWITCH_88E6352;
		case SWITCH_88E6361_PRODUCT_ID :
			sw_trace_print("Found Switch 88E6361\n");
			return SWITCH_88E6361;

		default:
			return NETWORK_CHIPSET_NONE;
	}
}

/*
 * Printing deteced adapter
 */
int sl_print_detected_adapter(adapter_enum_t adapter_name, adapter_enum_t default_adapter_name)
{
	char adapter_name_buf[50];

	memset(adapter_name_buf, 0, sizeof(adapter_name_buf));

	if(adapter_name == NETWORK_CHIPSET_NONE)
	{
		if(sl_get_adapter_name_string(default_adapter_name, adapter_name_buf))
		{
			sw_trace_print("No network device was found and defaulting to : \'%s\' registers loading\n", adapter_name_buf);
		}
	}
	else
	{
		if(sl_get_adapter_name_string(adapter_name, adapter_name_buf))
		{
			sw_trace_print("Detecting network adapter : Found %s\n", adapter_name_buf);
		}
	}

	return SWITCH_SUCCESS;
}


/*
 * Filling network chipset information
 */
void fill_network_adapter_info( network_adapter_t *ptr_adapter, adapter_enum_t adapter_enum, uint16_t phy_addresses[2])
{
	uint16_t phy_address = phy_addresses[0];

	if (ptr_adapter->phy_redundant)
		ptr_adapter->phy_address[1] = phy_addresses[1];
	switch(adapter_enum)
	{
		case PHY_KSZ9031:
			ptr_adapter->adapter_type = ADAPTER_PHY;
			ptr_adapter->adapter_name = PHY_KSZ9031;
			ptr_adapter->phy_address[0] = phy_address;
			ptr_adapter->product_identifier = PHY_MICREL_KSZ9031_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 1;
			break;
		case PHY_KSZ9131:
			ptr_adapter->adapter_type = ADAPTER_PHY;
			ptr_adapter->adapter_name = PHY_KSZ9131;
			ptr_adapter->phy_address[0] = phy_address;
			ptr_adapter->product_identifier = PHY_MICREL_KSZ9131_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 1;
			break;
		case PHY_MARVEL_88E1510:
			ptr_adapter->adapter_type = ADAPTER_PHY;
			ptr_adapter->adapter_name = PHY_MARVEL_88E1510;
			ptr_adapter->phy_address[0] = phy_address;
			ptr_adapter->product_identifier = PHY_MARVEL_88E1510_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 1;
			break;
		case PHY_VSC8541:
			ptr_adapter->adapter_type = ADAPTER_PHY;
			ptr_adapter->adapter_name = PHY_VSC8541;
			ptr_adapter->phy_address[0] = phy_address;
			ptr_adapter->product_identifier = PHY_MICROCHIP_VSC8541_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 1;
			break;
		case PHY_UNKNOWN:
			ptr_adapter->adapter_type = ADAPTER_PHY;
			ptr_adapter->adapter_name = PHY_UNKNOWN;
			ptr_adapter->phy_address[0] = phy_address;
			ptr_adapter->product_identifier = 0;
			ptr_adapter->switch_ports_info.num_switch_ports = 1;
			break;
		case SWITCH_UNKNOWN:
			ptr_adapter->adapter_type = ADAPTER_PHY;
			ptr_adapter->adapter_name = SWITCH_UNKNOWN;
			ptr_adapter->phy_address[0] = phy_address;
			ptr_adapter->product_identifier = 0;
			ptr_adapter->switch_ports_info.num_switch_ports = 1;
			break;
		case SWITCH_88E6123:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6123;
			ptr_adapter->product_identifier = SWITCH_88E123_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 3;
			ptr_adapter->switch_ports_info.num_phy_ports = 2;
			ptr_adapter->switch_ports_info.num_phy_port_start = 0;
			ptr_adapter->switch_ports_info.num_serdes_ports = 0;
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6161:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6161;
			ptr_adapter->product_identifier = SWITCH_88E6161_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 6;
			ptr_adapter->switch_ports_info.num_phy_ports = 5;
			ptr_adapter->switch_ports_info.num_phy_port_start = 0;
			ptr_adapter->switch_ports_info.num_serdes_ports = 1;	// serdes starts from index 4 and 1 serdes ports
			ptr_adapter->switch_ports_info.num_serdes_port_start = 4;	// serdes starts from index 4 and 1 serdes ports
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6350R:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6350R;
			ptr_adapter->product_identifier = SWITCH_88E6350R_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 7;	// including port 5&6
			ptr_adapter->switch_ports_info.num_phy_ports = 5;
			ptr_adapter->switch_ports_info.num_phy_port_start = 0;
			ptr_adapter->switch_ports_info.num_serdes_ports = 0;
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6351:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6351;
			ptr_adapter->product_identifier = SWITCH_88E6351_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 7;	// including port 5&6
			ptr_adapter->switch_ports_info.num_phy_ports = 5;
			ptr_adapter->switch_ports_info.num_phy_port_start = 0;
			ptr_adapter->switch_ports_info.num_serdes_ports = 0;
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6172:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6172;
			ptr_adapter->product_identifier = SWITCH_88E6172_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 7;	// including port 5&6
			ptr_adapter->switch_ports_info.num_phy_ports = 5;
			ptr_adapter->switch_ports_info.num_phy_port_start = 0;
			ptr_adapter->switch_ports_info.num_serdes_ports = 0;
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6176:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6176;
			ptr_adapter->product_identifier = SWITCH_88E6176_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 7;	// including port 5&6
			ptr_adapter->switch_ports_info.num_phy_ports = 5;
			ptr_adapter->switch_ports_info.num_phy_port_start = 0;
			ptr_adapter->switch_ports_info.num_serdes_ports = 1;	// serdes starts from index 4 and 1 serdes ports
			ptr_adapter->switch_ports_info.num_serdes_port_start = 4;	// serdes starts from index 4 and 1 serdes ports
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6352:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6352;
			ptr_adapter->product_identifier = SWITCH_88E6352_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 7;	// including port 5&6
			ptr_adapter->switch_ports_info.num_phy_ports = 5;
			ptr_adapter->switch_ports_info.num_phy_port_start = 0;
			ptr_adapter->switch_ports_info.num_serdes_ports = 1;	// serdes starts from index 4 and 1 serdes ports
			ptr_adapter->switch_ports_info.num_serdes_port_start = 4;	// serdes starts from index 4 and 1 serdes ports
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6320:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6320;
			ptr_adapter->product_identifier = SWITCH_88E6320_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 7;	// including port 5&6
			ptr_adapter->switch_ports_info.num_phy_ports = 2;
			ptr_adapter->switch_ports_info.num_phy_port_start = 3;	// phy starts from index 3
			ptr_adapter->switch_ports_info.num_serdes_ports = 2;	// serdes starts from index 0 and 2 serdes
			ptr_adapter->switch_ports_info.num_serdes_port_start = 0;
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6321:
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6321;
			ptr_adapter->product_identifier = SWITCH_88E6321_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 7;	// including port 5&6
			ptr_adapter->switch_ports_info.num_phy_ports = 2;
			ptr_adapter->switch_ports_info.num_phy_port_start = 3;	// phy starts from index 3
			ptr_adapter->switch_ports_info.num_serdes_ports = 2;	// serdes starts from index 0 and 2 serdes
			ptr_adapter->switch_ports_info.num_serdes_port_start = 0;
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		case SWITCH_88E6361:
			// Port mapping from logical to physical [0, 3, 4, 5, 6, 7, 9, 10], 3~7 are phy, 9~10 are serdes
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6361;
			ptr_adapter->product_identifier = SWITCH_88E6361_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 8;	// including port 0
			ptr_adapter->switch_ports_info.num_phy_ports = 5;
			ptr_adapter->switch_ports_info.num_phy_port_start = 1;	// phy starts from index 1
			ptr_adapter->switch_ports_info.num_serdes_ports = 2;
			ptr_adapter->switch_ports_info.num_serdes_port_start = 6;	// serdes starts from index 6
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
		default:
			// applying default network chipset.
			ptr_adapter->adapter_type = ADAPTER_SWITCH;
			ptr_adapter->adapter_name = SWITCH_88E6350R;
			ptr_adapter->product_identifier = SWITCH_88E6350R_PRODUCT_ID;
			ptr_adapter->switch_ports_info.num_switch_ports = 7;	// including port 5&6
			ptr_adapter->switch_ports_info.num_phy_ports = 5;
			ptr_adapter->switch_ports_info.num_phy_port_start = 0;
			ptr_adapter->switch_ports_info.num_serdes_ports = 0;
			ptr_adapter->switch_ports_info.base_reg_addr_offset = 0;
			break;
	}
}

/*
 * Getting MB attaehced port 5 filtering for failsafe vlan table
 */
int sl_get_dante_mb_ports_filter(uint32_t *failsafe_vlan_filter, uint32_t *failsafe_mb_vlan)
{
	if (sl_get_chipset_info()->adapter_name != SWITCH_88E6361)
	{
		*failsafe_vlan_filter = DANTE_DEFAULT_MB_PORTS_MASK;
		*failsafe_mb_vlan = DANTE_DEFAULT_MB_PORTS_VLAN;
	}
	else
	{
		*failsafe_vlan_filter = DANTE_88E6361_DEFAULT_PORTS_MASK;
		*failsafe_mb_vlan = DANTE_88E6361_DEFAULT_PORTS_VLAN;
	}

	return SWITCH_SUCCESS;
}

/*
 * get default vlan config vlaue
 */
uint32_t sl_get_default_vlan_config(void)
{
	adapter_enum_t network_adapter;

	network_adapter = sl_get_chipset_info()->adapter_name;

	switch(network_adapter)
	{
		case SWITCH_88E6123: 	return SWITCH_DEFAULT_VLAN_88E6123;
		case SWITCH_88E6161:	return SWITCH_DEFAULT_VLAN_88E6161;

		case SWITCH_88E6350R:
		case SWITCH_88E6351:	return SWITCH_DEFAULT_VLAN_88E6350R;
		case SWITCH_88E6172:
		case SWITCH_88E6176:
		case SWITCH_88E6352:	return SWITCH_DEFAULT_VLAN_88E6352;
		case SWITCH_88E6320:
		case SWITCH_88E6321:	return SWITCH_DEFAULT_VLAN_88E6320;
		case SWITCH_88E6361:	return SWITCH_DEFAULT_VLAN_88E6361;

		default: return SWITCH_DEFAULT_VLAN_CONFIG;
	}
}
