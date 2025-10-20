/*
 * File		: switch_lib.h
 * Created on	: 11/04/2014
 * Author	: Jerry Kim
 * Synopsis	: Switch library header file
 *
 * This software is copyright (c) 2004-2014 Audinate Pty Ltd and/or its licensors
 *
 * Audinate Copyright Header Version 1
 */

#ifndef SWITCH_LIB_H_
#define SWITCH_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

// Include Audinate data structure
#ifdef LIB_SWITCH_ZYNQ_PLATFORM
#include "zynq/switch_lib_aud_zynq.h"
#else
#include "switch_lib_aud.h"
#endif

#define SWITCH_ETHER_TYPE_DSA_DEFAULT_VALUE		0x150 		// should be in range between 0101-01FF as Experimental
#define DEFAULT_SWITCH_EXTERNAL_PHY_INT_STATUS_REG	19
#define SWITCH_PORT_BASE_ADDR				0x10
#define SWITCH_PORT_NUMBER_MAX				7
#define SWITCH_COPPER_PORT_NUMBER_MAX			5
#define SWITCH_PORT_SPEED(x)				((x>>8)&0x3)
#define SWITCH_PORT_DUPLEX				(1<<10)
#define SWITCH_PORT_LINK_UP				(1<<11)

#define SWITCH_MB_CTL_PORT2				2
#define SWITCH_MB_CTL_PORT5				5
#define SWITCH_MB_CTL_PORT6				6
#define SWITCH_88E6361_CTL_PORT0		0
#define SWITCH_88E6361_CTL_PORT9		9

/* Switch Register defs */
#define HALF_DUPLEX     0
#define FULL_DUPLEX     1
#define PORT_SPEED_10   0
#define PORT_SPEED_100  1
#define PORT_SPEED_1000 2

#define GET_SWITCH_SPEED(x)	((SWITCH_PORT_SPEED(x)) == PORT_SPEED_1000 ? 1000 :	\
				(SWITCH_PORT_SPEED(x)== PORT_SPEED_100) ? 100 : 	\
				(SWITCH_PORT_SPEED(x)== PORT_SPEED_10) ? 10 :		\
				(SWITCH_PORT_SPEED(x)== (PORT_SPEED_100|PORT_SPEED_1000)) ? 1000 : 10)
#define GET_SWITCH_LINK_UP(x)	((x) & SWITCH_PORT_LINK_UP ? 1 : 0)
#define GET_SWITCH_DUPLEX(x)	((x) & SWITCH_PORT_DUPLEX ? FULL_DUPLEX : HALF_DUPLEX)

#define SWITCH_DEFAULT_VLAN_88E6123			0x0100011	// port 0,1,5
#define SWITCH_DEFAULT_VLAN_88E6161			0x0111111	// port 0,1,5
#define SWITCH_DEFAULT_VLAN_88E6350R			0x1111111	// port 0,1,2,3,4,5,6
#define SWITCH_DEFAULT_VLAN_88E6352			0x1111111	// port 0,1,2,3,4,5,6
#define SWITCH_DEFAULT_VLAN_88E6320			0x1111111	// port 0,1,2,3,4,5,6
#define SWITCH_DEFAULT_VLAN_88E6361			0x11111111	// port 0,1,2,3,4,5,6,7
#define SWITCH_DEFAULT_VLAN_CONFIG			0x1111111	// port 0,1,2,3,4,5,6

#define SWITCH_ULTIMO_FAILSAFE_VLAN_88E6123		0x0100001	// port 0,5
#define SWITCH_ULTIMO_FAILSAFE_VLAN_88E6161		0x0100001	// port 0,5
#define SWITCH_ULTIMO_FAILSAFE_VLAN_88E6350R		0x0100001	// port 0,5
#define SWITCH_ULTIMO_FAILSAFE_VLAN_88E6352		0x0100001	// port 0,5
#define SWITCH_ULTIMO_FAILSAFE_VLAN_88E6320		0x0101000	// port 3,5

// default vlan config bit map for switches with copper & fiber ports only
#define SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6123	0x03	// port 0,1
#define SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6161	0x1F	// port 0,1
#define SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6350R	0x1F	// port 0,1,2,3,4
#define SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6352	0x1F	// port 0,1,2,3,4
#define SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6320	0x1B	// port 0,1,3,4
#define SWITCH_DEFAULT_COPPER_FIBER_VLAN_88E6361	0xFE	// port 1,2,3,4,5,6,7
#define SWITCH_DEFAULT_COPPER_FIBER_VLAN_CONFIG		0x1F	// port 0,1,2,3,4

// manual multicast mdns & conmon white list ip address
#define MULTICAST_WHITELIST_MDNS			0xE00000FB
#define MULTICAST_WHITELIST_CONMON			0xE00000E7


/*******************************************
 ************ enum definition **************
 *******************************************/
enum
{
	SWITCH_SUCCESS,
	SWITCH_INT_AVAILABLE,
	SWITCH_INT_NOT_AVAILABLE,
	SWITCH_MDIO_TIMEOUT,
	SWITCH_CANT_FIND_ANY,
	SWITCH_ONLY_USE,
	SWITCH_NOT_AVAILABLE_PORT,
	SWITCH_NOT_AVAILABLE_SERDES,
	SWITCH_NOT_AVAILABLE_EEE,
	PHY_ONLY_USE,
	SWITCH_DATA_INVALID,
	SWITCH_FAILURE = -1
};

/*******************************************
 ******** Function prototypes **************
 *******************************************/
// Split inner functions from others for ultimo ???
void sl_lib_init(void);
// Probe address 0x0 ~ x31 and try to detect a known chipiset	Uint8_t - number of try?? 96?
int sl_probe_chipset(uint8_t num_tries, adapter_enum_t *chipset_enum, adapter_enum_t *default_chipset_enum);
// Get adapter information
network_adapter_t * sl_get_chipset_info(void);
// Waiting for switch init ready
int sl_switch_init_ready(void);
// Check PPU enabled for indirect phy address access
void sl_switch_ppu_enable(void);
// SMI read function
int sl_read_smi(uint8_t device_addr, uint8_t device_reg, uint16_t *device_data);
// SMI write function
int sl_write_smi(uint8_t device_addr, uint8_t device_reg, uint16_t device_data);
// wait until switch init ready bit goes high
int sl_switch_init_ready(void);
// Copy string of adapter name with give adapter enum
char *sl_get_adapter_name_string(adapter_enum_t adapter_name, char *adapter_buf);
// get identified network chipset with product id
adapter_enum_t sl_identify_network_chipset(uint16_t product_id);
// print detected network adapter
int sl_print_detected_adapter(adapter_enum_t adapter_name, adapter_enum_t default_adapter_name);

/******* Interrupt enabling/clearing on switch/phy *********/
// set interrupt on all switch internal phys based on vlan config
int sl_set_intr_switch_phy_all(uint32_t vlan_config, uint32_t external_phys);
// set interrupt on a single switch internal phy
int sl_set_intr_switch_phy(int port_index);
// clear all internal phys interrupt from switch phys
int sl_clear_intr_switch_phy_all(uint16_t global2_status_reg);
// clear interrupt on a single phy port
int sl_clear_intr_switch_phy(int port_index);
// set interrupt on serdes ports based on vlan config
int sl_set_intr_serdes_all(uint32_t vlan_config);
// set interrupt on single serdes port
int sl_set_intr_serdes(int port_index);
// clear all interupts on serdes ports
int sl_clear_intr_serdes_all(uint16_t global2_status_reg);
// clear all interupts on serdes port
int sl_clear_intr_serdes(int port_index);
// Enable global interrupt
int sl_enable_global_interrupt(void);
// clear global interrupt/status register
int sl_clear_global_interrupt(void);
// get global2 register & clear interrupts
int sl_get_global2_int_src_reg(uint16_t *reg_val);
// Check if there is any interrupt,  then return SWITCH_INT_AVAILABLE
int sl_is_interrupt(uint16_t global2_status_reg);
// clear global interrupt/status register
int sl_clear_global_interrupt1(void);

/******* Disabling EEE on phy port *********/
// disable EEE based on detected on.???
int sl_disable_eee_all(uint32_t vlan_config);
// disable EEE on a phy port
int sl_disable_eee(int port_index);
// disable EEE on a PHY chipsets
int sl_phy_disable_eee(phy_ports_t phy_port);

/******* Configuring vlan configuration on ports *******/
// set up vlan configuration based on vlan config
int sl_set_switch_vlan_config_all(uint32_t vlan_config);
// set up vlan configuration on a single switch port
int sl_set_switch_vlan_config(int port_index, uint16_t vlan_config);
// set marvel header on given port number
int sl_set_switch_port_marvel_header(int port_index, switch_port_ctl_s control);
// Set all vlan config to default
int sl_set_switch_vlan_default(void);
// get ultimo failsafe vlan config for each switch
uint32_t sl_ultimo_get_failsafe_vlan_config(void);
// get default vlan config for each switch
uint32_t sl_get_default_vlan_config(void);
// get default copper/fiber only vlan config vlaue
uint32_t sl_get_default_copper_fiber_only_vlan_config(void);

/******* Static VTU entry management *******/
// adding a stu entry
int sl_load_stu_entry(uint16_t vid_value);
// adding a vtu entry
int sl_load_vtu_entry(uint16_t vid_value, uint32_t vlan_config);
// enabling 802.1Q on a port
int sl_set_802_1Q_enable(uint16_t vid_value, uint32_t vlan_config);
// setting default vlan id on a port
int sl_set_default_vlan_id_port(uint16_t vid_value, uint8_t port_number);
// setting default vlan id on a configured all ports
int sl_set_default_vlan_id_port_all(uint16_t vid_value, uint32_t vlan_config);
// setting default forwarding id on a port
int sl_set_default_fowarding_id_port(uint16_t vid_value, uint8_t port_number);
// setting default forwarding id on a configured all ports
int sl_set_default_fowarding_id_port_all(uint16_t vid_value, uint32_t vlan_config);

/******* Static ATU multicast filtering entrys management *******/
// add multicast filter list
int sl_set_multicast_filtering(uint8_t num_multicast, uint32_t *multicast_addr, uint32_t vlan_config, uint8_t ctl_port_map);
// add/remove one static multicast address filter at ATU DB
int sl_manage_atu_entry(uint32_t multicast_addr, switch_atu_ctl_t atu_ctl, uint16_t dpv_bitmap, uint8_t fid_val);
// probe one static multicast address filter
int sl_probe_atu_entry(uint32_t multicast_addr);
// Getting default DPV value for switch
uint16_t sl_get_switch_default_dpv(void);
/******* Configuring QoS on a port on switch *******/
// configure qos for all ports on a switch
int sl_set_switch_qos_all(void);
// configure qos on a switch port
int sl_set_switch_qos(int port_index);

/******* Controlling switch/phy ports enabling/disabling *******/
// disable all switch phys ports
int sl_disable_switch_phy_port_all(void);
// enable all switch ports based on vlan configuration
int sl_enable_switch_all(uint32_t vlan_config);
// enable/disable a switch port
int sl_ctrl_switch_port(int port_index, switch_port_ctl_s control);

// enable all phys ports on a switch based on vlan config
int sl_enable_phy_all(uint32_t vlan_config);
// enable all phys ports on a switch based on vlan config
int sl_ctrl_phy_port(int port_index, switch_port_ctl_s control);

// enable safesafe port only and disabling all other ports.
int sl_enable_failsafe(int port_index);

/******* Copper port Disabling 1G advertisement **********/
// Disabling PHY copper port 1G advertisement
int sl_disable_copper_1G_advert(int port_index);

// Disabling PHY copper port 1G advertisement with give vlan configuration
int sl_disable_copper_1G_advert_all(uint32_t vlan_config);

/******* phy related control functions ***********/
// reset phy for primary/secondary - what is primary/secondary???
int sl_reset_phy(phy_ports_t phy_port);
// isolate phys for primary/secondary
int sl_isolate_phy(phy_ports_t phy_port, phy_isolate_ctl_s control);
// Set phy chipset interrupt with link status change
int sl_clear_intr_phy(phy_ports_t phy_index);
// clear all phy chipsets interrupts
int sl_clear_intr_phy_all(void);
// set phy chipset interrupt with link status change
int sl_set_intr_phy(phy_ports_t phy_index);
// set interrupts for all phys
int sl_set_intr_phy_all(void);
// phy led configuration
int sl_phy_led_configuration(phy_ports_t phy_index);
// get phy addr
uint8_t sl_get_phy_addr(phy_ports_t phy_port);
// add rgmi delay for ksz 9031
int sl_set_rgmi_delay_ksz9031(phy_ports_t phy_port);
// add rgmi delay for ksz 9131
int sl_set_rgmi_delay_ksz9131(phy_ports_t phy_port);
// configure 88E1111 for SP605 platform
// Configure RGMII delay for VSC8541
int sl_set_rgmii_delay_vsc8541(phy_ports_t phy_port);

/******** Extra switch functions **********/
// set rgmi delay on internal RGMII ports
int sl_set_rgmi_delay(int port_index);
//  set a port as control port - drop unknown multicast
int sl_set_switch_drop_unknown_multicast(int port_index);
// set ether type dsa tag on packets
int sl_set_ethtype_dsa_tag(uint16_t ether_type_dsa_value, uint8_t dst_cpu_port);
// Clear all external phy interrupts
int sl_clr_ext_phys_all(uint32_t external_phys, uint16_t *ext_phy_reg_array);

/******** switch/phy status monitoring or getting information ********/
// switch busy check function
int sl_switch_busy_check(void);
// Get switch mode for a switch cpu-attached/test mode
int sl_get_switch_mode(uint8_t *switch_cpu_attached);
// get internal phys interrupt status register value from global register 2.
int sl_get_switch_phy_intr_status(void);
// get serdes interrupt status register value from global register 2.
int sl_get_serdes_intr_status(void);
// set all serdes ports with give vlan configuration
int sl_serdes_ports_up_all(uint32_t vlan_config);
// get switch port status register
int sl_get_switch_port_status(int port_index, uint16_t *status_reg);
// get phy status register
int sl_get_phy_port_status(phy_ports_t phy_port, uint16_t *status_reg);
// get phy speed & duplex
int sl_get_phy_speed_duplex(phy_ports_t phy_port, int *speed, int *duplex);
// get phy link status
int sl_get_phy_link_status(phy_ports_t phy_port, int *link_status);

// set switch phys led configuration for Bosch
int sl_marvel_sw_led_config(void);
// set switch led port control
int sl_marvel_port_led_control(uint8_t port_num, switch_port_led_action action);
// switch led copper port register access
int sl_marvel_copper_port_led_reg_access(uint8_t port_num, uint16_t *port_reg, switch_port_led_reg_access reg_access_action);
// get switch port c mode vaule as enum
int sl_get_switch_port_cmode(int switch_port, switch_port_cmode_t *port_cmode);
// get switch port c mode value as marvell cmode
int sl_get_switch_port_marvell_cmode(int switch_port, uint8_t *port_cmode);
// get switch port statistic error counter
int sl_get_port_statistic_error_counter(uint8_t port_num, uint32_t *sw_errors);
// Getting start bit num for serdes int enabling
uint16_t sl_get_global_reg2_serdes_int_status(adapter_enum_t switch_name, uint16_t global2_status_reg);
int sl_get_serdes_reg_index(adapter_enum_t switch_name);
/*
 * Getting start bit num for serdes int enabling
 * serdes global for 6320 - bit 0:1
 * serdes global for 6352 - bit 11
 * serdes global for 6161 - bit 11:12
 * serdes global for 6361 - bit 9:10
 */
int sl_get_global_reg2_serdes_int_bit_index(adapter_enum_t switch_name);

adapter_enum_t sl_identify_network_chipset(uint16_t product_id);
int sl_get_dante_mb_ports_filter(uint32_t *failsafe_vlan_filter, uint32_t *failsafe_mb_vlan);

//set switch led mode to the default mode
int sl_set_switch_led_default_mode(void);

// get physical port number according to the logical port number
int sl_get_physical_port_id(int logical_id);

// get logical port number according to the physical port number
int sl_get_logical_port_id(int physical_id);

// get the max switch port number
int sl_get_max_switch_port_number(void);

/******** switch/phy tools ********/
// code cycle delay
void sl_nop_delay_loop(unsigned int delay_value);
// register read, write functions for the Mdio bus
void sl_lib_smi_handler_register(phy_smi_read_t read, phy_smi_write_t write);
// Check smi phy interfacing is busy.
int smi_phy_cmd_busy(void);
// register clause45 read, write functions for the Mdio bus
void sl_lib_reg_clause45_handler_register(phy_reg_read_clause45_t read, phy_reg_write_clause45_t write);
// mido clause45 read function
int sl_read_reg_clause45(uint8_t dev_addr, uint8_t phy_addr, uint16_t reg_addr, uint16_t *reg_data);
// mido clause45 write function
int sl_write_reg_clause45(uint8_t dev_addr, uint8_t phy_addr, uint16_t reg_addr, uint16_t reg_data);
// set cpu port number
void sl_set_cpu_port(uint8_t cpu_port);
// get cpu port number
void sl_get_cpu_port(uint8_t *cpu_port);

#ifdef __cplusplus
}	// extern C
#endif

#endif /* SWITCH_LIB_H_ */
