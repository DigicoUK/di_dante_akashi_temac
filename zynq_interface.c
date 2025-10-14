#include "switch_lib_reg.h"
#include <linux/kernel.h>
#include <linux/delay.h>

static void phy_smi_read(uint8_t mii_addr, uint8_t reg_addr, uint16_t *reg_data)
{
	uint16_t addr = (mii_addr << 10) | (reg_addr << 5);
	volatile int timeout = 1000000;

	// If phy_smi_read_fn has been registered, just call it.
	if (sl_get_chipset_info()->phy_smi_read_fn)
	{
		sl_get_chipset_info()->phy_smi_read_fn(mii_addr, reg_addr, reg_data);
		return;
	}

	AUD_SYD_SMI_ADDR = addr;
	AUD_SYD_SMI_CONTROL = AUD_SYD_SMI_CTRL_READ;
	/* spin on done */
	do
	{
		if (0 == (--timeout)) {
			sw_trace_print("SMI Read Timeout!\n");
			break;
		}
	} while(!(AUD_SYD_SMI_INTR & AUD_SYD_SMI_INTR_RDY));
	/*perform smi read*/
	*reg_data = AUD_SYD_SMI_DATA_BASEADDR;
}

static void phy_smi_write(uint8_t mii_addr, uint8_t reg_addr, uint16_t reg_data)
{
	uint16_t addr = (mii_addr << 10) | (reg_addr << 5);
	volatile int timeout = 1000000;

	// If phy_smi_write_fn has been registered, just call it.
	if (sl_get_chipset_info()->phy_smi_write_fn)
	{
		sl_get_chipset_info()->phy_smi_write_fn(mii_addr, reg_addr, reg_data);
		return;
	}

	AUD_SYD_SMI_ADDR = addr;
	AUD_SYD_SMI_DATA_BASEADDR = reg_data;
	AUD_SYD_SMI_CONTROL = AUD_SYD_SMI_CTRL_WRITE;
	/* spin on done*/
	do
	{
		if (0 == (--timeout)) {
			sw_trace_print("SMI Write Timeout!\n");
			break;
		}
	}while(!(AUD_SYD_SMI_INTR & AUD_SYD_SMI_INTR_RDY));
}

// code cycle delay
void sl_nop_delay_loop(unsigned int delay_value)
{
#ifdef __AUD_ZYNQ_KERNEL__
	// There will be __bad_udelay compile error for udleay if delay_value > 2000
	if (delay_value > 1000)
		mdelay(delay_value / 1000);
	else
		udelay(delay_value);
#else
	while(delay_value--)
	__asm__ __volatile__ ("nop");
#endif
}

// Check smi phy interfacing is busy.
int smi_phy_cmd_busy(void)
{
	uint16_t smi_reg = 0xffff;
	uint8_t deadline_counter = 20;	// Check busy flag up to 20 times (2ms) and give up.
	while(deadline_counter--)
	{
		// Read smi phy reg.
		phy_smi_read(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_CMD_REG, &smi_reg);
		// busy? should be 0 for idle
		if(!(smi_reg&SWITCH_SMI_PHY_BUSY_BIT))
			break;
		sl_nop_delay_loop(100);	// delay 100 usec.
	}

	if( deadline_counter == 0 )
	{
		sw_trace_print("%s : SMI phy cmd busy check is not reponsing.\n", __func__);
		return SWITCH_FAILURE;
	}

	return SWITCH_SUCCESS;
}

/*
 * smi phy interface for write/read
 */
static void smi_phy_cmd_op(uint8_t cmd_op, uint8_t mii_addr, uint8_t reg_addr, uint16_t *reg_data)
{
	uint16_t smi_cmd_op;

	// Write operation - write data first.
	if(cmd_op == SMI_PHY_WRITE_OP)
	{
		phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_DATA_REG, *reg_data);
		smi_cmd_op = SWITCH_SMI_PHY_CMD_OP_WRITE;
	}
	else
	{
		smi_cmd_op = SWITCH_SMI_PHY_CMD_OP_READ;
	}

	// Start op
	smi_cmd_op |= (SWITCH_SMI_PHY_START_OP | SWITCH_SMI_PHY_CMD_CLAUSE22 | SWITCH_SMI_PHY_CMD_DEV_ADDR(mii_addr) | SWITCH_SMI_PHY_CMD_REG_ADDR(reg_addr));

	// Start smi phy transaction.
	phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_CMD_REG, smi_cmd_op);
}

/*
 * Audinate smi write function. 0x0 ~ 0xf : phy addresses
 */
static int single_switch_smi_write(uint8_t mii_addr, uint8_t reg_addr, uint16_t reg_data)
{
	uint8_t physical_mii_addr = mii_addr;

	// PHY addr range?
	if( mii_addr < SWITCH_PORT_BASE_ADDR )
	{
		if(sl_get_chipset_info()->adapter_name == SWITCH_88E6361)
		{
			// 88E6361 use the logical port value
			physical_mii_addr = sl_get_physical_port_id(mii_addr);
		}
		// Check busy flag.
		smi_phy_cmd_busy();
		// Write & forget
		smi_phy_cmd_op(SMI_PHY_WRITE_OP, physical_mii_addr, reg_addr, &reg_data);
	}
	else
	{
		if ((sl_get_chipset_info()->adapter_name == SWITCH_88E6361) && (mii_addr < SWITCH_GLOBAL_REGISTER_GROUP_1))
		{
			// 88E6361 port base address is 0, 88E6361 use the logical port value
			physical_mii_addr = sl_get_physical_port_id(mii_addr - SWITCH_PORT_BASE_ADDR);
		}
		phy_smi_write(physical_mii_addr, reg_addr, reg_data);
	}
	return SWITCH_SUCCESS;
}

/*
 * Audinate smi read function. 0x0 ~ 0xf : phy addresses
 */
static int single_switch_smi_read(uint8_t mii_addr, uint8_t reg_addr, uint16_t *reg_data)
{
	uint8_t physical_mii_addr = mii_addr;
	// PHY addr range?
	if( mii_addr < SWITCH_PORT_BASE_ADDR )
	{
		if(sl_get_chipset_info()->adapter_name == SWITCH_88E6361)
		{
			// 88E6361 use the logical port value
			physical_mii_addr = sl_get_physical_port_id(mii_addr);
		}
		// Check busy flag.
		smi_phy_cmd_busy();
		// Do read process
		smi_phy_cmd_op(SMI_PHY_READ_OP, physical_mii_addr, reg_addr, reg_data);
		// Wait until reading process finished.
		smi_phy_cmd_busy();
		// read value & return.
		phy_smi_read(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_DATA_REG, reg_data);
	}
	else
	{
		if ((sl_get_chipset_info()->adapter_name == SWITCH_88E6361) && (mii_addr < SWITCH_GLOBAL_REGISTER_GROUP_1))
		{
			// 88E6361 port base address is 0, 88E6361 use the logical port value
			physical_mii_addr = sl_get_physical_port_id(mii_addr - SWITCH_PORT_BASE_ADDR);
		}
		phy_smi_read(physical_mii_addr, reg_addr, reg_data);
	}
	return SWITCH_SUCCESS;
}

/*
 * smi phy interface for read by using clause45 mode
 */
static int smi_get_phy_reg_clause45(uint8_t dev_addr, uint8_t phy_addr, uint16_t reg_addr, uint16_t *reg_data)
{
	uint16_t smi_cmd_op;

	if(sl_get_chipset_info()->adapter_name == SWITCH_88E6361)
	{
		// 88E6361 use the logical port value
		phy_addr = sl_get_physical_port_id(phy_addr);
	}

	smi_phy_cmd_busy();
	// write device address
	phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_DATA_REG, reg_addr);

	// op for writing address
	smi_cmd_op = (SWITCH_SMI_PHY_START_OP | SWITCH_SMI_PHY_CMD_CLAUSE45 | SWITCH_SMI_PHY_CMD_OP_WRITE_ADDR_CLAUSE45 | SWITCH_SMI_PHY_CMD_DEV_ADDR(phy_addr) | SWITCH_SMI_PHY_CMD_REG_ADDR(dev_addr));
	smi_phy_cmd_busy();
	phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_CMD_REG, smi_cmd_op);

	// op for reading
	smi_cmd_op = (SWITCH_SMI_PHY_START_OP | SWITCH_SMI_PHY_CMD_CLAUSE45 | SWITCH_SMI_PHY_CMD_OP_READ_CLAUSE45 | SWITCH_SMI_PHY_CMD_DEV_ADDR(phy_addr) | SWITCH_SMI_PHY_CMD_REG_ADDR(dev_addr));
	smi_phy_cmd_busy();
	phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_CMD_REG, smi_cmd_op);

	smi_phy_cmd_busy();
	// read out register content
	phy_smi_read(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_DATA_REG, reg_data);

	return SWITCH_SUCCESS;
}

/*
 * smi phy interface for write by using clause45 mode
 */
static int smi_set_phy_reg_clause45(uint8_t dev_addr, uint8_t phy_addr, uint16_t reg_addr, uint16_t reg_data)
{
	uint16_t smi_cmd_op;

	if(sl_get_chipset_info()->adapter_name == SWITCH_88E6361)
	{
		// 88E6361 use the logical port value
		phy_addr = sl_get_physical_port_id(phy_addr);
	}

	smi_phy_cmd_busy();
	// write device address
	phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_DATA_REG, reg_addr);

	// op for writing address
	smi_cmd_op = (SWITCH_SMI_PHY_START_OP | SWITCH_SMI_PHY_CMD_CLAUSE45 | SWITCH_SMI_PHY_CMD_OP_WRITE_ADDR_CLAUSE45 | SWITCH_SMI_PHY_CMD_DEV_ADDR(phy_addr) | SWITCH_SMI_PHY_CMD_REG_ADDR(dev_addr));
	smi_phy_cmd_busy();
	phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_CMD_REG, smi_cmd_op);

	// op for writing
	smi_cmd_op = (SWITCH_SMI_PHY_START_OP | SWITCH_SMI_PHY_CMD_CLAUSE45 | SWITCH_SMI_PHY_CMD_OP_WRITE | SWITCH_SMI_PHY_CMD_DEV_ADDR(phy_addr) | SWITCH_SMI_PHY_CMD_REG_ADDR(dev_addr));
	smi_phy_cmd_busy();
	// write register value
	phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_DATA_REG, reg_data);
	// Start smi phy transaction.
	phy_smi_write(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_SMI_PHY_CMD_REG, smi_cmd_op);

	return SWITCH_SUCCESS;
}

/*
 * @Probe address 0x0 ~ x31 and try to detect a known chipiset	Uint8_t - number of try?? 96?
 * @num_tries			Number of probing tries
 * @chipset_enum		detected network chipset enum
 * @default_chipset_enum	default chipset enum in case network chipset is not found.
 */
int sl_probe_chipset(uint8_t num_tries, adapter_enum_t *chipset_enum, adapter_enum_t *default_chipset_enum)
{
	adapter_enum_t network_adapter = NETWORK_CHIPSET_NONE;
	int i, counter = 0, apapter_index = 0;
	uint16_t reg, scanned_phy_addr[2] = {0};
	adapter_enum_t detected_adaptor[2] = {NETWORK_CHIPSET_NONE, NETWORK_CHIPSET_NONE};

	*default_chipset_enum = NETWORK_CHIPSET_NONE;
	sw_trace_print("\n%s : Detecting network adapter : ", __func__);

	while(detected_adaptor[0] == NETWORK_CHIPSET_NONE && counter < num_tries)
	{
		// Start scanning from addr 1 in multi-chip & single-ship & phys. 0 - broadcast.
		for(i=0; i<32; i++)
		{
			phy_smi_read(i, PHY_IDENTIFIER_REGISTER_2, &reg);
			reg &= 0xfff0;

			// Any available data?
			if(reg == 0xfff0)
			{
#ifdef NETWORK_PROBING_EXTRA_DELAY
				// Timing on KHAJU might be a bit too fast.
				// Wait 100msec before probing the next chip.
				sl_nop_delay_loop(100000);
#endif
				continue;
			}
			sw_trace_print("Found chipset reg = %x \n", reg);

			network_adapter = sl_identify_network_chipset(reg);

			// If detected one is phy, keep scanning
			if(network_adapter >= PHY_KSZ9031 && network_adapter < PHY_UNKNOWN)
			{
				detected_adaptor[apapter_index] = sl_identify_network_chipset(reg);
				scanned_phy_addr[apapter_index] = i;
				apapter_index++;
			}
			else if(network_adapter >= SWITCH_88E6123 && network_adapter <= SWITCH_88E6321)
			{
				// Found switch and quit scanning
				detected_adaptor[0] = network_adapter;
				break;
			}

			/* Maximum only scan the first two phys */
			if (apapter_index >= 2)
				break;
		}

		/* If two phy chipset are scanned, then set phy_redundant */
		if ((detected_adaptor[0] > NETWORK_CHIPSET_NONE) && (detected_adaptor[0] < PHY_UNKNOWN) &&
			(detected_adaptor[1] > NETWORK_CHIPSET_NONE) && (detected_adaptor[1] < PHY_UNKNOWN))
		{
			sl_get_chipset_info()->phy_redundant = 1;
			break;
		}

		if (detected_adaptor[0] == NETWORK_CHIPSET_NONE)
		{
			counter++;
			sw_trace_print(".");
			sl_nop_delay_loop(10000);	// 10msec delay for all audinate product
		}
		else
		{
			// Found a network chipset and stop current loop
			// If we found only one phy chipset, scan one more around in case finding secondary phy.
			if(detected_adaptor[0] >= PHY_KSZ9031 && detected_adaptor[0] < PHY_UNKNOWN)
			{
				if(apapter_index == 1)
				{
					counter = num_tries - 1;
					sw_trace_print("One more scan for possible secondary phy \n");
					continue;
				}
			}
			break;
		}
	}

	// Couldn't find any network interface. so apply 8350R as default.
	if(detected_adaptor[0] == NETWORK_CHIPSET_NONE)
	{
		*default_chipset_enum = SWITCH_88E6350R;
		detected_adaptor[0] = SWITCH_PORT_BASE_ADDR;
		// fill network chipset information
		fill_network_adapter_info(sl_get_chipset_info(), SWITCH_88E6350R, scanned_phy_addr);
	}
	else
	{
		*default_chipset_enum = detected_adaptor[0];
		// fill network chipset information
		fill_network_adapter_info(sl_get_chipset_info(), detected_adaptor[0], scanned_phy_addr);
	}

	if(sl_get_chipset_info()->adapter_name == SWITCH_88E6361)
	{
		sl_lib_reg_clause45_handler_register(smi_get_phy_reg_clause45, smi_set_phy_reg_clause45);
	}

	*chipset_enum = network_adapter;

	return SWITCH_SUCCESS;
}

/*
 * MDIO reading process
 */
int sl_read_smi(uint8_t device_addr, uint8_t device_reg, uint16_t *device_data)
{
	if( sl_get_chipset_info()->adapter_type == ADAPTER_SWITCH)
		single_switch_smi_read(device_addr, device_reg, device_data);
	else
		phy_smi_read(device_addr, device_reg, device_data);

	return SWITCH_SUCCESS;
}

/*
 * MDIO writing process
 */
int sl_write_smi(uint8_t device_addr, uint8_t device_reg, uint16_t device_data)
{
	if( sl_get_chipset_info()->adapter_type == ADAPTER_SWITCH)
		single_switch_smi_write(device_addr, device_reg, device_data);
	else
		phy_smi_write(device_addr, device_reg, device_data);

	return SWITCH_SUCCESS;
}
