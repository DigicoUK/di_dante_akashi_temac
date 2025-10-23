/*
 * File : akashi_temac.c
 * Created : April 2022
 * Authors : Martin Siu
 * Synopsis: Akashi Temac Ethernet device driver.
 *
 * Copyright 2022 Audinate Pty Ltd and/or its licensors
 *
 */

/*
 * Change History:
 *  4.1.17: Get unused network interface index before calling register_netdev for IPCore.
 *  4.1.16: Get the FPGA space size from DTS
 *  4.1.15: Add spin lock to protect operating tx_priority_skb_list and tx_skb_list.
 *  4.1.14: Add SIOCDEVIFLINKSTATUS ioctl to get the interface link status.
 *  4.1.13: Support Marvell 88E6361 switch in the driver.
 *  4.1.12: Rename single_interface.c ad zynq_interface.c and relocate some switch codes to a new file
 *  4.1.11: Not link off if the control port is in 10Mbps.
 *  4.1.10: Enable CONFIG_AKASHI_EMAC_0_SMI_IRQ for US+ compatibility
 *  4.1.9: Fix ethernet switch config ioctl function pointer mapping for kernel >= 5.15.x
 *  4.1.8: Update network chipset support list
 *  4.1.7: Fix timestamp read when the transmit timestamp interrupt has more
 *         than one timestamp value.
 *  4.1.6: Get the correct ts entry number in ts interrupt handler.
 *  4.1.5: Optimize packet sending in akashi_start_xmit()
 *  4.1.4: Detect dual PHY dynamically, always in redundant mode for dual PHY.
 *  4.1.3: Support & handling device mac address correctly on 5.19.x kernel
 *  4.1.2: Revised a debug message on TX timestamp overflow.
 *  4.1.1: Fix AES67 reconnection issue.
 *  4.1.0: Add ZYNQ PS MAC driver to support the Ultrascale+ platform.
 *  4.0.0: Driver support for 5.X kernel and 4.X backward compatibiilty
 *  3.2.3: Support multicast white list feature
 *  3.2.2: Add a budget for rx to handle no more than the budget of packets in
 *         denet_recvBH. This is to avoid blocking CPU.
 *         Fix a memory leak issue when TX_SKB_LIST_STOP_THRESHOLD is met.
 *  3.2.1: Decrease the link status check time from 1s to 0.1s when using SMI
 *         interrupt. This can speed up handling link up event.
 *  3.2.0: Add link check when CONFIG_AKASHI_EMAC_0_SMI_IRQ is not defined. Add
 *         dual phy suport.
 *  3.1.1: Fixed compile error when CONFIG_AKASHI_EMAC_0_SMI_IRQ is not defined.
 *  3.1.0: Add 64-bit architecture support.
 *  3.0.0: Unified akashi temac driver for all Zynq products.
 */

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/pgalloc.h>
#include <asm/uaccess.h>
#include <generated/autoconf.h>
#include <linux/errqueue.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/inetdevice.h>
#include <linux/init.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/of_net.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/mdio.h>
#include <net/ip.h>
#include <net/sock.h>

#include "akashi_temac.h"
#include "smi_config.h"
#include "switch_lib_reg.h"

//==============================================================================
// DEFINES
//==============================================================================

#define DRIVER_AUTHOR                   "Audinate Pty Ltd <oem-support@audinate.com>"
#define DRIVER_NAME                     "Dante Akashi TEMAC driver" //Max length 32 Bytes
#define DRIVER_VERSION                  "4.1.17"
#define DRIVER_LICENSE                  "GPL"
#define FIRMWARE_VERSION                "1.00a"
#define BUS_INFO                        "IPIF"

#define DENET_MAJOR                     (247)
#define DENET_NAME                      "denet"
#define DENET_TIMESTAMP_MINOR           (250)
#define DENET_PORTSTAT_MINOR            (251)
#define DENET_PORTSTAT1_MINOR           (252)

#define AKASHI_PROC_ROOT_NAME           ("akashi")

#define AKASHI_DEBUG_BIT_MASK_RX_TS_HEX     (1 << 0)
#define AKASHI_DEBUG_BIT_MASK_RX_TS_DEC     (1 << 1)
#define AKASHI_DEBUG_BIT_MASK_RX_PAYLOAD    (1 << 2)
#define AKASHI_DEBUG_BIT_MASK_TX_TS         (1 << 3)
#define AKASHI_DEBUG_BIT_MASK_TX_PAYLOAD    (1 << 4)

/* linux-v4.10 dev_ioctl paths */
// SIOCETHTOOL                              0x8946 // -> ethtool_ops
// SIOCDEVPRIVATE 1..15                     0x89F0 // -> dev_ifsioc -> ndo_do_ioctl
// SIOCSMIIREG                              0x8948 // -> dev_ifsioc -> ndo_do_ioctl
// SIOCGMIIREG                              0x8949 // -> dev_ifsioc -> ndo_do_ioctl
#define SIOCDEVICEIDENTIFY                  0x89F7 // -> dev_ifsioc -> ndo_do_ioctl
#define SIOCDEVLINKUTIL                     0x89F3 // -> dev_ifsioc -> ndo_do_ioctl
#define SIOCDEVPHYID                        0x89F5 // -> dev_ifsioc -> ndo_do_ioctl
#define SIOCDEVREDUNDANT                    0x89F6 // -> dev_ifsioc -> ndo_do_ioctl
#define SIOCDEVERRORCLEAR                   0x89F8 // -> dev_ifsioc -> ndo_do_ioctl
#define SIOCDEVEXTPHYINTCLEARREG            0x89F9 // -> dev_ifsioc -> ndo_do_ioctl
#define SIOCDEVPORTSTATISTICERRORCOUNTER    0x89FA // -> dev_ifsioc -> ndo_do_ioctl
#define SIOCDEVIFLINKSTATUS                 0x89FB // -> dev_ifsioc -> ndo_do_ioctl

#define TIMER_DELAY_1SEC (HZ)
#define TIMER_DELAY_100MSEC (HZ / 10)

/* Define the budget as the same default value of net.core.netdev_budget */
#define AKASHI_RX_BUDGET        300

#define DEFAULT_ETHMADDR                "00:1d:c1:00:00:01"

/* Match table for of_platform binding */
static const struct of_device_id akashi_of_match[] =
{
    { .compatible = "xlnx,akashi", },
    {}
};
MODULE_DEVICE_TABLE(of, akashi_of_match);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE(DRIVER_LICENSE);
MODULE_VERSION(DRIVER_VERSION);
/* To avoid tainting the kernel on module loading */
MODULE_INFO(intree, "Y");

char * ethmaddr = DEFAULT_ETHMADDR;
module_param(ethmaddr,charp, S_IRUGO);
MODULE_PARM_DESC(ethmaddr, "Ethernet MAC address");  ///< parameter description

char * red_flag = "0:0:0:0";
module_param(red_flag, charp, S_IRUGO); ///< Param desc. charp = char ptr, S_IRUGO can be read/not changed
MODULE_PARM_DESC(red_flag, "Temac Parameter red_flag");  ///< parameter description

char * ctr_port = "0";
module_param(ctr_port, charp, S_IRUGO); ///< Param desc. charp = char ptr, S_IRUGO can be read/not changed
MODULE_PARM_DESC(ctr_port, "Temac Parameter ctr_port");  ///< parameter description

char * ctr_port_info = "0";
module_param(ctr_port_info, charp, S_IRUGO); ///< Param desc. charp = char ptr, S_IRUGO can be read/not changed
MODULE_PARM_DESC(ctr_port_info, "Temac Parameter ctr_port_info");  ///< parameter description

char * ctr_port_config = "0";
module_param(ctr_port_config, charp, S_IRUGO); ///< Param desc. charp = char ptr, S_IRUGO can be read/not changed
MODULE_PARM_DESC(ctr_port_config, "Temac Parameter ctr_port_config");  ///< parameter description

char * ex_phys = "0";
module_param(ex_phys, charp, S_IRUGO); ///< Param desc. charp = char ptr, S_IRUGO can be read/not changed
MODULE_PARM_DESC(ex_phys, "Temac Parameter ex_phys");  ///< parameter description

char * multicast_white_list = "0";
module_param(multicast_white_list, charp, S_IRUGO); ///< Param desc. charp = char ptr, S_IRUGO can be read/not changed
MODULE_PARM_DESC(multicast_white_list, "Temac Parameter multicast_white_list");  ///< parameter description

//==============================================================================
// VARIABLES & prototypes
//==============================================================================

volatile uint8_t *aud_syd_virtual_base_addr;

// global structure for switch/phy, required for lib-switch. Do NOT remove!
network_adapter_t network_adapter_info;

static net_common_t *net_common_context = NULL;

//static int txfifo_debugcounter = 0;
//static int txfifo_ptp_debugcounter = 0;
static int tx_debug_print = 0;

// bitmask of port numbers assigned to first RGMII in hyperport mode
static unsigned int digico_hyperport_primary_ports = 0;
// bitmask of port numbers assigned to second RGMII in hyperport mode
static unsigned int digico_hyperport_secondary_ports = 0;

static int denet_private_ioctl(struct net_device *dev, struct ifreq *rq, void __user *data, int cmd);

#ifdef CONFIG_AKASHI_DEBUG_DUMP_SKB_LIST
static void akashi_dump_tx_skb_list(struct sk_buff_head *list, char *str)
{
    unsigned long flags;
    struct sk_buff *skb;
    int i;

    spin_lock_irqsave(&list->lock, flags);
    printk(KERN_INFO "[%s] qlen: %d\n", str, list->qlen);
    skb = list->next;
    for (i = 0; i < list->qlen; i++)
    {
        printk(KERN_INFO"    [%d] %s %d\n", i, skb->dev->name, skb->len);
        skb = skb->next;
    }
    spin_unlock_irqrestore(&list->lock, flags);
}
#endif

static inline net_local_t *netdev_get_priv(struct net_device *dev)
{
    /* Return our private data. */
    return (net_local_t *) netdev_priv(dev);
}

static inline bool is_primary_dev(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;

    return (net_common->primary_dev == dev);
}

static inline bool is_secondary_dev(struct net_device *dev)
{
    return !is_primary_dev(dev);
}

static void akashi_set_hw_ipv4_addr(struct net_device *dev, u32 addr)
{
    u8 ip_addr[4];

    ip_addr[0] = (u8)((addr & 0x000000ff) >> 0);
    ip_addr[1] = (u8)((addr & 0x0000ff00) >> 8);
    ip_addr[2] = (u8)((addr & 0x00ff0000) >> 16);
    ip_addr[3] = (u8)((addr & 0xff000000) >> 24);

    printk(KERN_INFO "%s: IP %d.%d.%d.%d\n",
        dev->name, ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);

    addr = ntohl(addr);

    if (is_primary_dev(dev))
    {
        AUD_SYD_PROTO_IP_ADDR0_HI = (addr >> 16);
        AUD_SYD_PROTO_IP_ADDR0_LO = (addr & 0xFFFF);
    }
    else
    {
        AUD_SYD_PROTO_IP_ADDR1_HI = (addr >> 16);
        AUD_SYD_PROTO_IP_ADDR1_LO = (addr & 0xFFFF);
    }
}

static int device_nb_set_ipv4_addr(struct notifier_block *nb, unsigned long action, void *data)
{
    struct in_ifaddr *ifa = (struct in_ifaddr *) data;
    u32 addr = ifa->ifa_address;

    if (action == NETDEV_UP)
    {
        if (net_common_context->primary_dev && !strcmp(ifa->ifa_label, PRIMARY_INTERFACE_NAME))
        {
            akashi_set_hw_ipv4_addr(net_common_context->primary_dev, addr);
        }
        else if (net_common_context->secondary_dev && !strcmp(ifa->ifa_label, SECONDARY_INTERFACE_NAME))
        {
            akashi_set_hw_ipv4_addr(net_common_context->secondary_dev, addr);
        }
    }

    return 0;
}

//register device notifier
struct notifier_block akashi_nb =
{
    .notifier_call = device_nb_set_ipv4_addr,
    .next = NULL,
    .priority = 1, //advance the priority by 1
};

/*******************************************
 ***********  print function  **************
 *******************************************/
#define DUMP_BUF_SIZE (512)
static void akashi_dump_data(uint8_t *data, int len)
{
    int i;
    int col = 16;
    char buf[DUMP_BUF_SIZE];
    char *ptr;

    ptr = buf;
    for (i = 0; i < len; i++)
    {
        if (((i + 1) % col) == 0)
        {
            ptr += sprintf(ptr, "%02x", data[i]);
            printk(KERN_INFO "    %s\n", buf);
            memset(buf, 0, sizeof(buf));
            ptr = buf;
        }
        else
        {
            ptr += sprintf(ptr, "%02x ", data[i]);
        }
    }

    if (i % col != 0)
    {
        printk(KERN_INFO "    %s\n", buf);
    }
}

/*
 * Ported for Zynq ARM little-endian
 */
static inline u32 _swap_on_load(volatile u32* a, u32 b)
{
    volatile u32 *addr = a + b;
    u32 d = *addr;

#ifdef CONFIG_AKASHI_64BIT_ARCH
    asm volatile ("rev32 %0, %1 \n\t"
            :
            : "r"(d), "r"(d));
#else
    asm volatile ("rev %0, %1 \n\t"
            :
            : "r"(d), "r"(d));
#endif

    return d;
}

static inline void _swap_on_store(volatile u32* a, u32 b, u32 d)
{
    volatile u32 *addr = a + b;

#ifdef CONFIG_AKASHI_64BIT_ARCH
    asm volatile ("rev32 %0, %1 \n\t"
            :
            : "r"(d), "r"(d));
#else
    asm volatile ("rev %0, %1 \n\t"
            :
            : "r"(d), "r"(d));
#endif

    *addr = d;
}

u32 get_temac_vaddr(uint8_t ** addr)
{
    if(aud_syd_virtual_base_addr != NULL)
    {
        *addr = (uint8_t *)aud_syd_virtual_base_addr;
        return 0;
    }
    else
    {
        printk(KERN_ERR "%s: Error: Temac Sydney address is not mapped!\n", __func__);
    }
    return EFAULT; // Bad address
}

//-------------------------------------------------------------------
/* Take kernel cmdline option macaddr=... and set MAC address */
static int __init denet_hw_addr_setup(net_common_t *net_common, char *addrs);
static int __init denet_redundancy_setup(net_common_t *net_common, char *red);
static int __init denet_ctr_port_setup(net_common_t *net_common, char *ctr_port);
static int __init denet_ctr_port_info_setup(net_common_t *net_common, char *ctr_port_info);
static int __init denet_ctr_port_config_setup(net_common_t *net_common, char *ctr_port_config);
static int __init denet_external_phys_setup(net_common_t *net_common, char *ex_phys);
static int __init denet_multicast_filtering_setup(net_common_t *net_common, char *multicast_white_list);

//==============================================================================
// LOCAL FUNCTIONS
//==============================================================================
void akashi_txts_queue_init(net_common_t *net_common)
{
    int i = 0;
    txts_queue_t *q = &net_common->txts_queue;

    q->head_to_send = 0;
    q->head_sent = 0;
    q->tail = 0;

    for (i = 0; i < TXTS_QUEUE_SIZE; i++)
    {
        q->txts_status[i]       = TXTS_STATUS_READY;
        q->ids[i]               = (u16) (i+1);
        q->txts[i].dest_ipv4    = 0;
        q->txts[i].dest_port    = 0;
        q->txts[i].msg_type     = 0;
        q->txts[i].ptp_version  = 0;
        q->txts[i].padding      = 0;
    }
}

//-------------------------------------------------------------------
// Get whether an item is ready to to be sent by circular queue
static int akashi_txts_queue_get_count(net_common_t *net_common)
{
    int i;
    txts_queue_t *q = &net_common->txts_queue;

    for (i = 0; i < TXTS_QUEUE_SIZE; i++)
    {
        if (q->txts_status[i] == TXTS_STATUS_TX_INT)
        {
            return 1;
        }
    }

    return 0;
}

//==============================================================================
// GLOBAL FUNCTIONS
//==============================================================================

static void tx_hw_reset(struct net_device *dev, u16 duplex)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    unsigned int sched_intr_ctl_reg;

    printk(KERN_INFO "%s\n",__func__);

    /* FIXME: */
    /* Shouldn't really be necessary, but shouldn't hurt. */
    netif_stop_queue(dev);

    /* Remove and free all buffers inside the TX skbuff list */
    spin_lock_bh(&net_common->tx_lock);
    skb_queue_purge(&net_common->tx_skb_list);
    spin_unlock_bh(&net_common->tx_lock);
    spin_lock_bh(&net_common->tx_priority_lock);
    skb_queue_purge(&net_common->tx_priority_skb_list);
    spin_unlock_bh(&net_common->tx_priority_lock);

    // interrupt disable
    sched_intr_ctl_reg = ~AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_INTR_EN;

    if (net_common->has_priority_queue)
        sched_intr_ctl_reg &= ~AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_INT_EN;

    AUD_SYD_SCHED_INTR_CONTROL &= sched_intr_ctl_reg;

    // All fifo reset
    sched_intr_ctl_reg = AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_RST;

    if (net_common->has_priority_queue)
        sched_intr_ctl_reg |= AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_RST;

    AUD_SYD_SCHED_INTR_CONTROL |= sched_intr_ctl_reg;

    // Enable intr
    sched_intr_ctl_reg = AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_INTR_EN;

    if (net_common->has_priority_queue)
        sched_intr_ctl_reg |= AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_INT_EN;

    AUD_SYD_SCHED_INTR_CONTROL |= sched_intr_ctl_reg;

    // FIXME: get all interfaces?
    //dev->_tx->trans_start = 0xffffffff - TX_TIMEOUT - TX_TIMEOUT;    /* to exclude tx timeout */

    // FIXME:
    /* We're all ready to go.  Start the queue in case it was stopped. */
    //if (!bh_entry)
    if(netif_queue_stopped(dev))
        netif_wake_queue(dev);
}

static void rx_hw_reset(struct net_device *dev, u16 duplex)
{
    unsigned int sched_intr_ctl_reg;

    printk(KERN_INFO "%s\n",__func__);

    // interrupt disable
    sched_intr_ctl_reg = ~AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_INTR_EN;

    AUD_SYD_SCHED_INTR_CONTROL &= sched_intr_ctl_reg;

    // All fifo reset
    sched_intr_ctl_reg = AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_RST;

    AUD_SYD_SCHED_INTR_CONTROL |= sched_intr_ctl_reg;

    // Enable intr
    sched_intr_ctl_reg = AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_INTR_EN;

    AUD_SYD_SCHED_INTR_CONTROL |= sched_intr_ctl_reg;
}

/* This represets the state of our dante network interfaces */
static int get_phy_status(net_common_t *net_common)
{
    dante_network_st_t *net_st = &net_common->dante_net_st;
    u16 reg;
    int s, iface_reg[2] = {0,0};
    uint8_t changed=0, cpu_port;
    adapter_enum_t adapter_name = net_st->net_chip_info.adapter_name;
    sl_get_cpu_port(&cpu_port);
    cpu_port = sl_get_logical_port_id(cpu_port);

    if (net_st->net_chip_info.adapter_type == ADAPTER_SWITCH)
    {
        int i;

        // Check all switch phys
        for (i = 0; i < net_st->net_chip_info.switch_ports_info.num_switch_ports; i++)
        {
            // Do not calculate link status including control ports & switch port 5
            if (net_common->config.control_ports & (1 << i) || i == cpu_port)
                continue;

#ifndef CONFIG_AKASHI_CTL_PORT_6
            // Include port 6 for link/speed calculation?
            if ((i == SWITCH_MB_CTL_PORT6) && (adapter_name != SWITCH_88E6361))
            {
                // If it is external phy is not specified, then we don't include calculation for port 6.
                if (!net_common->config.external_phys || !(((net_common->config.external_phys >> (i * 4)) & ETH_SWITCH_EXTERNAL_PHY_MODE)))
                {
                    // printk("Port 6 is not included\n");
                    continue;
                }
            }
#endif
            // get switch port status
            s = sl_get_switch_port_status(i, &reg);
            if (s != 0)
            {
                printk(KERN_ERR "%s: Could not read PHY port %d stat register; error %d\n", __func__, i, s);
                return -1;
            }

            // Check if it is redundant or switch mode &
            if (net_common->config.vlan_pri || net_common->config.vlan_sec)
            {
                // it checks each vlan port numbers status.
                if ((net_common->config.vlan_pri & (1 << i)) || (net_common->config.vlan_sec & (1 << i)))
                {
                    if (reg != (net_st->ports.status_reg[i]))
                    {
                        // need port change notification
                        changed |= (1 << i);
                        net_st->ports.changed = changed;
                    }
                }
            }
            else
            {
                // No configuration & it checks for all ports.
                {
                    // need port change notification
                    changed |= (1 << i);
                    net_st->ports.changed = changed;
                }
            }

            // Calculation with redundant mode.
            if (net_common->config.red_en & ETH_SWITCH_MODE_REDUNDANT)
            {
                if ((net_common->config.vlan_pri & (1 << i)))
                {
                    net_st->ports.status_reg[i] = reg;
                    iface_reg[0] |= reg;
                }
                else if ((net_common->config.vlan_sec & (1 << i)))
                {
                    net_st->ports.status_reg[i] = reg;
                    iface_reg[1] |= reg;
                }
                else
                {

                }
            }
            else
            {
                // Calculation with vlan configuration.
                if ((net_common->config.red_en & ETH_SWITCH_MODE_SWITCH) || net_common->config.vlan_pri)
                {
                    // Switch mode with Vlan config
                    if( (net_common->config.vlan_pri & (1 << i)))
                    {
                        net_st->ports.status_reg[i] = reg;
                        iface_reg[0] |= reg;
                    }
                }
                else
                {
                    // No capability assign all to pri VLAN
                    net_st->ports.status_reg[i] = reg;
                    iface_reg[0] |= reg;
                }
            }
        }

        // We need at least one link and one duplex bit.
        // If we get one     of each 100/GbE we report 1000 below
        // Multicast filtering should be used to "protect" the 100 port
        if (net_common->config.red_en & ETH_SWITCH_MODE_REDUNDANT)
        {
            u16 i;

            for (i = 0; i < 2; i++)
            {
                net_st->interface_stat[i].link_status = GET_SWITCH_LINK_UP(iface_reg[i]);
                net_st->interface_stat[i].link_speed = GET_SWITCH_SPEED(iface_reg[i]);
                net_st->interface_stat[i].link_duplex = GET_SWITCH_DUPLEX(iface_reg[i]);
            }
        }
        else
        {
            net_st->interface_stat[0].link_status = GET_SWITCH_LINK_UP(iface_reg[0]);
            net_st->interface_stat[0].link_speed = GET_SWITCH_SPEED(iface_reg[0]);
            net_st->interface_stat[0].link_duplex = GET_SWITCH_DUPLEX(iface_reg[0]);
        }

        if (net_st->ports.changed)
        {
            // send port change notification
            net_common->ps_pending = 1;
            net_common->ps_pending1 = 1;
            wake_up_interruptible(&net_common->psqueue);
            wake_up_interruptible(&net_common->psqueue1);

            // back up port stats
            memcpy(&net_st->backup_ports, &net_st->ports, sizeof(portstat_t));
        }
    }
    else if (net_st->net_chip_info.adapter_type == ADAPTER_PHY)
    {
        int i, phy_loop;

        // If it is redundant, then loop twice.
        if (net_st->net_chip_info.phy_redundant)
            phy_loop = 2;
        else
            phy_loop = 1;

        for (i = 0; i < phy_loop; i++)
        {
            s = sl_read_smi(net_st->net_chip_info.phy_address[i], MII_BMCR, &reg);
            if (s != 0)
            {
                printk(KERN_ERR "%s: Could not read PHY control register; error %d\n", __func__, s);
                return -1;
            }

            if (!(reg & BMCR_ANENABLE))
            {
                /*
                 * Auto-negotiation is disabled so the full duplex bit in
                 * the control register tells us if the PHY is running
                 * half or full duplex.
                 */
                if (reg & BMCR_FULLDPLX)
                {
                    net_st->interface_stat[i].link_speed = 100;
                    net_st->interface_stat[i].link_duplex = FULL_DUPLEX;
                }
                else
                {
                    printk(KERN_INFO "Can not support half duplex connections.\n");
                    printk(KERN_INFO "Try another Ethernet port\n");
                }
            }
            else
            {
                // Get phy chipset speed & duplex
                sl_get_phy_speed_duplex((i == 0 ? PHY_PRIMARY_PORT : PHY_SECONDARY_PORT),
                                        &net_st->interface_stat[i].link_speed,
                                        &net_st->interface_stat[i].link_duplex);
            }

            // Getting link up status. same for all phy chipsets
            sl_get_phy_link_status((i == 0 ? PHY_PRIMARY_PORT : PHY_SECONDARY_PORT), &net_st->interface_stat[i].link_status);
            // Need to get link up status twice for 9131 as the first read of the register after link up says it is link down still
            if (adapter_name == PHY_KSZ9131)
                sl_get_phy_link_status((i == 0 ? PHY_PRIMARY_PORT : PHY_SECONDARY_PORT), &net_st->interface_stat[i].link_status);
        }
    }
    else
    {

    }
    return 0;
}

#ifdef CONFIG_AKASHI_CONFIGURE_SWITCH
/*
 * Set multicast filtering on ports & adding known whitelist to ATU
 * @param control_port_index : control port index
 */
static void set_multicast_filtering(uint32_t vlan_config, net_common_t *net_common)
{
    uint32_t control_ports_config;

    control_ports_config = net_common->config.control_ports_config;

    // Apply multicast filtering with index number
    if (control_ports_config)
    {
        int forwarding_port_number;
        uint8_t fset_atu_multicat_entries = 0;
        uint32_t control_port_bitmap = 0;

        // convert control port info to string.
        for (forwarding_port_number = 0; forwarding_port_number < 8; forwarding_port_number++)
        {
            // check port bit.
            if (control_ports_config & (ETH_SWITCH_CONTROL_PORT_MCAST_FILTER | ETH_SWITCH_CONTROL_PORT_MCAST_FILTER_LINK_CALCULATION))
            {
                fset_atu_multicat_entries = 1;
                // Set this port to drop unknown multicast packets.
                sl_set_switch_drop_unknown_multicast(forwarding_port_number);
            }

            // Making bitmap for kernel.
            if (control_ports_config & (ETH_SWITCH_CONTROL_PORT_MCAST_FILTER | ETH_SWITCH_CONTROL_PORT_LINK_CALCULATION))
            {
                control_port_bitmap |= (1<<forwarding_port_number);
            }

            control_ports_config >>= 4;	// Nibble based control.

            if (control_ports_config == 0)
                break;
        }

        // Set allowed multicast list.
        if (fset_atu_multicat_entries)
        {
            sl_set_multicast_filtering(MC_MAX_MULTICAST_ADDRESS_LIST, net_common->config.multicast_addr_white_list, vlan_config, control_port_bitmap);
        }

        // Configure control ports bitmap
        if (control_port_bitmap)
        {
            net_common->config.control_ports = control_port_bitmap;
        }
    }
}
#endif

// Configure smi interrupt
static void smi_int_init(net_common_t *net_common)
{
    if (net_common->dante_net_st.net_chip_info.adapter_type == ADAPTER_SWITCH)
    {
        uint32_t vlan_config = 0;
        uint32_t pri_temp, sec_temp;

        pri_temp = sec_temp = 0;

        // composing vlan configuration from red_en, vlan_pri, vlan_sec
        if (net_common->config.red_en)
        {
            int vlan_index = 0;
            // Compose vlan config data
            pri_temp = net_common->config.vlan_pri;
            sec_temp = net_common->config.vlan_sec;

            while (pri_temp)
            {
                if (pri_temp & 0x1)
                {
                    vlan_config |= (ETH_SWITCH_VLAN_PRI << (vlan_index * 4));
                }
                pri_temp >>= 1;
                vlan_index++;
            }

            vlan_index = 0;

            while (sec_temp)
            {
                if (sec_temp & 0x1)
                {
                    vlan_config |= (ETH_SWITCH_VLAN_SEC << (vlan_index * 4));
                }
                sec_temp >>= 1;
                vlan_index++;
            }
        }
        else
        {
            vlan_config = SWITCH_DEFAULT_VLAN_CONFIG;
        }

        // enable all switch phys interrupt with vlan configuration
        sl_set_intr_switch_phy_all(vlan_config, net_common->config.external_phys);
        // enable all serdes port interrupt with vlan configuration
        sl_set_intr_serdes_all(vlan_config);
        // enable global switch interrupt
        sl_enable_global_interrupt();
        // clear global switch interrupt
        sl_clear_global_interrupt();
    }
    else
    {
        // Enable all phys interrupt with link status change
        sl_set_intr_phy_all();
    }
}

/*
 * Set up led config
 */
static void marvell_switch_led_init(void)
{
#ifdef CONFIG_AKASHI_MARVEL_SWITCH_LED
    sl_marvel_sw_led_config();
#endif
}

// Clear error counter
static void clear_error_counter(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);

    net_local->stats.rx_crc_errors = 0;

    // Register is cleared on write, we accumulate in the stats structure
    if (is_primary_dev(dev))
    {
        AUD_SYD_MAC_RX0_BAD_PACKETS = 0;
    }
    else
    {
        AUD_SYD_MAC_RX1_BAD_PACKETS = 0;
    }
}

#ifdef CONFIG_AKASHI_EMAC_0_SMI_IRQ
// This process is invoked by 1 sec timer when link is up.
static void link_up_process(struct timer_list *t)
{
    net_local_t *net_local = from_timer(net_local, t, phy_timer);
    struct net_device *dev = net_local->dev;

    //Clear the FPGA TX FIFO
    //AUD_SYD_SCHED_INTR_CONTROL |= AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_RST;

    del_timer(&net_local->phy_timer);

    // Only set the carrier to on if the carrier is off
    if (netif_carrier_ok(dev) == 0)
    {
        netif_carrier_on(dev);
    }
    net_local->link_status = 1;

    net_local->stats.rx_crc_errors = 0;

    // Link is on & clear error counter.
    clear_error_counter(dev);

    printk(KERN_INFO "%s: Link carrier restored.\n", dev->name);
}

static void special_link_up_process(struct timer_list *t)
{
    net_local_t *net_local = from_timer(net_local, t, phy_timer);
    struct net_device *dev = net_local->dev;

    // Only set the carrier to on if the carrier is off
    if (netif_carrier_ok(dev) == 0)
    {
        netif_carrier_on(dev);
    }
}
#endif

/* Network link/speed/duplex status process routine */
static int network_mii_process(net_common_t *net_common)
{
    struct net_device *process_dev;
#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
    net_local_t *lp = netdev_get_priv(net_common->primary_dev);
#endif
    net_local_t *lp_query_interface;
    u16 phy_duplex, phy_speed, current_link, netif_link;
    int red_loop_cond=0, index_count=0;
    /* Use common structure for retrieving information on switch redundancy. */
    dante_network_st_t *net_st = &net_common->dante_net_st;

    // DIGICO patch: ignore switch IRQ when MDIO locked out
    if (net_common->mdio_locked)
        return 0;

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
    /* Use to check if link status really changes in polling mode */
    bool link_st_change = false;
#endif

    /* First, find out what's going on with the PHY. */
    if (get_phy_status(net_common))
    {
        printk(KERN_ERR "%s: Unable to get phy status.\n",
                net_common->primary_dev->name);
#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
        /* Set up the timer so we'll get called again in 2 seconds. */
        lp->phy_timer.expires = jiffies + 2 * TIMER_DELAY_1SEC;
        add_timer(&lp->phy_timer);
#endif
        return 0;
    }

    // decide logic how many interface processes.
    if (net_common->config.red_en > 0)
    {
        red_loop_cond = net_common->config.red_en - 1;
    }

    process_dev = net_common->primary_dev;    // start from interface 0.

    do
    {
        phy_duplex = net_st->interface_stat[index_count].link_duplex;
        current_link = net_st->interface_stat[index_count].link_status;
        phy_speed = net_st->interface_stat[index_count].link_speed;

        // Get virtual interface
        if (index_count == 1)
        {
            if (net_st->net_chip_info.adapter_type == ADAPTER_SWITCH)
            {
                // In case vnet is not up yet.
                if (!net_common->secondary_dev)
                    return 0;

                process_dev = net_common->secondary_dev;
            }
            else
            {
                if (net_st->net_chip_info.phy_redundant)
                {
                    // Get virtual interface
                    // In case vnet is not up yet.
                    if (!net_common->secondary_dev)
                        return 0;

                    process_dev = net_common->secondary_dev;
                }
            }
        }

        if (phy_duplex == DUPLEX_HALF && current_link)
        {
            printk(KERN_INFO "Can not support half duplex connections.\n");
            printk(KERN_INFO "Try another Ethernet port\n");
        }

        // managing each interface speed information for switch.
        lp_query_interface = netdev_get_priv(process_dev);
        netif_link = lp_query_interface->link_status;

        /* Set MAC speed to match PHY speed */
        if (current_link)
        {
            if (phy_speed == 1000 || phy_speed == 100)
            {
                if (net_st->net_chip_info.adapter_type == ADAPTER_SWITCH)
                {
                    //Since we have have a switch with multiple ports we need to keep an eye on our speed
                    // It can change without a link state change. We force a link down/up event to get the change to user.
                    if (lp_query_interface->mac_speed != phy_speed)
                    {
                        netif_carrier_off(process_dev);
                        lp_query_interface->link_status = 0;
                        lp_query_interface->mac_speed = phy_speed;

#ifdef CONFIG_AKASHI_EMAC_0_SMI_IRQ
                        del_timer(&lp_query_interface->phy_timer);
                        lp_query_interface->phy_timer.expires = jiffies + TIMER_DELAY_100MSEC;
                        timer_setup(&lp_query_interface->phy_timer, link_up_process, 0);
                        add_timer(&lp_query_interface->phy_timer);
#else
                        lp_query_interface->link_status = 1;
                        netif_carrier_on(process_dev);
#endif
                    }
                }
                else if (net_st->net_chip_info.adapter_type == ADAPTER_PHY)
                {
                    if (phy_speed != lp_query_interface->mac_speed)
                    {
                        lp_query_interface->mac_speed = phy_speed;
                    }
                }
                else
                {

                }
            }
            else
            {
                printk(KERN_INFO "10 Mbps not supported\n");
                printk(KERN_INFO "%s disabled\n",process_dev->name);
                lp_query_interface->link_status = 0;
#ifdef CONFIG_AKASHI_EMAC_0_SMI_IRQ
                del_timer(&lp_query_interface->phy_timer);    // kill current timer for link up.
#endif
                continue;
            }
        }

        if (current_link != netif_link)
        {
#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
	    link_st_change = true;
#endif
            if (current_link)
            {
                netif_carrier_off(process_dev);
#ifdef CONFIG_AKASHI_EMAC_0_SMI_IRQ
                del_timer(&lp_query_interface->phy_timer);
                lp_query_interface->phy_timer.expires = jiffies + TIMER_DELAY_100MSEC;
                timer_setup(&lp_query_interface->phy_timer, link_up_process, 0);
                add_timer(&lp_query_interface->phy_timer);
#else
                printk(KERN_INFO "%s: Link carrier restored.\n", process_dev->name);
                lp_query_interface->link_status = 1;
                netif_carrier_on(process_dev);
#endif
            }
            else
            {
#ifdef CONFIG_AKASHI_EMAC_0_SMI_IRQ
                del_timer(&lp_query_interface->phy_timer);    // kill current timer for link up.
#endif
                netif_carrier_off(process_dev);
                printk(KERN_INFO "%s: Link carrier lost.\n", process_dev->name);
                lp_query_interface->link_status = 0;
#ifdef CONFIG_AKASHI_EMAC_0_SMI_IRQ
                lp_query_interface->phy_timer.expires = jiffies + TIMER_DELAY_100MSEC;
                timer_setup(&lp_query_interface->phy_timer, special_link_up_process, 0);
                add_timer(&lp_query_interface->phy_timer);
#else
                netif_carrier_on(process_dev);
#endif
            }
        }
        else
        {
            // Multiple link up/down happened, need to stop current timer & up/down accroding to current condition.
            // condition - currently off + previous off.
            if(!current_link && !netif_link)
            {
                del_timer(&lp_query_interface->phy_timer);
            }
        }

        index_count++;
    } while(red_loop_cond--);

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
    /*
     * In polling mode, HW configuraiton is needed only when the first
     * time or link status change really happens.
     */
    if (!(net_common->check_link_first_time || link_st_change))
        return 0;

    // Mark the check_link_first_time flag into false
    net_common->check_link_first_time = false;
#endif

    // In case redundant PHYs with Akashi FPGA platform, it handles mac control with mixed link speed.
    if (net_st->net_chip_info.adapter_type == ADAPTER_PHY)
    {
        uint32_t syd_mac_switch_ctl = 0;
        uint32_t mac_control_gtx_clock = AUD_SYD_MAC_CONTROL_GTX_CLK;

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
        if (net_common->check_link_first_time || link_st_change)
        {
            net_common->check_link_first_time = false;
        }
#endif

        if (net_common->config.red_en)
            syd_mac_switch_ctl = AUD_SYD_MAC_SWITCH_CONTROL | AUD_SYD_MAC_REDUNDANT_TYPE;

        if (net_st->interface_stat[0].link_status || net_st->interface_stat[1].link_status)
        {
            if (net_st->net_chip_info.phy_redundant)
            {
                int speed_sum;

                speed_sum = net_st->interface_stat[0].link_speed + net_st->interface_stat[1].link_speed;

                // Mixed 1G and 100Mbps ??
                if (speed_sum == 1100)
                {
                    if (net_st->interface_stat[0].link_speed == 100)
                    {
                        // Shut down primary port.
                        sl_isolate_phy(net_st->net_chip_info.phy_address[0], PHY_ISOLATE_ENABLE);
                    }
                    else
                    {
                        // Shut down secondary port
                        sl_isolate_phy(net_st->net_chip_info.phy_address[1], PHY_ISOLATE_ENABLE);
                    }
                }
                else
                {
                    sl_isolate_phy(net_st->net_chip_info.phy_address[0], PHY_ISOLATE_DISABLE);
                    sl_isolate_phy(net_st->net_chip_info.phy_address[1], PHY_ISOLATE_DISABLE);
                }

                // 100M phy chipset - always 25Mhz, MII + GTX clock
                if (net_st->interface_stat[0].link_speed == 1000 || net_st->interface_stat[1].link_speed == 1000)
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock | syd_mac_switch_ctl;
                }
                else if (net_st->interface_stat[0].link_speed == 100 || net_st->interface_stat[1].link_speed == 100)
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_25MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock | syd_mac_switch_ctl;
                }
                else
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock | syd_mac_switch_ctl;
                }
            }
            else
            {
                // This is single phy.
                // 100M phy chipset - always 25Mhz, MII + GTX clock
                if (net_st->interface_stat[0].link_speed == 1000)
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock;
                }
                else
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_25MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock;
                }
            }
        }
        else
        {
            // 100M phy chipset - always 25Mhz, MII + GTX clock
            // Both link is not on and default is 1 G
            if (net_st->net_chip_info.phy_redundant)
            {
                AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock | syd_mac_switch_ctl;
            }
            else
            {
                AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock;
            }
        }
    }

    return 0;
}

#ifdef CONFIG_AKASHI_DEBUG_TIMER
static void debug_timer_func(struct timer_list *t)
{
    net_common_t *net_common = from_timer(net_common, t, debug_timer);

    printk(KERN_INFO "AUD_SYD_SCHED_INTR_CONTROL    0x%x\n", AUD_SYD_SCHED_INTR_CONTROL);
    printk(KERN_INFO "AUD_SYD_SCHED_CONTROL         0x%x\n", AUD_SYD_SCHED_CONTROL);
    printk(KERN_INFO "AUD_SYD_SCHED_FIFO_OCC        0x%x\n", AUD_SYD_SCHED_FIFO_OCC);
    printk(KERN_INFO "AUD_SYD_SCHED_INTR_STATUS     0x%x\n", AUD_SYD_SCHED_INTR_STATUS);

    net_common->debug_timer.expires = jiffies + TIMER_DELAY_1SEC;
    add_timer(&net_common->debug_timer);
}
#endif

static void tx_queue_drain_func(struct timer_list *t)
{
    net_common_t *net_common = from_timer(net_common, t, tx_drain_timer);

    tasklet_schedule(&net_common->send_tasklet);

    //net_common->tx_drain_timer.expires = jiffies + HZ;
    //timer_setup(&net_common->tx_drain_timer, tx_queue_drain_func, 0);
    add_timer(&net_common->tx_drain_timer);
}

// Toggle leds on switch for identifying.
static void network_led_action(char network_type, uint32_t network_ports, char action)
{
    switch_port_led_action action_cmd;

    // DIGICO patch: do not change switch LEDs when MDIO locked out
    if (net_common_context && net_common_context->mdio_locked)
        return;

    if (action == IDENTIFY_LED_FORCE_ON)
    {
        action_cmd = SWITCH_LED_CONTROL_FORCE_ON;
    }
    else if (action == IDENTIFY_LED_FORCE_OFF)
    {
        action_cmd = SWITCH_LED_CONTROL_FORCE_OFF;
    }
    else if (action == IDENTIFY_LED_DEFAULT)
    {
        action_cmd = SWITCH_LED_CONTROL_DEFAULT;
    }
    else
    {
        return;
    }

    if (network_type == ADAPTER_SWITCH)
    {
        int port_num = 0;
        if (network_ports)
        {
            while (network_ports)
            {
                if (network_ports&0x1)
                {
                    sl_marvel_port_led_control(port_num, action_cmd);
                }
                network_ports >>= 1;
                port_num++;
            }
        }
    }
    else
    {
        // This is phy.
    }
}

// Changing status & take an action for led identifying.
static void toggle_identify_led(net_common_t *net_common)
{
    uint32_t network_ports;
    int ports_temp;

    // Get network available ports.
    if (net_common->dante_net_st.net_chip_info.adapter_type == ADAPTER_SWITCH)
    {
        if (net_common->config.red_en == ETH_SWITCH_MODE_REDUNDANT)
        {
            network_ports = (net_common->config.vlan_pri | net_common->config.vlan_sec);
        }
        else if (net_common->config.red_en == ETH_SWITCH_MODE_SWITCH)
        {
            network_ports = net_common->config.vlan_pri;
        }
        else
        {
            ports_temp = net_common->dante_net_st.ports.num_ports;

            network_ports = 0;
            while(ports_temp--)
            {
                network_ports <<= 1;
                network_ports |= 1;
            }
        }
    }
    else
    {
        return;    // TODO :: Support for phy ports identifying
    }

    if (net_common->identify_led_activity == IDENTIFY_LED_FORCE_ON)
    {
        net_common->identify_led_activity = IDENTIFY_LED_FORCE_OFF;
        network_led_action(net_common->dante_net_st.net_chip_info.adapter_type, network_ports, IDENTIFY_LED_FORCE_OFF);
    }
    else if (net_common->identify_led_activity == IDENTIFY_LED_FORCE_OFF)
    {
        net_common->identify_led_activity = IDENTIFY_LED_FORCE_ON;
        network_led_action(net_common->dante_net_st.net_chip_info.adapter_type, network_ports, IDENTIFY_LED_FORCE_ON);
    }
    else if (net_common->identify_led_activity == IDENTIFY_LED_DEFAULT)
    {
        // Back to default status.
        net_common->identify_led_activity = IDENTIFY_LED_NONE;
        network_led_action(net_common->dante_net_st.net_chip_info.adapter_type, network_ports, IDENTIFY_LED_DEFAULT);
    }
}

// Timer callback function for led identify
static void led_identify(struct timer_list *t)
{
    net_common_t *net_common = from_timer(net_common, t, identify_timer);

    if (net_common->dante_net_st.net_chip_info.adapter_type == ADAPTER_SWITCH)
    {
        if (net_common->identify_led_activity != IDENTIFY_LED_NONE)
        {
            /* Set up the timer so we'll get called again in 1 seconds. */
            net_common->identify_timer.expires = jiffies + TIMER_DELAY_1SEC;
            add_timer(&net_common->identify_timer);

            toggle_identify_led(net_common);
        }
    }
}

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
// Timer callback function for link status check
static void check_link_status(struct timer_list *t)
{
    net_common_t *net_common = from_timer(net_common, t, check_link_st_timer);

    /* Set up the timer so we'll get called again in 1 seconds. */
    net_common->check_link_st_timer.expires = jiffies + TIMER_DELAY_1SEC;
    add_timer(&net_common->check_link_st_timer);

    // DIGICO patch: do not poll switch when MDIO locked out
    if (net_common->mdio_locked)
        return;

    network_mii_process(net_common);
}
#endif

static int akashi_is_mac_filter_enable(net_common_t *net_common)
{
    return ((net_common->debug.mac_filter[0] != 0) ||
            (net_common->debug.mac_filter[1] != 0) ||
            (net_common->debug.mac_filter[2] != 0) ||
            (net_common->debug.mac_filter[3] != 0) ||
            (net_common->debug.mac_filter[4] != 0) ||
            (net_common->debug.mac_filter[5] != 0));
}

static inline int rx_queued(void)
{
    return ((AUD_SYD_SCHED_FIFO_OCC & AUD_SYD_SCHED_FIFO_OCC__RX_NOT_EMPTY_MASK) >> 16);
}

static int akashi_print_mac_addr(char *str, uint8_t *mac)
{
    char *ptr = str;

    ptr += sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return (ptr - str);
}


static int denet_rxfifo(net_common_t *net_common, struct sk_buff *skb, u32 fifo_len, u32 *payload_len)
{
    u32* data = (u32*) skb->data;
    u32* ptr = data;
    u8 *octet = (u8 *)skb->data;
    u32 wordcount = fifo_len >> 2;
    u32 extrabytecount = fifo_len & 0x3;
    u32 lastword;
    u8 *extrabytesbuffer;
    int redundancy_port_number = 0;
    u32 rxstmpl = 0;
    u32 rxstmph = 0;
    u32 rxscntl = 0;
    u32 rxscnth = 0;
    s64 rxtmp_sec, rxtmp_nsec;
    long * cbuff = (long *) (skb->cb); // control buffer to store priv meta
    struct skb_shared_hwtstamps *hwtstamps;
    u32 remaining = 0;
    int i;
    u32 port_word;
    char *str;
    int max_len;
    int meta_count;
    u8 *src_mac;
    u8 *dst_mac;

    if (net_common->has_tscount)
    {
        meta_count = 6;
    }
    else
    {
        meta_count = 3;
    }

    /* Payload length is the number of bytes in the FIFO minus the meta data (eg timestamps etc). */
    *payload_len = fifo_len - (meta_count * sizeof(u32));

    /* Read all meta data into temp buf */
    for (i = 0; i < meta_count; i++)
    {
        net_common->rx_meta_buf[i] = AUD_SYD_SCHED_RX_FIFO_BASEADDR;
    }

    hwtstamps = skb_hwtstamps(skb); //hardware timestamp

    /* Word 0 */
    rxstmpl = ntohl(net_common->rx_meta_buf[0]);
    rxtmp_nsec = (s64)((rxstmpl));
    hwtstamps->hwtstamp = rxtmp_nsec;

    /* Word 1 */
    rxstmph = ntohl(net_common->rx_meta_buf[1]);
    rxtmp_sec =  (s64)(rxstmph);
    skb->tstamp = ktime_set(rxtmp_sec, rxtmp_nsec);
    hwtstamps->hwtstamp = ktime_set(rxtmp_sec, rxtmp_nsec);

    /* Word 2 */
    port_word = ntohl(net_common->rx_meta_buf[2]);
    port_word = port_word >> 8;
    port_word &= 0xff;
    if (port_word != REDUNDANCY_INTERFACE_NONE)
    {
        if (port_word == REDUNDANCY_INTERFACE_PORT_0 || port_word == REDUNDANCY_INTERFACE_PORT_1)
        {
            redundancy_port_number = port_word;
        }
        else
        {
            printk(KERN_INFO "Unknown redundancy interface port number %d and fifo length %d\n", port_word, fifo_len);
        }
    }

    if (net_common->has_tscount)
    {
        /* Word 3 */
        rxscntl = ntohl(net_common->rx_meta_buf[3]);
        cbuff[1] = rxscntl;

        /* Word 4 */
        rxscnth = ntohl(net_common->rx_meta_buf[4]);
        cbuff[2] = rxscnth;

        /* Word 5 - throw away, don't care */

        /* Word 6 onwards */
        if (wordcount > 6)
        {
            remaining = wordcount - 6;
        }
    }
    else
    {
        /* Word 3 onwards*/
        if (wordcount > 3)
        {
            remaining = wordcount - 3;
        }
    }

    /* Read all data payload, if any */
    for (i = 0; i < remaining; i++)
    {
        *ptr++ = AUD_SYD_SCHED_RX_FIFO_BASEADDR;
    }

    /*
     * if there are extra bytes to handle, read the last word from the FIFO
     * and insert the extra bytes into the buffer
     */
    if (extrabytecount > 0)
    {
        if (net_common->has_tscount)
        {
            extrabytesbuffer = (u8 *) (data + wordcount - 6);
        }
        else
        {
            extrabytesbuffer = (u8 *) (data + wordcount - 3);
        }
        lastword = _swap_on_load(AUD_SYD_SCHED_RX_FIFO_ADDR, 0);
        /*
         * one extra byte in the last word, put the byte into the next
         * location of the buffer, bytes in a word of the FIFO are
         * ordered from most significant byte to least
         */
        if (extrabytecount == 1)
        {
            extrabytesbuffer[0] = (u8) (lastword >> 24);
        }
        else if (extrabytecount == 2)
        {
            extrabytesbuffer[0] = (u8) (lastword >> 24);
            extrabytesbuffer[1] = (u8) (lastword >> 16);
        }
        else if (extrabytecount == 3)
        {
            extrabytesbuffer[0] = (u8) (lastword >> 24);
            extrabytesbuffer[1] = (u8) (lastword >> 16);
            extrabytesbuffer[2] = (u8) (lastword >> 8);
        }
    }

    dst_mac = octet;
    src_mac = octet + 6;

    if (akashi_is_mac_filter_enable(net_common))
    {
        if (memcmp(src_mac, net_common->debug.mac_filter, 6) == 0)
        {
            goto do_debug;
        }
    }
    else
    {
        goto do_debug;
    }

    /* Return without debug */
    return redundancy_port_number;

do_debug:
    /*
     * This section onwards is for debugging.
     * Warning!!:
     *      Enabling any of the debug bits may cause significant slow down,
     *      and may cause driver to mal-function. Use with care!
     */
    if ((net_common->debug.flag & AKASHI_DEBUG_BIT_MASK_RX_TS_HEX) ||
        (net_common->debug.flag & AKASHI_DEBUG_BIT_MASK_RX_TS_DEC))
    {
        memset(net_common->debug.buf, 0, sizeof(net_common->debug.buf));
        str = net_common->debug.buf;
        str += sprintf(str, "RX %4d bytes, ", *payload_len);
        str += sprintf(str, "port %x, ", port_word);
        str += akashi_print_mac_addr(str, src_mac);
        str += sprintf(str, " -> ");
        str += akashi_print_mac_addr(str, dst_mac);
        str += sprintf(str, " ");

        if (net_common->debug.flag & AKASHI_DEBUG_BIT_MASK_RX_TS_HEX)
        {
            str += sprintf(str, "rxstmp %08x.%08x, ", rxstmph, rxstmpl);
            //str += sprintf(str, "rxscnt %08x.%08x, ", rxscnth, rxscntl);
        }
        else
        {
            str += sprintf(str, "rxstmp %u.%u, ", rxstmph, rxstmpl);
            //str += sprintf(str, "rxscnt %u.%u, ", rxscnth, rxscntl);
            str += sprintf(str, "ktime %lld ", ktime_to_ns(hwtstamps->hwtstamp));
        }
        printk(KERN_INFO "%s\n", net_common->debug.buf);
    }

    if (net_common->debug.flag & AKASHI_DEBUG_BIT_MASK_RX_PAYLOAD)
    {
        memset(net_common->debug.buf, 0, sizeof(net_common->debug.buf));
        str = net_common->debug.buf;
        max_len = *payload_len > 200 ? 200 : *payload_len;
        str += sprintf(str, "RX %4d bytes, ", *payload_len);
        str += sprintf(str, "port %x, ", port_word);
        str += akashi_print_mac_addr(str, src_mac);
        str += sprintf(str, " -> ");
        str += akashi_print_mac_addr(str, dst_mac);
        str += sprintf(str, " ");
        printk(KERN_INFO "%s\n", net_common->debug.buf);
        akashi_dump_data(octet, max_len);
    }

    return redundancy_port_number;
}

static void rx_intr(int state)
{
    if (state)
    {
        // enable
        AUD_SYD_SCHED_INTR_CONTROL |= AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_INTR_EN;
    }
    else
    {
        // disable
        AUD_SYD_SCHED_INTR_CONTROL &= ~AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_INTR_EN;
    }
}

static irqreturn_t denet_rx_interrupt(int irq, void *dev_id)
{
    net_common_t *net_common = (net_common_t *)dev_id;

    // disable rx interrupts
    rx_intr(0);

    tasklet_hi_schedule(&net_common->recv_tasklet);

    return IRQ_HANDLED;
}

static void denet_recvBH(unsigned long p)
{
    net_common_t *net_common = (net_common_t *)p;
    net_local_t *net_local = netdev_get_priv(net_common->primary_dev);
    struct sk_buff *skb;
    unsigned int align;
    u32 fifo_len;
    u32 payload_len;
    int if_index;
    int work_done = 0;

    if (!rx_queued())
    {
        printk(KERN_INFO "Called while FIFO is empty\n");
        rx_intr(1);
        return;
    }

    while (rx_queued())
    {
        // While there are queued packets
        fifo_len = _swap_on_load(AUD_SYD_SCHED_RX_FIFO_ADDR, 0);
        if (fifo_len == 0)
        {
            net_local->stats.rx_dropped++;

            printk(KERN_ERR "%s: Packet has no length - resetting\n", __func__);
            spin_lock(&net_common->reset_lock);
            rx_hw_reset(net_local->dev, HALF_DUPLEX);
            spin_unlock(&net_common->reset_lock);
            break;
        }

        if (fifo_len > net_common->max_rx_len)
        {
            printk(KERN_ERR "%s: huge packet 0x%x word+1 0x%x word+2 0x%x\n",
                __func__, fifo_len, _swap_on_load(AUD_SYD_SCHED_RX_FIFO_ADDR, 0), _swap_on_load(AUD_SYD_SCHED_RX_FIFO_ADDR, 0));
            spin_lock(&net_common->reset_lock);
            rx_hw_reset(net_local->dev, HALF_DUPLEX);
            spin_unlock(&net_common->reset_lock);
            break;
        }

        if (!(skb = /*dev_ */alloc_skb((fifo_len & 0xffff) + ALIGNMENT, GFP_ATOMIC)))
        {
            /* Couldn't get memory. */
            net_local->stats.rx_dropped++;
            printk(KERN_ERR "%s: Could not allocate receive buffer.\n", __func__);
            break;
        }

        /*
         * A new skb should have the data word aligned, but this code is
         * here just in case that isn't true...  Calculate how many
         * bytes we should reserve to get the data to start on a word
         * boundary.  */
        align = BUFFER_ALIGN(skb->data);
        if (align)
            skb_reserve(skb, align);

        if_index = denet_rxfifo(net_common, skb, fifo_len, &payload_len);
        if (if_index == -1)
        {
            int need_reset = 1;

            net_local->stats.rx_errors++;
            dev_kfree_skb(skb);
            printk(KERN_ERR "%s: Could not receive buffer, error=%d%s.\n",
                    __func__, if_index,
                    need_reset ? ", resetting device." : "");
            if (need_reset)
            {
                spin_lock(&net_common->reset_lock);
                rx_hw_reset(net_local->dev, HALF_DUPLEX);
                spin_unlock(&net_common->reset_lock);
            }

            break;
        }

        skb_put(skb, payload_len); /* Tell the skb how much data we got. */

        /* Microblaze can't do unaligned struct accesses, meaning that the
           IP headers must reside on a word-boundary.  However, we must also
           keep the ethernet header intact, for the packet_socket interface to
           work.  The ethernet header is 22 bytes long...

           The solution?  Move the packet contents so that the ethernet
           header is halfword aligned, thus the IP header is word aligned.
           It's not pretty, but it works.  JW
         */
        /* Move the packet data.  Earlier code ensured that skb->head
           is word aligned, so skb->head+2 will always be halfword aligned */

        memmove(skb->head + 2, skb->data, skb->len);

        /* fix up skbuff ptrs */
        skb->tail -= (skb->data - (skb->head + 2));
        skb->data = skb->head + 2;

        if ((if_index == REDUNDANCY_INTERFACE_PORT_1) && (net_common->config.red_en & ETH_SWITCH_MODE_REDUNDANT))
        {
            if (net_common->secondary_dev)
            {
                /* Fill out required meta-data for eth1 */
                skb->dev = net_common->secondary_dev;
                skb->protocol = eth_type_trans(skb, skb->dev);
                skb->ip_summed = CHECKSUM_NONE;

                net_local = netdev_get_priv(net_common->secondary_dev);
                net_local->stats.rx_packets++;
                net_local->stats.rx_bytes += skb->len;
                netif_rx(skb);
            }
            else
            {
                /* eth1 not up, drop the packet */
                dev_kfree_skb(skb);
            }
        }
        else
        {
            /* Fill out required meta-data for eth0 */
            skb->dev = net_common->primary_dev;
            skb->protocol = eth_type_trans(skb, skb->dev);
            skb->ip_summed = CHECKSUM_NONE;

            net_local->stats.rx_packets++;
            net_local->stats.rx_bytes += skb->len;
            netif_rx(skb);
        }

        if (++work_done >= AKASHI_RX_BUDGET)
            goto rx_done;
    } //DEMAC_RxQueued

rx_done:
    // re-enable rx interrupts
    rx_intr(1);
}

static int denet_txfifo(net_common_t *net_common, struct sk_buff *skb, u16 timeval)
{
    u32 fifocount, priority_fifocount;
    u32 wordcount, extrabytecount, maxwords;
    u32* data = (u32*) skb->data;
    u32 fifo_reg = 0, fifo_reg_ts = 0;
    u16 val = timeval;
    int priority_flag = 0;
    txts_queue_t *q = &net_common->txts_queue;

    wordcount = skb->len >> 2;
    extrabytecount = skb->len & 0x3;

    maxwords = (extrabytecount > 0) ? wordcount + 1 : wordcount;
    priority_flag = timeval && net_common->has_priority_queue;

    // priority queue free space check
    if (priority_flag)
    {
        priority_fifocount = AUD_SYD_SCHED_PRIORITY_FIFO_FREE_SPACE & AUD_SYD_SCHED_FIFO_OCC__TX_FREE_MASK;
        if (priority_fifocount < maxwords)
        {
            // printk(KERN_INFO "no room in tx priority fifo, fifo free space = %d words, tx data = %d words\n", priority_fifocount, maxwords);
            return -1;
        }
    }
    else
    {
        fifocount = AUD_SYD_SCHED_FIFO_OCC & AUD_SYD_SCHED_FIFO_OCC__TX_FREE_MASK;
        if (fifocount < maxwords)
        {
            // printk(KERN_INFO "no room in tx fifo, fifo free space = %d words, tx data = %d words\n", fifocount, maxwords);
            return -1;
        }
        else
        {
            if(tx_debug_print%100 == 0)
            {
                //printk(KERN_INFO "%s: all g", __func__);
            }
            tx_debug_print++;
        }
    }

    if (val)
    {
        //To extract Dest Address and port no in buffer
        u8 * ptr = (u8*) skb->data;

        if (q->txts_status[q->head_to_send] != TXTS_STATUS_READY)
        {
            printk(KERN_WARNING "tx timstamp overflow, head %d status %d\n", q->head_to_send, q->txts_status[q->head_to_send]);
            if (q->txts_status[q->head_to_send] == TXTS_STATUS_FD_ASSIGNED)
                printk(KERN_WARNING "Reason: the related TS interrupt hasn't been generated\n");
            else if (q->txts_status[q->head_to_send] == TXTS_STATUS_TX_INT)
                printk(KERN_WARNING "Reason: the related TS hasn't been consumed\n");
        }

        q->txts[q->head_to_send].fd = (int)val;
        if (ptr[12] == 0x88 && ptr[13] == 0xf7)
        {
            //layer2 ptp (802.1as)
            q->txts[q->head_to_send].dest_ipv4 = 0;
            q->txts[q->head_to_send].dest_port = 0;
            q->txts[q->head_to_send].msg_type = ptr[14] & 0x0f;//extracts message_type
            q->txts[q->head_to_send].ptp_version = 0x2;
        }
        else
        {
            memcpy(&q->txts[q->head_to_send].dest_ipv4, ptr+30, 4);
            q->txts[q->head_to_send].dest_port = ptr[36] << 8 | ptr[37];
            q->txts[q->head_to_send].ptp_version = (ptr[43] & 0x0f);
        }
        q->txts_status[q->head_to_send] = TXTS_STATUS_FD_ASSIGNED;
        //printk("ts %d\n",txts_queue.txts[txts_queue.head_to_send].fd);
        //case 6101: which interface?
        if (skb->dev == net_common->secondary_dev)
        {
            q->txts[q->head_to_send].iface_index = 1;
            fifo_reg_ts |= AUD_SYD_SCHED_TXFIFO__TS_REQ_SEC;
        }
        else
        {
            q->txts[q->head_to_send].iface_index = 0;
        }

        fifo_reg = AUD_SYD_SCHED_TX_FIFO__TIMESTAMP_PKT;
        fifo_reg_ts |= AUD_SYD_SCHED_TXFIFO__TS_REQ_TIME;
        fifo_reg_ts |= (q->ids[q->head_to_send] << AUD_SYD_SCHED_TXFIFO__TS_REQ_ID_SHIFT);
#ifdef CONFIG_AKASHI_DEBUG_TIMESTAMP_PKT
        printk(KERN_INFO "fifo_reg_ts [%d] %08x  id %d\n", q->head_to_send, fifo_reg_ts, q->ids[q->head_to_send]);
#endif
        q->head_to_send++;
        if (q->head_to_send >= TXTS_QUEUE_SIZE)
        {
            q->head_to_send = 0;
        }
    }

    /* When redundancy is enabled, add tx interface number 0: eth0, 1: eth1 */
    if (skb->dev == net_common->secondary_dev)
    {
        AUD_SYD_SCHED_TX_FIFO__DEST_PHY1(fifo_reg);
    }
    else
    {
        AUD_SYD_SCHED_TX_FIFO__DEST_PHY0(fifo_reg);
    }

    AUD_SYD_SCHED_TX_FIFO_RELEASE = fifo_reg;

    // write len into fifo
    if (priority_flag)
    {
        // write len into fifo
        _swap_on_store(AUD_SYD_SCHED_TX_PRIORITY_FIFO_ADDR , 0, (skb->len | fifo_reg_ts));
    }
    else
    {
        // write len into fifo
        _swap_on_store(AUD_SYD_SCHED_TX_FIFO_ADDR , 0, (skb->len | fifo_reg_ts));
    }

    // write out packet
    if (priority_flag)
    {
        for (fifocount = 0; fifocount < wordcount; fifocount++)
        {
            AUD_SYD_SCHED_TX_PRIORITY_FIFO_BASEADDR = data[fifocount];
        }
    }
    else
    {
        for (fifocount = 0; fifocount < wordcount; fifocount++)
        {
            AUD_SYD_SCHED_TX_FIFO_BASEADDR = data[fifocount];
        }
    }

    if (extrabytecount > 0)
    {
        u32 lastword = 0;
        u8 *extrabytesbuffer = (u8 *) (data + wordcount);

        if (extrabytecount == 1)
        {
            lastword = extrabytesbuffer[0] << 24;
        }
        else if (extrabytecount == 2)
        {
            lastword = extrabytesbuffer[0] << 24 | extrabytesbuffer[1] << 16;
        }
        else if (extrabytecount == 3)
        {
            lastword = extrabytesbuffer[0] << 24 | extrabytesbuffer[1] << 16
                | extrabytesbuffer[2] << 8;
        }

        if (priority_flag)
        {
            // write len into fifo
            _swap_on_store(AUD_SYD_SCHED_TX_PRIORITY_FIFO_ADDR , 0, lastword);
        }
        else
        {
            // write len into fifo
            _swap_on_store(AUD_SYD_SCHED_TX_FIFO_ADDR , 0, lastword);
        }
    }

    // transmit it
    if (priority_flag)
    {
        AUD_SYD_SCHED_TX_FIFO_RELEASE = AUD_SYD_SCHED_TX_FIFO__RELEASE_PRIORITY_PKT;
        /*
        if (txfifo_ptp_debugcounter%100 == 0)
        {
            printk(KERN_INFO "%s: writing PTP to priority TX FIFO\n", __func__);
        }
        txfifo_ptp_debugcounter++;
        */
    }
    else
    {
        AUD_SYD_SCHED_TX_FIFO_RELEASE = AUD_SYD_SCHED_TX_FIFO__RELEASE_PKT;
        /*
        if (txfifo_debugcounter%1000 == 0)
        {
            printk(KERN_INFO "%s: We are writing to TX FIFO\n", __func__);
        }
        txfifo_debugcounter++;
        */
    }

    if (net_common->debug.flag & AKASHI_DEBUG_BIT_MASK_TX_PAYLOAD)
    {
        char *str;
        int max_len;
        u8 *octet = (u8 *)skb->data;
        memset(net_common->debug.buf, 0, sizeof(net_common->debug.buf));
        str = net_common->debug.buf;
        max_len = skb->len > 200 ? 200 : skb->len;
        str += sprintf(str, "TX %4d bytes, ", skb->len);
        printk(KERN_INFO "%s\n", net_common->debug.buf);
        akashi_dump_data(octet, max_len);
    }

    return 0;
}

static void denet_sendBH(unsigned long p)
{
    net_common_t *net_common = (net_common_t *)p;
    struct sk_buff *skb;

    while (1)
    {
        spin_lock_bh(&net_common->tx_lock);
        if (skb_queue_empty(&net_common->tx_skb_list))
        {
            spin_unlock_bh(&net_common->tx_lock);
            return;
        }
        skb = skb_dequeue(&net_common->tx_skb_list);
        spin_unlock_bh(&net_common->tx_lock);
        if (skb)
        {
            int ret = denet_txfifo(net_common, skb, skb->tstamp);
            if (ret != 0)
            {
                /* Failed to send out this packet,
                * Add it back to the head of the list for next BH job.
                */
                spin_lock_bh(&net_common->tx_lock);
                skb_queue_head(&net_common->tx_skb_list, skb);
                spin_unlock_bh(&net_common->tx_lock);
                return;
            }
            else
            {
                if(netif_queue_stopped(skb->dev))
                {
                    if(skb_queue_len(&net_common->tx_skb_list) < TX_SKB_LIST_WAKE_THRESHOLD)
                    {
                        //printk(KERN_INFO "%s: Transmit queue drained. Queue len now = %d, restarting queue.\n", skb->dev->name, skb_queue_len(&net_common->tx_skb_list));
                        //printk(KERN_INFO "%s: remove tx drain queue timer\n", __func__);
                        del_timer(&net_common->tx_drain_timer);
                        netif_wake_queue(skb->dev);
                    }
                }
                /* Notify kernel on reception of new skb. */
                //netif_wake_queue(skb->dev);

                /* Done with this skb, get rid of it for good. */
                dev_kfree_skb(skb);
            }
        }
    }
}

// Handling for deferred priority socket buffer
static void denet_priority_sendBH(unsigned long p)
{
    net_common_t *net_common = (net_common_t *)p;
    struct sk_buff *skb;

    while (1)
    {
        spin_lock_bh(&net_common->tx_priority_lock);
        if (skb_queue_empty(&net_common->tx_priority_skb_list))
        {
            spin_unlock_bh(&net_common->tx_priority_lock);
            return;
        }
        skb = skb_dequeue(&net_common->tx_priority_skb_list);
        spin_unlock_bh(&net_common->tx_priority_lock);
        if (skb)
        {
            int ret = denet_txfifo(net_common, skb, skb->tstamp);
            if (ret != 0)
            {
                /* Failed to send out this packet,
                * Add it back to the head of the list for next BH job.
                */
                spin_lock_bh(&net_common->tx_priority_lock);
                skb_queue_head(&net_common->tx_priority_skb_list, skb);
                spin_unlock_bh(&net_common->tx_priority_lock);
                return;
            }
            else
            {
                /* Notify kernel on reception of new skb. */
                netif_wake_queue(skb->dev);

                /* Done with this skb, get rid of it for good. */
                dev_kfree_skb(skb);
            }
        }
    }
}

static irqreturn_t denet_tx_interrupt(int irq, void *dev_id)
{
    unsigned long intr_status;
    net_common_t *net_common = (net_common_t *)dev_id;

    intr_status = AUD_SYD_SCHED_INTR_STATUS;

    /* handling for deferred priority socket buffer */
    if (intr_status & AUD_SYD_SCHED_INTR_STATUS__PRIORITY_DATA)
    {
        tasklet_schedule(&net_common->priority_send_tasklet);
    }

    if (intr_status & AUD_SYD_SCHED_INTR_STATUS__TX_FIFO)
    {
        tasklet_schedule(&net_common->send_tasklet);
    }

    return IRQ_HANDLED;
}

static int akashi_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    struct sk_buff *new_skb;
    unsigned int len, align;

    len = skb->len;

    // Need this or alignment is stuffed on microblaze

    /*
     * The packet FIFO requires the buffers to be 32/64 bit aligned.
     * The sk_buff data is not 32/64 bit aligned, so we have to do this
     * copy.  As you probably well know, this is not optimal.
     */
    if (!(new_skb = /*dev_ */alloc_skb(len + ALIGNMENT, GFP_ATOMIC)))
    {
        /* We couldn't get another skb. */
        dev_kfree_skb(skb);
        net_local->stats.tx_dropped++;
        printk(KERN_ERR "%s: Could not allocate transmit buffer.\n", dev->name);
        netif_stop_queue(dev);
        return NETDEV_TX_BUSY;
    }
    /*
     * A new skb should have the data word aligned, but this code is
     * here just in case that isn't true...  Calculate how many
     * bytes we should reserve to get the data to start on a word
     * boundary.  */
    align = BUFFER_ALIGN(new_skb->data);
    if (align)
    {
        skb_reserve(new_skb, align);
    }

    /* Copy the data from the original skb to the new one. */
    skb_put(new_skb, len);
    memcpy(new_skb->data, skb->data, len);
#ifdef CONFIG_AKASHI_DEBUG_DUMP_DATA
    akashi_dump_data(new_skb->data, len);
#endif

    /* Copy pointer to net_device structure */
    new_skb->dev = dev;

    if (skb->sk)
    {
        /* Store the timestamp from the source skbuff into our own */
        new_skb->tstamp = skb->sk->sk_tsflags;
#ifdef CONFIG_AKASHI_DEBUG_TIMESTAMP_PKT
        if (new_skb->tstamp)
        {
            printk(KERN_INFO "%s: update new_tstampns done - new_tstampns = %lld\n", __func__, new_skb->tstamp);
        }
#endif
    }
    else
    {
        new_skb->tstamp = 0;
    }

    /* Get rid of the original skb. */
    dev_kfree_skb(skb);

    /* Add the newest packet that we just received into the tail of the queue. */
    /* We must have at least one packet in the queue, so always schedule the tasklet to do the work. */
    if (new_skb->tstamp && net_common->has_priority_queue)
    {
        spin_lock_bh(&net_common->tx_priority_lock);
        skb_queue_tail(&net_common->tx_priority_skb_list, new_skb);
        spin_unlock_bh(&net_common->tx_priority_lock);
        tasklet_schedule(&net_common->priority_send_tasklet);
    }
    else
    {
        spin_lock_bh(&net_common->tx_lock);
        skb_queue_tail(&net_common->tx_skb_list, new_skb);
        spin_unlock_bh(&net_common->tx_lock);
        tasklet_schedule(&net_common->send_tasklet);
        if (skb_queue_len(&net_common->tx_skb_list) >= TX_SKB_LIST_STOP_THRESHOLD)
        {
            netif_stop_queue(dev);
            // printk(KERN_ERR "%s: Transmit queue is too long. Queue len = %d, stopping queue to drain.\n", dev->name, skb_queue_len(&net_common->tx_skb_list));
            net_common->tx_drain_timer.expires = jiffies + TIMER_DELAY_1SEC/2;
            timer_setup(&net_common->tx_drain_timer, tx_queue_drain_func, 0);
            add_timer(&net_common->tx_drain_timer);
        }
    }

    net_local->stats.tx_packets++;
    net_local->stats.tx_bytes += len;
    dev->_tx->trans_start = jiffies;

    return NETDEV_TX_OK;
}

static irqreturn_t denet_err_interrupt(int irq, void *dev_id)
{
    printk(KERN_INFO "%s()\n", __func__);

    return IRQ_HANDLED;
}

static irqreturn_t denet_ts_interrupt(int irq, void *dev_id)
{
    net_common_t *net_common = (net_common_t *)dev_id;
    timestamp_info_t * ptime_info;
    int iface_index, i, j, k, count;
    akashi_timespec_t timestamp;
    akashi_timespec_t timestamp_monotonic;

    u32 ts_id_reg = AUD_SYD_MAC_TX_TS_ID;
    txts_queue_t *q = &net_common->txts_queue;

    //get num of ts entries in queue
    count = (ts_id_reg >> AUD_SYD_MAC_TX_TS_NUM_ENTRIES_SHIFT) & 0x3;
    for (k = 0; k < count; k++)
    {
        u16 ts_id = AUD_SYD_MAC_TX_TS_ID & 0xff;
        iface_index = (AUD_SYD_MAC_TX_SEC_HI >> 31);
        timestamp.tv_sec = AUD_SYD_MAC_TX_SEC_LO;
        timestamp.tv_nsec = AUD_SYD_MAC_TX_NSEC;
        if (net_common->has_tscount)
        {
            if (iface_index == 0)
            {
                timestamp_monotonic.tv_sec = AUD_SYD_MAC_0_TX_COUNTER_HI;
                timestamp_monotonic.tv_nsec = AUD_SYD_MAC_0_TX_COUNTER_LO;
            }
            else
            {
                timestamp_monotonic.tv_sec = AUD_SYD_MAC_1_TX_COUNTER_HI;
                timestamp_monotonic.tv_nsec = AUD_SYD_MAC_1_TX_COUNTER_LO;
            }
        }

        j = q->head_sent;
        for (i = 0; i < TXTS_QUEUE_SIZE; i++)
        {

            if (q->txts_status[j] == TXTS_STATUS_FD_ASSIGNED && ts_id == q->ids[j])
            {
                ptime_info = &q->txts[j];
                ptime_info->tx_timestamp = timestamp;
                ptime_info->tx_timestamp_monotonic = timestamp_monotonic;
                q->txts_status[j] = TXTS_STATUS_TX_INT;
                q->head_sent = j+1;
                if (q->head_sent >= TXTS_QUEUE_SIZE)
                    q->head_sent = 0;
                break;
            }
            j++;
            if (j >= TXTS_QUEUE_SIZE)
                j = 0;
        }

        if(i == TXTS_QUEUE_SIZE -1 )
        {
            printk("warning: no matching buffer in queue for the acquired ts\n");
        }

    }

    wake_up_interruptible(&net_common->tsqueue);

    return IRQ_HANDLED;
}

static void denet_hw_start(net_common_t *net_common)
{
    unsigned int int_ctl_reg;
    u32 reg_val;

    // Enable Everything
    int_ctl_reg = AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_RST |
            AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_INTR_EN |
            AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_RST |
            AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_INTR_EN;

    // adding priority queue fifo interrupt
    if (net_common->has_priority_queue)
    {
        int_ctl_reg |= AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_RST | AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_INT_EN;
    }

    AUD_SYD_SCHED_INTR_CONTROL |= int_ctl_reg;

    reg_val = AUD_SYD_SCHED_CONTROL;
    reg_val &= ~(AUD_SYD_SCHED_CONTROL__RXEN |
            AUD_SYD_SCHED_CONTROL__PRI_TXEN |
            AUD_SYD_SCHED_CONTROL__SEC_TXEN |
            AUD_SYD_SCHED_CONTROL__REDEN);

    if (net_common->config.red_en & ETH_SWITCH_MODE_REDUNDANT)
    {
        printk(KERN_INFO "%s: enable interrupt under redundant mode\n", __func__);
        AUD_SYD_SCHED_CONTROL = reg_val | AUD_SYD_SCHED_CONTROL__RXEN |
            AUD_SYD_SCHED_CONTROL__PRI_TXEN |
            AUD_SYD_SCHED_CONTROL__SEC_TXEN |
            AUD_SYD_SCHED_CONTROL__REDEN;
    }
    else
    {
        printk(KERN_INFO "%s: enable interrupt\n", __func__);
        AUD_SYD_SCHED_CONTROL = reg_val | AUD_SYD_SCHED_CONTROL__RXEN |
                                AUD_SYD_SCHED_CONTROL__PRI_TXEN;
    }
}

static void denet_hw_stop(net_common_t *net_common)
{
    /*
    unsigned int int_ctl_reg;

    int_ctl_reg = ~AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_INTR_EN & ~AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_INTR_EN;

    if (net_common->has_priority_queue)
    {
        int_ctl_reg &= ~AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_INT_EN;
    }
    */
    printk(KERN_INFO "%s: disable all interrupts\n", __func__);
    /* Disable all interrupts */
    //AUD_SYD_SCHED_INTR_CONTROL &= int_ctl_reg;
    AUD_SYD_SCHED_INTR_CONTROL = 0;

    /* Only disable RX,TX,REDUN bits */
    AUD_SYD_SCHED_CONTROL &= ~(AUD_SYD_SCHED_CONTROL__RXEN |
                    AUD_SYD_SCHED_CONTROL__PRI_TXEN |
                    AUD_SYD_SCHED_CONTROL__SEC_TXEN |
                    AUD_SYD_SCHED_CONTROL__REDEN);

    printk(KERN_INFO "%s: clear all interrupts\n", __func__);
    /* Clear all interrupts */
    AUD_SYD_SCHED_INTR_STATUS = (AUD_SYD_SCHED_INTR_STATUS__TX_FIFO |
                                AUD_SYD_SCHED_INTR_STATUS__RX_FIFO |
                                AUD_SYD_SCHED_INTR_STATUS__TX_FIFO_OVERFLOW |
                                AUD_SYD_SCHED_INTR_STATUS__TX_FIFO_UNDERFLOW |
                                AUD_SYD_SCHED_INTR_STATUS__RX_FIFO_OVERFLOW |
                                AUD_SYD_SCHED_INTR_STATUS__RX_FLOW |
                                AUD_SYD_SCHED_INTR_STATUS__KAMISS |
                                AUD_SYD_SCHED_INTR_STATUS__PRIORITY_DATA |
                                AUD_SYD_SCHED_INTR_STATUS__PRIORITY_FIFO_OVERFLOW |
                                AUD_SYD_SCHED_INTR_STATUS__PRIORITY_FIFO_UNDERFLOW);

    /* Remove and free all buffers inside the TX skbuff list */
    spin_lock_bh(&net_common->tx_lock);
    skb_queue_purge(&net_common->tx_skb_list);
    spin_unlock_bh(&net_common->tx_lock);
    spin_lock_bh(&net_common->tx_priority_lock);
    skb_queue_purge(&net_common->tx_priority_skb_list);
    spin_unlock_bh(&net_common->tx_priority_lock);
}

#if defined(CONFIG_AKASHI_EMAC_0_SMI_IRQ) && !defined(CONFIG_AKASHI_PS_MAC)
static irqreturn_t smi_interrupt(int irq, void *dev_id)
{
    net_common_t *net_common = (net_common_t *)dev_id;

    tasklet_schedule(&net_common->smi_tasklet);

    return IRQ_HANDLED;
}
#endif

// Bottom half process for smi interrupt
static void denet_smiBH(unsigned long p)
{
    net_common_t *net_common = (net_common_t *)p;


    // switch interrupt service - clear interrupts status for phys & others
    if (net_common->dante_net_st.net_chip_info.adapter_type == ADAPTER_SWITCH)
    {
        uint16_t global2_status_reg;

        while (1)
        {
            // get & clear global interrupt
            sl_get_global2_int_src_reg(&global2_status_reg);

            // Clear interrupt status for external ports
            sl_clr_ext_phys_all(net_common->config.external_phys, net_common->dante_net_st.ext_phy_int_clear_reg);

            // If there on phy/serdes interrupt, then just return.
            if(sl_is_interrupt(global2_status_reg) != SWITCH_INT_AVAILABLE)
            {
                // clear global interrupt register 1 & 2
                sl_clear_global_interrupt();
                break;
            }

            // clear all switch phys interrupt
            sl_clear_intr_switch_phy_all(global2_status_reg);

            // clear SERDES interrupt status
            sl_clear_intr_serdes_all(global2_status_reg);

            // clear global interrupt register 1 & 2
            sl_clear_global_interrupt();
        }
    }
    else
    {
        // Clear all phy chipset interrupt
        sl_clear_intr_phy_all();
    }

    // Scaning changed ports.
    network_mii_process(net_common);
}

static void akashi_common_hw_init(net_common_t *net_common)
{
    denet_hw_start(net_common);

    /* Set up the LED identify timer. */
    // timer for identify on led
    net_common->identify_timer.expires = jiffies + TIMER_DELAY_1SEC;
    timer_setup(&net_common->identify_timer, led_identify, 0);
    net_common->identify_led_activity = IDENTIFY_LED_NONE;

#ifdef CONFIG_AKASHI_EMAC_0_SMI_IRQ
    network_mii_process(net_common);
#else
    // timer for check link status
    net_common->check_link_first_time = true;
    net_common->check_link_st_timer.expires = jiffies + TIMER_DELAY_1SEC;
    timer_setup(&net_common->check_link_st_timer, check_link_status, 0);
    add_timer(&net_common->check_link_st_timer);
#endif

}

static void akashi_common_hw_release(net_common_t *net_common)
{
    // Stop Rx /Tx
    // Turn off irqs
    denet_hw_stop(net_common);
}

#ifdef CONFIG_AKASHI_CONFIGURE_SWITCH
static int akashi_ifindex_get(struct net_device *dev)
{
    int ifindex = 1;
    struct net *net = dev_net(dev);
    for (;;)
    {
        if (!__dev_get_by_index(net, ifindex))
            return ifindex;
        ifindex++;
    }
}
#endif

/* --------------------------------------------------------------------------
 * ioctl: let user programs configure this interface
 */
int vnet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    /* mii_ioctl_data has 4 u16 fields: phy_id, reg_num, val_in & val_out */
    struct linkutil util;

    #if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
        // Private dev IO control
        if (cmd >= SIOCDEVPRIVATE && cmd <= SIOCDEVPRIVATE + 15)
            return denet_private_ioctl(dev, rq, NULL, cmd);
    #endif

    /* process the command */
    switch (cmd)
    {
#ifdef CONFIG_DEPRECATED_CODE_FIXME /* FIXME: deprecated */
        case SIOCETHTOOL:
        {
            return vnet_do_ethtool_ioctl(dev, rq);
        }
#endif

        case SIOCDEVLINKUTIL:
        {
            util.tx_bytes_sec = AUD_SYD_MAC_TX1_BYTES;
            util.rx_bytes_sec = AUD_SYD_MAC_RX1_GOOD_BYTES;

            if (copy_to_user(rq->ifr_data, &util, sizeof (struct linkutil)))
            {
                return  -EFAULT;
            }
            return 0;
        }

        case SIOCDEVERRORCLEAR:
        {
            clear_error_counter(dev);
            return 0;
        }

        default:
        {
            return -EOPNOTSUPP;
        }
    }

    return 0;
}

static int vnet_open(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    net_local_t *net_local_primary = netdev_get_priv(net_common->primary_dev);

    /* In this routine, we assume primary interface is always available */
    printk(KERN_INFO "%s: %s %s, %s %s\n", __func__,
        dev->name, net_local->up ? "up" : "down",
        net_common->primary_dev->name, net_local_primary->up ? "up" : "down");

    /* Both interfaces down, init hw */
    if (!net_local->up && !net_local_primary->up)
    {
        akashi_common_hw_init(net_common);
    }

    /* Mark ourself as up */
    net_local->up = true;

    /* Tell kernel to start transmit */
    netif_start_queue(dev);

    return 0;
}

static int vnet_stop(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    net_local_t *net_local_primary = netdev_get_priv(net_common->primary_dev);

    /* Mark ourself as down */
    net_local->up = false;

    /* Primary interface is down, hw no longer in use */
    if (!net_local_primary->up)
    {
        akashi_common_hw_release(net_common);
    }

    /* Tell kernel to stop transmit */
    netif_stop_queue(dev);

    return 0;
}

static int denet_open(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    net_local_t *net_local_secondary;

    /* Secondary link may not be available, check before dereferencing the pointer */
    if (net_common->secondary_dev)
    {
        net_local_secondary = netdev_get_priv(net_common->secondary_dev);

        printk(KERN_INFO "%s: %s %s, %s %s\n", __func__,
            dev->name, net_local->up ? "up" : "down",
            net_common->secondary_dev->name, net_local_secondary->up ? "up" : "down");

        /* Both interfaces down, init hw */
        if (!net_local->up && !net_local_secondary->up)
        {
            akashi_common_hw_init(net_common);
        }
    }
    else
    {
        /* Primary link only, always bring up the hardware */
        akashi_common_hw_init(net_common);
    }

    /* Mark ourself as up */
    net_local->up = true;

    /* Tell kernel to start transmit */
    netif_start_queue(dev);

    return 0;
}

static int denet_stop(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    net_local_t *net_local_secondary;

    /* Secondary link may not be available, check before dereferencing the pointer */
    if (net_common->secondary_dev)
    {
        net_local_secondary = netdev_get_priv(net_common->secondary_dev);

        /* Mark ourself as down */
        net_local->up = false;

        /* Secondary interface is down, hw no longer in use */
        if (!net_local_secondary->up)
        {
            akashi_common_hw_release(net_common);
        }
    }
    else
    {
        /* Primary link only, always shutdown the hardware */
        akashi_common_hw_release(net_common);
    }

    /* Tell kernel to stop transmit */
    netif_stop_queue(dev);

    return 0;
}

static struct net_device_stats *akashi_get_stats(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);

    // We classify all errors as crc errors because we have no further breakdown
    if (is_primary_dev(dev))
    {
        net_local->stats.rx_crc_errors = AUD_SYD_MAC_RX0_BAD_PACKETS;
    }
    else
    {
        net_local->stats.rx_crc_errors = AUD_SYD_MAC_RX1_BAD_PACKETS;
    }

    return &net_local->stats;
}

static struct net_device_stats *denet_get_stats(struct net_device *dev)
{
    return akashi_get_stats(dev);
}

static struct net_device_stats *vnet_get_stats(struct net_device *dev)
{
    return akashi_get_stats(dev);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static void denet_tx_timeout(struct net_device *dev)
#else
static void denet_tx_timeout(struct net_device *dev, unsigned int txqueue)
#endif
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    unsigned long flags;

    printk(KERN_ERR "%s: Exceeded transmit timeout of %lu ms.\n", dev->name, TX_TIMEOUT
            * 1000UL / HZ);

    spin_lock_irqsave(&net_common->reset_lock, flags);
    tx_hw_reset(dev, HALF_DUPLEX);
    spin_unlock_irqrestore(&net_common->reset_lock, flags);
}

int denet_ethtool_get_link_ksettings(struct net_device *dev,
				   struct ethtool_link_ksettings *cmd)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    dante_network_st_t *net_st = &net_common->dante_net_st;
    u16 mii_status;
    u16 mii_advControl;
    u16 mii_advControl1000;
    int s;
    u32 supported = 0, advertising = 0;

    memset (cmd, 0, sizeof(struct ethtool_link_ksettings));

    if (net_st->net_chip_info.adapter_type == ADAPTER_SWITCH)
    {
        cmd->base.speed = net_st->interface_stat[net_local->index].link_speed;
        cmd->base.duplex = net_st->interface_stat[net_local->index].link_duplex;
    }
    else if (net_st->net_chip_info.adapter_type == ADAPTER_PHY)
    {
        int speed_temp, duplex_temp;

        s = sl_read_smi(net_st->net_chip_info.phy_address[net_local->index], MII_BMSR, &mii_status);
        if (s != 0)
        {
            printk(KERN_ERR
                    "%s: Could not read mii status register; error %d\n",
                    dev->name, s);
            return -1;
        }

        s = sl_read_smi(net_st->net_chip_info.phy_address[net_local->index], MII_ADVERTISE, &mii_advControl);
        if (s != 0)
        {
            printk(KERN_ERR
                    "%s: Could not read mii advertisement control register; error %d\n",
                    dev->name, s);
            return -1;
        }

        s = sl_read_smi(net_st->net_chip_info.phy_address[net_local->index], MII_CTRL1000, &mii_advControl1000);
        if (s != 0)
        {
            printk(KERN_ERR
                    "%s: Could not read mii 1000 advertisement control register; error %d\n",
                    dev->name, s);
            return -1;
        }

        speed_temp = duplex_temp = 0;

        // getting phy speed & duplex information using temp valuable.
        s = sl_get_phy_speed_duplex(((net_local->index == 0) ? PHY_PRIMARY_PORT : PHY_SECONDARY_PORT), &speed_temp, &duplex_temp);
        if (s != SWITCH_SUCCESS)
        {
            printk(KERN_ERR
                    "%s: Could not read PHY Aux control and status register; error %d\n",
                    dev->name, s);
            return -1;
        }

        cmd->base.speed = speed_temp;
        cmd->base.duplex = duplex_temp;

        /* Dont support Half duplex even tho PHY is capable */
        if (mii_status & BMSR_100FULL)
            supported |= SUPPORTED_100baseT_Full;
        if (mii_status & BMSR_10FULL)
            supported |= SUPPORTED_10baseT_Full;
        if (mii_status & MII_ESTATUS)
            supported |= SUPPORTED_1000baseT_Full;

        if (mii_status & BMSR_ANEGCAPABLE)
            supported |= SUPPORTED_Autoneg;
        cmd->base.autoneg = AUTONEG_ENABLE;
        supported |= ADVERTISED_Autoneg;

        if (mii_advControl & ADVERTISE_10FULL)
            advertising |= ADVERTISED_10baseT_Full;
        if (mii_advControl & ADVERTISE_10HALF)
            advertising |= ADVERTISED_10baseT_Half;
        if (mii_advControl & ADVERTISE_100FULL)
            advertising |= ADVERTISED_100baseT_Full;
        if (mii_advControl & ADVERTISE_100HALF)
            advertising |= ADVERTISED_100baseT_Half;
        if (mii_advControl1000 & ADVERTISE_1000FULL)
            advertising |= ADVERTISED_1000baseT_Full;

        advertising |= ADVERTISED_MII;
        cmd->base.port = PORT_MII;
        cmd->base.transceiver = XCVR_INTERNAL;

        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
                                                supported);
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
                                                advertising);
    }

    return 0;
}

int
akashi_ethtool_get_settings (struct net_device *dev, struct ethtool_cmd* ecmd)
{
    int ret;
    struct ethtool_link_ksettings elk = {0};
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    dante_network_st_t *net_st = &net_common->dante_net_st;

    memset (ecmd, 0, sizeof(struct ethtool_cmd));
    ret = denet_ethtool_get_link_ksettings(dev, &elk);
    ecmd->speed = elk.base.speed;
    ecmd->duplex = elk.base.duplex;

    if (net_st->net_chip_info.adapter_type == ADAPTER_PHY)
    {
        ecmd->autoneg = AUTONEG_ENABLE;
        ecmd->port = PORT_MII;
        ecmd->transceiver = XCVR_INTERNAL;
        ethtool_convert_link_mode_to_legacy_u32(&ecmd->supported,
                                                elk.link_modes.supported);
        ethtool_convert_link_mode_to_legacy_u32(&ecmd->advertising,
                                                elk.link_modes.advertising);
    }

    return ret;
}

static int denet_ethtool_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo* ed)
{
    memset(ed, 0, sizeof(struct ethtool_drvinfo));
    strcpy(ed->driver, DRIVER_NAME);
    strcpy(ed->version, DRIVER_VERSION);
    strcpy(ed->fw_version, FIRMWARE_VERSION);
    strcpy(ed->bus_info, BUS_INFO);
    return 0;
}

struct mac_regsDump
{
    struct ethtool_regs hd;
    u16 data[EMAC_REGS_N];
};

static void denet_ethtool_get_drvinfo_410(struct net_device *dev, struct ethtool_drvinfo* ed)
{
    denet_ethtool_get_drvinfo(dev, ed);
}

static int denet_ethtool_get_regs_len(struct net_device *dev)
{
    return EMAC_REGS_N * sizeof(u16);
}

static void denet_ethtool_get_regs (struct net_device *dev, struct ethtool_regs* regs, void* regbuf)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    dante_network_st_t *net_st = &net_common->dante_net_st;
    u16 * data = regbuf;
    int i,r = 0;

    printk(KERN_INFO "Enter: %s\n",__func__);

    if (regs)
    {
        regs->version = 0;
        regs->len = denet_ethtool_get_regs_len(dev);
    }

    if (data)
    {
        for (i = 0; i < EMAC_REGS_N; i++)
        {
            r = sl_read_smi(net_st->net_chip_info.phy_address[net_local->index], i, &(data[i]));
            if (r != 0)
            {
                printk (KERN_INFO "PhyRead ERROR %d\n", r);
                return;
            }
        }
    }

    return;
}

static int denet_ethtool_nway_reset(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    dante_network_st_t *net_st = &net_common->dante_net_st;
    int result;
    u16 mii_reg_autoneg;
    unsigned long flags;

    printk(KERN_INFO "Enter: %s\n",__func__);

    mii_reg_autoneg = 0;
    mii_reg_autoneg |= (BMCR_ANENABLE | BMCR_ANRESTART);
    spin_lock_irqsave(&net_common->reset_lock, flags);

    result = sl_write_smi(net_st->net_chip_info.phy_address[net_local->index], MII_BMCR, mii_reg_autoneg);

    spin_unlock_irqrestore(&net_common->reset_lock, flags);

    return 0;
}

static const struct ethtool_ops primary_ethtool_ops =
{
    .get_link_ksettings = denet_ethtool_get_link_ksettings, //ETHTOOL_GLINKSETTINGS
    .get_drvinfo    = denet_ethtool_get_drvinfo_410,        //ETHTOOL_GDRVINFO
    .get_regs       = denet_ethtool_get_regs,               //ETHTOOL_GREGS
    .get_regs_len   = denet_ethtool_get_regs_len,           //ETHTOOL_GREGS
    .nway_reset     = denet_ethtool_nway_reset,             //ETHTOOL_NWAY_RST
};

static const struct ethtool_ops secondary_ethtool_ops =
{
    .get_link_ksettings = denet_ethtool_get_link_ksettings, //ETHTOOL_GLINKSETTINGS
};

static int denet_do_ethtool_ioctl (struct net_device *dev, struct ifreq *rq)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    dante_network_st_t *net_st = &net_common->dante_net_st;
    struct ethtool_cmd ecmd;
    struct ethtool_pauseparam epp;
    struct ethtool_drvinfo edrv;
    struct mac_regsDump regs;
    int ret = -EOPNOTSUPP;
    int result;
    u16 mii_reg_autoneg;
    unsigned long flags;

    if (copy_from_user(&ecmd, rq->ifr_data, sizeof (ecmd.cmd)))
        return -EFAULT;

    switch (ecmd.cmd)
    {
        case ETHTOOL_GSET:
            ret = akashi_ethtool_get_settings(dev, &ecmd);
            if (ret >= 0)
            {
                if (copy_to_user(rq->ifr_data, &ecmd, sizeof (ecmd)))
                    ret = -EFAULT;
            }
            break;

        case ETHTOOL_GDRVINFO:
            edrv.cmd = edrv.cmd;
            ret = denet_ethtool_get_drvinfo(dev, &edrv);
            if (ret >= 0)
            {
                if (copy_to_user(rq->ifr_data, &edrv, sizeof (struct ethtool_drvinfo)))
                    ret = -EFAULT;
            }
            break;

        case ETHTOOL_GREGS:
            regs.hd.cmd = edrv.cmd;

            denet_ethtool_get_regs (dev, &(regs.hd), &ret);
            if (ret >= 0)
            {
                if (copy_to_user(rq->ifr_data, &regs, sizeof (struct mac_regsDump)))
                    ret = -EFAULT;
            }
            break;

        case ETHTOOL_NWAY_RST:
            epp.cmd = ecmd.cmd;
            mii_reg_autoneg = 0;
            mii_reg_autoneg |= (BMCR_ANENABLE | BMCR_ANRESTART);
            spin_lock_irqsave(&net_common->reset_lock, flags);

            result = sl_write_smi(net_st->net_chip_info.phy_address[net_local->index], MII_BMCR, mii_reg_autoneg);

            spin_unlock_irqrestore(&net_common->reset_lock, flags);
            if (result != 0)
            {
                ret = -EIO;
                break;
            }
            ret = 0;
            break;

        default:
            break;
    }
    return ret;
}

static int denet_private_ioctl(struct net_device *dev, struct ifreq *rq, void __user *data, int cmd)
{
	net_local_t *net_local = netdev_get_priv(dev);
	net_common_t *net_common = net_local->common;
	dante_network_st_t *net_st = &net_common->dante_net_st;
	network_adapter_t *chip_info = &net_st->net_chip_info;
	/* mii_ioctl_data has 4 u16 fields: phy_id, reg_num, val_in & val_out */
	struct mii_ioctl_data mii_data;
	unsigned long flags;
	int Result;

	switch (cmd)
	{
		case SIOCDEVPRIVATE:    /* for binary compat, remove in 2.5 */
			if (copy_from_user(&mii_data, rq->ifr_data, sizeof(struct mii_ioctl_data)))
				return -EFAULT;

			mii_data.phy_id = chip_info->phy_address[net_local->index];
			/* Fall Through */

		case SIOCDEVPRIVATE + 1:    /* for binary compat, remove in 2.5 */
		{
			if (copy_from_user(&mii_data, rq->ifr_data, sizeof(struct mii_ioctl_data)))
				return -EFAULT;

			if ((mii_data.phy_id > 31 || mii_data.reg_num > 31) && (!mdio_phy_id_is_c45(mii_data.phy_id)))
				return -ENXIO;

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
			/* Stop the PHY timer to prevent reentrancy. */
			del_timer_sync(&net_local->phy_timer);
#endif
			spin_lock_irqsave(&net_common->reset_lock, flags);
			if (mdio_phy_id_is_c45(mii_data.phy_id))
			{
				Result = sl_read_reg_clause45(mdio_phy_id_prtad(mii_data.phy_id), mdio_phy_id_devad(mii_data.phy_id),
					mii_data.reg_num, &mii_data.val_out);
			}
			else
			{
				Result = sl_read_smi(mii_data.phy_id, mii_data.reg_num, &mii_data.val_out);
			}
			/* Start the PHY timer up again. */
			spin_unlock_irqrestore(&net_common->reset_lock, flags);
#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
			/* Start the PHY timer up again. */
			net_local->phy_timer.expires = jiffies + 2 * TIMER_DELAY_1SEC;
			add_timer(&net_local->phy_timer);
#endif
			if (Result != 0)
			{
				printk(KERN_ERR
					"%s: Could not read from PHY, error=%d.\n",
				dev->name, Result);
				return (Result == 1) ? -EBUSY : -EIO;
			}
			return copy_to_user(rq->ifr_data, &mii_data, sizeof(struct mii_ioctl_data)) ? -EFAULT : 0;
		}

		case SIOCDEVPRIVATE + 2:    /* for binary compat, remove in 2.5 */
		{
			if (copy_from_user(&mii_data, rq->ifr_data, sizeof(struct mii_ioctl_data)))
				return -EFAULT;

			if (!capable(CAP_NET_ADMIN))
				return -EPERM;
			if ((mii_data.phy_id > 31 || mii_data.reg_num > 31) && (!mdio_phy_id_is_c45(mii_data.phy_id)))
				return -ENXIO;
#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
			/* Stop the PHY timer to prevent reentrancy. */
			del_timer_sync(&net_local->phy_timer);
#endif
			spin_lock_irqsave(&net_common->reset_lock, flags);
			if (mdio_phy_id_is_c45(mii_data.phy_id))
			{
				Result = sl_write_reg_clause45(mdio_phy_id_prtad(mii_data.phy_id), mdio_phy_id_devad(mii_data.phy_id),
					mii_data.reg_num, mii_data.val_in);
			}
			else
			{
				Result = sl_write_smi(mii_data.phy_id, mii_data.reg_num, mii_data.val_in);
			}
			spin_unlock_irqrestore(&net_common->reset_lock, flags);
#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
			/* Start the PHY timer up again. */
			net_local->phy_timer.expires = jiffies + 2 * TIMER_DELAY_1SEC;
			add_timer(&net_local->phy_timer);
#endif
			if (Result != 0)
			{
				printk(KERN_ERR	"%s: Could not write to PHY, error=%d.\n", dev->name, Result);
				return (Result == 1) ? -EBUSY : -EIO;
			}
		}
		return 0;

		case SIOCDEVLINKUTIL:
		{
			struct linkutil util;

			util.tx_bytes_sec = AUD_SYD_MAC_TX0_BYTES;
			util.rx_bytes_sec = AUD_SYD_MAC_RX0_GOOD_BYTES;
			if (copy_to_user(rq->ifr_data, &util, sizeof (struct linkutil)))
			return  -EFAULT;
		}
		return 0;

		case SIOCDEVPHYID:
		{
			if (copy_to_user(rq->ifr_data, &chip_info->product_identifier, sizeof (uint16_t)))
				return -EFAULT;
		}
		return 0;

		case SIOCDEVREDUNDANT:
		{
			if (copy_to_user(rq->ifr_data, &chip_info->phy_redundant, sizeof (uint8_t)))
				return -EFAULT;
		}
		return 0;

		case SIOCDEVERRORCLEAR:
			clear_error_counter(dev);
			return 0;

		case SIOCDEVEXTPHYINTCLEARREG:
		{
			ext_phy_int_reg_t ext_phy_info;

			if (copy_from_user(&ext_phy_info, rq->ifr_data, sizeof(ext_phy_int_reg_t)))
				return -EFAULT;

			// Dont go there
			if (chip_info->adapter_type != ADAPTER_SWITCH)
				return -EPERM;

			if (ext_phy_info.phy_port_index > 6 || ext_phy_info.phy_port_index == 5)
				return -EFAULT;

			// Set external phy interrupt clear register
			net_st->ext_phy_int_clear_reg[ext_phy_info.phy_port_index] = ext_phy_info.phy_int_status_reg;
		}
		return 0;

		case SIOCDEVPORTSTATISTICERRORCOUNTER:
		{
			int port_num;
			uint32_t sw_port_counter;

			// Dont go there
			if (chip_info->adapter_type != ADAPTER_SWITCH)
				return -EPERM;

			if (copy_from_user(&port_num, rq->ifr_data, sizeof (uint32_t)))
				return -EFAULT;

			// get port statistic error counter value
			sl_get_port_statistic_error_counter(port_num, &sw_port_counter);

			if (copy_to_user(rq->ifr_data, &sw_port_counter, sizeof (uint32_t)))
				return -EFAULT;
		}
		return 0;

		case SIOCDEVICEIDENTIFY:
		{
			int identify_device;

			// Dont go there
			if (chip_info->adapter_type != ADAPTER_SWITCH)
				return -EPERM;

			if (copy_from_user(&identify_device, rq->ifr_data, sizeof (int)))
				return -EFAULT;

			if (identify_device == 1)
			{
				// Delete timer now
				del_timer_sync(&net_common->identify_timer);
				net_common->identify_led_activity = IDENTIFY_LED_FORCE_OFF;
				// Start identifying led on switch.
				toggle_identify_led(net_common);
				net_common->identify_timer.expires = jiffies + HZ;
				add_timer(&net_common->identify_timer);
			}
			else
			{
				// Stop identifying led off switch.
				// Stop timer & get back to default status.
				del_timer_sync(&net_common->identify_timer);
				net_common->identify_led_activity = IDENTIFY_LED_DEFAULT;
				toggle_identify_led(net_common);
			}
		}
		return 0;

		case SIOCDEVIFLINKSTATUS:
		{
			uint8_t link_status = net_local->link_status;

			if (copy_to_user(rq->ifr_data, &link_status, sizeof(uint8_t)))
			{
				return -EFAULT;
			}
		}
		return 0;

		default:
			return -EOPNOTSUPP;
	}
}

static int denet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	net_local_t *net_local = netdev_get_priv(dev);
	net_common_t *net_common = net_local->common;
	/* mii_ioctl_data has 4 u16 fields: phy_id, reg_num, val_in & val_out */
	struct mii_ioctl_data mii_data;
	unsigned long flags;
	int Result;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
	// Private dev IO control
	if (cmd >= SIOCDEVPRIVATE && cmd <= SIOCDEVPRIVATE + 15)
		return denet_private_ioctl(dev, rq, NULL, cmd);
#endif

	switch (cmd)
	{
		case SIOCETHTOOL:
			return denet_do_ethtool_ioctl(dev, rq);

		case SIOCSMIIREG:    /* Write MII Switch/PHY register. */
		{
			if (copy_from_user(&mii_data, rq->ifr_data, sizeof(struct mii_ioctl_data)))
				return -EFAULT;

			if ((mii_data.phy_id > 31 || mii_data.reg_num > 31) && (!mdio_phy_id_is_c45(mii_data.phy_id)))
				return -ENXIO;

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
			/* Stop the PHY timer to prevent reentrancy. */
			del_timer_sync(&net_local->phy_timer);
#endif
			spin_lock_irqsave(&net_common->reset_lock, flags);
			if (mdio_phy_id_is_c45(mii_data.phy_id))
			{
				Result = sl_write_reg_clause45(mdio_phy_id_prtad(mii_data.phy_id), mdio_phy_id_devad(mii_data.phy_id),
					mii_data.reg_num, mii_data.val_in);
			}
			else
			{
				Result = sl_write_smi(mii_data.phy_id, mii_data.reg_num, mii_data.val_in);
			}
			spin_unlock_irqrestore(&net_common->reset_lock, flags);
#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
			/* Start the PHY timer up again. */
			net_local->phy_timer.expires = jiffies + 2 * TIMER_DELAY_1SEC;
			add_timer(&net_local->phy_timer);
#endif
			if (Result != 0)
			{
				printk(KERN_ERR "%s: Could not write from PHY, error=%d.\n", dev->name, Result);
				return (Result == 1) ? -EBUSY : -EIO;
			}
		}
		return 0;

		case SIOCGMIIREG:    /* Read MII Switch/PHY register. */
		{
			if (copy_from_user(&mii_data, rq->ifr_data, sizeof(struct mii_ioctl_data)))
				return -EFAULT;

			if ((mii_data.phy_id > 31 || mii_data.reg_num > 31) && (!mdio_phy_id_is_c45(mii_data.phy_id)))
				return -ENXIO;

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
			/* Stop the PHY timer to prevent reentrancy. */
			del_timer_sync(&net_local->phy_timer);
#endif
			spin_lock_irqsave(&net_common->reset_lock, flags);
			if (mdio_phy_id_is_c45(mii_data.phy_id))
			{
				Result = sl_read_reg_clause45(mdio_phy_id_prtad(mii_data.phy_id), mdio_phy_id_devad(mii_data.phy_id),
					mii_data.reg_num, &mii_data.val_out);
			}
			else
			{
				Result = sl_read_smi(mii_data.phy_id, mii_data.reg_num, &mii_data.val_out);
			}
			spin_unlock_irqrestore(&net_common->reset_lock, flags);
#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
			/* Start the PHY timer up again. */
			net_local->phy_timer.expires = jiffies + 2 * TIMER_DELAY_1SEC;
			add_timer(&net_local->phy_timer);
#endif
			if (Result != 0)
			{
				printk(KERN_ERR "%s: Could not read from PHY, error=%d.\n", dev->name, Result);
				return (Result == 1) ? -EBUSY : -EIO;
			}
			return copy_to_user(rq->ifr_data, &mii_data, sizeof(struct mii_ioctl_data)) ? -EFAULT : 0;
		}

		case SIOCDEVIFLINKSTATUS:
		{
			net_local_t *net_local = netdev_get_priv(dev);
			uint8_t link_status = net_local->link_status;

			if (copy_to_user(rq->ifr_data, &link_status, sizeof(uint8_t)))
			{
				return -EFAULT;
			}
		}
		return 0;

		default:
			return -EOPNOTSUPP;
	}
}

//-------------------------------------------------------------------
// calculate_multicast_hash
// inputs: struct eth_addr *multi_addr: multicast address used to
//                                      compute hash address
// output: u32_t: 8-bit address into 256-bit hash table.
static u32 calculate_multicast_hash(u8 eth_addr[])
{
    u8 i, j;
    u32 crc = 0xFFFFFFFF;
    u32 carry;
    u8 eth_addr_buf[6];

    /* Put the ethernet address into a local array */
    memcpy(eth_addr_buf, eth_addr, 6);

    /* Cycle through each character of the address */
    for (i = 0; i < 6; ++i)
    {
        /* Cycle through each bit of this character */
        for (j = 0; j < 8; ++j)
        {
            /* Update the CRC for this address */
            carry = ((crc & 0x80000000) ? 0x01 : 0x00) ^ (eth_addr_buf[i] & 0x01);
            crc <<= 1;
            eth_addr_buf[i] >>= 1;
            if (carry)
            {
                crc = ((crc ^ CRC_POLYNOMIAL_BE) | carry);
            }
        }
    }

    /* Return the upper 8 bits of the CRC. */
    crc >>= 24;
    return crc;
}



static void akashi_init_misc(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    int is_primary = (dev == net_common->primary_dev);
    int i;

    if (is_primary)
    {
        AUD_SYD_PROTO_PHY0_REJECT(0) = 6000;
        AUD_SYD_PROTO_PHY0_REJECT(1) = 6040;
        /* Zero rest of broadcast registers. */
        for (i = 2; i < 8; i++)
        {
            AUD_SYD_PROTO_PHY0_REJECT(i) = 0;
        }

        // Block any unicast audio flooding from reaching the CPU
        AUD_SYD_PROTO_PHY0_AUDIO_RANGE_HI = AUD_SYD_PROTO_AUDIO_PORT_HI;
        AUD_SYD_PROTO_PHY0_AUDIO_RANGE_LO = AUD_SYD_PROTO_AUDIO_PORT_LO;
        AUD_SYD_PROTO_PHY0_KA_RANGE_HI = AUD_SYD_PROTO_KA_PORT_HI;
        AUD_SYD_PROTO_PHY0_KA_RANGE_LO = AUD_SYD_PROTO_KA_PORT_LO;
    }
    else
    {
        AUD_SYD_PROTO_PHY1_REJECT(0) = 6000;
        AUD_SYD_PROTO_PHY1_REJECT(1) = 6040;
        /* Zero rest of broadcast registers. */
        for (i = 2; i < 8; i++)
        {
            AUD_SYD_PROTO_PHY1_REJECT(i) = 0;
        }

        // Block any unicast audio flooding from reaching the CPU
        AUD_SYD_PROTO_PHY1_AUDIO_RANGE_HI = AUD_SYD_PROTO_AUDIO_PORT_HI;
        AUD_SYD_PROTO_PHY1_AUDIO_RANGE_LO = AUD_SYD_PROTO_AUDIO_PORT_LO;
        AUD_SYD_PROTO_PHY1_KA_RANGE_HI = AUD_SYD_PROTO_KA_PORT_HI;
        AUD_SYD_PROTO_PHY1_KA_RANGE_LO = AUD_SYD_PROTO_KA_PORT_LO;
    }
}

//-------------------------------------------------------------------
// Set multicast list.
static void akashi_set_multicast_list(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);
    net_common_t *net_common = net_local->common;
    u32 reg_idx;
    int i;
    u32 m_hash[AUD_SYD_PROTO_HASH_FILTERS];
    struct netdev_hw_addr *mc_ptr;
    struct netdev_hw_addr_list *list = &dev->mc;
    int mc_count;
    int is_primary = (dev == net_common->primary_dev);

    if (dev->flags & IFF_PROMISC)
    {
        for (i = 0; i < AUD_SYD_PROTO_HASH_FILTERS; i++)
        {
            if (is_primary)
            {
                AUD_SYD_PROTO_HASH0(i) = 0xffff;
            }
            else
            {
                AUD_SYD_PROTO_HASH1(i) = 0xffff;
            }
        }
        return;
    }
    mc_count = dev->mc.count;

    // If there's more addresses than we handle, get all multicast
    // packets and sort them out in software.
    if ((dev->flags & IFF_ALLMULTI) || (mc_count >= AUD_SYD_PROTO_HASH_BITS))
    {
        for (i = 0; i < AUD_SYD_PROTO_HASH_FILTERS; i++)
        {
            if (is_primary)
            {
                AUD_SYD_PROTO_HASH0(i) = 0xffff;
            }
            else
            {
                AUD_SYD_PROTO_HASH1(i) = 0xffff;
            }
        }
        return;
    }

    // No multicast?  Just get our own stuff
    if (mc_count == 0)
    {
        // Get own packets.
        return;
    }

    memset(&m_hash, 0, sizeof(uint32_t) * AUD_SYD_PROTO_HASH_FILTERS);

    list_for_each_entry(mc_ptr, &list->list, list)
    {
        u32 hash, bit_val;
        // We join our audio mcast groups so IGMP management works
        // however we don't want to add these to the hash filter to be
        // received by the microblaze. Ignore them here to avoid hackery further
        // up in the protocol stack
        if (mc_ptr->addr[3] == AUD_SYD_AUDIO_MCAST_IDENT)
        {
            continue;
        }
        hash = calculate_multicast_hash(mc_ptr->addr);

        reg_idx = (hash & 0xE0) >> 5;
        /*
         * DANTEIP-506:
         *      Ensure bit shift operation is less than 32bits.
         *      Use this example as sanity test.
         *      bit_val = (1 << 0xf9)
         *              = (1 << 249)
         *              = (1 << (249 % 32))
         *              = (1 << 25)
         *              = 0x02000000
         */
        hash &= 0xFF; /* Mask 8bits only */
        hash %= 0x20; /* Mod 32 */
        bit_val = 0x1 << hash;
        m_hash[reg_idx * 2] |= (bit_val & 0xFFFF);
        m_hash[reg_idx * 2 + 1] |= (bit_val >> 16);
    }

    // Write out hash registers
    for (i = 0; i < AUD_SYD_PROTO_HASH_FILTERS; i++)
    {
        if (m_hash[i] != net_local->mcast_hash[i])
        {
            if (is_primary)
            {
                AUD_SYD_PROTO_HASH0(i) = m_hash[i];
            }
            else
            {
                AUD_SYD_PROTO_HASH1(i) = m_hash[i];
            }
            net_local->mcast_hash[i] = m_hash[i];
        }
    }
}

//-------------------------------------------------------------------
// Read Tx timestamp
static ssize_t denet_ts_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    net_common_t *net_common = (net_common_t *)filp->private_data;
    timestamp_info_t * ptime_info;
    int i, ready_to_send = 0;
    char * pbuf = net_common->buf_to_send;
    int evt;
    txts_queue_t *q = &net_common->txts_queue;

    evt = wait_event_interruptible(net_common->tsqueue, akashi_txts_queue_get_count(net_common));
    if (evt)
    {
        return evt;
    }

    memset(net_common->buf_to_send, 0, TXTS_QUEUE_SIZE * sizeof(timestamp_info_t));

    for (i = 0; i < TXTS_QUEUE_SIZE; i++)
    {
        if (q->txts_status[q->tail] != TXTS_STATUS_TX_INT)
        {
            q->tail++;
            if (q->tail >= TXTS_QUEUE_SIZE)
            {
                q->tail = 0;
            }
            continue;
        }

        ptime_info = &q->txts[q->tail];
        memcpy(pbuf, (char *)ptime_info, sizeof(timestamp_info_t));
        pbuf += sizeof(timestamp_info_t);

        q->txts_status[q->tail] = TXTS_STATUS_READY;
        q->tail++;
        if (q->tail >= TXTS_QUEUE_SIZE)
        {
            q->tail = 0;
        }
        ready_to_send++;
    }

    if (copy_to_user(buf, net_common->buf_to_send, ready_to_send * sizeof(timestamp_info_t)))
    {
        printk(KERN_ERR "%s: error on copy_to_user\n", __func__);
    }

    return 1;
}


//-------------------------------------------------------------------
// Poll for Tx timestamp
static unsigned int denet_ts_poll(struct file *filp, poll_table *wait)
{
    net_common_t *net_common = (net_common_t *)filp->private_data;
    unsigned int mask = 0;

    if (akashi_txts_queue_get_count(net_common) == 0)
        poll_wait(filp, &net_common->tsqueue,  wait);

    if (akashi_txts_queue_get_count(net_common) > 0)
        mask = POLLIN | POLLRDNORM;    /* readable */

    return mask;
}

//-------------------------------------------------------------------
// Read port status
static ssize_t denet_ps_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    net_common_t *net_common = (net_common_t *)filp->private_data;
    dante_network_st_t *adapter = (dante_network_st_t *)&net_common->dante_net_st;
    int evt;
    evt = wait_event_interruptible(net_common->psqueue, net_common->ps_pending);
    if(evt)
        return evt;

    if (copy_to_user(buf, &adapter->ports, sizeof(portstat_t)))
    {
        printk(KERN_ERR "%s: error on copy_to_user\n", __func__);
    }

    net_common->ps_pending = 0;
    if (net_common->ps_pending1 == 0)
    {
        adapter->ports.changed = 0;
    }

    return 1;
}

static ssize_t denet_ps1_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    net_common_t *net_common = (net_common_t *)filp->private_data;
    dante_network_st_t *adapter = (dante_network_st_t *)&net_common->dante_net_st;
    int evt;

    evt = wait_event_interruptible(net_common->psqueue1, net_common->ps_pending1);
    if(evt)
        return evt;

    if (copy_to_user(buf, &adapter->ports, sizeof(portstat_t)))
    {
        printk(KERN_ERR "%s: error on copy_to_user\n", __func__);
    }

    net_common->ps_pending1 = 0;
    if (net_common->ps_pending == 0)
    {
        adapter->ports.changed = 0;
    }

    return 1;
}

//-------------------------------------------------------------------
// Poll port status
static unsigned int denet_ps_poll(struct file *filp, poll_table *wait)
{
    net_common_t *net_common = (net_common_t *)filp->private_data;
    unsigned int mask = 0;

    if (!net_common->ps_pending)
        poll_wait(filp, &net_common->psqueue, wait);

    if (net_common->ps_pending)
        mask |= POLLIN | POLLRDNORM;    /* readable */

    return mask;
}

static unsigned int denet_ps1_poll(struct file *filp, poll_table *wait)
{
    net_common_t *net_common = (net_common_t *)filp->private_data;
    unsigned int mask = 0;

    if (!net_common->ps_pending1)
        poll_wait(filp, &net_common->psqueue1, wait);

    if (net_common->ps_pending1)
        mask |= POLLIN | POLLRDNORM;    /* readable */

    return mask;
}

//-------------------------------------------------------------------
// Open denet char device
static int denet_reg_open (struct inode *inode, struct file *filp)
{
    filp->private_data = net_common_context;

    return 0;
}

static unsigned denet_reg_poll(struct file *filp, poll_table *wait)
{
    dev_t i_rdev = filp->f_inode->i_rdev;

    if (MINOR(i_rdev) == DENET_TIMESTAMP_MINOR)
    {
        return denet_ts_poll(filp, wait);
    }
    else if (MINOR(i_rdev) == DENET_PORTSTAT_MINOR)
    {
        return denet_ps_poll(filp, wait);
    }
    else if (MINOR(i_rdev) == DENET_PORTSTAT1_MINOR)
    {
        return denet_ps1_poll(filp, wait);
    }

    return 0;
}

static ssize_t denet_reg_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    dev_t i_rdev = filp->f_inode->i_rdev;

    if (MINOR(i_rdev) == DENET_TIMESTAMP_MINOR)
    {
        return denet_ts_read(filp, buf, count, f_pos);
    }
    else if (MINOR(i_rdev) == DENET_PORTSTAT_MINOR)
    {
        return denet_ps_read(filp, buf, count, f_pos);
    }
    else if (MINOR(i_rdev) == DENET_PORTSTAT1_MINOR)
    {
        return denet_ps1_read(filp, buf, count, f_pos);
    }
    else
    {
        return -ENODEV;
    }
}

static struct file_operations denet_fops =
{
    .owner = THIS_MODULE,
    .open  = denet_reg_open,
    .read  = denet_reg_read,
    .poll  = denet_reg_poll
};

/*
 * Printing deteced adapter
 */
void print_detected_adapter(adapter_enum_t adapter_name, adapter_enum_t default_adapter_name)
{
    char adapter_name_buf[50];

    memset(adapter_name_buf, 0, sizeof(adapter_name_buf));

    if (adapter_name == NETWORK_CHIPSET_NONE)
    {
        if(sl_get_adapter_name_string(default_adapter_name, adapter_name_buf))
        {
            printk("No network device was found and defaulting to : \'%s\' registers loading\n", adapter_name_buf);
        }
    }
    else
    {
        if (sl_get_adapter_name_string(adapter_name, adapter_name_buf))
        {
            printk("Detecting network adapter : Found %s\n", adapter_name_buf);
        }
    }
}

// load network info from switch library
void load_network_info(int index, net_local_t *net_local)
{
    net_common_t *net_common = net_local->common;
    dante_network_st_t *net_st = &net_common->dante_net_st;

    /* load adapter structure info */
    memcpy(&net_st->net_chip_info, sl_get_chipset_info(), sizeof(network_adapter_t));

    /* load detected number of switch ports. */
    net_st->ports.num_ports = net_st->net_chip_info.switch_ports_info.num_switch_ports;
    // Asigning phy address for switches
    net_local->mii_addr = net_st->net_chip_info.phy_address[0];
}

// set cpu port number
void akashi_configure_cpu_port(net_common_t *net_common)
{
    uint8_t cpu_port;

    dante_network_st_t *net_st = &net_common->dante_net_st;
    adapter_enum_t adapter_name = net_st->net_chip_info.adapter_name;

#ifdef CONFIG_AKASHI_CTL_PORT_6
    cpu_port = SWITCH_MB_CTL_PORT6;
#else
    if (adapter_name == SWITCH_88E6361)
    {
#ifdef CONFIG_AKASHI_CONFIGURE_SWITCH
        // IPCore uses port9 as the cpu port
        cpu_port = SWITCH_88E6361_CTL_PORT0;
#else
        // BK3 uses port0 as the cpu port
        cpu_port = SWITCH_88E6361_CTL_PORT0;
#endif
    }
    else
    {
        cpu_port = SWITCH_MB_CTL_PORT5;
    }
#endif

    sl_set_cpu_port(cpu_port);
}

static void enter_hyperport_mode(void)
{
    int port_idx;
    uint32_t vlan_config = 0;
    uint16_t global_reg_2;

    pr_info("enter_hyperport_mode\n");
    if (!digico_hyperport_primary_ports || !digico_hyperport_secondary_ports)
        pr_warn("One of primary or secondary are not mapped to any ports for hyperport\n");

    // build VLAN mapping. Format is one nibble per port. Port 0 is in the least
    // significant nibble The bits of the nibble assign the port to:
    // ETH_SWITCH_VLAN_PRI = (1 << 0)
    // ETH_SWITCH_VLAN_SEC = (1 << 1)
    // ETH_SWITCH_VLAN_2   = (1 << 2)
    // ETH_SWITCH_VLAN_3   = (1 << 3)
    // VLAN_2 and VLAN_3 enable 802.1Q external VLAN tagging which we don't want
    for (port_idx = 0; port_idx < sl_get_max_switch_port_number(); port_idx++) {
        if (digico_hyperport_primary_ports & (1 << port_idx)) {
            // this port is assigned to primary
            vlan_config |= ETH_SWITCH_VLAN_PRI << (4 * port_idx);
        }
        if (digico_hyperport_secondary_ports & (1 << port_idx)) {
            // this port is assigned to primary
            vlan_config |= ETH_SWITCH_VLAN_SEC << (4 * port_idx);
        }
    }
    // Port 5 (RGMII 0) is always primary VLAN otherwise no packets reach PL
    vlan_config |= ETH_SWITCH_VLAN_PRI << (4 * 5);
    // Port 6 (RGMII 1) is always secondary VLAN
    vlan_config |= ETH_SWITCH_VLAN_SEC << (4 * 6);

    sl_disable_switch_phy_port_all();
    sl_set_switch_vlan_default();

    sl_read_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL_MANAGEMENT_REG, &global_reg_2);
    sl_write_smi(SWITCH_GLOBAL_REGISTER_GROUP_2, SWITCH_GLOBAL_MANAGEMENT_REG, global_reg_2 | SWITCH_FLOOD_BROADCAST );

    sl_set_switch_vlan_config_all(vlan_config);
    sl_enable_phy_all(vlan_config);
    sl_serdes_ports_up_all(vlan_config);
    sl_enable_switch_all(vlan_config);
}

static void leave_hyperport_mode(void)
{
    pr_info("leave_hyperport_mode\n");
    // restore Dante VLAN and port config
    sl_set_switch_vlan_config_all(net_common_context->config.vlan_config);
    sl_enable_phy_all(net_common_context->config.vlan_config);
    sl_serdes_ports_up_all(net_common_context->config.vlan_config);
    sl_enable_switch_all(net_common_context->config.vlan_config);

    // re-scan switch port status
    network_mii_process(net_common_context);
}

static void set_mdio_locked(bool locked)
{
    pr_info("temac MDIO %s\n", locked ? "locked" : "unlocked");
    if (net_common_context) {
        net_common_context->mdio_locked = locked;
        wmb(); // poor man's atomics :^)
    }
}

static ssize_t digico_mdiolock_store(
    struct class *class,
    struct class_attribute *attr,
    const char *buf,
    size_t len)
{
    unsigned int mdio_lock_enable;

    if (sscanf(buf, "%u", &mdio_lock_enable) != 1)
        return -EINVAL;

    switch(mdio_lock_enable)
    {
        case 0:
            set_mdio_locked(false);
            // re-re-scan switch port status just to be sure??
            network_mii_process(net_common_context);
            break;
        case 1:
            set_mdio_locked(true);
            break;
        default:
            pr_err("Invalid digico_mdiolock value: %u\n", mdio_lock_enable);
            return -EINVAL;
    }
    return len;
}
static CLASS_ATTR_WO(digico_mdiolock);

static ssize_t digico_hypermode_store(
    struct class *class,
    struct class_attribute *attr,
    const char *buf,
    size_t len)
{
    unsigned int hyper_mode_enable;

    if (sscanf(buf, "%u", &hyper_mode_enable) != 1)
        return -EINVAL;

    if (!net_common_context)
        return -ENODEV;

    switch(hyper_mode_enable)
    {
        case 0:
            leave_hyperport_mode();
            break;
        case 1:
            if (!net_common_context->mdio_locked) {
                pr_warn("Enabling hyperport mode without locking MDIO, locking now\n");
                set_mdio_locked(true);
            }
            enter_hyperport_mode();
            break;
        default:
            pr_err("Invalid digico_hypermode value: %u\n", hyper_mode_enable);
            return -EINVAL;
    }
    return len;
}
static CLASS_ATTR_WO(digico_hypermode);

static void print_hyperport_assignments(void)
{
    int port_idx;
    for(port_idx = 0; port_idx < sl_get_max_switch_port_number(); port_idx++) {
        if (digico_hyperport_primary_ports & (1 << port_idx))
            pr_info("Port %d assigned to primary\n", port_idx);
    }
    for(port_idx = 0; port_idx < sl_get_max_switch_port_number(); port_idx++) {
        if (digico_hyperport_secondary_ports & (1 << port_idx))
            pr_info("Port %d assigned to secondary\n", port_idx);
    }
}

static ssize_t digico_hyper_vlans_store(
    struct class *class,
    struct class_attribute *attr,
    const char *buf,
    size_t len)
{
    if (sscanf(buf, "%x:%x", &digico_hyperport_primary_ports, &digico_hyperport_secondary_ports) != 2)
        return -EINVAL;

    if (!net_common_context)
        return -ENODEV;

    print_hyperport_assignments();
    return len;
}
static CLASS_ATTR_WO(digico_hyper_vlans);

static int register_digico_sysfs_attributes(struct class* class)
{
    int ret = 0;

    ret = class_create_file(class, &class_attr_digico_mdiolock);
    if (ret) {
        pr_err("unable to create digico_mdiolock sysfs (%d)\n", ret);
        return ret;
    }

    ret = class_create_file(class, &class_attr_digico_hypermode);
    if (ret) {
        pr_err("unable to create digico_hypermode sysfs (%d)\n", ret);
        goto out_rm_lockout;
    }

    ret = class_create_file(class, &class_attr_digico_hyper_vlans);
    if (ret) {
        pr_err("unable to create digico_hyper_vlans sysfs (%d)\n", ret);
        goto out_rm_hypermode;
    }

    return ret;

out_rm_hypermode:
    class_remove_file(class, &class_attr_digico_hypermode);
out_rm_lockout:
    class_remove_file(class, &class_attr_digico_mdiolock);
    return ret;
}

static void unregister_digico_sysfs_attributes(struct class* class)
{
    class_remove_file(class, &class_attr_digico_mdiolock);
    class_remove_file(class, &class_attr_digico_hypermode);
    class_remove_file(class, &class_attr_digico_hyper_vlans);
}

static void unregister_char_device(struct net_device *dev)
{
    net_local_t *net_local = netdev_get_priv(dev);

#ifdef CONFIG_AKASHI_DEBUG_CHAR_DEV
    printk(KERN_INFO "%s...\n", __func__);
#endif

    unregister_digico_sysfs_attributes(net_local->denet_class);

    device_destroy(net_local->denet_class, net_local->denet_devt);
    device_destroy(net_local->denet_class, net_local->denet_ps_devt);
    device_destroy(net_local->denet_class, net_local->denet_portstat_devt);
    class_destroy(net_local->denet_class);

    unregister_chrdev(net_local->dev_major, DENET_NAME);
}

static int register_char_devices(struct net_device *dev)
{
    int status;
    net_local_t *net_local = netdev_get_priv(dev);

#ifdef CONFIG_AKASHI_DEBUG_CHAR_DEV
    printk(KERN_INFO "%s...\n", __func__);
#endif

    /* create and register a cdev occupying a range of minors */
#ifdef CONFIG_AKASHI_DYNAMIC_MAJOR
    net_local->dev_major = 0; /* Set to zero to let kernel allocate an un-used major number */
    /* For dynamic major, kernel will return the major number being allocated */
    net_local->dev_major = register_chrdev(net_local->dev_major, DENET_NAME, &denet_fops);
    if (net_local->dev_major < 0)
    {
        printk(KERN_ERR "failed to register chrdev %s with dynamic major number \n", DENET_NAME);
        return -EIO;
    }
#else
    net_local->dev_major = DENET_MAJOR; /* Fixed major */
    status = register_chrdev(net_local->dev_major, DENET_NAME, &denet_fops);
    if (status < 0)
    {
        printk(KERN_ERR "failed to register chrdev %s major %d\n", DENET_NAME, net_local->dev_major);
        return -EIO;
    }
#endif

    net_local->denet_class = class_create(THIS_MODULE, "denet");
    if (IS_ERR(net_local->denet_class))
    {
        printk(KERN_ERR "Cannot create class");
        unregister_chrdev(net_local->dev_major, DENET_NAME);
        return PTR_ERR(net_local->denet_class);
    }
#ifdef CONFIG_AKASHI_DEBUG_CHAR_DEV
    printk("%s: Class '%s' created", __func__, net_local->denet_class->name);
#endif

    /* create /dev/denet to 'net_local->dev_major':DENET_TIMESTAMP_MINOR */
    net_local->denet_devt = MKDEV(net_local->dev_major, DENET_TIMESTAMP_MINOR);
    net_local->denet_devfs = device_create(net_local->denet_class, &dev->dev, net_local->denet_devt, net_local, "denet");
    status = PTR_ERR_OR_ZERO(net_local->denet_devfs);
#ifdef CONFIG_AKASHI_DEBUG_CHAR_DEV
    if (status == 0)
    {
        printk("%s: Device '%s' created with minor %d", __func__, \
                dev_name(net_local->denet_devfs),            \
                MINOR(net_local->denet_devt));
    }
#endif

    /* create /dev/denet_ps to net_local->dev_major:DENET_PORTSTAT_MINOR */
    net_local->denet_ps_devt = MKDEV(net_local->dev_major, DENET_PORTSTAT_MINOR);
    net_local->denet_ps_devfs = device_create(net_local->denet_class, &dev->dev, net_local->denet_ps_devt, net_local, "denet_ps");
    status = PTR_ERR_OR_ZERO(net_local->denet_ps_devfs);
#ifdef CONFIG_AKASHI_DEBUG_CHAR_DEV
    if (status == 0)
    {
        //set_bit(DENET_PORTSTAT_MINOR, minors);
        printk("%s: Device '%s' created with minor %d", __func__, \
                dev_name(net_local->denet_ps_devfs),            \
                MINOR(net_local->denet_ps_devt));
    }
#endif

    /* create /dev/denet_portstat to net_local->dev_major:DENET_PORTSTAT1_MINOR */
    net_local->denet_portstat_devt = MKDEV(net_local->dev_major, DENET_PORTSTAT1_MINOR);
    net_local->denet_portstat_devfs = device_create(net_local->denet_class, &dev->dev, net_local->denet_portstat_devt, net_local, "denet_portstat");
    status = PTR_ERR_OR_ZERO(net_local->denet_portstat_devfs);
#ifdef CONFIG_AKASHI_DEBUG_CHAR_DEV
    if (status == 0)
    {
        //set_bit(DENET_PORTSTAT1_MINOR, minors);
        printk("%s: Device '%s' created with minor %d", __func__, \
                dev_name(net_local->denet_portstat_devfs),            \
                MINOR(net_local->denet_portstat_devt));
    }
#endif

    status = register_digico_sysfs_attributes(net_local->denet_class);

    return status;
}

#ifdef CONFIG_AKASHI_CONFIGURE_SWITCH
/* Set up all registers related to redundancy */
static void akashi_switch_redundancy_setup(net_common_t *net_common)
{
    uint32_t vlan_pri_tx_bitmap;
    uint32_t vlan_sec_tx_bitmap;
    int i;
    volatile uint32_t *aud_syd_switch_baseaddr;
    uint8_t cpu_port;

    sl_get_cpu_port(&cpu_port);

    vlan_pri_tx_bitmap = 0;
    vlan_sec_tx_bitmap = 0;

    aud_syd_switch_baseaddr =(volatile uint32_t *)(AUD_SYD_BASEADDR + 0x0820);

    /* Setup per port primary VLAN membership */
    for (i = 0; i < sl_get_max_switch_port_number(); i++)
    {
#ifdef CONFIG_AKASHI_CTL_PORT_6
        /* Port 6 egress is configured by using Marvell Header not VLANs */
        if (((net_common->config.vlan_config >> (i*4)) & ETH_SWITCH_VLAN_PRI) && (i != SWITCH_MB_CTL_PORT6))
#else
        /* Port 5 egress is configured by using Marvell Header not VLANs */
        if (((net_common->config.vlan_config >> (i*4)) & ETH_SWITCH_VLAN_PRI) && (i != cpu_port))
#endif
        {
            // increase 4 bytes from base addr. and apply port as primarry
            *(aud_syd_switch_baseaddr++) = (0x20000 | sl_get_physical_port_id(i));
            vlan_pri_tx_bitmap |= (1<<sl_get_physical_port_id(i));
        }
    }

    /* Setup per port secondary VLAN membership */
    for (i = 0; i < sl_get_max_switch_port_number(); i++)
    {
#ifdef CONFIG_AKASHI_CTL_PORT_6
        /* Port 6 egress is configured by using Marvell Header not VLANs */
        if (((net_common->config.vlan_config >> i*4) & ETH_SWITCH_VLAN_SEC) && (i != SWITCH_MB_CTL_PORT6))
#else
        /* Port 5 egress is configured by using Marvell Header not VLANs */
        if (((net_common->config.vlan_config >> i*4) & ETH_SWITCH_VLAN_SEC) && (i != cpu_port))
#endif
        {
            // increase 4 bytes from base addr. and apply port as secondary
            *(aud_syd_switch_baseaddr++) = (0x30000 | sl_get_physical_port_id(i));
            vlan_sec_tx_bitmap |= (1<<sl_get_physical_port_id(i));
        }
    }

    // Is this redundant?
    if ((vlan_pri_tx_bitmap && vlan_sec_tx_bitmap) &&  net_common->config.red_en)
    {
        AUD_SYD_MAC_RX_SW_EN = 0xf;

        // Using bitmap for tx. e.g vlan port 0,2 = 0101(0x4), port 234    = 11100 (0x1c)
        AUD_SYD_MAC_TX_SWITCH_0 = vlan_pri_tx_bitmap;
        AUD_SYD_MAC_TX_SWITCH_1 = vlan_sec_tx_bitmap;

        AUD_SYD_SCHED_CONTROL = AUD_SYD_SCHED_CONTROL__RXEN | AUD_SYD_SCHED_CONTROL__PRI_TXEN | AUD_SYD_SCHED_CONTROL__SEC_TXEN | AUD_SYD_SCHED_CONTROL__REDEN;

        // In redundant, need to enable Marvel header for not HC products
        if (!net_common->high_channel_count)
        {
            sl_set_switch_port_marvel_header(cpu_port, PORT_ENABLE);
        }
        else
        {
            sl_set_switch_port_marvel_header(cpu_port, PORT_DISABLE);
        }
    }
    else
    {
        AUD_SYD_SCHED_CONTROL = AUD_SYD_SCHED_CONTROL__RXEN | AUD_SYD_SCHED_CONTROL__PRI_TXEN;
    }
}

static void akashi_configure_switch(net_common_t *net_common)
{
    uint8_t cpu_port;
    printk(KERN_INFO "Configuring network switch chipset\n");

    /* Wait for init_ready bit. */
    sl_switch_init_ready();
    /* Check PPU and make sure this is enabled */
    sl_switch_ppu_enable();

    sl_disable_switch_phy_port_all();
    sl_set_switch_vlan_default();

    /* configure vlan configuration on switch chipset */
    sl_set_switch_vlan_config_all(net_common->config.vlan_config);

    /* set up redundancy config on FPGA */
    akashi_switch_redundancy_setup(net_common);

    // setup QoS configuration
    sl_set_switch_qos_all();

    // disabling switch EEE for 88E6352 & 88E6320 series
    sl_disable_eee_all(net_common->config.vlan_config);

    if (net_common->dante_net_st.net_chip_info.adapter_name == SWITCH_88E6361)
    {
        sl_get_cpu_port(&cpu_port);
        sl_set_rgmi_delay(cpu_port);
    }
    else
    {
        /* set rgmii port 5 delay */
        sl_set_rgmi_delay(SWITCH_MB_CTL_PORT5);

        /* set rgmii port 6 delay */
        sl_set_rgmi_delay(SWITCH_MB_CTL_PORT6);
    }

    // multicast filtering configuration
    set_multicast_filtering(net_common->config.vlan_config, net_common);

    /* Enable all switch phy/serdes/switch ports */
    sl_enable_phy_all(net_common->config.vlan_config);
    sl_serdes_ports_up_all(net_common->config.vlan_config);
    sl_enable_switch_all(net_common->config.vlan_config);
}
#endif /* CONFIG_AKASHI_CONFIGURE_SWITCH */

static __inline__ uint8_t akashi_bcd_to_uint(uint8_t bcd)
{
    uint8_t hi, lo;
    uint8_t num;

    hi = (bcd & 0xf0) >> 4;
    if (hi > 9)
    {
        /* error case, set it to zero */
        hi = 0;
        printk(KERN_ERR "%s: invalid bcd 0x%x\n", __func__, bcd);
        return 0;
    }

    lo = bcd & 0x0f;
    if (lo > 9)
    {
        /* error case, set it to zero */
        lo = 0;
        printk(KERN_ERR "%s: invalid bcd 0x%x\n", __func__, bcd);
        return 0;
    }

    num = (hi * 10) + lo;
    return num;
}

static void akashi_read_signature(void)
{
    uint8_t *name;

    name = (uint8_t *)(AUD_SYD_BASEADDR);
    printk(KERN_INFO "Platform Name <%c%c%c%c>, FPGA version <%d.%d.%d-rc%d>", \
            name[3],name[2],name[1],name[0], \
            akashi_bcd_to_uint(AUD_SYD_FPGA_VERSION_MAJOR),
            akashi_bcd_to_uint(AUD_SYD_FPGA_VERSION_MINOR),
            akashi_bcd_to_uint(AUD_SYD_FPGA_VERSION_PATCH),
            akashi_bcd_to_uint(AUD_SYD_FPGA_VERSION_RC));
}

static void akashi_tasklet_init(net_common_t *net_common)
{
    tasklet_init(&net_common->send_tasklet, denet_sendBH, (unsigned long)net_common);
    tasklet_init(&net_common->priority_send_tasklet, denet_priority_sendBH, (unsigned long)net_common);
    tasklet_init(&net_common->recv_tasklet, denet_recvBH, (unsigned long)net_common);
    tasklet_init(&net_common->smi_tasklet, denet_smiBH, (unsigned long)net_common);
}

static int akashi_irq_init(net_common_t *net_common)
{
    int r;

    /* SMI IRQ */
#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: request akashi-smi irq %d\n", __func__, net_common->smi_irq);
#endif

#if defined(CONFIG_AKASHI_EMAC_0_SMI_IRQ) && !defined(CONFIG_AKASHI_PS_MAC)
    r = request_irq(net_common->smi_irq, &smi_interrupt, net_common->irq_flags, "akashi-smi", net_common);
    if (r)
    {
        printk(KERN_ERR "%s: could not allocate interrupt %d\n", __func__, net_common->smi_irq);
        return r;
    }
#endif

    /* RX IRQ */
#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: request akashi-rx irq %d\n", __func__, net_common->rx_irq);
#endif
    r = request_irq(net_common->rx_irq, &denet_rx_interrupt, net_common->irq_flags, "akashi-rx", net_common);
    if (r)
    {
        printk(KERN_ERR "%s: Could not allocate interrupt %d\n", __func__, net_common->rx_irq);
        return r;
    }

    // TX IRQ
#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: request akashi-tx irq %d\n", __func__, net_common->tx_irq);
#endif
    r = request_irq(net_common->tx_irq, &denet_tx_interrupt, net_common->irq_flags, "akashi-tx", net_common);
    if (r)
    {
        printk(KERN_ERR "%s: Could not allocate interrupt %d\n", __func__, net_common->tx_irq);
        return r;
    }

    // RX ERR IRQ
#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: request akashi-err irq %d\n", __func__, net_common->err_irq);
#endif
    r = request_irq(net_common->err_irq, &denet_err_interrupt, net_common->irq_flags, "akashi-err", net_common);
    if (r)
    {
        printk(KERN_ERR "%s: Could not allocate interrupt %d\n", __func__, net_common->err_irq);
        return r;
    }

    // TS IRQ
#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: request akashi-ts irq %d\n", __func__,  net_common->ts_irq);
#endif
    r = request_irq(net_common->ts_irq, &denet_ts_interrupt, net_common->irq_flags, "akashi-ts", net_common);
    if (r)
    {
        printk(KERN_ERR "%s: Could not allocate interrupt %d\n", __func__, net_common->ts_irq);
        return r;
    }

    return 0;
}

static void akashi_irq_cleanup(net_common_t *net_common)
{
#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: free akashi-rx irq %d\n", __func__, net_common->rx_irq);
#endif
    free_irq(net_common->rx_irq, net_common);

#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: free akashi-tx irq %d\n", __func__, net_common->tx_irq);
#endif
    free_irq(net_common->tx_irq, net_common);

#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: free akashi-ts irq %d\n", __func__, net_common->ts_irq);
#endif
    free_irq(net_common->ts_irq, net_common);

#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: free akashi-err irq %d\n", __func__, net_common->err_irq);
#endif
    free_irq(net_common->err_irq, net_common);

#if defined(CONFIG_AKASHI_EMAC_0_SMI_IRQ) && !defined(CONFIG_AKASHI_PS_MAC)
#ifdef CONFIG_AKASHI_DEBUG_IRQ
    printk(KERN_INFO "%s: free akashi-smi irq %d\n", __func__, net_common->smi_irq);
#endif

    free_irq(net_common->smi_irq, net_common);
#endif
}

static void akashi_set_hw_mac_addr(net_local_t *net_local)
{
    struct net_device *dev = net_local->dev;
    net_common_t *net_common = net_local->common;
    u32 mac_h, mac_m, mac_l;
    u8 *macptr_l, *macptr_m, *macptr_h, dev_addr[ETH_ALEN];
    bool is_primary = is_primary_dev(dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,16,0)
     memcpy(dev->dev_addr, net_common->base_mac_addr, ETH_ALEN);
     if (!is_primary)
     	dev->dev_addr[5]++;

     memcpy(dev_addr, dev->dev_addr, ETH_ALEN);
#else
    ether_addr_copy(dev_addr, net_common->base_mac_addr);
    if (!is_primary)
        eth_addr_inc(dev_addr);

    eth_hw_addr_set(dev, dev_addr);
#endif

    // Write MAC address
    mac_h = (dev_addr[0] << 8) | dev_addr[1];
    mac_m = (dev_addr[2] << 8) | dev_addr[3];
    mac_l = (dev_addr[4] << 8) | dev_addr[5];
    macptr_h = (u8*) &mac_h;
    macptr_m = (u8*) &mac_m;
    macptr_l = (u8*) &mac_l;

    printk(KERN_INFO "%s: %s mac address %02x:%02x:%02x:%02x:%02x:%02x\n",
        __func__,
        dev->name,
        dev_addr[0],
        dev_addr[1],
        dev_addr[2],
        dev_addr[3],
        dev_addr[4],
        dev_addr[5]);

    if (is_primary)
    {
        AUD_SYD_PROTO_MAC_ADDR0_HI = mac_h;
        AUD_SYD_PROTO_MAC_ADDR0_ME = mac_m;
        AUD_SYD_PROTO_MAC_ADDR0_LO = mac_l;
    }
    else
    {
        AUD_SYD_PROTO_MAC_ADDR1_HI = mac_h;
        AUD_SYD_PROTO_MAC_ADDR1_ME = mac_m;
        AUD_SYD_PROTO_MAC_ADDR1_LO = mac_l;
    }
}

static int akashi_set_mac_address(struct net_device *dev, void *p)
{
    net_local_t *net_local = netdev_priv(dev);
    net_common_t *net_common = net_local->common;
    struct sockaddr * addr = p;

    if (!is_valid_ether_addr(addr->sa_data))
        return -EINVAL;

    printk("addr->sa_data MAC address is %02X:%02X:%02X:%02X:%02X:%02X\n",
        addr->sa_data[0], addr->sa_data[1],
        addr->sa_data[2], addr->sa_data[3],
        addr->sa_data[4], addr->sa_data[5]);

    memcpy(net_common->base_mac_addr, addr->sa_data, 6);

    akashi_set_hw_mac_addr(net_local);

    return 0;
}

#ifdef CONFIG_PROC_FS
static int akashi_proc_mac_addr_show(struct seq_file *m, void *v)
{
    const struct file *file = m->file;
    struct inode *inode = file->f_inode;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
    net_common_t *net_common = (net_common_t *)PDE_DATA(inode);
#else
    net_common_t *net_common = pde_data(inode);
#endif

    seq_printf(m, "%02X:%02X:%02X:%02X:%02X:%02X\n",
        net_common->base_mac_addr[0],
        net_common->base_mac_addr[1],
        net_common->base_mac_addr[2],
        net_common->base_mac_addr[3],
        net_common->base_mac_addr[4],
        net_common->base_mac_addr[5]);

    return 0;
}

static int akashi_proc_mac_addr_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
	return single_open(file, akashi_proc_mac_addr_show, PDE_DATA(inode));
#else
	return single_open(file, akashi_proc_mac_addr_show, pde_data(inode));
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations proc_mac_addr_fops =
{
    .owner      = THIS_MODULE,
    .open       = akashi_proc_mac_addr_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#else
static const struct proc_ops proc_mac_addr_fops = {
    .proc_open       = akashi_proc_mac_addr_open,
    .proc_read       = seq_read,
    .proc_lseek      = seq_lseek,
    .proc_release    = single_release,
};
#endif
static int akashi_proc_red_flag_show(struct seq_file *m, void *v)
{
    const struct file *file = m->file;
    struct inode *inode = file->f_inode;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
    net_common_t *net_common = (net_common_t *)PDE_DATA(inode);
#else
    net_common_t *net_common = pde_data(inode);
#endif

    seq_printf(m, "%d:%d:%d:%d\n",
        net_common->config.red_en,
        net_common->config.vlan_pri,
        net_common->config.vlan_sec,
        net_common->config.vlan_config);

    return 0;
}

static int akashi_proc_red_flag_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
	return single_open(file, akashi_proc_red_flag_show, PDE_DATA(inode));
#else
	return single_open(file, akashi_proc_red_flag_show, pde_data(inode));
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations proc_red_flag_fops =
{
    .owner      = THIS_MODULE,
    .open       = akashi_proc_red_flag_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#else
static const struct proc_ops proc_red_flag_fops = {
    .proc_open       = akashi_proc_red_flag_open,
    .proc_read       = seq_read,
    .proc_lseek      = seq_lseek,
    .proc_release    = single_release,
};
#endif

static int akashi_proc_ctr_port_show(struct seq_file *m, void *v)
{
    const struct file *file = m->file;
    struct inode *inode = file->f_inode;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
    net_common_t *net_common = (net_common_t *)PDE_DATA(inode);
#else
    net_common_t *net_common = pde_data(inode);
#endif

    seq_printf(m, "0x%x\n", net_common->config.control_ports);

    return 0;
}

static int akashi_proc_ctr_port_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
    return single_open(file, akashi_proc_ctr_port_show, PDE_DATA(inode));
#else
    return single_open(file, akashi_proc_ctr_port_show, pde_data(inode));
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations proc_ctr_port_fops =
{
    .owner      = THIS_MODULE,
    .open       = akashi_proc_ctr_port_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#else
static const struct proc_ops proc_ctr_port_fops = {
    .proc_open       = akashi_proc_ctr_port_open,
    .proc_read       = seq_read,
    .proc_lseek      = seq_lseek,
    .proc_release    = single_release,
};
#endif

static int akashi_proc_ex_phys_show(struct seq_file *m, void *v)
{
    const struct file *file = m->file;
    struct inode *inode = file->f_inode;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
    net_common_t *net_common = (net_common_t *)PDE_DATA(inode);
#else
    net_common_t *net_common = pde_data(inode);
#endif

    seq_printf(m, "0x%x\n", net_common->config.external_phys);

    return 0;
}

static int akashi_proc_ex_phys_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
    return single_open(file, akashi_proc_ex_phys_show, PDE_DATA(inode));
#else
    return single_open(file, akashi_proc_ex_phys_show, pde_data(inode));
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations proc_ex_phys_fops =
{
    .owner      = THIS_MODULE,
    .open       = akashi_proc_ex_phys_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#else
static const struct proc_ops proc_ex_phys_fops = {
    .proc_open       = akashi_proc_ex_phys_open,
    .proc_read       = seq_read,
    .proc_lseek      = seq_lseek,
    .proc_release    = single_release,
};
#endif

static int akashi_proc_debug_open(struct inode *inode, struct file *file)
{
    file->private_data = net_common_context;

    return 0;
}

static ssize_t akashi_proc_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    net_common_t *net_common = (net_common_t *)file->private_data;
    ssize_t ret;
    char flag_str[128];
    int writelen;

    memset(flag_str, 0, sizeof(flag_str));
    ret = simple_write_to_buffer(flag_str, sizeof(flag_str), offset, buf, count);
    if (ret > 0)
        writelen = max_t(int, writelen, *offset);

    sscanf(flag_str, "%x", &net_common->debug.flag);

    return ret;
}

static ssize_t akashi_proc_debug_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    net_common_t *net_common = (net_common_t *)file->private_data;
    char flag_str[16];

    sprintf(flag_str, "0x%08x\n", net_common->debug.flag);

    return simple_read_from_buffer(buf, count, offset, flag_str, strlen(flag_str));
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations proc_debug_fops =
{
    .owner      = THIS_MODULE,
    .open       = akashi_proc_debug_open,
    .read       = akashi_proc_debug_read,
    .write      = akashi_proc_debug_write,
};
#else
static const struct proc_ops proc_debug_fops = {
    .proc_open       = akashi_proc_debug_open,
    .proc_read       = akashi_proc_debug_read,
    .proc_write      = akashi_proc_debug_write,
};
#endif

static int akashi_proc_mcast_open(struct inode *inode, struct file *file)
{
    file->private_data = net_common_context;

    return 0;
}

static int akashi_print_mcast_info(char *buf, struct net_device *dev)
{
    net_local_t *net_local ;
    struct netdev_hw_addr_list *list;
    struct netdev_hw_addr *mc_ptr;
    char *str = buf;
    int i;

    str += sprintf(str, "[%s]\n", dev->name);
    str += sprintf(str, "  MAC:\n");
    list = &dev->mc;
    list_for_each_entry(mc_ptr, &list->list, list)
    {
        str += sprintf(str, "    %02x:%02x:%02x:%02x:%02x:%02x\n",
            mc_ptr->addr[0],
            mc_ptr->addr[1],
            mc_ptr->addr[2],
            mc_ptr->addr[3],
            mc_ptr->addr[4],
            mc_ptr->addr[5]);
    }

    str += sprintf(str, "  Hash Filters:\n");
    net_local = netdev_get_priv(dev);
    for (i = 0; i < AUD_SYD_PROTO_HASH_FILTERS; i += 2)
    {
        str += sprintf(str, "    %04x %04x\n", net_local->mcast_hash[i], net_local->mcast_hash[i + 1]);
    }

    return (str - buf);
}

static ssize_t akashi_proc_mcast_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    net_common_t *net_common = (net_common_t *)file->private_data;
    char *str = net_common->debug.buf;
    int len;

    if (net_common->primary_dev)
    {
        str += akashi_print_mcast_info(str, net_common->primary_dev);
    }

    if (net_common->secondary_dev)
    {
        str += akashi_print_mcast_info(str, net_common->secondary_dev);
    }

    /* Total length of print buffer */
    len = str - net_common->debug.buf;

    return simple_read_from_buffer(buf, count, offset, net_common->debug.buf, len);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations proc_mcast_fops =
{
    .owner      = THIS_MODULE,
    .open       = akashi_proc_mcast_open,
    .read       = akashi_proc_mcast_read,
};
#else
static const struct proc_ops proc_mcast_fops = {
    .proc_open       = akashi_proc_mcast_open,
    .proc_read       = akashi_proc_mcast_read,
};
#endif

static int akashi_proc_mac_filter_open(struct inode *inode, struct file *file)
{
    file->private_data = net_common_context;

    return 0;
}

static ssize_t akashi_proc_mac_filter_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    net_common_t *net_common = (net_common_t *)file->private_data;
    ssize_t ret;
    char mac_str[64];
    int mac_int[6];
    uint8_t *mac = net_common->debug.mac_filter;
    int writelen;
    int i;

    memset(mac_str, 0, sizeof(mac_str));
    ret = simple_write_to_buffer(mac_str, sizeof(mac_str), offset, buf, count);
    if (ret > 0)
        writelen = max_t(int, writelen, *offset);

    sscanf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x",
        mac_int + 0,
        mac_int + 1,
        mac_int + 2,
        mac_int + 3,
        mac_int + 4,
        mac_int + 5);

    for (i = 0; i < 6; i++)
    {
        mac[i] = mac_int[i] & 0xff;
    }

    return ret;
}

static ssize_t akashi_proc_mac_filter_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    net_common_t *net_common = (net_common_t *)file->private_data;
    char mac_str[64];
    uint8_t *mac = net_common->debug.mac_filter;

    sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return simple_read_from_buffer(buf, count, offset, mac_str, strlen(mac_str));
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations proc_mac_filter_fops =
{
    .owner      = THIS_MODULE,
    .open       = akashi_proc_mac_filter_open,
    .read       = akashi_proc_mac_filter_read,
    .write      = akashi_proc_mac_filter_write,
};
#else
static const struct proc_ops proc_mac_filter_fops = {
    .proc_open       = akashi_proc_mac_filter_open,
    .proc_read       = akashi_proc_mac_filter_read,
    .proc_write      = akashi_proc_mac_filter_write,
};
#endif

static const struct net_device_ops denet_primary_netdev_ops = {
    /* the hardware transmission method */
    .ndo_start_xmit         = akashi_start_xmit,
    .ndo_open               = denet_open,
    .ndo_stop               = denet_stop,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)
    .ndo_eth_ioctl          = denet_ioctl,
    .ndo_siocdevprivate     = denet_private_ioctl,
#else
    .ndo_do_ioctl           = denet_ioctl,
#endif
    .ndo_tx_timeout         = denet_tx_timeout,
    .ndo_get_stats          = denet_get_stats,
    .ndo_set_rx_mode        = akashi_set_multicast_list,
    .ndo_set_mac_address    = akashi_set_mac_address,
};

static const struct net_device_ops denet_secondary_netdev_ops = {
    /* the hardware transmission method */
    .ndo_start_xmit         = akashi_start_xmit,
    .ndo_open               = vnet_open,
    .ndo_stop               = vnet_stop,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)
    .ndo_eth_ioctl          = vnet_ioctl,
    .ndo_siocdevprivate     = denet_private_ioctl,
#else
    .ndo_do_ioctl           = vnet_ioctl,
#endif
    .ndo_tx_timeout         = denet_tx_timeout,
    .ndo_get_stats          = vnet_get_stats,
    .ndo_set_rx_mode        = akashi_set_multicast_list,
};


static void akashi_create_proc_entries(net_common_t *net_common)
{
    char *entry_name;

    net_common->proc.root = proc_mkdir(AKASHI_PROC_ROOT_NAME, NULL);
    if (net_common->proc.root)
    {
        entry_name = "ethmaddr";
        net_common->proc.mac_addr = proc_create_data(entry_name, 0, net_common->proc.root, &proc_mac_addr_fops, net_common);
        if (!net_common->proc.mac_addr)
        {
            printk(KERN_ERR "Error: failed to create /proc/%s/%s\n", AKASHI_PROC_ROOT_NAME, entry_name);
        }

        entry_name = "red_flag";
        net_common->proc.red_flag = proc_create_data(entry_name, 0, net_common->proc.root, &proc_red_flag_fops, net_common);
        if (!net_common->proc.mac_addr)
        {
            printk(KERN_ERR "Error: failed to create /proc/%s/%s\n", AKASHI_PROC_ROOT_NAME, entry_name);
        }

        entry_name = "ctr_port";
        net_common->proc.ctr_port = proc_create_data(entry_name, 0, net_common->proc.root, &proc_ctr_port_fops, net_common);
        if (!net_common->proc.ctr_port)
        {
            printk(KERN_ERR "Error: failed to create /proc/%s/%s\n", AKASHI_PROC_ROOT_NAME, entry_name);
        }

        entry_name = "ex_phys";
        net_common->proc.ex_phys = proc_create_data(entry_name, 0, net_common->proc.root, &proc_ex_phys_fops, net_common);
        if (!net_common->proc.ex_phys)
        {
            printk(KERN_ERR "Error: failed to create /proc/%s/%s\n", AKASHI_PROC_ROOT_NAME, entry_name);
        }

        entry_name = "debug";
        net_common->proc.debug = proc_create_data(entry_name, 0, net_common->proc.root, &proc_debug_fops, net_common);
        if (!net_common->proc.debug)
        {
            printk(KERN_ERR "Error: failed to create /proc/%s/%s\n", AKASHI_PROC_ROOT_NAME, entry_name);
        }

        entry_name = "mcast";
        net_common->proc.mcast = proc_create_data(entry_name, 0, net_common->proc.root, &proc_mcast_fops, net_common);
        if (!net_common->proc.mcast)
        {
            printk(KERN_ERR "Error: failed to create /proc/%s/%s\n", AKASHI_PROC_ROOT_NAME, entry_name);
        }

        entry_name = "mac_filter";
        net_common->proc.mac_filter = proc_create_data(entry_name, 0, net_common->proc.root, &proc_mac_filter_fops, net_common);
        if (!net_common->proc.mac_filter)
        {
            printk(KERN_ERR "Error: failed to create /proc/%s/%s\n", AKASHI_PROC_ROOT_NAME, entry_name);
        }
    }
    else
    {
        printk(KERN_ERR "Error: failed to create /proc/%s\n", AKASHI_PROC_ROOT_NAME);
    }
}

static void akashi_destroy_proc_entries(net_common_t *net_common)
{
    if (net_common->proc.root)
    {
        if (net_common->proc.mac_addr)
        {
            proc_remove(net_common->proc.mac_addr);
            net_common->proc.mac_addr = NULL;
        }

        if (net_common->proc.red_flag)
        {
            proc_remove(net_common->proc.red_flag);
            net_common->proc.red_flag = NULL;
        }

        if (net_common->proc.ctr_port)
        {
            proc_remove(net_common->proc.ctr_port);
            net_common->proc.ctr_port = NULL;
        }

        if (net_common->proc.ex_phys)
        {
            proc_remove(net_common->proc.ex_phys);
            net_common->proc.ex_phys = NULL;
        }

        if (net_common->proc.debug)
        {
            proc_remove(net_common->proc.debug);
            net_common->proc.debug = NULL;
        }

        if (net_common->proc.mcast)
        {
            proc_remove(net_common->proc.mcast);
            net_common->proc.mcast = NULL;
        }

        if (net_common->proc.mac_filter)
        {
            proc_remove(net_common->proc.mac_filter);
            net_common->proc.mac_filter = NULL;
        }

        proc_remove(net_common->proc.root);
        net_common->proc.root = NULL;
    }
}
#endif

//-------------------------------------------------------------------
// Init Device
static int akashi_probe(struct platform_device *pdev)
{
    net_local_t *net_local;
    net_common_t *net_common;
    adapter_enum_t network_adapter, default_network_adapter;
    const struct of_device_id *match;
    struct resource *res;
    int r;
    dante_network_st_t *net_st;
    u32 redundancy_options[4];
    u32 control_ports_option;
    u32 external_phys_option;

    /* Allocate data for common data structure */
    net_common = (net_common_t *)kmalloc(sizeof(net_common_t), GFP_KERNEL);
    if (!net_common)
    {
        printk(KERN_ERR "Could not allocate Dante enet common context\n");
        return -ENOMEM;
    }

    /* Zero init all fields */
    memset(net_common, 0, sizeof(net_common_t));

    /* Save a pointer to our global variable */
    net_common_context = net_common;
    akashi_tasklet_init(net_common);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,13,0)
    if (!memcmp(ethmaddr, DEFAULT_ETHMADDR, sizeof(DEFAULT_ETHMADDR)) &&
        !of_get_mac_address(pdev->dev.of_node, net_common->base_mac_addr))
    {
        char addr[32];
        sprintf(addr, "%pM", net_common->base_mac_addr);
        ethmaddr = devm_kstrdup(&pdev->dev, addr, GFP_KERNEL);
    }
#endif

    denet_hw_addr_setup(net_common, ethmaddr);
    denet_redundancy_setup(net_common, red_flag);
    denet_ctr_port_setup(net_common, ctr_port);
    denet_ctr_port_info_setup(net_common, ctr_port_info);
    denet_ctr_port_config_setup(net_common, ctr_port_config);
    denet_external_phys_setup(net_common, ex_phys);
    denet_multicast_filtering_setup(net_common, multicast_white_list);

    /* =============================================================
     * Begin initialisation of eth0 physical interface
     */
    net_common->primary_dev = alloc_netdev(sizeof(net_local_t), PRIMARY_INTERFACE_NAME, NET_NAME_PREDICTABLE, ether_setup);
    if (!net_common->primary_dev)
    {
        printk(KERN_ERR "Could not allocate %s device\n", PRIMARY_INTERFACE_NAME);
        return -ENOMEM;
    }

    /* This is our primary physical link eth0 */
    printk(KERN_INFO "%s: alloc net device '%s'\n", __func__, net_common->primary_dev->name);

    /* Generic network setup */
    ether_setup(net_common->primary_dev);

    /* Initialize our private data. */
    net_local = netdev_get_priv(net_common->primary_dev);
    memset(net_local, 0, sizeof(net_local_t));
    net_local->index = 0; /* eth0 */
    net_local->dev = net_common->primary_dev;

    /* Initialize pointer to common interface data. */
    net_local->common = net_common;
    net_st = &net_common->dante_net_st;

    /*
     * IRQ definitions came from device tree and are no longer harcoded.
     * This flag is for manual override of the IRQ trigger type.
     * The default trigger type is defined in the device tree.
     * Please configure the irq trigger type in the device tree accordingly
     * and leave this 'irq_flags' field at zero during driver initialization.
     */
    net_common->irq_flags = 0;

    /* TX skbuff list init */
    skb_queue_head_init(&net_common->tx_skb_list);
    skb_queue_head_init(&net_common->tx_priority_skb_list);

    /* Init spin lock for TX skbuff list */
    spin_lock_init(&net_common->tx_priority_lock);
    spin_lock_init(&net_common->tx_lock);

    /* Init spin lock for reset */
    spin_lock_init(&net_common->reset_lock);

    init_waitqueue_head(&net_common->tsqueue);
    init_waitqueue_head(&net_common->psqueue);
    init_waitqueue_head(&net_common->psqueue1);

    match = of_match_node(akashi_of_match, pdev->dev.of_node);
    if (!match)
    {
        printk(KERN_ERR "Failed to find matching akashi from device tree\n");
        return -ENODEV;
    }

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
    {
        printk(KERN_ERR "Failed to find resource for akashi\n");
        return -ENODEV;
    }

    /* Device tree parameters shall override module params */
    r = of_property_read_variable_u32_array(pdev->dev.of_node,
                    "redundancy", redundancy_options, 4, 4);
    if (r == 4)
    {
        net_common->config.red_en = redundancy_options[0];
        net_common->config.vlan_pri = redundancy_options[1];
        net_common->config.vlan_sec = redundancy_options[2];
        net_common->config.vlan_config = redundancy_options[3];
    }

    r = of_property_read_variable_u32_array(pdev->dev.of_node,
                    "control-ports", &control_ports_option, 1, 1);
    if (r == 1)
    {
        net_common->config.control_ports = control_ports_option;
    }

    r = of_property_read_variable_u32_array(pdev->dev.of_node,
                    "external-phys", &external_phys_option, 1, 1);
    if (r == 1)
    {
        net_common->config.external_phys = external_phys_option;
    }

    /* Change the addresses to be virtual */
    net_common->phyaddr = res->start;
    net_common->remap_size = res->end - res->start + 1;
#ifdef CONFIG_AKASHI_64BIT_ARCH
    printk(KERN_INFO "Akashi resource address: 0x%016llx, size: 0x%016llx\n", res->start, net_common->remap_size);
#else
    printk(KERN_INFO "Akashi resource address: 0x%08x, size: 0x%08x\n", res->start, net_common->remap_size);
#endif
    if (!request_mem_region(net_common->phyaddr, net_common->remap_size, "akashi"))
    {
        return -ENOMEM;
    }

#ifdef CONFIG_AKASHI_64BIT_ARCH
    net_common->baseaddr = (u64) ioremap(net_common->phyaddr, net_common->remap_size);
#else
    net_common->baseaddr = (u32) ioremap(net_common->phyaddr, net_common->remap_size);
#endif
    /* This global variable is critical as all the register access macros are based on this virtual address */
    aud_syd_virtual_base_addr = (uint8_t *)(net_common->baseaddr);

#ifdef CONFIG_AKASHI_DEBUG_IOMAP
    /* For security reasons, do not expose virtual address */
#ifdef CONFIG_AKASHI_64BIT_ARCH
    printk(KERN_INFO "%s: Akashi EMAC at 0x%016llx mapped to 0x%016llx\n",
            __func__, net_common->phyaddr, net_common->baseaddr);
#else
    printk(KERN_INFO "%s: Akashi EMAC at 0x%08lx mapped to 0x%08lx\n",
            __func__, net_common->phyaddr, net_common->baseaddr);
#endif
#endif

    /* Define FPGA product type. HC has 256 channels */
    if (AUD_SYD_INFO_TXC > 64)
    {
        net_common->high_channel_count = 1;
        printk(KERN_INFO "High Channel Count Device");
    }

    /*
     * Perform sanity check by means of register access.
     * This allow us to detect memory access failure early in the driver initialization.
     */
    akashi_read_signature();

    /* check if fpga has priority queue capability */
    if (AUD_SYD_INFO_CAPABILITY & AUD_SYD__CAPABILITY_PRIORITY_TX_QUEUEE)
    {
        net_common->has_priority_queue = 1;
        printk(KERN_INFO "%s: has priority tx queue\n", __func__);
    }

    if (AUD_SYD_INFO_CAPABILITY & AUD_SYD__CAPABILITY_TSCOUNT)
    {
        net_common->has_tscount = 1;
        net_common->max_rx_len = 1540;
        printk(KERN_INFO "%s: has timestamp count info\n", __func__);
    }
    else
    {
        net_common->max_rx_len = 1528;
    }

#ifdef CONFIG_AKASHI_PS_MAC
    r = zynq_macb_init(pdev, net_common->primary_dev);
    if (r)
    {
        printk(KERN_ERR "%s: failed to zynq_macb_init for net device %s, aborting\n", __func__, net_common->primary_dev->name);
        return r;
    }
#endif

    /* probing network chipset */
    sl_probe_chipset(1, &network_adapter, &default_network_adapter);

    /* Printing detected adapter */
    print_detected_adapter(network_adapter, default_network_adapter);

    /* load adapter info to driver */
    load_network_info(net_local->index, net_local);

    /* setup cpu port number */
    akashi_configure_cpu_port(net_common);

    /* First, find out what's going on with the PHY. */
    if (get_phy_status(net_common))
    {
        printk(KERN_ERR "%s: Unable to get phy status\n", __func__);
    }

    /* Set MAC speed to match PHY speed */
    if (net_st->net_chip_info.adapter_type == ADAPTER_SWITCH)
    {

#ifdef CONFIG_AKASHI_CONFIGURE_SWITCH
        /*
         * Additional switch configuration
         * Note:
         *   On Zynq IP Core, we do not provide uboot as a bundle for the Dante
         *   Container. Therefore, it is the sole responsibility of the kernel akashi
         *   to configure the network switch.
         *
         *   On Brooklyn3 which is a platform built entirely by Audinate, we provide
         *   our own port of uboot which is responsible for setting up and configuring
         *   the network switch. This block of code should be disabled for Brooklyn3 to
         *   avoid configuring the network switch chipset twice which is confirmed to
         *   cause network issues.
         */
        akashi_configure_switch(net_common);
#endif

        if (net_common->config.red_en & ETH_SWITCH_MODE_REDUNDANT)
        {
            AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | AUD_SYD_MAC_SWITCH_CONTROL;
        }
        else
        {
            AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII;
        }

    }
    else if (net_st->net_chip_info.adapter_type == ADAPTER_PHY)
    {
        uint32_t syd_mac_switch_ctl = 0;
        uint32_t mac_control_gtx_clock = AUD_SYD_MAC_CONTROL_GTX_CLK;

        // copy mii addr for primary
        net_local->mii_addr = net_st->net_chip_info.phy_address[0];

        if (net_st->net_chip_info.phy_redundant)
        {
            net_common->config.red_en = ETH_SWITCH_MODE_REDUNDANT;
        }
        else
        {
            net_common->config.red_en = ETH_SWITCH_MODE_SWITCH;
        }

#ifdef CONFIG_AKASHI_CONFIGURE_SWITCH
        // In case phy ksz9031, need to give RGMII delay
        if(net_st->net_chip_info.adapter_name == PHY_KSZ9031)
        {
            sl_set_rgmi_delay_ksz9031(PHY_PRIMARY_PORT);
            if(sl_get_chipset_info()->phy_redundant)
                sl_set_rgmi_delay_ksz9031(PHY_SECONDARY_PORT);
        }
        else if(net_st->net_chip_info.adapter_name == PHY_KSZ9131) 
        {
            sl_set_rgmi_delay_ksz9131(PHY_PRIMARY_PORT);
            if(sl_get_chipset_info()->phy_redundant)
                sl_set_rgmi_delay_ksz9131(PHY_SECONDARY_PORT);
        }
#endif

        if (net_common->config.red_en)
        {
            syd_mac_switch_ctl = AUD_SYD_MAC_SWITCH_CONTROL | AUD_SYD_MAC_REDUNDANT_TYPE;
        }

        if (net_st->interface_stat[0].link_status || net_st->interface_stat[1].link_status)
        {
            if (net_st->net_chip_info.phy_redundant)
            {
                int speed_sum;

                speed_sum = net_st->interface_stat[0].link_speed + net_st->interface_stat[1].link_speed;

                // Mixed 1G and 100Mbps ??
                if (speed_sum == 1100)
                {
                    if (net_st->interface_stat[0].link_speed == 100)
                    {
                        // Shut down primary port.
                        sl_isolate_phy(net_st->net_chip_info.phy_address[0], PHY_ISOLATE_ENABLE);
                    }
                    else
                    {
                        // Shut down secondary port
                        sl_isolate_phy(net_st->net_chip_info.phy_address[1], PHY_ISOLATE_ENABLE);
                    }
                }

                if (net_st->interface_stat[0].link_speed == 1000 || net_st->interface_stat[1].link_speed == 1000)
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock | syd_mac_switch_ctl;
                }
                else if (net_st->interface_stat[0].link_speed == 100 || net_st->interface_stat[1].link_speed == 100)
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_25MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock | syd_mac_switch_ctl;
                }
                else
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock | syd_mac_switch_ctl;
                }
            }
            else
            {
                if (net_st->interface_stat[0].link_speed == 1000)
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock;
                }
                else
                {
                    AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_25MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock;
                }
            }
        }
        else
        {
            // Both link is not on and default is 1 G
            if (net_st->net_chip_info.phy_redundant)
            {
                AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock | syd_mac_switch_ctl;
            }
            else
            {
                AUD_SYD_MAC_CONTROL = AUD_SYD_MAC_CONTROL_125MHZ | AUD_SYD_MAC_CONTROL_RGMII | mac_control_gtx_clock;
            }
        }
    }
    else
    {
        printk(KERN_ERR "Adapter Type: Unknown\n");
    }

    /* Configure MAC address for eth0 */
    akashi_set_hw_mac_addr(net_local);

    /* Init miscellaneous registers related to broadcast and port range */
    akashi_init_misc(net_common->primary_dev);

    if (net_st->net_chip_info.adapter_type == ADAPTER_SWITCH)
    {
        printk(KERN_INFO "Adapter Type: Switch Detected\n");
        // Check if we actually have a switch - we don't on the test jig
        if (sl_switch_busy_check() != 0)
        {

            printk(KERN_ERR "Cannot access switch - skip switch config\n");
            goto skip_switch_config;
        }
        else
        {
            marvell_switch_led_init();
        }

        if (net_common->config.external_phys)
        {
            int i;
            // Set default external phys clear interrupt register addr
            for (i = 0; i < MAX_PHY_CHIPSETS; i++)
            {
                net_st->ext_phy_int_clear_reg[i] = DEFAULT_SWITCH_EXTERNAL_PHY_INT_STATUS_REG;
            }
        }
    }
    else if (net_st->net_chip_info.adapter_type == ADAPTER_PHY)
    {
        printk(KERN_INFO "Adapter Type: Phy Detected\n");
        // Configure PHY LEDs - Marvel 88E1510 led configuring
        sl_phy_led_configuration(PHY_PRIMARY_PORT);
        if (net_st->net_chip_info.phy_redundant)
        {
            sl_phy_led_configuration(PHY_SECONDARY_PORT);
        }
    }
    else
    {
        printk(KERN_ERR "Adapter Type: Unknown\n");
    }

    if (!net_common->config.red_en)
    {
        printk(KERN_INFO "%s: Redundancy Option: DISABLED\n", __func__);
    }
    else
    {
        if (net_common->config.red_en & ETH_SWITCH_MODE_SWITCH)
        {
            printk(KERN_INFO "%s: Redundancy Option: SWITCH\n", __func__);
        }
        else if (net_common->config.red_en & ETH_SWITCH_MODE_REDUNDANT)
        {
            printk(KERN_INFO "%s: Redundancy Option: REDUNDANT\n", __func__);
        }
        else if (net_common->config.red_en & ETH_SWITCH_MODE_VLANS)
        {
            printk(KERN_INFO "%s: Redundancy Option: VLANS\n", __func__);
        }

        printk(KERN_INFO "VLAN primary         : 0x%02x\n", net_common->config.vlan_pri);
        printk(KERN_INFO "VLAN secondary       : 0x%02x\n", net_common->config.vlan_sec);
        printk(KERN_INFO "VLAN config          : 0x%08x\n", net_common->config.vlan_config);
        if (net_common->config.control_ports)
            printk(KERN_INFO "Control Ports        : 0x%02x\n", net_common->config.control_ports);

        if (net_common->config.control_ports_config)
            printk(KERN_INFO "Control Ports config : 0x%08x\n", net_common->config.control_ports_config);

        printk(KERN_INFO "External Phys        : 0x%x\n", net_common->config.external_phys);
    }

    // init for network interface smi interrupt.
    smi_int_init(net_common);

    net_common->smi_irq = platform_get_irq_byname(pdev, "smi");
    if (net_common->smi_irq <= 0)
    {
        printk(KERN_ERR "%s: failed to get platform irq for SMI\n", __func__);
        return -ENXIO;
    }

    net_common->ts_irq = platform_get_irq_byname(pdev, "ts");
    if (net_common->ts_irq <= 0)
    {
        printk(KERN_ERR "%s: failed to get platform irq for TS\n", __func__);
        return -ENXIO;
    }

    net_common->rx_irq = platform_get_irq_byname(pdev, "rx");
    if (net_common->rx_irq <= 0)
    {
        printk(KERN_ERR "%s: failed to get platform irq for RX\n", __func__);
        return -ENXIO;
    }

    net_common->tx_irq = platform_get_irq_byname(pdev, "tx");
    if (net_common->tx_irq <= 0)
    {
        printk(KERN_ERR "%s: failed to get platform irq for TX\n", __func__);
        return -ENXIO;
    }

    net_common->err_irq = platform_get_irq_byname(pdev, "rxerr");
    if (net_common->err_irq <= 0)
    {
        printk(KERN_ERR "%s: failed to get platform irq for ERR\n", __func__);
        return -ENXIO;
    }

    /* Init TX timestamps queue */
    akashi_txts_queue_init(net_common);

    /* Init various interrupts lines */
    r = akashi_irq_init(net_common);
    if (r != 0)
    {
        return r;
    }

#if defined(CONFIG_AKASHI_EMAC_0_SMI_IRQ) && defined(CONFIG_AKASHI_PS_MAC)
    /* SMI interrupt connects with GEM0 of the PS MAC, under the primary_dev */
    r = zynq_macb_ext_irq_init(net_common->primary_dev);
    if (r)
    {
        printk(KERN_ERR "%s: failed to initialize PS MAC IRQ\n", __func__);
        return r;
    }
#endif

#if defined(CONFIG_AKASHI_EMAC_0_SMI_IRQ) && !defined(CONFIG_AKASHI_PS_MAC)
    // enable interrupt.
    AUD_SYD_SMI_INTR = AUD_SYD_SMI_INT_ENABLE;
#endif

skip_switch_config:
    printk(KERN_INFO "%s: using fifo mode\n", __func__);

    net_common->primary_dev->netdev_ops = &denet_primary_netdev_ops;
    net_common->primary_dev->watchdog_timeo = TX_TIMEOUT;
    net_common->primary_dev->ethtool_ops = &primary_ethtool_ops;
    net_common->primary_dev->flags |= IFF_MULTICAST;
    net_common->primary_dev->features = 0;

    printk(KERN_INFO "%s: registering net device %s...\n", __func__, net_common->primary_dev->name);
#ifdef CONFIG_AKASHI_CONFIGURE_SWITCH
    /*
     * Within the container, IPCore utilizes a uint32 bitmap value to store the interface index.
     * However, there is a limitation: the maximum interface index (ifindex) must not exceed 32.
     * If the Akashi driver module is frequently unloaded and reloaded, the ifindex may surpass 32.
     * Thus, the goal here is to identify and reclaim unused interface indices.
     */
    net_common->primary_dev->ifindex = akashi_ifindex_get(net_common->primary_dev);
#endif
    r = register_netdev(net_common->primary_dev);
    if (r)
    {
        printk(KERN_ERR "%s: cannot register net device %s, aborting\n", __func__, net_common->primary_dev->name);
        return -ENODEV;
    }

    //printk(KERN_INFO "%s: setup IPv4 address...", __func__);

    /* little-endian from network layer */
    //net_local->self_ip = 0xE6AFFEA9; /* 169.254.175.230 */

    /* set ping ip */
    //denet_set_ipv4_addr(0, net_local->self_ip);
    printk(KERN_INFO "%s: register inet addr notifier\n", __func__);
    register_inetaddr_notifier(&akashi_nb);

    /* Clear RX bad packets register */
    clear_error_counter(net_common->primary_dev);

    if (register_char_devices(net_common->primary_dev))
    {
        printk(KERN_ERR "%s: error on register_char_devices!\n", __func__);
    }

    /* =============================================================
     * Begin initialisation of eth1 virtual interface
     */
    if (!(net_common->config.red_en & ETH_SWITCH_MODE_REDUNDANT))
    {
        /* Don't create eth1 if not redundant mode */
        goto init_exit;
    }

    net_common->secondary_dev = alloc_netdev(sizeof(net_local_t), SECONDARY_INTERFACE_NAME, NET_NAME_PREDICTABLE, ether_setup);
    if (!net_common->secondary_dev)
    {
        printk(KERN_ERR "Could not allocate %s device\n", SECONDARY_INTERFACE_NAME);
        return -ENOMEM;
    }

    /* This is our secondary virtual link eth1 */
    printk(KERN_INFO "%s: alloc net device '%s'\n", __func__, net_common->secondary_dev->name);

    /* Generic network setup */
    ether_setup(net_common->secondary_dev);

    /* Initialize our private data. */
    net_local = netdev_get_priv(net_common->secondary_dev);
    memset(net_local, 0, sizeof(net_local_t));
    net_local->index = 1; /* eth1 */
    net_local->dev = net_common->secondary_dev;

    /* Initialize pointer to common interface data. */
    net_local->common = net_common;

    net_common->secondary_dev->netdev_ops = &denet_secondary_netdev_ops;
    net_common->secondary_dev->watchdog_timeo = TX_TIMEOUT;
    net_common->secondary_dev->ethtool_ops = &secondary_ethtool_ops;
    net_common->secondary_dev->flags |= IFF_MULTICAST;
    net_common->secondary_dev->features = 0;

#ifdef CONFIG_AKASHI_PS_MAC
    /* In redundant mode of HC, GEM1 is used for eth1 */
    if (net_common->high_channel_count)
    {
        r = zynq_macb_init(pdev, net_common->secondary_dev);
        if (r)
        {
            printk(KERN_ERR "%s: failed to zynq_macb_init for net device %s, aborting\n", __func__, net_common->secondary_dev->name);
            return r;
        }
    }
#endif

    /* Configure MAC address for eth1 */
    akashi_set_hw_mac_addr(net_local);

    /* Init miscellaneous registers related to broadcast and port range */
    akashi_init_misc(net_common->secondary_dev);

    printk(KERN_INFO "%s: registering net device %s...\n", __func__, net_common->secondary_dev->name);
#ifdef CONFIG_AKASHI_CONFIGURE_SWITCH
    net_common->secondary_dev->ifindex = akashi_ifindex_get(net_common->secondary_dev);
#endif
    r = register_netdev(net_common->secondary_dev);
    if (r)
    {
        printk(KERN_ERR "%s: cannot register net device %s, aborting\n", __func__, net_common->secondary_dev->name);
        return -ENODEV;
    }

    /* Clear RX bad packets register */
    clear_error_counter(net_common->secondary_dev);

init_exit:
#ifdef CONFIG_AKASHI_DEBUG_TIMER
    /* Periodic debug function */
    printk(KERN_INFO "%s: starting debug timer\n", __func__);
    net_common->debug_timer.expires = jiffies + TIMER_DELAY_1SEC;
    timer_setup(&net_common->debug_timer, debug_timer_func, 0);
    add_timer(&net_common->debug_timer);
#endif

#ifdef CONFIG_PROC_FS
    akashi_create_proc_entries(net_common);
#endif

    return 0;
}

static int akashi_remove(struct platform_device *pdev)
{
    net_common_t *net_common = net_common_context;

#ifdef CONFIG_PROC_FS
    akashi_destroy_proc_entries(net_common);
#endif

#ifdef CONFIG_AKASHI_DEBUG_TIMER
    printk(KERN_INFO "%s: remove debug timer\n", __func__);
    del_timer(&net_common->debug_timer);
#endif

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
    printk(KERN_INFO "%s: remove check_link_status timer\n", __func__);
    del_timer(&net_common->check_link_st_timer);
#endif

    /* unregister notifier */
    printk(KERN_INFO "%s: unregister inet addr notifier\n", __func__);
    unregister_inetaddr_notifier(&akashi_nb);

    if (net_common->primary_dev)
    {
        /* Only eth0 has character interface */
        unregister_char_device(net_common->primary_dev);
        printk(KERN_INFO "%s: unregister net device '%s'\n", __func__, net_common->primary_dev->name);
        unregister_netdev(net_common->primary_dev);
    }

    if (net_common->secondary_dev)
    {
        printk(KERN_INFO "%s: unregister net device '%s'\n", __func__, net_common->secondary_dev->name);
        unregister_netdev(net_common->secondary_dev);
    }

    akashi_irq_cleanup(net_common);

#ifdef CONFIG_AKASHI_DEBUG_IOMAP
#ifdef CONFIG_AKASHI_64BIT_ARCH
    printk(KERN_INFO "%s: iounmap 0x%016llx\n", __func__, net_common->baseaddr);
#else
    printk(KERN_INFO "%s: iounmap 0x%08lx\n", __func__, net_common->baseaddr);
#endif
#endif
    iounmap((void *)net_common->baseaddr);

#ifdef CONFIG_AKASHI_64BIT_ARCH
    printk(KERN_INFO "%s: release mem region 0x%016llx\n", __func__, net_common->phyaddr);
#else
    printk(KERN_INFO "%s: release mem region 0x%08lx\n", __func__, net_common->phyaddr);
#endif
    release_mem_region(net_common->phyaddr, net_common->remap_size);

    if (net_common->primary_dev)
    {
#ifdef CONFIG_AKASHI_PS_MAC
        zynq_macb_cleanup(net_common->primary_dev);
#endif
        printk(KERN_INFO "%s: free net device '%s'\n", __func__, net_common->primary_dev->name);
        free_netdev(net_common->primary_dev);
        net_common->primary_dev = NULL;
    }

    if (net_common->secondary_dev)
    {
#ifdef CONFIG_AKASHI_PS_MAC
        zynq_macb_cleanup(net_common->secondary_dev);
#endif
        printk(KERN_INFO "%s: free net device '%s'\n", __func__, net_common->secondary_dev->name);
        free_netdev(net_common->secondary_dev);
        net_common->secondary_dev = NULL;
    }

    /* Remove and free all buffers inside the TX skbuff list */
    skb_queue_purge(&net_common->tx_skb_list);
    spin_lock_bh(&net_common->tx_priority_lock);
    skb_queue_purge(&net_common->tx_priority_skb_list);
    spin_unlock_bh(&net_common->tx_priority_lock);

    /* Free common structure */
    kfree(net_common);

    /* Be self organise and always cleanup */
    net_common_context = NULL;

    return 0;
}

static struct platform_driver akashi_platform_driver =
{
    .probe   = akashi_probe,
    .remove  = akashi_remove,
    .driver  =
    {
        .name = "Akashi",
        .of_match_table = akashi_of_match,
    },
};

//-------------------------------------------------------------------
/* Take kernel cmdline option macaddr=... and set MAC address */
static int __init denet_hw_addr_setup(net_common_t *net_common, char *addrs)
{
    unsigned int hw_addr[6];
    int count;

    /* Scan the kernel param for HW MAC address */
    count = sscanf(addrs, "%2x:%2x:%2x:%2x:%2x:%2x",
            hw_addr + 0,
            hw_addr + 1,
            hw_addr + 2,
            hw_addr + 3,
            hw_addr + 4,
            hw_addr + 5);
    /* Did we get 6 hex digits? */
    if (count != 6)
        return 0;

    for (count = 0; count < 6; count++)
    {
        net_common->base_mac_addr[count] = hw_addr[count] & 0xFF;
    }

    /* Increase interface number, for next time */
    return 1;
}

//-------------------------------------------------------------------
/* Take kernel cmdline option redundancy option */
static int __init denet_redundancy_setup(net_common_t *net_common, char *red)
{
    int count;
    akashi_driver_config_t *config = &net_common->config;

    count = sscanf(red, "%d:%d:%d:%d",
                    &config->red_en,
                    &config->vlan_pri,
                    &config->vlan_sec,
                    &config->vlan_config);

    if (count != 4)
    {
        printk("redundancy not configured\n");
        config->red_en = 0;
        config->vlan_pri = 0;
        config->vlan_sec = 0;
        config->vlan_config = 0;
    }

    return 1;
}

//-------------------------------------------------------------------
/* Take kernel cmdline option control port option */
static int __init denet_ctr_port_setup(net_common_t *net_common, char *ctr_port)
{
    int count;

    count = sscanf(ctr_port, "%x",&net_common->config.control_ports);

    if (count != 1)
    {
        net_common->config.control_ports = 0;
    }

    return 1;
}

//-------------------------------------------------------------------
/* Take kernel cmdline option control port option */
static int __init denet_ctr_port_info_setup(net_common_t *net_common, char *ctr_port_info)
{
    int count;

    count = sscanf(ctr_port_info, "%x",&net_common->config.control_ports_info);

    if (count != 1)
    {
        net_common->config.control_ports_info = 0;
    }

    return 1;
}

//-------------------------------------------------------------------
/* Take kernel cmdline option control port configuration */
static int __init denet_ctr_port_config_setup(net_common_t *net_common, char *ctr_port_config)
{
    int count;

    count = sscanf(ctr_port_config, "%x",&net_common->config.control_ports_config);

    if (count != 1)
    {
        net_common->config.control_ports_config = 0;
    }

    return 1;
}


//-------------------------------------------------------------------
/* Take kernel cmdline option redundancy option */
static int __init denet_external_phys_setup(net_common_t *net_common, char *ex_phys)
{
    int count;

    count = sscanf(ex_phys, "%x", &net_common->config.external_phys);

    if (count != 1)
    {
        printk("external phys not configured\n");
        net_common->config.external_phys = 0;
    }

    return 1;
}

//-------------------------------------------------------------------
/* Take kernel cmdline option redundancy option */
static int __init denet_multicast_filtering_setup(net_common_t *net_common, char *multicast_white_list)
{
    int maddr_count = 0;
    char *multicast_white_list_str, *maddr_str;
    uint32_t maddr;

    multicast_white_list_str = multicast_white_list;

    /*
     * find valid multicast addr through given string
     * Multicast string format = 0xE0010181 (224.1.1.129) with delimeter :
     */
    while ((maddr_str = strsep(&multicast_white_list_str, ":")) != NULL)
    {
        if (sscanf(maddr_str, "0x%x\n", &maddr ) == 1)
        {
            if ((maddr & 0xF0000000) == 0xE0000000)
            {
                net_common->config.multicast_addr_white_list[maddr_count++] = maddr;
                printk("Given multicast address - %x \n", maddr);

                if (maddr_count == MC_MAX_MULTICAST_ADDRESS_LIST)
                {
                    printk("Multicast whilte list is full and stop populating\n");
                    break;
                }
            }
        }
    }

    return 1;
}

//-------------------------------------------------------------------
// Init Function
static int __init akashi_init(void)
{
    int retval = 0;

    printk(KERN_INFO "%s, driver version %s\n", DRIVER_NAME, DRIVER_VERSION);
    printk(KERN_INFO "%s\n", DRIVER_AUTHOR);
    printk(KERN_INFO "Modified for Hyperport by DiGiCo UK Ltd <liam.pribis@digiconsoles.com> 2025\n");

    /* Register the platform driver */
    retval = platform_driver_register(&akashi_platform_driver);
    if (retval)
    {
        printk(KERN_ERR "Failed to register akashi platform driver\n");
    }

    return retval;
}

//-------------------------------------------------------------------
// Exit Function
static void __exit akashi_cleanup(void)
{
    /* Unregister the platform driver */
    platform_driver_unregister(&akashi_platform_driver);
}

EXPORT_SYMBOL(get_temac_vaddr);
EXPORT_SYMBOL(aud_syd_virtual_base_addr);
EXPORT_SYMBOL(network_adapter_info);

module_init(akashi_init);
module_exit(akashi_cleanup);
