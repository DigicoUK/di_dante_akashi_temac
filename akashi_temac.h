/*
 * File : akashi_temac.h
 * Created : April 2022
 * Authors : Martin Siu
 * Synopsis: Akashi Register Interface device driver head file.
 *
 * Copyright 2022 Audinate Pty Ltd and/or its licensors
 *
 */

#ifndef AKASHI_TEMAC_H
#define AKASHI_TEMAC_H

#include "switch_lib_reg.h"
#ifdef CONFIG_AKASHI_PS_MAC
#include "zynq_ps_mac.h"
#endif

#define AUD_SYD_SCHED_BASEADDR                                  (0x24000)
#define AUD_SYD_PROTO_BASEADDR                                  (0x4000)

#define CRC_POLYNOMIAL_BE                                       (0x04c11db7UL)

#define AUD_SYD_PLATFORM_NAME                                   (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0))
#define AUD_SYD_FPGA_VERSION                                    (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x4))
#define AUD_SYD_INFO_TXC                                        (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x10))
#define AUD_SYD_INFO_RXC                                        (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x14))

#define AUD_SYD_FPGA_VERSION_MAJOR                              ((AUD_SYD_FPGA_VERSION & 0xff000000) >> 24)
#define AUD_SYD_FPGA_VERSION_MINOR                              ((AUD_SYD_FPGA_VERSION & 0x00ff0000) >> 16)
#define AUD_SYD_FPGA_VERSION_PATCH                              ((AUD_SYD_FPGA_VERSION & 0x0000ff00) >> 8)
#define AUD_SYD_FPGA_VERSION_RC                                 ((AUD_SYD_FPGA_VERSION & 0xff))
#define AUD_SYD_SCHED_RX_FIFO_ADDR                              ((volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x800))
#define AUD_SYD_SCHED_TX_FIFO_ADDR                              ((volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x400))
#define AUD_SYD_SCHED_TX_PRIORITY_FIFO_ADDR                     ((volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x1800))

#define AUD_SYD_INFO_CAPABILITY                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x2c))
#define AUD_SYD__CAPABILITY_RGMII_12mA                          (1<<0)
#define AUD_SYD__CAPABILITY_BK2_JETPLL                          (1<<1)
#define AUD_SYD__CAPABILITY_1722_AVB                            (1<<2)
#define AUD_SYD__CAPABILITY_AES                                 (1<<3)
#define AUD_SYD__CAPABILITY_EXT_LATENCY                         (1<<4)
#define AUD_SYD__CAPABILITY_TSCOUNT                             (1<<5)
#define AUD_SYD__CAPABILITY_BK1_JETPLL                          (1<<8)
#define AUD_SYD__CAPABILITY_PEAVEY                              (1<<9)
#define AUD_SYD__CAPABILITY_BK2_RAMLESS                         (1<<10)
#define AUD_SYD__CAPABILITY_VIDEO_TRANSMIT                      (1<<11)
#define AUD_SYD__CAPABILITY_VIDEO_RECEIVE                       (1<<12)
#define AUD_SYD__CAPABILITY_PRIORITY_TX_QUEUEE                  (1<<13)

//----------------------------------------------                -------------------------------------------
#define AUD_SYD_MAC_CONTROL                                     (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0800)) //0: 1=125Mhz 0=25Mhz
#define AUD_SYD_MAC_CONTROL_125MHZ                              (1<<0)
#define AUD_SYD_MAC_CONTROL_25MHZ                               (0<<0)
#define AUD_SYD_MAC_CONTROL_RGMII                               (1<<1)
#define AUD_SYD_MAC_CONTROL_MII                                 (0<<1)
#define AUD_SYD_MAC_CONTROL_GTX_CLK                             (1<<2)                                         //0:Switch, 1:PHY
#define AUD_SYD_MAC_SWITCH_CONTROL                              (1<<3)
#define AUD_SYD_MAC_REDUNDANT_TYPE                              (1<<4)                                         //0:normal, 1:redundancy


#define AUD_SYD_MAC_TX0_BYTES                                   (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0804))
#define AUD_SYD_MAC_RX0_GOOD_BYTES                              (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0808))
#define AUD_SYD_MAC_RX0_BAD_PACKETS                             (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x080c))
#define AUD_SYD_MAC_TX1_BYTES                                   (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0844))
#define AUD_SYD_MAC_RX1_GOOD_BYTES                              (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0848))
#define AUD_SYD_MAC_RX1_BAD_PACKETS                             (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x084c))

#define AUD_SYD_MAC_TOD_0                                       (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0810))
#define AUD_SYD_MAC_TOD_1                                       (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0814))
#define AUD_SYD_MAC_TOD_2                                       (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0818))
#define AUD_SYD_MAC_RX_SW_EN                                    (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x081c))
#define AUD_SYD_MAC_RX_SWITCH_0                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0820))
#define AUD_SYD_MAC_RX_SWITCH_1                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0824))
#define AUD_SYD_MAC_RX_SWITCH_2                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0828))
#define AUD_SYD_MAC_RX_SWITCH_3                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x082c))
#define AUD_SYD_MAC_RX_SWITCH_4                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0830))
#define AUD_SYD_MAC_RX_SWITCH_5                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0834))
#define AUD_SYD_MAC_TX_SWITCH_0                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0838))
#define AUD_SYD_MAC_TX_SWITCH_1                                 (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x083c))

//----------------------------------------------                -------------------------------------------
#define AUD_SYD_PROTO_MAC_ADDR0_HI                              (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1200))
#define AUD_SYD_PROTO_MAC_ADDR0_ME                              (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1204))
#define AUD_SYD_PROTO_MAC_ADDR0_LO                              (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1208))
#define AUD_SYD_PROTO_MAC_ADDR1_HI                              (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1600))
#define AUD_SYD_PROTO_MAC_ADDR1_ME                              (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1604))
#define AUD_SYD_PROTO_MAC_ADDR1_LO                              (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1608))

#define AUD_SYD_PROTO_IP_ADDR0_HI                               (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x120c))
#define AUD_SYD_PROTO_IP_ADDR0_LO                               (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1210))
#define AUD_SYD_PROTO_IP_ADDR1_HI                               (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x160c))
#define AUD_SYD_PROTO_IP_ADDR1_LO                               (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1610))

#define AUD_SYD_PROTO_AUDIO_PORT_LO                             (0x3800) // Audio destination port range
#define AUD_SYD_PROTO_AUDIO_PORT_HI                             (0x39FF)
#define AUD_SYD_PROTO_KA_PORT_LO                                (0xF000) // Audio source port range base - this becomes the KA destination port
#define AUD_SYD_PROTO_KA_PORT_HI                                (0xF0FF)

#define AUD_SYD_PROTO_PHY0_AUDIO_RANGE_HI                       (*(volatile uint32_t *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1220))
#define AUD_SYD_PROTO_PHY0_AUDIO_RANGE_LO                       (*(volatile uint32_t *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1224))
#define AUD_SYD_PROTO_PHY0_KA_RANGE_HI                          (*(volatile uint32_t *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1228))
#define AUD_SYD_PROTO_PHY0_KA_RANGE_LO                          (*(volatile uint32_t *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x122c))
#define AUD_SYD_PROTO_PHY1_AUDIO_RANGE_HI                       (*(volatile uint32_t *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1620))
#define AUD_SYD_PROTO_PHY1_AUDIO_RANGE_LO                       (*(volatile uint32_t *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1624))
#define AUD_SYD_PROTO_PHY1_KA_RANGE_HI                          (*(volatile uint32_t *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1628))
#define AUD_SYD_PROTO_PHY1_KA_RANGE_LO                          (*(volatile uint32_t *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x162c))

//----------------------------------------------                -------------------------------------------
#define AUD_SYD_PROTO_PHY0_REJECT(port)                         (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1240 + port*4))
#define AUD_SYD_PROTO_PHY1_REJECT(port)                         (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1640 + port*4))

//----------------------------------------------                -------------------------------------------
#define AUD_SYD_PROTO_HASH0(reg_idx)                            (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1280 + (4 * reg_idx)))
#define AUD_SYD_PROTO_HASH1(reg_idx)                            (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_PROTO_BASEADDR + 0x1680 + (4 * reg_idx)))
#define AUD_SYD_PROTO_HASH_BITS                                 (256)
#define AUD_SYD_PROTO_HASH_REG_COUNT                            (AUD_SYD_PROTO_HASH_BITS / 16)
#define AUD_SYD_PROTO_HASH_FILTERS                              (16)

//----------------------------------------------                -------------------------------------------
#define AUD_SYD_SCHED_TX_FIFO_BASEADDR                          (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x400))
#define AUD_SYD_SCHED_RX_FIFO_BASEADDR                          (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x800))
#define AUD_SYD_SCHED_TX_PRIORITY_FIFO_BASEADDR                 (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x1800))
//-----------------------------------------------------------------------------------------
#define AUD_SYD_SCHED_INTR_CONTROL                              (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x0))

#define AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_RST                 (1<<0)
#define AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_INTR_EN             (1<<1)
#define AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_OVER_EN             (1<<2)
#define AUD_SYD_SCHED_INTR_CONTROL__TX_FIFO_UNDER_EN            (1<<3)
#define AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_RST                 (1<<4)
#define AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_INTR_EN             (1<<5)
#define AUD_SYD_SCHED_INTR_CONTROL__RX_FIFO_OVER_EN             (1<<6)
#define AUD_SYD_SCHED_FLOW_CONTROL__RX_FLOW_SET_INT             (1<<8)
#define AUD_SYD_SCHED_FLOW_CONTROL__RX_FLOW_RESET_INT           (1<<9)
#define AUD_SYD_SCHED_FLOW_CONTROL__KAMISS_SET_INT              (1<<10)
#define AUD_SYD_SCHED_FLOW_CONTROL__KAMISS_RESET_INT            (1<<11)
#define AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_RST           (1<<12)
#define AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_INT_EN        (1<<13)
#define AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_OVERFLOW_EN   (1<<14)
#define AUD_SYD_SCHED_INTR_CONTROL__PRIORITY_FIFO_UNDERFLOW_EN  (1<<15)

//-----------------------------------------------------------------------------------------
#define AUD_SYD_SCHED_CONTROL                                   (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x4))
#define AUD_SYD_SCHED_CONTROL__RXEN                             (1<<0)
#define AUD_SYD_SCHED_CONTROL__PRI_TXEN                         (1<<1)
#define AUD_SYD_SCHED_CONTROL__SEC_TXEN                         (1<<2)
#define AUD_SYD_SCHED_CONTROL__REDEN                            (1<<3)
#define AUD_SYD_SCHED_CONTROL__AVB_EN                           (1<<4)
#define AUD_SYD_SCHED_CONTROL__AES67_EN                         (1<<5)
#define AUD_SYD_SCHED_CONTROL__PTPV1_DEL_REQ_EN                 (1<<6)
#define AUD_SYD_SCHED_CONTROL__PTPV2_DEL_REQ_EN                 (1<<7)
#define AUD_SYD_SCHED_CONTROL__VIDEO_ENABLE                     (1<<8)

//-----------------------------------------------------------------------------------------
#define AUD_SYD_SCHED_TXFIFO__TS_REQ_TIME                       (1<<31)
#define AUD_SYD_SCHED_TXFIFO__TS_REQ_PRIM                       (0<<30)
#define AUD_SYD_SCHED_TXFIFO__TS_REQ_SEC                        (1<<30)
#define AUD_SYD_SCHED_TXFIFO__TS_REQ_ID_SHIFT                   (16)
//-----------------------------------------------------------------------------------------
#define AUD_SYD_SCHED_TX_FIFO_RELEASE                           (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x8))
                                                                // 0: Release packet
                                                                // 1: Timestamp packet
                                                                // 3:2 Destination PHY
#define AUD_SYD_SCHED_TX_FIFO__RELEASE_PKT                      (1<<0)
#define AUD_SYD_SCHED_TX_FIFO__TIMESTAMP_PKT                    (1<<1)
#define AUD_SYD_SCHED_TX_FIFO__DEST_PHY0(reg)                   (reg &= ~(1<<2))
#define AUD_SYD_SCHED_TX_FIFO__DEST_PHY1(reg)                   (reg |= (1<<2))
#define AUD_SYD_SCHED_TX_FIFO__RELEASE_PRIORITY_PKT             (1<<4)

//-----------------------------------------------------------------------------------------
#define AUD_SYD_SCHED_FIFO_OCC                                  (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0xc))
                                                                // 15:0 Tx FIFO Free (32 bit words)
                                                                // 16   Rx FIFO not empty flag
#define AUD_SYD_SCHED_FIFO_OCC__TX_FREE_MASK                    (0xFFFF)
#define AUD_SYD_SCHED_FIFO_OCC__RX_NOT_EMPTY_MASK               (0x10000)

#define AUD_SYD_SCHED_PRIORITY_FIFO_FREE_SPACE                  (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x3c)) // 15:0, priority fifo space available
//-----------------------------------------------------------------------------------------
#define AUD_SYD_SCHED_INTR_STATUS                               (*(volatile u32 *)(AUD_SYD_BASEADDR + AUD_SYD_SCHED_BASEADDR + 0x10))

#define AUD_SYD_SCHED_INTR_STATUS__TX_FIFO                      (1<<0)
#define AUD_SYD_SCHED_INTR_STATUS__RX_FIFO                      (1<<1)
#define AUD_SYD_SCHED_INTR_STATUS__TX_FIFO_OVERFLOW             (1<<2)
#define AUD_SYD_SCHED_INTR_STATUS__TX_FIFO_UNDERFLOW            (1<<3)
#define AUD_SYD_SCHED_INTR_STATUS__RX_FIFO_OVERFLOW             (1<<4)
#define AUD_SYD_SCHED_INTR_STATUS__RX_FLOW                      (1<<5)
#define AUD_SYD_SCHED_INTR_STATUS__KAMISS                       (1<<6)
#define AUD_SYD_SCHED_INTR_STATUS__PRIORITY_DATA                (1<<8)
#define AUD_SYD_SCHED_INTR_STATUS__PRIORITY_FIFO_OVERFLOW       (1<<9)
#define AUD_SYD_SCHED_INTR_STATUS__PRIORITY_FIFO_UNDERFLOW      (1<<10)
//-----------------------------------------------------------------------------------------
// PTP Tx timestamps
#define AUD_SYD_MAC_TX_TS_NUM_ENTRIES_SHIFT                     (24)

#define AUD_SYD_MAC_TX_SEC_HI                                   (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0810)) // 15:0 tod_sec(46:32) // 31: PHY
#define AUD_SYD_MAC_TX_SEC_LO                                   (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0814)) // 31:0 tod_sec(31:0)
#define AUD_SYD_MAC_TX_NSEC                                     (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0818)) // 31:0 tod_nsec(31:0)
#define AUD_SYD_MAC_TX_TS_ID                                    (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x081c)) // 25:24 num of entries, 7:0 ts id

#define AUD_SYD_MAC_0_TX_COUNTER_HI                             (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0868)) // 15:0 counter(46:32)
#define AUD_SYD_MAC_0_TX_COUNTER_LO                             (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x086c)) // 31:0 counter(31:0)
//PHY 1
#define AUD_SYD_MAC_1_TX_COUNTER_HI                             (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0870)) // 15:0 counter(46:32)
#define AUD_SYD_MAC_1_TX_COUNTER_LO                             (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0874)) // 31:0 counter(31:0)

//--------------------------------------------------------------------------------
#define AUD_SYD_PROTOCOL_STACK_INTERFACE_PHYS                   (*(volatile u32 *)(AUD_SYD_BASEADDR + 0x0c80)) // 15:0 tod_sec(47:32)

//--------------------------------------------------------------------------------

#define AUD_SYD_AUDIO_MCAST_IDENT                               (0x7f) // audio mcast range is 239.255.X.X (01.00.5e.7f.X.X)

#define TX_TIMEOUT                                              (10*HZ)    /* Transmission timeout is 60 seconds. */

#define ALIGNMENT                                               (8)
/* BUFFER_ALIGN(adr) calculates the number of bytes to the next alignment. */
#ifdef CONFIG_AKASHI_64BIT_ARCH
#define BUFFER_ALIGN(adr)                                       ((ALIGNMENT - ((u64) adr)) % ALIGNMENT)
#else
#define BUFFER_ALIGN(adr)                                       ((ALIGNMENT - ((u32) adr)) % ALIGNMENT)
#endif
#define ARRAY_LEN(a)                                            (sizeof(a) / sizeof(a[0]))

#define MAX_PHY_CHIPSETS                                        (7)

#define PRIMARY_INTERFACE_NAME                                  "eth0"
#define SECONDARY_INTERFACE_NAME                                "eth1"

#define TXTS_QUEUE_SIZE                                         (42)
#define EMAC_REGS_N                                             (32)

#define REDUNDANCY_INTERFACE_NONE                               (0)
#define REDUNDANCY_INTERFACE_PORT_0                             (2)
#define REDUNDANCY_INTERFACE_PORT_1                             (3)

#define AKASHI_DEBUG_BUF_SIZE                                   (4096)

#define TX_SKB_LIST_WAKE_THRESHOLD                              (10)
#define TX_SKB_LIST_STOP_THRESHOLD                              (200)

/* --------------------------------------------------------------------------
 * ports/link status
 * --------------------------------------------------------------------------*/
typedef struct interface_stat_s
{
    int                     link_duplex;
    int                     link_status;
    int                     link_speed;
} interface_stat_t;

typedef struct portstat
{
    uint16_t                status_reg[MAX_PHY_CHIPSETS];
    uint8_t                 changed;
    uint8_t                 num_ports;
} portstat_t;

typedef enum txts_status
{
    TXTS_STATUS_READY       = 0,    //not used at all
    TXTS_STATUS_FD_ASSIGNED = 1,    //socket fd assigned
    TXTS_STATUS_TX_INT      = 2     //sent packet and interupput recv
} txts_status_t;

/*
 * Network adapter information for pri/sec
 */
typedef struct dante_network_st_s
{
    portstat_t              ports;                                  // Only for switch interface to pass info to app level.
    portstat_t              backup_ports;                           // Backup port status info to avoid race condition.
    interface_stat_t        interface_stat[2];                      // Pri/Sec information.
    network_adapter_t       net_chip_info;                          // structure of network adapter including phy/switch
    uint16_t                ext_phy_int_clear_reg[MAX_PHY_CHIPSETS];// register number for external phy interrupt clearing & default is marvel phy reg.
} dante_network_st_t;

typedef struct akashi_timespec
{                       // The standard struct timespec provided by Linux kernel has tv_sec and tv_nsec variables of type long.
    int32_t tv_sec;     // On some 64-bit platforms, sizeof(long) = 8. This causes issues for PTP syncing.
    int32_t tv_nsec;    // We require a 32-bit tv_sec and tv_nsec, as provided here.
} akashi_timespec_t;

typedef struct timestamp_info
{
    akashi_timespec_t         tx_timestamp;
    akashi_timespec_t         tx_timestamp_monotonic;
    int                     iface_index;
    int                     fd;
    u32                     dest_ipv4;
    u16                     dest_port;
    u16                     msg_type;       // used only when 802.1as is used;; this field acts as 16bit padding for normal layer3 message
    u16                     ptp_version;    //useful when using unicast messages. unicast messages use shared path handler.
    u16                     padding;
} timestamp_info_t;

typedef struct
{
    timestamp_info_t        txts[TXTS_QUEUE_SIZE];
    txts_status_t           txts_status[TXTS_QUEUE_SIZE];
    u16                     ids[TXTS_QUEUE_SIZE];
    int                     head_to_send;   // write pt before sending
    int                     head_sent;      // write pt after sending
    int                     tail;           // read pt
} txts_queue_t;

typedef struct akashi_driver_config
{
    uint32_t                control_ports;
    uint32_t                control_ports_info;
    uint32_t                control_ports_config;
    uint32_t                red_en;
    uint32_t                vlan_pri;
    uint32_t                vlan_sec;
    uint32_t                vlan_config;
    uint32_t                external_phys;
    uint32_t                multicast_addr_white_list[MC_MAX_MULTICAST_ADDRESS_LIST];
} akashi_driver_config_t;

#ifdef CONFIG_PROC_FS
typedef struct akashi_proc
{
    struct proc_dir_entry   *root;
    struct proc_dir_entry   *mac_addr;
    struct proc_dir_entry   *red_flag;
    struct proc_dir_entry   *ctr_port;
    struct proc_dir_entry   *ex_phys;
    struct proc_dir_entry   *debug;
    struct proc_dir_entry   *mcast;
    struct proc_dir_entry   *mac_filter;
} akashi_proc_t;
#endif

typedef struct akashi_debug
{
    uint32_t                flag;
    char                    buf[AKASHI_DEBUG_BUF_SIZE];
    char                    data_buf[AKASHI_DEBUG_BUF_SIZE];
    uint8_t                 mac_filter[6];
} akashi_debug_t;

/* Common data for all interfaces */
typedef struct net_common
{
    akashi_driver_config_t  config;

#ifdef CONFIG_AKASHI_64BIT_ARCH
    unsigned long long           phyaddr;
    unsigned long long           baseaddr;
    unsigned long long           remap_size;
#else
    unsigned long           phyaddr;
    unsigned long           baseaddr;
    unsigned long           remap_size;
#endif
    /*
     * This flag is for manual override of the IRQ trigger type.
     * The default trigger type is defined in the device tree.
     * Please configure the irq trigger type in the device tree accordingly
     * and leave this 'irq_flags' field at zero during driver initialization.
     */
    u32                     irq_flags;

    /* Additional IRQs */
    int                     smi_irq;
    int                     rx_irq;
    int                     tx_irq;
    int                     err_irq;
    int                     ts_irq;

    struct sk_buff_head     tx_priority_skb_list;
    struct sk_buff_head     tx_skb_list;
    spinlock_t              tx_priority_lock;
    spinlock_t              tx_lock;

    unsigned char           has_priority_queue;

    struct tasklet_struct   send_tasklet;
    struct tasklet_struct   priority_send_tasklet;
    struct tasklet_struct   recv_tasklet;
    struct tasklet_struct   smi_tasklet;

    /* eth0 primiary physical link */
    struct net_device       *primary_dev;

    /* eth1 secondary redundant link */
    struct net_device       *secondary_dev;

    /* Global pointer to base MAC address */
    uint8_t                 base_mac_addr[6];

    /* Debug Timer */
    struct timer_list       debug_timer;
    struct timer_list       tx_drain_timer;

    /* Network switch or phy info */
    dante_network_st_t      dante_net_st;

    /* Buffer for returnning info to user space */
    char                    buf_to_send[TXTS_QUEUE_SIZE * sizeof(timestamp_info_t)];

    /* TX timestamps queue */
    txts_queue_t            txts_queue;

    /* LED identify timer */
    struct timer_list       identify_timer;
    int                     identify_led_activity;

    /* For exclusion of all program flows (processes, ISRs and BHs) possible to share data with current one */
    spinlock_t              reset_lock;

    /* Tx Timestamp handling */
    wait_queue_head_t       tsqueue;

    /* Port stats handling */
    wait_queue_head_t       psqueue;
    wait_queue_head_t       psqueue1;
    int                     ps_pending;
    int                     ps_pending1;

    /* Is TSCOUNT info available? */
    int                     has_tscount;
    uint32_t                max_rx_len;
    uint32_t                rx_meta_buf[6];

    /* A boolean flag to indicate HC or DDR FPGA version */
    int                     high_channel_count;

#ifndef CONFIG_AKASHI_EMAC_0_SMI_IRQ
    struct timer_list       check_link_st_timer;
    bool                    check_link_first_time;
#endif

#ifdef CONFIG_PROC_FS
    akashi_proc_t           proc;
#endif

    akashi_debug_t          debug;
} net_common_t;

/* --------------------------------------------------------------------------
 * definition of the "private" data structure used by this interface
 * --------------------------------------------------------------------------*/
typedef struct net_local
{
    net_common_t *common;

    /* Statistics for this device */
    struct net_device_stats stats;

    /* this device */
    struct net_device       *dev;

    /* PHY monitoring timer */
    struct timer_list       phy_timer;

    /* Which interface is this */
    u32                     index;

    /* The MII address of the PHY */
    u8                      mii_addr;

    /* Mac Speed */
    int                     mac_speed;

    dev_t                   denet_devt;
    struct device *         denet_devfs;
    dev_t                   denet_ps_devt;
    struct device *         denet_ps_devfs;
    dev_t                   denet_portstat_devt;
    struct device *         denet_portstat_devfs;

    /* Current mcast hash register state */
    u32                     mcast_hash[AUD_SYD_PROTO_HASH_FILTERS];

    /* MAC address of the local interface */
    u8                      mac_addr[6];

    /* For character device interface on primiary interface */
    struct class            *denet_class;

    /* Interface up? */
    bool                    up;

    /* our character device major number */
    int                     dev_major;

    /* network interface link status, a shaddow value of interface_stat[i].link_status */
    u8                      link_status;

#ifdef CONFIG_AKASHI_PS_MAC
    /* MACB data structure for ZYNQ PS MAC */
    struct macb             macb;
#endif
} net_local_t;

struct linkutil
{
    u32                     tx_bytes_sec;
    u32                     rx_bytes_sec;
};

/*
 * External phy port number (start from 0 to 6 except 5) and phy interrupt clear reg number
 */
typedef struct ext_phy_int_reg_s
{
    unsigned char           phy_port_index;        // 0 ~ 6 except 5.
    unsigned char           phy_int_status_reg;    // Phy interrupt clear status register
} ext_phy_int_reg_t;

enum
{
    IDENTIFY_LED_NONE,
    IDENTIFY_LED_FORCE_ON,
    IDENTIFY_LED_FORCE_OFF,
    IDENTIFY_LED_DEFAULT
};

#endif
