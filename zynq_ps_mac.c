/*
 * Cadence MACB/GEM Ethernet Controller driver
 *
 * Copyright (C) 2004-2006 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include "zynq_ps_mac.h"
#include "akashi_temac.h"

#define MACB_NETIF_LSO		NETIF_F_TSO

/* This net device is used to do the mdio bus access */
static struct net_device	*smi_net_dev;

/* I/O accessors */
static u32 hw_readl_native(struct macb *bp, int offset)
{
	return __raw_readl(bp->regs + offset);
}

static void hw_writel_native(struct macb *bp, int offset, u32 value)
{
	__raw_writel(value, bp->regs + offset);
}

static u32 hw_readl(struct macb *bp, int offset)
{
	return readl_relaxed(bp->regs + offset);
}

static void hw_writel(struct macb *bp, int offset, u32 value)
{
	writel_relaxed(value, bp->regs + offset);
}

/* Find the CPU endianness by using the loopback bit of NCR register. When the
 * CPU is in big endian we need to program swapped mode for management
 * descriptor access.
 */
static bool hw_is_native_io(void __iomem *addr)
{
	u32 value = MACB_BIT(LLB);

	__raw_writel(value, addr + MACB_NCR);
	value = __raw_readl(addr + MACB_NCR);

	/* Write 0 back to disable everything */
	__raw_writel(0, addr + MACB_NCR);

	return value == MACB_BIT(LLB);
}

static bool hw_is_gem(void __iomem *addr, bool native_io)
{
	u32 id;

	if (native_io)
		id = __raw_readl(addr + MACB_MID);
	else
		id = readl_relaxed(addr + MACB_MID);

	return MACB_BFEXT(IDNUM, id) >= 0x2;
}

static void macb_set_hwaddr(struct macb *bp)
{
	u32 bottom;
	u16 top;

	bottom = cpu_to_le32(*((u32 *)bp->dev->dev_addr));
	macb_or_gem_writel(bp, SA1B, bottom);
	top = cpu_to_le16(*((u16 *)(bp->dev->dev_addr + 4)));
	macb_or_gem_writel(bp, SA1T, top);

	gem_writel(bp, RXPTPUNI, bottom);
	gem_writel(bp, TXPTPUNI, bottom);

	/* Clear unused address register sets */
	macb_or_gem_writel(bp, SA2B, 0);
	macb_or_gem_writel(bp, SA2T, 0);
	macb_or_gem_writel(bp, SA3B, 0);
	macb_or_gem_writel(bp, SA3T, 0);
	macb_or_gem_writel(bp, SA4B, 0);
	macb_or_gem_writel(bp, SA4T, 0);
}

static int macb_mdio_wait_for_idle(struct macb *bp)
{
	ulong timeout;

	timeout = jiffies + msecs_to_jiffies(1000);
	/* wait for end of transfer */
	while (1) {
		if (MACB_BFEXT(IDLE, macb_readl(bp, NSR)))
			break;

		if (time_after_eq(jiffies, timeout)) {
			netdev_err(bp->dev, "wait for end of transfer timed out\n");
			return -ETIMEDOUT;
		}

		cpu_relax();
	}

	return 0;
}

static int zynq_macb_smi_read(struct net_device *dev, int mii_id, int regnum)
{
	net_local_t *net_local = netdev_priv(dev);
	struct macb *bp = &(net_local->macb);
	int value;
	int err;

	err = macb_mdio_wait_for_idle(bp);
	if (err < 0)
		return err;

	macb_writel(bp, MAN, (MACB_BF(SOF, MACB_MAN_SOF)
			      | MACB_BF(RW, MACB_MAN_READ)
			      | MACB_BF(PHYA, mii_id)
			      | MACB_BF(REGA, regnum)
			      | MACB_BF(CODE, MACB_MAN_CODE)));

	err = macb_mdio_wait_for_idle(bp);
	if (err < 0)
		return err;

	value = MACB_BFEXT(DATA, macb_readl(bp, MAN));
	return value;
}

static int zynq_macb_smi_write(struct net_device *dev, int mii_id, int regnum,
			   u16 value)
{
	net_local_t *net_local = netdev_priv(dev);
	struct macb *bp = &(net_local->macb);
	int err;

	err = macb_mdio_wait_for_idle(bp);
	if (err < 0)
		return err;

	macb_writel(bp, MAN, (MACB_BF(SOF, MACB_MAN_SOF)
			      | MACB_BF(RW, MACB_MAN_WRITE)
			      | MACB_BF(PHYA, mii_id)
			      | MACB_BF(REGA, regnum)
			      | MACB_BF(CODE, MACB_MAN_CODE)
			      | MACB_BF(DATA, value)));

	err = macb_mdio_wait_for_idle(bp);
	if (err < 0)
		return err;

	return 0;
}

static void macb_reset_hw(struct macb *bp)
{
	struct macb_queue *queue;
	unsigned int q;
	u32 ctrl = macb_readl(bp, NCR);

	/* Disable RX and TX (XXX: Should we halt the transmission
	 * more gracefully?)
	 */
	ctrl &= ~(MACB_BIT(RE) | MACB_BIT(TE));

	/* Clear the stats registers (XXX: Update stats first?) */
	ctrl |= MACB_BIT(CLRSTAT);

	macb_writel(bp, NCR, ctrl);

	/* Clear all status flags */
	macb_writel(bp, TSR, -1);
	macb_writel(bp, RSR, -1);

	/* Disable RX partial store and forward and reset watermark value */
	if (bp->caps & MACB_CAPS_PARTIAL_STORE_FORWARD)
		gem_writel(bp, PBUFRXCUT, 0xFFF);

	/* Disable all interrupts */
	for (q = 0, queue = bp->queues; q < bp->num_queues; ++q, ++queue) {
		queue_writel(queue, IDR, -1);
		queue_readl(queue, ISR);
		if (bp->caps & MACB_CAPS_ISR_CLEAR_ON_WRITE)
			queue_writel(queue, ISR, -1);
	}
}

static u32 gem_mdc_clk_div(struct macb *bp)
{
	u32 config;
	unsigned long pclk_hz = clk_get_rate(bp->pclk);

	if (pclk_hz <= 20000000)
		config = GEM_BF(CLK, GEM_CLK_DIV8);
	else if (pclk_hz <= 40000000)
		config = GEM_BF(CLK, GEM_CLK_DIV16);
	else if (pclk_hz <= 80000000)
		config = GEM_BF(CLK, GEM_CLK_DIV32);
	else if (pclk_hz <= 120000000)
		config = GEM_BF(CLK, GEM_CLK_DIV48);
	else if (pclk_hz <= 160000000)
		config = GEM_BF(CLK, GEM_CLK_DIV64);
	else
		config = GEM_BF(CLK, GEM_CLK_DIV96);

	return config;
}

static u32 macb_mdc_clk_div(struct macb *bp)
{
	u32 config;
	unsigned long pclk_hz;

	if (macb_is_gem(bp))
		return gem_mdc_clk_div(bp);

	pclk_hz = clk_get_rate(bp->pclk);
	if (pclk_hz <= 20000000)
		config = MACB_BF(CLK, MACB_CLK_DIV8);
	else if (pclk_hz <= 40000000)
		config = MACB_BF(CLK, MACB_CLK_DIV16);
	else if (pclk_hz <= 80000000)
		config = MACB_BF(CLK, MACB_CLK_DIV32);
	else
		config = MACB_BF(CLK, MACB_CLK_DIV64);

	return config;
}

/* Get the DMA bus width field of the network configuration register that we
 * should program.  We find the width from decoding the design configuration
 * register to find the maximum supported data bus width.
 */
static u32 macb_dbw(struct macb *bp)
{
	if (!macb_is_gem(bp))
		return 0;

	switch (GEM_BFEXT(DBWDEF, gem_readl(bp, DCFG1))) {
	case 4:
		return GEM_BF(DBW, GEM_DBW128);
	case 2:
		return GEM_BF(DBW, GEM_DBW64);
	case 1:
	default:
		return GEM_BF(DBW, GEM_DBW32);
	}
}

static void macb_init_hw(struct macb *bp)
{
	u32 config;

	macb_reset_hw(bp);
	macb_set_hwaddr(bp);

	config = macb_mdc_clk_div(bp);
	if (bp->phy_interface == PHY_INTERFACE_MODE_SGMII)
		config |= GEM_BIT(SGMIIEN) | GEM_BIT(PCSSEL);
	config |= MACB_BF(RBOF, NET_IP_ALIGN);	/* Make eth data aligned */
	config |= MACB_BIT(PAE);		/* PAuse Enable */

	/* Do not discard Rx FCS if RX checsum offload disabled */
	if (bp->dev->features & NETIF_F_RXCSUM)
		config |= MACB_BIT(DRFCS);		/* Discard Rx FCS */

	if (bp->caps & MACB_CAPS_JUMBO)
		config |= MACB_BIT(JFRAME);	/* Enable jumbo frames */
	else
		config |= MACB_BIT(BIG);	/* Receive oversized frames */
	if (bp->dev->flags & IFF_PROMISC)
		config |= MACB_BIT(CAF);	/* Copy All Frames */
	else if (macb_is_gem(bp) && bp->dev->features & NETIF_F_RXCSUM)
		config |= GEM_BIT(RXCOEN);
	if (!(bp->dev->flags & IFF_BROADCAST))
		config |= MACB_BIT(NBC);	/* No BroadCast */

	/* need to be 0 in external fifo mode */
	config |= GEM_BF(DBW, GEM_DBW32);

	/* full duplex and gigabit mode */
	config |= (MACB_BIT(FD) | GEM_BIT(GBE));

	/* Copy All Frames to forward packets with Marvell header to Akashi */
	config |= MACB_BIT(CAF);

	macb_writel(bp, NCFGR, config);
	if ((bp->caps & MACB_CAPS_JUMBO) && bp->jumbo_max_len)
		gem_writel(bp, JML, bp->jumbo_max_len);

	/* Enable external fifo interface */
	gem_writel(bp, EFI, (gem_readl(bp, EFI) | GEM_BIT(ENEFI)));

	bp->speed = SPEED_10;
	if (bp->caps & MACB_CAPS_PARTIAL_STORE_FORWARD)
		bp->duplex = DUPLEX_FULL;
	else
		bp->duplex = DUPLEX_HALF;
	bp->rx_frm_len_mask = MACB_RX_FRMLEN_MASK;
	if (bp->caps & MACB_CAPS_JUMBO)
		bp->rx_frm_len_mask = MACB_RX_JFRMLEN_MASK;

	/* Enable RX partial store and forward and set watermark */
	if (bp->caps & MACB_CAPS_PARTIAL_STORE_FORWARD) {
		gem_writel(bp, PBUFRXCUT,
			   (gem_readl(bp, PBUFRXCUT) &
			   GEM_BF(WTRMRK, bp->rx_watermark)) |
			   GEM_BIT(ENCUTTHRU));
	}

	if ((bp->phy_interface == PHY_INTERFACE_MODE_SGMII) &&
	    (bp->caps & MACB_CAPS_PCS))
		gem_writel(bp, PCSCNTRL,
			   gem_readl(bp, PCSCNTRL) | GEM_BIT(PCSAUTONEG));

	/* Enable TX and RX */
	macb_writel(bp, NCR, macb_readl(bp, NCR) | MACB_BIT(RE) | MACB_BIT(TE) |
		    MACB_BIT(PTPUNI));
}

/* Configure peripheral capabilities according to device tree
 * and integration options used
 */
static void macb_configure_caps(struct macb *bp,
				const struct macb_config *dt_conf)
{
	u32 dcfg;
	int retval;

	if (dt_conf)
		bp->caps = dt_conf->caps;

	/* By default we set to partial store and forward mode for zynqmp.
	 * Disable if not set in devicetree.
	 */
	if (bp->caps & MACB_CAPS_PARTIAL_STORE_FORWARD) {
		retval = of_property_read_u16(bp->pdev->dev.of_node,
					      "rx-watermark",
					      &bp->rx_watermark);

		/* Disable partial store and forward in case of error or
		 * invalid watermark value
		 */
		if (retval || bp->rx_watermark > 0xFFF) {
			dev_info(&bp->pdev->dev,
				 "Not enabling partial store and forward\n");
			bp->caps &= ~MACB_CAPS_PARTIAL_STORE_FORWARD;
		}
	}

	if (hw_is_gem(bp->regs, bp->native_io)) {
		bp->caps |= MACB_CAPS_MACB_IS_GEM;

		dcfg = gem_readl(bp, DCFG1);
		if (GEM_BFEXT(IRQCOR, dcfg) == 0)
			bp->caps |= MACB_CAPS_ISR_CLEAR_ON_WRITE;
		dcfg = gem_readl(bp, DCFG2);
		if ((dcfg & (GEM_BIT(RX_PKT_BUFF) | GEM_BIT(TX_PKT_BUFF))) == 0)
			bp->caps |= MACB_CAPS_FIFO_MODE;
	}

	dev_dbg(&bp->pdev->dev, "Cadence caps 0x%08x\n", bp->caps);
}

static void macb_probe_queues(void __iomem *mem,
			      bool native_io,
			      unsigned int *queue_mask,
			      unsigned int *num_queues)
{
	unsigned int hw_q;

	*queue_mask = 0x1;
	*num_queues = 1;

	/* is it macb or gem ?
	 *
	 * We need to read directly from the hardware here because
	 * we are early in the probe process and don't have the
	 * MACB_CAPS_MACB_IS_GEM flag positioned
	 */
	if (!hw_is_gem(mem, native_io))
		return;

	/* bit 0 is never set but queue 0 always exists */
	*queue_mask = readl_relaxed(mem + GEM_DCFG6) & 0xff;

	*queue_mask |= 0x1;

	for (hw_q = 1; hw_q < MACB_MAX_QUEUES; ++hw_q)
		if (*queue_mask & (1 << hw_q))
			(*num_queues)++;
}

static char *macb_clk_table[][5] = {
	{"pclk0", "hclk0", "tx_clk0", "rx_clk0", "tsu_clk0"},
	{"pclk1", "hclk1", "tx_clk1", "rx_clk1", "tsu_clk1"}
};
#define MACB_CLK_TABLE_GROUP_NUM sizeof(macb_clk_table)/sizeof(macb_clk_table[0])

static int macb_clk_init(struct platform_device *pdev, int group, struct clk **pclk,
			 struct clk **hclk, struct clk **tx_clk,
			 struct clk **rx_clk, struct clk **tsu_clk)
{
	int err;

	if (group >= MACB_CLK_TABLE_GROUP_NUM) {
		err = -ENODEV;
		dev_err(&pdev->dev, "group(%d) exceeds the maximum number (%u)\n", group, err);
		return err;
	}
	*pclk = devm_clk_get(&pdev->dev, macb_clk_table[group][PCLK_INDEX]);
	if (IS_ERR_OR_NULL(*pclk)) {
		err = PTR_ERR(*pclk);
		if (!err)
			err = -ENODEV;

		dev_err(&pdev->dev, "failed to get macb_clk (%u)\n", err);
		return err;
	}

	*hclk = devm_clk_get(&pdev->dev, macb_clk_table[group][HCLK_INDEX]);
	if (IS_ERR_OR_NULL(*hclk)) {
		err = PTR_ERR(*hclk);
		if (!err)
			err = -ENODEV;

		dev_err(&pdev->dev, "failed to get hclk (%u)\n", err);
		return err;
	}

	*tx_clk = devm_clk_get(&pdev->dev, macb_clk_table[group][TXCLK_INDEX]);
	if (IS_ERR(*tx_clk))
		*tx_clk = NULL;

	*rx_clk = devm_clk_get(&pdev->dev, macb_clk_table[group][RXCLK_INDEX]);
	if (IS_ERR(*rx_clk))
		*rx_clk = NULL;

	*tsu_clk = devm_clk_get(&pdev->dev, macb_clk_table[group][TSUCLK_INDEX]);
	if (IS_ERR(*tsu_clk))
		*tsu_clk = NULL;

	err = clk_prepare_enable(*pclk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable pclk (%u)\n", err);
		return err;
	}

	err = clk_prepare_enable(*hclk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable hclk (%u)\n", err);
		goto err_disable_pclk;
	}

	err = clk_prepare_enable(*tx_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable tx_clk (%u)\n", err);
		goto err_disable_hclk;
	}

	err = clk_prepare_enable(*rx_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable rx_clk (%u)\n", err);
		goto err_disable_txclk;
	}

	err = clk_prepare_enable(*tsu_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable tsu_clk (%u)\n", err);
		goto err_disable_rxclk;
	}

	return 0;

err_disable_rxclk:
	clk_disable_unprepare(*rx_clk);

err_disable_txclk:
	clk_disable_unprepare(*tx_clk);

err_disable_hclk:
	clk_disable_unprepare(*hclk);

err_disable_pclk:
	clk_disable_unprepare(*pclk);

	return err;
}

static int macb_init(struct net_device *dev)
{
	unsigned int hw_q, q;
	net_local_t *net_local = netdev_priv(dev);
	struct macb *bp = &(net_local->macb);
	struct macb_queue *queue;
	u32 val, reg;

	/* set the queue register mapping once for all: queue0 has a special
	 * register mapping but we don't want to test the queue index then
	 * compute the corresponding register offset at run time.
	 */
	for (hw_q = 0, q = 0; hw_q < MACB_MAX_QUEUES; ++hw_q) {
		if (!(bp->queue_mask & (1 << hw_q)))
			continue;

		queue = &bp->queues[q];
		queue->bp = bp;
		if (hw_q) {
			queue->ISR  = GEM_ISR(hw_q - 1);
			queue->IER  = GEM_IER(hw_q - 1);
			queue->IDR  = GEM_IDR(hw_q - 1);
			queue->IMR  = GEM_IMR(hw_q - 1);
			queue->TBQP = GEM_TBQP(hw_q - 1);
			queue->RBQP = GEM_RBQP(hw_q - 1);
			queue->RBQS = GEM_RBQS(hw_q - 1);
		} else {
			/* queue0 uses legacy registers */
			queue->ISR  = MACB_ISR;
			queue->IER  = MACB_IER;
			queue->IDR  = MACB_IDR;
			queue->IMR  = MACB_IMR;
			queue->TBQP = MACB_TBQP;
			queue->RBQP = MACB_RBQP;
		}

		q++;
	}

	/* Set features */
	dev->hw_features = NETIF_F_SG;

	/* Check LSO capability */
	if (GEM_BFEXT(PBUF_LSO, gem_readl(bp, DCFG6)))
		dev->hw_features |= MACB_NETIF_LSO;

	/* Checksum offload is only available on gem with packet buffer */
	if (macb_is_gem(bp) && !(bp->caps & MACB_CAPS_FIFO_MODE))
		dev->hw_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	if (bp->caps & MACB_CAPS_PARTIAL_STORE_FORWARD)
		dev->hw_features &= ~NETIF_F_RXCSUM;
	if (bp->caps & MACB_CAPS_SG_DISABLED)
		dev->hw_features &= ~NETIF_F_SG;
	dev->features = dev->hw_features;

	/* Check RX Flow Filters support.
	 * Max Rx flows set by availability of screeners & compare regs:
	 * each 4-tuple define requires 1 T2 screener reg + 3 compare regs
	 */
	reg = gem_readl(bp, DCFG8);
	bp->max_tuples = min((GEM_BFEXT(SCR2CMP, reg) / 3),
			GEM_BFEXT(T2SCR, reg));
	if (bp->max_tuples > 0) {
		/* also needs one ethtype match to check IPv4 */
		if (GEM_BFEXT(SCR2ETH, reg) > 0) {
			/* program this reg now */
			reg = 0;
			reg = GEM_BFINS(ETHTCMP, (uint16_t)ETH_P_IP, reg);
			gem_writel_n(bp, ETHT, SCRT2_ETHT, reg);
			/* Filtering is supported in hw but don't enable it in kernel now */
			dev->hw_features |= NETIF_F_NTUPLE;
		} else
			bp->max_tuples = 0;
	}

	if (!(bp->caps & MACB_CAPS_USRIO_DISABLED)) {
		val = 0;
		if (bp->phy_interface == PHY_INTERFACE_MODE_RGMII)
			val = GEM_BIT(RGMII);
		else if (bp->phy_interface == PHY_INTERFACE_MODE_RMII &&
			 (bp->caps & MACB_CAPS_USRIO_DEFAULT_IS_MII_GMII))
			val = MACB_BIT(RMII);
		else if (!(bp->caps & MACB_CAPS_USRIO_DEFAULT_IS_MII_GMII))
			val = MACB_BIT(MII);

		if (bp->caps & MACB_CAPS_USRIO_HAS_CLKEN)
			val |= MACB_BIT(CLKEN);

		macb_or_gem_writel(bp, USRIO, val);
	}

	/* Set MII management clock divider */
	val = macb_mdc_clk_div(bp);
	val |= macb_dbw(bp);
	if (bp->phy_interface == PHY_INTERFACE_MODE_SGMII)
		val |= GEM_BIT(SGMIIEN) | GEM_BIT(PCSSEL);
	macb_writel(bp, NCFGR, val);

	if ((bp->phy_interface == PHY_INTERFACE_MODE_SGMII) &&
	    (bp->caps & MACB_CAPS_PCS))
		gem_writel(bp, PCSCNTRL,
			   gem_readl(bp, PCSCNTRL) | GEM_BIT(PCSAUTONEG));

	return 0;
}

#if defined(CONFIG_OF)
static const struct macb_config zynqmp_config = {
	.caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE |
			MACB_CAPS_JUMBO |
			MACB_CAPS_GEM_HAS_PTP | MACB_CAPS_BD_RD_PREFETCH |
			MACB_CAPS_PCS |
		MACB_CAPS_PARTIAL_STORE_FORWARD | MACB_CAPS_WOL,
	.dma_burst_length = 16,
	.clk_init = macb_clk_init,
	.init = macb_init,
	.jumbo_max_len = 10240,
};

static const struct macb_config zynq_config = {
	.caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE | MACB_CAPS_NO_GIGABIT_HALF |
		MACB_CAPS_NEEDS_RSTONUBR,
	.dma_burst_length = 16,
	.clk_init = macb_clk_init,
	.init = macb_init,
};

static const struct macb_config zynqmp_akashi_config = {
	.caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE |
			MACB_CAPS_JUMBO |
			MACB_CAPS_GEM_HAS_PTP | MACB_CAPS_BD_RD_PREFETCH |
			MACB_CAPS_PCS | MACB_CAPS_SG_DISABLED | MACB_CAPS_FIFO_MODE |
			MACB_CAPS_PARTIAL_STORE_FORWARD | MACB_CAPS_WOL,
	.dma_burst_length = 16,
	.clk_init = macb_clk_init,
	.init = macb_init,
	.jumbo_max_len = 10240,
};

static const struct of_device_id macb_dt_ids[] = {
	{ .compatible = "cdns,zynqmp-gem", .data = &zynqmp_config},
	{ .compatible = "cdns,zynq-gem", .data = &zynq_config },
	{ .compatible = "xlnx,akashi", .data = &zynqmp_akashi_config},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, macb_dt_ids);
#endif /* CONFIG_OF */

static const struct macb_config default_gem_config = {
	.caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE |
			MACB_CAPS_JUMBO |
			MACB_CAPS_GEM_HAS_PTP,
	.dma_burst_length = 16,
	.clk_init = macb_clk_init,
	.init = macb_init,
	.jumbo_max_len = 10240,
};

static void akashi_ps_smi_read(uint8_t mii_addr, uint8_t reg_addr, uint16_t *reg_data)
{
    *reg_data = zynq_macb_smi_read(smi_net_dev, mii_addr, reg_addr);
}

static void akashi_ps_smi_write(uint8_t mii_addr, uint8_t reg_addr, uint16_t reg_data)
{
    zynq_macb_smi_write(smi_net_dev, mii_addr, reg_addr, reg_data);
}

int zynq_macb_init(struct platform_device *pdev, struct net_device *dev)
{
	const struct macb_config *macb_config = &zynqmp_config;
	int (*clk_init)(struct platform_device *, int group, struct clk **,
			struct clk **, struct clk **,  struct clk **,
			struct clk **) = macb_config->clk_init;
	int (*init)(struct net_device *) = macb_config->init;
	struct device_node *np = pdev->dev.of_node;
	struct clk *pclk, *hclk = NULL, *tx_clk = NULL, *rx_clk = NULL;
	struct clk *tsu_clk = NULL;
	unsigned int queue_mask, num_queues;
	bool native_io;
	struct resource *regs;
	void __iomem *mem;
	struct macb *bp;
	int err;
	net_local_t *net_local = netdev_priv(dev);

	/*
	 * GEM0 belongs to eth0 and uses the second device resource in DT
	 * GEM1 belongs to eth1 and uses the third device resource in DT
	 */
	int mem_id = net_local->index+1;

	/* 1 means the second range in reg */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, mem_id);
	mem = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(mem))
		return PTR_ERR(mem);

	if (np) {
		const struct of_device_id *match;

		match = of_match_node(macb_dt_ids, np);
		if (match && match->data) {
			macb_config = match->data;
			clk_init = macb_config->clk_init;
			init = macb_config->init;
		}
	}

	err = clk_init(pdev, net_local->index, &pclk, &hclk, &tx_clk, &rx_clk, &tsu_clk);
	if (err)
		return err;

	native_io = hw_is_native_io(mem);

	macb_probe_queues(mem, native_io, &queue_mask, &num_queues);

	dev->base_addr = regs->start;

	bp = &(net_local->macb);
	bp->pdev = pdev;
	bp->dev = dev;
	bp->regs = mem;
	bp->native_io = native_io;
	if (native_io) {
		bp->macb_reg_readl = hw_readl_native;
		bp->macb_reg_writel = hw_writel_native;
	} else {
		bp->macb_reg_readl = hw_readl;
		bp->macb_reg_writel = hw_writel;
	}
	bp->num_queues = num_queues;
	bp->queue_mask = queue_mask;
	if (macb_config)
		bp->dma_burst_length = macb_config->dma_burst_length;
	bp->pclk = pclk;
	bp->hclk = hclk;
	bp->tx_clk = tx_clk;
	bp->rx_clk = rx_clk;
	bp->tsu_clk = tsu_clk;
	if (tsu_clk)
		bp->tsu_rate = clk_get_rate(tsu_clk);

	if (macb_config)
		bp->jumbo_max_len = macb_config->jumbo_max_len;

	/* setup capabilities */
	macb_configure_caps(bp, macb_config);

	platform_set_drvdata(pdev, dev);

	/* IP specific init */
	err = init(dev);
	if (err)
		goto err_out;

	/* Enable management port */
	macb_writel(bp, NCR, MACB_BIT(MPE));

	macb_init_hw(bp);

	/* Register mdio read/write functions for GEM0, used to access switch chip */
	if (net_local->index == 0)
	{
		smi_net_dev = dev;
		sl_lib_smi_handler_register(akashi_ps_smi_read, akashi_ps_smi_write);
	}

	return 0;

err_out:
	return err;
}

#ifdef CONFIG_AKASHI_EMAC_0_SMI_IRQ
static irqreturn_t macb_ext_interrupt(int irq, void *dev_id)
{
	net_common_t *net_common = (net_common_t *)dev_id;
	net_local_t *net_local = netdev_priv(net_common->primary_dev);
	struct macb *bp = &(net_local->macb);
	u32 status = macb_readl(bp, ISR);

	/* clear interrupt status register */
	macb_writel(bp, ISR, status);
	tasklet_schedule(&net_common->smi_tasklet);
    return IRQ_HANDLED;
}

int zynq_macb_ext_irq_init(struct net_device *dev)
{
	net_local_t *net_local = netdev_priv(dev);
	struct macb *bp = &(net_local->macb);
	net_common_t *net_common = net_local->common;
	int result = 0;

	/* Disable all interrupts */
	macb_writel(bp, IDR, -1);

	/* Switch SMI interrupt line connects with GEM0 external interrupt of the PS MAC */
	result = devm_request_irq(&bp->pdev->dev, net_common->smi_irq, &macb_ext_interrupt, net_common->irq_flags, "gem0-ext", net_common);
	if (result)
    {
        printk(KERN_ERR "%s: could not allocate interrupt %d\n", __func__, net_common->smi_irq);
        return result;
    }

	/* Disable all interrupts except external irq */
	macb_writel(bp, IDR, ~MACB_BIT(EXTINT));
	/* Enable external irq */
	macb_writel(bp, IER, MACB_BIT(EXTINT));

	return result;
}
#endif

void zynq_macb_cleanup(struct net_device *dev)
{
	net_local_t *net_local = netdev_priv(dev);
	struct macb *bp = &(net_local->macb);

	clk_disable_unprepare(bp->tx_clk);
	clk_disable_unprepare(bp->hclk);
	clk_disable_unprepare(bp->pclk);
	clk_disable_unprepare(bp->rx_clk);
	clk_disable_unprepare(bp->tsu_clk);
}
