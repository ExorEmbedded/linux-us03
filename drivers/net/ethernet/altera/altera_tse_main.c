/* Altera Triple-Speed Ethernet over PCIe MAC driver (Exor S.p.a. FPGA design implementation)
 * Copyright (C) 2017 Exor S.p.a.
 * 
 * Written by: Giovanni Pavoni, Exor S.p.a.
 * 
 * Based on:
 * Altera Triple-Speed Ethernet MAC driver
 * Copyright (C) 2008-2014 Altera Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <asm/cacheflush.h>

#include "altera_utils.h"
#include "altera_tse.h"
#include "altera_msgdma.h"

#define TSE_TX_DESC_BASE     (0x400)
#define TSE_RX_DESC_BASE     (0x440)
#define TSE_DESCRIPTORS_SIZE (0x20)
#define TSE_REGISTERS_BASE   (0x000)
#define TSE_SGDMA_TX_BASE    (0x420)
#define TSE_SGDMA_RX_BASE    (0x460)
#define TSE_RESP_BASE        (0x480)

//On PCI-e DMA we need to align data chunks on 2KB boundaries
#define DMAALIGNSIZE         2048

void* iobase;
static atomic_t instance_count = ATOMIC_INIT(~0);
/* Module parameters */
static int debug = -1;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
					NETIF_MSG_LINK | NETIF_MSG_IFUP |
					NETIF_MSG_IFDOWN);

#define RX_DESCRIPTORS 16
static int dma_rx_num = RX_DESCRIPTORS;
module_param(dma_rx_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_rx_num, "Number of descriptors in the RX list");

#define TX_DESCRIPTORS 16
static int dma_tx_num = TX_DESCRIPTORS;
module_param(dma_tx_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_tx_num, "Number of descriptors in the TX list");

#define POLL_PHY (-1)

/* Make sure DMA buffer size is larger than the max frame size
 * plus some alignment offset and a VLAN header. If the max frame size is
 * 1518, a VLAN header would be additional 4 bytes and additional
 * headroom for alignment is 2 bytes, 2048 is just fine.
 */
#define ALTERA_RXDMABUFFER_SIZE	2048

/* Allow network stack to resume queueing packets after we've
 * finished transmitting at least 1/4 of the packets in the queue.
 */
#define TSE_TX_THRESH(x)	(x->tx_ring_size / 4)

#define TXQUEUESTOP_THRESHHOLD	2

static void skb_align(struct sk_buff *skb, int align)
{
	int off = ((unsigned long)skb->data) & (align - 1);

	if (off)
		skb_reserve(skb, align - off);
}

static struct sk_buff* netdev_alloc_skb_2K_align(struct net_device *dev, unsigned int len)
{
  struct sk_buff *new_skb;
  
  /* Alloc new skb */
  new_skb = netdev_alloc_skb(dev, len + DMAALIGNSIZE);
  if (!new_skb)
    return NULL;
  
  /* Make sure new skb is properly aligned */
  skb_align(new_skb, DMAALIGNSIZE);
  
  return new_skb;
}

/*
 * Our PCI-e DMA requeries 2Kbyte alignment for TX data buffer!
 */
static struct sk_buff* tx_skb_align_workaround(struct net_device *dev, struct sk_buff *skb)
{
	struct sk_buff *new_skb;

	/* Alloc new skb */
	new_skb = netdev_alloc_skb_2K_align(dev, skb->len);
	if (!new_skb)
		return NULL;

	/* Copy data to new skb ... */
	skb_copy_from_linear_data(skb, new_skb->data, skb->len);
	skb_put(new_skb, skb->len);

	/* ... and free an old one */
	dev_kfree_skb_any(skb);

	return new_skb;
}

static const struct of_device_id altera_tse_ids[];

static inline u32 tse_tx_avail(struct altera_tse_private *priv)
{
	return priv->tx_cons + priv->tx_ring_size - priv->tx_prod - 1;
}

/* MDIO specific functions
 */
static int altera_tse_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct net_device *ndev = bus->priv;
	struct altera_tse_private *priv = netdev_priv(ndev);
	int ret;
	
	spin_lock(&priv->global_lock);	

	/* set MDIO address */
	csrwr32((mii_id & 0x1f), priv->mac_dev,
		tse_csroffs(mdio_phy1_addr));

	/* get the data */
	ret = csrrd32(priv->mac_dev,
		       tse_csroffs(mdio_phy1) + regnum * 4) & 0xffff;

	spin_unlock(&priv->global_lock);
	return ret;
}

static int altera_tse_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
				 u16 value)
{
	struct net_device *ndev = bus->priv;
	struct altera_tse_private *priv = netdev_priv(ndev);

	spin_lock(&priv->global_lock);	
	/* set MDIO address */
	csrwr32((mii_id & 0x1f), priv->mac_dev,
		tse_csroffs(mdio_phy1_addr));

	/* write the data */
	csrwr32(value, priv->mac_dev, tse_csroffs(mdio_phy1) + regnum * 4);
	spin_unlock(&priv->global_lock);
	return 0;
}

static int altera_tse_mdio_create(struct net_device *dev, unsigned int id)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int ret;
	int i;
	struct mii_bus *mdio = NULL;
	mdio = mdiobus_alloc();
	if (mdio == NULL) {
		netdev_err(dev, "Error allocating MDIO bus\n");
		return -ENOMEM;
	}

	mdio->name = ALTERA_TSE_RESOURCE_NAME;
	mdio->read = &altera_tse_mdio_read;
	mdio->write = &altera_tse_mdio_write;
	snprintf(mdio->id, MII_BUS_ID_SIZE, "%s-%u", mdio->name, id);

	mdio->irq = kcalloc(PHY_MAX_ADDR, sizeof(int), GFP_KERNEL);
	if (mdio->irq == NULL) {
		ret = -ENOMEM;
		goto out_free_mdio;
	}
	for (i = 0; i < PHY_MAX_ADDR; i++)
		mdio->irq[i] = PHY_POLL;

	mdio->priv = dev;
	mdio->parent = priv->device;
	ret = mdiobus_register(mdio);

	if (ret != 0) {
		netdev_err(dev, "Cannot register MDIO bus %s\n",
			   mdio->id);
		goto out_free_mdio_irq;
	}

	if (netif_msg_drv(priv))
		netdev_info(dev, "MDIO bus %s: created\n", mdio->id);

	priv->mdio = mdio;
	return 0;
out_free_mdio_irq:
	kfree(mdio->irq);
out_free_mdio:
	mdiobus_free(mdio);
	mdio = NULL;
	return ret;
}

static void altera_tse_mdio_destroy(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);

	if (priv->mdio == NULL)
		return;

	if (netif_msg_drv(priv))
		netdev_info(dev, "MDIO bus %s: removed\n",
			    priv->mdio->id);

	mdiobus_unregister(priv->mdio);
	kfree(priv->mdio->irq);
	mdiobus_free(priv->mdio);
	priv->mdio = NULL;
}

static int tse_init_rx_buffer(struct altera_tse_private *priv,
			      struct tse_buffer *rxbuffer, int len)
{
	rxbuffer->skb = netdev_alloc_skb_2K_align(priv->dev, len);
	if (!rxbuffer->skb)
		return -ENOMEM;
	
	rxbuffer->dma_addr = dma_map_single(priv->device, rxbuffer->skb->data,
						len,
						DMA_FROM_DEVICE);

	if (dma_mapping_error(priv->device, rxbuffer->dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(rxbuffer->skb);
		return -EINVAL;
	}
	rxbuffer->len = len;
	return 0;
}

static void tse_free_rx_buffer(struct altera_tse_private *priv,
			       struct tse_buffer *rxbuffer)
{
	struct sk_buff *skb = rxbuffer->skb;
	dma_addr_t dma_addr = rxbuffer->dma_addr;

	if (skb != NULL) {
		if (dma_addr)
			dma_unmap_single(priv->device, dma_addr,
					 rxbuffer->len,
					 DMA_FROM_DEVICE);
		dev_kfree_skb_any(skb);
		rxbuffer->skb = NULL;
		rxbuffer->dma_addr = 0;
	}
}

/* Unmap and free Tx buffer resources
 */
static void tse_free_tx_buffer(struct altera_tse_private *priv,
			       struct tse_buffer *buffer)
{
	if (buffer->dma_addr) {
		dma_unmap_single(priv->device, buffer->dma_addr, buffer->len, DMA_TO_DEVICE);
		buffer->dma_addr = 0;
	}
	if (buffer->skb) {
		dev_kfree_skb_any(buffer->skb);
		buffer->skb = NULL;
	}
}

static int alloc_init_skbufs(struct altera_tse_private *priv)
{
	int ret = -ENOMEM;
	int i;
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	/* Create Rx ring buffer */
	priv->rx_ring = kcalloc(rx_descs, sizeof(struct tse_buffer),
				GFP_KERNEL);
	if (!priv->rx_ring)
		goto err_rx_ring;

	/* Create Tx ring buffer */
	priv->tx_ring = kcalloc(tx_descs, sizeof(struct tse_buffer),
				GFP_KERNEL);
	if (!priv->tx_ring)
		goto err_tx_ring;

	priv->tx_cons = 0;
	priv->tx_prod = 0;

	/* Init Rx ring */
	for (i = 0; i < rx_descs; i++) {
		ret = tse_init_rx_buffer(priv, &priv->rx_ring[i],
					 priv->rx_dma_buf_sz);
		if (ret)
			goto err_init_rx_buffers;
	}

	priv->rx_cons = 0;
	priv->rx_prod = 0;

	return 0;
err_init_rx_buffers:
	while (--i >= 0)
		tse_free_rx_buffer(priv, &priv->rx_ring[i]);
	kfree(priv->tx_ring);
err_tx_ring:
	kfree(priv->rx_ring);
err_rx_ring:
	return ret;
}

static void free_skbufs(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int i;

	/* Release the DMA TX/RX socket buffers */
	for (i = 0; i < rx_descs; i++)
		tse_free_rx_buffer(priv, &priv->rx_ring[i]);
	for (i = 0; i < tx_descs; i++)
		tse_free_tx_buffer(priv, &priv->tx_ring[i]);


	kfree(priv->tx_ring);
}

/* Reallocate the skb for the reception process
 */
static inline void tse_rx_refill(struct altera_tse_private *priv)
{
	unsigned int rxsize = priv->rx_ring_size;
	unsigned int entry;
	int ret;

	for (; priv->rx_cons - priv->rx_prod > 0;
			priv->rx_prod++) {
		entry = priv->rx_prod % rxsize;
		if (likely(priv->rx_ring[entry].skb == NULL)) {
			ret = tse_init_rx_buffer(priv, &priv->rx_ring[entry],
				priv->rx_dma_buf_sz);
			if (unlikely(ret != 0))
				break;
			priv->dmaops->add_rx_desc(priv, &priv->rx_ring[entry]);
		}
	}
}

/* Pull out the VLAN tag and fix up the packet
 */
static inline void tse_rx_vlan(struct net_device *dev, struct sk_buff *skb)
{
	struct ethhdr *eth_hdr;
	u16 vid;
	if ((dev->features & NETIF_F_HW_VLAN_CTAG_RX) &&
	    !__vlan_get_tag(skb, &vid)) {
		eth_hdr = (struct ethhdr *)skb->data;
		memmove(skb->data + VLAN_HLEN, eth_hdr, ETH_ALEN * 2);
		skb_pull(skb, VLAN_HLEN);
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vid);
	}
}

/* Receive a packet: retrieve and pass over to upper levels
 */
static int tse_rx(struct altera_tse_private *priv, int limit)
{
	unsigned int count = 0;
	unsigned int next_entry;
	struct sk_buff *skb;
	unsigned int entry = priv->rx_cons % priv->rx_ring_size;
	u32 rxstatus;
	u16 pktlength;
	u16 pktstatus;
	
	/* Check for count < limit first as get_rx_status is changing
	* the response-fifo so we must process the next packet
	* after calling get_rx_status if a response is pending.
	* (reading the last byte of the response pops the value from the fifo.)
	*/
	while ((count < limit) &&
	       ((rxstatus = priv->dmaops->get_rx_status(priv)) != 0)) {
		pktstatus = rxstatus >> 16;
		pktlength = rxstatus & 0xffff;

		if ((pktstatus & 0xFF) || (pktlength == 0))
			netdev_err(priv->dev,
				   "RCV pktstatus %08X pktlength %08X\n",
				   pktstatus, pktlength);

		count++;
		next_entry = (++priv->rx_cons) % priv->rx_ring_size;

		skb = priv->rx_ring[entry].skb;
		if (unlikely(!skb)) {
			netdev_err(priv->dev,
				   "%s: Inconsistent Rx descriptor chain\n",
				   __func__);
			priv->dev->stats.rx_dropped++;
			break;
		}
		priv->rx_ring[entry].skb = NULL;

		skb_put(skb, pktlength);

		/* make cache consistent with receive packet buffer */
		dma_sync_single_for_cpu(priv->device,
					priv->rx_ring[entry].dma_addr,
					priv->rx_ring[entry].len,
					DMA_FROM_DEVICE);

		dma_unmap_single(priv->device, priv->rx_ring[entry].dma_addr,
				 priv->rx_ring[entry].len, DMA_FROM_DEVICE);
#if 0
		if (netif_msg_pktdata(priv)) {
			netdev_info(priv->dev, "frame received %d bytes\n",
				    pktlength);
			print_hex_dump(1, "data: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, pktlength, true);
		}
#endif
		tse_rx_vlan(priv->dev, skb);

		skb->protocol = eth_type_trans(skb, priv->dev);
		skb_checksum_none_assert(skb);

		napi_gro_receive(&priv->napi, skb);

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += pktlength;

		entry = next_entry;
		spin_lock(&priv->global_lock);
		tse_rx_refill(priv);
		spin_unlock(&priv->global_lock);
	}

	return count;
}

/* Reclaim resources after transmission completes
 */
static int tse_tx_complete(struct altera_tse_private *priv)
{
	unsigned int txsize = priv->tx_ring_size;
	u32 ready;
	unsigned int entry;
	struct tse_buffer *tx_buff;
	int txcomplete = 0;

	spin_lock(&priv->global_lock);

	ready = priv->dmaops->tx_completions(priv);

	/* Free sent buffers */
	while (ready && (priv->tx_cons != priv->tx_prod)) {
		entry = priv->tx_cons % txsize;
		tx_buff = &priv->tx_ring[entry];

		if (netif_msg_tx_done(priv))
			netdev_dbg(priv->dev, "%s: curr %d, dirty %d\n",
				   __func__, priv->tx_prod, priv->tx_cons);

		if (likely(tx_buff->skb))
			priv->dev->stats.tx_packets++;

		tse_free_tx_buffer(priv, tx_buff);
		priv->tx_cons++;

		txcomplete++;
		ready--;
	}

	if (unlikely(netif_queue_stopped(priv->dev) &&
		     tse_tx_avail(priv) > TSE_TX_THRESH(priv))) {
		netif_tx_lock(priv->dev);
		if (netif_queue_stopped(priv->dev) &&
		    tse_tx_avail(priv) > TSE_TX_THRESH(priv)) {
			if (netif_msg_tx_done(priv))
				netdev_dbg(priv->dev, "%s: restart transmit\n",
					   __func__);
			netif_wake_queue(priv->dev);
		}
		netif_tx_unlock(priv->dev);
	}

	spin_unlock(&priv->global_lock);
	return txcomplete;
}

/* NAPI polling function
 */
static int tse_poll(struct napi_struct *napi, int budget)
{
	struct altera_tse_private *priv =
			container_of(napi, struct altera_tse_private, napi);
	int rxcomplete = 0;
	unsigned long int flags;

	tse_tx_complete(priv);

	rxcomplete = tse_rx(priv, budget);

	if (rxcomplete < budget) {

		napi_gro_flush(napi, false);
		__napi_complete(napi);

		netdev_dbg(priv->dev,
			   "NAPI Complete, did %d packets with budget %d\n",
			   rxcomplete, budget);

		spin_lock_irqsave(&priv->global_lock, flags);
		priv->dmaops->enable_rxirq(priv);
		priv->dmaops->enable_txirq(priv);
		spin_unlock_irqrestore(&priv->global_lock, flags);
	}
	return rxcomplete;
}

/* DMA TX & RX FIFO interrupt routing
 */
static irqreturn_t altera_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct altera_tse_private *priv;

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	priv = netdev_priv(dev);

	spin_lock(&priv->global_lock);
	/* reset IRQs */
	priv->dmaops->clear_rxirq(priv);
	priv->dmaops->clear_txirq(priv);
	spin_unlock(&priv->global_lock);

	if (likely(napi_schedule_prep(&priv->napi))) {
		spin_lock(&priv->global_lock);
		priv->dmaops->disable_rxirq(priv);
		priv->dmaops->disable_txirq(priv);
		spin_unlock(&priv->global_lock);
		__napi_schedule(&priv->napi);
	}


	return IRQ_HANDLED;
}

/* Transmit a packet (called by the kernel). Dispatches
 * either the SGDMA method for transmitting or the
 * MSGDMA method, assumes no scatter/gather support,
 * implying an assumption that there's only one
 * physically contiguous fragment starting at
 * skb->data, for length of skb_headlen(skb).
 */
static int tse_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	unsigned int txsize = priv->tx_ring_size;
	unsigned int entry;
	struct tse_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int nopaged_len = skb_headlen(skb);
	enum netdev_tx ret = NETDEV_TX_OK;
	dma_addr_t dma_addr;

	spin_lock_bh(&priv->global_lock);

	if (unlikely(tse_tx_avail(priv) < nfrags + 1)) {
		if (!netif_queue_stopped(dev)) {
			netif_stop_queue(dev);
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx list full when queue awake\n",
				   __func__);
		}
		ret = NETDEV_TX_BUSY;
		goto out;
	}

	/* Map the first skb fragment */
	entry = priv->tx_prod % txsize;
	buffer = &priv->tx_ring[entry];

	skb = tx_skb_align_workaround(dev, skb);
	if(!skb)
	{
	  ret = NETDEV_TX_BUSY;
	  goto out;
	}
	
	dma_addr = dma_map_single(priv->device, skb->data, nopaged_len, DMA_TO_DEVICE); 
	
	if (dma_mapping_error(priv->device, dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		ret = NETDEV_TX_OK;
		goto out;
	}

	buffer->skb = skb;
	buffer->dma_addr = dma_addr;
	buffer->len = nopaged_len;

	/* Push data out of the cache hierarchy into main memory */
	dma_sync_single_for_device(priv->device, buffer->dma_addr,
				   buffer->len, DMA_TO_DEVICE);

	priv->dmaops->tx_buffer(priv, buffer);

	skb_tx_timestamp(skb);

	priv->tx_prod++;
	dev->stats.tx_bytes += skb->len;

	if (unlikely(tse_tx_avail(priv) <= TXQUEUESTOP_THRESHHOLD)) {
		if (netif_msg_hw(priv))
			netdev_dbg(priv->dev, "%s: stop transmitted packets\n",
				   __func__);
		netif_stop_queue(dev);
	}

out:
	spin_unlock_bh(&priv->global_lock);

	return ret;
}

/* Called every time the controller might need to be made
 * aware of new link state.  The PHY code conveys this
 * information through variables in the phydev structure, and this
 * function converts those variables into the appropriate
 * register values, and can bring down the device if needed.
 */
static void altera_tse_adjust_link(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	struct phy_device *phydev = priv->phydev;
	int new_state = 0;

	/* only change config if there is a link */
	spin_lock(&priv->global_lock);
	if (phydev->link) {
		/* Read old config */
		u32 cfg_reg = ioread32(&priv->mac_dev->command_config);

		/* Check duplex */
		if (phydev->duplex != priv->oldduplex) {
			new_state = 1;
			if (!(phydev->duplex))
				cfg_reg |= MAC_CMDCFG_HD_ENA;
			else
				cfg_reg &= ~MAC_CMDCFG_HD_ENA;

			netdev_dbg(priv->dev, "%s: Link duplex = 0x%x\n",
				   dev->name, phydev->duplex);

			priv->oldduplex = phydev->duplex;
		}

		/* Check speed */
		if (phydev->speed != priv->oldspeed) {
			new_state = 1;
			switch (phydev->speed) {
			case 1000:
				cfg_reg |= MAC_CMDCFG_ETH_SPEED;
				cfg_reg &= ~MAC_CMDCFG_ENA_10;
				break;
			case 100:
				cfg_reg &= ~MAC_CMDCFG_ETH_SPEED;
				cfg_reg &= ~MAC_CMDCFG_ENA_10;
				break;
			case 10:
				cfg_reg &= ~MAC_CMDCFG_ETH_SPEED;
				cfg_reg |= MAC_CMDCFG_ENA_10;
				break;
			default:
				if (netif_msg_link(priv))
					netdev_warn(dev, "Speed (%d) is not 10/100/1000!\n",
						    phydev->speed);
				break;
			}
			priv->oldspeed = phydev->speed;
		}
		iowrite32(cfg_reg, &priv->mac_dev->command_config);

		if (!priv->oldlink) {
			new_state = 1;
			priv->oldlink = 1;
		}
	} else if (priv->oldlink) {
		new_state = 1;
		priv->oldlink = 0;
		priv->oldspeed = 0;
		priv->oldduplex = -1;
	}

	if (new_state && netif_msg_link(priv))
		phy_print_status(phydev);

	spin_unlock(&priv->global_lock);
}

static struct phy_device *connect_local_phy(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	char phy_id_fmt[MII_BUS_ID_SIZE + 3];

	if (priv->phy_addr != POLL_PHY) {
		snprintf(phy_id_fmt, MII_BUS_ID_SIZE + 3, PHY_ID_FMT,
			 priv->mdio->id, priv->phy_addr);

		netdev_dbg(dev, "trying to attach to %s\n", phy_id_fmt);

		phydev = phy_connect(dev, phy_id_fmt, &altera_tse_adjust_link,
				     priv->phy_iface);
		if (IS_ERR(phydev))
			netdev_err(dev, "Could not attach to PHY\n");

	} else {
		int ret;
		phydev = phy_find_first(priv->mdio);
		if (phydev == NULL) {
			netdev_err(dev, "No PHY found\n");
			return phydev;
		}

		ret = phy_connect_direct(dev, phydev, &altera_tse_adjust_link,
				priv->phy_iface);
		if (ret != 0) {
			netdev_err(dev, "Could not attach to PHY\n");
			phydev = NULL;
		}
	}
	return phydev;
}

/* Initialize driver's PHY state, and attach to the PHY
 */
static int init_phy(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	struct phy_device *phydev;

	/* Avoid init phy in case of no phy present */
	if (!priv->phy_iface)
		return 0;

	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;
	
	phydev = connect_local_phy(dev);
	if (!phydev) {
		netdev_err(dev, "Could not find the PHY\n");
		return -ENODEV;
	}

	/* Stop Advertising 1000BASE Capability if interface is not GMII
	 */
	if ((priv->phy_iface == PHY_INTERFACE_MODE_MII) ||
	    (priv->phy_iface == PHY_INTERFACE_MODE_RMII))
		phydev->advertising &= ~(SUPPORTED_1000baseT_Half |
					 SUPPORTED_1000baseT_Full);

	netdev_dbg(dev, "attached to PHY %d UID 0x%08x Link = %d\n", phydev->addr, phydev->phy_id, phydev->link);

	priv->phydev = phydev;
	return 0;
}

static void tse_update_mac_addr(struct altera_tse_private *priv, u8 *addr)
{
	u32 msb;
	u32 lsb;

	msb = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	lsb = ((addr[5] << 8) | addr[4]) & 0xffff;

	/* Set primary MAC address */
	csrwr32(msb, priv->mac_dev, tse_csroffs(mac_addr_0));
	csrwr32(lsb, priv->mac_dev, tse_csroffs(mac_addr_1));
}

/* MAC software reset.
 * When reset is triggered, the MAC function completes the current
 * transmission or reception, and subsequently disables the transmit and
 * receive logic, flushes the receive FIFO buffer, and resets the statistics
 * counters.
 */
static int reset_mac(struct altera_tse_private *priv)
{
	int counter;
	u32 dat;

	dat = csrrd32(priv->mac_dev, tse_csroffs(command_config));
	dat &= ~(MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA);
	dat |= MAC_CMDCFG_SW_RESET | MAC_CMDCFG_CNT_RESET;
	csrwr32(dat, priv->mac_dev, tse_csroffs(command_config));

	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		if (tse_bit_is_clear(priv->mac_dev, tse_csroffs(command_config),
				     MAC_CMDCFG_SW_RESET))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		dat = csrrd32(priv->mac_dev, tse_csroffs(command_config));
		dat &= ~MAC_CMDCFG_SW_RESET;
		csrwr32(dat, priv->mac_dev, tse_csroffs(command_config));
		return -1;
	}
	return 0;
}

/* Initialize MAC core registers
*/
static int init_mac(struct altera_tse_private *priv)
{
	unsigned int cmd = 0;
	u32 frm_length;

	/* Setup Rx FIFO */
	csrwr32(priv->rx_fifo_depth - ALTERA_TSE_RX_SECTION_EMPTY,
		priv->mac_dev, tse_csroffs(rx_section_empty));

	csrwr32(ALTERA_TSE_RX_SECTION_FULL, priv->mac_dev,
		tse_csroffs(rx_section_full));

	csrwr32(ALTERA_TSE_RX_ALMOST_EMPTY, priv->mac_dev,
		tse_csroffs(rx_almost_empty));

	csrwr32(ALTERA_TSE_RX_ALMOST_FULL, priv->mac_dev,
		tse_csroffs(rx_almost_full));

	/* Setup Tx FIFO */
	csrwr32(priv->tx_fifo_depth - ALTERA_TSE_TX_SECTION_EMPTY,
		priv->mac_dev, tse_csroffs(tx_section_empty));

	csrwr32(ALTERA_TSE_TX_SECTION_FULL, priv->mac_dev,
		tse_csroffs(tx_section_full));

	csrwr32(ALTERA_TSE_TX_ALMOST_EMPTY, priv->mac_dev,
		tse_csroffs(tx_almost_empty));

	csrwr32(ALTERA_TSE_TX_ALMOST_FULL, priv->mac_dev,
		tse_csroffs(tx_almost_full));

	/* MAC Address Configuration */
	tse_update_mac_addr(priv, priv->dev->dev_addr);

	/* MAC Function Configuration */
	frm_length = ETH_HLEN + priv->dev->mtu + ETH_FCS_LEN;
	csrwr32(frm_length, priv->mac_dev, tse_csroffs(frm_length));

	csrwr32(ALTERA_TSE_TX_IPG_LENGTH, priv->mac_dev,
		tse_csroffs(tx_ipg_length));

	/* Disable RX/TX shift 16 for alignment of all received frames on 16-bit
	 * start address
	 */
	tse_set_bit(priv->mac_dev, tse_csroffs(rx_cmd_stat),
		    ALTERA_TSE_RX_CMD_STAT_RX_SHIFT16);

	tse_clear_bit(priv->mac_dev, tse_csroffs(tx_cmd_stat),
		      ALTERA_TSE_TX_CMD_STAT_TX_SHIFT16 |
		      ALTERA_TSE_TX_CMD_STAT_OMIT_CRC);

	/* Set the MAC options */
	cmd = csrrd32(priv->mac_dev, tse_csroffs(command_config));
	cmd &= ~MAC_CMDCFG_PAD_EN;	/* No padding Removal on Receive */
	cmd &= ~MAC_CMDCFG_CRC_FWD;	/* CRC Removal */
	cmd |= MAC_CMDCFG_RX_ERR_DISC;	/* Automatically discard frames
					 * with CRC errors
					 */
	cmd |= MAC_CMDCFG_CNTL_FRM_ENA;
	cmd &= ~MAC_CMDCFG_TX_ENA;
	cmd &= ~MAC_CMDCFG_RX_ENA;

	/* Default speed and duplex setting, full/100 */
	cmd &= ~MAC_CMDCFG_HD_ENA;
	cmd &= ~MAC_CMDCFG_ETH_SPEED;
	cmd &= ~MAC_CMDCFG_ENA_10;

	csrwr32(cmd, priv->mac_dev, tse_csroffs(command_config));

	csrwr32(ALTERA_TSE_PAUSE_QUANTA, priv->mac_dev,
		tse_csroffs(pause_quanta));

	if (netif_msg_hw(priv))
		dev_dbg(priv->device,
			"MAC post-initialization: CMD_CONFIG = 0x%08x\n", cmd);

	return 0;
}

/* Start/stop MAC transmission logic
 */
static void tse_set_mac(struct altera_tse_private *priv, bool enable)
{
	u32 value = csrrd32(priv->mac_dev, tse_csroffs(command_config));

	if (enable)
		value |= MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA;
	else
		value &= ~(MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA);

	csrwr32(value, priv->mac_dev, tse_csroffs(command_config));
	
	printk("tse_set_mac enable=0x%x reg_value=0x%x\n",enable,value);
	
	value=0x12345678;
	csrwr32(value, priv->mac_dev, tse_csroffs(scratch_pad));
}

/* Change the MTU
 */
static int tse_change_mtu(struct net_device *dev, int new_mtu)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	unsigned int max_mtu = priv->max_mtu;
	unsigned int min_mtu = ETH_ZLEN + ETH_FCS_LEN;

	if (netif_running(dev)) {
		netdev_err(dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}

	if ((new_mtu < min_mtu) || (new_mtu > max_mtu)) {
		netdev_err(dev, "invalid MTU, max MTU is: %u\n", max_mtu);
		return -EINVAL;
	}

	dev->mtu = new_mtu;
	netdev_update_features(dev);

	return 0;
}

static void altera_tse_set_mcfilter(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int i;
	struct netdev_hw_addr *ha;

	/* clear the hash filter */
	for (i = 0; i < 64; i++)
		csrwr32(0, priv->mac_dev, tse_csroffs(hash_table) + i * 4);

	netdev_for_each_mc_addr(ha, dev) {
		unsigned int hash = 0;
		int mac_octet;

		for (mac_octet = 5; mac_octet >= 0; mac_octet--) {
			unsigned char xor_bit = 0;
			unsigned char octet = ha->addr[mac_octet];
			unsigned int bitshift;

			for (bitshift = 0; bitshift < 8; bitshift++)
				xor_bit ^= ((octet >> bitshift) & 0x01);

			hash = (hash << 1) | xor_bit;
		}
		csrwr32(1, priv->mac_dev, tse_csroffs(hash_table) + hash * 4);
	}
}


static void altera_tse_set_mcfilterall(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int i;

	/* set the hash filter */
	for (i = 0; i < 64; i++)
		csrwr32(1, priv->mac_dev, tse_csroffs(hash_table) + i * 4);
}

/* Set or clear the multicast filter for this adaptor
 */
static void tse_set_rx_mode_hashfilter(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);

	spin_lock(&priv->global_lock);

	if (dev->flags & IFF_PROMISC)
		tse_set_bit(priv->mac_dev, tse_csroffs(command_config),
			    MAC_CMDCFG_PROMIS_EN);

	if (dev->flags & IFF_ALLMULTI)
		altera_tse_set_mcfilterall(dev);
	else
		altera_tse_set_mcfilter(dev);

	spin_unlock(&priv->global_lock);
}

/* Set or clear the multicast filter for this adaptor
 */
static void tse_set_rx_mode(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);

	spin_lock(&priv->global_lock);

	if ((dev->flags & IFF_PROMISC) || (dev->flags & IFF_ALLMULTI) ||
	    !netdev_mc_empty(dev) || !netdev_uc_empty(dev))
		tse_set_bit(priv->mac_dev, tse_csroffs(command_config),
			    MAC_CMDCFG_PROMIS_EN);
	else
		tse_clear_bit(priv->mac_dev, tse_csroffs(command_config),
			      MAC_CMDCFG_PROMIS_EN);

	spin_unlock(&priv->global_lock);
}

/* Open and initialize the interface
 */
static int tse_open(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int ret = 0;
	int i;
	unsigned long int flags;

	/* Reset and configure TSE MAC and probe associated PHY */
	ret = priv->dmaops->init_dma(priv);
	if (ret != 0) {
		netdev_err(dev, "Cannot initialize DMA\n");
		goto phy_error;
	}

	if (netif_msg_ifup(priv))
		netdev_warn(dev, "device MAC address %pM\n",
			    dev->dev_addr);

	if ((priv->revision < 0xd00) || (priv->revision > 0xe00))
		netdev_warn(dev, "TSE revision %x\n", priv->revision);

	spin_lock(&priv->global_lock);
	ret = reset_mac(priv);
	/* Note that reset_mac will fail if the clocks are gated by the PHY
	 * due to the PHY being put into isolation or power down mode.
	 * This is not an error if reset fails due to no clock.
	 */
	if (ret)
		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);

	ret = init_mac(priv);
	spin_unlock(&priv->global_lock);
	if (ret) {
		netdev_err(dev, "Cannot init MAC core (error: %d)\n", ret);
		goto alloc_skbuf_error;
	}

	priv->dmaops->reset_dma(priv);

	/* Create and initialize the TX/RX descriptors chains. */
	priv->rx_ring_size = dma_rx_num;
	priv->tx_ring_size = dma_tx_num;
	ret = alloc_init_skbufs(priv);
	if (ret) {
		netdev_err(dev, "DMA descriptors initialization failed\n");
		goto alloc_skbuf_error;
	}


	/* Register RX interrupt */
	ret = request_irq(priv->msi_irq, altera_isr, IRQF_SHARED,
			  dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register msi interrupt %d\n",
			   priv->msi_irq);
		goto init_error;
	}

	/* Enable DMA interrupts */
	spin_lock_irqsave(&priv->global_lock, flags);
	priv->dmaops->enable_rxirq(priv);
	priv->dmaops->enable_txirq(priv);

	/* Setup RX descriptor chain */
	for (i = 0; i < priv->rx_ring_size; i++)
		priv->dmaops->add_rx_desc(priv, &priv->rx_ring[i]);

	spin_unlock_irqrestore(&priv->global_lock, flags);

	if (priv->phydev)
		phy_start(priv->phydev);

	napi_enable(&priv->napi);
	netif_start_queue(dev);

	priv->dmaops->start_rxdma(priv);

	/* Start MAC Rx/Tx */
	spin_lock(&priv->global_lock);
	tse_set_mac(priv, true);
	spin_unlock(&priv->global_lock);

	return 0;
init_error:
	free_skbufs(dev);
alloc_skbuf_error:
phy_error:
	return ret;
}

/* Stop TSE MAC interface and put the device in an inactive state
 */
static int tse_shutdown(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int ret;
	unsigned long int flags;

	/* Stop the PHY */
	if (priv->phydev)
		phy_stop(priv->phydev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);

	/* Disable DMA interrupts */
	spin_lock_irqsave(&priv->global_lock, flags);
	priv->dmaops->disable_rxirq(priv);
	priv->dmaops->disable_txirq(priv);
	spin_unlock_irqrestore(&priv->global_lock, flags);

	/* Free the IRQ lines */
	free_irq(priv->msi_irq, dev);

	/* disable and reset the MAC, empties fifo */
	spin_lock(&priv->global_lock);

	ret = reset_mac(priv);
	/* Note that reset_mac will fail if the clocks are gated by the PHY
	 * due to the PHY being put into isolation or power down mode.
	 * This is not an error if reset fails due to no clock.
	 */
	if (ret)
		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);
	priv->dmaops->reset_dma(priv);
	free_skbufs(dev);

	spin_unlock(&priv->global_lock);

	priv->dmaops->uninit_dma(priv);

	return 0;
}

static struct net_device_ops altera_tse_netdev_ops = {
	.ndo_open		= tse_open,
	.ndo_stop		= tse_shutdown,
	.ndo_start_xmit		= tse_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_set_rx_mode	= tse_set_rx_mode,
	.ndo_change_mtu		= tse_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static const struct altera_dmaops altera_dtype_msgdma = {
	.altera_dtype = ALTERA_DTYPE_MSGDMA,
	.dmamask = 64,
	.reset_dma = msgdma_reset,
	.enable_txirq = msgdma_enable_txirq,
	.enable_rxirq = msgdma_enable_rxirq,
	.disable_txirq = msgdma_disable_txirq,
	.disable_rxirq = msgdma_disable_rxirq,
	.clear_txirq = msgdma_clear_txirq,
	.clear_rxirq = msgdma_clear_rxirq,
	.tx_buffer = msgdma_tx_buffer,
	.tx_completions = msgdma_tx_completions,
	.add_rx_desc = msgdma_add_rx_desc,
	.get_rx_status = msgdma_rx_status,
	.init_dma = msgdma_initialize,
	.uninit_dma = msgdma_uninitialize,
	.start_rxdma = msgdma_start_rxdma,
};

/**************************************************************************************************************************
 * The following code is related to the specific TSE support through the PCI-e bus
 **************************************************************************************************************************/
#include <linux/pci.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>

static unsigned char mac1addr[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
static unsigned char mac2addr[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

/* Probe function to initialize one instance of the TSE ove PCIe core
 */
static int altera_tse_pciedev_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
  int rc = -ENODEV;
  struct net_device *ndev;
  int ret = -ENODEV;
  resource_size_t res_start;
  resource_size_t res_end;
  struct altera_tse_private *priv;
  unsigned char* macaddr;
  
  //Enable the PCIe device
  rc = pci_enable_device(pdev);
  if (rc)
  {
    printk("altera_tse_pciedev_probe: failed to enable the PCIe device.\n");
    return rc;
  }
  
  // Check if BAR 0 is correctly mapped
  if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM)) 
  {
    printk("altera_tse_pciedev_probe: Incorrect BAR configuration.\n");
    rc = -ENODEV;
    goto err_check_bar_mapping;
  }
  
  // Takes ownership of BAR0 region
  rc = pci_request_region(pdev, 0, ALTERA_TSE_RESOURCE_NAME);
  if (rc) 
  {
    printk("altera_tse_pciedev_probe: pci_request_regions() failed.\n");
    goto err_request_regions;
  } 
  
  // Maps BAR0 region to usable iospace
  iobase = pci_iomap(pdev, 0, 0);
  if (!iobase) 
  {
    printk("altera_tse_pciedev_probe: pci_iomap() failed\n");
    rc = -ENODEV;
    goto err_iomap0;
  }
  
  // Set private data structure
  ndev = alloc_etherdev(sizeof(struct altera_tse_private));
  if (!ndev) 
  {
    printk("altera_tse_pciedev_probe: Could not allocate network device\n");
    rc = -ENODEV;
    goto err_eth_alloc;
  }
  
  SET_NETDEV_DEV(ndev, &pdev->dev);
  priv = netdev_priv(ndev);
  priv->device = &pdev->dev;
  priv->dev = ndev;
  priv->msg_enable = netif_msg_init(debug, default_msg_level);

  // DMA configuration (MSGDMA option)
  priv->dmaops = (struct altera_dmaops*) &altera_dtype_msgdma;
  priv->rx_dma_resp = iobase + TSE_RESP_BASE;

  /* Start of that memory is for transmit descriptors */
  priv->tx_dma_desc = iobase + TSE_TX_DESC_BASE;
  priv->txdescmem = (TSE_DESCRIPTORS_SIZE);
  priv->txdescmem_busaddr = (dma_addr_t)TSE_TX_DESC_BASE;
    
  priv->rx_dma_desc = iobase + TSE_RX_DESC_BASE;
  priv->rxdescmem = (TSE_DESCRIPTORS_SIZE);
  priv->rxdescmem_busaddr = (dma_addr_t)TSE_RX_DESC_BASE;

  if (!dma_set_mask(priv->device, DMA_BIT_MASK(priv->dmaops->dmamask)))
    dma_set_coherent_mask(priv->device, DMA_BIT_MASK(priv->dmaops->dmamask));
  else if (!dma_set_mask(priv->device, DMA_BIT_MASK(32)))
    dma_set_coherent_mask(priv->device, DMA_BIT_MASK(32));
  else
  {
    rc = -ENODEV;
    goto err_free_netdev;
  }
  
  // Setup TSE MAC resources
  priv->mac_dev = iobase + TSE_REGISTERS_BASE;
  res_start = TSE_REGISTERS_BASE;
  res_end = TSE_REGISTERS_BASE + 0x400;
  
  //xSGDMA Rx Dispatcher address space
  priv->rx_dma_csr = iobase + TSE_SGDMA_RX_BASE;
  
  //xSGDMA Tx Dispatcher address space
  priv->tx_dma_csr = iobase + TSE_SGDMA_TX_BASE;
  //Enable MSI interrupt (single interrupt)
  pci_set_master(pdev);
  rc = pci_enable_msi(pdev);
  if(rc)
  {
    printk("altera_tse_pciedev_probe: Could not enable the MSI interrupts\n");
    goto err_free_netdev;
  }
  
  priv->msi_irq = pdev-> irq;
  
  //Rx and Tx FIFO depths
  priv->rx_fifo_depth = 2048;
  priv->tx_fifo_depth = 2048;
  
  //Misc datas
  priv->hash_filter = 0;
  priv->added_unicast = 1;
  priv->max_mtu = ETH_DATA_LEN;
  priv->rx_dma_buf_sz = ALTERA_RXDMABUFFER_SIZE;

  //Init spinlock
  spin_lock_init(&priv->global_lock);
 
  //Get MAC address from cmdline
  if(pdev->subsystem_device==0)
    macaddr = mac1addr; 
  else
    macaddr = mac2addr; 
    
  if (is_valid_ether_addr(macaddr))
    ether_addr_copy(ndev->dev_addr, macaddr);
  else
    eth_hw_addr_random(ndev);
  
  // get phy addr and create mdio (autodetect phy address)
  priv->phy_iface = PHY_INTERFACE_MODE_RMII;
  priv->phy_addr = POLL_PHY;

  rc = altera_tse_mdio_create(ndev, atomic_add_return(1, &instance_count));
  if(rc)
  {
    printk("altera_tse_pciedev_probe: Could not attach phy\n");
    goto err_free_netdev;
  }
  
  // initialize netdev
  ndev->mem_start = res_start;
  ndev->mem_end = res_end;
  ndev->netdev_ops = &altera_tse_netdev_ops;
  altera_tse_set_ethtool_ops(ndev);
  
  altera_tse_netdev_ops.ndo_set_rx_mode = tse_set_rx_mode;
  
  if (priv->hash_filter)
    altera_tse_netdev_ops.ndo_set_rx_mode = tse_set_rx_mode_hashfilter;
  
  // Scatter/gather IO is not supported, so it is turned off
  ndev->hw_features &= ~NETIF_F_SG;
  ndev->features |= ndev->hw_features | NETIF_F_HIGHDMA;
  
  /* VLAN offloading of tagging, stripping and filtering is not
   * supported by hardware, but driver will accommodate the
   * extra 4-byte VLAN tag for processing by upper layers
   */
  ndev->features |= NETIF_F_HW_VLAN_CTAG_RX;
  
  // setup NAPI interface 
  netif_napi_add(ndev, &priv->napi, tse_poll, NAPI_POLL_WEIGHT);
  
  rc = register_netdev(ndev);
  if (rc) 
  {
    printk("altera_tse_pciedev_probe: failed to register TSE net device\n");
    goto err_register_netdev;
  }
  
  dev_set_drvdata(&pdev->dev,ndev);
  priv->revision = ioread32(&priv->mac_dev->megacore_revision);
  printk("TSE PCIe MAC revision: 0x%x\n",priv->revision);
  
  if (netif_msg_probe(priv))
    dev_info(&pdev->dev, "Altera TSE MAC version %d.%d at 0x%08lx irq %d\n",
	     (priv->revision >> 8) & 0xff,
	     priv->revision & 0xff,
	     (unsigned long) res_start, priv->msi_irq);
  
   rc = init_phy(ndev);
  if (rc != 0) 
  {
    printk("altera_tse_pciedev_probe: Cannot attach to PHY (error: %d)\n", ret);
    goto err_init_phy;
  }
  
  // Probe OK
  return 0;

err_init_phy:
  unregister_netdev(ndev);
err_register_netdev:
  netif_napi_del(&priv->napi);
  altera_tse_mdio_destroy(ndev);
err_free_netdev:
  free_netdev(ndev);  
err_eth_alloc:
  pci_iounmap(pdev, iobase);
err_iomap0:
  pci_release_regions(pdev);
err_request_regions:  
err_check_bar_mapping:  
  pci_disable_device(pdev);

  return rc;
}  

/*
 * Remove one instance of the TSE over PCIe core
 */
static void altera_tse_pciedev_remove(struct pci_dev *pdev)
{
  struct net_device *ndev = pci_get_drvdata(pdev);
  struct altera_tse_private *priv = netdev_priv(ndev);
  if (ndev) 
  {
    unregister_netdev(ndev);
    netif_napi_del(&priv->napi);
    altera_tse_mdio_destroy(ndev);
    free_netdev(ndev);
    pci_iounmap(pdev, priv->mac_dev);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
  }	
}

static const struct pci_device_id altera_tse_pciedev_ids[] = {
	{ PCI_DEVICE(0x1172, 0xe3ac) },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, altera_tse_pciedev_ids);

static struct pci_driver altera_tse_driver = {
	.probe		= altera_tse_pciedev_probe,
	.remove		= altera_tse_pciedev_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.name	= ALTERA_TSE_RESOURCE_NAME,
	.id_table = altera_tse_pciedev_ids,
};

//This to register the PCI driver
module_pci_driver(altera_tse_driver); 

/*--------------------------------------------------------------------*
  MAC addresses taken from cmdline options
 *--------------------------------------------------------------------*/
static int __init getmac1addr(char* str)
{
  sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac1addr[0], &mac1addr[1], &mac1addr[2], &mac1addr[3], &mac1addr[4], &mac1addr[5]);
  return 1;
}

static int __init getmac2addr(char* str)
{
  sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac2addr[0], &mac2addr[1], &mac2addr[2], &mac2addr[3], &mac2addr[4], &mac2addr[5]);
  return 1;
}

__setup("pcie_tse1addr=",getmac1addr);
__setup("pcie_tse2addr=",getmac2addr);

MODULE_AUTHOR("Altera Corporation");
MODULE_DESCRIPTION("Altera Triple Speed Ethernet MAC driver");
MODULE_LICENSE("GPL v2");
