/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/etherdevice.h>
#include "dsa_priv.h"

#define KAIROS_TRAILER_LEN	1

static struct sk_buff *kairos_tag_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_slave_priv *p = netdev_priv(dev);
	u8 *phdr, hdr;

//printk(KERN_INFO "%s %d %d port %d\n", __func__, skb->len, skb->data_len, p->port);

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	if (skb_cow_head(skb, 0) < 0)
		goto out_free;

#if 0
	skb_put(skb, KAIROS_TRAILER_LEN);

	phdr = (u8*)skb->data + (skb->len-KAIROS_TRAILER_LEN);

	/* set destination port information */
	hdr = 0x00;
	hdr = (0x01 << p->port) << 2;

	*phdr = hdr;
#endif

//printk(KERN_INFO "%s (2) %d %d\n", __func__, skb->len, skb->data_len);
	return skb;

out_free:
	kfree_skb(skb);
	return NULL;
}

static int kairos_tag_rcv(struct sk_buff *skb, struct net_device *dev,
		       struct packet_type *pt, struct net_device *orig_dev)
{
	struct dsa_switch_tree *dst = dev->dsa_ptr;
	struct dsa_switch *ds;
	int port;
	u8 *phdr, hdr;

/*
printk(KERN_INFO "%s len %d\n", __func__, skb->len);
if (skb->len == 46)
{
	int i;
	for (i=0; i<=skb->len; i++)
		printk(KERN_INFO "%02X ", skb->data[i]);
	printk(KERN_INFO "\n");
}
*/

	if (unlikely(!dst))
		goto out_drop;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

#if 0	
	phdr = (u8*)skb->data + (skb->len-KAIROS_TRAILER_LEN);
	hdr = *phdr;

	skb_trim(skb, KAIROS_TRAILER_LEN);

	/* Get source port information */
	port = hdr & 0x03;
#endif

	/* This protocol doesn't support cascading multiple switches so it's
	 * safe to assume the switch is first in the tree
	 */
	ds = dst->ds[0];
//printk(KERN_INFO "%s (2) %p\n", __func__, ds);
	if (!ds)
		goto out_drop;

	//@todo
	port = 0;
//printk(KERN_INFO "%s (3) %02X %d %p\n", __func__, hdr, port, ds->ports[port].netdev);
	if (!ds->ports[port].netdev)
		goto out_drop;

//printk(KERN_INFO "%s (4) %02X %d %p\n", __func__, hdr, port, ds->ports[port].netdev);
	/* Update skb & forward the frame accordingly */
	skb->dev = ds->ports[port].netdev;
	skb_push(skb, ETH_HLEN);
	skb->pkt_type = PACKET_HOST;
	skb->protocol = eth_type_trans(skb, skb->dev);

	skb->dev->stats.rx_packets++;
	skb->dev->stats.rx_bytes += skb->len;

	netif_receive_skb(skb);

	return 0;

out_drop:
	kfree_skb(skb);
out:
	return 0;
}

const struct dsa_device_ops kairos_netdev_ops = {
	.xmit	= kairos_tag_xmit,
	.rcv	= kairos_tag_rcv,
};
