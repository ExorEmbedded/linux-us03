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

#define KAIROS_ALWAYS_OBT

static struct sk_buff *kairos_tag_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_slave_priv *p = netdev_priv(dev);
	int trail_length;
	u8 *phdr;
	u8* data;
	u8 hdr;

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	if (skb_cow_head(skb, 0) < 0) {
		printk(KERN_ERR "%s: ERROR len: %d, data_len: %d,  port: %d\n", __func__, skb->len, skb->data_len, p->port);
		goto out_free;
	}

#ifdef KAIROS_ALWAYS_OBT
	// TX (i.e. from CPU to ports) OBT format
	// bit 7: force untagged transmit
	// bit 0: n/a
	// bit 1: tunnel
	// bit 2: front port 1
	// bit 3: front port 2

	trail_length = KAIROS_TRAILER_LEN;
	if (skb->len < 64)
		trail_length += (64 - skb->len);

	if (skb_tailroom(skb) < trail_length)
	{
		struct sk_buff* new_skb;
		new_skb = skb_copy_expand(skb, 0, trail_length, GFP_ATOMIC);

		kfree_skb(skb);
		skb = new_skb;
	}

	data = (u8*)skb->data + skb->len;
	skb_put(skb, trail_length);

	memset(data, 0, trail_length);

	phdr = (u8*)skb->data + (skb->len-KAIROS_TRAILER_LEN);

	/* set destination port information (port ranges from 0 to 1) */
	hdr = (0x01 << p->port) << 2;
	*phdr = hdr;
#endif

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
	int port = 0;
	u8 *phdr;
	u8 hdr = 0;

	if (unlikely(!dst))
		goto out_drop;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

#ifdef KAIROS_ALWAYS_OBT	
	// RX (i.e. from ports to CPU) OBT format
	// bit 7: ingress tag status
	// bit 0..1: port ID
	//		00: n/a
	//      01: tunnel
	//      10: front port 1
	//      11: front port 2

	phdr = (u8*)skb->data + (skb->len-KAIROS_TRAILER_LEN);
	hdr = *phdr;

	skb_trim(skb, skb->len-KAIROS_TRAILER_LEN);

	/* Get source port information */
	if (! (hdr & 0x02))
	{
		printk(KERN_ERR "%s: invalid HDR: %02X \n", __func__, hdr);
		goto out_drop;
	}

	port = hdr & 0x01;
	if ((port != 0) && (port != 1))
	{
		printk(KERN_ERR "%s: invalid HDR: %02X \n", __func__, hdr);
		goto out_drop;
	}
#endif

	/* This protocol doesn't support cascading multiple switches so it's
	 * safe to assume the switch is first in the tree
	 */
	ds = dst->ds[0];
	if (!ds)
		goto out_drop;

	if (!ds->ports[port].netdev)
		goto out_drop;

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
