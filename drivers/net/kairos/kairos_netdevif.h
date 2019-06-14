/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2013 Flexibilis Oy

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License version 2
   as published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef DEIPCE_NETDEVIF_H
#define DEIPCE_NETDEVIF_H

#include <linux/interrupt.h>

#include "deipce_types.h"

/// Number of bits in management trailer for MACsec
#define DEIPCE_TRAILER_MACSEC_BITS 1

int deipce_netdevif_init(struct deipce_dev_priv *dp);
void deipce_netdevif_cleanup(struct deipce_dev_priv *dp);
void deipce_netdevif_cleanup_stamper(struct deipce_tx_stamper *stamper);
int deipce_xmit(struct deipce_dev_priv *dp, struct sk_buff *skb,
                unsigned int trailer);
int deipce_irq_init(struct deipce_dev_priv *dp);
void deipce_irq_enable(struct deipce_dev_priv *dp);
void deipce_irq_disable(struct deipce_dev_priv *dp);
void deipce_irq_cleanup(struct deipce_dev_priv *dp);
int deipce_update_ipo_rules(struct deipce_port_priv *pp);
unsigned int deipce_get_mgmt_tc(struct deipce_dev_priv *dp);
int deipce_set_mgmt_tc(struct deipce_dev_priv *dp, unsigned int tc);
int deipce_get_mirror_port(struct deipce_port_priv *pp);
int deipce_set_mirror_port(struct deipce_port_priv *pp, int mirror_port);

/**
 * Get value for FRS interrupt mask register.
 * @param dp FRS device privates.
 * @return Value to write to FRS interrupt mask register.
 */
static inline uint16_t deipce_get_intmask(struct deipce_dev_priv *dp)
{
    // With port N timestampers use only TX timestamp interrupt.
    if (dp->use_port_ts)
        return FRS_INT_TX_TSTAMP;
    // With port 0 (CPU port) timestampers use only RX timestamp interrupt.
    return FRS_INT_RX_TSTAMP;
}

/**
 * Get management trailer with MACsec bit set for given switch.
 * @param dp FRS device privates.
 * @return Management trailer which has only the MACsec bit set.
 */
static inline uint16_t deipce_get_macsec_trailer(struct deipce_dev_priv *dp)
{
    // Most significant bit.
    if (dp->features.macsec_ports)
        return 1u << (dp->trailer_len*8 - 1);
    return 0;
}

/**
 * Get management trailer from an sk_buff, where it appears big-endian.
 * @param skb sk_buff with an Ethernet frame to get management trailer from.
 * @param trailer_len Number of octets in management trailer.
 * @return Management trailer.
 */
static inline unsigned int deipce_skb_get_trailer(const struct sk_buff *skb,
                                                  unsigned int trailer_len)
{
    const uint8_t *skb_trailer = skb_tail_pointer(skb) - trailer_len;

    if (trailer_len == 1) {
        return (unsigned int) skb_trailer[0];
    }
    else {
        return
            ((unsigned int) skb_trailer[0] << 8) |
            ((unsigned int) skb_trailer[1] << 0);
    }

    return 0;
}

/**
 * Write management trailer to an sk_buff, in big-endian form.
 * @param skb sk_buff to add management trailer to.
 * @param trailer Management trailer to add.
 * @parm trailer_len Number of octets in management trailer.
 */
static inline void deipce_skb_set_trailer(uint8_t *skb_trailer,
                                          unsigned int trailer,
                                          unsigned int trailer_len)
{
    if (trailer_len == 1) {
        skb_trailer[0] = (uint8_t) ((trailer >> 0) & 0xffu);
    }
    else {
        skb_trailer[0] = (uint8_t) ((trailer >> 8) & 0xffu);
        skb_trailer[1] = (uint8_t) ((trailer >> 0) & 0xffu);
    }

    return;
}

int deipce_skb_add_trailer(struct sk_buff *skb, unsigned int trailer,
                           unsigned int trailer_len);

#endif
