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

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/rtnetlink.h>
#include <linux/sched.h>

#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_if.h"
#include "deipce_hw.h"
#include "deipce_preempt.h"
#include "deipce_time.h"
#include "deipce_netdev.h"
#include "deipce_netdevif.h"

/// Uncomment to enable FRS port resolve debug messages
//#define DEBUG_FRS_RESOLVE_PORT

#ifdef DEBUG_FRS_RESOLVE_PORT
# define dev_resolve_dbg(dev, ...) dev_printk(KERN_DEBUG, (dev), __VA_ARGS__)
#else
# define dev_resolve_dbg(dev, ...) do { } while (0)
#endif

/// Uncomment to enable FRS timestamp debug messages
//#define DEBUG_FRS_TS
//#define DEBUG_FRS_TS_DUMP

#ifdef DEBUG_FRS_TS
# define dev_ts_dbg(dev, ...) dev_printk(KERN_INFO, (dev), __VA_ARGS__)
#else
# define dev_ts_dbg(dev, ...) do { } while (0)
#endif

//#define DEBUG_FRS_MODIFY_XMIT

/**
 * IPO entry information.
 */
struct deipce_ipo {
    uint8_t address[ETH_ALEN];  ///< destination MAC address
    unsigned int cmp_len;       ///< number of bits to compare
    uint16_t cfg0;              ///< cfg0 bits
    uint16_t cfg1;              ///< cfg1 bits
};

/**
 * IPO entry information for reserved MAC addresses
 * which are only forwarded to CPU.
 */
static const struct deipce_ipo deipce_ipo_reserved[] = {
    // DMAC 01-80-C2-00-00-0x: management priority, disable cut-through
    { { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x00 }, 44,
        PORT_ETH_ADDR_ENABLE | PORT_ETH_ADDR_DEST,
        PORT_ETH_ADDR_CMP_ORDER | PORT_ETH_ADDR_CT_DISABLE,
    },
    // DMAC 01-80-C2-00-00-2x: management priority
    { { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x20 }, 44,
        PORT_ETH_ADDR_ENABLE | PORT_ETH_ADDR_DEST,
        PORT_ETH_ADDR_CMP_ORDER,
    },
};

/// Total number of IPO entries
#define DEIPCE_IPO_ENTRIES 16

/// Null MAC address for unused IPO entries
static const uint8_t null_mac_addr[] = {
    0, 0, 0, 0, 0, 0
};

/**
 * Get management trailer port mask bits.
 * @param dp FRS device privates.
 * @param trailer Management trailer.
 * @return Management trailer with only port bits set.
 */
static inline uint16_t deipce_trailer_port_mask(struct deipce_dev_priv *dp,
                                                uint16_t trailer)
{
    return trailer &
        ~deipce_get_macsec_trailer(dp) &
        ~deipce_preempt_trailer_mask(dp);
}

/**
 * Add Management trailer to frame.
 * @param skb Socket buffer to add trailer to.
 * @param trailer trailer to add.
 * @param trailer_len Trailer length.
 */
int deipce_skb_add_trailer(struct sk_buff *skb, unsigned int trailer,
                           unsigned int trailer_len)
{
    int ret = -ENOBUFS;

    // Ethernet has minimum packet length, pad also if necessary.
    if (skb->len < ETH_ZLEN) {
        // Verify that there is space for padding and trailer.
        unsigned int skb_len_dif = ETH_ZLEN - skb->len;
        ret = skb_pad(skb, skb_len_dif + trailer_len);
        if (ret)
            return ret;

        memset(skb_put(skb, skb_len_dif), 0, skb_len_dif);
    }
    else {
        // Verify that there is space for trailer.
        ret = skb_pad(skb, trailer_len);
        if (ret)
            return ret;
    }

    deipce_skb_set_trailer(skb_put(skb, trailer_len), trailer, trailer_len);

    return 0;
}

/**
 * Get CPU port of a FES instance.
 * @param dp FES device privates.
 * @return CPU port privates, or NULL if FES does not have a CPU port.
 * It is expected that it has one.
 */
static struct deipce_port_priv *deipce_get_cpu_port(struct deipce_dev_priv *dp)
{
    unsigned long int port_mask = dp->cpu_port_mask;
    unsigned int port_num = find_first_bit(&port_mask, DEIPCE_MAX_PORTS);
    struct deipce_port_priv *port = NULL;

    if (port_num >= DEIPCE_MAX_PORTS) {
        dev_err(dp->this_dev, "No CPU port\n");
        return NULL;
    }

    port = dp->port[port_num];
    if (!port) {
        dev_err(dp->this_dev, "CPU port %u is unknown\n", port_num);
        return NULL;
    }

    return port;
}

/**
 * Detect if given MAC address matches an IPO entry.
 * @param address MAC address to check for match.
 * @param ipo IPO entry with MAC address and compare length.
 * @return True if IPO entry matches the MAC address.
 */
static bool deipce_match_ipo(const uint8_t *address,
                             const struct deipce_ipo *ipo)
{
    unsigned int bytes = ipo->cmp_len / 8;
    uint8_t mask = ~((1u << (ipo->cmp_len - bytes * 8)) - 1u);

    if (memcmp(address, ipo->address, bytes))
        return false;
    if (mask != 0xffu) {
        if ((address[bytes] & mask) != (ipo->address[bytes] & mask))
            return false;
    }
    return true;
}

/**
 * Check whether MAC address matches any of reserved MAC addresses.
 * @param address MAC address to check.
 * @return true if addr is a reserved MAC address.
 */
static bool deipce_match_reserved(const uint8_t *address)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(deipce_ipo_reserved); i++) {
        if (deipce_match_ipo(address, &deipce_ipo_reserved[i]))
            return true;
    }

    return false;
}

/**
 * Get prioritization for incoming management traffic.
 * RTNL lock must be held when calling this.
 * @param dp Switch privates.
 * @return Traffic class.
 */
unsigned int deipce_get_mgmt_tc(struct deipce_dev_priv *dp)
{
    return dp->mgmt_tc;
}

/**
 * Set traffic class for incoming management traffic.
 * RTNL lock must be held when calling this.
 * @param pp Port privates.
 * @param tc traffic class.
 */
int deipce_set_mgmt_tc(struct deipce_dev_priv *dp, unsigned int tc)
{
    struct deipce_port_priv *pp;
    unsigned int port_num;
    int ret;

    if (tc >= dp->features.prio_queues)
        return -EINVAL;

    dp->mgmt_tc = tc;

    for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
        pp = dp->port[port_num];
        if (!pp)
            continue;

        ret = deipce_update_ipo_rules(pp);
        if (ret)
            break;
    }

    return ret;
}

/**
 * Get incoming traffic mirror port.
 * RTNL lock must be held when calling this.
 * @param pp Port privates.
 * @return Port number for mirroring all traffic, or -1 if disabled.
 */
int deipce_get_mirror_port(struct deipce_port_priv *pp)
{
    return pp->mirror_port;
}

/**
 * Set incoming traffic mirroring.
 * RTNL lock must be held when calling this.
 * @param pp Port privates.
 * @param mirror_port Port number of the port to mirror traffic to,
 * or -1 to disable mirroring.
 */
int deipce_set_mirror_port(struct deipce_port_priv *pp, int mirror_port)
{
    struct deipce_dev_priv *dp = pp->dp;
    int ret;

    if (mirror_port >= 0 && mirror_port < dp->num_of_ports)
        pp->mirror_port = mirror_port;
    else
        pp->mirror_port = -1;
    ret = deipce_update_ipo_rules(pp);

    return ret;
}

/**
 * Deal with automatically handled IPO rules.
 * RTNL lock must be held when calling this.
 * @param pp Port privates.
 */
int deipce_update_ipo_rules(struct deipce_port_priv *pp)
{
    struct deipce_dev_priv *dp = pp->dp;
    unsigned int entry = 0;
    unsigned int i = 0;
    const struct deipce_ipo *ipodata = NULL;
    uint16_t all_ports = (1u << dp->num_of_ports) - 1u;
    uint16_t allow_reserved;
    uint16_t mirror_reserved;
    uint16_t mirror_all = 0;
    int ret = -EIO;
    uint16_t mgmt_prio;

    netdev_dbg(pp->netdev, "%s()\n", __func__);

    if (pp->mirror_port >= 0)
        mirror_all = 1u << pp->mirror_port;

    // In case of 4 queues, map classes (0-3) <--> queues (0,2,4,6) in IP.
    if (dp->features.prio_queues == 4)
        mgmt_prio = PORT_ETH_ADDR_FROM_PRIO(dp->mgmt_tc << 1);
    else
        mgmt_prio = PORT_ETH_ADDR_FROM_PRIO(dp->mgmt_tc);

    if (pp->flags & DEIPCE_PORT_CPU) {
        // Allow forwarding reserved traffic to all ports.
        allow_reserved = all_ports;
        mirror_reserved = mirror_all;
    }
    else {
        /*
         * Allow and force forwarding reserved traffic only to CPU port
         * and possibly to mirror port.
         */
        allow_reserved = dp->cpu_port_mask | mirror_all;
        mirror_reserved = allow_reserved;
    }

    for (i = 0; i < ARRAY_SIZE(deipce_ipo_reserved); i++) {
        ipodata = &deipce_ipo_reserved[i];
        ret = deipce_write_port_ipo(pp, entry++,
                                    ipodata->cfg0 | mgmt_prio,
                                    ipodata->cfg1,
                                    allow_reserved, mirror_reserved, 0,
                                    ipodata->address, ipodata->cmp_len);
        if (ret)
            return ret;
    }

    if (mirror_all)
        ret = deipce_write_port_ipo(pp, entry++,
                                    PORT_ETH_ADDR_ENABLE |
                                    PORT_ETH_ADDR_DEST |
                                    PORT_ETH_ADDR_PRESERVE_PRIORITY,
                                    0,
                                    all_ports, mirror_all, 0,
                                    null_mac_addr, 0);
    else
        ret = deipce_write_port_ipo(pp, entry++,
                                    0, 0, 0, 0, 0,
                                    null_mac_addr, 0);
    if (ret)
        return ret;

    // Clear the rest IPO entries.
    for (; entry < DEIPCE_IPO_ENTRIES; entry++) {
        ret = deipce_write_port_ipo(pp, entry,
                                    0, 0, 0, 0, 0,
                                    null_mac_addr, 0);
        if (ret)
            return ret;
    }

    netdev_dbg(pp->netdev, "%s() done\n", __func__);

    return 0;
}

/**
 * Determine if PTP message is a sync message, from common PTP header type
 * field.
 * @param ptp_type PTP type field value.
 * @return True if sync message, false otherwise.
 */
static inline bool deipce_is_ptp_sync(uint8_t ptp_type)
{
    return (ptp_type & 0xfu) == 0;
}

/**
 * Determine if PTP message is an event message, from common PTP header type
 * field.
 * @param ptp_type PTP type field value.
 * @return True if event message, false otherwise.
 */
static inline bool deipce_is_ptp_event(uint8_t ptp_type)
{
    ptp_type &= 0x0f;
    return ptp_type >= 0 && ptp_type <= 3;
}

/**
 * Get start of PTP header in an sk_buff for PTP event messages.
 * @param dp FES device privates (PTP mode is checked).
 * @param skb Buffer with assumed PTP message.
 * @return Pointer to start of PTP header or NULL if skb does not contain
 * a PTP event message or FES PTP mode does not match.
 */
static const uint8_t *deipce_skb_get_ptp_hdr(struct deipce_dev_priv *dp,
                                             struct sk_buff *skb)
{
    const struct ethhdr *eth = eth_hdr(skb);

    if (dp->tstamp_cfg.rx_filter == HWTSTAMP_FILTER_PTP_V2_L2_EVENT) {
        if (eth->h_proto == htons(ETH_P_1588)) {
            // Only event messages are timestamped.
            const uint8_t *ptp = (uint8_t *)(eth + 1);
            if (deipce_is_ptp_event(ptp[0]))
                return (uint8_t *)(eth + 1);
        }
    }
    else if (eth->h_proto == htons(ETH_P_IP)) {
        const struct iphdr *ip = ip_hdr(skb);

        if (ip->version == 4 && ip->protocol == IPPROTO_UDP) {
            const struct udphdr *udp = udp_hdr(skb);

            // Only event messages (dport 319) are timestamped.
            if ((udp->dest == htons(319)) &&
                (ntohs(udp->len) >= sizeof(*udp) + FRS_TS_HDR_LEN))
                return (uint8_t *)(udp + 1);
        }
    }

    return NULL;
}

/**
 * Prepare frame for HW TX timestamping.
 * Make a clone of the sk_buff for MAC driver and remember the original for
 * timestamping purposes. Underlying MAC driver will kind of eat the sk_buff
 * passed to it, and information in original (but missing in clone) is needed
 * when passing back to error queue.
 * @param dp Device privates of switch with CPU port.
 * @param skb Frame to prepare for timestamping.
 * @param trailer Management trailer of the frame.
 * @return Frame to actually sent, may be different from skb.
 */
static struct sk_buff *deipce_prepare_tx_timestamp(struct deipce_dev_priv *dp,
                                                   struct sk_buff *skb,
                                                   unsigned int trailer)
{
    struct deipce_port_priv *port = NULL;
    const uint8_t *ptp = deipce_skb_get_ptp_hdr(dp, skb);
    struct deipce_tx_stamper *stamper = &dp->rx_stamper;
    struct sk_buff *skb_tx = NULL;
    unsigned long int flags = 0;
    unsigned int slot = 0;
    bool dropped = false;

    // Do not bother if it cannot be timestamped.
    if (!ptp) {
        dev_ts_dbg(dp->this_dev, "TX: TS not possible for frame\n");
        return skb;
    }

    skb_tx = skb_clone(skb, GFP_ATOMIC);
    if (!skb_tx) {
        dev_err(dp->this_dev,
                "Failed to clone skb for FRS TX timestamping\n");
        return skb;
    }

    /*
     * Normally use port 0 RX timestamper (in switch registers),
     * but if port timestampers are in use, use port N (N != 0)
     * TX timestamper.
     */
    if (dp->use_port_ts) {
        unsigned long int port_mask = trailer;
        unsigned int port_num;

        port_mask &= ~deipce_get_macsec_trailer(dp);

        // Store in first matching port.
        port_num = find_first_bit(&port_mask, DEIPCE_MAX_PORTS);

        if (port_num < ARRAY_SIZE(dp->port))
            port = dp->port[port_num];

        if (port) {
            if (!(port->flags & DEIPCE_PORT_CPU) &&
                (dp->features.ts_ports & (1u << port->port_num)))
                stamper = &port->tx_stamper;
            else
                port = NULL;
        }
    }

    spin_lock_irqsave(&stamper->lock, flags);

    slot = stamper->next_skb;
    if (stamper->orig_skb[slot]) {
        // Give up on old skb timestamping and reuse slot.
        dev_kfree_skb_any(stamper->orig_skb[slot]);
        dropped = true;
        dp->stats.tx_stamp_lost++;
    }
    stamper->orig_skb[slot] = skb;
    stamper->next_skb = (slot + 1) % FRS_TS_NUMBER_TIMESTAMPS;

    spin_unlock_irqrestore(&stamper->lock, flags);

    skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

    if (dropped) {
        dev_ts_dbg(dp->this_dev, "TX: TS drop msg 0x%02x from slot %u %s\n",
                   ptp[0], slot,
                   netdev_name(port ? port->netdev : dp->netdev));
    }

    dev_ts_dbg(dp->this_dev,
               "TX: TS clone msg 0x%02x %p %p tx_flags 0x%x slot %u %s\n",
               ptp[0], skb_tx, skb,
               skb_shinfo(skb_tx)->tx_flags,
               slot,
               netdev_name(port ? port->netdev : dp->netdev));

    skb_tx_timestamp(skb);

    /*
     * Do not let MAC driver timestamping interfere.
     * shinfo is shared between clones.
     * These are restored before passing skb back with timestamp.
     */
    skb_shinfo(skb)->tx_flags &= ~SKBTX_HW_TSTAMP;
    skb_shinfo(skb)->tx_flags &= ~SKBTX_IN_PROGRESS;

    // Pass the clone to MAC.
    return skb_tx;
}

/**
 * Determine whether frame should be transmitted locally.
 * @param dp Device privates of switch with CPU port.
 * @param skb Frame to transmit.
 * @param trailer Management trailer for sending.
 * @return true if frame must be transmitted locally. In that case
 * skb->dev has been updated to the receiving netdevice.
 */
static bool deipce_xmit_local(struct deipce_dev_priv *dp, struct sk_buff *skb,
                              unsigned int trailer)
{
    struct deipce_port_priv *cpu_port = deipce_get_cpu_port(dp);

    if (!cpu_port)
        return false;

    // Checks for frames sent via endpoint netdevice.
    if (trailer == deipce_get_macsec_trailer(dp)) {
        /*
         * Pass frames sent to CPU port MAC address immediately back as being
         * received from CPU port netdevice.
         */
        const struct ethhdr *eth = eth_hdr(skb);

        if (ether_addr_equal(eth->h_dest, cpu_port->netdev->dev_addr)) {
            skb->dev = cpu_port->netdev;
            dev_resolve_dbg(dp->this_dev,
                            "Resolve trailer 0x%x -> %s"
                            " (local, CPU port unicast)\n",
                            trailer, netdev_name(cpu_port->netdev));
            return true;
        }

        /*
         * Pass frames sent to reserved MAC addresses immediately back as being
         * received from CPU port net device.
         */
        if (deipce_match_reserved(eth->h_dest)) {
            skb->dev = cpu_port->netdev;
            dev_resolve_dbg(dp->this_dev,
                            "Resolve trailer 0x%x -> %s"
                            " (local, reserved)\n",
                            trailer, netdev_name(cpu_port->netdev));
            return true;
        }

        return false;
    }

    // Checks for frames sent via CPU port netdevice.
    if (trailer == cpu_port->trailer) {
        /*
         * Pass all frames immediately back as being received from endpoint
         * netdevice.
         */
        skb->dev = dp->netdev;
        dev_resolve_dbg(dp->this_dev,
                        "Resolve trailer 0x%x -> %s (local)\n",
                        trailer, netdev_name(dp->netdev));
        return true;
    }

    return false;
}

/**
 * Transmit frame. This eats the skb, it must not be accessed after call.
 * @param dp Device privates of frame with CPU port.
 * @param skb Frame to send, without management trailer.
 * @param trailer Management trailer of the frame,
 * used for PTP frame TX timestamp handling.
 */
int deipce_xmit(struct deipce_dev_priv *dp, struct sk_buff *skb,
                unsigned int trailer)
{
    int ret = -ENOBUFS;
    bool local = false;

    local = deipce_xmit_local(dp, skb, trailer);
    if (local) {
        deipce_rx_frame(skb);
        return 0;
    }

    if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) {
        skb = deipce_prepare_tx_timestamp(dp, skb, trailer);
    }
    else {
        skb_tx_timestamp(skb);
    }

    ret = deipce_skb_add_trailer(skb, trailer, dp->trailer_len);
    if (ret)
        goto drop;

#ifdef DEBUG_FRS_MODIFY_XMIT
    dev_printk(KERN_DEBUG, dp->this_dev,
               "Sending with trailer 0x%x\n", trailer);
    print_hex_dump_bytes("tx_frame: ", DUMP_PREFIX_OFFSET,
                         &skb->data[0], skb->len);
#endif

    // Set new destination.
    skb->dev = dp->real_netdev;
    dev_queue_xmit(skb);

    return 0;

drop:
    dev_kfree_skb_any(skb);
    return ret;
}

/**
 * Function to resolve receiving netdevice. Resolves correct netdevice by using
 * the rx_frame management trailer. May modify rx_frame, e.g. remove trailer.
 * @param dp Device privates with CPU port from which frame was received.
 * @param rx_frame Received frame. Field dev is updated to receiving netdevice.
 * @return Port from which frame was actually received, or NULL if frame should
 * be dropped. Needed for timestamping.
 */
static struct deipce_port_priv *deipce_resolve_rx_netdev(
        struct deipce_dev_priv *dp,
        struct sk_buff *rx_frame)
{
    struct deipce_port_priv *port = NULL;
    struct deipce_port_priv *cpu_port = NULL;
    unsigned int trailer = 0;
    unsigned int port_trailer = 0;
    unsigned long int port_mask = 0;
    const struct ethhdr *eth = eth_hdr(rx_frame);
    unsigned int port_num = 0;

    if (!eth) {
        dev_err(dp->this_dev, "No data\n");
        return NULL;
    }

    cpu_port = deipce_get_cpu_port(dp);
    if (!cpu_port) {
        dev_err(dp->this_dev, "No CPU port\n");
        return NULL;
    }

#ifdef DEBUG_FRS_RESOLVE_PORT
    print_hex_dump_bytes("rx_frame: ", DUMP_PREFIX_OFFSET,
                         eth, rx_frame->len);
    dev_resolve_dbg(dp->this_dev, "Trailer length %i skb length %i\n",
                    dp->trailer_len, rx_frame->len);
#endif

    // Determine management trailer with and without additional bits.
    trailer = deipce_skb_get_trailer(rx_frame, dp->trailer_len);
    port_trailer = deipce_trailer_port_mask(dp, trailer);

    // Remove trailer from the frame length.
    skb_trim(rx_frame, rx_frame->len - dp->trailer_len);
    // Newer kernels emit HW csum failure without this.
    if (rx_frame->ip_summed == CHECKSUM_COMPLETE)
        rx_frame->ip_summed = CHECKSUM_NONE;

    dev_resolve_dbg(dp->this_dev,
                    "Resolve trailer 0x%x length %i skb length %i"
                    " eth_hdr %li mac_header %li proto 0x%x\n",
                    trailer, dp->trailer_len, rx_frame->len,
                    (long int)((unsigned char *)eth - rx_frame->head),
                    (long int)(skb_mac_header(rx_frame) - rx_frame->head),
                    (int)ntohs(eth->h_proto));

    /*
     * Zero management trailer means frame was originally sent by CPU,
     * with management trailer zero (from endpoint net device), and mirrored
     * back to CPU. Or there is a serious problem.
     */
    if (port_trailer == 0) {
        dev_resolve_dbg(dp->this_dev,
                        "Resolve trailer 0x%x %s -> %s (ep -> cpu)\n",
                        trailer, netdev_name(cpu_port->netdev),
                        netdev_name(cpu_port->netdev));
        rx_frame->dev = cpu_port->netdev;
        return cpu_port;
    }

    // There can be only one bit set in valid management trailer port mask.
    if (port_trailer & (port_trailer - 1u)) {
        dev_err(dp->this_dev, "Resolve trailer 0x%x -> invalid\n", trailer);
        return NULL;
    }

    /*
     * Management trailer matching CPU port means that frame was originally
     * sent by CPU from the CPU port net device, and thus mirrored back to CPU
     * (without IPO).
     */
    if (port_trailer == (cpu_port->trailer & ~deipce_get_macsec_trailer(dp))) {
        dev_resolve_dbg(dp->this_dev,
                        "Resolve trailer 0x%x %s -> %s (cpu -> ep)\n",
                        trailer, netdev_name(cpu_port->netdev),
                        netdev_name(dp->netdev));
        rx_frame->dev = dp->netdev;
        return cpu_port;
    }

    // Determine port number from management trailer.
    port_mask = port_trailer >> dp->trailer_offset;
    port_num = find_first_bit(&port_mask, DEIPCE_MAX_PORTS);
    if (port_num >= ARRAY_SIZE(dp->port)) {
        dev_err(dp->this_dev, "Resolve trailer 0x%x -> invalid\n", trailer);
        return NULL;
    }

    port = dp->port[port_num];
    if (!port) {
        // Refuse to handle frames from unknown ports.
        dev_err(dp->this_dev,
                "Resolve trailer 0x%x -> invalid (unknown port %u)\n",
                trailer, port_num);
        return NULL;
    }

    // Preemption verification frames are handled internally in driver.
    if (dp->features.preempt_ports & (1u << port_num)) {
        if (deipce_preempt_rx_frame(port, rx_frame, trailer))
            return NULL;
    }

    // We do not use Linux bridge for traffic.
#if 0
    // When e.g. attached to bridge, always use actual switch port.
    if (port->flags & DEIPCE_HAS_MASTER)
        rx_frame->dev = port->netdev;
        return port;
#endif

    // Check port own unicast address.
    if (ether_addr_equal(port->netdev->dev_addr, eth->h_dest) &&
        !ether_addr_equal(port->netdev->dev_addr,
                          cpu_port->netdev->dev_addr)) {
        dev_resolve_dbg(dp->this_dev,
                        "Resolve trailer 0x%x %s -> %s (port unicast)\n",
                        trailer, netdev_name(port->netdev),
                        netdev_name(port->netdev));
        rx_frame->dev = port->netdev;
        return port;
    }

    // Check reserved multicast addresses.
    if (deipce_match_reserved(eth->h_dest)) {
        dev_resolve_dbg(dp->this_dev,
                        "Resolve trailer 0x%x %s -> %s (reserved)\n",
                        trailer, netdev_name(port->netdev),
                        netdev_name(port->netdev));
        rx_frame->dev = port->netdev;
        return port;
    }

    // Pass all other frames from endpoint netdevice.
    dev_resolve_dbg(dp->this_dev,
                    "Resolve trailer 0x%x %s -> %s (endpoint)\n",
                    trailer, netdev_name(port->netdev),
                    netdev_name(dp->netdev));
    rx_frame->dev = dp->netdev;
    return port;
}

/**
 * Extend partial frame timestamper time to full time and convert to ktime_t.
 * @param ts_sec Bitwidth-limited timestamper time seconds part.
 * @param ts_nsec Timestamper time nanoseconds part.
 * @param phc_time Current PHC time for calculating all seconds bits.
 * @param delay_corr Delay correction in nanoseconds for the timestamp.
 * @param timestamp Place for converted timestamp.
 */
static int deipce_timestamper_to_ktime(uint64_t ts_sec, uint32_t ts_nsec,
                                       const struct timespec64 *phc_time,
                                       long int delay_corr,
                                       ktime_t *timestamp)
{
    struct timespec64 corr_ts;
    uint64_t full_sec = phc_time->tv_sec;

    /*
     * Get full seconds for bitwidth-limited timestamper seconds.
     * There is no way to know amount of time passed between timestamper
     * and current time from PTP clock, assume max. 1 s has passed.
     */
    ts_sec &= FRS_TS_SEC_MASK;
    if ((full_sec & 0x1ull) != (ts_sec & 0x1ull))
        full_sec--;
    if ((full_sec & FRS_TS_SEC_MASK) != ts_sec)
        return -EINVAL;

    /*
     * Use nanoseconds from timestamper to form complete timestamp,
     * corrected by delays.
     */
    ts_nsec &= FRS_TS_NSEC_MASK;
    set_normalized_timespec64(&corr_ts, full_sec,
                              (long int)ts_nsec + delay_corr);
    *timestamp = timespec64_to_ktime(corr_ts);

    return 0;
}

/**
 * Get frame timestamp from FRS (port 0) timestamper.
 * @param dp Device privates.
 * @param stamper_base Offset to timestamper registers.
 * @param phc_time Current PHC time for calculating all seconds bits.
 * @param timestamp Place for frame timestamp.
 */
static int deipce_get_timestamp(struct deipce_dev_priv *dp,
                                unsigned int stamper_base,
                                const struct timespec64 *phc_time,
                                ktime_t *timestamp)
{
    uint64_t sec;
    uint32_t nsec;
    int ret;

    sec = deipce_read_switch_reg(dp, stamper_base + FRS_TS_S_LO);
    nsec = deipce_read_switch_reg(dp, stamper_base + FRS_TS_NS_HI) << 16;
    nsec |= deipce_read_switch_reg(dp, stamper_base + FRS_TS_NS_LO);

    ret = deipce_timestamper_to_ktime(sec, nsec, phc_time, 0, timestamp);
    if (ret) {
        dev_ts_dbg(dp->this_dev,
                   "%s: TS mismatch: phc 0x%llx stamper 0x%llx\n",
                   stamper_base >= FRS_TS_RX_BASE ? "RX" : "TX",
                   (long long int)phc_time->tv_sec, sec);
        return ret;
    }

    dev_ts_dbg(dp->this_dev, "%s: TS %lli.%09li s\n",
               stamper_base >= FRS_TS_RX_BASE ? "RX" : "TX",
               (long long int)ktime_to_timespec64(*timestamp).tv_sec,
               ktime_to_timespec64(*timestamp).tv_nsec);

    return 0;
}

/**
 * Get frame timestamp from FRS port (!= 0) timestamper.
 * @param pp Port privates.
 * @param stamper_base Offset to timestamper registers.
 * @param phc_time Current PHC time for calculating all seconds bits.
 * @param delay_corr Amount of nanoseconds by which to correct
 * the timestamp.
 * @param timestamp Place for frame timestamp.
 */
static int deipce_get_port_timestamp(struct deipce_port_priv *pp,
                                     unsigned int stamper_base,
                                     const struct timespec64 *phc_time,
                                     long int delay_corr,
                                     ktime_t *timestamp)
{
    uint64_t sec;
    uint32_t nsec;
    int ret;

    sec = deipce_read_port_reg(pp, stamper_base + FRS_TS_S_LO);
    nsec = deipce_read_port_reg(pp, stamper_base + FRS_TS_NS_HI) << 16;
    nsec |= deipce_read_port_reg(pp, stamper_base + FRS_TS_NS_LO);

    ret = deipce_timestamper_to_ktime(sec, nsec, phc_time, delay_corr,
                                      timestamp);
    if (ret) {
        dev_ts_dbg(pp->dp->this_dev,
                   "%s %s: TS mismatch: phc 0x%llx stamper 0x%llx\n",
                   netdev_name(pp->netdev),
                   stamper_base >= PORT_TS_RX_BASE ? "RX" : "TX",
                   (long long int)phc_time->tv_sec, sec);
        return ret;
    }

    dev_ts_dbg(pp->dp->this_dev, "%s %s: TS %lli.%09li s\n",
               netdev_name(pp->netdev),
               stamper_base >= PORT_TS_RX_BASE ? "RX" : "TX",
               (long long int)ktime_to_timespec64(*timestamp).tv_sec,
               ktime_to_timespec64(*timestamp).tv_nsec);

    return 0;
}

/**
 * Read FRS RX timestamp.
 * @param dp FRS private data.
 * @param ptp_hdr Place for PTP header of the received frame.
 * @param timestamp Place for frame timestamp.
 */
static int deipce_rx_timestamp(struct deipce_dev_priv *dp,
                               uint8_t *ptp_hdr, ktime_t *timestamp)
{
    uint16_t *tmp = NULL;
    unsigned int i = 0;
    int ret = -ENOENT;

    if (!dp->irq_work_time_valid) {
        ret = deipce_time_get_time(dp->time, DEIPCE_TIME_SEL_TS,
                                   &dp->irq_work_time);
        if (ret)
            return ret;

        dp->irq_work_time_valid = true;
    }

    // Copy frame.
    tmp = (uint16_t *) ptp_hdr;
    for (i = 0; i < FRS_TS_HDR_LEN / 2; i++) {
        // LSB contains first byte.
        tmp[i] = __cpu_to_le16(deipce_read_switch_reg(
                        dp, FRS_TS_RX_HDR(dp->rx_stamper.next, i)));
    }

    ret = deipce_get_timestamp(dp, FRS_TS_RX_OFS(dp->rx_stamper.next),
                               &dp->irq_work_time, timestamp);

    return ret;
}

/**
 * Read sent PTP frame timestamp from FRS (port N TX, N != 0).
 * @param port FRS port privates.
 * @param ptp_hdr Place for PTP header of the sent frame.
 * @param timestamp Place for frame timestamp.
 */
static int deipce_port_tx_timestamp(struct deipce_port_priv *port,
                                    uint8_t *ptp_hdr, ktime_t *timestamp)
{
    struct deipce_dev_priv *dp = port->dp;
    uint16_t *tmp = (uint16_t *) ptp_hdr;
    unsigned int i = 0;
    int ret;

    if (!dp->irq_work_time_valid) {
        ret = deipce_time_get_time(dp->time, DEIPCE_TIME_SEL_TS,
                                   &dp->irq_work_time);
        if (ret)
            return ret;

        dp->irq_work_time_valid = true;
    }

    // Copy frame.
    for (i = 0; i < FRS_TS_HDR_LEN / 2; i++) {
        // LSB contains first byte.
        tmp[i] = __cpu_to_le16(deipce_read_port_reg(
                        port, PORT_TS_TX_HDR(port->tx_stamper.next, i)));
    }

    // 802.1AS: <egressTimestamp> = <egressMeasuredTimestamp> + <egressLatency>
    ret = deipce_get_port_timestamp(port, PORT_TS_TX_OFS(port->tx_stamper.next),
                                    &dp->irq_work_time,
                                    port->cur_delay.tx, timestamp);

    return ret;
}

/**
 * Read received PTP frame timestamp from FRS (port 0 TX).
 * @param dp FRS private data.
 * @param ptp_hdr PTP header of the received frame.
 * @param timestamp Place for frame timestamp.
 */
static int deipce_tx_timestamp(struct deipce_dev_priv *dp,
                               const uint8_t *ptp_hdr,
                               ktime_t *timestamp)
{
    struct timespec64 cur_time;
    const uint16_t *tmp = NULL;
    uint16_t tx_ts_ctrl = deipce_read_switch_reg(dp, FRS_REG_TS_CTRL_TX);
    uint16_t tx_ts_ctrl_enable = 0;
    unsigned int i = 0;
    uint16_t data = 0;
    bool found = false;
    int ret;

    dev_ts_dbg(dp->this_dev, "TX: TS_CTRL 0x%x 0x%x %i check\n",
               tx_ts_ctrl, 1u << dp->tx_stamper, dp->tx_stamper);

    // This assumes frames are seen in FRS output order.
    while (!(tx_ts_ctrl & (1u << dp->tx_stamper))) {
        // Timestamp is available.
        tx_ts_ctrl_enable = 1u << dp->tx_stamper;

        // Avoid loop.
        tx_ts_ctrl |= tx_ts_ctrl_enable;

        dev_ts_dbg(dp->this_dev, "TX: TS_CTRL 0x%x 0x%x %i available\n",
                   tx_ts_ctrl, tx_ts_ctrl_enable, dp->tx_stamper);

        // Check if timestamp header is the desired one.
        tmp = (uint16_t *) ptp_hdr;
        found = true;
        for (i = 0; i < FRS_TS_HDR_LEN / 2; i++) {
            data = deipce_read_switch_reg(dp, FRS_TS_TX_HDR(dp->tx_stamper, i));
            // LSB contains first byte.
            if (__le16_to_cpu(tmp[i]) != data) {
                found = false;
                break;
            }
        }

        if (found) {
            ret = deipce_time_get_time(dp->time, DEIPCE_TIME_SEL_TS, &cur_time);
            if (ret == 0)
                ret = deipce_get_timestamp(dp, FRS_TS_TX_OFS(dp->tx_stamper),
                                           &cur_time, timestamp);
        }

        // Always enable timestamper.
        deipce_write_switch_reg(dp, FRS_REG_TS_CTRL_TX, tx_ts_ctrl_enable);

        // Go to next entry.
        dp->tx_stamper = (dp->tx_stamper + 1) % FRS_TS_NUMBER_TIMESTAMPS;

        if (found)
            return ret;
    }

    return -ENOENT;
}

/**
 * Read received PTP frame timestamp from FRS (port N RX, N != 0).
 * @param port FRS port privates.
 * @param ptp_hdr PTP header of the received frame.
 * @param timestamp Place for frame timestamp.
 */
static int deipce_port_rx_timestamp(struct deipce_port_priv *port,
                                    const uint8_t *ptp_hdr,
                                    ktime_t *timestamp)
{
    struct deipce_dev_priv *dp = port->dp;
    struct timespec64 cur_time;
    const uint16_t *tmp = NULL;
    uint16_t rx_ts_ctrl = deipce_read_port_reg(port, PORT_REG_TS_CTRL_RX);
    uint16_t rx_ts_ctrl_enable = 0;
    uint16_t data = 0;
    unsigned int i = 0;
    bool found = false;
    int ret;

    dev_ts_dbg(dp->this_dev, "%s RX: TS_CTRL 0x%x 0x%x %i check\n",
               netdev_name(port->netdev),
               rx_ts_ctrl, 1u << port->rx_stamper, port->rx_stamper);

    // This assumes frames are seen in FRS input order.
    while (!(rx_ts_ctrl & (1u << port->rx_stamper))) {
        // Timestamp available
        rx_ts_ctrl_enable = 1u << port->rx_stamper;

        // Avoid loop.
        rx_ts_ctrl |= rx_ts_ctrl_enable;

        dev_ts_dbg(dp->this_dev,
                   "%s RX: TS_CTRL 0x%x 0x%x %i available\n",
                   netdev_name(port->netdev),
                   rx_ts_ctrl, rx_ts_ctrl_enable, port->rx_stamper);

        // Check if timestamp header is the desired one.
        tmp = (uint16_t *) ptp_hdr;
        found = true;
        for (i = 0; i < FRS_TS_HDR_LEN / 2; i++) {
            data = deipce_read_port_reg(port,
                                        PORT_TS_RX_HDR(port->rx_stamper, i));
            // LSB contains first byte.
            if (__le16_to_cpu(tmp[i]) != data) {
                found = false;
                break;
            }
        }

        /*
         * 802.1AS:
         * <ingressTimestamp> = <ingressMeasuredTimestamp> - <ingressLatency>
         */
        if (found) {
            ret = deipce_time_get_time(dp->time, DEIPCE_TIME_SEL_TS, &cur_time);
            if (ret == 0)
                ret = deipce_get_port_timestamp(
                        port,
                        PORT_TS_RX_OFS(port->rx_stamper),
                        &cur_time,
                        -(long int)port->cur_delay.rx,
                        timestamp);
        }

        // Always enable timestamper.
        deipce_write_port_reg(port, PORT_REG_TS_CTRL_RX, rx_ts_ctrl_enable);

        // Go to next entry.
        port->rx_stamper = (port->rx_stamper + 1) % FRS_TS_NUMBER_TIMESTAMPS;

        if (found)
            return ret;
    }

    return -ENOENT;
}

/**
 * Handle sent frame timestamp.
 * @param dp FRS private data.
 * @param stamper Stamper to to use.
 * @param frame PTP frame from FRS frame timestamp registers.
 * @param timestamp Frame timestamp.
 */
static void deipce_handle_tx_timestamp(struct deipce_dev_priv *dp,
                                       struct deipce_tx_stamper *stamper,
                                       const uint8_t *frame,
                                       ktime_t timestamp)
{
    struct sk_buff *skb = NULL;
    const uint8_t *ptp = NULL;
    unsigned int i = 0;
    unsigned long int flags = 0;
    struct skb_shared_hwtstamps hwtstamps = {
        .hwtstamp = timestamp,
    };

#ifdef DEBUG_FRS_TS_DUMP
    print_hex_dump_bytes("TX: TS frame: ", DUMP_PREFIX_OFFSET,
                         frame, FRS_TS_HDR_LEN);
#endif

    spin_lock_irqsave(&stamper->lock, flags);

    for (i = 0; i < FRS_TS_NUMBER_TIMESTAMPS; i++) {
        skb = stamper->orig_skb[i];
        if (!skb)
            continue;

        ptp = deipce_skb_get_ptp_hdr(dp, skb);
        if (!ptp)
            continue;

#ifdef DEBUG_FRS_TS_DUMP
        print_hex_dump_bytes("TX: TS cmp skb: ", DUMP_PREFIX_OFFSET,
                             //ptp, FRS_TS_HDR_LEN);
                             eth_hdr(skb), skb->len);
#endif

        if (memcmp(frame, ptp, FRS_TS_HDR_LEN) == 0) {
            // Found it.
            break;
        }
    }

    if (i < FRS_TS_NUMBER_TIMESTAMPS) {
        stamper->orig_skb[i] = NULL;
    }
    else {
        skb = NULL;
    }

    spin_unlock_irqrestore(&stamper->lock, flags);

    if (!skb) {
        dev_ts_dbg(dp->this_dev, "TX: TS skb not found\n");
        return;
    }

    dev_ts_dbg(dp->this_dev, "TX: TS got msg 0x%02x from slot %u %s\n",
               ptp[0], i,
               netdev_name(stamper == &dp->rx_stamper ?
                           dp->netdev : skb->dev));

    // Restore timestamping flags.
    skb_shinfo(skb)->tx_flags |= SKBTX_HW_TSTAMP;
    skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

    dev_ts_dbg(dp->this_dev, "TX: TS skb %p slot %u\n", skb, i);

    // Must not pass back PTP sync frames in onestep mode.
    switch (dp->tstamp_cfg.tx_type) {
    case HWTSTAMP_TX_ONESTEP_SYNC:
        if (deipce_is_ptp_sync(ptp[0])) {
            dev_ts_dbg(dp->this_dev, "TX: TS onestep, ignore sync\n");
            dev_kfree_skb_any(skb);
        }
        else {
            skb_complete_tx_timestamp(skb, &hwtstamps);
        }
        break;
    case HWTSTAMP_TX_ON:
        skb_complete_tx_timestamp(skb, &hwtstamps);
        break;
    case HWTSTAMP_TX_OFF:
    default:
        dev_ts_dbg(dp->this_dev, "TX: TS off\n");;
        dev_kfree_skb_any(skb);
    }

    return;
}

/**
 * Get sent frame timestamp from FRS (port 0 RX).
 * @param dp FRS private data.
 */
static void deipce_get_rx_timestamp(struct deipce_dev_priv *dp)
{
    ktime_t timestamp = ktime_set(0, 0);
    uint8_t frame[FRS_TS_HDR_LEN] = { 0 };
    uint16_t rx_ts_ctrl = deipce_read_switch_reg(dp, FRS_REG_TS_CTRL_RX);
    uint16_t rx_ts_ctrl_enable;
    int ret;

    // Check port 0 RX frame timestamps from switch registers.
    // Port N RX timestamps are handled when frame is received from MAC.

    dev_ts_dbg(dp->this_dev, "RX: TS_CTRL 0x%x 0x%x %i check\n",
               rx_ts_ctrl, 1u << dp->rx_stamper.next, dp->rx_stamper.next);

    while (!(rx_ts_ctrl & (1u << dp->rx_stamper.next))) {
        // Timestamp is available.
        rx_ts_ctrl_enable = 1u << dp->rx_stamper.next;

        // Avoid loop.
        rx_ts_ctrl |= rx_ts_ctrl_enable;

        dev_ts_dbg(dp->this_dev, "RX: TS_CTRL 0x%x 0x%x %i available\n",
                   rx_ts_ctrl, rx_ts_ctrl_enable, dp->rx_stamper.next);

        ret = deipce_rx_timestamp(dp, frame, &timestamp);

        // Always enable timestamper.
        deipce_write_switch_reg(dp, FRS_REG_TS_CTRL_RX, rx_ts_ctrl_enable);

        if (ret == 0)
            deipce_handle_tx_timestamp(dp, &dp->rx_stamper, frame, timestamp);

        // Go to next entry.
        dp->rx_stamper.next =
            (dp->rx_stamper.next + 1) % FRS_TS_NUMBER_TIMESTAMPS;
    }

    return;
}

/**
 * Get sent frame timestamp from FRS (port N != 0 TX).
 * @param dp FRS private data.
 */
static void deipce_get_tx_timestamp(struct deipce_dev_priv *dp)
{
    ktime_t timestamp = ktime_set(0, 0);
    uint8_t frame[FRS_TS_HDR_LEN] = { 0 };
    struct deipce_port_priv *port = NULL;
    unsigned int port_num = 0;
    uint16_t tx_ts_ctrl;
    uint16_t tx_ts_ctrl_enable;
    int ret;

    // Check port N (N != 0) TX frame timestamps from port registers.
    // Port 0 TX timestamps are handled when frame is received from MAC.
    for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
        port = dp->port[port_num];
        if (!port)
            continue;

        if (!(dp->features.ts_ports & (1u << port_num)))
            continue;

        tx_ts_ctrl = deipce_read_port_reg(port, PORT_REG_TS_CTRL_TX);

        dev_ts_dbg(dp->this_dev, "%s TX: TS_CTRL 0x%x check\n",
                   netdev_name(port->netdev), tx_ts_ctrl);

        while (!(tx_ts_ctrl & (1u << port->tx_stamper.next))) {
            // Timestamp is available.
            tx_ts_ctrl_enable = 1u << port->tx_stamper.next;

            // Avoid loop.
            tx_ts_ctrl |= tx_ts_ctrl_enable;

            dev_ts_dbg(dp->this_dev, "%s TX: TS_CTRL 0x%x 0x%x %i available\n",
                       netdev_name(port->netdev),
                       tx_ts_ctrl, tx_ts_ctrl_enable, port->tx_stamper.next);

            ret = deipce_port_tx_timestamp(port, frame, &timestamp);

            // Always enable timestamper.
            deipce_write_port_reg(port, PORT_REG_TS_CTRL_TX, tx_ts_ctrl_enable);

            if (ret == 0)
                deipce_handle_tx_timestamp(dp, &port->tx_stamper, frame,
                                           timestamp);

            // Go to next entry.
            port->tx_stamper.next =
                (port->tx_stamper.next + 1) % PORT_TS_NUMBER_TIMESTAMPS;
        }
    }

    return;
}

/**
 * Update received PTP frame timestamp from FRS (port 0 TX or port N RX)
 * to sk_buff.
 * @param dp FRS private data of the FRS with CPU port.
 * @param port Port on which the frame was actually received.
 * @param rx_frame Received frame whose HW RX PTP timestamp to update.
 * @param ptp Common PTP header.
 */
static void deipce_update_rx_timestamp(struct deipce_dev_priv *dp,
                                       struct deipce_port_priv *port,
                                       struct sk_buff *rx_frame,
                                       const uint8_t *ptp)
{
    ktime_t *timestamp = &skb_hwtstamps(rx_frame)->hwtstamp;
    int ret;

    dev_ts_dbg(dp->this_dev, "%s RX: skb ptp %p\n",
               netdev_name(rx_frame->dev), ptp);

    // Get timestamp
    switch (ptp[0] & 0x0f) {
    case 0:
    case 1:
    case 2:
    case 3:
        // Event messages, timestamped by FRS.
        if (dp->use_port_ts &&
            (dp->features.ts_ports & (1u << port->port_num)))
            ret = deipce_port_rx_timestamp(port, ptp, timestamp);
        else
            ret = deipce_tx_timestamp(dp, ptp, timestamp);
        if (ret == 0) {
            // Timestamp is set.
        }
        break;
    }

    return;
}

/**
 * Receive frames from CPU port (via underlying Ethernet MAC).
 * Note: already called with rcu_read_lock.
 * @param pskb Pointer to pointer to received frame.
 * @return rx_handler_result value.
 */
static rx_handler_result_t deipce_handle_frame(struct sk_buff **pskb)
{
    struct sk_buff *skb = *pskb;
    const struct net_device *netdev = skb->dev;
    struct deipce_dev_priv *dp = NULL;
    struct deipce_port_priv *port = NULL;
    const uint8_t *ptp = NULL;

    if (!netdev) {
        // No input device
        pr_err(DRV_NAME ": No input device\n");
        goto drop;
    }

    // Device with CPU port.
    dp = rcu_dereference(netdev->rx_handler_data);

    dev_resolve_dbg(dp->this_dev, "Receiving frame from %s pkt_type 0x%x\n",
                    netdev_name(netdev), skb->pkt_type);

    if (unlikely(skb->pkt_type == PACKET_LOOPBACK))
        return RX_HANDLER_PASS;

    skb = skb_share_check(skb, GFP_ATOMIC);
    if (!skb)
        return RX_HANDLER_CONSUMED;

    port = deipce_resolve_rx_netdev(dp, skb);
    if (!port)
        goto drop;

    dev_resolve_dbg(dp->this_dev, "Receiving frame via %s\n",
                    netdev_name(skb->dev));

    // Handle HW RX PTP timestamps.
    ptp = deipce_skb_get_ptp_hdr(dp, skb);
    if (ptp) {
        deipce_update_rx_timestamp(dp, port, skb, ptp);
    }

    deipce_rx_frame(skb);
    return RX_HANDLER_CONSUMED;

drop:
    dev_kfree_skb_any(skb);

    return RX_HANDLER_CONSUMED;
}

/**
 * FRS interrupt work handler.
 * This is used to deal with FRS interrupts using workqueue.
 */
static void deipce_interrupt_work(struct work_struct *work)
{
    struct deipce_dev_priv *dp =
        container_of(work, struct deipce_dev_priv, irq_work);
    uint16_t intstat = deipce_read_switch_reg(dp, FRS_REG_INTSTAT);

    deipce_write_switch_reg(dp, FRS_REG_INTSTAT, ~intstat);

    dp->irq_work_time_valid = false;

    if (intstat & FRS_INT_RX_TSTAMP) {
        // PTP event frame in RX frame timestamp registers.
        dp->stats.rx_stamp++;
        if (!dp->use_port_ts)
            deipce_get_rx_timestamp(dp);
    }
    if (intstat & FRS_INT_TX_TSTAMP) {
        // PTP event frame in TX frame timestamp registers.
        dp->stats.tx_stamp++;
        if (dp->use_port_ts)
            deipce_get_tx_timestamp(dp);
    }

    if (intstat & FRS_INT_RX_ERROR) {
        // RX error happened
        dp->stats.rx_error++;
    }
    if (intstat & FRS_INT_CONGESTED) {
        // Congested situation
        dp->stats.congested++;
    }

    // Reenable interrupts from FRS.
    deipce_write_switch_reg(dp, FRS_REG_INTMASK, deipce_get_intmask(dp));

    return;
}

/**
 * FRS interrupt handler for dealing with interrupts from workqueue.
 * Kicks a work to deal with the interrupt.
 * @param irq IRQ number.
 * @param device Device data.
 * @return IRQ return value.
 */
static irqreturn_t deipce_interrupt(int irq, void *device)
{
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct deipce_dev_priv *dp = device;
    uint16_t intmask;
    uint16_t intstat;
    bool work_queued;

    intmask = deipce_read_switch_reg(dp, FRS_REG_INTMASK);
    if (!intmask)
        return IRQ_NONE;

    intstat = deipce_read_switch_reg(dp, FRS_REG_INTSTAT);
    if (!(intmask & intstat))
        return IRQ_NONE;

    // Disable interrupts from FRS until we have handled them.
    deipce_write_switch_reg(dp, FRS_REG_INTMASK, 0);

    work_queued = queue_work(drv->wq_high, &dp->irq_work);
    if (!work_queued) {
        /*
         * Work has already enabled interrupts but is still running
         * (not yet finished). Make sure interrupt is enabled until
         * a new work is started.
         */
        deipce_write_switch_reg(dp, FRS_REG_INTMASK, deipce_get_intmask(dp));
    }

    return IRQ_HANDLED;
}

/**
 * Initialize switch interrupt handling.
 * @param dp Switch privates.
 */
int deipce_irq_init(struct deipce_dev_priv *dp)
{
    int ret;

    dev_dbg(dp->this_dev, "%s() IRQ %u\n", __func__, dp->irq);

    if (!dp->irq) {
        // Allow operation without IRQ.
        dev_warn(dp->this_dev, "No IRQ defined\n");
        return 0;
    }

    INIT_WORK(&dp->irq_work, &deipce_interrupt_work);
    ret = request_irq(dp->irq, &deipce_interrupt, IRQF_SHARED, DRV_NAME, dp);
    if (ret) {
        dev_err(dp->this_dev, "unable to allocate IRQ %u, error 0x%x",
                dp->irq, ret);
        return ret;
    }

    return 0;
}

/**
 * Enable interrupts from switch.
 * @param dp Switch privates.
 */
void deipce_irq_enable(struct deipce_dev_priv *dp)
{
    if (!dp->irq)
        return;

    // Enable needed interrupts.
    deipce_write_switch_reg(dp, FRS_REG_INTMASK, deipce_get_intmask(dp));
    return;
}

/**
 * Disable interrupts from switch.
 * @param dp Switch privates.
 */
void deipce_irq_disable(struct deipce_dev_priv *dp)
{
    if (!dp->irq)
        return;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    // Disable interrupt from FRS. First ensure interrupt will not fire.
    disable_irq(dp->irq);

    cancel_work_sync(&dp->irq_work);
    deipce_write_switch_reg(dp, FRS_REG_INTMASK, 0);
    deipce_write_switch_reg(dp, FRS_REG_INTSTAT, 0);

    enable_irq(dp->irq);

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

/**
 * Cleanup switch interrupt handling.
 * @param dp Switch privates.
 */
void deipce_irq_cleanup(struct deipce_dev_priv *dp)
{
    if (!dp->irq)
        return;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    free_irq(dp->irq, dp);

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

/**
 * Initialize netdevif interface, to receive frames from the used Ethernet
 * interface.
 * @param dp private dev data
 * @return 0 on success ore negative Linux error code.
 */
int deipce_netdevif_init(struct deipce_dev_priv *dp)
{
    int ret = -ENODEV;

    dev_dbg(dp->this_dev, "Init netdevif\n");

    // Register rx handler for underlying MAC netdevice.
    rtnl_lock();
    ret = netdev_rx_handler_register(dp->real_netdev,
                                     &deipce_handle_frame, dp);
    rtnl_unlock();

    if (ret) {
        dev_err(dp->this_dev, "netdev_rx_handler_register failed for %s\n",
                netdev_name(dp->real_netdev));
    }

    return ret;
}

/**
 * Cleanup remembered sk_buffs waiting for a timestamp.
 * @param stamper Stamper to cleanup.
 */
void deipce_netdevif_cleanup_stamper(struct deipce_tx_stamper *stamper)
{
    unsigned long int flags = 0;
    struct sk_buff *skb = NULL;
    unsigned int i;

    spin_lock_irqsave(&stamper->lock, flags);

    for (i = 0; i < FRS_TS_NUMBER_TIMESTAMPS; i++) {
        skb = stamper->orig_skb[i];
        if (!skb)
            continue;

        stamper->orig_skb[i] = NULL;
        dev_kfree_skb(skb);
    }

    spin_unlock_irqrestore(&stamper->lock, flags);

    return;
}

/**
 * Close netdevif interface.
 * @param dp private dev data
 */
void deipce_netdevif_cleanup(struct deipce_dev_priv *dp)
{
    struct deipce_port_priv *port = NULL;
    unsigned int port_num;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    // Unregister rx handler for MAC netdevice.
    rtnl_lock();
    netdev_rx_handler_unregister(dp->real_netdev);
    rtnl_unlock();

    deipce_netdevif_cleanup_stamper(&dp->rx_stamper);

    for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
        port = dp->port[port_num];
        if (!port)
            continue;

        deipce_netdevif_cleanup_stamper(&port->tx_stamper);
    }

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

