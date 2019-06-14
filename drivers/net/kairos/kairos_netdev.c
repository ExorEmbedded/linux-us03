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
#include <linux/version.h>
#include <linux/etherdevice.h>
#include <linux/phy.h>
#include <linux/if_bridge.h>
#include <linux/platform_device.h>
#include <linux/of_mdio.h>
#include <linux/sched.h>

#include "deipce_port_sysfs.h"
#include "deipce_fsc_sysfs.h"
#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_if.h"
#include "deipce_ioctl.h"
#include "deipce_ethtool.h"
#include "deipce_adapter.h"
#include "deipce_sfp.h"
#include "deipce_hw.h"
#include "deipce_switchdev.h"
#include "deipce_preempt.h"
#include "deipce_shaper.h"
#include "deipce_netdevif.h"
#include "deipce_netdev.h"

/// Default NETIF message level
#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV|NETIF_MSG_PROBE|NETIF_MSG_LINK)

// Module parameters
static int debug = -1;                  ///< -1 means defaults above
module_param(debug, int, S_IRUGO);
MODULE_PARM_DESC(debug, "NETIF message level.");

/// Port statistics capture interval in jiffies
#define DEIPCE_STATS_CAPTURE_INTERVAL (1*HZ)

/// Packets should be transmitted in 60 seconds
#define TX_TIMEOUT (60*HZ)

// phydev_name appeared in Linux 4.5.
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)

static inline const char *phydev_name(const struct phy_device *phydev)
{
    return dev_name(&phydev->dev);
}

static inline struct device *phydev_dev(struct phy_device *phydev)
{
    return &phydev->dev;
}

#else

static inline struct device *phydev_dev(struct phy_device *phydev)
{
    return &phydev->mdio.dev;
}

#endif

/**
 * Set FRS port link mode to account for anything that depends on link speed.
 * Link mode mutex must be held when calling this.
 * @param netdev Netdevice associated with an FRS port.
 * @param link_mode New link mode for port, ignored if link mode is forced.
 */
static int deipce_set_port_mode(struct net_device *netdev,
                                enum link_mode link_mode)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    int ret;

    ret = deipce_set_port_state(netdev, link_mode);

    deipce_update_delays(pp);

    if (dp->features.preempt_ports & (1u << pp->port_num))
        deipce_preempt_update_link(pp, link_mode);

    return ret;
}

/**
 * Adjust FRS port link mode taking net device running status into account.
 * Link mode mutex must be held when calling this.
 * @param netdev Netdevice associated with an FRS port.
 * @param link_mode New link mode for port, ignored if link mode is forced.
 */
int deipce_update_port_mode(struct net_device *netdev,
                            enum link_mode link_mode)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    int ret = 0;

    if (!netif_running(netdev)) {
        link_mode = LM_DOWN;
    }
    else if (np->force_link_mode != LM_DOWN) {
        if ((pp->ext_phy.phydev && !pp->ext_phy.phydev->link) ||
            (pp->sfp.phy.phydev && !pp->sfp.phy.phydev->link))
            link_mode = LM_DOWN;
        else
            link_mode = np->force_link_mode;
    }

    if (link_mode != np->link_mode)
        ret = deipce_set_port_mode(netdev, link_mode);

    return ret;
}

/**
 * PHY device callback function to adjust FRS port link mode.
 * This is called from PHY polling thread.
 * @param netdev Netdevice associated with an FRS port.
 */
static void deipce_phy_adjust_link(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct phy_device *phydev;

    // Ignore information from PHY when using external/automatic signals.
    if (pp->flags & DEIPCE_PORT_SPEED_EXT)
        return;

    mutex_lock(&np->link_mode_lock);

    phydev = pp->ext_phy.phydev;
    if (phydev) {
        enum link_mode link_mode = deipce_get_phy_link_mode(phydev);

        netdev_dbg(netdev, "PHY link %s autoneg %s speed %u %s"
                   " link mode %i forced %i"
                   " supported 0x%x adv 0x%x lpa 0x%x state %i\n",
                   phydev->link ? "UP" : "DOWN",
                   phydev->autoneg ? "ON" : "OFF",
                   phydev->speed,
                   phydev->duplex == DUPLEX_FULL ? "full-duplex" :
                   (phydev->duplex == DUPLEX_HALF ? "half-duplex" : "unknown"),
                   link_mode, np->force_link_mode,
                   phydev->supported, phydev->advertising,
                   phydev->lp_advertising,
                   phydev->state);

        deipce_update_port_mode(netdev, link_mode);
    }

    mutex_unlock(&np->link_mode_lock);

    return;
}

/**
 * SFP PHY device callback function to adjust FRS port link mode.
 * This is called from PHY polling thread.
 * @param netdev Netdevice associated with an FRS port.
 */
static void deipce_sfp_phy_adjust_link(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct phy_device *phydev;

    // Ignore information from PHY when using external/automatic signals.
    if (pp->flags & DEIPCE_PORT_SPEED_EXT)
        return;

    mutex_lock(&np->link_mode_lock);

    phydev = pp->sfp.phy.phydev;
    if (phydev) {
        enum link_mode link_mode = deipce_get_phy_link_mode(phydev);

        netdev_dbg(netdev, "SFP PHY link %s autoneg %s speed %u %s"
                   " link mode %i forced %i"
                   " supported 0x%x adv 0x%x lpa 0x%x state %i\n",
                   phydev->link ? "UP" : "DOWN",
                   phydev->autoneg ? "ON" : "OFF",
                   phydev->speed,
                   phydev->duplex == DUPLEX_FULL ? "full-duplex" :
                   (phydev->duplex == DUPLEX_HALF ? "half-duplex" : "unknown"),
                   link_mode, np->force_link_mode,
                   phydev->supported, phydev->advertising,
                   phydev->lp_advertising,
                   phydev->state);

        /*
         * Use notifications from primary PHY to avoid periods of
         * conflicting information.
         */
        if (!pp->ext_phy.phydev) {
            deipce_update_port_mode(netdev, link_mode);
        }
        else {
            // Revert conflicting netif_carrier_on from PHY framework.
            if (!pp->ext_phy.phydev->link)
                netif_carrier_off(netdev);
        }
    }

    mutex_unlock(&np->link_mode_lock);

    return;
}

/**
 * Helper function to try to connect FRS port netdevice to PHY device.
 * @param netdev FRS port net device.
 * @param phy PHY context of PHY to connect to.
 * @param adjust_link Callback function for PHY to inform link changes.
 * @return true if successfully connected, false otherwise.
 */
static bool deipce_phy_connect(struct net_device *netdev,
                               struct deipce_phy *phy,
                               void (*adjust_link)(struct net_device *netdev))
{
    struct phy_device *orig_phydev = netdev->phydev;

    if (!phy->node) {
        netdev_dbg(netdev, "No PHY configured\n");
        return false;
    }

    // We may be attaching more than one PHY device to netdev.
    netdev->phydev = NULL;

    phy->phydev = of_phy_connect(netdev, phy->node, adjust_link, 0,
                                 phy->interface);
    if (!phy->phydev) {
        netdev_dbg(netdev, "Failed to attach PHY\n");
        goto out;
    }

    // Save original supported features.
    phy->orig_supported = phy->phydev->supported;

    netdev_info(netdev, "Attached PHY driver [%s] (mii_bus:phy_addr=%s)\n",
                phy->phydev->drv->name, phydev_name(phy->phydev));

out:
    if (orig_phydev)
        netdev->phydev = orig_phydev;

    return phy->phydev != NULL;
}

/**
 * Disconnect PHY device from FRS port.
 * @param netdev FRS port net device.
 * @param phy PHY context of PHY to disconnect.
 * @param alt_phydev Alternate PHY device to leave net device pointed to.
 */
static void deipce_phy_disconnect(struct net_device *netdev,
                                  struct deipce_phy *phy,
                                  struct phy_device *alt_phydev)
{
    int ret = 0;

    // PHY device may already be going away, don't access its pointers here.
    netdev_info(netdev, "Detach PHY (mii_bus:phy_addr=%s)\n",
                phydev_name(phy->phydev));

    netif_carrier_off(netdev);

    // We may be attached to more than one PHY device.
    netdev->phydev = phy->phydev;
    phy_stop(phy->phydev);

    if (phy->orig_supported) {
        // Restore original supported features.
        phy->phydev->supported = phy->orig_supported;
        phy->phydev->advertising = phy->orig_supported;
    }

    /*
     * In addition to phy_disconnect, release PHY from its driver
     * (temporarily), that allows SFP I2C driver to safely remove PHY and
     * thus handle SFP module changes. Reattach PHY device to a driver
     * immediately so that correct driver will still be used with it
     * if PHY device is not going to be removed entirely from system.
     */

    get_device(phydev_dev(phy->phydev));
    phy_disconnect(phy->phydev);
    device_release_driver(phydev_dev(phy->phydev));
    ret = device_attach(phydev_dev(phy->phydev));
    put_device(phydev_dev(phy->phydev));
    phy->phydev = NULL;

    netdev->phydev = alt_phydev;

    WARN_ON(ret < 0);

    return;
}

/**
 * Helper function to try to connect FRS port netdevice to SFP PHY device.
 * @param pp FRS port privates.
 * @return true if successfully connected to SFP PHY, false otherwise.
 */
static inline bool deipce_phy_connect_sfp(struct deipce_port_priv *pp)
{
    // Connect to SFP PHY, if configured.
    return deipce_phy_connect(pp->netdev, &pp->sfp.phy,
                              &deipce_sfp_phy_adjust_link);
}

/**
 * Try to connect FRS port netdevice to PHY and SFP PHY device(s).
 * @param pp FRS port privates.
 * @return true if successfully connected to PHY(s), false otherwise.
 */
static bool deipce_phy_connect_all(struct deipce_port_priv *pp)
{
    // Connect to external PHY, if configured.
    deipce_phy_connect(pp->netdev, &pp->ext_phy,
                       &deipce_phy_adjust_link);

    // Connect to SFP PHY, if configured.
    deipce_phy_connect_sfp(pp);

    return pp->ext_phy.phydev || pp->sfp.phy.phydev;
}

/**
 * Detect if PHY is present.
 * When e.g. SFP is removed, PHY register accesses fail and cause the PHY
 * to be put in PHY_HALTED state by the PHY framework.
 * @param phy PHY context of PHY to check.
 * @return true if PHY is still present.
 */
static inline bool deipce_is_phy_present(struct deipce_phy *phy)
{
    struct phy_device *phydev = phy->phydev;
    bool present = false;

    if (phydev) {
        mutex_lock(&phydev->lock);

        present = phydev->state != PHY_HALTED;

        mutex_unlock(&phydev->lock);
    }

    return present;
}

/**
 * Link polling helper function to acquire locks.
 * @param pp FRS port privates.
 * @param locked Placed for already locked flag.
 */
static void deipce_poll_lock(struct deipce_port_priv *pp, bool *locked)
{
    if (!*locked) {
        struct net_device *netdev = pp->netdev;
        struct deipce_netdev_priv *np = netdev_priv(netdev);

        /*
         * Prevent races with PHY callbacks.
         * Acquire mutexes in correct (the same) order.
         * Note that PHY polling calls adjust_link with
         * PHY lock held.
         */

        if (pp->ext_phy.phydev)
            mutex_lock(&pp->ext_phy.phydev->lock);
        mutex_lock(&np->link_mode_lock);

        *locked = true;
    }

    return;
}

/**
 * Link polling helper function to release locks.
 * @param pp FRS port privates.
 * @param locked Placed for locked flag.
 */
static void deipce_poll_unlock(struct deipce_port_priv *pp, bool *locked)
{
    if (*locked) {
        struct net_device *netdev = pp->netdev;
        struct deipce_netdev_priv *np = netdev_priv(netdev);

        mutex_unlock(&np->link_mode_lock);
        if (pp->ext_phy.phydev)
            mutex_unlock(&pp->ext_phy.phydev->lock);
        *locked = false;
    }

    return;
}

/**
 * Work function to check link state.
 * If PHY is present, actual link state from PHY framework is used,
 * otherwise this function checks link state from port adapter.
 * Detect SFP module changes, too, and connect to or disconnect from
 * SFP PHY as needed.
 * Reinitialize also port adapter when required.
 * @param work Port work structure.
 */
static void deipce_poll_link(struct work_struct *work)
{
    struct deipce_port_priv *pp =
        container_of(work, struct deipce_port_priv, check_link.work);
    struct deipce_phy *sfp_phy = &pp->sfp.phy;
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    struct net_device *netdev = pp->netdev;
    enum link_mode link_mode;
    bool restart_ext_phy = false;

    // Delay locking until needeed to reduce latencies on PHY polling.
    // Be careful with locking here.
    bool locked = false;

    // Detect SFP module changes.
    if (pp->medium_type == DEIPCE_MEDIUM_SFP) {
        // Detect SFP module from SFP EEPROM.
        if (pp->sfp.eeprom) {
            enum deipce_sfp_type sfp = deipce_detect_sfp(pp);
            if (sfp != pp->sfp.type) {
                deipce_poll_lock(pp, &locked);

                deipce_set_sfp(pp, sfp);

                // Trigger complete SFP PHY reinitialization (see below).
                if (sfp_phy->phydev) {
                    deipce_phy_disconnect(pp->netdev, sfp_phy,
                                          pp->ext_phy.phydev);
                }

                // Adjust adapter to new SFP module.
                deipce_init_adapter(pp);

                restart_ext_phy = true;

                /*
                 * Do dot reconnect to PHY at the same iteration,
                 * let it be removed completely from system.
                 */
                goto unlock_and_out;
            }
        }

        // Handle connection to SFP PHY.
        if (pp->flags & DEIPCE_HAS_SFP_PHY) {
            /*
             * This must be checked before acquiring link mode mutex
             * to avoid deadlock.
             */
            bool sfp_phy_present = deipce_is_phy_present(sfp_phy);

            deipce_poll_lock(pp, &locked);

            if (sfp_phy->phydev) {
                if (!sfp_phy_present) {
                    // SFP PHY disappeared, e.g. SFP module removed.
                    deipce_phy_disconnect(pp->netdev, sfp_phy,
                                          pp->ext_phy.phydev);
                    deipce_init_adapter(pp);

                    restart_ext_phy = true;
                }
            }
            else {
                if (deipce_phy_connect_sfp(pp)) {
                    // SFP PHY appeared, e.g. SFP module plugged in.
                    deipce_init_adapter(pp);

                    // Must release mutex before phy_start.
                    deipce_poll_unlock(pp, &locked);

                    phy_start(sfp_phy->phydev);

                    restart_ext_phy = true;
                    goto out;
                }
            }
        }
    }

    // Take link status from adapter when there is no PHY,
    // or from external/automatic speed selection signals.
    if ((pp->flags & DEIPCE_PORT_SPEED_EXT) ||
        (!pp->ext_phy.phydev && !sfp_phy->phydev &&
         pp->adapter.ops.check_link)) {

        deipce_poll_lock(pp, &locked);

        // Adapter interface might have changed already.
        // In any case external/automatic signals override everything else.
        if (pp->flags & DEIPCE_PORT_SPEED_EXT) {
            link_mode = deipce_get_ext_link_mode(pp);
            deipce_update_port_mode(netdev, link_mode);
        }
        else if (pp->adapter.ops.check_link) {
            link_mode = pp->adapter.ops.check_link(pp);
            deipce_update_port_mode(netdev, link_mode);
        }
    }

unlock_and_out:
    deipce_poll_unlock(pp, &locked);

out:
    /*
     * Inform PHY driver about interface changes
     * by restarting auto-negotiation.
     */
    if (pp->ext_phy.phydev && restart_ext_phy) {
        phy_start_aneg(pp->ext_phy.phydev);
    }

    // Reschedule check.
    queue_delayed_work(drv->wq_low, &pp->check_link,
                       DEIPCE_LINK_CHECK_INTERVAL);

    return;
}

/**
 * Work function to capture port statistics.
 * @param work Port work structure.
 */
static void deipce_capture_stats(struct work_struct *work)
{
    struct deipce_port_priv *pp =
        container_of(work, struct deipce_port_priv, capture_stats.work);
    struct deipce_drv_priv *drv = deipce_get_drv_priv();

    mutex_lock(&pp->stats_lock);

    deipce_update_port_stats(pp);

    mutex_unlock(&pp->stats_lock);

    // Reschedule capture.
    queue_delayed_work(drv->wq_low, &pp->capture_stats,
                       DEIPCE_STATS_CAPTURE_INTERVAL);

    return;
}

/**
 * Returns statistics of the network interface.
 * @param dev Netdevice.
 * @return statistics.
 */
static struct net_device_stats *deipce_get_stats(struct net_device *netdev)
{
    static struct net_device_stats stats;
    struct deipce_netdev_priv *np = netdev_priv(netdev);

    // Start from local statistics
    stats = np->stats;

    return &stats;
}

/**
 * Send a frame with management trailer.
 * @param skb Buffer to send.
 * @param dev netdevice.
 * @param trailer Management trailer to use.
 * @return 0 on success or negative error code.
 */
static netdev_tx_t deipce_start_xmit(struct sk_buff *skb,
                                     struct net_device *netdev,
                                     unsigned int trailer)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;
    unsigned int skb_len = skb->len;
    int ret = 0;

    // Transmit buffer.
    ret = deipce_xmit(dp, skb, trailer);
    if (ret == 0) {
        np->stats.tx_packets++;
        np->stats.tx_bytes += skb_len;
    }

    return NETDEV_TX_OK;
}

/**
 * Send a frame from switch port netdevice.
 * @param skb Buffer to send.
 * @param dev Switch port netdevice.
 * @return 0 on success or negative error code.
 */
static netdev_tx_t deipce_port_start_xmit(struct sk_buff *skb,
                                          struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;

    return deipce_start_xmit(skb, netdev, pp->trailer);
}

/**
 * Send a frame from endpoint netdevice.
 * @param skb Buffer to send.
 * @param dev Endpoint netdevice.
 * @return 0 on success or negative error code.
 */
static netdev_tx_t deipce_ep_start_xmit(struct sk_buff *skb,
                                        struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;

    return deipce_start_xmit(skb, netdev, deipce_get_macsec_trailer(dp));
}

/**
 * Netdevice timeout, currently unimplemented.
 */
static void deipce_netdev_tx_timeout(struct net_device *netdev)
{
    netdev_dbg(netdev, "Netdev timeout\n");
}

/**
 * Change the MAC address, the address can be changed also on-the-fly.
 */
static int deipce_set_mac_address(struct net_device *netdev, void *p)
{
    struct sockaddr *addr = p;

    netdev_printk(KERN_DEBUG, netdev, "Set MAC address %pM\n", addr->sa_data);

    if (!is_valid_ether_addr(addr->sa_data)) {
        return -EADDRNOTAVAIL;
    }

    memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

    return 0;
}

/**
 * Pass received frame to OS.
 * @param rx_frame Received frame, with dev set to resolved netdevice.
 */
void deipce_rx_frame(struct sk_buff *rx_frame)
{
    struct net_device *netdev = rx_frame->dev;
    struct deipce_netdev_priv *np = netdev_priv(netdev);

    if (!netif_carrier_ok(netdev)) {
        netdev_dbg(netdev, "%s(): No carrier\n", __func__);
        dev_kfree_skb(rx_frame);
        np->stats.rx_errors++;
        return;
    }

#ifdef CONFIG_NET_SWITCHDEV
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0)
    rx_frame->offload_fwd_mark = netdev->offload_fwd_mark;
#else
    rx_frame->offload_fwd_mark = 1;
#endif
#endif

    /*
     * If MAC address has been changed for a port, unicast frames are
     * received with an IPO rule set by deipce_set_mac_address(). When the
     * address does not match the address of the real device, pkt_type in
     * frames may get changed to PACKET_OTHERHOST. Higher level would drop such
     * frames, hence change pkt_type in this case to PACKET_HOST.
     */
    if (rx_frame->pkt_type == PACKET_OTHERHOST) {
        rx_frame->pkt_type = PACKET_HOST;
    }
    // Update statistics
    np->stats.rx_bytes += rx_frame->len;
    np->stats.rx_packets++;

    //netdev_dbg(netdev, "%s(): RX %i\n", __func__, rx_frame->len);

    // Give the packet to the stack
    netif_rx(rx_frame);
    // The buffer was given away

    // Update jiffies
    netdev->last_rx = jiffies;

    return;
}

/**
 * Enable interface.
 * @param dev Netdevice.
 */
static int deipce_enable_interface(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    int ret;

    netif_info(np, ifup, netdev, "Enabling interface\n");

    INIT_DELAYED_WORK(&pp->check_link, &deipce_poll_link);

    ret = deipce_init_adapter(pp);
    if (ret)
        return ret;

    if (pp->medium_type == DEIPCE_MEDIUM_NOPHY) {
        if (np->link_mode == LM_DOWN)
            np->link_mode = deipce_best_adapter_link_mode(pp);
        if (np->link_mode == LM_DOWN) {
            if (dp->features.flags & FLX_FRS_FEAT_GIGABIT)
                np->link_mode = LM_1000FULL;
            else
                np->link_mode = LM_100FULL;
        }
    }
    if (pp->ext_phy.phydev) {
        phy_start(pp->ext_phy.phydev);
        np->link_mode = deipce_get_phy_link_mode(pp->ext_phy.phydev);
    }
    if (pp->sfp.phy.phydev) {
        phy_start(pp->sfp.phy.phydev);
        if (!pp->ext_phy.phydev)
            np->link_mode = deipce_get_phy_link_mode(pp->sfp.phy.phydev);
    }

    deipce_set_port_mode(netdev, np->link_mode);

    // Poll link state.
    queue_delayed_work(drv->wq_low, &pp->check_link,
                       DEIPCE_LINK_CHECK_INTERVAL);

    deipce_switchdev_enable(netdev);

    return 0;
}

/**
 * Disable interface.
 * @param dev Netdevice.
 */
static void deipce_disable_interface(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;

    netif_info(np, ifdown, netdev, "Disabling interface\n");

    deipce_switchdev_disable(netdev);

    // Stop polling link state.
    cancel_delayed_work_sync(&pp->check_link);

    /*
     * Disconnect PHY(s).
     * Note: adjust_link may still be running, cannot acquire (just)
     * link mode lock from here. We don't need to.
     */
    if (pp->sfp.phy.phydev) {
        deipce_phy_disconnect(pp->netdev, &pp->sfp.phy, pp->ext_phy.phydev);
    }
    if (pp->ext_phy.phydev) {
        deipce_phy_disconnect(pp->netdev, &pp->ext_phy, NULL);
    }

    deipce_set_port_mode(netdev, LM_DOWN);

    netdev_dbg(netdev, "%s() done\n", __func__);

    return;
}

/**
 * Open the switch port network device interface.
 * @param dev Netdevice.
 * @return 0 on success or negative error code.
 */
static int deipce_port_open(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    struct deipce_drv_priv *drv = deipce_get_drv_priv();
    int ret = 0;

    netdev_dbg(netdev, "%s()\n", __func__);

    deipce_phy_connect_all(pp);

    switch (pp->medium_type) {
    case DEIPCE_MEDIUM_PHY:
        // Allow autoneg.
        np->force_link_mode = LM_DOWN;
        np->link_mode = LM_DOWN;
        break;
    case DEIPCE_MEDIUM_SFP:
        // Allow autoneg.
        np->force_link_mode = LM_DOWN;
        np->link_mode = LM_DOWN;
        deipce_init_sfp(pp);
        if (pp->sfp.eeprom) {
            // Detect SFP so that adapter can be initialized correctly.
            deipce_set_sfp(pp, deipce_detect_sfp(pp));
        }
        break;
    case DEIPCE_MEDIUM_NOPHY:
        // Try to determine link mode from EMAC (CPU port).
        if (pp->flags & DEIPCE_PORT_CPU) {
            if (dp->real_netdev->phydev)
                np->link_mode =
                    deipce_get_phy_link_mode(dp->real_netdev->phydev);
        }
        break;
    case DEIPCE_MEDIUM_NONE:
    default:
        netdev_dbg(netdev, "Port not in use\n");
        return -ENODEV;
    }

    deipce_enable_interface(netdev);

    // Capture statistics counters periodically.
    if (dp->features.flags & FLX_FRS_FEAT_STATS) {
        INIT_DELAYED_WORK(&pp->capture_stats, &deipce_capture_stats);
        queue_delayed_work(drv->wq_low, &pp->capture_stats,
                           DEIPCE_STATS_CAPTURE_INTERVAL);
    }

    // Start interface.
    netif_start_queue(netdev);

    // Ensure we get all frames from the underlying MAC.
    dev_set_promiscuity(dp->real_netdev, 1);

    netdev_info(netdev, "Interface open\n");

    netdev_dbg(netdev,
               "Supported PHY 0x%x SFP 0x%x adapter 0x%x\n",
               pp->ext_phy.phydev ? pp->ext_phy.phydev->supported : 0,
               pp->sfp.supported,
               pp->adapter.supported);

    return ret;
}

/**
 * Open the endpoint network device interface.
 * @param dev Netdevice.
 * @return 0 on success or negative error code.
 */
static int deipce_ep_open(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;
    int ret = 0;

    netdev_dbg(netdev, "%s()\n", __func__);

    // Ensure we get all frames from underlying MAC.
    dev_set_promiscuity(dp->real_netdev, 1);

    if (dp->features.flags & FLX_FRS_FEAT_GIGABIT)
        np->link_mode = LM_1000FULL;
    else
        np->link_mode = LM_100FULL;
    netif_carrier_on(netdev);

    // Start interface.
    netif_start_queue(netdev);

    netdev_info(netdev, "Interface open\n");

    return ret;
}

/**
 * Close the switch port network interface.
 * @param dev Netdevice.
 * @return 0 on success or negative error code.
 */
static int deipce_port_close(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;

    netdev_dbg(netdev, "%s()\n", __func__);

    dev_set_promiscuity(dp->real_netdev, -1);

    // We can not take any more packets to be sent.
    if (!netif_queue_stopped(netdev))
        netif_stop_queue(netdev);

    // Stop capturing statistics counters.
    if (dp->features.flags & FLX_FRS_FEAT_STATS) {
        cancel_delayed_work_sync(&pp->capture_stats);
    }

    if (dp->features.preempt_ports & (1u << pp->port_num))
        deipce_preempt_reset(pp);
    deipce_disable_interface(netdev);

    if (pp->medium_type == DEIPCE_MEDIUM_SFP) {
        deipce_cleanup_sfp(pp);
    }

    // Forget stored, sent PTP frames waiting for timestamp.
    deipce_netdevif_cleanup_stamper(&pp->tx_stamper);

    netdev_dbg(netdev, "Interface closed\n");

    return 0;
}

/**
 * Close the endpoint network interface.
 * @param dev Netdevice.
 * @return 0 on success or negative error code.
 */
static int deipce_ep_close(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;

    netdev_dbg(netdev, "%s()\n", __func__);

    dev_set_promiscuity(dp->real_netdev, -1);

    // We can not take any more packets to be sent.
    if (!netif_queue_stopped(netdev))
        netif_stop_queue(netdev);

    np->link_mode = LM_DOWN;
    netif_carrier_off(netdev);

    netdev_dbg(netdev, "Interface closed\n");

    return 0;
}

/**
 * New netdevice common initialization function.
 */
static void deipce_netdev_setup(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);

    ether_setup(netdev);

    netdev->tx_queue_len = 0;

    netdev->destructor = &free_netdev;

    *np = (struct deipce_netdev_priv) {
        .msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE),
        .stp_state = BR_STATE_FORWARDING,
    };

    return;
}

/**
 * New netdevice initialization function for switch ports.
 */
static void deipce_port_netdev_setup(struct net_device *netdev)
{
    deipce_netdev_setup(netdev);

    netdev->ethtool_ops = &deipce_ethtool_ops;

    deipce_switchdev_setup_netdev(netdev);

    return;
}

/**
 * New netdevice initialization function for endpoint.
 */
static void deipce_ep_netdev_setup(struct net_device *netdev)
{
    deipce_netdev_setup(netdev);

    netdev->ethtool_ops = &deipce_ep_ethtool_ops;

    return;
}

/// Switch port netdevice operations
static const struct net_device_ops deipce_netdev_ops = {
    .ndo_open = &deipce_port_open,
    .ndo_start_xmit = &deipce_port_start_xmit,
    .ndo_stop = &deipce_port_close,
    .ndo_get_stats = &deipce_get_stats,
    .ndo_do_ioctl = &deipce_netdev_ioctl,
    .ndo_validate_addr = NULL,
    .ndo_set_rx_mode = NULL,
    .ndo_set_mac_address = &deipce_set_mac_address,
    .ndo_tx_timeout = &deipce_netdev_tx_timeout,
    .ndo_change_mtu = NULL,
#ifdef CONFIG_NET_SWITCHDEV
    .ndo_get_phys_port_name = &deipce_switchdev_get_phys_port_name,
    .ndo_bridge_setlink = &switchdev_port_bridge_setlink,
    .ndo_bridge_getlink = &switchdev_port_bridge_getlink,
    .ndo_bridge_dellink = &switchdev_port_bridge_dellink,
    .ndo_fdb_add = &switchdev_port_fdb_add,
    .ndo_fdb_del = &switchdev_port_fdb_del,
    .ndo_fdb_dump = &switchdev_port_fdb_dump,
#endif
};

/// Switch endpoint netdevice operations
static const struct net_device_ops deipce_ep_netdev_ops = {
    .ndo_open = &deipce_ep_open,
    .ndo_start_xmit = &deipce_ep_start_xmit,
    .ndo_stop = &deipce_ep_close,
    .ndo_get_stats = &deipce_get_stats,
    .ndo_do_ioctl = &deipce_netdev_ioctl,
    .ndo_validate_addr = NULL,
    .ndo_set_rx_mode = NULL,
    .ndo_set_mac_address = &deipce_set_mac_address,
    .ndo_tx_timeout = &deipce_netdev_tx_timeout,
    .ndo_change_mtu = NULL,
};

/**
 * Determine whether a netdevice refers to DE-IP Core Edge port.
 * @param netdev Net device to check.
 * @return True if netdev is a DE-IP Core Edge port.
 */
bool deipce_is_port(struct net_device *netdev)
{
    // Must not match endpoint net device.
    if (netdev->netdev_ops == &deipce_netdev_ops)
        return true;

    return false;
}

/**
 * Add netdevice for switch.
 * @param dp Device privates.
 * @param pp Port privates or NULL for endpoint.
 * @param name Name for netdevice or NULL for automatic.
 * @return New netdevice.
 */
static struct net_device *deipce_add_netdev(struct deipce_dev_priv *dp,
                                            struct deipce_port_priv *pp,
                                            const char *name)
{
    struct deipce_netdev_priv *np;
    struct net_device *netdev;
    void (*setup)(struct net_device *netdev) =
        pp ? deipce_port_netdev_setup : deipce_ep_netdev_setup;
    int ret = 0;

    if (name) {
        netdev = alloc_netdev(sizeof(struct deipce_netdev_priv),
                              name, NET_NAME_PREDICTABLE, setup);
    }
    else {
        // Like alloc_etherdev, but provide own setup function.
        netdev = alloc_netdev(sizeof(struct deipce_netdev_priv),
                              "eth%d", NET_NAME_UNKNOWN, setup);
    }
    if (!netdev) {
        dev_warn(dp->this_dev, "Cannot allocate net_device\n");
        return NULL;
    }

    np = netdev_priv(netdev);
    np->dp = dp;
    np->pp = pp;

    mutex_init(&np->link_mode_lock);
    if (pp) {
        pp->netdev = netdev;
        netdev->netdev_ops = &deipce_netdev_ops;
    }
    else {
        dp->netdev = netdev;
        netdev->netdev_ops = &deipce_ep_netdev_ops;
    }
    netdev->irq = dp->irq;

    SET_NETDEV_DEV(netdev, dp->this_dev);

    eth_hw_addr_random(netdev);

    netdev->base_addr = dp->real_netdev->base_addr;
    netdev->irq = dp->real_netdev->irq;

    netdev->watchdog_timeo = TX_TIMEOUT;

    // We can handle multicast packets.
    netdev->flags |= IFF_MULTICAST;

    // Required to work without qdisc on a preemptable (PREEMPT_RT) kernel.
    netdev->features |= NETIF_F_LLTX;

    ret = register_netdev(netdev);
    if (ret) {
        dev_err(dp->this_dev, "register_netdev failed\n");
        goto free_netdev;
    }

    return netdev;

free_netdev:
    if (pp)
        pp->netdev = NULL;
    else
        dp->netdev = NULL;
    free_netdev(netdev);

    return NULL;
}

/**
 * Prevent unloading MAC driver in case it is a module, using try_module_get.
 * @param dp Device privates.
 */
static int deipce_mac_driver_get(struct deipce_dev_priv *dp)
{
    // Must use parent device driver.
    struct device *dev = dp->real_netdev->dev.parent;

    if (dev && dev->driver) {
        if (!try_module_get(dev->driver->owner)) {
            dev_err(dp->this_dev, "Failed to get MAC owner module\n");
            return -ENODEV;
        }
    }

    return 0;
}

/**
 * Allow MAC driver unloading again in case it is a module, using module_put.
 * @param dp Device privates.
 */
static void deipce_mac_driver_put(struct deipce_dev_priv *dp)
{
    // Must use parent device driver.
    struct device *dev = dp->real_netdev->dev.parent;

    if (dev && dev->driver)
        module_put(dev->driver->owner);

    return;
}

/**
 * Initialize switch netdevice functionalities.
 * @param dp Switch privates.
 * @param config Additional switch configuration time information.
 */
int deipce_netdev_init_switch(struct deipce_dev_priv *dp,
                              struct deipce_switch_config *config)
{
    struct net_device *netdev = NULL;
    int ret = -ENODEV;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    // Get MAC netdevice and remember it.
    dp->real_netdev = dev_get_by_name(&init_net, config->mac_name);
    if (!dp->real_netdev) {
        dev_err(dp->this_dev, "MAC net_device %s not found\n",
                config->mac_name);
        return -ENODEV;
    }

    // Prevent unloading its driver.
    ret = deipce_mac_driver_get(dp);
    if (ret)
        goto err_mac_driver_get;

    netdev = deipce_add_netdev(dp, NULL, config->ep_name);
    if (!netdev) {
        ret = -ENODEV;
        goto err_endpoint;
    }

    ret = deipce_netdevif_init(dp);
    if (ret)
        goto err_netdevif;

    netdev_dbg(netdev, "DE-IP Core Edge endpoint\n");

    return 0;

err_netdevif:
    unregister_netdev(dp->netdev);
    dp->netdev = NULL;

err_endpoint:
    deipce_mac_driver_put(dp);

err_mac_driver_get:
    dev_put(dp->real_netdev);
    dp->real_netdev = NULL;

    return ret;
}

/**
 * Initialize switch port netdevice functionalities.
 * @param dp Switch privates.
 * @param pp Port privates.
 * @param config Additional switch port specific configuration time information.
 */
int deipce_netdev_init_port(struct deipce_dev_priv *dp,
                            struct deipce_port_priv *pp,
                            struct deipce_port_config *config)
{
    struct net_device *netdev;
    int ret = -ENODEV;

    dev_dbg(dp->this_dev, "%s() Port %u\n", __func__, pp->port_num);

    netdev = deipce_add_netdev(dp, pp, config->name);
    if (!netdev)
        goto err_netdev;

    ret = deipce_init_adapter(pp);
    if (ret)
        goto err_adapter;

    if (dp->features.preempt_ports & (1u << pp->port_num))
        deipce_preempt_init(pp);

    deipce_set_port_mode(netdev, LM_DOWN);
    deipce_update_ipo_rules(pp);

    if (dp->features.flags & FLX_FRS_FEAT_SHAPER)
        deipce_shaper_init(pp);

    netdev_dbg(netdev, "DE-IP Core Edge port %u\n", pp->port_num);

    dev_dbg(dp->this_dev, "%s() Port %u sysfs\n", __func__, pp->port_num);

    ret = deipce_port_sysfs_init(pp);
    if (ret)
        goto err_port_sysfs;

    if (pp->sched.fsc) {
        ret = deipce_fsc_sysfs_dev_init(pp->sched.fsc, pp->sched.num, pp);
        if (ret)
            goto err_fsc_sysfs;
    }

    return 0;

err_fsc_sysfs:
    deipce_port_sysfs_cleanup(pp);

err_port_sysfs:
    deipce_cleanup_adapter(pp);

err_adapter:
    unregister_netdev(pp->netdev);
    pp->netdev = NULL;

err_netdev:
    return ret;
}

/**
 * Cleanup switch netdevice functionalities.
 * @param dp Switch privates.
 */
void deipce_netdev_cleanup_switch(struct deipce_dev_priv *dp)
{
    dev_dbg(dp->this_dev, "%s()\n", __func__);

    deipce_netdevif_cleanup(dp);

    unregister_netdev(dp->netdev);
    dp->netdev = NULL;

    deipce_mac_driver_put(dp);
    dev_put(dp->real_netdev);
    dp->real_netdev = NULL;

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

/**
 * Cleanup switch port netdevice functionalities.
 * @param dp Switch privates.
 * @param pp Switch port privates.
 */
void deipce_netdev_cleanup_port(struct deipce_dev_priv *dp,
                                struct deipce_port_priv *pp)
{
    dev_dbg(dp->this_dev, "%s()\n", __func__);

    if (pp->sched.fsc)
        deipce_fsc_sysfs_dev_cleanup(pp->sched.fsc, pp->sched.num, pp);

    deipce_port_sysfs_cleanup(pp);
    if (dp->features.preempt_ports & (1u << pp->port_num))
        deipce_preempt_cleanup(pp);

    deipce_cleanup_adapter(pp);
    unregister_netdev(pp->netdev);
    pp->netdev = NULL;

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

