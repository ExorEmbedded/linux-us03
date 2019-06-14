/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2007 Flexibilis Oy

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

#include <linux/phy.h>
#include <linux/version.h>

#include "deipce_ibc_main.h"
#include "deipce_types.h"
#include "deipce_netdev.h"
#include "deipce_if.h"
#include "deipce_main.h"
#include "deipce_adapter.h"
#include "deipce_fsc_main.h"
#include "deipce_time.h"
#include "deipce_ethtool.h"

// mdio was added to struct phy_device in Linux 4.5.
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)

static inline int phydev_addr(struct phy_device *phydev)
{
    return phydev->addr;
}

#else

static inline int phydev_addr(struct phy_device *phydev)
{
    return phydev->mdio.addr;
}

#endif

/**
 * Determine link mode from PHY state.
 * @param phy PHY device.
 * @return Link mode.
 */
enum link_mode deipce_get_phy_link_mode(struct phy_device *phy)
{
    int speed = phy->speed;

    if (!phy->link)
        return LM_DOWN;

    switch (speed) {
    case SPEED_1000:
        return LM_1000FULL;
    case SPEED_100:
        return LM_100FULL;
    case SPEED_10:
        return LM_10FULL;
    }

    return LM_DOWN;
}

/**
 * Determine link mode from adapter state.
 * @param pp FRS port privates.
 * @return Link mode.
 */
static enum link_mode deipce_get_adapter_link_mode(
        struct deipce_port_priv *pp)
{
    enum link_mode link_mode = LM_1000FULL;

    if (pp->adapter.ops.check_link) {
        link_mode = pp->adapter.ops.check_link(pp);
    }
    else {
        link_mode = deipce_best_adapter_link_mode(pp);
    }

    return link_mode;
}

/**
 * Determine ETHTOOL speed from link mode.
 * @param link_mode Link mode.
 * @return ETHTOOL SPEED_xxx constant.
 */
static uint32_t link_mode_to_ethtool_speed(enum link_mode link_mode)
{
    switch (link_mode) {
    case LM_1000FULL:
        return SPEED_1000;
    case LM_100FULL:
        return SPEED_100;
    case LM_10FULL:
        return SPEED_10;
    case LM_DOWN:
        return SPEED_UNKNOWN;
    }
    return SPEED_UNKNOWN;
}

/**
 * Get various device settings including Ethernet link
 * settings. The cmd parameter is expected to have been cleared
 * before get_settings is called.
 * @param netdev Netdevice
 * @param cmd Ethtool data
 * @return negative error code or zero.
 */
static int deipce_get_settings(struct net_device *netdev,
                               struct ethtool_cmd *cmd)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_phy *ext_phy = &pp->ext_phy;
    struct deipce_phy *sfp_phy = &pp->sfp.phy;
    int ret = 0;

    cmd->maxtxpkt = 1;
    cmd->maxrxpkt = 1;

    mutex_lock(&np->link_mode_lock);

    cmd->lp_advertising = 0;

    // half-duplex is not supported
    switch (pp->medium_type) {
    case DEIPCE_MEDIUM_SFP:
        // Show those modes supported by current SFP module.
        if (ext_phy->phydev && ext_phy->phydev->state != PHY_DOWN) {
            // PHY present and used.
            ret = phy_ethtool_gset(ext_phy->phydev, cmd);
            if (ret == 0) {
                cmd->supported &= pp->adapter.supported;
                if (pp->flags & DEIPCE_HAS_SEPARATE_SFP)
                    cmd->supported |= pp->adapter.supported;

                // Drop fiber if not fiber SFP.
                if (!(pp->sfp.supported & SUPPORTED_FIBRE))
                    cmd->supported &= ~SUPPORTED_FIBRE;

            }
            cmd->port = pp->adapter.port;
        }
        else if (!pp->ext_phy.phydev &&
                 sfp_phy->phydev && sfp_phy->phydev->state != PHY_DOWN) {
            // SFP PHY present and used.
            ret = phy_ethtool_gset(sfp_phy->phydev, cmd);
            if (ret == 0) {
                cmd->supported &= pp->adapter.supported;
            }
            cmd->port = pp->adapter.port;
        }
        else {
            // No PHY.
            cmd->supported = pp->sfp.supported & pp->adapter.supported;
            cmd->advertising = cmd->supported;
            if (cmd->advertising & ADVERTISED_Autoneg)
                cmd->autoneg = AUTONEG_ENABLE;
            else
                cmd->autoneg = AUTONEG_DISABLE;
            ethtool_cmd_speed_set(cmd,
                                  link_mode_to_ethtool_speed(np->link_mode));
            cmd->duplex = DUPLEX_FULL;
            cmd->port = pp->adapter.port;
            cmd->phy_address = 0;
            cmd->transceiver = XCVR_EXTERNAL;
        }
        break;
    case DEIPCE_MEDIUM_PHY:
        if (ext_phy->phydev && ext_phy->phydev->state != PHY_DOWN) {
            // PHY present and used.
            ret = phy_ethtool_gset(ext_phy->phydev, cmd);
            if (ret == 0) {
                cmd->supported &= pp->adapter.supported;
            }
            cmd->port = pp->adapter.port;
        }
        else {
            // PHY not recognized.
            cmd->supported = pp->adapter.supported;
            cmd->advertising = cmd->supported;
            if (cmd->advertising & ADVERTISED_Autoneg)
                cmd->autoneg = AUTONEG_ENABLE;
            else
                cmd->autoneg = AUTONEG_DISABLE;
            ethtool_cmd_speed_set(cmd, SPEED_UNKNOWN);
            cmd->duplex = DUPLEX_FULL;
            cmd->port = pp->adapter.port;
            cmd->phy_address = 0;
            cmd->transceiver = XCVR_EXTERNAL;
        }
        break;
    case DEIPCE_MEDIUM_NONE:
        cmd->supported = 0;
        cmd->advertising = 0;
        cmd->autoneg = AUTONEG_DISABLE;
        ethtool_cmd_speed_set(cmd, SPEED_UNKNOWN);
        cmd->duplex = DUPLEX_FULL;
        cmd->port = PORT_NONE;
        cmd->phy_address = 0;
        cmd->transceiver = XCVR_INTERNAL;
        break;
    case DEIPCE_MEDIUM_NOPHY:
    default:
        cmd->supported = pp->adapter.supported;
        cmd->advertising = cmd->supported;
        if (cmd->advertising & ADVERTISED_Autoneg)
            cmd->autoneg = AUTONEG_ENABLE;
        else
            cmd->autoneg = AUTONEG_DISABLE;
        ethtool_cmd_speed_set(cmd,
                              link_mode_to_ethtool_speed(np->link_mode));
        cmd->duplex = DUPLEX_FULL;
        cmd->port = pp->adapter.port;
        cmd->phy_address = 0;
        cmd->transceiver = XCVR_INTERNAL;
        break;
    }

    mutex_unlock(&np->link_mode_lock);

    return ret;
}

/**
 * Set various device settings including Ethernet link settings.
 * @param netdev Netdevice
 * @param cmd Ethtool data
 * @return negative error code or zero.
 */
static int deipce_set_settings(struct net_device *netdev,
                               struct ethtool_cmd *cmd)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_phy *ext_phy = &pp->ext_phy;
    struct deipce_phy *sfp_phy = &pp->sfp.phy;
    enum link_mode force = LM_DOWN;
    enum link_mode link_mode = LM_DOWN;
    int ret = -EINVAL;

    mutex_lock(&np->link_mode_lock);

    // half-duplex is not supported
    switch (pp->medium_type) {
    case DEIPCE_MEDIUM_SFP:
    case DEIPCE_MEDIUM_PHY:
        // Accept settings even if not supported by current SFP module.
        if (cmd->autoneg == AUTONEG_ENABLE) {
            // Autoneg. Do not advertise anything unsupported.
            // Silently drop unsupported half-duplex advertising.
            cmd->advertising &= ~(ADVERTISED_1000baseT_Half |
                                  ADVERTISED_100baseT_Half |
                                  ADVERTISED_10baseT_Half);
            if (cmd->advertising & ~DEIPCE_ETHTOOL_SUPPORTED)
                break;
            if (cmd->advertising & ~pp->adapter.supported)
                break;
            force = LM_DOWN;
            if (ext_phy->phydev && ext_phy->phydev->state != PHY_DOWN) {
                // PHY present and used
                link_mode = deipce_get_phy_link_mode(ext_phy->phydev);
            }
            else if (sfp_phy->phydev && sfp_phy->phydev->state != PHY_DOWN) {
                // SFP PHY present and used
                link_mode = deipce_get_phy_link_mode(sfp_phy->phydev);
            }
            else {
                link_mode = deipce_get_adapter_link_mode(pp);
            }
        } else {
            // Forced
            if (cmd->speed == SPEED_1000 && cmd->duplex == DUPLEX_FULL)
                force = LM_1000FULL;
            else if (cmd->speed == SPEED_100 && cmd->duplex == DUPLEX_FULL)
                force = LM_100FULL;
            else if (cmd->speed == SPEED_10 && cmd->duplex == DUPLEX_FULL)
                force = LM_10FULL;
            else
                break;
            if (!deipce_is_supported_by_adapter(pp, force))
                break;
            link_mode = force;
        }

        np->force_link_mode = force;
        ret = deipce_update_port_mode(netdev, link_mode);

        if (ret == 0 &&
            ((ext_phy->phydev && (ext_phy->phydev->state != PHY_DOWN)) ||
             (sfp_phy->phydev && (sfp_phy->phydev->state != PHY_DOWN)))) {

            struct deipce_drv_priv *drv = deipce_get_drv_priv();

            // Must release mutex first, so ensure access is safe
            // by cancelling link check work temporarily.
            mutex_unlock(&np->link_mode_lock);
            cancel_delayed_work_sync(&pp->check_link);

            // Needed to keep PHY and netdevice link status synchronized.
            netif_carrier_off(netdev);

            // Situation might have changed.
            if (ext_phy->phydev && ext_phy->phydev->state != PHY_DOWN) {
                cmd->phy_address = phydev_addr(ext_phy->phydev);
                ret = phy_ethtool_sset(ext_phy->phydev, cmd);
            }
            if (ret == 0 &&
                sfp_phy->phydev && sfp_phy->phydev->state != PHY_DOWN) {
                cmd->phy_address = phydev_addr(sfp_phy->phydev);
                ret = phy_ethtool_sset(sfp_phy->phydev, cmd);
            }

            queue_delayed_work(drv->wq_low, &pp->check_link,
                               DEIPCE_LINK_CHECK_INTERVAL);
            goto out;
        }
        break;
    case DEIPCE_MEDIUM_NONE:
        break;
    case DEIPCE_MEDIUM_NOPHY:
    default:
        if (cmd->speed == SPEED_1000 && cmd->duplex == DUPLEX_FULL)
            force = LM_1000FULL;
        else if (cmd->speed == SPEED_100 && cmd->duplex == DUPLEX_FULL)
            force = LM_100FULL;
        else if (cmd->speed == SPEED_10 && cmd->duplex == DUPLEX_FULL)
            force = LM_10FULL;
        else
            break;
        if (!deipce_is_supported_by_adapter(pp, force))
            break;
        np->force_link_mode = force;
        ret = deipce_update_port_mode(netdev, force);
    }

    mutex_unlock(&np->link_mode_lock);

out:
    return ret;
}

/**
 * Report driver/device information. Should only set the driver,
 * version, fw_version and bus_info fields
 * @param netdev Netdevice
 * @param info Ethtool data
 */
static void deipce_get_drvinfo(struct net_device *netdev,
                               struct ethtool_drvinfo *info)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;
    uint32_t rev_id = deipce_read_switch_uint32(dp, FRS_REG_BODY_SVN_ID);

    strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
    strlcpy(info->version, DRV_VERSION, sizeof(info->version));
    snprintf(info->fw_version, sizeof(info->fw_version), "%u", rev_id);
    strcpy(info->bus_info, "N/A");

    return;
}

/**
 * Report whether Wake-on-Lan is enabled.
 * @param netdev Netdevice
 * @param wol Ethtool data
 */
static void deipce_get_wol(struct net_device *netdev,
                           struct ethtool_wolinfo *wol)
{
    wol->supported = 0;
    wol->wolopts = 0;
}

/**
 * Turn Wake-on-Lan on or off.
 * @param netdev Netdevice
 * @param wol Ethtool data
 * @return negative error code or zero.
 */
static int deipce_set_wol(struct net_device *netdev,
                          struct ethtool_wolinfo *wol)
{
    return -EOPNOTSUPP;
}

/**
 * Report driver message level.  This should be the value
 * of the msg_enable field used by netif logging functions.
 * @param netdev Netdevice
 * @return NETIF message level.
 */
static uint32_t deipce_get_msglevel(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    return np->msg_enable;
}

/**
 * Set driver message level.
 * @param netdev Netdevice
 * @param value NETIF message level.
 */
static void deipce_set_msglevel(struct net_device *netdev, u32 value)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    np->msg_enable = value;
}

/**
 * Restart autonegotiation.
 * @param netdev Netdevice
 * @return negative error code or zero
 */
static int deipce_nway_reset(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    int ret = 0;

    mutex_lock(&np->link_mode_lock);

    if (!netif_running(netdev)) {
        ret = -EINVAL;
        goto out;
    }

    if (pp->ext_phy.phydev) {
        ret = phy_start_aneg(pp->ext_phy.phydev);
    }

    if (pp->sfp.phy.phydev) {
        int ret2 = phy_start_aneg(pp->sfp.phy.phydev);
        if (ret2 < 0 && ret == 0)
            ret = ret2;
    }

out:
    mutex_unlock(&np->link_mode_lock);

    return ret;
}

/**
 * Report whether physical link is up.  Will only be called if
 * the netdev is up.  Should usually be set to ethtool_op_get_link(),
 * which uses netif_carrier_ok().
 * @param netdev Netdevice
 * @return Link state.
 */
static u32 deipce_get_link(struct net_device *netdev)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    //struct deipce_port_priv *pp = np->pp;

    switch (np->link_mode) {
    case LM_DOWN:
        return 0;
    default:
        return 1;
    }
}

const struct deipce_stat deipce_stat_info[] = {
    // FRS counter based statistics
    [FRS_CNT_RX_GOOD_OCTETS] = { "rx_good_octets", PORT_REG_RX_GOOD_L },
    [FRS_CNT_RX_BAD_OCTETS] = { "rx_bad_octets", PORT_REG_RX_BAD_L },
    [FRS_CNT_RX_UNICAST] = { "rx_unicast_packets", PORT_REG_RX_UNICAST_L },
    [FRS_CNT_RX_BROADCAST] =
        { "rx_broadcast_packets", PORT_REG_RX_BROADCAST_L },
    [FRS_CNT_RX_MULTICAST] =
        { "rx_multicast_packets", PORT_REG_RX_MULTICAST_L },
    [FRS_CNT_RX_UNDERSIZE] =
        { "rx_undersize_packets", PORT_REG_RX_UNDERSIZE_L },
    [FRS_CNT_RX_FRAGMENT] =
        { "rx_fragment_packets", PORT_REG_RX_FRAGMENT_L },
    [FRS_CNT_RX_OVERSIZE] =
        { "rx_oversize_packets", PORT_REG_RX_OVERSIZE_L },
    [FRS_CNT_RX_JABBER] = { "rx_jabber_packets", PORT_REG_RX_JABBER_L },
    [FRS_CNT_RX_ERR] = { "rx_error_packets", PORT_REG_RX_ERR_L },
    [FRS_CNT_RX_CRC] = { "rx_crc_error_packets", PORT_REG_RX_CRC_L },
    [FRS_CNT_RX_MEM_FULL_DROP] =
        { "rx_mem_full_drop_packets", PORT_REG_RX_MEM_FULL_DROP_L },

    [FRS_CNT_RX_64] = { "rx_64_octets_packets", PORT_REG_RX_64_L },
    [FRS_CNT_RX_65_127] =
        { "rx_65_127_octets_packets", PORT_REG_RX_65_127_L },
    [FRS_CNT_RX_128_255] =
        { "rx_128_255_octets_packets", PORT_REG_RX_128_255_L },
    [FRS_CNT_RX_256_511] =
        { "rx_256_511_octets_packets", PORT_REG_RX_256_511_L },
    [FRS_CNT_RX_512_1023] =
        { "rx_512_1023_octets_packets", PORT_REG_RX_512_1023_L },
    [FRS_CNT_RX_1024_1536] =
        { "rx_1024_1536_octets_packets", PORT_REG_RX_1024_1536_L },

    [FRS_CNT_RX_HSRPRP] = { "rx_hsr_prp_packets", PORT_REG_RX_HSRPRP_L },
    [FRS_CNT_RX_WRONGLAN] =
        { "rx_prp_wrong_lan_packets", PORT_REG_RX_WRONGLAN_L },
    [FRS_CNT_RX_DUPLICATE] =
        { "rx_hsr_prp_duplicate_packets", PORT_REG_RX_DUPLICATE_L },

    [FRS_CNT_RX_POLICED] = { "rx_policed_packets", PORT_REG_RX_POLICED_L },
    [FRS_CNT_RX_MACSEC_UNTAGGED] =
        { "rx_macsec_untagged_packets", PORT_REG_RX_MACSEC_UNTAGGED_L },
    [FRS_CNT_RX_MACSEC_NOTSUPP] =
        { "rx_macsec_notsupp_packets", PORT_REG_RX_MACSEC_NOTSUPP_L },
    [FRS_CNT_RX_MACSEC_UNKNOWN_SCI] =
        { "rx_macsec_unknown_sci_packets", PORT_REG_RX_MACSEC_UNKNOWN_SCI_L },
    [FRS_CNT_RX_MACSEC_NOTVALID] =
        { "rx_macsec_notvalid_packets", PORT_REG_RX_MACSEC_NOTVALID_L },
    [FRS_CNT_RX_MACSEC_LATE] =
        { "rx_macsec_late_packets", PORT_REG_RX_MACSEC_LATE_L },

    [FRS_CNT_RX_PREEMPT_SMD_ERR] =
        { "rx_preempt_smd_error_fragments", PORT_REG_RX_PREEMPT_SMD_ERR_L },
    [FRS_CNT_RX_PREEMPT_ASS_ERR] =
        { "rx_preempt_assembly_errors", PORT_REG_RX_PREEMPT_ASS_ERR_L },
    [FRS_CNT_RX_PREEMPT_ASS_OK] =
        { "rx_preempt_assembled_packets", PORT_REG_RX_PREEMPT_ASS_OK_L },
    [FRS_CNT_RX_PREEMPT_FRAG] =
        { "rx_preempt_extra_fragments", PORT_REG_RX_PREEMPT_FRAG_L },

    [FRS_CNT_TX_OCTETS] = { "tx_octets", PORT_REG_TX_L },
    [FRS_CNT_TX_UNICAST] = { "tx_unicast_packets", PORT_REG_TX_UNICAST_L },
    [FRS_CNT_TX_BROADCAST] =
        { "tx_broadcast_packets", PORT_REG_TX_BROADCAST_L },
    [FRS_CNT_TX_MULTICAST] =
        { "tx_multicast_packets", PORT_REG_TX_MULTICAST_L },

    [FRS_CNT_TX_HSRPRP] = { "tx_hsr_prp_packets", PORT_REG_TX_HSRPRP_L },

    [FRS_CNT_TX_PRIQ_DROP] =
        { "tx_priority_queue_drop", PORT_REG_TX_PRIQ_DROP_L },
    [FRS_CNT_TX_EARLY_DROP] = { "tx_early_drop", PORT_REG_TX_EARLY_DROP_L },
    [FRS_CNT_TX_PREEMPT_FRAG] =
        { "tx_preempt_extra_fragments", PORT_REG_TX_PREEMPT_FRAG_L },
    [FRS_CNT_TX_OVERRUN] = { "tx_overrun_events", PORT_REG_TX_OVERRUN_L },
};

/**
 * Capture FRS port statistics counters.
 * Statistics lock must be acquired while calling this.
 * @param pp FRS port privates.
 */
void deipce_update_port_stats(struct deipce_port_priv *pp)
{
    uint64_t *stats = &pp->stats[0];
    unsigned int i = 1000;
    uint16_t data;

    // Capture counters.
    deipce_write_port_reg(pp, PORT_REG_CNT_CTRL,
                          PORT_REG_CNT_CTRL_CAPTURE);
    do {
        if (i-- == 0) {
            netdev_err(pp->netdev, "CAPTURE timeout\n");
            break;
        }
        schedule();
        data = deipce_read_port_reg(pp, PORT_REG_CNT_CTRL);
    } while (data & PORT_REG_CNT_CTRL_CAPTURE);

    for (i = 0; i < FRS_CNT_REG_COUNT; i++) {
        stats[i] +=
            deipce_read_port_counter(pp, deipce_stat_info[i].reg);
    }

    return;
}

/**
 * Get number of strings that get_strings will write
 * @param netdev Netdevice
 * @param sset String set.
 * @return Number of strings.
 */
static int deipce_get_sset_count(struct net_device *netdev, int sset)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;

    if (!(dp->features.flags & FLX_FRS_FEAT_STATS))
        return -EOPNOTSUPP;

    switch (sset) {
    case ETH_SS_TEST:
        return 0;
    case ETH_SS_STATS:
        return FRS_CNT_REG_COUNT;
    default:
        return -EOPNOTSUPP;
    }
    return -EOPNOTSUPP;
}

/**
 * Return extended statistics about the device.
 * This is only useful if the device maintains statistics not
 * included in struct rtnl_link_stats64.
 * @param netdev Netdevice
 * @param dummy Not used
 * @param data Statistics buffer
 */
static void deipce_get_ethtool_stats(struct net_device *netdev,
                                     struct ethtool_stats *dummy,
                                     u64 *data)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;

    if (!(dp->features.flags & FLX_FRS_FEAT_STATS))
        return;

    // Set fresh FRS counter based statistics.
    mutex_lock(&pp->stats_lock);

    deipce_update_port_stats(pp);
    memcpy(&data[0], &pp->stats[0], FRS_CNT_REG_COUNT * sizeof(data[0]));

    mutex_unlock(&pp->stats_lock);

    return;
}

/**
 * Return a set of strings that describe the requested objects.
 * @param netdev Netdevice
 * @param sset String set.
 * @param data String buffer.
 */
static void deipce_get_strings(struct net_device *netdev,
                               u32 sset, u8 *data)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;
    int i = 0;

    if (!(dp->features.flags & FLX_FRS_FEAT_STATS))
        return;

    switch (sset) {
    case ETH_SS_TEST:
        break;
    case ETH_SS_STATS:
        for (i = 0; i < FRS_CNT_REG_COUNT; i++) {
            memcpy(data, deipce_stat_info[i].name, ETH_GSTRING_LEN);
            data += ETH_GSTRING_LEN;
        }
        break;
    }
}

/**
 * Get timestamping capabilities information.
 * @param netdev Netdevice.
 * @param info Timestamping capabilities information.
 */
static int deipce_get_ts_info(struct net_device *netdev,
                              struct ethtool_ts_info *info)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;

    info->so_timestamping =
        SOF_TIMESTAMPING_TX_HARDWARE |
        SOF_TIMESTAMPING_RX_HARDWARE |
        SOF_TIMESTAMPING_RAW_HARDWARE;
    info->phc_index = deipce_time_get_avail_phc(dp->time, 0);
    info->tx_types =
        (1u << HWTSTAMP_TX_OFF) |
        (1u << HWTSTAMP_TX_ON) |
        (1u << HWTSTAMP_TX_ONESTEP_SYNC);
    info->rx_filters =
        (1u << HWTSTAMP_FILTER_NONE) |
        (1u << HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
        (1u << HWTSTAMP_FILTER_PTP_V2_L2_EVENT);

#if defined(ETHTOOL_STSCLK) || defined(ETHTOOL_SWRKCLK)
    info->phc_timestamper = deipce_time_get_phc(dp->time, DEIPCE_TIME_SEL_TS);
    info->phc_index_2nd = deipce_time_get_avail_phc(dp->time, 1);
    info->phc_worker = deipce_time_get_phc(dp->time, DEIPCE_TIME_SEL_WRK);

    netdev_dbg(netdev, "%s() phc %i 2nd %i stamper %i worker %i\n",
               __func__, info->phc_index, info->phc_index_2nd,
               info->phc_timestamper, info->phc_worker);
#endif

    return 0;
}

#ifdef ETHTOOL_STSCLK
/**
 * Change timestamper clock.
 * @param netdev Switch port net device.
 * @param clk Request with new PHC to use for timestamping.
 */
static int deipce_set_ts_clk(struct net_device *netdev,
                             const struct ethtool_clock *clk)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;
    int ret;

    netdev_dbg(netdev, "%s() PHC %i\n", __func__, clk->phc_index);

    ret = deipce_time_set_phc(dp->time, DEIPCE_TIME_SEL_TS, clk->phc_index);

    return ret;
}
#endif

#ifdef ETHTOOL_SWRKCLK
/**
 * Change worker clock.
 * @param netdev Switch port net device.
 * @param clk Request with new PHC to use as "worker".
 */
static int deipce_set_wrk_clk(struct net_device *netdev,
                              const struct ethtool_clock *clk)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_port_priv *pp = np->pp;
    struct deipce_dev_priv *dp = np->dp;
    int ret;

    netdev_dbg(netdev, "%s() PHC %i\n", __func__, clk->phc_index);

    if (!pp->sched.fsc)
        return -EOPNOTSUPP;

    ret = deipce_time_set_phc(dp->time, DEIPCE_TIME_SEL_WRK,
                              clk->phc_index);

    netdev_dbg(netdev, "%s() PHC %i ret %i \n",
                __func__, clk->phc_index, ret);

    return ret;
}
#endif

/// ethtool ops for switch ports
const struct ethtool_ops deipce_ethtool_ops = {
    .get_settings = &deipce_get_settings,
    .set_settings = &deipce_set_settings,
    .get_drvinfo = &deipce_get_drvinfo,
    .get_wol = &deipce_get_wol,
    .set_wol = &deipce_set_wol,
    .get_msglevel = &deipce_get_msglevel,
    .set_msglevel = &deipce_set_msglevel,
    .nway_reset = &deipce_nway_reset,
    .get_link = &deipce_get_link,
    .get_strings = &deipce_get_strings,
    .get_ethtool_stats = &deipce_get_ethtool_stats,
    .get_sset_count = &deipce_get_sset_count,
    .get_ts_info = &deipce_get_ts_info,
#ifdef ETHTOOL_STSCLK
    .set_ts_clk = &deipce_set_ts_clk,
#endif
#ifdef ETHTOOL_SWRKCLK
    .set_wrk_clk = &deipce_set_wrk_clk,
#endif
};

/// ethtool ops for endpoint
const struct ethtool_ops deipce_ep_ethtool_ops = {
    .get_drvinfo = &deipce_get_drvinfo,
    .get_wol = &deipce_get_wol,
    .set_wol = &deipce_set_wol,
    .get_msglevel = &deipce_get_msglevel,
    .set_msglevel = &deipce_set_msglevel,
    .get_link = &deipce_get_link,
    .get_ts_info = &deipce_get_ts_info,
};

