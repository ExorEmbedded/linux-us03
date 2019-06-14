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

#include <linux/types.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/sockios.h>
#include <linux/uaccess.h>

#include "deipce_main.h"
#include "deipce_types.h"
#include "deipce_if.h"
#include "deipce_hw.h"
#include "deipce_ioctl.h"

/**
 * Handle set PTP hardware timestamp configuration ioctl.
 * @param pp Port privates.
 * @param rq Request
 */
static int deipce_handle_set_hwtstamp_ioctl(struct deipce_dev_priv *dp,
                                            struct ifreq *rq)
{
    struct hwtstamp_config config = { .flags = 0 };
    unsigned long int not_copied;
    uint16_t orig_gen = 0;
    uint16_t new_gen = 0;

    not_copied = copy_from_user(&config, rq->ifr_data, sizeof(config));
    if (not_copied)
        return -EFAULT;

    // Future reservation.
    if (config.flags)
        return -EINVAL;

    // Must return -ERANGE if requested packets cannot be timestamped.
    switch (config.tx_type) {
    case HWTSTAMP_TX_ONESTEP_SYNC:
        // Only two-step is possible with port timestampers.
        if (dp->use_port_ts)
            return -ERANGE;
        new_gen |= FRS_GEN_PTP_MODIFY_SYNC;
        break;
    case HWTSTAMP_TX_ON:
    case HWTSTAMP_TX_OFF:
        break;
    default:
        return -ERANGE;
    }

    switch (config.rx_filter) {
    case HWTSTAMP_FILTER_NONE:
        new_gen |= FRS_GEN_PTP_UDP;
        break;
    case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
    case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
    case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
        config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
        new_gen |= FRS_GEN_PTP_UDP;
        break;
    case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
    case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
    case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
        config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L2_EVENT;
        new_gen |= FRS_GEN_PTP_ETH;
        break;
    case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
    case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
    case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
    case HWTSTAMP_FILTER_PTP_V2_EVENT:
    case HWTSTAMP_FILTER_PTP_V2_SYNC:
    case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
    case HWTSTAMP_FILTER_ALL:
    case HWTSTAMP_FILTER_SOME:
    default:
        return -ERANGE;
    }

    if (dp->use_port_ts)
        new_gen |= FRS_GEN_PTP_DISABLE_CORRECTION;

    mutex_lock(&dp->common_reg_lock);

    orig_gen = deipce_read_switch_reg(dp, FRS_REG_GEN);

    new_gen |= orig_gen & ~(FRS_GEN_PTP_MASK |
                            FRS_GEN_PTP_MODIFY_SYNC |
                            FRS_GEN_PTP_DISABLE_CORRECTION);

    // Do not write unnecessarily.
    if (new_gen != orig_gen)
        deipce_write_switch_reg(dp, FRS_REG_GEN, new_gen);

    dp->tstamp_cfg = config;

    mutex_unlock(&dp->common_reg_lock);

    not_copied = copy_to_user(rq->ifr_data, &config, sizeof(config));
    if (not_copied)
        return -EFAULT;

    return 0;
}

/**
 * Handle get PTP hardware timestamp configuration ioctl.
 * @param pp Port privates.
 * @param rq Request
 */
static int deipce_handle_get_hwtstamp_ioctl(struct deipce_dev_priv *dp,
                                            struct ifreq *rq)
{
    unsigned long int not_copied;

    mutex_lock(&dp->common_reg_lock);

    not_copied = copy_to_user(rq->ifr_data, &dp->tstamp_cfg,
                              sizeof(dp->tstamp_cfg));

    mutex_unlock(&dp->common_reg_lock);

    if (not_copied)
        return -EFAULT;

    return 0;
}

/**
 * Handle switch (common, not just endpoint nor port) related netdevice ioctls.
 * @param dev Netdevice.
 * @param rq Request.
 * @param cmd Command.
 * @return Error code.
 */
int deipce_netdev_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
    struct deipce_netdev_priv *np = netdev_priv(netdev);
    struct deipce_dev_priv *dp = np->dp;
    int ret = -EOPNOTSUPP;

    // Note: rq will not be udpated to userspace if non-zero value is returned.

    switch (cmd) {
    case SIOCSHWTSTAMP:
        ret = deipce_handle_set_hwtstamp_ioctl(dp, rq);
        break;
    case SIOCGHWTSTAMP:
        ret = deipce_handle_get_hwtstamp_ioctl(dp, rq);
        break;
    default:
        netdev_dbg(netdev, "Unknown IOCTL command 0x%x\n", cmd);
    }

    return ret;
}

