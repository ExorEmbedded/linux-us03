/** @file
 */

/*

   Flexibilis time driver for Linux

   Copyright (C) 2016 Flexibilis Oy

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
#include <linux/types.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include "deipce_main.h"
#include "deipce_fpts_main.h"
#include "deipce_clock_types.h"
#include "deipce_clock_nco.h"
#include "deipce_clock_ptp.h"

/**
 * PTP hardware clock support: adjust frequency.
 * @param ptp PTP clock structure.
 * @param ppb Frequency adjustment in PPB.
 */
static int flx_frtc_ptp_adjfreq(struct ptp_clock_info *ptp, int32_t ppb)
{
    struct flx_frtc_dev_priv *dp =
        container_of(ptp, struct flx_frtc_dev_priv, ptp_info);

    return flx_frtc_nco_adj_freq(dp, ppb);
}

/**
 * PTP hardware clock support: adjust time.
 * @param ptp PTP clock structure.
 * @param nsec Time adjustment in nanoseconds.
 */
static int flx_frtc_ptp_adjtime(struct ptp_clock_info *ptp, int64_t nsec)
{
    struct flx_frtc_dev_priv *dp =
        container_of(ptp, struct flx_frtc_dev_priv, ptp_info);
    int sign = nsec < 0 ? -1 : 1;
    struct timespec64 adjust_time;

    if (sign < 0)
        nsec = -nsec;
    adjust_time = ns_to_timespec64(nsec);

    return flx_frtc_nco_adj_time(dp, sign, &adjust_time);
}

/**
 * PTP hardware clock support: get current time.
 * @param ptp PTP clock structure.
 * @param ts Place for current time.
 */
static int flx_frtc_ptp_gettime64(struct ptp_clock_info *ptp,
                                  struct timespec64 *ts)
{
    struct flx_frtc_dev_priv *dp =
        container_of(ptp, struct flx_frtc_dev_priv, ptp_info);
    struct flx_frtc_time_data time_data = {
        .counter = 0,
    };

    int ret = flx_frtc_nco_read_time(dp, &time_data);

    *ts = time_data.timestamp;

    return ret;
}

/**
 * PTP hardware clock support: set current time.
 * @param ptp PTP clock structure.
 * @param ts New time.
 */
static int flx_frtc_ptp_settime64(struct ptp_clock_info *ptp,
                                  const struct timespec64 *ts)
{
    struct flx_frtc_dev_priv *dp =
        container_of(ptp, struct flx_frtc_dev_priv, ptp_info);

    // Set time using two adjustments.
    struct flx_frtc_time_data cur_time = {
        .counter = 0,
    };

    int ret = flx_frtc_nco_read_time(dp, &cur_time);

    if (ret < 0)
        return ret;

    ret = flx_frtc_nco_adj_time(dp, -1, &cur_time.timestamp);
    if (ret < 0)
        return ret;

    ret = flx_frtc_nco_adj_time(dp, 1, ts);

    return ret;
}

// getcrosststamp appeared in Linux 4.6
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
/**
 * PTP hardware clock support: Get system and device time simultaneously.
 * This is not precise, but still slightly better than two system calls.
 * @param ptp PTP clock structure.
 * @param cts Place for simultaneously sampled timestamps.
 */
static int flx_frtc_ptp_getcrosststamp(struct ptp_clock_info *ptp,
                                       struct system_device_crosststamp *cts)
{
    struct timespec64 ts;
    int ret = flx_frtc_ptp_gettime64(ptp, &ts);

    if (ret)
        return ret;

    cts->device = ktime_set(ts.tv_sec, ts.tv_nsec);

    // Return host system time as source_time.
    getnstimeofday64(&ts);
    cts->sys_realtime = ktime_set(ts.tv_sec, ts.tv_nsec);

    cts->sys_monoraw = ktime_set(0, 0);

    return 0;
}
#endif

#ifdef PTP_PTP_OFFSET_PRECISE
/**
 * Custom 2nd PHC patch support: Get cross-device timestamps.
 * @param ptp PTP clock structure.
 * @param ptp_peer PTP clock structure of the peer clock.
 * @param cts Place for cross-device timestamps.
 */
static int flx_frtc_ptp_getphcxtstamp(struct ptp_clock_info *ptp,
                                      struct ptp_clock_info *ptp_peer,
                                      struct device_device_crosststamp *cts)
{
    struct flx_frtc_dev_priv *dp =
        container_of(ptp, struct flx_frtc_dev_priv, ptp_info);
    struct flx_frtc_dev_priv *dp_peer =
        container_of(ptp_peer, struct flx_frtc_dev_priv, ptp_info);
    struct flx_frtc_dev_priv *dp_trigger = NULL;
    struct flx_frtc_dev_priv *dp_fpts = NULL;
    struct flx_frtc_time_data trigger_timestamp = {
        .counter = 0,
    };
    struct deipce_fpts_event fpts_event = { .counter = 0 };
    unsigned long int flags;
    int ret = 0;

    /*
     * Figure out which of the two clocks triggers FPTS (dp_trigger) and which
     * one is used for timestamping by FPTS (dp_fpts).
     */
    if (dp->event.trigger && dp->event.trigger == dp_peer) {
        dp_trigger = dp_peer;
        dp_fpts = dp;
    }
    else if (dp_peer->event.trigger && dp_peer->event.trigger == dp) {
        dp_trigger = dp;
        dp_fpts = dp_peer;
    }

    // Sanity checks.
    if (!dp_trigger || !dp_fpts || !dp_fpts->event.fpts)
        return -EOPNOTSUPP;

    spin_lock_irqsave(&dp_trigger->lock, flags);

    deipce_fpts_enable(dp_fpts->event.fpts);

    ret = flx_frtc_nco_read_time_locked(dp_trigger, &trigger_timestamp);
    if (ret == 0)
        ret = deipce_fpts_get_event(dp_fpts->event.fpts, &fpts_event);

    deipce_fpts_disable(dp_fpts->event.fpts);

    spin_unlock_irqrestore(&dp_trigger->lock, flags);

    if (ret == 0) {
        // Store timestamps in correct order.
        if (dp_trigger == dp) {
            cts->device = ktime_set(trigger_timestamp.timestamp.tv_sec,
                                    trigger_timestamp.timestamp.tv_nsec);
            cts->peer_device = ktime_set(fpts_event.time.tv_sec,
                                         fpts_event.time.tv_nsec);
        }
        else {
            cts->device = ktime_set(fpts_event.time.tv_sec,
                                    fpts_event.time.tv_nsec);
            cts->peer_device = ktime_set(trigger_timestamp.timestamp.tv_sec,
                                         trigger_timestamp.timestamp.tv_nsec);
        }
    }

    return ret;
}
#endif

/**
 * PTP hardware clock information.
 */
static const struct ptp_clock_info flx_frtc_ptp_caps = {
    .owner = THIS_MODULE,
    .name = DRV_NAME,
    .max_adj = 1000000,
    .n_alarm = 0,
    .n_ext_ts = 0,
    .n_per_out = 0,
    .n_pins = 0,
    .pps = 0,
    .adjfreq = &flx_frtc_ptp_adjfreq,
    .adjtime = &flx_frtc_ptp_adjtime,
    .gettime64 = &flx_frtc_ptp_gettime64,
    .settime64 = &flx_frtc_ptp_settime64,
    // getcrosststamp appeared in Linux 4.6
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
    .getcrosststamp = &flx_frtc_ptp_getcrosststamp,
#endif
#ifdef PTP_PTP_OFFSET_PRECISE
    .getphcxtstamp = &flx_frtc_ptp_getphcxtstamp,
#endif
};

/**
 * Setup PTP hardware clock support for component.
 * @param dp Clock device privates.
 */
int flx_frtc_ptp_init(struct flx_frtc_dev_priv *dp)
{
    int ret = -EOPNOTSUPP;

    // Register PHC.
    dp->ptp_info = flx_frtc_ptp_caps;
    strlcpy(dp->ptp_info.name, "FRTC", sizeof(dp->ptp_info.name));

    dp->ptp_clock = ptp_clock_register(&dp->ptp_info, &dp->pdev->dev);
    if (IS_ERR(dp->ptp_clock)) {
        ret = PTR_ERR(dp->ptp_clock);
        dp->ptp_clock = NULL;
        dev_warn(&dp->pdev->dev, "PHC registration failed\n");
        return ret;
    }

    return 0;
}

/**
 * Setup PTP hardware clock for cross-device timestamping.
 * @param dp Clock device privates.
 */
int flx_frtc_ptp_init_devxtstamp(struct flx_frtc_dev_priv *dp)
{
    int ret = -EOPNOTSUPP;

#ifdef PTP_PTP_OFFSET_PRECISE
    if (!dp->event.trigger)
        return ret;

    ret = ptp_clock_set_peer(dp->event.trigger->ptp_clock, dp->ptp_clock);
    if (ret) {
        dev_warn(&dp->pdev->dev, "Pairing failed\n");
        dp->event.trigger = NULL;
    }
#endif

    return ret;
}

/**
 * Drop PTP hardware clock support from component.
 * @param dp Clock device privates.
 */
void flx_frtc_ptp_cleanup(struct flx_frtc_dev_priv *dp)
{
#ifdef PTP_PTP_OFFSET_PRECISE
    if (dp->event.trigger) {
        int ret = ptp_clock_set_peer(dp->event.trigger->ptp_clock, NULL);

        if (ret)
            dev_warn(&dp->pdev->dev, "Unpairing failed\n");
        dp->event.trigger = NULL;
    }
#endif

    // Remove PHC.
    if (dp->ptp_clock) {
        if (dp->event.fpts) {
            deipce_fpts_disable(dp->event.fpts);
            dp->event.fpts = NULL;
        }
        ptp_clock_unregister(dp->ptp_clock);
        dp->ptp_clock = NULL;
    }
    return;
}

/**
 * Get PHC index of an FRTC PHC.
 * @param phc PHC information of an FRTC device.
 * @return PHC index or NULL.
 */
int deipce_clock_get_phc_index(struct flx_frtc_dev_priv *dp)
{
    return ptp_clock_index(dp->ptp_clock);
}

