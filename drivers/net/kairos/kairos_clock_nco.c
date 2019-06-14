/** @file
 */

/*

   Flexibilis Real-Time Clock Linux driver

   Copyright (C) 2009 Flexibilis Oy

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
#include <linux/io.h>
#include <linux/time.h>
#include <linux/platform_device.h>

#include "deipce_main.h"
#include "deipce_clock_types.h"
#include "deipce_clock_regs.h"
#include "deipce_clock_nco.h"

/**
 * Set FRTC command and wait until command has been completed.
 * @param dp Device privates.
 * @param cmd FRTC command bits.
 */
static int flx_frtc_nco_wait_cmd(struct flx_frtc_dev_priv *dp, uint32_t cmd)
{
    unsigned int timeout = 100;

    flx_nco_write32(dp, NCO_CMD_REG, cmd);
    while ((flx_nco_read32(dp, NCO_CMD_REG) & cmd) != 0) {
        if ((timeout--) == 0) {
            dev_printk(KERN_DEBUG, &dp->pdev->dev, "NCO read timeout\n");
            return -EIO;
        }
        cpu_relax();
    }

    return 0;
}

/**
 * Adjust NCO frequency.
 * @param dp Device privates.
 * @param ppb Frequency adjustment in PPB.
 */
int flx_frtc_nco_adj_freq(struct flx_frtc_dev_priv *dp, int32_t ppb)
{
    unsigned long int flags;
    uint64_t nsec_subnsec;
    int ret;

    // Set initial step size.
    nsec_subnsec =
        ((uint64_t)dp->step_nsec << 32) |
        (uint64_t)dp->step_subnsec;

    // Adjust data is in ppb (1e-9), use scale factor.
    nsec_subnsec += (int64_t)ppb * dp->adjust_scale_factor;

    // The nsec register width is limited.
    if ((uint32_t)(nsec_subnsec >> 32) & ~(uint32_t)NCO_STEP_NSEC_MASK) {
        dev_printk(KERN_DEBUG, &dp->pdev->dev,
                   "NCO frequency adjustment to invalid value.\n");
        return -EINVAL;
    }

    spin_lock_irqsave(&dp->lock, flags);

    // Record step size nanoseconds part for time adjustments.
    dp->cur_step_nsec = (uint32_t) (nsec_subnsec >> 32);

    // Write data to hw.
    flx_nco_write32(dp, NCO_STEP_NSEC_REG,
                    dp->cur_step_nsec & NCO_STEP_NSEC_MASK);
    flx_nco_write32(dp, NCO_STEP_SUBNSEC_REG,
                    (uint32_t) nsec_subnsec);

    // Write command to hw.
    ret = flx_frtc_nco_wait_cmd(dp, NCO_CMD_ADJUST_STEP);

    spin_unlock_irqrestore(&dp->lock, flags);

    return ret;
}

/**
 * Adjust NCO time.
 * @param dp Device privates.
 * @param sign Direction of adjustment: -1=backward, 1=forward.
 * @param ts Amount to adjust. Both seconds and nanoseconds parts
 * must be positive.
 */
int flx_frtc_nco_adj_time(struct flx_frtc_dev_priv *dp,
                          int sign, const struct timespec64 *ts)
{
    unsigned long int flags;
    int64_t adj_sec;
    int32_t adj_nsec;
    int ret;

    // Sanity checks.
    if (ts->tv_sec < 0)
        return -EINVAL;
    if (ts->tv_nsec < 0)
        return -EINVAL;
    if (ts->tv_nsec >= NSEC_PER_SEC)
        return -EINVAL;

    adj_sec = ts->tv_sec;
    adj_nsec = ts->tv_nsec;

    if (sign < 0) {
        adj_sec = -adj_sec;
        adj_nsec = -adj_nsec;
    }

    spin_lock_irqsave(&dp->lock, flags);

    /*
     * FRTC >= 1.5 does not automatically add step size to the adjustment,
     * so do it here. Subnanoseconds are fine.
     * Handle all possible adjustments correctly.
     * Value written to nanoseconds step size register must be normalized
     * between 0 <= x < 1e9.
     */
    adj_nsec += dp->cur_step_nsec;
    while (adj_nsec < 0) {
        adj_nsec += NSEC_PER_SEC;
        adj_sec--;
    }
    while (adj_nsec >= NSEC_PER_SEC) {
        adj_nsec -= NSEC_PER_SEC;
        adj_sec++;
    }

    flx_nco_write32(dp, NCO_ADJ_SEC_HI_REG,
                    (uint32_t) ((adj_sec >> 32) & NCO_ADJ_SEC_HI_MASK));
    flx_nco_write32(dp, NCO_ADJ_SEC_REG,
                    (uint32_t) adj_sec);
    flx_nco_write32(dp, NCO_ADJ_NSEC_REG,
                    (uint32_t) adj_nsec & NCO_ADJ_NSEC_MASK);

    // Write command to hw.
    ret = flx_frtc_nco_wait_cmd(dp, NCO_CMD_ADJUST_CLOCK);

    spin_unlock_irqrestore(&dp->lock, flags);

    return ret;
}

/**
 * Read current NCO time.
 * Mutex must be held when calling this.
 * @param dp Device privates.
 * @param data Place for time data from NCO.
 */
int flx_frtc_nco_read_time_locked(struct flx_frtc_dev_priv *dp,
                                  struct flx_frtc_time_data *data)
{
    int ret;

    ret = flx_frtc_nco_wait_cmd(dp, NCO_CMD_READ);
    if (ret)
        return ret;

    data->counter =
        ((uint64_t) flx_nco_read32(dp, NCO_CCCNT_HI_REG) << 32) |
         (uint64_t) flx_nco_read32(dp, NCO_CCCNT_REG);
    data->counter &= NCO_CC_MASK;

    // NCO is the time source, so we provide the same data for both structs.
    data->timestamp.tv_sec =
        ((uint64_t) flx_nco_read32(dp, NCO_SEC_HI_REG) << 32) |
         (uint64_t) flx_nco_read32(dp, NCO_SEC_REG);
    data->timestamp.tv_sec &= NCO_SEC_MASK;

    data->timestamp.tv_nsec = flx_nco_read32(dp, NCO_NSEC_REG);
    data->timestamp.tv_nsec &= NCO_NSEC_MASK;

    return 0;
}

/**
 * Read current NCO time.
 * @param dp Device privates.
 * @param data Place for time data from NCO.
 */
int flx_frtc_nco_read_time(struct flx_frtc_dev_priv *dp,
                           struct flx_frtc_time_data *data)
{
    unsigned long int flags;
    int ret;

    spin_lock_irqsave(&dp->lock, flags);
    ret = flx_frtc_nco_read_time_locked(dp, data);
    spin_unlock_irqrestore(&dp->lock, flags);

    return ret;
}

/**
 * Initialize NCO.
 * Sets step size and verifies clock is running.
 * @param dp Device privates.
 */
int flx_frtc_nco_init_registers(struct flx_frtc_dev_priv *dp)
{
    int ret = -ENXIO;
    int tmp;
    struct timespec64 ts;
    struct flx_frtc_time_data read_time;

    dp->step_nsec = flx_nco_read32(dp, NCO_GENERICS_STEP_NSEC_REG);
    dp->step_nsec &= NCO_STEP_NSEC_MASK;
    dp->step_subnsec = flx_nco_read32(dp, NCO_GENERICS_STEP_SUBNSEC_REG);
    dp->cur_step_nsec = dp->step_nsec;

    /*
     * Calculate scaling factor.
     * subnsec is 32 bits, adjust data is in ppb (1e-9), 32bit is 4.29e9.
     * nco_adjust_scale_factor = 10^-9 * nominal_value / (2^-32 ns) ==
     * 4.29 * nominal_value, calculate using U32.8 * U32.8
     */
    dp->adjust_scale_factor = 1100 * ((dp->step_nsec << 8) |
                                      (dp->step_subnsec >> 24));
    dp->adjust_scale_factor >>= 16;

    tmp = flx_nco_read32(dp, GENERAL_REG);

    flx_nco_write32(dp, NCO_STEP_SUBNSEC_REG, dp->step_subnsec);
    flx_nco_write32(dp, NCO_STEP_NSEC_REG, dp->step_nsec);

    /*
     * Initialise the nco with the time of the host system.
     * That is rather inaccurate, but anyway we avoid starting from zero.
     */
    getnstimeofday64(&ts);

    dev_dbg(&dp->pdev->dev, "Using current time in init (%lli.%09li)\n",
            (long long int)ts.tv_sec, ts.tv_nsec);

    // Subtract current NCO time to get it close to zero.
    flx_frtc_nco_read_time(dp, &read_time);
    flx_frtc_nco_adj_time(dp, -1, &read_time.timestamp);

    // Add current OS time.
    flx_frtc_nco_adj_time(dp, 1, &ts);

    // Write the above step to hw.
    flx_nco_write32(dp, NCO_CMD_REG, NCO_CMD_ADJUST_STEP);

    // Verify that the NCO started.
    ret = flx_frtc_nco_read_time(dp, &read_time);
    if (read_time.timestamp.tv_sec < ts.tv_sec) {
        dev_printk(KERN_ERR, &dp->pdev->dev,
                   "NCO init failed (seconds not running properly: %lli).\n",
                   (long long int)read_time.timestamp.tv_sec);
        return -ENXIO;
    }
    if ((read_time.timestamp.tv_sec == ts.tv_sec)
        && (read_time.timestamp.tv_nsec == 0)) {
        dev_printk(KERN_ERR, &dp->pdev->dev,
                   "NCO init failed (nanoseconds not running properly)\n");
        return -ENXIO;
    }

    dev_printk(KERN_DEBUG, &dp->pdev->dev,
               "NCO using step size %u ns %u subns adjust_scale factor %u\n",
               dp->step_nsec, dp->step_subnsec, dp->adjust_scale_factor);

    return 0;
}

