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

#ifndef DEIPCE_TIME_PRIV_H
#define DEIPCE_TIME_PRIV_H

#include <linux/types.h>
#include <linux/spinlock.h>

#include "deipce_types.h"

#define DEIPCE_TIME_MAX_CLOCKS 2

struct deipce_fpts;
struct deipce_ibc;

/**
 * Time interface context
 */
struct deipce_time {
#ifndef DEIPCE_AUTOCONFIG
    struct list_head list;              ///< linked list
#endif

    struct {
        bool can_mux_ts;                ///< can mux TS clock flag
        bool can_mux_wrk;               ///< can mux worker clock flag
    } capability;                       ///<< capabilities
    /// list of clocks
    struct flx_frtc_dev_priv *clock[DEIPCE_TIME_MAX_CLOCKS];
    struct deipce_fpts_dev_priv *fpts;  ///< event timestamper
    struct deipce_ibc_dev_priv *ibc;    ///< IBC (MUX)

    spinlock_t lock;                    ///< synchronize changes to cur_clock
    /**
     * currently used clocks, index is enum deipce_time_sel value,
     * value is index to clock array
     */
    unsigned int cur_clock[DEIPCE_TIME_MAX_CLOCKS];
};

/**
 * Get access to clock by selector.
 * @param dt Time interface.
 * @param sel Clock selector.
 * @return Clock instance or NULL.
 */
static inline struct flx_frtc_dev_priv *deipce_time_get_clock(
        struct deipce_time *dt, enum deipce_time_sel sel)
{
    unsigned long int flags;
    unsigned int clock_index;

    spin_lock_irqsave(&dt->lock, flags);

    clock_index = dt->cur_clock[sel];

    spin_unlock_irqrestore(&dt->lock, flags);

    return dt->clock[clock_index];
}

#endif
