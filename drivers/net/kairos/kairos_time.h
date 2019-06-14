/** @file
 */

/*

   Real-Time Clock Linux driver

*/

#ifndef KAIROS_TIME_H
#define KAIROS_TIME_H

#include <linux/types.h>

struct timespec64;
struct deipce_time;

/**
 * Clock selectors.
 */
enum kairos_time_sel {
    KAIROS_TIME_SEL_TS,         ///< timestamping clock
    KAIROS_TIME_SEL_WRK,        ///< worker clock
};

int kairos_time_get_phc(struct kairos_time *dt, enum kairos_time_sel sel);
int kairos_time_get_avail_phc(struct kairos_time *dt, unsigned int num);
int kairos_time_set_phc(struct kairos_time *dt, enum deipce_time_sel sel,
                        int phc_index);

int kairos_time_get_time(struct kairos_time *dt, enum kairos_time_sel sel,
                         struct timespec64 *time);
int kairos_time_get_granularity(struct kairos_time *dt,
                                enum kairos_time_sel sel,
                                uint32_t *granularity);

// Until autoconfig is in place
#ifndef KAIROSS_AUTOCONFIG

struct flx_frtc_dev_priv;
struct kairos_fpts_dev_priv;
struct kairos_ibc_dev_priv;
struct ptp_clock_info;

void kairos_time_init_driver(void);
int kairos_time_create_by_clocks(struct flx_frtc_dev_priv *clk1,
                                 struct flx_frtc_dev_priv *clk2,
                                 struct deipce_fpts_dev_priv *fpts);
void kairos_time_remove_clock(struct flx_frtc_dev_priv *clk);
int kairos_time_add_mux(struct flx_frtc_dev_priv *clk,
                        struct kairos_ibc_dev_priv *ibc);
void kairos_time_remove_mux(struct kairos_ibc_dev_priv *ibc);
struct kairos_time *kairos_time_get_by_clock(struct flx_frtc_dev_priv *clk);

#endif

#endif

