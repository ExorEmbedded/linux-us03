/** @file
 */

/*

   Real-Time Clock Linux driver

*/

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of.h>

#include "kairos_main.h"
#include "kairos_time.h"
#include "kairos_clock_types.h"
#include "kairos_clock_nco.h"
#include "kairos_clock_ptp.h"
#include "kairos_clock_regs.h"
#include "kairos_clock_main.h"

/// Driver privates.
struct flx_frtc_drv_priv flx_frtc_drv_priv;

/// Get access to driver privates.
struct flx_frtc_drv_priv *flx_frtc_get_drv(void)
{
    return &flx_frtc_drv_priv;
}

/**
 * Get FRTC of an FRTC node.
 * @param node FRTC node.
 * @return FRTC or NULL if not an FRTC node or an error.
 */
struct flx_frtc_dev_priv *kairos_clock_of_get_clock(struct device_node *node)
{
    struct flx_frtc_drv_priv *drv = flx_frtc_get_drv();
    static struct flx_frtc_dev_priv *dp = NULL;

    list_for_each_entry(dp, &drv->devices, list) {
        if (dp->pdev->dev.of_node == node)
            return dp;
    }

    return NULL;
}

/**
 * Get FRTC time granularity.
 * @param dp Clock device privates.
 * @param granularity Place for granularity, value is tenths of nanoseconds.
 */
void kairos_clock_get_granularity(struct flx_frtc_dev_priv *dp,
                                  uint32_t *granularity)
{
    *granularity = dp->step_nsec * 10u +
        (uint32_t)(((uint64_t)dp->step_subnsec * 10u) >> 32);

    return;
}

/**
 * Platform devices and probe functions.
 */
static const struct of_device_id flx_frtc_match[] = {
    { .compatible = "flx,frtc" },
    { },
};

/**
 * FRTC platform device initialization function.
 */
static int flx_frtc_dev_init(struct platform_device *pdev)
{
    int ret = -ENXIO;
    struct flx_frtc_drv_priv *drv = flx_frtc_get_drv();
    struct flx_frtc_dev_priv *dp = NULL;
    struct resource *res = NULL;
#ifdef CONFIG_PTP_1588_CLOCK
    struct device_node *node = NULL;
#endif

    dev_dbg(&pdev->dev, "probe device\n");

    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_warn(&pdev->dev, "kmalloc failed\n");
        ret = -ENOMEM;
        goto err_alloc;
    }
    *dp = (struct flx_frtc_dev_priv){
        .pdev = pdev,
    };
    spin_lock_init(&dp->lock);

    INIT_LIST_HEAD(&dp->list);
    list_add(&dp->list, &drv->devices);

#ifdef CONFIG_PTP_1588_CLOCK
    node = of_parse_phandle(pdev->dev.of_node, "ext-ts", 0);
    if (node) {
        dp->event.fpts = deipce_fpts_of_get_device_by_node(node);
        if (!dp->event.fpts)
            dev_warn(&pdev->dev, "Failed to get FPTS\n");
        of_node_put(node);
    }

#ifdef PTP_PTP_OFFSET_PRECISE
    // Record cross-device timestamping trigger node for resolving them later.
    dp->event.trigger_node =
        of_parse_phandle(pdev->dev.of_node, "ext-ts-trigger-clock", 0);
#endif
#endif

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "I/O registers not defined\n");
        goto err_regs;
    }

    dp->ioaddr = ioremap_nocache(res->start, resource_size(res));
    if (!dp->ioaddr) {
        dev_warn(&pdev->dev, "Component ioremap failed\n");
        ret = -ENOMEM;
        goto err_ioremap;
    }

    flx_frtc_nco_init_registers(dp);
    ret = flx_frtc_ptp_init(dp);
    if (ret)
        goto err_ptp;

    return 0;

err_ptp:
    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

err_ioremap:
err_regs:
    list_del(&dp->list);
    dp->pdev = NULL;
    kfree(dp);

err_alloc:
    return ret;
}

/**
 * FRTC platform device cleanup function.
 */
static void flx_frtc_dev_cleanup(struct flx_frtc_dev_priv *dp)
{
    struct platform_device *pdev = dp->pdev;

    dev_dbg(&pdev->dev, "Cleanup\n");

    deipce_time_remove_clock(dp);
    flx_frtc_ptp_cleanup(dp);

    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

    list_del(&dp->list);
    dp->pdev = NULL;

    kfree(dp);

    dev_dbg(&pdev->dev, "Cleanup done\n");

    return;
}

static struct platform_driver frtc_dev_driver = {
    .driver = {
        .name = "flx_frtc",
        .owner = THIS_MODULE,
        .of_match_table = flx_frtc_match,
    },
    .probe = &flx_frtc_dev_init,
};

/**
 * Driver initialization.
 * @return 0 if success.
 */
int __init kairos_clock_init_driver(void)
{
    struct flx_frtc_drv_priv *drv = flx_frtc_get_drv();
#ifdef CONFIG_PTP_1588_CLOCK
#ifdef PTP_PTP_OFFSET_PRECISE
    static struct flx_frtc_dev_priv *dp = NULL;
#endif
#endif

    pr_debug(DRV_NAME ": Initialize clock driver\n");

    INIT_LIST_HEAD(&drv->devices);

    platform_driver_register(&frtc_dev_driver);

#ifdef CONFIG_PTP_1588_CLOCK
#ifdef PTP_PTP_OFFSET_PRECISE
    // Resolve cross-device timestamping setups.
    list_for_each_entry(dp, &drv->devices, list) {
        if (dp->event.trigger_node) {
            dp->event.trigger =
                deipce_clock_of_get_clock(dp->event.trigger_node);
            if (!dp->event.trigger) {
                dev_warn(&dp->pdev->dev,
                         "Failed to get ext TS trigger clock\n");
            }
            else {
                deipce_time_create_by_clocks(dp->event.trigger, dp,
                                             dp->event.fpts);
                // Ignore errors.
            }

            of_node_put(dp->event.trigger_node);
            dp->event.trigger_node = NULL;
        }
        else {
            deipce_time_create_by_clocks(dp, NULL, NULL);
            // Ignore errors.
        }
    }
#endif
#endif

    return 0;
}

/**
 * Driver cleanup.
 */
void kairos_clock_cleanup_driver(void)
{
    struct flx_frtc_drv_priv *drv = flx_frtc_get_drv();
    static struct flx_frtc_dev_priv *dp = NULL;
    static struct flx_frtc_dev_priv *tmp = NULL;

    pr_debug(DRV_NAME ": Cleanup clock driver\n");

    list_for_each_entry_safe(dp, tmp, &drv->devices, list) {
        flx_frtc_dev_cleanup(dp);
    }

    platform_driver_unregister(&frtc_dev_driver);

    return;
}

