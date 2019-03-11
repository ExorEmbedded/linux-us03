/*
 * generic videomode helper
 *
 * Copyright (c) 2012 Steffen Trumtrar <s.trumtrar@pengutronix.de>, Pengutronix
 *
 * This file is released under the GPLv2
 */
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/of.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <video/displayconfig.h>

/*----------------------------------------------------------------------------------------------------------------*
   Helper functions to retrieve the display id value, when passed from the cmdline, and use it to set the
   display parameters/timings (the contents of the DTB file are overridden, if a valid dispaly id is passed from
   cmdline.
 *----------------------------------------------------------------------------------------------------------------*/
extern int hw_dispid; //This is an exported variable holding the display id value, if passed from cmdline

int dispid_get_videomode(struct videomode* vm, int dispid)
{
	int i=0;

	// Scan the display array to search for the required dispid
	if(dispid == NODISPLAY)
		return -1;

	while((displayconfig[i].dispid != NODISPLAY) && (displayconfig[i].dispid != dispid))
		i++;

	if(displayconfig[i].dispid == NODISPLAY)
		return -1;

	// If we are here, we have a valid array index pointing to the desired display
	vm->hactive         = displayconfig[i].rezx;
	vm->hback_porch  = displayconfig[i].hs_bp;
	vm->hfront_porch = displayconfig[i].hs_fp;
	vm->hsync_len    = displayconfig[i].hs_w;

	vm->vactive         = displayconfig[i].rezy;
	vm->vback_porch = displayconfig[i].vs_bp;
	vm->vfront_porch = displayconfig[i].vs_fp;
	vm->vsync_len    = displayconfig[i].vs_w;
	vm->pixelclock = 1000 * displayconfig[i].pclk_freq;

	vm->flags = 0;
	if(displayconfig[i].hs_inv == 0)
		vm->flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else
		vm->flags |= DISPLAY_FLAGS_HSYNC_LOW;

	if(displayconfig[i].vs_inv == 0)
		vm->flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else
		vm->flags |= DISPLAY_FLAGS_VSYNC_LOW;

	if(displayconfig[i].blank_inv == 0)
		vm->flags |= DISPLAY_FLAGS_DE_HIGH;
	else
		vm->flags |= DISPLAY_FLAGS_DE_LOW;

	if(displayconfig[i].pclk_inv == 0)
		vm->flags |= DISPLAY_FLAGS_PIXDATA_POSEDGE;
	else
		vm->flags |= DISPLAY_FLAGS_PIXDATA_NEGEDGE;

	return 0;
}

/**
 * of_get_videomode - get the videomode #<index> from devicetree
 * @np - devicenode with the display_timings
 * @vm - set to return value
 * @index - index into list of display_timings
 *	    (Set this to OF_USE_NATIVE_MODE to use whatever mode is
 *	     specified as native mode in the DT.)
 *
 * DESCRIPTION:
 * Get a list of all display timings and put the one
 * specified by index into *vm. This function should only be used, if
 * only one videomode is to be retrieved. A driver that needs to work
 * with multiple/all videomodes should work with
 * of_get_display_timings instead.
 **/
int of_get_videomode(struct device_node *np, struct videomode *vm,
		     int index)
{
	struct display_timings *disp;
	int ret;
	
	if(!dispid_get_videomode(vm, hw_dispid))
	    return 0;

	disp = of_get_display_timings(np);
	if (!disp) {
		pr_err("%pOF: no timings specified\n", np);
		return -EINVAL;
	}

	if (index == OF_USE_NATIVE_MODE)
		index = disp->native_mode;

	ret = videomode_from_timings(disp, vm, index);

	display_timings_release(disp);

	return ret;
}
EXPORT_SYMBOL_GPL(of_get_videomode);
