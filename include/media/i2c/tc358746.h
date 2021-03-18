/*
 * tc358746 - Toshiba parallel to CSI-2 bridge
 *
 */

/*
 * References (c = chapter, p = page):
 * REF_01 - Toshiba, TC358743XBG (H2C), Functional Specification, Rev 0.60
 * REF_02 - Toshiba, TC358743XBG_HDMI-CSI_Tv11p_nm.xls
 */

#ifndef _TC358746_
#define _TC358746_

struct tc358746_platform_data {
	/*
	 * The FIFO size is 512x32, so Toshiba recommend to set the default FIFO
	 * level to somewhere in the middle (e.g. 300), so it can cover speed
	 * mismatches in input and output ports.
	 */
	u16 fifo_level;

	/* CSI
	 * Calculate CSI parameters with REF_02 for the highest resolution your
	 * CSI interface can handle. The driver will adjust the number of CSI
	 * lanes in use according to the pixel clock.
	 *
	 * The values in brackets are calculated with REF_02 when the number of
	 * bps pr lane is 823.5 MHz, and can serve as a starting point.
	 */
	u32 lineinitcnt;	/* (0x00001770) */
	u32 lptxtimecnt;	/* (0x00000005) */
	u32 tclk_headercnt;	/* (0x00001d04) */
	u32 tclk_trailcnt;	/* (0x00000000) */
	u32 ths_headercnt;	/* (0x00000505) */
	u32 twakeup;		/* (0x00004650) */
	u32 tclk_postcnt;	/* (0x00000000) */
	u32 ths_trailcnt;	/* (0x00000004) */
	u32 hstxvregcnt;	/* (0x00000005) */

	/* Reset PHY automatically when TMDS clock goes from DC to AC.
	 * Sets PHY_AUTO_RST2 in register PHY_CTL2.
	 * Default: false
	 */
	bool hdmi_phy_auto_reset_tmds_detected;

	/* Reset PHY automatically when TMDS clock passes 21 MHz.
	 * Sets PHY_AUTO_RST3 in register PHY_CTL2.
	 * Default: false
	 */
	bool hdmi_phy_auto_reset_tmds_in_range;

	/* Reset PHY automatically when TMDS clock is detected.
	 * Sets PHY_AUTO_RST4 in register PHY_CTL2.
	 * Default: false
	 */
	bool hdmi_phy_auto_reset_tmds_valid;

	/* Reset HDMI PHY automatically when hsync period is out of range.
	 * Sets H_PI_RST in register HV_RST.
	 * Default: false
	 */
	bool hdmi_phy_auto_reset_hsync_out_of_range;

	/* Reset HDMI PHY automatically when vsync period is out of range.
	 * Sets V_PI_RST in register HV_RST.
	 * Default: false
	 */
	bool hdmi_phy_auto_reset_vsync_out_of_range;
};

/* custom controls */

#endif
