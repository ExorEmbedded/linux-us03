/*
 * Defines array containing list of all available/known displays and related physical parameters.
 *
 * Copyright (C) 2013 Exor International
 * Author: Giovanni Pavoni (Exor)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
 
 /*
 * HISTORY REVISION
 * Version  Signature       Date      	Modification reason		
 * 1.0  		SS							08.07.14		Started from displayconfig_rev2.00.xml; the min backlight is set to minimum value supported by the hw (display as
 *																			on US01 there is NO support of the gamma correction and therefor the min backlight value might be different from 
 *																			the value used on Serie500 (displayconfig.xml file).
 * 1.1			SS              25.08.14    Alligned to displayconfog_rev2.2.1.xml:
 *																			Display code #48: changed max brightness to 70%
 *																			Display code #46: changed max brightness to 100%
 *																			Display code #47: changed max brightness to 100%
 *																			Added display code #49: 7" Rocktech for ECO
 * 1.2			SS              12.09.14    Alligned to displayconfog_rev2.2.2.xml:
 *																			Added display code #50: 10" Rocktech for ECO
 * 						              18.09.14    Alligned to displayconfog_rev2.3.1.xml:
 *																			Display code #46: changed horizontal back porch to 39
 * 1.3			SS              17.02.15    Alligned to displayconfog_rev2.3.2.xml:
 *																			Added display code #51: 4,3" Rocktech for ECO
 * 1.4			SS              24.02.15    On displayconfog_rev2.3.3.xml we removed all ECO/LINUX displays; no allignement will be performed.
 *																			Added display code #52: for Altera kit, same code as #31 but MAX brightness=100% 
 *																			Changed on display code #47 .pclk_freq = 64000 (original 72000): otherwise it will not work on Altera kit
 * 1.5			SS              13.04.15    Changed on display code #51 .pclk_freq = 12000 (original 9000): Linux driver is now generating 12MHz, please check OS driver 
 *                                      in order to generate pclk freq lower than 12MHz with proper accurancy!
 *																			As the eSMART04 has been certified with 12MHz pixel clock setting, this is kept also for future fixes into the OS driver.
 * 1.6			GP              13.04.16    Added display code #55: Innolux G101ICE-L01 LVDS 24 bit 1280x800 for serie 700
 * NEXT AVAILABLE DISPLAY CODE: 56
 */
 
#ifndef DISPLAYCONFIG_H
#define DISPLAYCONFIG_H

#define NODISPLAY 0xffff

/* -----------------------------------------------------------
structure which describes the LCD parameters
-----------------------------------------------------------*/
struct t_DisplayParams{
  unsigned long dispid;                           // Display id
  unsigned short brightness_min, brightness_max;  // Inverter's minimum and maximum brightness 	(expressed as pwm dutycycle; range 0...100)
  unsigned long pwmfreq;                          // Frequency of PWM controller [Hz]
  unsigned long rezx, rezy, bpp;                  // Resolution and bpp
  unsigned long hs_fp, hs_bp, hs_w, hs_inv;       // Hsync params
  unsigned long vs_fp, vs_bp, vs_w, vs_inv;       // Vsync params
  unsigned long blank_inv;                        // Blank params
  unsigned long pclk_freq, pclk_inv;              // Pixel clock params (f in Khz)
};

/* 
 * Add to this list any further display device description
 * NOTE: Please remember the last element works as terminator and MUST have the .dispid=NODISPLAY
 */
static struct t_DisplayParams displayconfig[] = {
    /* 25: Powertip PS480272T-005-I11Q 480x272*/
    {
        .dispid    = 25,
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 12000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 3, 
        .hs_bp     = 1, 
        .hs_w      = 41, 
        .hs_inv    = 1,
        
        .vs_fp     = 1, 
        .vs_bp     = 3, 
        .vs_w      = 10, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 50,
    },
    /* 31: Evervision VGG804806_HSE03_PWM 800x480*/
    {
        .dispid    = 31,
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 30000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 60,
    },
    /* 32: CHIMEI TG070Y2-L01 800x480*/
    {
        .dispid    = 32,
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 30000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 20,
        .brightness_max = 100,
    },
    /* 36: AUO G104SN02_V2 800x600*/
    {
        .dispid    = 36,
        .rezx      = 800, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 41000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 5,
        .brightness_max = 100,
    },
    /* 37: Powertip 320x240 */
    {
        .dispid    = 37,
        .rezx      = 320, 
        .rezy      = 240, 
        .bpp       = 16,
        
        .pclk_freq = 8000,  
        .pclk_inv  = 1,
        
        .hs_fp     = 20, 
        .hs_bp     = 38, 
        .hs_w      = 30, 
        .hs_inv    = 1,
        
        .vs_fp     = 5, 
        .vs_bp     = 15, 
        .vs_w      = 4, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 60,
    },
    /* 38: AUO G121SN01V4 800x600*/
    {
        .dispid    = 38,
        .rezx      = 800, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 36000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 5,
        .brightness_max = 100,
    },
    /* 39: AUO G150XG01 1024x768*/
    {
        .dispid    = 39,
        .rezx      = 1024, 
        .rezy      = 768, 
        .bpp       = 16,
        
        .pclk_freq = 61000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 5,
        .brightness_max = 100,
    },
    /* 40: Innolux AT050TN33 480x272*/
    {
        .dispid    = 40,
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 12000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 3, 
        .hs_bp     = 1, 
        .hs_w      = 41, 
        .hs_inv    = 1,
        
        .vs_fp     = 1, 
        .vs_bp     = 3, 
        .vs_w      = 10, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 100,
    },
    /* 41: Chimei G133IGE-L03 1280x800*/
    {
        .dispid    = 41,
        .rezx      = 1280, 
        .rezy      = 800, 
        .bpp       = 16,
        
        .pclk_freq = 72000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 11, 
        .hs_bp     = 110, 
        .hs_w      = 50, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 28, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 4000,
        .brightness_min = 10,
        .brightness_max = 65,
    },
    /* 42: Chimei G121I1-L01 1280x800*/
    {
	.dispid    = 42,
        .rezx      = 1280, 
        .rezy      = 800, 
        .bpp       = 16,
        
        .pclk_freq = 72000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 11, 
        .hs_bp     = 221, 
        .hs_w      = 101, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 28, 
        .vs_w      = 11, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 10,
        .brightness_max = 100,
    },
    /* 43: Ampire AM-800480R2TMQW-T02H 800x480*/
    {
        .dispid    = 43,
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 28000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 60,
    },
    /* 44: Tianma TM043NBH02 480x272*/
    {
        .dispid    = 44,
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 12000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 3, 
        .hs_bp     = 1, 
        .hs_w      = 41, 
        .hs_inv    = 1,
        
        .vs_fp     = 1, 
        .vs_bp     = 3, 
        .vs_w      = 10, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 50,
    },
    /* 45: AGL VM15B2 V4 1024x768 15" */
    {
        .dispid    = 45,
        .rezx      = 1024, 
        .rezy      = 768, 
        .bpp       = 16,
        
        .pclk_freq = 72000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 16, 
        .hs_bp     = 149, 
        .hs_w      = 69, 
        .hs_inv    = 1,
        
        .vs_fp     = 2, 
        .vs_bp     = 36, 
        .vs_w      = 7, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 2,
        .brightness_max = 80,
    },
    /* 46: TIANMA TM050RDH03 800x480 */
    {
      .dispid    = 46,
      .rezx      = 800, 
      .rezy      = 480, 
      .bpp       = 16,
      
      .pclk_freq = 27000, 
      .pclk_inv  = 0,
      
      .hs_fp     = 40, 
      .hs_bp     = 39, 
      .hs_w      = 48, 
      .hs_inv    = 1,
      
      .vs_fp     = 13, 
      .vs_bp     = 30, 
      .vs_w      = 3, 
      .vs_inv    = 1,
      
      .blank_inv      = 0,
      
      .pwmfreq        = 10000,
      .brightness_min = 1,
      .brightness_max = 100,
    },
    /* 47: AUO G101EVN01.0 1280x800 */
    {
      .dispid    = 47,
      .rezx      = 1280, 
      .rezy      = 800, 
      .bpp       = 16,
      
      .pclk_freq = 64000, 
      .pclk_inv  = 0,
      
      .hs_fp     = 11, 
      .hs_bp     = 110, 
      .hs_w      = 50, 
      .hs_inv    = 1,
      
      .vs_fp     = 2, 
      .vs_bp     = 28, 
      .vs_w      = 11, 
      .vs_inv    = 1,
      
      .blank_inv      = 0,
      
      .pwmfreq        = 4000,
      .brightness_min = 5,
      .brightness_max = 100,
    },    
    /* 48: Evervision VGG804806 for eTOP607 800x480 */
    {
      .dispid    = 48,
      .rezx      = 800, 
      .rezy      = 480, 
      .bpp       = 16,
      
      .pclk_freq = 30000, 
      .pclk_inv  = 0,
      
      .hs_fp     = 41, 
      .hs_bp     = 35, 
      .hs_w      = 129, 
      .hs_inv    = 1,
      
      .vs_fp     = 12, 
      .vs_bp     = 35, 
      .vs_w      = 3, 
      .vs_inv    = 1,
      
      .blank_inv      = 0,
      
      .pwmfreq        = 10000,
      .brightness_min = 1,
      .brightness_max = 70,
    },   
    /* 49: Rocktech RK070EH1401-T 800x480*/
    {
        .dispid    = 49,
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 30000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 205, 
        .hs_bp     = 46, 
        .hs_w      = 3, 
        .hs_inv    = 1,
        
        .vs_fp     = 20, 
        .vs_bp     = 23, 
        .vs_w      = 2, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 100,
    },     
    /* 50: Rocktech RK101EH1401-T 1024x600*/
    {
        .dispid    = 50,
        .rezx      = 1024, 
        .rezy      = 600, 
        .bpp       = 16,
        
        .pclk_freq = 51000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 10, 
        .hs_bp     = 320, 
        .hs_w      = 10, 
        .hs_inv    = 1,
        
        .vs_fp     = 10, 
        .vs_bp     = 35, 
        .vs_w      = 10, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 100,
    },  
		/* 51: Rocktech RK043EH1401-T 480x272*/
    {
        .dispid    = 51,
        .rezx      = 480, 
        .rezy      = 272, 
        .bpp       = 16,
        
        .pclk_freq = 12000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 8, 
        .hs_bp     = 43, 
        .hs_w      = 1, 
        .hs_inv    = 1,
        
        .vs_fp     = 4, 
        .vs_bp     = 12, 
        .vs_w      = 1, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 80,
    },  
    /* 52: Evervision VGG804806_PWM for ALTERA kit 800x480*/
    {
        .dispid    = 52,
        .rezx      = 800, 
        .rezy      = 480, 
        .bpp       = 16,
        
        .pclk_freq = 30000, 
        .pclk_inv  = 0,
        
        .hs_fp     = 41, 
        .hs_bp     = 35, 
        .hs_w      = 129, 
        .hs_inv    = 1,
        
        .vs_fp     = 12, 
        .vs_bp     = 35, 
        .vs_w      = 3, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 10000,
        .brightness_min = 1,
        .brightness_max = 100,
    },  
    /* 55: Innolux G101ICE-L01 LVDS 24 bit 1280x800 */
    {
        .dispid    = 55,
        .rezx      = 1280, 
        .rezy      = 800, 
        .bpp       = 24,
        
        .pclk_freq = 71100, 
        .pclk_inv  = 0,
        
        .hs_fp     = 30, 
        .hs_bp     = 30, 
        .hs_w      = 100, 
        .hs_inv    = 0,
        
        .vs_fp     = 3, 
        .vs_bp     = 10, 
        .vs_w      = 10, 
        .vs_inv    = 0,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 200,
        .brightness_min = 1,
        .brightness_max = 100,
    },              
    /* END OF LIST */
    {
      .dispid    = NODISPLAY,
      .rezx      = NODISPLAY, 
      .rezy      = NODISPLAY, 
      .bpp       = NODISPLAY,
      
      .pclk_freq = NODISPLAY, 
      .pclk_inv  = NODISPLAY,
      
      .hs_fp     = NODISPLAY, 
      .hs_bp     = NODISPLAY, 
      .hs_w      = NODISPLAY, 
      .hs_inv    = NODISPLAY,
      
      .vs_fp     = NODISPLAY, 
      .vs_bp     = NODISPLAY, 
      .vs_w      = NODISPLAY, 
      .vs_inv    = NODISPLAY,
      
      .blank_inv      = NODISPLAY,
      
      .pwmfreq        = NODISPLAY,
      .brightness_min = NODISPLAY,
      .brightness_max = NODISPLAY,
    },
    
};

#endif
