/*
 * Defines array containing list of all available DVI resolutions for the DVI plugin module
 *
 * Copyright (C) 2018 Exor International
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
 *
 * NEXT AVAILABLE DISPLAY CODE: 67
 */
 
#ifndef DVICONFIG_H
#define DVICONFIG_H

#define DVI_FIXEDCLKFREQ (64000)

/* 
 * Add to this list any further DVI configuration description
 * NOTE: Please remember the last element works as terminator and MUST have the .dispid=NODISPLAY
 */
static struct t_DisplayParams dviconfig[] = {
    /* 0: DVI XGA 1024x768*/
    {
        .dispid    = 0,
        .rezx      = 1024, 
        .rezy      = 768, 
        .bpp       = 24,
        
        .pclk_freq = 64000, 
        .pclk_inv  = 1,
        
        .hs_fp     = 24, 
        .hs_bp     = 160, 
        .hs_w      = 136, 
        .hs_inv    = 1,
        
        .vs_fp     = 3, 
        .vs_bp     = 29, 
        .vs_w      = 6, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 500,
        .brightness_min = 10,
        .brightness_max = 100,
    },
    /* 1: DVI SXGA 1280x1024*/
    {
        .dispid    = 1,
        .rezx      = 1280, 
        .rezy      = 1024, 
        .bpp       = 24,
        
        .pclk_freq = 108000, 
        .pclk_inv  = 1,
        
        .hs_fp     = 48, 
        .hs_bp     = 248, 
        .hs_w      = 112, 
        .hs_inv    = 0,
        
        .vs_fp     = 1, 
        .vs_bp     = 38, 
        .vs_w      = 3, 
        .vs_inv    = 0,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 500,
        .brightness_min = 10,
        .brightness_max = 100,
    },
    /* 2: DVI FULL-HD 1920x1080*/
    {
        .dispid    = 2,
        .rezx      = 1920, 
        .rezy      = 1080, 
        .bpp       = 24,
        
        .pclk_freq = 148000, 
        .pclk_inv  = 1,
        
        .hs_fp     = 88, 
        .hs_bp     = 148, 
        .hs_w      = 44, 
        .hs_inv    = 0,
        
        .vs_fp     = 4, 
        .vs_bp     = 36, 
        .vs_w      = 5, 
        .vs_inv    = 0,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 500,
        .brightness_min = 10,
        .brightness_max = 100,
    },
    /* 3: DVI WXGA 1368x768*/
    {
        .dispid    = 3,
        .rezx      = 1368, 
        .rezy      = 768, 
        .bpp       = 24,
        
        .pclk_freq = 76000, 
        .pclk_inv  = 1,
        
        .hs_fp     = 48, 
        .hs_bp     = 172, 
        .hs_w      = 32, 
        .hs_inv    = 0,
        
        .vs_fp     = 2, 
        .vs_bp     = 15, 
        .vs_w      = 5, 
        .vs_inv    = 1,
        
        .blank_inv      = 0,
        
        .pwmfreq        = 500,
        .brightness_min = 10,
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
