/*
 *  dviplugin_impl - Helper functions for the Exor DVI plugin module, based on HDAxx boards
 *
 *  Copyright (c) 2018 Exor Int.
 *  Written by: Giovanni Pavoni, Exor Int.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 or
 *  later as publishhed by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <video/displayconfig.h>
#include <video/dviconfig.h>
#include "dviplugin.h"
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/kthread.h>

#define XYFIFOSIZE 3

// ========================================================================================
// Static helper functions
// ========================================================================================
/*
 * Helper function to redirect mouse/touch events to USB device (Cypress)
 */
void Redirect2USBDEvice(struct dviplugin_data *pDevice)
{
	int mouse_frame[4] = {0,0,0,0};
	static int x_old[XYFIFOSIZE], y_old[XYFIFOSIZE], lbutt_old;
	u32 tmp;
	int i;

	int x = pDevice->mouseevent_x;
	int y = pDevice->mouseevent_y;
	char lbutt = pDevice->mouseevent_lbutt;
	char rbutt = pDevice->mouseevent_rbutt;

	// Filtering when touch pressed
	if((lbutt_old==0) && (lbutt))
	{
		for(i=0; i<XYFIFOSIZE; i++)
		{
			x_old[i] = x;
			y_old[i] = y;
		}
	}

	if(lbutt || lbutt_old)
	{
		int xo = x_old[0];
		int yo = y_old[0];

		for(i=0; i<(XYFIFOSIZE-1); i++)
		{
			x_old[i] = x_old[i+1];
			y_old[i] = y_old[i+1];
		}

		x_old[XYFIFOSIZE-1] = x;
		y_old[XYFIFOSIZE-1] = y;

		x = xo;
		y = yo;
	}
	lbutt_old = lbutt;

	// Coordinate scaling (the range 0...999 is mapped to -127...+127 range)
	if(x < 0)
		x = 0;

	if(x > MAXCOORDVALUE)
		x = MAXCOORDVALUE;

	if(y < 0)
		y = 0;

	if(y > MAXCOORDVALUE)
		y = MAXCOORDVALUE;

	x = (((x * 254) + 127) / (MAXCOORDVALUE)) - 127;
	y = (((y * 254) + 127) / (MAXCOORDVALUE)) - 127;

	if(x > 127)
		x = 127;

	if(x < -127)
		x = -127;

	if(y > 127)
		y = 127;

	if(y < -127)
		y = -127;

	mouse_frame[1] = x;
	mouse_frame[2] = y;

	// Buttons
	if(lbutt)
		mouse_frame[0] |= 0x01;
	
	if(rbutt)
		mouse_frame[0] |= 0x02;
	
	//And now, try to dispatch the mouseevent
	tmp = DVIreadReg(pDevice->spi, REG_HID_STATUS);
	if(tmp & EMPTY_MASK) 
	{
		//fifo is empty, dispatch the frame...
		int i;
		// write the frame in fifo
		for (i = 0; i < 4; i++)
		{
			DVIwriteReg(pDevice->spi, REG_HID_DATA, mouse_frame[i]);
		}
		
		// commit the frame to cypress            
		DVIwriteReg(pDevice->spi, REG_HID_CTL, COMMIT_MASK); 
	}
}


/* 
 * MouseThread thread for forwarding the mouse/touch coordinates 
 */
int  mouse_thread(void* p)
{
	struct dviplugin_data *pDevice = dev_get_drvdata(p);
	int infinite_timeout = 1;

	pDevice->mouseevent_x = 0;
	pDevice->mouseevent_y = 0;
	pDevice->mouseevent_lbutt = 0;
	pDevice->mouseevent_rbutt = 0;

	while(!kthread_should_stop())
	{
	  if (kthread_should_stop())
	    break;
	  
	  if(infinite_timeout)
	    wait_event_interruptible(pDevice->send_wq, pDevice->mouseevent_flag);
	  else
	    wait_event_interruptible_timeout(pDevice->send_wq, pDevice->mouseevent_flag, 1); //1 jiffies timeout

	  mutex_lock(&pDevice->lock);
	  pDevice->mouseevent_flag = 0;
	  Redirect2USBDEvice(pDevice);
	  
	  if(pDevice->mouseevent_lbutt)
	    infinite_timeout = 0;
	  else
    	    infinite_timeout = 1;

	  mutex_unlock(&pDevice->lock);
	  msleep(5);
	}
	return 0;
}


// ========================================================================================
// Exported functions
// ========================================================================================

/* 
 * Monitoring thread for polling the actual status of the DVI cores and eventually take 
 * the proper decisions accordingly.
 */
int monitor_thread(void *p)
{
  struct dviplugin_data *pDevice = dev_get_drvdata(p);
  volatile u32 wce_cvistatus = 0;
  volatile u32 adp_cvostatus = 0;
  volatile u32 dvi_cvistatus = 0;
  volatile u32 dvi_cvostatus = 0;
  volatile u32 dvi_samplecount = 0;
  volatile u32 dvi_linecount = 0;
  u32 dvi_cvistatus_old = 0;
  u32 dvi_cvi_enabled_old = 0;
  u32 dvi_samplecount_old = 0;
  u32 dvi_linecount_old = 0;
  u32 need_reinit_vidin = 0;
  
  while (!kthread_should_stop()) 
  {
    if (kthread_should_stop())
      break;
    
    mutex_lock(&pDevice->lock);
    adp_cvostatus = DVIreadReg(pDevice->spi, ADP_CVO_BASE + CVO_STATUS);
    wce_cvistatus = DVIreadReg(pDevice->spi, WCE_CVI_BASE + CVI_STATUS);
    dvi_cvostatus = DVIreadReg(pDevice->spi, DVI_CVO_BASE + CVO_STATUS);
    dvi_cvistatus = DVIreadReg(pDevice->spi, DVI_CVI_BASE + CVI_STATUS);
    dvi_samplecount = DVIreadReg(pDevice->spi, DVI_CVI_BASE + CVI_SAMPLECOUNT);
    dvi_linecount = DVIreadReg(pDevice->spi, DVI_CVI_BASE + CVI_LINECOUNT);
    pDevice->dvi_cvi_enabled = (DVIreadReg(pDevice->spi, DVIINPUTSTATREG)) & 0x01;
    mutex_unlock(&pDevice->lock);
#if 0    
    printk("%s DVI CVI video signal presence=0x%x \n",__func__, pDevice->dvi_cvi_enabled);
    printk("%s adp_cvostatus=0x%x \n",__func__, adp_cvostatus);
    printk("%s wce_cvistatus=0x%x \n",__func__, wce_cvistatus);
    printk("%s dvi_cvostatus=0x%x \n",__func__, dvi_cvostatus);
    printk("%s dvi_cvistatus=0x%x \n",__func__, dvi_cvistatus);
    printk("%s dvi_samplecount=0x%x \n",__func__, dvi_samplecount);
    printk("%s dvi_linecount=0x%x \n",__func__, dvi_linecount);
#endif    
    //Update the DVI input port status infos
    if((pDevice->dvi_cvi_enabled) && ((dvi_cvistatus & DVI_CVIINPUTISVALID) == DVI_CVIINPUTISVALID))
    {
      pDevice->DVIinStatus = ((dvi_samplecount & 0xffff) | ((dvi_linecount & 0xffff)<<16));
    }
    else
    {
      pDevice->DVIinStatus = 0;
    }
    
    //Reinitialize the WCE_CVI and ADP_CVO cores if at least one of them should be enabled but is not actually producing data
    if( (pDevice->wce_cvi_enabled && !(wce_cvistatus & 0x01)) || (pDevice->adp_cvo_enabled && ((adp_cvostatus & 0x01) != 0x01)))
    {
      printk("%s DVIplugin: reinitializing cores...\n",__func__);
      InitFPGACores(pDevice);
    }
    
    //Check if something changed and so the videoinput subsection needs to be reconfigured 
    need_reinit_vidin = 0;
    if(pDevice->dvi_cvi_enabled != dvi_cvi_enabled_old) 
      need_reinit_vidin = 1;
    
    if(dvi_cvistatus_old != dvi_cvistatus)
      need_reinit_vidin = 1;
    
    if(dvi_samplecount_old != dvi_samplecount)
      need_reinit_vidin = 1;
    
    if(dvi_linecount_old != dvi_linecount)
      need_reinit_vidin = 1;
    
    dvi_linecount_old = dvi_linecount;
    dvi_samplecount_old = dvi_samplecount;
    dvi_cvistatus_old = dvi_cvistatus;
    dvi_cvi_enabled_old = pDevice->dvi_cvi_enabled;
    
    if(need_reinit_vidin)
      VIDIN_Configure(pDevice);
    
    //The DVI_CVO is actually assumed to be always active; if the core is not producing data, reinit it
    pDevice->dvi_cvo_enabled = 1;
    if(pDevice->dvi_cvo_enabled && !(dvi_cvostatus & 0x01))
    {
      printk("%s DVIplugin: initializing DVI_CVO core...\n",__func__);
      InitDVI_CVO(pDevice);
    }
    
    //The DVI_CVI needs to be enabled if a valid video input signal is detected 
    if(pDevice->dvi_cvi_enabled && !(dvi_cvistatus & 0x01)) 
    {
      printk("%s DVIplugin: initializing DVI_CVI core...\n",__func__);
      InitDVI_CVI(pDevice);
    }
    
    msleep_interruptible(1000);
  }
  
  complete_all(&pDevice->auto_update_stop);
  return 0;
}

/*
 * Initialize the DVI_CVI FPGA cores
 */
int InitDVI_CVI(struct dviplugin_data *pDevice)
{
  mutex_lock(&pDevice->lock);
  //Stop-Start 
  DVIwriteReg(pDevice->spi, DVI_CVI_BASE + CVI_CONTROL, 0);
  msleep(1);
  
  // Initialize the WCE_CVI cores (just need to be enabled)
  if(! DVIwriteReg(pDevice->spi, DVI_CVI_BASE + CVI_CONTROL, 1))
  {
    printk("%s unable to perform access to FPGA core registers\n",__func__);
    mutex_unlock(&pDevice->lock);
    return -1;
  }
  
  mutex_unlock(&pDevice->lock);  
  return 0;
}

/*
 * Initialize the DVI_CVO FPGA core
 */
int InitDVI_CVO(struct dviplugin_data *pDevice)
{
  mutex_lock(&pDevice->lock);
  //Stop-Start 
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_CONTROL, 0);
  msleep(1);
  
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_CONTROL,     0);     // OFF CVO, just in case
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_VALID, 0);     // Mode 1 invalid
  //Set timings and params according with the DVI chain settings...
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_CTRL,  0);     // Progressive, parallel
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_XRES,  pDevice->DVIparams.rezx);
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_VRES,  pDevice->DVIparams.rezy);     
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_HSFP,  pDevice->DVIparams.hs_fp);     
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_HSW,  pDevice->DVIparams.hs_w);     
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_HSSUM,  pDevice->DVIparams.hs_w + pDevice->DVIparams.hs_fp + pDevice->DVIparams.hs_bp);     
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_VSFP,  pDevice->DVIparams.vs_fp);     
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_VSW,  pDevice->DVIparams.vs_w);     
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_VSSUM,  pDevice->DVIparams.vs_w + pDevice->DVIparams.vs_fp + pDevice->DVIparams.vs_bp);     
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_MODEX_VALID, 1);     // Mode 1 valid
  DVIwriteReg(pDevice->spi, DVI_CVO_BASE + CVO_CONTROL,     1);     // ON CVO

  mutex_unlock(&pDevice->lock);  
  return 0;
}

/*
 * Initialize the PLL to the desired frequency
 */
void PLL_Init(struct dviplugin_data *pDevice)
{
	int i;
	int best = 0;
	int min_err = 1000000;
	int err;
	u32 fkhz = pDevice->DVIparams.pclk_freq;

	PLL_DATA plldata[] = {
		//f_khz   PLL_MODE PLL_NCOUNT PLL_MCOUNT PLL_CCOUNT PLL_PHASE PLL_KFRAC PLL_BANDWIDTH PLL_CHARGEPUMP PLL_VCODIV
		 {108000, 0x01,    0x202,     0x20e0d,   0x20201,   0x200001, 0xffffff, 0x07,         0x01,          0x00},
		 {65000,  0x01,    0x202,     0x22120,   0x606,     0x200001, 0xffffff, 0x06,         0x01,          0x00},
		 {162000, 0x01,    0x202,     0x22e0d,   0x601,     0x200001, 0xffffff, 0x07,         0x01,          0x00},
		 {148000, 0x01,    0x202,     0x23231,   0x404,     0x200001, 0xffffff, 0x04,         0x01,          0x00},
		 {76000,  0x01,    0x20403,   0x23E3D,   0x20605,   0x200001, 0xffffff, 0x04,         0x01,          0x00},
		 {0     , 0,       0,         0,         0,         0,        0,        0,            0,             0x00},
	};
	
	printk("%s fkhz=%d\n",__func__, fkhz );

	//Search best fit PLL frequency
	for(i=0; plldata[i].f_Khz !=0; i++)
	{
		if(plldata[i].f_Khz > fkhz)
			err = plldata[i].f_Khz - fkhz;
		else
			err = fkhz - plldata[i].f_Khz;

		if(err < min_err)
		{
			best = i;
			min_err = err;
		}
	}

	printk("%s configuring PLL to %d khz\n",__func__, plldata[best].f_Khz );

	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_MODE, plldata[best].pll_mode); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_NCOUNT, plldata[best].pll_ncount); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_MCOUNT, plldata[best].pll_mcount); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_CCOUNT, plldata[best].pll_ccount); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_PHASE, plldata[best].pll_phase); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_KFRAC, plldata[best].pll_kfrac); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_BANDWIDTH, plldata[best].pll_bandwidth); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_CHARGEPUMP, plldata[best].pll_chargepump); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_VCODIV, plldata[best].pll_vcodiv); 
	DVIwriteReg(pDevice->spi, DVIPLLBASE + PLL_START, 0x00); 
}

/*
 * Initialize the FPGA cores
 */
int InitFPGACores(struct dviplugin_data *pDevice)
{
	u32 dctrl = 0;

	mutex_lock(&pDevice->lock);

	//Start with both WCE_CVI and ADP_CVO cores in OFF status
	DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_CONTROL, 0);
	DVIwriteReg(pDevice->spi, WCE_CVI_BASE + CVI_CONTROL, 0);
	DVIwriteReg(pDevice->spi, WCE_CVI3_BASE + CVI_CONTROL, 0);
	pDevice->adp_cvo_enabled = 0;
	pDevice->wce_cvi_enabled = 0;
	msleep(1);

	// Setup sync polarities on output (both local and DVI outs)
	//DVI out
	dctrl |= DCTRL_VSYNC_EN | DCTRL_HSYNC_EN | DCTRL_BLANK_EN;

	if(pDevice->DVIparams.hs_inv)
		dctrl |= DCTRL_HSYNC_INV;
	
	if(pDevice->DVIparams.vs_inv)
		dctrl |= DCTRL_VSYNC_INV;
	
	if(pDevice->DVIparams.blank_inv)
		dctrl |= DCTRL_BLANK_INV;
	
	if(pDevice->DVIparams.pclk_inv)
		dctrl |= DCTRL_VCLK_INV;

	dctrl = (dctrl << 8) & 0xff00;
	//Local display out (if local display present)
	if(pDevice->ADPparams.pclk_freq != 0)
	{
		dctrl |= DCTRL_VSYNC_EN | DCTRL_HSYNC_EN | DCTRL_BLANK_EN;

		if(pDevice->ADPparams.hs_inv)
			dctrl |= DCTRL_HSYNC_INV;
		
		if(pDevice->ADPparams.vs_inv)
			dctrl |= DCTRL_VSYNC_INV;
		
		if(pDevice->ADPparams.blank_inv)
			dctrl |= DCTRL_BLANK_INV;
		
		if(pDevice->ADPparams.pclk_inv)
			dctrl |= DCTRL_VCLK_INV;
	}
	DVIwriteReg(pDevice->spi, DATACTRL_REG, dctrl);

	// Alpha layer L1 (Vidin) off, Enable Scaler, default cfg
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L1_ENA, 0);
	DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_WIDTH, 64);
	DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_HEIGHT, 64);
	DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_CONTROL, 1);

	// Initialize the fractional PLL for generating the required DVI pixel clock 
	PLL_Init(pDevice);
	msleep(100);

	// Initialize the WCE_CVI core (just need to be enabled)
	if(DVIwriteReg(pDevice->spi, WCE_CVI_BASE + CVI_CONTROL, 1))
	{
		printk("%s ERROR: unable to perform access to FPGA core registers\n",__func__);
		mutex_unlock(&pDevice->lock);
		return -1;
	}
	DVIwriteReg(pDevice->spi, WCE_CVI3_BASE + CVI_CONTROL, 1);
	pDevice->wce_cvi_enabled = 1;

	// If the local display is used, set the ADP_CVO accordingly with APD params...
	if(pDevice->ADPparams.pclk_freq != 0)
	{
		printk("%s Initializing the ADP_CVO core with local display params...\n",__func__);
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_CONTROL,     0);     // OFF CVO, just in case
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VALID, 0);     // Mode 1 invalid
		//Set timings and params...
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_CTRL,  0);     // Progressive, parallel
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_XRES,  pDevice->ADPparams.rezx);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VRES,  pDevice->ADPparams.rezy);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_HSFP,  pDevice->ADPparams.hs_fp);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_HSW,  pDevice->ADPparams.hs_w);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_HSSUM,  pDevice->ADPparams.hs_w + pDevice->ADPparams.hs_fp + pDevice->ADPparams.hs_bp);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VSFP,  pDevice->ADPparams.vs_fp);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VSW,  pDevice->ADPparams.vs_w);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VSSUM,  pDevice->ADPparams.vs_w + pDevice->ADPparams.vs_fp + pDevice->ADPparams.vs_bp);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VALID, 1);     // Mode 1 valid
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_CONTROL,     1);     // ON CVO
		pDevice->adp_cvo_enabled = 1;                                      // Set the corresponding flag
	}
	else
	{
		printk("%s Initializing the ADP_CVO core with DVI display params...\n",__func__);
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_CONTROL,     0);     // OFF CVO, just in case
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VALID, 0);     // Mode 1 invalid
		//Set timings and params...
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_CTRL,  0);     // Progressive, parallel
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_XRES,  pDevice->DVIparams.rezx);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VRES,  pDevice->DVIparams.rezy);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_HSFP,  pDevice->DVIparams.hs_fp);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_HSW,  pDevice->DVIparams.hs_w);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_HSSUM,  pDevice->DVIparams.hs_w + pDevice->DVIparams.hs_fp + pDevice->DVIparams.hs_bp);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VSFP,  pDevice->DVIparams.vs_fp);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VSW,  pDevice->DVIparams.vs_w);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VSSUM,  pDevice->DVIparams.vs_w + pDevice->DVIparams.vs_fp + pDevice->DVIparams.vs_bp);     
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_MODEX_VALID, 1);     // Mode 1 valid
		DVIwriteReg(pDevice->spi, ADP_CVO_BASE + CVO_CONTROL,     1);     // ON CVO
		pDevice->adp_cvo_enabled = 1;                                      // Set the corresponding flag
	}
	
	//HID mouse out of reset
	DVIwriteReg(pDevice->spi, REG_RESET, HID_MASK & ~RESET_MASK); 

	mutex_unlock(&pDevice->lock);

	//Initialize the alpha blender core
	VIDIN_Configure(pDevice);

	//Configure the video switch
	DVI_Configureswitch(pDevice);
	return 0;
}

/*
 * Configure the DVI output switch
 */
int DVI_Configureswitch(struct dviplugin_data *pDevice)             
{
	mutex_lock(&pDevice->lock);

	//Stop the TFP410 DVI output chip
	if(TFP410_Stop(pDevice->i2cadap))
	{
		msleep(100);
		TFP410_Stop(pDevice->i2cadap);
	}

	msleep(50);

	//Step 0: Stop all affected cores
	DVIwriteReg(pDevice->spi, WCE_SCALER_BASE + SCALER_CONTROL, 0);
	DVIwriteReg(pDevice->spi, WCE_CVI2_BASE + CVI_CONTROL, 0);
	DVIwriteReg(pDevice->spi, DVO_TP_BASE + TP_CONTROL, 0);
	DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_CONTROL, 0);
	msleep(20);

	//Step 1: Initialize the DVO Test Pattern generator II to produce a black background
	DVIwriteReg(pDevice->spi, DVO_TP_BASE + TP_WIDTH, pDevice->DVIparams.rezx);
	DVIwriteReg(pDevice->spi, DVO_TP_BASE + TP_HEIGHT, pDevice->DVIparams.rezy);
	DVIwriteReg(pDevice->spi, DVO_TP_BASE + TP_R, 0);
	DVIwriteReg(pDevice->spi, DVO_TP_BASE + TP_G, 0);
	DVIwriteReg(pDevice->spi, DVO_TP_BASE + TP_B, 0);

	if(pDevice->ADPparams.pclk_freq != 0)
	{ //Step 2.1: If local display is used, scale the WCE image to the DVIout resolution and keep the aspect ratio
		int kx = (1000 * pDevice->DVIparams.rezx) / pDevice->ADPparams.rezx;
		int ky = (1000 * pDevice->DVIparams.rezy) / pDevice->ADPparams.rezy;

		if(kx < ky)
		{ // Fit to x-axis, border on y-axis
			unsigned int rezy;
			DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_L1_X, 0);
			DVIwriteReg(pDevice->spi, WCE_SCALER_BASE + SCALER_WIDTH, pDevice->DVIparams.rezx);

			rezy = (pDevice->DVIparams.rezx * pDevice->ADPparams.rezy) / pDevice->ADPparams.rezx;
			if(rezy > pDevice->DVIparams.rezy)
				rezy = pDevice->DVIparams.rezy;

			DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_L1_Y, (pDevice->DVIparams.rezy - rezy)/2 );
			DVIwriteReg(pDevice->spi, WCE_SCALER_BASE + SCALER_HEIGHT, rezy);
		}
		else
		{ // Fit to y-axis, border on x-axis
			unsigned int rezx;
			DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_L1_Y, 0);
			DVIwriteReg(pDevice->spi, WCE_SCALER_BASE + SCALER_HEIGHT, pDevice->DVIparams.rezy);

			rezx = (pDevice->DVIparams.rezy * pDevice->ADPparams.rezx) / pDevice->ADPparams.rezy;
			if(rezx > pDevice->DVIparams.rezx)
				rezx = pDevice->DVIparams.rezx;

			DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_L1_X, (pDevice->DVIparams.rezx - rezx)/2 );
			DVIwriteReg(pDevice->spi, WCE_SCALER_BASE + SCALER_WIDTH, rezx);
		}
	}
	else
	{ //Step 2.2: No local display used, use the DVI resolution for the DVO image output
		DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_L1_X, 0);
		DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_L1_Y, 0);
		DVIwriteReg(pDevice->spi, WCE_SCALER_BASE + SCALER_WIDTH, pDevice->DVIparams.rezx);
		DVIwriteReg(pDevice->spi, WCE_SCALER_BASE + SCALER_HEIGHT, pDevice->DVIparams.rezy);
	}

	//Step 3: Enable all the affected cores
	DVIwriteReg(pDevice->spi, WCE_SCALER_BASE + SCALER_CONTROL, 1);
	DVIwriteReg(pDevice->spi, WCE_CVI2_BASE + CVI_CONTROL, 1);
	DVIwriteReg(pDevice->spi, DVO_TP_BASE + TP_CONTROL, 1);
	DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_CONTROL, 1);
	DVIwriteReg(pDevice->spi, DVO_ALPHA_BASE + ALPHA_L1_ENA, 1);
	msleep(20);

	if((pDevice->seldviout == SHOW_DVI_INPUT) || (pDevice->seldviout == SHOW_DVI_INPUT2)) // Show WCE input on local display (if any), DVI input on ext DVI monitor 
	{
		printk("%s Show OS input on local display + DVI passthrough\n",__func__);
		DVIwriteReg(pDevice->spi, DVISELOUTREG, pDevice->adp_dualLVDS | 0x01);

		pca9557_set_out(pDevice->i2cadap, 4); //select2=1 USB passthrough

		if((pDevice->ADPparams.pclk_freq != 0) && (pDevice->seldviout == SHOW_DVI_INPUT2))
			pca9557_clr_out(pDevice->i2cadap, 3); //select1=0 (If local display is present and SHOW_DVI_INPUT2, connect CYPRESS USB device)
		else
			pca9557_set_out(pDevice->i2cadap, 3); //select1=1 (passthrough ext. touch)
	}
	else                                  // Show WCE input on ext DVI out monitor
	{
		printk("%s Show OS input DVI output\n",__func__);
		DVIwriteReg(pDevice->spi, DVISELOUTREG, pDevice->adp_dualLVDS);

		// If the local display is used, connect the Cypress to the USB device port
		if(pDevice->ADPparams.pclk_freq != 0)
		{
			pca9557_clr_out(pDevice->i2cadap, 3); //select1=0 
			pca9557_set_out(pDevice->i2cadap, 4); //select2=1 
		}
		else
		{
			pca9557_clr_out(pDevice->i2cadap, 3); //select1=0 
			pca9557_clr_out(pDevice->i2cadap, 4); //select2=0 
		}
	}
	//DVIwriteReg(pDevice->spi, MISCCTRLREG, 0x00); //Set Cypress and USB hubs to Reset state
	pca9557_clr_out(pDevice->i2cadap, 5); //Set Cypress and USB hubs to Reset state
	msleep(1500);
	pca9557_set_out(pDevice->i2cadap, 5); //Release Cypress and USB hubs from Reset state
	//DVIwriteReg(pDevice->spi, MISCCTRLREG, 0x04); //Release Cypress and USB hubs from Reset state

	// Reenable the TFP410 DVI out chip
	if(TFP410_Configure(pDevice->i2cadap))
	{
		msleep(100);
		TFP410_Configure(pDevice->i2cadap);
	}

	mutex_unlock(&pDevice->lock);
	return 0;
}

/*
 * Video input configuration
 */
int VIDIN_Configure(struct dviplugin_data* pDevice)
{
	volatile u32 dvi_cvistatus = 0;
	volatile u32 dvi_samplecount = 0;
	volatile u32 dvi_linecount = 0;
	
	printk("%s called\n",__func__);

	mutex_lock(&pDevice->lock);

	dvi_cvistatus = DVIreadReg(pDevice->spi, DVI_CVI_BASE + CVI_STATUS);
	dvi_samplecount = DVIreadReg(pDevice->spi, DVI_CVI_BASE + CVI_SAMPLECOUNT);
	dvi_linecount = DVIreadReg(pDevice->spi, DVI_CVI_BASE + CVI_LINECOUNT);
	pDevice->dvi_cvi_enabled = (DVIreadReg(pDevice->spi, DVIINPUTSTATREG)) & 0x01;

	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L1_ENA, 0);
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L2_ENA, 0);
	msleep(50);
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_CONTROL, 1);
	msleep(500);

	//The layer 0 is WCE, always enabled, used for setting the image size of the background layer.

	//The layer 2 (WCE) has offset=0,0 and is always enabled
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L2_X, 0);
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L2_Y, 0);
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L2_ENA, 1);

	//The layer 1 (DVI rescaled videoinput) follows the videoinput configuration
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L1_X, pDevice->vidin.xoffs);
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L1_Y, pDevice->vidin.yoffs);
	if(pDevice->vidin.enabled != 0)
	{   //If videoinput enabled...
		if(!(pDevice->dvi_cvi_enabled) || ((dvi_cvistatus & DVI_CVIINPUTISVALID) != DVI_CVIINPUTISVALID))
		{	//...disable the L1 DVI layer if DVI input stream missing or not valid/stable
			DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L1_ENA, 0);
			DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_WIDTH, 64);
			DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_HEIGHT, 64);
			DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_CONTROL, 1);
		}
		else
		{	//...enable the L1 DVI layer and set the scaler accordingly if the DVI input stream is valid
			DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_WIDTH, pDevice->vidin.width);
			DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_HEIGHT, pDevice->vidin.height);
			DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_CONTROL, 1);
			printk("%s Scaler enabled width=%d height=%d\n",__func__, pDevice->vidin.width, pDevice->vidin.height);
			msleep(20);
			DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L1_ENA, 1);
		}
	}
	else
	{
		DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_L1_ENA, 0);
		DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_WIDTH, 64);
		DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_HEIGHT, 64);
		DVIwriteReg(pDevice->spi, VIDIN_SCALER_BASE + SCALER_CONTROL, 1);
	}

	//Then enable the whole alpha blender core
	DVIwriteReg(pDevice->spi, VIDIN_ALPHA_BASE + ALPHA_CONTROL, 1);

	mutex_unlock(&pDevice->lock);
	return 0;
}
