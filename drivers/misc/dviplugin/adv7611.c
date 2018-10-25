/*
 *  adv7611 - Helper functions to handle the adv7611 chip (DVI out) via I2C bus
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

//------------------------------------------------------------------------------
// Module defines
//------------------------------------------------------------------------------
// Device address
#define ADV7611ADDR         0x4C

// Device registers
#define ADV7611_REV         0xea
#define ADV7611_PWRCTRL     0x0c
#define ADV7611_SETPOL      0x06
#define ADV7611_OUTFMT      0x03
#define ADV7611_COLORSPACE  0x02

static inline int adv7611_writereg(struct i2c_adapter  *adap, unsigned char devaddr, unsigned char regoffset, unsigned char value)
{
  int err;
  struct i2c_msg msg[1];
  unsigned char data[2];
  
  msg->addr = devaddr;     /* Device address */
  msg->flags = 0;
  msg->len = 2;
  msg->buf = data;
  data[0] = regoffset;     /* register num */
  data[1] = value;         /* register data */
  err = i2c_transfer(adap, msg, 1);
  if (err >= 0)
    return 0;
  return err;
}

//------------------------------------------------------------------------------
// Exported functions
//------------------------------------------------------------------------------

/* adv7611_CheckPresence()
 * Check if chip is present by trying to perform a dummy write to a readonly register
 */
int adv7611_CheckPresence(struct i2c_adapter  *adap)
{
  if(adv7611_writereg(adap, ADV7611ADDR, ADV7611_REV, 0xff))
    if(adv7611_writereg(adap, ADV7611ADDR, ADV7611_REV, 0xff))
      return -1;

  return 0;
}


/* adv7611_Configure()
 * ADV7611 chip configuration
 */
int adv7611_Configure(struct i2c_adapter  *adap)
{
  int ret = 0;
  int i;
  u8 val;
  u8 reg;
  u8 sa;
  
  u32 setting[] = {
  //0xaarrvv  aa=8bit slave address  rr=register offset  vv=value
  0x98F480, // CEC
  0x98F57C, // INFOFRAME
  0x98F84C, // DPLL
  0x98F964, // KSV
  0x98FA6C, // EDID
  0x98FB68, // HDMI
  0x98FD44, // CP
  0x980106, // Prim_Mode =110b HDMI-GR
  0x980013, //!!! VID per 1366x768
  0x980212, // RGB color mapping
  0x980340, // 24 bit SDR 444 Mode 0
  0x980528, // AV Codes Off
  0x9806A6, // !!! Invert VS,HS pins
  0x980B44, // Power up part
  0x980C42, // Power up part
  0x98143F, // Max Drive Strength
  0x981580, // Disable Tristate of Pins
  0x981983, // LLC DLL phase
  0x983340, // LLC DLL enable
  0x44BA01, // Set HDMI FreeRun
  0x644081, // Disable HDCP 1.1 features
  0x689B03, // ADI recommended setting
  0x68C101, // ADI recommended setting
  0x68C201, // ADI recommended setting
  0x68C301, // ADI recommended setting
  0x68C401, // ADI recommended setting
  0x68C501, // ADI recommended setting
  0x68C601, // ADI recommended setting
  0x68C701, // ADI recommended setting
  0x68C801, // ADI recommended setting
  0x68C901, // ADI recommended setting
  0x68CA01, // ADI recommended setting
  0x68CB01, // ADI recommended setting
  0x68CC01, // ADI recommended setting
  0x680000, // Set HDMI Input Port A
  0x6883FE, // Enable clock terminator for port A
  0x686F08, // ADI recommended setting
  0x68851F, // ADI recommended setting
  0x688770, // ADI recommended setting
  0x688D04, // LFG
  0x688E1E, // HFG
  0x681A8A, // unmute audio
  0x6857DA, // ADI recommended setting
  0x685801, // ADI recommended setting
  0x6C4003, // Suggested for DVI
  };
  
  for(i=0; i<sizeof(setting)/4;i++)
  {
    sa = (u8)((setting[i] >> 17) & 0xff);
    reg = (u8)((setting[i] >> 8) & 0xff);
    val = (u8)((setting[i] >> 0) & 0xff);
    
    printk("%s sa=0x%x reg=0x%x val=0x%x\n", __func__, sa, reg, val);
    if(!ret)
    {
      ret = adv7611_writereg(adap, sa, reg, val);
      if(ret)
	printk("%s FAILED writing registers\n", __func__);
    }
  }
  //Set sync polarity (fixed value, all signals set to active HI)
  val = 0xa0;
  if(!ret)
    ret = adv7611_writereg(adap, ADV7611ADDR, ADV7611_SETPOL, val);
  return ret;
}
