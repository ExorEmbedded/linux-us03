/*
 *  tfp410 - Helper functions to handle the tfp410 chip (DVI out) via I2C bus
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
#define TFP410ADDR      0x38

// Device registers
#define TFP_VENDID      0x00
#define TFP_DEVID       0x02
#define TFP_CTL1_MODE   0x08
#define TFP_CTL2_MODE   0x09
#define TFP_CTL3_MODE   0x0A

// Register values and fields
#define CTL1_DEF_MODE   0xb5
#define CTL1_REDGE      0x02
#define CTL1_STOP_MODE  0xb4

static inline int tfp410_writereg(struct i2c_adapter  *adap, unsigned char regoffset, unsigned char value)
{
  int err;
  struct i2c_msg msg[1];
  unsigned char data[2];
  
  msg->addr = TFP410ADDR;  /* I2C address of chip */
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

/* TFP410_CheckPresence()
 * Check presence of of the TFP410 by performing a dummy write access to a Read only register
 */
int TFP410_CheckPresence(struct i2c_adapter  *adap)
{
  if(tfp410_writereg(adap, TFP_VENDID, 0xff))
    if(tfp410_writereg(adap, TFP_VENDID, 0xff))
      return -1;
  
  return 0;
}


/*
 * TFP410 chip configuration 
 */
int TFP410_Configure(struct i2c_adapter  *adap)
{
  if(tfp410_writereg(adap, TFP_CTL1_MODE, CTL1_DEF_MODE))
      if(tfp410_writereg(adap, TFP_CTL1_MODE, CTL1_DEF_MODE))
	return -1;
  
  return 0;
}


/*
 * TFP410 stop
 * Output: TRUE if success
 */
int TFP410_Stop(struct i2c_adapter  *adap)
{
  if(tfp410_writereg(adap, TFP_CTL1_MODE, CTL1_STOP_MODE))
      if(tfp410_writereg(adap, TFP_CTL1_MODE, CTL1_STOP_MODE))
	return -1;
  
  return 0;

}