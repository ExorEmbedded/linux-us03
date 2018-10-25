/*
 *  pca9557 - Helper functions to handle the pca9557 i/o expander of the plugin module
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
#define PCA9557ADDR         0x18

// Device registers
#define PCA9557_OUT         0x01
#define PCA9557_CFG         0x03

static unsigned char outstat = 0xff;

static inline int pca9557_writereg(struct i2c_adapter  *adap, unsigned char regoffset, unsigned char value)
{
  int err;
  struct i2c_msg msg[1];
  unsigned char data[2];
  
  msg->addr = PCA9557ADDR; /* I2C address of chip */
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

/*
 * Init all outputs to 1
 */
int pca9557_init(struct i2c_adapter  *adap)
{
  if(pca9557_writereg(adap, PCA9557_CFG, 0))
    if(pca9557_writereg(adap, PCA9557_CFG, 0))
      return -1;
    
  if(pca9557_writereg(adap, PCA9557_OUT, outstat))
    if(pca9557_writereg(adap, PCA9557_OUT, outstat))
      return -1;
  
  return 0;
}

/*
 * Set the specified output to 1
 */
int pca9557_set_out(struct i2c_adapter  *adap, unsigned char n)
{
  outstat |= (0x01 << n);

  if(pca9557_writereg(adap, PCA9557_OUT, outstat))
    if(pca9557_writereg(adap, PCA9557_OUT, outstat))
      return -1;
  
  return 0;
}

/*
 * Set the specified output to 0
 */
int pca9557_clr_out(struct i2c_adapter  *adap, unsigned char n)
{
  outstat &= ~(0x01 << n);
  
  if(pca9557_writereg(adap, PCA9557_OUT, outstat))
    if(pca9557_writereg(adap, PCA9557_OUT, outstat))
      return -1;
  
  return 0;
}
