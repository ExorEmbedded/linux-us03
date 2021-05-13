/*
 *  plxx_manager.c - Linux plugins manager driver
 *
 *  Written by: Giovanni Pavoni, Exor S.p.a.
 *  Copyright (c) 2016 Exor S.p.a.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/I2CSeeprom.h>

#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/plxx_manager.h>
#include <linux/nvmem-consumer.h>

#define SEE_FUNCAREA_NBITS (SEE_FUNCAREALEN * 8)
#define FULLEEPROMSIZE (256)

static DEFINE_MUTEX(plxx_lock);

struct plxx_data 
{
  struct nvmem_device *    seeprom_device;
  struct nvmem_device *    ioexp_device;
  u32                      index;                   // Plugin index
  u32                      sel_gpio;                // Gpio index to select the plugin
  u32                      installed;               // Indicates if the plugin is physically installed
  u8                       eeprom[FULLEEPROMSIZE];  // Image of the I2C SEEPROM contents of the plugin
  bool                     f_updated;               // Flag indicating if datas for the current plugin were still taken
  u8                       plcmversion;             // Flag indicating PCLM version (09,10,...)
};

/* -------------------------------------------------------------------------------------------------------------- *
 *
 * PLCMxx related stuff
 * 
 * -------------------------------------------------------------------------------------------------------------- */
#define PLCMxx_INREG  (0x00)
#define PLCMxx_OUTREG (0x01)
#define PLCMxx_CFGREG (0x03)

#define PLCMxx_VERSION_INVALID  0
#define PLCMxx_VERSION_09  9
#define PLCMxx_VERSION_10 10
#define PLCMxx_VERSION_11 11 // PLCM10 ONLY 4G
#define PLCMxx_VERSION_12 12 // PLCM10 ONLY WIFI


/* -------------------------------------------------------------------------------------------------------------- *
 *
 * PLCM09 related stuff
 * 
 * -------------------------------------------------------------------------------------------------------------- */
#define PLCM09_U3ADDR (0x22) //Addr 0x74
#define PLCM09_U5ADDR (0x21) //Addr 0x72

#define PLCM09_LED1        PLCM09_U3ADDR,1
#define PLCM09_LED2        PLCM09_U3ADDR,2
#define PLCM09_LED_POL     PLCM09_U3ADDR,3
#define PLCM09_XO1         PLCM09_U3ADDR,4
#define PLCM09_XO2         PLCM09_U3ADDR,5
#define PLCM09_XI1         PLCM09_U3ADDR,6
#define PLCM09_XI2         PLCM09_U3ADDR,7

#define PLCM09_PWR_KEY     PLCM09_U5ADDR,0
#define PLCM09_PWR_DW      PLCM09_U5ADDR,1
#define PLCM09_RESET       PLCM09_U5ADDR,2
#define PLCM09_RING        PLCM09_U5ADDR,5
#define PLCM09_AP_READY    PLCM09_U5ADDR,7

/* -------------------------------------------------------------------------------------------------------------- *
 *
 * PLCM10 related stuff
 * 
 * -------------------------------------------------------------------------------------------------------------- */
/*
#define PLCM10_U5ADDR  (0x22) //Addr 0x74
#define PLCM10_U6ADDR  (0x21) //Addr 0x72
#define PLCM10_U23ADDR (0x23) //Addr 0x76
*/
#define PLCM10_U5ADDR  (0x3A) //Addr 0x74
#define PLCM10_U6ADDR  (0x39) //Addr 0x72
#define PLCM10_U23ADDR (0x3B) //Addr 0x76

#define PLCM10_RST_HUB    PLCM10_U5ADDR,0
#define PLCM10_LED1       PLCM10_U5ADDR,1
#define PLCM10_LED2       PLCM10_U5ADDR,2
#define PLCM10_LED_POL    PLCM10_U5ADDR,3
#define PLCM10_XO1        PLCM10_U5ADDR,4
#define PLCM10_XO2        PLCM10_U5ADDR,5
#define PLCM10_XI1        PLCM10_U5ADDR,6
#define PLCM10_XI2        PLCM10_U5ADDR,7

#define PLCM10_PWR_KEY     PLCM10_U6ADDR,0
#define PLCM10_RESET       PLCM10_U6ADDR,1
#define PLCM10_WAKEUP      PLCM10_U6ADDR,2
#define PLCM10_W_DISABLE   PLCM10_U6ADDR,3
#define PLCM10_AP_READY    PLCM10_U6ADDR,4
#define PLCM10_EN_BUS      PLCM10_U6ADDR,5
#define PLCM10_ENT_IN      PLCM10_U6ADDR,6
#define PLCM10_STATUS_IN   PLCM10_U6ADDR,7

#define PLCM10_WL_EN       PLCM10_U23ADDR,0
#define PLCM10_WL_RST      PLCM10_U23ADDR,1

#define shiftb(a,x) (0x01 << (x))
#define b(...) shiftb(__VA_ARGS__)


static char* plcm10_get_devName( int plcmversion )
{
  switch (plcmversion)
  {
    case PLCMxx_VERSION_09 : return "PLCM09";
    case PLCMxx_VERSION_10 : return "PLCM10";
    case PLCMxx_VERSION_11 : return "PLCM11";
    case PLCMxx_VERSION_12 : return "PLCM12";
    default : return "PLCM??";
  }
}

/* Checks the PLCM09 presence and, if present, initializes the gpio expanders
 */
static int plcm09_init(struct plxx_data *data)
{
  struct i2c_msg msg;
  int ret = 0;
  unsigned char buf[2];

  struct i2c_adapter* adapter = i2c_get_adapter(CONFIG_PLXX_I2C_BUS);
  printk("plxx switch i2c bus to %d\n",CONFIG_PLXX_I2C_BUS);
  if(!adapter)
    return -1;

#ifndef CONFIG_SOC_IMX6Q
  // PLCM09 disabled on USOM4
  ret = -1;
  goto init_end;
#endif

  //Try to init U3 i2c gpio expander
  msg.addr = PLCM09_U3ADDR;
  msg.flags = 0;
  msg.len = 2;
  msg.buf = buf;

  buf[0] = PLCMxx_OUTREG;
  buf[1] = 0xF1; //All outs = 1 except LED_POL, LED1, LED2
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
    goto init_end;

  buf[0] = PLCMxx_CFGREG;
  buf[1] = 0xC0; //P6,7 = cfg input
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
    goto init_end;

  //Try to init U5 i2c gpio expander
  msg.addr = PLCM09_U5ADDR;
  msg.flags = 0;
  msg.len = 2;
  msg.buf = buf;

  buf[0] = PLCMxx_OUTREG;
  buf[1] = 0x00; //All outs = 0
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
    goto init_end;

  buf[0] = PLCMxx_CFGREG;
  buf[1] = 0xF0; //P3...7 = cfg input
  ret = i2c_transfer(adapter, &msg, 1);

init_end:
  i2c_put_adapter(adapter);
  return ret;
}

/* Checks the PLCM10 presence and, if present, initializes the gpio expanders
 */
static int plcm10_init(struct plxx_data *data)
{
  struct i2c_msg msg;
  int ret = 0;
  unsigned char buf[2];

  struct i2c_adapter* adapter = i2c_get_adapter(CONFIG_PLXX_I2C_BUS);
  if(!adapter)
  {
    printk("plxx i2c_get_adapter error");
    return -1;
  }
  //Try to init U5 i2c gpio expander
  msg.addr = PLCM10_U5ADDR;
  msg.flags = 0;
  msg.len = 2;
  msg.buf = buf;

  buf[0] = PLCMxx_OUTREG;
  buf[1] = 0xFF - (b(PLCM10_LED_POL) | b(PLCM10_LED1) | b(PLCM10_LED2) | b(PLCM10_XO1) | b(PLCM10_XO2));
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
  {
    return ret;
  }
  buf[0] = PLCMxx_CFGREG;
  buf[1] = 0xC0;//b(PLCM10_XI1) | b(PLCM10_XI2);
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
  {
    return ret;
  }

  //Try to init U6 i2c gpio expander
  msg.addr = PLCM10_U6ADDR;
  msg.flags = 0;
  msg.len = 2;
  msg.buf = buf;

  buf[0] = PLCMxx_OUTREG;
  buf[1] = 0x00; //All outs = 0
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
  {
    return ret;
  }
  buf[0] = PLCMxx_CFGREG;
  buf[1] = b(PLCM10_ENT_IN) | b(PLCM10_STATUS_IN);
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
  {
    return ret;
  }

  //Try to init U23 i2c gpio expander
  msg.addr = PLCM10_U23ADDR;
  msg.flags = 0;
  msg.len = 2;
  msg.buf = buf;

  buf[0] = PLCMxx_OUTREG;
  buf[1] = 0x00; //All outs = 0
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
  {
    return ret;
  }
  buf[0] = PLCMxx_CFGREG;
  buf[1] = 0;
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
  {
    return ret;
  }
  return 0;
}

/*
static int plcmxx_init(struct plxx_data *data)
{
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 :
      case PLCMxx_VERSION_11 :
      case PLCMxx_VERSION_12 : return plcm10_init(data);
    default : return plcm09_init(data);
  }
}
*/

/* Sets the level of the specified plcmxx output line
 */
static int plcmxx_set_out(struct plxx_data *data, unsigned char sa, unsigned char pin, unsigned char val)
{
  struct i2c_msg msg;
  int ret = 0;
  unsigned char buf[2];
  unsigned char currval, newval;

  struct i2c_adapter* adapter = i2c_get_adapter(CONFIG_PLXX_I2C_BUS);
  if(!adapter)
    return -1;

  //Get the current OUTREG value 
  msg.addr = sa;
  msg.flags = 0;
  msg.len = 1;
  msg.buf = buf;

  buf[0] = PLCMxx_OUTREG;
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
    return ret;
  
  msg.flags = I2C_M_RD;
  msg.len = 1;
  msg.buf = &currval;
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
    return ret;
  
  //Compute the new OUTREG value 
  if(val)
    newval = currval | (1 << pin);
  else
    newval = currval & (~(1<<pin));
  
  //Write the new OUTREG value
  msg.addr = sa;
  msg.flags = 0;
  msg.len = 2;
  msg.buf = buf;
  
  buf[0] = PLCMxx_OUTREG;
  buf[1] = newval;
  ret = i2c_transfer(adapter, &msg, 1);
  
  return ret;
}

/* Return the level of the specified plcmxx line
 */
static int plcmxx_get_in(struct plxx_data *data, unsigned char sa, unsigned char pin)
{
  struct i2c_msg msg;
  unsigned char regaddr;
  unsigned char currval;
  int ret;

  struct i2c_adapter* adapter = i2c_get_adapter(CONFIG_PLXX_I2C_BUS);
  if(!adapter)
    return -1;

  //Get the current INREG value 
  msg.addr = sa;
  msg.flags = 0;
  msg.len = 1;
  msg.buf = &regaddr;
  regaddr = PLCMxx_INREG;
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
    return ret;
  
  msg.flags = I2C_M_RD;
  msg.len = 1;
  msg.buf = &currval;
  ret = i2c_transfer(adapter, &msg, 1);
  if(ret < 0)
    return ret;
  
  //Compute the return value 
  ret = 0;
  if(currval & (1<<pin))
    ret=1;
  
  return ret;
}

static ssize_t plcmxx_led1_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 1;  
  
  mutex_lock(&plxx_lock);
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 : 
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : if (plcmxx_get_in(data,PLCM10_LED1)==0) tmp = 0; 
                             break;
    default: if (plcmxx_get_in(data,PLCM09_LED1)==0) tmp = 0; 
                             break;
  }
  mutex_unlock(&plxx_lock);

  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcmxx_led1_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  unsigned char tmp = 1;

  mutex_lock(&plxx_lock);

  if(buf[0] == '0')
    tmp = 0;
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 : 
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : plcmxx_set_out(data, PLCM10_LED1, tmp); 
                             break;
    default : plcmxx_set_out(data, PLCM09_LED1, tmp);
                             break;                           
  }
  mutex_unlock(&plxx_lock);

  return size;
}

static ssize_t plcmxx_led2_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 1;
  
  mutex_lock(&plxx_lock);
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 : 
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : if (plcmxx_get_in(data,PLCM10_LED2)==0) tmp = 0; 
                             break;
    default : if (plcmxx_get_in(data,PLCM09_LED2)==0) tmp = 0; 
                             break;                            
  }
  mutex_unlock(&plxx_lock);

  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcmxx_led2_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  unsigned char tmp = 1;

  mutex_lock(&plxx_lock);

  if(buf[0] == '0')
    tmp = 0;
  
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 :
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : plcmxx_set_out(data, PLCM10_LED2, tmp); 
                             break;
    default : plcmxx_set_out(data, PLCM09_LED2, tmp);
                             break;                        
  }
  
  mutex_unlock(&plxx_lock);
  return size;
}

static ssize_t plcm10_pol_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 1;  
  
  mutex_lock(&plxx_lock);

  if (plcmxx_get_in(data,PLCM10_LED_POL) == 0) 
    tmp = 0;

  mutex_unlock(&plxx_lock);

  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcm10_pol_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  unsigned char tmp = 1;

  if(buf[0] == '0')
    tmp = 0;

  mutex_lock(&plxx_lock);
  
  plcmxx_set_out(data, PLCM10_LED_POL, tmp); 

  mutex_unlock(&plxx_lock);

  return size;
}

static ssize_t plcmxx_xo1_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 0;
  
  mutex_lock(&plxx_lock);
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 : 
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : if (plcmxx_get_in(data,PLCM10_XO1)==0) tmp = 1; 
                             break;
    default : if (plcmxx_get_in(data,PLCM09_XO1)==0) tmp = 1; 
                             break;
  }
  mutex_unlock(&plxx_lock);

  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcmxx_xo1_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  unsigned char tmp = 0;

  mutex_lock(&plxx_lock);

  if(buf[0] == '0')
    tmp = 1;
  
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 : 
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : plcmxx_set_out(data, PLCM10_XO1, tmp); 
                             break;
    default : plcmxx_set_out(data, PLCM09_XO1, tmp);
                             break;
  }
  
  mutex_unlock(&plxx_lock);
  return size;
}

static ssize_t plcmxx_xo2_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 0;
  
  mutex_lock(&plxx_lock);
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 :
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : if (plcmxx_get_in(data,PLCM10_XO2)==0) tmp = 1; 
                             break;
    default : if (plcmxx_get_in(data,PLCM09_XO2)==0) tmp = 1; 
                             break;
  }
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcmxx_xo2_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  unsigned char tmp = 0;

  mutex_lock(&plxx_lock);

  if(buf[0] == '0')
    tmp = 1;
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 : 
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : plcmxx_set_out(data, PLCM10_XO2, tmp); 
                             break;
    default : plcmxx_set_out(data, PLCM09_XO2, tmp);
                             break;
                            
  }
  
  mutex_unlock(&plxx_lock);
  return size;
}

static ssize_t plcmxx_xi1_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 0;
  
  mutex_lock(&plxx_lock);  
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 :
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : if (plcmxx_get_in(data,PLCM10_XI1)>0) tmp = 1; 
                             break;
    default : if (plcmxx_get_in(data,PLCM09_XI1)>0) tmp = 1; 
                             break;
  }
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcmxx_xi2_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 0;
  
  mutex_lock(&plxx_lock);

  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 :
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : if (plcmxx_get_in(data,PLCM10_XI2)>0) tmp = 1; 
                             break;
    default : if (plcmxx_get_in(data,PLCM09_XI2)>0) tmp = 1; 
                             break;
  }

  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcmxx_ringing_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 0;
  
  mutex_lock(&plxx_lock);  
  if (plcmxx_get_in(data,PLCM09_RING)==0) tmp = 1; 
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcmxx_status_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  int tmp = 0;
  
  mutex_lock(&plxx_lock);  
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 : 
    case PLCMxx_VERSION_11 : 
    case PLCMxx_VERSION_12 : if (plcmxx_get_in(data,PLCM10_STATUS_IN)==0) tmp = 1; 
                             break;
  }
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",tmp);
}

static ssize_t plcm09_power_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);

  mutex_lock(&plxx_lock);

  if(buf[0] == '1')
  {
    printk("PLCM09 power ON\n");
    plcmxx_set_out(data, PLCM09_PWR_KEY, 1);
    msleep(200);
    plcmxx_set_out(data, PLCM09_PWR_KEY, 0);
  }
  else
  {
    printk("PLCM09 power OFF\n");
    plcmxx_set_out(data, PLCM09_PWR_DW, 1);
    msleep(200);
    plcmxx_set_out(data, PLCM09_PWR_DW, 0);
  }
  
  mutex_unlock(&plxx_lock);
  return size;
}

static ssize_t plcm10_power_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size, const char* devName)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  u8 on = 0;

  mutex_lock(&plxx_lock);

  on = plcmxx_get_in(data,PLCM10_STATUS_IN);
  //printk("%s power %s, status %d\n", devName, buf, on);

  if((buf[0] == '1') && (on>0))
  {
    printk("%s power ON\n",devName);
    plcmxx_set_out(data, PLCM10_PWR_KEY, 1);
    msleep(500);
    plcmxx_set_out(data, PLCM10_PWR_KEY, 0);
  }
  else
  if ((buf[0] == '0') && (on == 0))
  {
    printk("%s power OFF\n",devName);
    plcmxx_set_out(data, PLCM10_PWR_KEY, 1);
    msleep(650);
    plcmxx_set_out(data, PLCM10_PWR_KEY, 0);
  }
  
  mutex_unlock(&plxx_lock);
  return size;
}

static ssize_t plcmxx_power_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);
  switch (data->plcmversion)
  {
    case PLCMxx_VERSION_10 : 
      case PLCMxx_VERSION_11 : 
      case PLCMxx_VERSION_12 : return plcm10_power_set(dev,attr,buf,size,plcm10_get_devName(data->plcmversion));
    default : return plcm09_power_set(dev,attr,buf,size);
  }
}

static ssize_t plcm10_reset_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);

  mutex_lock(&plxx_lock);

  if (buf[0] == '1')
  {
    printk("%s RESET\n",plcm10_get_devName(data->plcmversion));
    plcmxx_set_out(data, PLCM10_RESET, 1);
    msleep(500);
    plcmxx_set_out(data, PLCM10_RESET, 0);
  } 
  mutex_unlock(&plxx_lock);
  return size;
}

static ssize_t plcm10_usb_reset_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);

  mutex_lock(&plxx_lock);

  if (buf[0] == '1')
  {
    printk("%s USB RESET\n",plcm10_get_devName(data->plcmversion));
    plcmxx_set_out(data, PLCM10_RST_HUB, 0);
    msleep(500);
    plcmxx_set_out(data, PLCM10_RST_HUB, 1);
  } 
  mutex_unlock(&plxx_lock);
  return size;
}

static ssize_t plcm10_wlan_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct plxx_data *data = dev_get_drvdata(dev);

  mutex_lock(&plxx_lock);

  if(buf[0] == '1')
  {
    printk("%s WLAN ON\n",plcm10_get_devName(data->plcmversion));
    plcmxx_set_out(data, PLCM10_WL_EN, 1);
    msleep(200);
    plcmxx_set_out(data, PLCM10_WL_RST, 1);
  }
  else
  if(buf[0] == '0')
  {
    printk("%s WLAN OFF\n",plcm10_get_devName(data->plcmversion));
    plcmxx_set_out(data, PLCM10_WL_RST, 0);
    msleep(200);
    plcmxx_set_out(data, PLCM10_WL_EN, 0);    
  }
  else
  if(buf[0] == '2')
  {
    printk("%s WLAN RESET\n",plcm10_get_devName(data->plcmversion));
    plcmxx_set_out(data, PLCM10_WL_RST, 0);
    msleep(20);
    plcmxx_set_out(data, PLCM10_WL_RST, 1);
  }
  
  mutex_unlock(&plxx_lock);
  return size;
}

/*
static DEVICE_ATTR(plcmxx_led1, S_IRUGO | S_IWUSR, plcmxx_led1_get, plcmxx_led1_set);
static DEVICE_ATTR(plcmxx_led2, S_IRUGO | S_IWUSR, plcmxx_led2_get, plcmxx_led2_set);
static DEVICE_ATTR(plcmxx_xo1, S_IRUGO | S_IWUSR, plcmxx_xo1_get, plcmxx_xo1_set);
static DEVICE_ATTR(plcmxx_xo2, S_IRUGO | S_IWUSR, plcmxx_xo2_get, plcmxx_xo2_set);
static DEVICE_ATTR(plcmxx_xi1, S_IRUGO, plcmxx_xi1_get, NULL);
static DEVICE_ATTR(plcmxx_xi2, S_IRUGO, plcmxx_xi2_get, NULL);
static DEVICE_ATTR(plcmxx_ringing, S_IRUGO, plcmxx_ringing_get, NULL);
static DEVICE_ATTR(plcmxx_power, S_IWUSR, NULL, plcmxx_power_set);
*/

static DEVICE_ATTR(plcm09_led1, S_IRUGO | S_IWUSR, plcmxx_led1_get, plcmxx_led1_set);
static DEVICE_ATTR(plcm09_led2, S_IRUGO | S_IWUSR, plcmxx_led2_get, plcmxx_led2_set);
static DEVICE_ATTR(plcm09_xo1, S_IRUGO | S_IWUSR, plcmxx_xo1_get, plcmxx_xo1_set);
static DEVICE_ATTR(plcm09_xo2, S_IRUGO | S_IWUSR, plcmxx_xo2_get, plcmxx_xo2_set);
static DEVICE_ATTR(plcm09_xi1, S_IRUGO, plcmxx_xi1_get, NULL);
static DEVICE_ATTR(plcm09_xi2, S_IRUGO, plcmxx_xi2_get, NULL);
static DEVICE_ATTR(plcm09_ringing, S_IRUGO, plcmxx_ringing_get, NULL);
static DEVICE_ATTR(plcm09_power, S_IWUSR, NULL, plcmxx_power_set);

static DEVICE_ATTR(plcmxx_led1, S_IRUGO | S_IWUSR, plcmxx_led1_get, plcmxx_led1_set);
static DEVICE_ATTR(plcmxx_led2, S_IRUGO | S_IWUSR, plcmxx_led2_get, plcmxx_led2_set);
static DEVICE_ATTR(plcmxx_pol, S_IRUGO | S_IWUSR, plcm10_pol_get, plcm10_pol_set);
static DEVICE_ATTR(plcmxx_xo1, S_IRUGO | S_IWUSR, plcmxx_xo1_get, plcmxx_xo1_set);
static DEVICE_ATTR(plcmxx_xo2, S_IRUGO | S_IWUSR, plcmxx_xo2_get, plcmxx_xo2_set);
static DEVICE_ATTR(plcmxx_xi1, S_IRUGO, plcmxx_xi1_get, NULL);
static DEVICE_ATTR(plcmxx_xi2, S_IRUGO, plcmxx_xi2_get, NULL);
static DEVICE_ATTR(plcmxx_status, S_IRUGO, plcmxx_status_get, NULL);
static DEVICE_ATTR(plcmxx_power, S_IWUSR, NULL, plcmxx_power_set);
static DEVICE_ATTR(plcmxx_wlan, S_IWUSR, NULL, plcm10_wlan_set);
static DEVICE_ATTR(plcmxx_reset, S_IWUSR, NULL, plcm10_reset_set);
static DEVICE_ATTR(plcmxx_usb_reset, S_IWUSR, NULL, plcm10_usb_reset_set);

static struct attribute *plcm09_sysfs_attributes[] = {
  &dev_attr_plcm09_led1.attr,
  &dev_attr_plcm09_led2.attr,
  &dev_attr_plcm09_xo1.attr,
  &dev_attr_plcm09_xo2.attr,
  &dev_attr_plcm09_xi1.attr,
  &dev_attr_plcm09_xi2.attr,
  &dev_attr_plcm09_ringing.attr,
  &dev_attr_plcm09_power.attr,
  NULL
};

static struct attribute *plcm10_sysfs_attributes[] = {
  &dev_attr_plcmxx_led1.attr,
  &dev_attr_plcmxx_led2.attr,
  &dev_attr_plcmxx_pol.attr,
  &dev_attr_plcmxx_xo1.attr,
  &dev_attr_plcmxx_xo2.attr,
  &dev_attr_plcmxx_xi1.attr,
  &dev_attr_plcmxx_xi2.attr,
  &dev_attr_plcmxx_status.attr,
  &dev_attr_plcmxx_power.attr,
  &dev_attr_plcmxx_wlan.attr,
  &dev_attr_plcmxx_reset.attr,
  &dev_attr_plcmxx_usb_reset.attr,
  NULL
};

static struct attribute *plcm11_sysfs_attributes[] = {
  &dev_attr_plcmxx_led1.attr,
  &dev_attr_plcmxx_led2.attr,
  &dev_attr_plcmxx_pol.attr,
  &dev_attr_plcmxx_xo1.attr,
  &dev_attr_plcmxx_xo2.attr,
  &dev_attr_plcmxx_xi1.attr,
  &dev_attr_plcmxx_xi2.attr,
  &dev_attr_plcmxx_status.attr,
  &dev_attr_plcmxx_power.attr,
  &dev_attr_plcmxx_reset.attr,
  &dev_attr_plcmxx_usb_reset.attr,
  NULL
};

static struct attribute *plcm12_sysfs_attributes[] = {
  &dev_attr_plcmxx_led1.attr,
  &dev_attr_plcmxx_led2.attr,
  &dev_attr_plcmxx_pol.attr,
  &dev_attr_plcmxx_xo1.attr,
  &dev_attr_plcmxx_xo2.attr,
  &dev_attr_plcmxx_xi1.attr,
  &dev_attr_plcmxx_xi2.attr,
  &dev_attr_plcmxx_status.attr,
  &dev_attr_plcmxx_power.attr,
  &dev_attr_plcmxx_wlan.attr,
  &dev_attr_plcmxx_reset.attr,
  &dev_attr_plcmxx_usb_reset.attr,
  NULL
};

static const struct attribute_group plcm09_attr_group = {
  .attrs = plcm09_sysfs_attributes,    //sysfs single entries
};

static const struct attribute_group plcm10_attr_group = {
  .attrs = plcm10_sysfs_attributes,    //sysfs single entries
};

static const struct attribute_group plcm11_attr_group = {
  .attrs = plcm11_sysfs_attributes,    //sysfs single entries
};

static const struct attribute_group plcm12_attr_group = {
  .attrs = plcm12_sysfs_attributes,    //sysfs single entries
};


/* -------------------------------------------------------------------------------------------------------------- *
 * 
 * END of PLCM09 related stuff
 * 
 * -------------------------------------------------------------------------------------------------------------- */

static int UpdatePluginData(struct plxx_data *data);

/* External function to sen commands to the plxx_manager driver from other drivers.
 * Commands are actually implemented only for setting FullDuplex or HalfDuplex mode
 * of the RS422/485 plugin modules
 */
int plxx_manager_sendcmd(struct platform_device *pdev, unsigned int cmd)
{
    struct plxx_data *data = dev_get_drvdata(&pdev->dev);

    if(data == NULL)
      return -EPROBE_DEFER;
    
    /* only supporting the 2 following commands */
    if ((cmd != RS422_485_IF_SETFD) &&  (cmd != RS422_485_IF_SETHD))
      return -ENOTSUPP;
    
    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
      UpdatePluginData(data);
      data->f_updated = true;
    }
    
    gpio_set_value_cansleep(data->sel_gpio, 1);                      //Select the plugin I2C bus
    msleep(1);
    
    // If we have a RS485/422 plugin module (bit #3 in func. area) perform the command
    if(data->eeprom[SEE_FUNCT_AREA_OFF] & (0x01 << RS422_485_IF_FLAG))
    {
      u8 tmp;
      tmp=0x0e;
      nvmem_device_write(data->ioexp_device, 3, sizeof(u8), (void*)&tmp);
      msleep(1);
      if(cmd == RS422_485_IF_SETFD)
	      tmp = 0x00;
      else
        tmp = 0x01;
      
      printk("plxx_manager_sendcmd cmd=0x%x\n",cmd);
      nvmem_device_write(data->ioexp_device, 1, sizeof(u8), (void*)&tmp);
    }
    
    gpio_set_value_cansleep(data->sel_gpio, 0);                      //Deselect the plugin I2C bus
    msleep(1);
    mutex_unlock(&plxx_lock);
    return 0;
}
EXPORT_SYMBOL(plxx_manager_sendcmd);

/*
 * static helper function for parsing the DTB tree
 */
#ifdef CONFIG_OF
static int plxx_parse_dt(struct device *dev, struct plxx_data *data)
{
  struct device_node* node = dev->of_node;
  int                 ret;
  struct nvmem_device*  nvmem;

  /* Parse the DT to find the I2C SEEPROM bindings*/
  nvmem = nvmem_device_get(dev, "eeprom");
  if (IS_ERR(nvmem))
  {
    dev_err(dev, "nvmem_device_get:eeprom err=%ld\n", PTR_ERR(nvmem));
    return PTR_ERR(nvmem);
  }
  data->seeprom_device = nvmem;

  /* Parse the DT to find the I2C SEEPROM bindings*/
  nvmem = nvmem_device_get(dev, "ioexp");
  if (IS_ERR(nvmem))
  {
    dev_err(dev, "nvmem_device_get:ioexp err=%ld\n", PTR_ERR(nvmem));
    return PTR_ERR(nvmem);
  }
  data->ioexp_device = nvmem;

  /* Get the plugin index */
  ret = of_property_read_u32(node, "index", &data->index);
  if (ret)
    return ret;

  /* Get the sel_gpio */
  data->sel_gpio = of_get_named_gpio(node, "sel-gpio", 0);
  if (gpio_is_valid(data->sel_gpio))
  {
    ret = gpio_request(data->sel_gpio, "plxx_sel-gpio");
    if (ret < 0)
      return -EPROBE_DEFER;
    ret = gpio_direction_output(data->sel_gpio,0);
    if (ret < 0)
      return ret;

    gpio_set_value_cansleep(data->sel_gpio, 0);
  }
  else
    return -EPROBE_DEFER;
  return 0;
}
#else
static int hrs_parse_dt(struct device *dev, struct hrs_data *data)
{
  return -ENODEV;
}
#endif

/*
 * sysfs interface 
 */
static ssize_t show_installed(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);

  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  tmp = data->installed;
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_hwcode(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  tmp = data->eeprom[SEE_CODE_OFF];
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_hwsubcode(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  tmp = data->eeprom[SEE_SUBCODE_OFF];
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_fpgacode(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  tmp = data->eeprom[SEE_XILCODE_OFF];
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_fpgasubcode(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  tmp = data->eeprom[SEE_XILSUBCODE_OFF];
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp[SEE_MODULENAMELEN+1];
  struct plxx_data *data = dev_get_drvdata(dev);
  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  memcpy(tmp, &data->eeprom[SEE_NAME_OFF], SEE_MODULENAMELEN);
  tmp[SEE_MODULENAMELEN]=0;
  mutex_unlock(&plxx_lock);
  return sprintf(buf, "%s\n",tmp);
}

void AssignPlcmVersion(struct plxx_data* data)
{
  int hwcode = 0;
  int funcarea = 0;

  data->plcmversion = PLCMxx_VERSION_INVALID;

  if (data==NULL)
    return;

  hwcode = data->eeprom[SEE_CODE_OFF];
  funcarea = data->eeprom[SEE_FUNCT_AREA_OFF] + (data->eeprom[SEE_FUNCT_AREA_OFF+1] << 8);

  if (hwcode == 13)
    data->plcmversion = PLCMxx_VERSION_09;
  else if (hwcode == 14)
  {
    if ( (funcarea & (0x01 << FFA_PLCM11)) && (funcarea & (0x01 << FFA_PLCM12)) )
      data->plcmversion =  PLCMxx_VERSION_10;
    else if (funcarea & (0x01 << FFA_PLCM11))
      data->plcmversion =  PLCMxx_VERSION_11;
    else if (funcarea & (0x01 << FFA_PLCM12))
      data->plcmversion =  PLCMxx_VERSION_12;
    else // bits are zero
    {
      printk("plxx invalid version: hwcode %d func %d\n",hwcode,funcarea);
      data->plcmversion =  PLCMxx_VERSION_INVALID;
    }
  }
  else
    data->plcmversion =  PLCMxx_VERSION_09;
}

/* The function bit area of the plugin is seen as a RO binary file, where the i-th byte represents the binary status (0|1) of the i-th bit of the 
 * function bit area
 */
static ssize_t func_bit_area_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
  struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));
  int bitIndex;
  int i;

  if(count == 0)
    return count;
  
  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  
  if(count > (SEE_FUNCAREA_NBITS - off))
    count = SEE_FUNCAREA_NBITS - off;
  
  for(i=0; i < count; i++)
  {
    bitIndex = off + i;
    buf[i] = (data->eeprom[SEE_FUNCT_AREA_OFF + bitIndex/8] & (1 << (bitIndex & 7)))? 1 : 0;
  }

  mutex_unlock(&plxx_lock);
  return count;
}

/* Show the eeprom contents of the plugin as a raw file 
 */
static ssize_t eeprom_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
  struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));
  int i;

  if(count == 0)
    return count;
  
  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  
  if(count > (FULLEEPROMSIZE - off))
    count = FULLEEPROMSIZE - off;
  
  for(i=0; i < count; i++)
  {
    buf[i] = data->eeprom[i+off];
  }

  mutex_unlock(&plxx_lock);
  return count;
}

/* Write the eeprom contents of the plugin as a raw file 
 */
static ssize_t eeprom_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
  struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));
  int i;

  if(count == 0)
    return count;
  
  if(off >= FULLEEPROMSIZE)
    return 0;
  
  mutex_lock(&plxx_lock);
  if(!data->f_updated)
  {
    UpdatePluginData(data);
    data->f_updated = true;
  }
  
  if(count > (FULLEEPROMSIZE - off))
    count = FULLEEPROMSIZE - off;
 
  gpio_set_value_cansleep(data->sel_gpio, 1);                      //Select the plugin I2C bus
  msleep(1);
  
  i = nvmem_device_write(data->seeprom_device, off, count, (void*)buf); 
  usleep_range(1, 2);
  
  nvmem_device_read(data->seeprom_device, off, count, (void*)&data->eeprom[off]); 
  usleep_range(1, 2);
  
  gpio_set_value_cansleep(data->sel_gpio, 0);                      //Select the plugin I2C bus
  mutex_unlock(&plxx_lock);
  return i;
}

/* Show the registers of the i2cexpander of the plugin as a raw file 
 */
static ssize_t i2cexpander_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));

  if(count == 0)
    return count;
  
  mutex_lock(&plxx_lock);
  
  if(count > (FULLEEPROMSIZE - off))
    count = FULLEEPROMSIZE - off;
  
  gpio_set_value_cansleep(data->sel_gpio, 1);                      //Select the plugin I2C bus
  usleep_range(1, 2);

  nvmem_device_read(data->ioexp_device, off, count, (void*)buf);
  usleep_range(1, 2);

  gpio_set_value_cansleep(data->sel_gpio, 0);                      //Select the plugin I2C bus
  mutex_unlock(&plxx_lock);
  return count;
}

/* Write the registers of the i2cexpander of the plugin as a raw file 
 */
static ssize_t i2cexpander_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));
    int i;

  if(count == 0)
    return count;
  
  if(off >= FULLEEPROMSIZE)
    return 0;
  
  mutex_lock(&plxx_lock);
   
  if(count > (FULLEEPROMSIZE - off))
    count = FULLEEPROMSIZE - off;
 
  gpio_set_value_cansleep(data->sel_gpio, 1);                      //Select the plugin I2C bus
  usleep_range(1, 2);

  i = nvmem_device_write(data->ioexp_device, off, count, (void*)buf);
  usleep_range(1, 2);

  gpio_set_value_cansleep(data->sel_gpio, 0);                      //Select the plugin I2C bus
  mutex_unlock(&plxx_lock);
  return i;
}

static DEVICE_ATTR(installed, S_IRUGO, show_installed, NULL);
static DEVICE_ATTR(hwcode, S_IRUGO, show_hwcode, NULL);
static DEVICE_ATTR(hwsubcode, S_IRUGO, show_hwsubcode, NULL);
static DEVICE_ATTR(fpgacode, S_IRUGO, show_fpgacode, NULL);
static DEVICE_ATTR(fpgasubcode, S_IRUGO, show_fpgasubcode, NULL);
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static BIN_ATTR_RO(func_bit_area, SEE_FUNCAREA_NBITS);
static BIN_ATTR_RW(eeprom, FULLEEPROMSIZE);
static BIN_ATTR_RW(i2cexpander, FULLEEPROMSIZE);

static struct attribute *plxx_sysfs_attributes[] = {
  &dev_attr_installed.attr,
  &dev_attr_name.attr,
  &dev_attr_hwcode.attr,
  &dev_attr_hwsubcode.attr,
  &dev_attr_fpgacode.attr,
  &dev_attr_fpgasubcode.attr,
  NULL
};

static struct bin_attribute *plxx_sysfs_bin_attrs[] = {
  &bin_attr_func_bit_area,
  &bin_attr_eeprom,
  &bin_attr_i2cexpander,  
  NULL,
};

static const struct attribute_group plxx_attr_group = {
  .attrs = plxx_sysfs_attributes,    //sysfs single entries
  .bin_attrs = plxx_sysfs_bin_attrs, //sysfs binary file for the function bit area
};

/*
 * Read and validate the whole plugin SEEPROM and update the plugin data structure accordingly
 */
static int UpdatePluginData(struct plxx_data *data)
{
  int ret = 0;
  int n;

  gpio_set_value_cansleep(data->sel_gpio, 1);                      //Select the plugin I2C bus
  msleep(1);
  n = nvmem_device_read(data->seeprom_device, 0, FULLEEPROMSIZE, (void*)data->eeprom);

  if(n < FULLEEPROMSIZE)
  {
    ret = -1;                                        //Plugin not found
    memset(data->eeprom, 0, FULLEEPROMSIZE);
    data->installed = 0;
  }
  else
    data->installed = 1;

  //If plugin found, now check if the seeprom contents are valid
  if(ret==0)                                              
  {
    u8 chksum = 1;
    int i;
    bool eeprom_isvalid = true;

    for(i = SEE_CHKSM_START; i < SEE_FACTORYSIZE; i++)
      chksum += data->eeprom[i];
    chksum -= 0xaa;
    
    if(chksum != data->eeprom[SEE_CHKSM_START - 1])       //Checksum check
      eeprom_isvalid = false;
    
    if(data->eeprom[0] != 0xaa)                           //Header check
      eeprom_isvalid = false;
			
    if(data->eeprom[1] != 0x55)                           //Header check
      eeprom_isvalid = false;
     
    if(data->eeprom[2] != 0x03)                           //Header check
      eeprom_isvalid = false;

    AssignPlcmVersion(data);

#ifndef CONFIG_SOC_IMX6Q
    if (data->plcmversion == PLCMxx_VERSION_09)
    {
       data->installed = 0;
       ret = -1;
    }
    else
#endif
    if(eeprom_isvalid == false)
    {                                                     //The plugin was detected bus has invalid SEEPROM contents ...
      memset(data->eeprom, 0, SEE_FACTORYSIZE);
    }
  }
  gpio_set_value_cansleep(data->sel_gpio, 0);                      //Deselect the plugin I2C bus
  return ret;
}

/*
 * Probe and remove functions
 */
static int plxx_probe(struct platform_device *pdev)
{
  int res = 0;
  int version;
  struct plxx_data *data;

  printk("plxx probe\n");
  data = kzalloc(sizeof(struct plxx_data), GFP_KERNEL);
  if (data == NULL)
  {
    dev_err(&pdev->dev, "Memory allocation failed\n");
    return -ENOMEM;
  }
  memset(data, 0, sizeof(struct plxx_data));
  dev_set_drvdata(&pdev->dev, data);

  res = plxx_parse_dt(&pdev->dev, data);
  if (res)
  {
    dev_err(&pdev->dev, "Could not find valid DT data.\n");
    goto plxx_error1;
  }

  data->f_updated = false;

  // Create sysfs entry
  res = sysfs_create_group(&pdev->dev.kobj, &plxx_attr_group);
  if (res)
  {
    dev_err(&pdev->dev, "plxx device create file failed\n");
    goto plxx_error1;
  }

  //PLCMxx detection and init
  data->plcmversion = PLCMxx_VERSION_INVALID;
  version = PLCMxx_VERSION_INVALID;
  if(plcm09_init(data) >= 0)
    version = PLCMxx_VERSION_09;
  else if (plcm10_init(data)>=0)
    version = PLCMxx_VERSION_10;

  if (version!=PLCMxx_VERSION_INVALID)
  {
    //PLCMxx detected, add the corresponding sysfs group
    printk("plxx plugin module detected; index=%d; version=%d\n",data->index,version);
    switch (version)
    {
      case PLCMxx_VERSION_10 : res = sysfs_create_group(&pdev->dev.kobj, &plcm10_attr_group);
                               break;
      case PLCMxx_VERSION_11 : res = sysfs_create_group(&pdev->dev.kobj, &plcm11_attr_group);
                               break;
      case PLCMxx_VERSION_12 : res = sysfs_create_group(&pdev->dev.kobj, &plcm12_attr_group);
                               break;
      default : res = sysfs_create_group(&pdev->dev.kobj, &plcm09_attr_group);
                break;
    }
    if (res)
    {
      dev_err(&pdev->dev, "plxx device create file failed\n");
      goto plxx_error1;
    }
  } else dev_err(&pdev->dev, "plxx device invalid version\n");

  return res;
plxx_error1:
  kfree(data);
  return res;
}

static int plxx_remove(struct platform_device *pdev)
{
  struct plxx_data *data = dev_get_drvdata(&pdev->dev);

  sysfs_remove_group(&pdev->dev.kobj, &plxx_attr_group);
  nvmem_device_put(data->seeprom_device);
  nvmem_device_put(data->ioexp_device);
  kfree(data);
  return 0;
}

/*
 * Driver instantiation
 */
#ifdef CONFIG_OF
static struct of_device_id plxx_of_match[] = {
  { .compatible = "exor,plxx_manager" },
  { }
};

MODULE_DEVICE_TABLE(of, plxx_of_match);
#endif

static struct platform_driver plxx_driver = {
  .driver		= {
    .name		= "plxx_manager",
    .owner		= THIS_MODULE,
    .of_match_table	= of_match_ptr(plxx_of_match),
  },
  .probe		= plxx_probe,
  .remove		= plxx_remove,
};

module_platform_driver(plxx_driver);

MODULE_DESCRIPTION("plxx_manager");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:plxx_manager");
