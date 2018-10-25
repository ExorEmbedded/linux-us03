/*
 *  dviplugin - driver for the Exor DVI plugin module, based on HDAxx boards
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

#define DEVICE_NAME	"dvipluginhdaxx"

extern int hw_dispid;  //This is an exported variable holding the display id value, if passed from cmdline
extern int dvi_dispid; //This is the external variable indicating if the DVI plugin module is installed and the selected cfg.

/*******************************************************************************************************************
 * sysfs interface
 *******************************************************************************************************************/
static ssize_t vidin_pos_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  u32 x, y;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  x = data->vidin.xoffs;
  y = data->vidin.yoffs;
  
  return sprintf(buf, "%d,%d\n",x,y);
}

static ssize_t vidin_pos_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  u32 x=0, y=0;
  int ret;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  ret=sscanf(buf,"%d,%d",&x,&y);
  if (ret != 2)
    return -EINVAL;
  
  if((x > MAXRESVAL) || (y > MAXRESVAL))
    return -EINVAL;
  
  data->vidin.xoffs = x;
  data->vidin.yoffs = y;
  VIDIN_Configure(data);
  
  return size;
}

static ssize_t vidin_size_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  u32 x, y;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  x = data->vidin.width;
  y = data->vidin.height;
  
  return sprintf(buf, "%d,%d\n",x,y);
}

static ssize_t vidin_size_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  u32 x=0, y=0;
  int ret;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  ret=sscanf(buf,"%d,%d",&x,&y);
  if (ret != 2)
    return -EINVAL;
  
  if((x > MAXRESVAL) || (y > MAXRESVAL))
    return -EINVAL;
  
  data->vidin.width = x;
  data->vidin.height = y;
  VIDIN_Configure(data);
  
  return size;
}

static ssize_t vidin_enable_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  u32 x;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  x = data->vidin.enabled;
  if(x!=0)
    x=1;
  
  return sprintf(buf, "%d\n",x);
}

static ssize_t vidin_enable_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  u32 x=0;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  if(buf[0]=='1')
    x=1;
  
  data->vidin.enabled = x;
  VIDIN_Configure(data);
  
  return size;
}

static ssize_t dviout_sel_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  u32 x;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  x = data->seldviout;
  
  return sprintf(buf, "%d\n",x);
}


static ssize_t dviout_sel_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  u32 x=0;
  int ret;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  ret=sscanf(buf,"%d",&x);
  if (ret != 1)
    return -EINVAL;
  
  if(x>2)
    return -EINVAL;
  
  data->seldviout = x;
  DVI_Configureswitch(data);
  
  return size;
}

static ssize_t mouse_event_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  u32 x=0, y=0, butt=0;
  int ret;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  ret=sscanf(buf,"%d,%d,%d",&x,&y,&butt);
  if (ret != 3)
    return -EINVAL;
  
  if((x > MAXCOORDVALUE) || (y > MAXCOORDVALUE))
    return -EINVAL;
  
  if(butt > 3)
    return -EINVAL;
  
  mutex_lock(&data->lock);
  data->mouseevent_x = x;
  data->mouseevent_y = y;
  data->mouseevent_lbutt = 0;
  data->mouseevent_rbutt = 0;
  
  if(butt & 0x01)
    data->mouseevent_lbutt = 1;
    
  if(butt & 0x02)
    data->mouseevent_rbutt = 1;

  data->mouseevent_flag = 1;
  mutex_unlock(&data->lock);
  
  wake_up_interruptible(&data->send_wq);
  
  return size;
}

static ssize_t dviin_stat_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  u32 x;
  struct dviplugin_data *data = dev_get_drvdata(dev);
  
  x = data->DVIinStatus;
  
  if(x==0)
    return 0;
  
  return sprintf(buf, "%d,%d\n",(x & 0xffff), ((x>>16) & 0xffff));
}

static ssize_t dvi_modes_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  int n=0;
  int i=0;
  
  while((dviconfig[i].dispid != NODISPLAY))
  {
    n+=sprintf(&buf[n], "%d:%dx%d\n",(int)dviconfig[i].dispid, (int)dviconfig[i].rezx, (int)dviconfig[i].rezy);
    i++;
  }
  return n;
}

static ssize_t dvi_mode_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  int n=0;
  int i=0;
  
  while((dviconfig[i].dispid != NODISPLAY))
  {
    if(dviconfig[i].dispid == dvi_dispid)
      n+=sprintf(&buf[n], "%d:%dx%d\n",(int)dviconfig[i].dispid, (int)dviconfig[i].rezx, (int)dviconfig[i].rezy);
    i++;
  }
  return n;
}

static DEVICE_ATTR(vidin_pos, S_IRUGO | S_IWUSR, vidin_pos_get, vidin_pos_set);
static DEVICE_ATTR(vidin_size, S_IRUGO | S_IWUSR, vidin_size_get, vidin_size_set);
static DEVICE_ATTR(vidin_enable, S_IRUGO | S_IWUSR, vidin_enable_get, vidin_enable_set);
static DEVICE_ATTR(dviout_sel, S_IRUGO | S_IWUSR, dviout_sel_get, dviout_sel_set);
static DEVICE_ATTR(mouse_event, S_IWUSR, NULL, mouse_event_set);
static DEVICE_ATTR(dviin_stat, S_IRUGO, dviin_stat_get, NULL);
static DEVICE_ATTR(dvi_modes, S_IRUGO, dvi_modes_get, NULL);
static DEVICE_ATTR(dvi_mode, S_IRUGO, dvi_mode_get, NULL);

static struct attribute *dvi_sysfs_attributes[] = {
  &dev_attr_vidin_pos.attr,
  &dev_attr_vidin_size.attr,
  &dev_attr_vidin_enable.attr,
  &dev_attr_dviout_sel.attr,
  &dev_attr_mouse_event.attr,
  &dev_attr_dviin_stat.attr,
  &dev_attr_dvi_modes.attr,
  &dev_attr_dvi_mode.attr,
  NULL
};

static const struct attribute_group dvi_attr_group = {
  .attrs = dvi_sysfs_attributes,    //sysfs single entries
};


/*
 * Helper function to get the local display parameters, based on the given dispid
 */
static int GetDisplayParams(int dispid, struct t_DisplayParams* params)
{
  int i=0;
  
  // Scan the display array to search for the required dispid
  if(dispid == NODISPLAY)
    return -1;
  
  while((displayconfig[i].dispid != NODISPLAY) && (displayconfig[i].dispid != dispid))
    i++;
  
  if(displayconfig[i].dispid == NODISPLAY)
    return -1;
  
  memcpy(params, &displayconfig[i], sizeof(struct t_DisplayParams));
  return 0;
}

/*
 * Helper function to get the DVI timing parameters, based on the given dispid/DVI id
 */
static int GetDVIParams(int dispid, struct t_DisplayParams* params)
{
  int i=0;
  
  // Scan the display array to search for the required dispid
  if(dispid == NODISPLAY)
    return -1;
  
  while((dviconfig[i].dispid != NODISPLAY) && (dviconfig[i].dispid != dispid))
    i++;
  
  if(dviconfig[i].dispid == NODISPLAY)
    return -1;

  memcpy(params, &dviconfig[i], sizeof(struct t_DisplayParams));
  return 0;
}

/*
 * Returns if DUAL LVDS mode has to be used used for the ADP out, based on the resulting frame refresh frequency
 */
bool adp_get_splitmode(int dispid)
{
  int i=0;
  int frefresh;
  
  // Scan the display array to search for the required dispid
  if(dispid == NODISPLAY)
    return false;
  
  while((displayconfig[i].dispid != NODISPLAY) && (displayconfig[i].dispid != dispid))
    i++;
  
  if(displayconfig[i].dispid == NODISPLAY)
    return false;
  
  frefresh = (1000 * displayconfig[i].pclk_freq)/((displayconfig[i].rezx + displayconfig[i].hs_bp + displayconfig[i].hs_fp + displayconfig[i].hs_w) * 
                                                  (displayconfig[i].rezy + displayconfig[i].vs_bp + displayconfig[i].vs_fp + displayconfig[i].vs_w) +1);  
  
  printk("adp_get_splitmode frefresh=%d\n",frefresh);
  
  //A refresh rate < 40Hz indicates dual LVDS interface
  if(frefresh < 40)
    return true;
  
  return false;
}

/*
 * Driver probe function
 */
static int dviplugin_probe(struct spi_device *spi)
{
	struct dviplugin_data *pdata;
	struct i2c_adapter    *adap;	
	u32                   i2cnr;
	struct device_node    *np = spi->dev.of_node;
	
	printk("%s \n", __func__);
	
	// Initial preliminary checks
	if(dvi_dispid == NODISPLAY)
	{
	  printk("%s : No DVI plugin module installed, exiting...\n", __func__);
	  return -ENODEV;
	}
	
	if(!np)
	{
	  printk("%s : No module description in DTB, exiting...\n", __func__);
	  return -ENODEV;
	}
	
	// Get properties from dtb descriptor
	if (of_property_read_u32(np, "i2c-nr", &i2cnr) != 0)
	{
	  printk("%s : Missing i2c-nr property, exiting...\n", __func__);
	  return -ENODEV;
	}
	printk("%s i2cnr=%d\n", __func__, i2cnr);
	
	/* Get The I2C adapter */
	adap = i2c_get_adapter(i2cnr);
	if (!adap)
	{
	  printk("%s : Unable to get the I2C adapter, deferring...\n", __func__);
	  return -EPROBE_DEFER;
	}

	//Init the pca9557 I2C expander and connect the QSPI Flash to SPI bus for allowing FPGA updates
	if(pca9557_init(adap))
	{
	  printk("%s : Failed to init the PCA9557 chip, exiting...\n", __func__);
	  return -ENODEV;
	}
	pca9557_clr_out(adap, 0);
	//Connect the I2C memory to the Cypress internal I2C bus
	pca9557_clr_out(adap, 2); //Connect the I2C memory to Cypress I2C bus
	
	/* Configure the SPI bus */
	spi->mode = (SPI_MODE_0);
	spi->bits_per_word = 8;
	spi_setup(spi);
	
	pdata = devm_kzalloc(&spi->dev, sizeof(struct dviplugin_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	
	mutex_init(&pdata->lock);
	pdata->spi = spi;
	spi_set_drvdata(spi, pdata);
	pdata->i2cadap = adap;
	pdata->adp_dualLVDS = 0;
	
	// Get the video timings for the local display (ADP) if used
	if(hw_dispid != NODISPLAY)
	{
	  if(GetDisplayParams(hw_dispid, &(pdata->ADPparams)))
	  {
	    printk("%s : Failed to get the local display timings, exiting...\n", __func__);
	    return -ENODEV;
	  }
	  
	  if(adp_get_splitmode(hw_dispid))
	    pdata->adp_dualLVDS = 0x02;
	  
	  printk("%s: Local display timings: %d x %d @ %d Khz\n",__func__, (int)pdata->ADPparams.rezx, (int)pdata->ADPparams.rezy, (int)pdata->ADPparams.pclk_freq);
	}
	else
	  printk("%s : Local display not used\n", __func__);
	
	// Get the video timings for the DVI chain
	if(GetDVIParams(dvi_dispid, &(pdata->DVIparams)))
	{
	  printk("%s : Failed to get the DVI video timings, exiting...\n", __func__);
	  return -ENODEV;
	}
	printk("%s: DVI timings: %d x %d @ %d Khz\n",__func__, (int)pdata->DVIparams.rezx, (int)pdata->DVIparams.rezy, (int)pdata->DVIparams.pclk_freq);

	//Configure the TFP410 DVI out chip
	if(TFP410_Configure(adap))
	{
	  printk("%s : Failed to init the TFP410 chip, exiting...\n", __func__);
	  return -ENODEV;
	}

	//Try to configure the adv7611 chip, if present
	if(!adv7611_CheckPresence(adap))
	{
  	  printk("%s : Found adv7611 chip\n", __func__);
	  adv7611_Configure(adap);
	}

	// If local display is present, route OS stream on local display
	// If no ADP present, route OS stream on DVI output (ext. monitor)
	if(pdata->ADPparams.pclk_freq != 0) // if ADP present ...
	  pdata->seldviout = SHOW_DVI_INPUT;
	else
	  pdata->seldviout = SHOW_WCE_INPUT;

	//Initialize the FPGA core
	if(InitFPGACores(pdata))
	{
	  printk("%s : Failed to init the FPGA core, exiting...\n", __func__);
	  return -ENODEV;
	}
	
	//Create the sysfs entries
	if(sysfs_create_group(&spi->dev.kobj, &dvi_attr_group))
	{
	  printk("%s : Failed to create sysfs entries...\n", __func__);
	}
	
	//Start the monitor thread
	init_completion(&pdata->auto_update_stop);
	pdata->auto_update = kthread_run(monitor_thread, &spi->dev, "%s", dev_name(&spi->dev));
	
	//Start the emulated mouse thread
	init_waitqueue_head(&pdata->send_wq);
	init_completion(&pdata->mouse_update_stop);
	pdata->mouse_update = kthread_run(mouse_thread, &spi->dev, "%s", dev_name(&spi->dev));
	
	printk("%s : SUCCESS\n", __func__);
	return 0;
}

static int dviplugin_remove(struct spi_device *spi)
{
	struct dviplugin_data *pdata = spi_get_drvdata(spi);
	
	//Stop the monitor thread
	kthread_stop(pdata->auto_update);
	wait_for_completion(&pdata->auto_update_stop);

	//Stop the emulated mouse thread
	kthread_stop(pdata->mouse_update);
	wait_for_completion(&pdata->mouse_update_stop);
	
	i2c_put_adapter(pdata->i2cadap);

	return 0;
}

/*-------------------------------------------------------------*/
static const struct of_device_id dviplugin_of_match[] = {
	{ .compatible = "exor,dvipluginhdaxx", },
	{ }
};
MODULE_DEVICE_TABLE(of, dviplugin_of_match);

static struct spi_driver dviplugin_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dviplugin_of_match,
		
	},

	.probe = dviplugin_probe,
	.remove = dviplugin_remove,
};

module_spi_driver(dviplugin_driver);

MODULE_AUTHOR("Giovanni Pavoni , Exor Int.");
MODULE_DESCRIPTION("Exor DVI plugin module driver");
MODULE_LICENSE("GPL");
