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
#include <linux/list.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <dt-bindings/gpio/gpio.h>

#include <linux/xethm_vs.h>

#define SUCCESS   0

#define XETHM_VS_TX_QUEUE_SIZE  100

/** Defines the debug level of EoE processing.
 *
 * 0 = No debug messages.
 * 1 = Output warnings.
 * 2 = Output actions.
 * 3 = Output actions and frame data.
 */
#define XETHM_VS_DEBUG_LEVEL 1


MODULE_LICENSE("GPL");            ///< The license type -- this affects available functionality
MODULE_AUTHOR("Exor");            ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("XETHM Virtual Switch");  ///< The description -- see modinfo
MODULE_VERSION("0.1");            ///< A version number to inform users
 

struct xethm_vs_stats
{

};

struct xethm_vs_data 
{
    u32                      installed;               // Indicates if the plugin is physically installed
    struct xethm_vs_stats    stats;
};

struct xethm_vs_frame
{
    struct list_head queue; /**< list item */
    struct sk_buff *skb; /**< socket buffer */
};

struct xethm_vs_eoe 
{
  unsigned int master_instance; /** EtherCAT master istance number */
  unsigned int slave_pos; /**< pointer to the corresponding slave */

  int majorNumber;
  struct class* eoe_class;
  struct device* chrdev; /**< char device for reading/writing data */
  struct net_device *dev; /**< net_device for virtual ethernet device */
  struct net_device_stats stats; /**< device statistics */
  unsigned long rate_jiffies; /**< time of last rate output */
  unsigned int opened; /**< net_device is opened */

  struct sk_buff *rx_skb; /**< current rx socket buffer */
  uint32_t rx_counter; /**< octets received during last second */
  uint32_t rx_rate; /**< receive rate (bps) */
  off_t rx_skb_offset; /**< current write pointer in the socket buffer */
  size_t rx_skb_size; /**< size of the allocated socket buffer memory */
  unsigned int rx_idle; /**< Idle flag. */

  struct list_head tx_queue; /**< queue for frames to send */
  struct semaphore tx_queue_sem; /**< Semaphore for the send queue. */
  unsigned int tx_queue_size; /**< Transmit queue size. */  
  unsigned int tx_queue_active; /**< kernel netif queue started */
  unsigned int tx_queued_frames; /**< number of frames in the queue */  
  uint8_t tx_frame_number; /**< number of the transmitted frame */
  uint32_t tx_counter; /**< octets transmitted during last second */
  uint32_t tx_rate; /**< transmit rate (bps) */
  struct xethm_vs_frame *tx_frame; /**< current TX frame */
  unsigned int tx_idle; /**< Idle flag. */
  wait_queue_head_t tx_wait_queue;
  int wait_queue_flag;
};

struct xethm_eoe 
{
  struct net_device *dev; /**< net_device for virtual ethernet device */
  struct net_device_stats stats; /**< device statistics */
  struct sk_buff *rx_skb; /**< current rx socket buffer */
};

struct xethm_vs_instance
{
  struct xethm_vs_eoe* eoe;
  struct list_head list; /* kernel's list structure */
};

// net_device functions
static int xethm_vs_netdev_open(struct net_device *);
static int xethm_vs_netdev_stop(struct net_device *);
static int xethm_vs_netdev_tx(struct sk_buff *, struct net_device *);
static struct net_device_stats *xethm_vs_netdev_stats(struct net_device *);

static const struct net_device_ops xethm_vs_netdev_ops = {
    .ndo_open = xethm_vs_netdev_open,
    .ndo_stop = xethm_vs_netdev_stop,
    .ndo_start_xmit = xethm_vs_netdev_tx,
    .ndo_get_stats = xethm_vs_netdev_stats,
};

// character device functions
static int     xethm_vs_dev_open(struct inode *, struct file *);
static int     xethm_vs_dev_release(struct inode *, struct file *);
static ssize_t xethm_vs_dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t xethm_vs_dev_write(struct file *, const char *, size_t, loff_t *);

static struct file_operations xethm_vs_dev_fops =
{
   .open = xethm_vs_dev_open,
   .read = xethm_vs_dev_read,
   .write = xethm_vs_dev_write,
   .release = xethm_vs_dev_release,
};


static struct class* xethm_vs_eoe_class = NULL;
static struct xethm_vs_instance xethm_vs_instances;
static DEFINE_MUTEX(xethm_vs_lock);

/******************************************************************** 
 * Internal functions
 ********************************************************************/

/* 
 * Flush tx queue
 */
static void xethm_vs_flush(struct xethm_vs_eoe *eoe /**< EoE handler */)
{
  struct xethm_vs_frame *frame, *next;

  down(&eoe->tx_queue_sem);

  list_for_each_entry_safe(frame, next, &eoe->tx_queue, queue) {
      list_del(&frame->queue);
      dev_kfree_skb(frame->skb);
      kfree(frame);
  }

  eoe->tx_queued_frames = 0;

  up(&eoe->tx_queue_sem);
}

/* 
 * Initialize character device
 */
static int xethm_vs_init_chardev(struct xethm_vs_eoe *eoe)
{
  char deviceName[50];

  sprintf(deviceName, "eoe%da%d", eoe->master_instance, eoe->slave_pos);

  // Try to dynamically allocate a major number for the device -- more difficult but worth it
  eoe->majorNumber = register_chrdev(0, deviceName, &xethm_vs_dev_fops);
  if (eoe->majorNumber < 0)
  {
#if XETHM_VS_DEBUG_LEVEL >= 2    
    printk(KERN_ALERT "XETHM_VS: failed to register a major number\n");
#endif

    return eoe->majorNumber;
  }
  
#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: registered correctly with major number %d\n", eoe->majorNumber);
#endif

  // Register the device class
  if (xethm_vs_eoe_class == NULL)
  {
    xethm_vs_eoe_class = class_create(THIS_MODULE, "eoe");
    if (IS_ERR(eoe->eoe_class))
    {                // Check for error and clean up if there is
#if XETHM_VS_DEBUG_LEVEL >= 1    
      printk(KERN_ALERT "XETHM_VS: Failed to register device class\n");
#endif

      unregister_chrdev(eoe->majorNumber, deviceName);
      return PTR_ERR(eoe->eoe_class);          // Correct way to return an error on a pointer
    }

#if XETHM_VS_DEBUG_LEVEL >= 3    
    printk(KERN_INFO "XETHM_VS: device class registered correctly\n");
#endif
  }

  eoe->eoe_class = xethm_vs_eoe_class;

  // Register the device driver
  eoe->chrdev = device_create(eoe->eoe_class, NULL, MKDEV(eoe->majorNumber, 0), NULL, deviceName);
  if (IS_ERR(eoe->chrdev))
  {               
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ALERT "Failed to create the device\n");
#endif

    // Clean up if there is an error
    class_destroy(eoe->eoe_class);
    unregister_chrdev(eoe->majorNumber, deviceName);
    return PTR_ERR(eoe->chrdev);
   }

#if XETHM_VS_DEBUG_LEVEL >= 3    
   printk(KERN_INFO "XETHM_VS: device class created correctly\n"); // Made it! device was initialized
#endif
   return 0;
}

/*
 * Finalize character device
 */
static void xethm_vs_deinit_chardev(struct xethm_vs_eoe *eoe)
{
  char deviceName[50];

  sprintf(deviceName, "eoe%da%d", eoe->master_instance, eoe->slave_pos);

  device_destroy(eoe->eoe_class, MKDEV(eoe->majorNumber, 0));     // remove the device
  class_unregister(eoe->eoe_class);                          // unregister the device class
  class_destroy(eoe->eoe_class);                             // remove the device class
  unregister_chrdev(eoe->majorNumber, deviceName);             // unregister the major number

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Char device destroyed correctly\n");
#endif
}

/*
 * get network driver instance
 */
static struct xethm_vs_instance* xethm_vs_getinstance(int master_instance, int slave_pos)
{
  int found = 0;
  struct xethm_vs_instance* instance;
  struct xethm_vs_eoe* eoe;

  list_for_each_entry(instance, &xethm_vs_instances.list, list) 
  {
    //access the list node through node
    eoe = instance->eoe;
    if ((eoe->master_instance == master_instance) && (eoe->slave_pos == slave_pos))
    {
      found = 1;
      break;
    }
  }

  if (!found)
    return NULL;

  return instance;
}

/*
 * get EoE object instance
 */
static struct xethm_vs_eoe* xethm_vs_geteoe(int master_instance, int slave_pos)
{
  struct xethm_vs_instance* instance = xethm_vs_getinstance(master_instance, slave_pos);
  if (instance == NULL)
    return NULL;

  return instance->eoe;
}

/*
 * get EoE object instance for the given device filename
 */
static struct xethm_vs_eoe* xethm_vs_geteoebyfilename(char* filename)
{
  int len = strlen(filename);
  int instance = filename[len-3] - '0';
  int slave_pos = filename[len-1] - '0';

  return xethm_vs_geteoe(instance, slave_pos);
}

/********************************************************************
 * SYSFS functions
 ********************************************************************/

/*
 * open device
 */
static int xethm_vs_dev_open(struct inode *inodep, struct file *filep)
{
  char* filename = filep->f_path.dentry->d_iname;
  struct xethm_vs_eoe* eoe = xethm_vs_geteoebyfilename(filename);

  if (eoe == NULL)
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Device %s cannot be found\n", filename);
#endif

    return -ENODEV;
  }

  filep->private_data = eoe;

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Device has been opened\n");
#endif

  return 0;
}

/*
 * close device
 */
static int xethm_vs_dev_release(struct inode *inodep, struct file *filep)
{
  struct xethm_vs_eoe *eoe = (struct xethm_vs_eoe *)filep->private_data;
  if (eoe == NULL)
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Can't close device\n");
#endif

    return -ENODEV;
  }

  // release any pending read operation
  wake_up_interruptible(&eoe->tx_wait_queue);

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Device successfully closed\n");
#endif

  return 0;
}

/*
 * read
 */
static ssize_t xethm_vs_dev_read(struct file *filp, char *buff, size_t len, loff_t *off)
{
  ssize_t size;
  int count;
  int txEmpty;
  struct xethm_vs_eoe *eoe = (struct xethm_vs_eoe *) filp->private_data;

  if (eoe == NULL)
  {
#if XETHM_VS_DEBUG_LEVEL >= 0    
    printk(KERN_WARNING "XETHM_VS: EoE device not found\n");
#endif

    return -ENODEV;
  }

#if XETHM_VS_DEBUG_LEVEL >= 3    
  //printk(KERN_INFO "XETHM_VS: Slave %d - Getting packets to EoE...\n", eoe->slave_pos);
#endif

retry:
  down(&eoe->tx_queue_sem);
  txEmpty = ((!eoe->tx_queued_frames) || list_empty(&eoe->tx_queue));
  up(&eoe->tx_queue_sem);

  if (txEmpty)
  {
    eoe->tx_idle = 1;

#if XETHM_VS_DEBUG_LEVEL >= 3    
    printk(KERN_INFO "XETHM_VS: Slave %d - No packets available\n", eoe->slave_pos);
#endif

    if(filp->f_flags & O_NONBLOCK)
      return -EAGAIN;

    // blocking read: wait for data ready
    wait_event_interruptible(eoe->tx_wait_queue, eoe->wait_queue_flag != 0);
    if (eoe->wait_queue_flag == 0)
      return -EAGAIN;

    eoe->wait_queue_flag = 0;

    // blocking read: go and wait for data  
    goto retry;
  }

  // take the first frame out of the queue
  down(&eoe->tx_queue_sem);

  eoe->tx_frame = list_entry(eoe->tx_queue.next, struct xethm_vs_frame, queue);
  list_del(&eoe->tx_frame->queue);

  if (!eoe->tx_queue_active && eoe->tx_queued_frames == eoe->tx_queue_size / 2) 
  {
    netif_wake_queue(eoe->dev);
    eoe->tx_queue_active = 1;
  }

  eoe->tx_queued_frames--;
  up(&eoe->tx_queue_sem);

  eoe->tx_idle = 0;

  size = eoe->tx_frame->skb->len;
  if (size > len)
  {
#if XETHM_VS_DEBUG_LEVEL >= 3    
    printk(KERN_INFO "XETHM_VS: Slave %d - Bad frame size %d\n", eoe->slave_pos, size);    
#endif

    eoe->stats.tx_errors++;  
  
    dev_kfree_skb(eoe->tx_frame->skb);
    kfree(eoe->tx_frame);
    eoe->tx_frame = NULL;

    return -ENOMEM;
  }

  count = copy_to_user(buff, eoe->tx_frame->skb->data, size);
  if (count != 0)
  {
#if XETHM_VS_DEBUG_LEVEL >= 2    
    printk(KERN_WARNING "XETHM_VS: Slave %d - Copy error (%d / %d)\n", eoe->slave_pos, count, size);    
#endif

    eoe->stats.tx_errors++;
  }

  dev_kfree_skb(eoe->tx_frame->skb);
  kfree(eoe->tx_frame);
  eoe->tx_frame = NULL;

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Slave %d - %d bytes returned\n", eoe->slave_pos, size);
#endif

  return size;
}

/*
 * write
 */
static ssize_t xethm_vs_dev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
  struct xethm_vs_eoe *eoe = (struct xethm_vs_eoe *)filp->private_data;
  if (eoe == NULL)
  {
#if XETHM_VS_DEBUG_LEVEL >= 0    
    printk(KERN_WARNING "XETHM_VS: EoE device not found\n");
#endif

    return -ENODEV;
  }

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Slave %d - TXing packets...\n", eoe->slave_pos);
#endif

  if (eoe->rx_skb) 
  {
#if XETHM_VS_DEBUG_LEVEL >= 2    
    printk(KERN_WARNING "XETHM_VS: Slave %d - EoE RX freeing old socket buffer.\n", eoe->slave_pos);
#endif

    dev_kfree_skb(eoe->rx_skb);
  }

  // new socket buffer
  if (!(eoe->rx_skb = dev_alloc_skb(len))) 
  {
    if (printk_ratelimit())
#if XETHM_VS_DEBUG_LEVEL >= 2    
      printk(KERN_WARNING "XETHM_VS: Slave %d - EoE RX low on mem, frame dropped.\n", eoe->slave_pos);
#endif

    eoe->stats.rx_dropped++;
    return -ENOMEM;
  }

  eoe->rx_skb_offset = 0;
  eoe->rx_skb_size = len;

  // copy fragment into socket buffer
  memcpy(skb_put(eoe->rx_skb, len), buff, len);
  eoe->rx_skb_offset += len;

  // update statistics
  eoe->stats.rx_packets++;
  eoe->stats.rx_bytes += eoe->rx_skb->len;
  eoe->rx_counter += eoe->rx_skb->len;

#if XETHM_VS_DEBUG_LEVEL >= 2
  printk(KERN_INFO "XETHM_VS: Slave %d - EoE %s RX frame completed with %u octets.\n", eoe->slave_pos, eoe->dev->name, eoe->rx_skb->len);
#endif

  // pass socket buffer to network stack
  eoe->rx_skb->dev = eoe->dev;
  eoe->rx_skb->protocol = eth_type_trans(eoe->rx_skb, eoe->dev);
  eoe->rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
  if (netif_rx(eoe->rx_skb)) {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Slave %d - EoE RX netif_rx failed.\n", eoe->slave_pos);
#endif
  }

  //kfree packet
  eoe->rx_skb = NULL;

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Slave %d - Successfully TXed %d bytes\n", eoe->slave_pos, len);
#endif

  return SUCCESS;
}

/********************************************************************
 * NETDEV functions
 ********************************************************************/

/*
 * open
 */
static int xethm_vs_netdev_open(struct net_device *dev /**< EoE net_device */)
{
  struct xethm_vs_eoe *eoe = *((struct xethm_vs_eoe **) netdev_priv(dev));
  xethm_vs_flush(eoe);
  eoe->opened = 1;
  eoe->rx_idle = 0;
  eoe->tx_idle = 0;

  netif_start_queue(dev);
  eoe->tx_queue_active = 1;

#if XETHM_VS_DEBUG_LEVEL >= 3
  printk(KERN_DEBUG "XETHM_VS: Slave %d - dev %s opened\n", eoe->slave_pos, dev->name);
#endif
  
  return SUCCESS;
}

/*
 * stop
 */
static int xethm_vs_netdev_stop(struct net_device *dev /**< EoE net_device */)
{
  struct xethm_vs_eoe *eoe = *((struct xethm_vs_eoe **) netdev_priv(dev));
  netif_stop_queue(dev);
  eoe->rx_idle = 1;
  eoe->tx_idle = 1;
  eoe->tx_queue_active = 0;
  eoe->opened = 0;

  xethm_vs_flush(eoe);

#if XETHM_VS_DEBUG_LEVEL >= 3
  printk(KERN_DEBUG "XETHM_VS: Slave %d - dev %s stopped\n", eoe->slave_pos, dev->name);
#endif

  return SUCCESS;
}

/*
 * transmit packet
 */
static int xethm_vs_netdev_tx(struct sk_buff *skb, /**< transmit socket buffer */
                 struct net_device *dev /**< EoE net_device */)
{
  struct xethm_vs_eoe *eoe = *((struct xethm_vs_eoe **) netdev_priv(dev));
  struct xethm_vs_frame *frame;

#if 0
  if (skb->len > eoe->slave->configured_tx_mailbox_size - 10) 
  {
    EC_SLAVE_WARN(eoe->slave, "EoE TX frame (%u octets)"
            " exceeds MTU. dropping.\n", skb->len);
    dev_kfree_skb(skb);
    eoe->stats.tx_dropped++;
    return 0;
  }
#endif

  if (!(frame = (struct xethm_vs_frame *) kmalloc(sizeof(struct xethm_vs_frame), GFP_ATOMIC))) 
  {
    if (printk_ratelimit())
#if XETHM_VS_DEBUG_LEVEL >= 3    
      printk(KERN_INFO "XETHM_VS: Slave %d - EoE TX: low on mem. frame dropped.\n", eoe->slave_pos);
#endif

    return 1;
  }

  frame->skb = skb;

  down(&eoe->tx_queue_sem);
  list_add_tail(&frame->queue, &eoe->tx_queue);
  eoe->tx_queued_frames++;
  if (eoe->tx_queued_frames == eoe->tx_queue_size) 
  {
    netif_stop_queue(dev);
    eoe->tx_queue_active = 0;
  }
  up(&eoe->tx_queue_sem);

#if XETHM_VS_DEBUG_LEVEL >= 3
  printk(KERN_DEBUG "XETHM_VS: Slave %d - EoE %s TX queued frame with %u octets (%u frames queued).\n",
      eoe->slave_pos, eoe->dev->name, skb->len, eoe->tx_queued_frames);

  if (!eoe->tx_queue_active)
    printk(KERN_INFO "XETHM_VS: Slave %d - EoE TX queue is now full.\n", eoe->slave_pos);
#endif

  // wake up processes blocked on read
  eoe->wait_queue_flag = 1;
  wake_up_interruptible(&eoe->tx_wait_queue);

  return SUCCESS;
}

/*
 * stats
 */
static struct net_device_stats *xethm_vs_netdev_stats(
        struct net_device *dev /**< EoE net_device */)
{
  struct xethm_vs_eoe *eoe = *((struct xethm_vs_eoe **) netdev_priv(dev));
  return &eoe->stats;
}


/******************************************************************** 
 * IOCTL functions
 ********************************************************************/

/* 
 * add NICs
 */
static int xethm_vs_add_nic(struct nic_definition* nic)
{
  int i, ret = 0;
  char name[30];
  struct xethm_vs_eoe* eoe;
  struct xethm_vs_eoe** priv;
  struct xethm_vs_instance* instance;

  instance = xethm_vs_getinstance(nic->master_instance, nic->slave_pos);;
  if ((instance != NULL) && (instance->eoe != NULL))
    return SUCCESS;

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Adding NIC %d.%d...\n", nic->master_instance, nic->slave_pos); 
#endif

  eoe = (struct xethm_vs_eoe *) kmalloc(sizeof(struct xethm_vs_eoe), GFP_ATOMIC);
  if (eoe == NULL)
  {  
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Unable to allocate memory for NIC %d.%d\n", nic->master_instance, nic->slave_pos); 
#endif

    return -ENOMEM;
  }

  eoe->master_instance = nic->master_instance;
  eoe->slave_pos = nic->slave_pos;

  eoe->opened = 0;
  eoe->rx_skb = NULL;

  INIT_LIST_HEAD(&eoe->tx_queue);
  eoe->tx_frame = NULL;
  eoe->tx_queue_active = 0;
  eoe->tx_queue_size = XETHM_VS_TX_QUEUE_SIZE;
  eoe->tx_queued_frames = 0;

  sema_init(&eoe->tx_queue_sem, 1);
  eoe->tx_frame_number = 0xFF;
  memset(&eoe->stats, 0, sizeof(struct net_device_stats));

  eoe->rx_counter = 0;
  eoe->tx_counter = 0;
  eoe->rx_rate = 0;
  eoe->tx_rate = 0;
  eoe->rate_jiffies = 0;
  eoe->rx_idle = 1;
  eoe->tx_idle = 1;

  eoe->wait_queue_flag = 0;
  init_waitqueue_head(&eoe->tx_wait_queue);

  // device name eoe<MASTER>[as]<SLAVE>, because networking scripts don't
  // like hyphens etc. in interface names. */
  snprintf(name, sizeof(name)-1, "eoe%ua%u", nic->master_instance, nic->slave_pos);

  if (!(eoe->dev = alloc_netdev(sizeof(struct xethm_vs_eoe *), name, NET_NAME_UNKNOWN, ether_setup))) 
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Slave %d - Unable to allocate net_device %s for EoE handler!\n", nic->slave_pos, name);
#endif

    ret = -ENODEV;
    goto out_free;
  }

  sema_init(&eoe->tx_queue_sem, 1);

  // initialize net_device
  eoe->dev->netdev_ops = &xethm_vs_netdev_ops;
  
  for (i = 0; i < ETH_ALEN; i++)
    eoe->dev->dev_addr[i] = i | (i << 4);

  // initialize private data
  dev_set_drvdata(&eoe->dev->dev, eoe);

  priv = netdev_priv(eoe->dev);
  *priv = eoe;

  // Usually setting the MTU appropriately makes the upper layers
  // do the frame fragmenting. In some cases this doesn't work
  // so the MTU is left on the Ethernet standard value and fragmenting
  // is done "manually".
#if 0
  eoe->dev->mtu = slave->configured_rx_mailbox_size - ETH_HLEN - 10;
#endif

  // connect the net_device to the kernel
  ret = register_netdev(eoe->dev);
  if (ret) 
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Slave %d - Unable to register net_device: error %i\n", eoe->slave_pos, ret);
#endif

    goto out_free;
  }

  // make the last address octet unique
  eoe->dev->dev_addr[ETH_ALEN - 1] = (uint8_t) eoe->dev->ifindex;

  instance = (struct xethm_vs_instance*)kmalloc(sizeof(struct xethm_vs_instance), GFP_ATOMIC);
  if (instance == NULL) 
  {
    ret = -ENODEV;

#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Slave %d - Unable to allocate memory\n", eoe->slave_pos);
#endif

    goto out_free;
  }

  // register char device
  ret = xethm_vs_init_chardev(eoe);
  if (ret) 
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Slave %d - Unable to create sysfs: error %i\n", eoe->slave_pos, ret);
#endif

    goto out_free;
  }

  instance->eoe = eoe;
  list_add_tail(&instance->list, &xethm_vs_instances.list);

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Successfully added NIC %d.%d\n", nic->master_instance, nic->slave_pos); 
#endif

  return SUCCESS;

out_free:
  kfree(eoe);
  return ret;
}

/* 
 * remove NICs
 */
static int xethm_vs_remove_nic(struct nic_definition* nic)
{
  struct xethm_vs_eoe* eoe;
  struct xethm_vs_instance* instance = xethm_vs_getinstance(nic->master_instance, nic->slave_pos);;

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Removing NIC %d.%d\n", nic->master_instance, nic->slave_pos); 
#endif

  if((instance == NULL) || (instance->eoe == NULL))
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Unable to find NIC %d.%d\n", nic->master_instance, nic->slave_pos); 
#endif

    return -ENODEV;
  }

  eoe = instance->eoe;

  // wake up processes blocked on read
  wake_up_interruptible(&eoe->tx_wait_queue);

  // remove char device
  xethm_vs_deinit_chardev(eoe);
  unregister_netdev(eoe->dev); // possibly calls close callback

  // empty transmit queue
  xethm_vs_flush(eoe);

  if (eoe->tx_frame) 
  {
    dev_kfree_skb(eoe->tx_frame->skb);
    kfree(eoe->tx_frame);
  }

  if (eoe->rx_skb)
    dev_kfree_skb(eoe->rx_skb);

  free_netdev(eoe->dev);
  
  list_del(&instance->list);
  kfree(instance);

  kfree(eoe);

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Successfully removed NIC %d.%d\n", nic->master_instance, nic->slave_pos); 
#endif

  return SUCCESS;
}

/*
 * sysfs interface
 */
static ssize_t xethm_vs_show_installed(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct xethm_vs_data *data = dev_get_drvdata(dev);
  mutex_lock(&xethm_vs_lock);

  tmp = data->installed;

  mutex_unlock(&xethm_vs_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t xethm_vs_show_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 tmp;
  struct xethm_vs_data *data = dev_get_drvdata(dev);
  mutex_lock(&xethm_vs_lock);

  //TODO
  tmp = data->installed;

  mutex_unlock(&xethm_vs_lock);
  return sprintf(buf, "%d\n",(u32)tmp);
}

/********************************************************************
 * /dev/xethm_vs dev interface
 ********************************************************************/
static int xethm_vs_open(struct inode* node, struct file* f)
{
  return SUCCESS;
}

static int xethm_vs_close(struct inode* node, struct file* f)
{
  return SUCCESS;
}

static long xethm_vs_ioctl(struct file* f, unsigned int ioctl_num, unsigned long ioctl_param)
{
  struct nic_definition nic;

  /* Switch according to the ioctl called */
  switch (ioctl_num) 
  {
    case XETHM_VS_IOCTL_ADD_NIC:
      if (copy_from_user(&nic, (u8*)ioctl_param, sizeof(nic)))
        return -EBUSY;

      xethm_vs_add_nic(&nic);
      break;
    case XETHM_VS_IOCTL_REMOVE_NIC:
      if (copy_from_user(&nic, (u8*)ioctl_param, sizeof(nic)))
        return -EBUSY;

      xethm_vs_remove_nic(&nic);
      break;
  }

  return SUCCESS;
}

 
static DEVICE_ATTR(installed, S_IRUGO, xethm_vs_show_installed, NULL);
static DEVICE_ATTR(stats, S_IRUGO, xethm_vs_show_stats, NULL);

static struct attribute *xethm_vs_sysfs_attributes[] = {
  &dev_attr_installed.attr,
  &dev_attr_stats.attr,
  NULL
};

static struct bin_attribute *xethm_vs_sysfs_bin_attrs[] = {
  NULL,
};

static const struct attribute_group xethm_vs_attr_group = {
  .attrs = xethm_vs_sysfs_attributes,    //sysfs single entries
  .bin_attrs = xethm_vs_sysfs_bin_attrs, //sysfs binary file for the function bit area
};

static const struct file_operations xethm_vs_fops = {
  .owner   = THIS_MODULE,
  .read    = NULL,
  .poll    = NULL,
  .write   = NULL,
  .unlocked_ioctl = xethm_vs_ioctl,
  .open    = xethm_vs_open,
  .release = xethm_vs_close,
  .fasync  = NULL,
  .llseek  = NULL,
};

static struct miscdevice xethm_vs_miscdev = {
  .minor    = MISC_DYNAMIC_MINOR,
  .name   = "xethm_vs",
  .fops   = &xethm_vs_fops
};

/********************************************************************
 * eoe network interface
 ********************************************************************/
struct xethm_eoe_private {
  struct net_device* priv_device;
};

int xethm_eoe_open(struct net_device* dev)
{
  return SUCCESS;
}

int xethm_eoe_stop(struct net_device* dev)
{
  return SUCCESS;
}

int xethm_eoe_tx(struct sk_buff* skb, struct net_device* dev)
{
  struct xethm_eoe *eoe = *((struct xethm_eoe **) netdev_priv(dev));
  if (eoe == NULL)
  {
#if XETHM_VS_DEBUG_LEVEL >= 0    
    printk(KERN_WARNING "XETHM_VS: EoE device not found\n");
#endif

    return -ENODEV;
  }

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: TXing packets...\n");
#endif

  if (eoe->rx_skb) 
  {
#if XETHM_VS_DEBUG_LEVEL >= 2    
    printk(KERN_WARNING "XETHM_VS: EoE RX freeing old socket buffer.\n");
#endif

    dev_kfree_skb(eoe->rx_skb);
  }

  // new socket buffer
  if (!(eoe->rx_skb = skb_clone(skb, GFP_KERNEL))) 
  {
    if (printk_ratelimit())
#if XETHM_VS_DEBUG_LEVEL >= 2    
      printk(KERN_WARNING "XETHM_VS: EoE RX low on mem, frame dropped.\n");
#endif

    eoe->stats.tx_dropped++;
    return -ENOMEM;
  }

  // update statistics
  eoe->stats.tx_packets++;
  eoe->stats.tx_bytes += eoe->rx_skb->len;

#if XETHM_VS_DEBUG_LEVEL >= 2
  printk(KERN_INFO "XETHM_VS: EoE %s RX frame completed with %u octets.\n", eoe->dev->name, eoe->rx_skb->len);
#endif

  // pass socket buffer to network stack
  eoe->rx_skb->dev = eoe->dev;
  eoe->rx_skb->protocol = eth_type_trans(eoe->rx_skb, eoe->dev);
  eoe->rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
  if (netif_rx(eoe->rx_skb)) {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: EoE RX netif_rx failed.\n");
#endif
  }

  //kfree packet
  eoe->rx_skb = NULL;

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Successfully TXed %d bytes\n", skb->len);
#endif

  return SUCCESS;
}

static struct net_device_stats *xethm_eoe_netdev_stats(
        struct net_device *dev /**< EoE net_device */)
{
  struct xethm_eoe *eoe = *((struct xethm_eoe **) netdev_priv(dev));
  return &eoe->stats;
}

static const struct net_device_ops xethm_eoe_ops = {
    .ndo_open = xethm_eoe_open,
    .ndo_stop = xethm_eoe_stop,
    .ndo_start_xmit = xethm_eoe_tx,
    .ndo_get_stats = xethm_eoe_netdev_stats,
};

struct xethm_eoe* xethm_eoe_netdev;

static int __init xethm_vs_init(void)
{
  int res;
  int i;
  char name[10];
  struct xethm_eoe** priv;

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Initializing...\n");
#endif

  // Register the device driver
  res = misc_register(&xethm_vs_miscdev);
  if (res)
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Unable to register device (error %d)\n", res);
#endif

    goto xethm_vs_error1;
  }

  xethm_eoe_netdev = (struct xethm_eoe *) kmalloc(sizeof(struct xethm_eoe), GFP_ATOMIC);
  if (xethm_eoe_netdev == NULL)
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Unable to allocate memory (1)\n");
#endif

    res = -ENOMEM;
    goto xethm_vs_error2;
  }

  snprintf(name, sizeof(name)-1, "eoe");
  if (!(xethm_eoe_netdev->dev = alloc_netdev(sizeof(struct xethm_eoe *), name, NET_NAME_UNKNOWN, ether_setup))) 
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Unable to allocate net_device %s!\n", name);
#endif

    res = -ENODEV;
    goto xethm_vs_error3;
  }

  // initialize net_device
  xethm_eoe_netdev->rx_skb = NULL;
  xethm_eoe_netdev->dev->netdev_ops = &xethm_eoe_ops;
  
  for (i = 0; i < ETH_ALEN; i++)
    xethm_eoe_netdev->dev->dev_addr[i] = 0xAA | (i << 4);

  // initialize private data
  dev_set_drvdata(&xethm_eoe_netdev->dev->dev, xethm_eoe_netdev);

  priv = netdev_priv(xethm_eoe_netdev->dev);
  *priv = xethm_eoe_netdev;

  res = register_netdev(xethm_eoe_netdev->dev);
  if (res)
  {
#if XETHM_VS_DEBUG_LEVEL >= 1    
    printk(KERN_ERR "XETHM_VS: Unable to register network device (error %d)\n", res);
#endif

    goto xethm_vs_error4;
  }

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Initialization completed\n"); 
#endif

  INIT_LIST_HEAD(&xethm_vs_instances.list);

  return res;

xethm_vs_error4:
  free_netdev(xethm_eoe_netdev->dev);

xethm_vs_error3:
  kfree(xethm_eoe_netdev);

xethm_vs_error2:
  misc_deregister(&xethm_vs_miscdev);

xethm_vs_error1:
  return res;
}
                 
static void __exit xethm_vs_exit(void)
{
  struct xethm_vs_instance* instance;
  struct xethm_vs_eoe* eoe = NULL;

  list_for_each_entry(instance, &xethm_vs_instances.list, list) 
  {
    //access the list node through node
    eoe = instance->eoe;
    
    xethm_vs_deinit_chardev(eoe);

    unregister_netdev(eoe->dev); // possibly calls close callback
    free_netdev(eoe->dev);

    // empty transmit queue
    xethm_vs_flush(eoe);

    if (eoe->tx_frame) 
    {
      dev_kfree_skb(eoe->tx_frame->skb);
      kfree(eoe->tx_frame);
    }

    if (eoe->rx_skb)
      dev_kfree_skb(eoe->rx_skb);

    kfree(eoe);
  }

  misc_deregister(&xethm_vs_miscdev);

  if (xethm_eoe_netdev->rx_skb)
    dev_kfree_skb(xethm_eoe_netdev->rx_skb);

  unregister_netdev(xethm_eoe_netdev->dev); // possibly calls close callback
  free_netdev(xethm_eoe_netdev->dev);
  kfree(xethm_eoe_netdev);

#if XETHM_VS_DEBUG_LEVEL >= 3    
  printk(KERN_INFO "XETHM_VS: Finalize completed\n"); 
#endif
}

module_exit(xethm_vs_exit);
module_init(xethm_vs_init);
