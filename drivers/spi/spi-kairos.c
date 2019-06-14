/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

// PTP support
#include <linux/ptp_clock_kernel.h>
#include <net/dsa.h>

#include <asm/uaccess.h>

//#define VERBOSE

#define KAIROS_MAX_PORTS		2
#define N_EXT_TS				1	
#define KAIROS_MODULE_GENERAL	0x01
#define KAIROS_MODULE_TSN		0x02
#define KAIROS_MODULE_PTP		0x03

#define KAIROS_REG_PTP_STATUS	0x0C
#define KAIROS_REG_PTP_CLK_RD	0x0D
#define KAIROS_REG_PTP_CLK_WR	0x0E
#define KAIROS_REG_PTP_OFFSET	0x0F
#define KAIROS_REG_PTP_DRIFT	0x10

#define KAIROS_REG_GR_MODID    0x00
#define KAIROS_REG_GR_RELLB    0x01
#define KAIROS_REG_GR_RELHB    0x02
#define KAIROS_REG_GR_BDTLB    0x03
#define KAIROS_REG_GR_BDTHB    0x04
#define KAIROS_REG_GR_CTRL     0x05
#define KAIROS_REG_GR_GPIO_IN  0x18
#define KAIROS_REG_GR_GPIO_OUT 0x19
#define KAIROS_REG_GR_GPIO_TRIS 0x1A
#define KAIROS_REG_GR_GPIOA_IN  0x1B
#define KAIROS_REG_GR_GPIOA_OUT 0x1C
#define KAIROS_REG_GR_GPIOA_TRIS 0x1D
#define KAIROS_REG_GR_GPIOB_IN  0x1F
#define KAIROS_REG_GR_GPIOB_OUT 0x20
#define KAIROS_REG_GR_GPIOB_TRIS 0x21

#define GET_YEAR(x)     ((x & 0xFC000000) >> 26)
#define GET_MONTH(x)    ((x & 0x03C00000) >> 24)
#define GET_DAY(x)      ((x & 0x003E0000) >> 17)
#define GET_HOUR(x)     ((x & 0x0001F000) >> 12)
#define GET_MIN(x)      ((x & 0x00000FC0) >>  6)
#define GET_SEC(x)      ((x & 0x0000003F) >>  0)

#define GET_MAJOR(x)    ((x & 0xFF000000) >> 24)
#define GET_MINOR(x)    ((x & 0x00FF0000) >> 16)
#define GET_BUGFIX(x)   ((x & 0x0000FF00) >>  8)
#define GET_BSTEP(x)    ((x & 0x000000FF) >>  0)

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define KAIROS_MAJOR			510	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

struct kairos_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct bin_attribute	bin;

	struct phy_device* phys[KAIROS_MAX_PORTS];

	/* PTP support fields */
	struct ptp_clock *ptp_clocks[KAIROS_MAX_PORTS+1];
	struct ptp_clock_info caps;

	int exts0_enabled;
	int exts1_enabled;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex	buf_lock;
	unsigned		users;
	u8				tx_buffer[16];
	u8				rx_buffer[16];
	u32				speed_hz;

	/* PTP management thread */
	struct task_struct* ptp_thread;

	struct dsa_switch* ds;
	int sw_addr;
};

/*
 * structure of the operations that the FEC driver will perform on PHC device
 * structures are organized in a linked list
 */
#define KAIROS_OP_ADJ_TIME		0x00
#define KAIROS_OP_ADJ_FREQ		0x01

struct kairos_ptp_operation {
	u8 op;
	s64 value;

	struct list_head list;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DEFINE_SPINLOCK(kairos_ptp_list_lock);
static struct kairos_ptp_operation kairos_ptp_list;

static struct kairos_data* kairos_global;


static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

extern int kairos_phy_add(unsigned int irq, int phy_addr,
		  struct fixed_phy_status *status,
		  struct dsa_switch* ds);
extern struct phy_device *kairos_phy_register(unsigned int irq,
	      struct fixed_phy_status *status,
	      struct dsa_switch* ds,
	      struct device_node *np);
extern void kairos_phy_unregister(struct phy_device* phy);


/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void kairos_complete(void *arg)
{
	complete(arg);
}

static ssize_t
kairos_sync(struct kairos_data *kairos, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = kairos_complete;
	message->context = &done;

	spin_lock_irq(&kairos->spi_lock);
	if (kairos->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(kairos->spi, message);
	spin_unlock_irq(&kairos->spi_lock);

//printk(KERN_INFO "after spi-async %d\n",status);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
//printk(KERN_INFO "after wait_for_compeltion %d\n",status);
		if (status == 0)
			status = message->actual_length;
	}
//printk(KERN_INFO "returning %d\n",status);
	return status;
}

static inline ssize_t
kairos_sync_write(struct kairos_data *kairos, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= kairos->tx_buffer,
			.len		= len,
			.speed_hz	= kairos->speed_hz,
		};
	struct spi_message	m;

#ifdef VERBOSE
	dev_info(&kairos->spi->dev,
		"  writing %zd bytes @%u Hz - %02x %02x %02x %02x \n",
		len, kairos->speed_hz,
		kairos->tx_buffer[0],
		kairos->tx_buffer[1],
		kairos->tx_buffer[2],
		kairos->tx_buffer[3]);
#endif

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return kairos_sync(kairos, &m);
}

static inline ssize_t
kairos_sync_read(struct kairos_data *kairos, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= kairos->rx_buffer,
			.len		= len,
			.speed_hz	= kairos->speed_hz,
		};
	struct spi_message	m;
	size_t res;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	res = kairos_sync(kairos, &m);

#ifdef VERBOSE
	dev_info(&kairos->spi->dev,
		"  reading %zd bytes - %02x %02x %02x %02x\n",
		len,
		kairos->rx_buffer[0],
		kairos->rx_buffer[1],
		kairos->rx_buffer[2],
		kairos->rx_buffer[3]);
#endif

	return res;
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
kairos_read(struct file *filp, struct kobject *kobj,
			struct bin_attribute *bin_attr,
			char *buf, loff_t off, size_t count)
{
	struct device* dev;
	struct kairos_data	*kairos;
	ssize_t			status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	dev = container_of(kobj, struct device, kobj);
	kairos = dev_get_drvdata(dev);

	mutex_lock(&kairos->buf_lock);
	memset(kairos->rx_buffer, 0xAA, 4);
	status = kairos_sync_read(kairos, count);
	if (status > 0) {
		unsigned long	missing;

		missing = 0; //copy_to_user(buf, kairos->rx_buffer, status);
		memcpy(buf, kairos->rx_buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&kairos->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
kairos_write(struct file *filp, struct kobject *kobj,
			struct bin_attribute *bin_attr,
			char *buf, loff_t off, size_t count)
{
	struct device		*dev;
	struct kairos_data	*kairos;
	ssize_t			status = 0;
	unsigned long		missing;

	dev = container_of(kobj, struct device, kobj);

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	kairos = dev_get_drvdata(dev);

	mutex_lock(&kairos->buf_lock);
dev_info(&kairos->spi->dev, "kairos_write (1)\n");
	missing = 0; //copy_from_user(kairos->tx_buffer, buf, count);
	memcpy(kairos->tx_buffer, buf, count);
dev_info(&kairos->spi->dev, "kairos_write (2 - %lu)\n", missing);
	if (missing == 0) {
		status = kairos_sync_write(kairos, count);
	} else
		status = -EFAULT;
	mutex_unlock(&kairos->buf_lock);

	return status;
}

static int kairos_message(struct kairos_data *kairos,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = kairos->tx_buffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		buf += k_tmp->len;

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_info(&kairos->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : kairos->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : kairos->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = kairos_sync(kairos, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = kairos->rx_buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}


/*-------------------------------------------------------------------------*/
/* 
 * Basic register access 
 */                                   
/*-------------------------------------------------------------------------*/
static int kairos_reg_read(struct kairos_data* kairos, u8 module, u8 reg, u16* data)
{
	int status;
	u16 tmp;

	memset(kairos->rx_buffer, 0xcc, 4);

	kairos->tx_buffer[0] = module;
	kairos->tx_buffer[1] = reg;
	kairos_sync_write(kairos, 2);

	status = kairos_sync_read(kairos, 2);
	if (status <= 0)
		return -EFAULT;

printk(KERN_INFO "%s %02X %02X\n", __func__, kairos->rx_buffer[0], kairos->rx_buffer[1]);
	tmp = kairos->rx_buffer[0];
	tmp = (tmp << 8) | kairos->rx_buffer[1];

	*data = tmp;
	return 0;
}

static int kairpos_reg_write(struct kairos_data* kairos, u8 module, u8 reg, u16 data)
{
	kairos->tx_buffer[0] = module;
	kairos->tx_buffer[1] = reg;
	kairos->tx_buffer[2] = (u8)((data >> 8) & 0x00FF);
	kairos->tx_buffer[3] = (u8)(data & 0x00FF);

	kairos_sync_write(kairos, 4);
	return 0;
}

/*-------------------------------------------------------------------------*/
/*
 * PTP clock operations
 */
/*-------------------------------------------------------------------------*/
static int kairos_pch_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u16 addendH, addendL;
	int neg_adj = 0;
	struct kairos_data *kairos = container_of(ptp, struct kairos_data, caps);
	struct spi_device *spi = kairos->spi;

	dev_info(&spi->dev, "kairos_pch_adjfreq: %d\n", ppb);

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	addendH = (u16)((ppb >> 16) & 0x03FFF);
	addendL = (u16)(ppb & 0x0FFFF);

	mutex_lock(&kairos->buf_lock);

	// write negative bit
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_DRIFT;
	kairos->tx_buffer[2] = neg_adj? 0x80 : 0x00;
	kairos->tx_buffer[3] = 0x00;
	kairos_sync_write(kairos, 4);

	// write (dummy)
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_DRIFT;
	kairos->tx_buffer[2] = 0x00;
	kairos->tx_buffer[3] = 0x00;
	kairos_sync_write(kairos, 4);

	// write (dummy)
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_DRIFT;
	kairos->tx_buffer[2] = 0x00;
	kairos->tx_buffer[3] = 0x00;
	kairos_sync_write(kairos, 4);

	// write addendH
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_DRIFT;
	kairos->tx_buffer[2] = (u8)((addendH >> 8) & 0xFF);
	kairos->tx_buffer[3] = (u8)(addendH & 0xFF);
	kairos_sync_write(kairos, 4);

	// write addendL
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_DRIFT;
	kairos->tx_buffer[2] = (u8)((addendL >> 8) & 0xFF);
	kairos->tx_buffer[3] = (u8)(addendL & 0xFF);
	kairos_sync_write(kairos, 4);

	mutex_unlock(&kairos->buf_lock);

	return 0;
}

static int kairos_pch_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	int neg_adj = 0;
	u16 nsVal;
	u16 countH, countL;

	struct kairos_data *kairos = container_of(ptp, struct kairos_data, caps);

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	nsVal = 1;
	while (delta > 0x3FFFFFFF)
	{
		nsVal ++;
		delta >>= 2;
	}

	countH = (u16)((delta >> 16) & 0x03FFF);
	countL = (u16)(delta & 0x0FFFF);

	mutex_lock(&kairos->buf_lock);

	// write negative bit
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_OFFSET;
	kairos->tx_buffer[2] = neg_adj? 0x80 : 0x00;
	kairos->tx_buffer[3] = 0x00;
	kairos_sync_write(kairos, 4);

	// write nsVal
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_OFFSET;
	kairos->tx_buffer[2] = 0x00;
	kairos->tx_buffer[3] = nsVal;
	kairos_sync_write(kairos, 4);

	// write interval
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_OFFSET;
	kairos->tx_buffer[2] = 0x00;
	kairos->tx_buffer[3] = 0x00;
	kairos_sync_write(kairos, 4);

	// write countH
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_OFFSET;
	kairos->tx_buffer[2] = (u8)((countH >> 8) & 0xFF);
	kairos->tx_buffer[3] = (u8)(countH & 0xFF);
	kairos_sync_write(kairos, 4);

	// write countL
	kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_OFFSET;
	kairos->tx_buffer[2] = (u8)((countL >> 8) & 0xFF);
	kairos->tx_buffer[3] = (u8)(countL & 0xFF);
	kairos_sync_write(kairos, 4);

	mutex_unlock(&kairos->buf_lock);

	return 0;
}

static int kairos_pch_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	int i;
	int status;
	u16 tmp;
	u16 secH = 0;
	u16 secM = 0;
	u16 secL = 0;
	u16 nsecH = 0;
	u16 nsecL = 0;
	struct kairos_data *kairos = container_of(ptp, struct kairos_data, caps);

	mutex_lock(&kairos->buf_lock);

	for (i=0; i<5; i++)
	{
		// read 
		kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
		kairos->tx_buffer[1] = KAIROS_REG_PTP_CLK_RD;
		kairos_sync_write(kairos, 2);

		status = kairos_sync_read(kairos, 2);
		if (status <= 0)
			return -EFAULT;

		tmp = kairos->rx_buffer[0];
		tmp = (tmp << 8) | kairos->rx_buffer[1];

		switch (i)
		{
			case 0: // sec (H)
				secH = tmp;
				break;
			case 1: // sec (M)
				secM = tmp;
				break;
			case 2: // sec (L)
				secL = tmp;
				break;
			case 3: // nanosec (H)
				nsecH = tmp;
				break;
			case 4: // nanosec (L)
				nsecL = tmp;
				break;
		}
	}

	mutex_unlock(&kairos->buf_lock);

	ts->tv_sec = secH;
	ts->tv_sec = (ts->tv_sec << 8) | secM;
	ts->tv_sec = (ts->tv_sec << 8) | secL;

	ts->tv_nsec = nsecH;
	ts->tv_nsec = (ts->tv_nsec << 8) | nsecL;

	return 0;
}

static int kairos_pch_settime(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	int i;
	u64 tmp64;
	u16 tmp16;
	u16 secH, secM, secL;
	u16 nsecH, nsecL;
	struct kairos_data *kairos = container_of(ptp, struct kairos_data, caps);

	tmp64 = ts->tv_sec;
	secL = (u16)(tmp64 & 0xFFFF); tmp64 = tmp64 >> 16;
	secM = (u16)(tmp64 & 0xFFFF); tmp64 = tmp64 >> 16;
	secH = (u16)(tmp64 & 0xFFFF);

	tmp64 = ts->tv_nsec;
	nsecL = (u16)(tmp64 & 0xFFFF); tmp64 = tmp64 >> 16;
	nsecH = (u16)(tmp64 & 0xFFFF);

	mutex_lock(&kairos->buf_lock);

	for (i=0; i<5; i++)
	{
		switch (i)
		{
			case 0: // sec (H)
				tmp16 = secH;
				break;
			case 1: // sec (M)
				tmp16 = secM;
				break;
			case 2: // sec (L)
				tmp16 = secL;
				break;
			case 3: // nanosec (H)
				tmp16 = nsecH;
				break;
			case 4: // nanosec (L)
				tmp16 = nsecL;
				break;
		}

		// write clock
		kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
		kairos->tx_buffer[1] = KAIROS_REG_PTP_CLK_WR;
		kairos->tx_buffer[2] = (u8)((tmp16 >> 8) & 0xFF);
		kairos->tx_buffer[3] = (u8)(tmp16 & 0xFF);
		kairos_sync_write(kairos, 4);
	}

	mutex_unlock(&kairos->buf_lock);

	return 0;
}

static int kairos_pch_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	struct kairos_data *kairos = container_of(ptp, struct kairos_data, caps);

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		switch (rq->extts.index) {
		case 0:
			kairos->exts0_enabled = on ? 1 : 0;

			// enable clock
			kairos->tx_buffer[0] = KAIROS_MODULE_PTP;
			kairos->tx_buffer[1] = KAIROS_REG_PTP_STATUS;
			kairos->tx_buffer[2] = 0x50;
			kairos->tx_buffer[3] = 0x00;
			kairos_sync_write(kairos, 4);
			break;
		case 1:
			kairos->exts1_enabled = on ? 1 : 0;
			break;
		default:
			return -EINVAL;
		}
		return 0;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int kairos_ptp_thread(void* data)
{
	struct kairos_data* kairos = (struct kairos_data*)data;
	struct kairos_ptp_operation* op;

	while (1)
	{
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock(&kairos_ptp_list_lock);
		if (list_empty(&kairos_ptp_list.list))
		{
			spin_unlock(&kairos_ptp_list_lock);
			schedule();

			spin_lock(&kairos_ptp_list_lock);
		}

		set_current_state(TASK_RUNNING);

		list_for_each_entry(op, &kairos_ptp_list.list, list)
		{
			// execute requested operation
			if (op->op == KAIROS_OP_ADJ_TIME)
				kairos_pch_adjtime(&kairos->caps, op->value);
			else if (op->op == KAIROS_OP_ADJ_FREQ)
				kairos_pch_adjfreq(&kairos->caps, (s32)op->value);

			// remove entry
			list_del(&op->list);
			kfree(op);
		}

		spin_unlock(&kairos_ptp_list_lock);
	}

	return 0;
}

static struct ptp_clock_info kairos_pch_caps = {
	.owner		= THIS_MODULE,
	.name		= "PCH timer",
	.max_adj	= 50000000,
	.n_ext_ts	= N_EXT_TS,
	.pps		= 0,
	.adjfreq	= kairos_pch_adjfreq,
	.adjtime	= kairos_pch_adjtime,
	.gettime64	= kairos_pch_gettime,
	.settime64	= kairos_pch_settime,
	.enable		= kairos_pch_enable,
};

/*-------------------------------------------------------------------------*/
/* 
 * Switch implementation 
 */                                   
/*-------------------------------------------------------------------------*/
static struct fixed_phy_status kairos_phy_status __initdata = {
	.link		= 1,
	.speed		= 100,
	.duplex		= 1,
};

static struct net_device* kairos_get_netdev(struct dsa_switch* ds)
{
	struct net_device* netdev = ds->dst->master_netdev;
	if (!netdev)
		netdev = ds->master_netdev;

	return netdev;
}

static enum dsa_tag_protocol kairos_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_KAIROS;
}

static const char *kairos_drv_probe(struct device *dsa_dev,
				       struct device *host_dev, int sw_addr,
				       void **_priv)
{
	int i; 
	struct dsa_switch* ds = (struct dsa_switch*)container_of(dsa_dev, struct dsa_switch, dev);

	struct kairos_data *kairos = kairos_global;

	printk(KERN_INFO "%s: %p %p %d %p", __func__, dsa_dev, host_dev, sw_addr, _priv);
	printk(KERN_INFO "%s (2): %p %p ", __func__, ds, kairos);

	for (i=0; i<KAIROS_MAX_PORTS; i++)
	{
		// add simulated phys
		kairos->phys[i] = kairos_phy_register(-1, &kairos_phy_status, ds, NULL);

		// add PHC devices
		kairos->caps = kairos_pch_caps;
		kairos->ptp_clocks[i] = ptp_clock_register(&kairos->caps, &kairos->spi->dev);
		if (IS_ERR(kairos->ptp_clocks[i])) 
		{
			dev_err(ds->dev, "error creating pch device: %ld\n", PTR_ERR(kairos->ptp_clocks[i]));
		}
		else
		{
			dev_info(ds->dev, "pch device ptp%d created\n", ptp_clock_index(kairos->ptp_clocks[i]));
		}
	}

	kairos->ds = ds;
	kairos->sw_addr = sw_addr;

	*_priv = kairos;

	dev_info(ds->dev, "kairos switch probed");

	return "KAIROS";
}

static int kairos_switch_reset(struct dsa_switch *ds)
{
	//@todo
	struct net_device* netdev = kairos_get_netdev(ds);

	dev_info(&netdev->dev, "kairos switch reset");
	return 0;
}

static int kairos_setup_global(struct dsa_switch *ds)
{
	//@todo
	struct net_device* netdev = kairos_get_netdev(ds);

	dev_info(&netdev->dev, "kairos switch setup global");
	return 0;
}

static int kairos_setup_port(struct dsa_switch *ds, int p)
{
	//@todo
	struct net_device* netdev = kairos_get_netdev(ds);

	dev_info(&netdev->dev, "kairos switch setup port");

	return 0;
}

static int kairos_setup(struct dsa_switch *ds)
{
	u32 build_date;
	u16 tmp;
	struct net_device* netdev;
	struct kairos_data *kairos;

	netdev = kairos_get_netdev(ds);
	if (!netdev)
		return -EINVAL;

	kairos = (struct kairos_data*)ds->priv;
	if (!kairos)
		return -EINVAL;

    printk(KERN_INFO "priv: %p SPI address: %p\n", kairos, kairos->spi);

	kairos_reg_read(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_BDTHB, &tmp);
	build_date = tmp;

	kairos_reg_read(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_BDTLB, &tmp);
	build_date = (build_date << 16) | tmp;

    printk(KERN_INFO "Build date: %d/%d/%d %d:%d:%d\n",
           GET_YEAR(build_date), GET_MONTH(build_date), GET_DAY(build_date),
           GET_HOUR(build_date), GET_MIN(build_date), GET_SEC(build_date)); 

	dev_info(&netdev->dev, "kairos switch setup");
	return 0;
}

static int kairos_set_addr(struct dsa_switch *ds, u8 *addr)
{
	//@todo
	struct net_device* netdev = ds->dst->master_netdev;
	if (!netdev)
		netdev = ds->master_netdev;

	dev_info(&netdev->dev, "kairos switch set addr");
	return 0;
}

static int kairos_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	u16 link_status;
	int result;
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;

	struct net_device* netdev = ds->dst->master_netdev;
;
	dev_info(&netdev->dev, "kairos switch phy read port %d, reg %d", port, regnum);

	// only read of register 0x01 (status register) is supported
	if (regnum != 0x01)
		return 0xffff;

	//@todo
	// read link status
	kairos->tx_buffer[0] = KAIROS_MODULE_GENERAL;
	kairos->tx_buffer[1] = KAIROS_REG_PTP_CLK_RD;
	kairos_sync_read(kairos, 2);

	link_status = kairos->rx_buffer[0];
	link_status = (link_status << 8) | kairos->rx_buffer[1];

	result = link_status;

	return result;
}

static int kairos_phy_write(struct dsa_switch *ds, int port, int regnum, u16 val)
{
	// write not supported
	struct net_device* netdev = ds->master_netdev;
	dev_info(&netdev->dev, "kairos switch phy write");
	return 0xffff;
}

static struct dsa_switch_ops kairos_switch_ops = {
	.get_tag_protocol = kairos_get_tag_protocol,
	.probe		= kairos_drv_probe,
	.setup		= kairos_setup,
	.set_addr	= kairos_set_addr,
	.phy_read	= kairos_phy_read,
	.phy_write	= kairos_phy_write,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/kairosB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *kairos_class;

/*-------------------------------------------------------------------------*/

static int kairos_probe(struct spi_device *spi)
{
	struct kairos_data	*kairos;
	int			status;

	dev_info(&spi->dev, "probing (SPI address %p)...\n", spi);

	/* Allocate driver data */
	kairos = kzalloc(sizeof(*kairos), GFP_KERNEL);
	if (!kairos)
	{
		dev_dbg(&spi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	/* Initialize the driver data */
	kairos->spi = spi;
	spin_lock_init(&kairos->spi_lock);
	mutex_init(&kairos->buf_lock);

	INIT_LIST_HEAD(&kairos->device_entry);

	kairos->speed_hz = spi->max_speed_hz;
	spi_set_drvdata(spi, kairos);

	/* Export the EEPROM bytes through sysfs, since that's convenient.
	 * And maybe to other kernel code; it might hold a board's Ethernet
	 * address, or board-specific calibration data generated on the
	 * manufacturing floor.
	 *
	 * Default to root-only access to the data; EEPROMs often hold data
	 * that's sensitive for read and/or write, like ethernet addresses,
	 * security codes, board-specific manufacturing calibrations, etc.
	 */
	sysfs_bin_attr_init(&kairos->bin);
	kairos->bin.attr.name = "kairos-dev";
	kairos->bin.attr.mode = S_IRUSR | S_IWUSR;
	kairos->bin.read = kairos_read;
	kairos->bin.write = kairos_write;

	kairos->bin.size = 0xFF;

	status = sysfs_create_bin_file(&spi->dev.kobj, &kairos->bin);
	if (status)
	{
		dev_err(&spi->dev, "error creating bin file: %d\n", status);
		return status;
	}

	kairos->caps = kairos_pch_caps;
	kairos->ptp_clocks[KAIROS_MAX_PORTS] = ptp_clock_register(&kairos->caps, &spi->dev);
	if (IS_ERR(kairos->ptp_clocks[KAIROS_MAX_PORTS])) {
		status = PTR_ERR(kairos->ptp_clocks[KAIROS_MAX_PORTS]);
		kfree(kairos);

		dev_err(&spi->dev, "error creating pch device: %d\n", status);
	}

	dev_err(&spi->dev, "pch device ptp%d created\n", ptp_clock_index(kairos->ptp_clocks[KAIROS_MAX_PORTS]));

	spin_lock_init(&kairos_ptp_list_lock);
	INIT_LIST_HEAD(&kairos_ptp_list.list);

	kairos->ptp_thread = kthread_create(kairos_ptp_thread, kairos, "KPTP");
	if (kairos->ptp_thread != NULL)
	{
		wake_up_process(kairos->ptp_thread);
	}
	else
		dev_err(&spi->dev, "error creating PTP thread\n");

	kairos_global = kairos;

	// register switch device
	register_switch_driver(&kairos_switch_ops);

	dev_info(&spi->dev, "probe completed (%d)\n", status);

	return status;
}

static int kairos_remove(struct spi_device *spi)
{
	int i;
	struct kairos_data *kairos = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&kairos->spi_lock);
	kairos->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&kairos->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&kairos->device_entry);
	device_destroy(kairos_class, kairos->devt);
	clear_bit(MINOR(kairos->devt), minors);
	if (kairos->users == 0)
		kfree(kairos);
	mutex_unlock(&device_list_lock);

	for (i=0; i<KAIROS_MAX_PORTS+1; i++)
	{
		if (kairos->ptp_clocks[i])
			ptp_clock_unregister(kairos->ptp_clocks[i]);
	}

	for (i=0; i<KAIROS_MAX_PORTS; i++)
	{
		if (kairos->phys[i])
			kairos_phy_unregister(kairos->phys[i]);
	}

	if (kairos->ptp_thread != NULL)
	{
		kthread_stop(kairos->ptp_thread);
	}

	return 0;
}


static const struct of_device_id kairos_dt_ids[] = {
	{ .compatible = "generic,kairos" },
	{ }
};

MODULE_DEVICE_TABLE(of, kairos_dt_ids);

static struct spi_driver kairos_spi_driver = {
	.driver = {
		.name =		"kairos",
		.owner =	THIS_MODULE,
		.of_match_table = kairos_dt_ids,
	},
	.probe =	kairos_probe,
	.remove =	kairos_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};
module_spi_driver(kairos_spi_driver);

/*-------------------------------------------------------------------------*/

MODULE_AUTHOR("Exor International");
MODULE_DESCRIPTION("Kairos SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:kairos");

/*-------------------------------------------------------------------------*/

/*
 * interface functions
 */
static int kairos_ptp_adjust(u8 op, s64 value)
{
	struct kairos_ptp_operation* p;

	p = kmalloc(sizeof(struct kairos_ptp_operation), GFP_KERNEL);
	if (p == NULL)
		return -ENOMEM;

	p->op = op;
	p->value = value;

	spin_lock(&kairos_ptp_list_lock);
	list_add_tail(&p->list, &kairos_ptp_list.list);
	spin_unlock(&kairos_ptp_list_lock);

	return 0;
}

int kairos_ptp_adj_time(s64 delta)
{
	return 0;
	return kairos_ptp_adjust(KAIROS_OP_ADJ_TIME, delta);
}

int kairos_ptp_adj_freq(s32 ppb)
{
	return 0;
	return kairos_ptp_adjust(KAIROS_OP_ADJ_FREQ, ppb);
}
