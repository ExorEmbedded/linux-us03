/*
 * Kairos SPI driver
 *
 */

//#define VERBOSE

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>

#include "kairos.h"

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static LIST_HEAD(kairos_list);
static DEFINE_MUTEX(kairos_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

extern struct ptp_clock_info kairos_pch_caps;
extern struct dsa_switch_ops kairos_switch_ops;

struct kairos_data* kairos_global;


/*-------------------------------------------------------------------------*/

ssize_t kairos_sync(struct kairos_data *kairos, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	struct spi_device *spi;

	spin_lock_irq(&kairos->spi_lock);
	spi = kairos->spi;
	spin_unlock_irq(&kairos->spi_lock);

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}

ssize_t kairos_sync_write(struct kairos_data *kairos, size_t len)
{
	ssize_t status;
	struct spi_message	m;
	struct spi_transfer* k_tmp;
	struct spi_transfer* k_xfers = kcalloc(1, sizeof(*k_tmp), GFP_KERNEL);

	spi_message_init(&m);

	k_tmp = k_xfers;
	k_tmp->tx_buf	= kairos->tx_buffer;
	k_tmp->len		= len;
	k_tmp->speed_hz	= kairos->speed_hz;

#ifdef VERBOSE
	dev_info(&kairos->spi->dev,
		"  xfer mode %d len %u %s%s%s%dbits %u usec %uHz %d %d %d\n",
		kairos->spi->mode,
		k_tmp->len,
		k_tmp->rx_buf ? "rx " : "",
		k_tmp->tx_buf ? "tx " : "",
		k_tmp->cs_change ? "cs " : "",
		k_tmp->bits_per_word ? : kairos->spi->bits_per_word,
		k_tmp->delay_usecs,
		k_tmp->speed_hz ? : kairos->spi->max_speed_hz,
		k_tmp->cs_change, k_tmp->tx_nbits, k_tmp->rx_nbits);
#endif
	spi_message_add_tail(k_tmp, &m);

	status = kairos_sync(kairos, &m);

	kfree(k_xfers);
	return status;
}

ssize_t kairos_sync_read(struct kairos_data *kairos, size_t len)
{
	ssize_t status;
	struct spi_message	m;
	struct spi_transfer* k_tmp;
	struct spi_transfer* k_xfers = kcalloc(2, sizeof(*k_tmp), GFP_KERNEL);

	spi_message_init(&m);

	k_tmp = k_xfers;
	k_tmp->tx_buf	= kairos->tx_buffer;
	k_tmp->len		= 2;
	k_tmp->speed_hz	= kairos->speed_hz;

#ifdef VERBOSE
	dev_info(&kairos->spi->dev,
		"  xfer mode %d len %u %s%s%s%dbits %u usec %uHz %d %d %d\n",
		kairos->spi->mode,
		k_tmp->len,
		k_tmp->rx_buf ? "rx " : "",
		k_tmp->tx_buf ? "tx " : "",
		k_tmp->cs_change ? "cs " : "",
		k_tmp->bits_per_word ? : kairos->spi->bits_per_word,
		k_tmp->delay_usecs,
		k_tmp->speed_hz ? : kairos->spi->max_speed_hz,
		k_tmp->cs_change, k_tmp->tx_nbits, k_tmp->rx_nbits);
#endif
	spi_message_add_tail(k_tmp, &m);

	k_tmp ++;
	k_tmp->rx_buf	= kairos->rx_buffer;
	k_tmp->len		= len;
	k_tmp->speed_hz	= kairos->speed_hz;

#ifdef VERBOSE
	dev_info(&kairos->spi->dev,
		"  xfer mode %d len %u %s%s%s%dbits %u usec %uHz %d %d %d\n",
		kairos->spi->mode,
		k_tmp->len,
		k_tmp->rx_buf ? "rx " : "",
		k_tmp->tx_buf ? "tx " : "",
		k_tmp->cs_change ? "cs " : "",
		k_tmp->bits_per_word ? : kairos->spi->bits_per_word,
		k_tmp->delay_usecs,
		k_tmp->speed_hz ? : kairos->spi->max_speed_hz,
		k_tmp->cs_change, k_tmp->tx_nbits, k_tmp->rx_nbits);
#endif

	spi_message_add_tail(k_tmp, &m);

	status = kairos_sync(kairos, &m);

	kfree(k_xfers);
	return status;
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
kairos_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct kairos_data	*kairos;
	ssize_t			status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	kairos = filp->private_data;

	mutex_lock(&kairos->buf_lock);
	status = kairos_sync_read(kairos, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, kairos->rx_buffer, status);
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
kairos_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct kairos_data	*kairos;
	ssize_t			status = 0;
	unsigned long		missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	kairos = filp->private_data;

	mutex_lock(&kairos->buf_lock);
	missing = copy_from_user(kairos->tx_buffer, buf, count);
	if (missing == 0)
		status = kairos_sync_write(kairos, count);
	else
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
	unsigned		n, total, tx_total, rx_total;
	u8			*tx_buf, *rx_buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = kairos->tx_buffer;
	rx_buf = kairos->rx_buffer;
	total = 0;
	tx_total = 0;
	rx_total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Since the function returns the total length of transfers
		 * on success, restrict the total to positive int values to
		 * avoid the return value looking like an error.  Also check
		 * each transfer length to avoid arithmetic overflow.
		 */
		if (total > INT_MAX || k_tmp->len > INT_MAX) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			/* this transfer needs space in RX bounce buffer */
			rx_total += k_tmp->len;
			if (rx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->rx_buf = rx_buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
			rx_buf += k_tmp->len;
		}
		if (u_tmp->tx_buf) {
			/* this transfer needs space in TX bounce buffer */
			tx_total += k_tmp->len;
			if (tx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->tx_buf = tx_buf;
			if (copy_from_user(tx_buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
			tx_buf += k_tmp->len;
		}

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
		if (!k_tmp->speed_hz)
			k_tmp->speed_hz = kairos->speed_hz;
#ifdef VERBOSE
		dev_info(&kairos->spi->dev,
			"  xfer mode %d len %u %s%s%s%dbits %u usec %uHz %d %d %d\n",
			kairos->spi->mode,
			k_tmp->len,
			k_tmp->rx_buf ? "rx " : "",
			k_tmp->tx_buf ? "tx " : "",
			k_tmp->cs_change ? "cs " : "",
			k_tmp->bits_per_word ? : kairos->spi->bits_per_word,
			k_tmp->delay_usecs,
			k_tmp->speed_hz ? : kairos->spi->max_speed_hz,
			k_tmp->cs_change, k_tmp->tx_nbits, k_tmp->rx_nbits);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = kairos_sync(kairos, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	rx_buf = kairos->rx_buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, rx_buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
			rx_buf += u_tmp->len;
		}
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static struct spi_ioc_transfer *
kairos_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
		unsigned *n_ioc)
{
	struct spi_ioc_transfer	*ioc;
	u32	tmp;

	/* Check type, command number and direction */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC
			|| _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
			|| _IOC_DIR(cmd) != _IOC_WRITE)
		return ERR_PTR(-ENOTTY);

	tmp = _IOC_SIZE(cmd);
	if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
		return ERR_PTR(-EINVAL);
	*n_ioc = tmp / sizeof(struct spi_ioc_transfer);
	if (*n_ioc == 0)
		return NULL;

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (!ioc)
		return ERR_PTR(-ENOMEM);
	if (__copy_from_user(ioc, u_ioc, tmp)) {
		kfree(ioc);
		return ERR_PTR(-EFAULT);
	}
	return ioc;
}

static long
kairos_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct kairos_data	*kairos;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	kairos = filp->private_data;
	spin_lock_irq(&kairos->spi_lock);
	spi = spi_dev_get(kairos->spi);
	spin_unlock_irq(&kairos->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&kairos->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MODE32:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u32 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(kairos->speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE)
			retval = __get_user(tmp, (u8 __user *)arg);
		else
			retval = __get_user(tmp, (u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u16)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval >= 0)
				kairos->speed_hz = tmp;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
			spi->max_speed_hz = save;
		}
		break;

	default:
		/* segmented and/or full-duplex I/O request */
		/* Check message and copy into scratch area */
		ioc = kairos_get_ioc_message(cmd,
				(struct spi_ioc_transfer __user *)arg, &n_ioc);
		if (IS_ERR(ioc)) {
			retval = PTR_ERR(ioc);
			break;
		}
		if (!ioc)
			break;	/* n_ioc is also 0 */

		/* translate to spi_message, execute */
		retval = kairos_message(kairos, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&kairos->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
kairos_compat_ioc_message(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct spi_ioc_transfer __user	*u_ioc;
	int				retval = 0;
	struct kairos_data		*kairos;
	struct spi_device		*spi;
	unsigned			n_ioc, n;
	struct spi_ioc_transfer		*ioc;

	u_ioc = (struct spi_ioc_transfer __user *) compat_ptr(arg);
	if (!access_ok(VERIFY_READ, u_ioc, _IOC_SIZE(cmd)))
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	kairos = filp->private_data;
	spin_lock_irq(&kairos->spi_lock);
	spi = spi_dev_get(kairos->spi);
	spin_unlock_irq(&kairos->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* SPI_IOC_MESSAGE needs the buffer locked "normally" */
	mutex_lock(&kairos->buf_lock);

	/* Check message and copy into scratch area */
	ioc = kairos_get_ioc_message(cmd, u_ioc, &n_ioc);
	if (IS_ERR(ioc)) {
		retval = PTR_ERR(ioc);
		goto done;
	}
	if (!ioc)
		goto done;	/* n_ioc is also 0 */

	/* Convert buffer pointers */
	for (n = 0; n < n_ioc; n++) {
		ioc[n].rx_buf = (uintptr_t) compat_ptr(ioc[n].rx_buf);
		ioc[n].tx_buf = (uintptr_t) compat_ptr(ioc[n].tx_buf);
	}

	/* translate to spi_message, execute */
	retval = kairos_message(kairos, ioc, n_ioc);
	kfree(ioc);

done:
	mutex_unlock(&kairos->buf_lock);
	spi_dev_put(spi);
	return retval;
}

static long
kairos_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	if (_IOC_TYPE(cmd) == SPI_IOC_MAGIC
			&& _IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(0))
			&& _IOC_DIR(cmd) == _IOC_WRITE)
		return kairos_compat_ioc_message(filp, cmd, arg);

	return kairos_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define kairos_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int kairos_open(struct inode *inode, struct file *filp)
{
	struct kairos_data	*kairos;
	int			status = -ENXIO;

	mutex_lock(&kairos_list_lock);

	list_for_each_entry(kairos, &kairos_list, device_entry) {
		if (kairos->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("kairos: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	kairos->users++;
	filp->private_data = kairos;
	nonseekable_open(inode, filp);

	mutex_unlock(&kairos_list_lock);
	return 0;

err_find_dev:
	mutex_unlock(&kairos_list_lock);
	return status;
}

static int kairos_release(struct inode *inode, struct file *filp)
{
	struct kairos_data	*kairos;

	mutex_lock(&kairos_list_lock);
	kairos = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	kairos->users--;

	mutex_unlock(&kairos_list_lock);

	return 0;
}

static const struct file_operations kairos_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	kairos_write,
	.read =		kairos_read,
	.unlocked_ioctl = kairos_ioctl,
	.compat_ioctl = kairos_compat_ioctl,
	.open =		kairos_open,
	.release =	kairos_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/kairosB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *kairos_class;

#ifdef CONFIG_OF
static const struct of_device_id kairos_dt_ids[] = {
	{ .compatible = "generic,kairos" },
	{},
};
MODULE_DEVICE_TABLE(of, kairos_dt_ids);
#endif


/*-------------------------------------------------------------------------*/

static int kairos_probe(struct spi_device *spi)
{
	struct kairos_data	*kairos;
	int			status;
	unsigned long		minor;

	/*
	 * kairos should never be referenced in DT without a specific
	 * compatible string, it is a Linux implementation thing
	 * rather than a description of the hardware.
	 */
	if (spi->dev.of_node && !of_match_device(kairos_dt_ids, &spi->dev)) {
		dev_err(&spi->dev, "buggy DT: kairos listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
			!of_match_device(kairos_dt_ids, &spi->dev));
	}

	/* Allocate driver data */
	kairos = kzalloc(sizeof(*kairos), GFP_KERNEL);
	if (!kairos)
		return -ENOMEM;

	/* Initialize the driver data */
	kairos->spi = spi;
	spin_lock_init(&kairos->spi_lock);
	mutex_init(&kairos->buf_lock);

	INIT_LIST_HEAD(&kairos->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&kairos_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		kairos->devt = MKDEV(KAIROS_MAJOR, minor);
		dev = device_create(kairos_class, &spi->dev, kairos->devt,
				    kairos, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&kairos->device_entry, &kairos_list);
	}
	mutex_unlock(&kairos_list_lock);

	kairos->speed_hz = spi->max_speed_hz;

	if (status == 0)
	{
		spi_set_drvdata(spi, kairos);
		kairos->tx_buffer = (u8*)kmalloc(bufsiz, GFP_KERNEL);
		if (!kairos->tx_buffer) {
			dev_dbg(&kairos->spi->dev, "open/ENOMEM\n");
			
			kfree(kairos);
			status = -ENOMEM;
		}

		kairos->rx_buffer = (u8*)kmalloc(bufsiz, GFP_KERNEL);
		if (!kairos->rx_buffer) {
			dev_dbg(&kairos->spi->dev, "open/ENOMEM\n");

			kfree(kairos->tx_buffer);
			kfree(kairos);
			status = -ENOMEM;
		}
	}
	else
		kfree(kairos);

	kairos_global = kairos;

	kairos_ptp_init(kairos);
	kairos_switch_init(kairos);

	dev_info(&kairos->spi->dev, "device probed successfully\n");

	return status;
}

static int kairos_remove(struct spi_device *spi)
{
	struct kairos_data	*kairos = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&kairos->spi_lock);
	kairos->spi = NULL;
	spin_unlock_irq(&kairos->spi_lock);

	/* prevent new opens */
	mutex_lock(&kairos_list_lock);

	list_del(&kairos->device_entry);
	device_destroy(kairos_class, kairos->devt);
	clear_bit(MINOR(kairos->devt), minors);
	if (kairos->users == 0)
	{
		int		dofree;

		kfree(kairos->tx_buffer);
		kairos->tx_buffer = NULL;

		kfree(kairos->rx_buffer);
		kairos->rx_buffer = NULL;

		/* ... after we unbound from the underlying device? */
		dofree = (kairos->spi == NULL);

		if (dofree)
			kfree(kairos);
	}

	mutex_unlock(&kairos_list_lock);

	return 0;
}

static struct spi_driver kairos_spi_driver = {
	.driver = {
		.name =		"kairos",
		.of_match_table = of_match_ptr(kairos_dt_ids),
	},
	.probe =	kairos_probe,
	.remove =	kairos_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init kairos_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(KAIROS_MAJOR, "spi-kairos", &kairos_fops);
	if (status < 0)
		return status;

	kairos_class = class_create(THIS_MODULE, "kairos");
	if (IS_ERR(kairos_class)) {
		unregister_chrdev(KAIROS_MAJOR, kairos_spi_driver.driver.name);
		return PTR_ERR(kairos_class);
	}

	status = spi_register_driver(&kairos_spi_driver);
	if (status < 0) {
		class_destroy(kairos_class);
		unregister_chrdev(KAIROS_MAJOR, kairos_spi_driver.driver.name);
	}
	return status;
}
module_init(kairos_init);

static void __exit kairos_exit(void)
{
	spi_unregister_driver(&kairos_spi_driver);
	class_destroy(kairos_class);
	unregister_chrdev(KAIROS_MAJOR, kairos_spi_driver.driver.name);
}
module_exit(kairos_exit);

MODULE_AUTHOR("Ambrogio Galbusera");
MODULE_DESCRIPTION("User mode KAIROS device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:kairos");
