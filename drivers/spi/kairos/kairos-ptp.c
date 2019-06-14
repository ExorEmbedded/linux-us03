
#include "kairos.h"

static DEFINE_SPINLOCK(kairos_ptp_list_lock);
static struct kairos_ptp_operation kairos_ptp_list;

extern struct kairos_data* kairos_global;


/*-------------------------------------------------------------------------*/
/*
 * PTP clock operations
 */
/*-------------------------------------------------------------------------*/
static int kairos_pch_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u16 tmp;
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
	tmp = neg_adj? 0x8000 : 0x0000;
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_DRIFT, tmp);

	// write (dummy)
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_DRIFT, 0x0000);

	// write (dummy)
	kairos_reg_write(kairos, KAIROS_MODULE_PTP,  KAIROS_REG_PTP_DRIFT, 0x0000);

	// write addendH
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_DRIFT, addendH);

	// write addendL
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_DRIFT, addendL);

	mutex_unlock(&kairos->buf_lock);

	return 0;
}

static int kairos_pch_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	int neg_adj = 0;
	u16 tmp;
	u16 nsVal;
	u16 countH, countL;

	struct kairos_data *kairos = container_of(ptp, struct kairos_data, caps);

	dev_info(&kairos->spi->dev, "kairos_pch_adjtime: %lld\n", delta);

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
	tmp = neg_adj? 0x8000 : 0x0000;
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_OFFSET, tmp);

	// write nsVal
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_OFFSET, nsVal);

	// write interval
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_OFFSET, 0x0000);

	// write countH
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_OFFSET, countH);

	// write countL
	kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_OFFSET, countL);

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

	dev_info(&kairos->spi->dev, "kairos_pch_gettime\n");

	mutex_lock(&kairos->buf_lock);

	for (i=0; i<5; i++)
	{
		// read 
		tmp = 0x0000;
		status = kairos_reg_read(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_CLK_RD, &tmp);
		if (status < 0)
		{
			dev_err(&kairos->spi->dev, "error reading register %d: %d", i, status);
			mutex_unlock(&kairos->buf_lock);
			return -EFAULT;
		}

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

	dev_info(&kairos->spi->dev, "kairos_pch_gettime %lld.%ld\n", ts->tv_sec, ts->tv_nsec);

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

	dev_info(&kairos->spi->dev, "kairos_pch_settime %lld.%ld\n", ts->tv_sec, ts->tv_nsec);

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
		kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_CLK_WR, tmp16);
	}

	mutex_unlock(&kairos->buf_lock);

	return 0;
}

static int kairos_pch_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	u16 tmp;
	struct kairos_data *kairos = container_of(ptp, struct kairos_data, caps);

	dev_info(&kairos->spi->dev, "kairos_pch_enable %d\n", on);

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		switch (rq->extts.index) {
		case 0:
			kairos->exts0_enabled = on ? 1 : 0;

			// enable clock
			tmp = 0x5000;
			kairos_reg_write(kairos, KAIROS_MODULE_PTP, KAIROS_REG_PTP_STATUS, tmp);
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

struct ptp_clock_info kairos_pch_caps = {
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

int kairos_ptp_init(struct kairos_data* kairos)
{
	spin_lock_init(&kairos_ptp_list_lock);
	INIT_LIST_HEAD(&kairos_ptp_list.list);

	kairos->ptp_thread = kthread_create(kairos_ptp_thread, kairos, "KPTP");
	if (kairos->ptp_thread != NULL)
	{
		wake_up_process(kairos->ptp_thread);
	}
	else
		dev_err(&kairos->spi->dev, "error creating PTP thread\n");

	return 0;
}