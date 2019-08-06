

#include "kairos.h"

extern ssize_t kairos_sync(struct kairos_data *kairos, struct spi_message *message);


/*-------------------------------------------------------------------------*/
/* 
 * Basic register access 
 */                                   
/*-------------------------------------------------------------------------*/
int kairos_reg_read(struct kairos_data* kairos, u8 module, u8 reg, u16* data)
{
	int result;

	mutex_lock(&kairos->buf_lock);
	result = _kairos_reg_read(kairos, module, reg, data);
	mutex_unlock(&kairos->buf_lock);

	return result;
}

int _kairos_reg_read(struct kairos_data* kairos, u8 module, u8 reg, u16* data)
{
	u16 tmp;
	int status;
	struct spi_message	m;
	struct spi_transfer* k_tmp;
	struct spi_transfer* k_xfers = kcalloc(2, sizeof(*k_tmp), GFP_KERNEL);

	kairos->tx_buffer[0] = module;
	kairos->tx_buffer[1] = reg;
	kairos->tx_buffer[2] = (u8)((*data >> 8) & 0x00FF);
	kairos->tx_buffer[3] = (u8)(*data & 0x00FF);

	spi_message_init(&m);

	/*
	k_tmp = k_xfers;
	k_tmp->tx_buf	= kairos->tx_buffer;
	k_tmp->len		= 2;
	k_tmp->speed_hz	= kairos->speed_hz;

	spi_message_add_tail(k_tmp, &m);

	k_tmp ++;
	k_tmp->rx_buf	= kairos->rx_buffer;
	k_tmp->len		= 2;
	k_tmp->speed_hz	= kairos->speed_hz;
	*/

	k_tmp = k_xfers;
	k_tmp->tx_buf	= kairos->tx_buffer;
	k_tmp->rx_buf	= kairos->rx_buffer;
	k_tmp->len		= 4;
	k_tmp->speed_hz	= kairos->speed_hz;

	spi_message_add_tail(k_tmp, &m);

	status = kairos_sync(kairos, &m);

	kfree(k_xfers);
	if (status <= 0)
		return -EINVAL;

	tmp = kairos->rx_buffer[2];
	tmp = (tmp << 8) | kairos->rx_buffer[3];

	dev_dbg(&kairos->spi->dev, "%s reg %d.%02X --> %02X %02X %02X %02X - %04X\n", __func__, module, reg, kairos->rx_buffer[0], kairos->rx_buffer[1], kairos->rx_buffer[2], kairos->rx_buffer[3], tmp);

	*data = tmp;
	return 0;
}

int kairos_reg_write(struct kairos_data* kairos, u8 module, u8 reg, u16 data)
{
	int result;

	mutex_lock(&kairos->buf_lock);
	result = _kairos_reg_write(kairos, module, reg, data);
	mutex_unlock(&kairos->buf_lock);

	return result;
}

int _kairos_reg_write(struct kairos_data* kairos, u8 module, u8 reg, u16 data)
{
	int status;
	struct spi_message	m;
	struct spi_transfer* k_tmp;
	struct spi_transfer* k_xfers = kcalloc(1, sizeof(*k_tmp), GFP_KERNEL);

	kairos->tx_buffer[0] = module;
	kairos->tx_buffer[1] = reg;
	kairos->tx_buffer[2] = (u8)((data >> 8) & 0x00FF);
	kairos->tx_buffer[3] = (u8)(data & 0x00FF);

	dev_dbg(&kairos->spi->dev, "%s reg %d.%02X value %04X --> %02X %02X %02X %02X\n", __func__, module, reg, data, kairos->tx_buffer[0], kairos->tx_buffer[1], kairos->tx_buffer[2], kairos->tx_buffer[3]);

	spi_message_init(&m);

	k_tmp = k_xfers;
	k_tmp->tx_buf	= kairos->tx_buffer;
	k_tmp->len		= 4;
	k_tmp->speed_hz	= kairos->speed_hz;

#if 1
	spi_message_add_tail(k_tmp, &m);

	status = kairos_sync(kairos, &m);

	kfree(k_xfers);

	if (status <= 0)
		return -EINVAL;
#endif

	return 0;
}

void kairos_build_date(struct kairos_data* kairos)
{
	u32 build_date;
	u16 tmp;

	tmp = 0x0000;
	kairos_reg_read(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_BDTHB, &tmp);
	build_date = tmp;

	tmp = 0x0000;
	kairos_reg_read(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_BDTLB, &tmp);
	build_date = (build_date << 16) | tmp;

    dev_info(&kairos->spi->dev, "Build date: %d/%d/%d %d:%d:%d\n",
           GET_YEAR(build_date), GET_MONTH(build_date), GET_DAY(build_date),
           GET_HOUR(build_date), GET_MIN(build_date), GET_SEC(build_date)); 
}

void kairos_fw_release(struct kairos_data* kairos)
{
    uint32_t release;
    uint16_t tmp;

	tmp = 0x0000;
    kairos_reg_read(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_RELHB, &tmp);
    release = tmp;

	tmp = 0x0000;
    kairos_reg_read(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_RELLB, &tmp);
    release = (release << 16) | tmp;

    dev_info(&kairos->spi->dev, "Release: %d.%d.%d - %d\n",
           GET_MAJOR(release), GET_MINOR(release), GET_BUGFIX(release), GET_BSTEP(release));
} 

u16 kairos_status(struct kairos_data* kairos)
{
	u16 data;

	data = 0x0001;
	kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_TR_CTRL, &data);
    dev_info(&kairos->spi->dev, "Current status: %04X\n", data); 

    return data;
}

static const u8 COUNTER_BASE_ADDRS[] = 
{
    0x00,
    0x01,
    0x02,
    0x03,
    0x04,
    0x05,
    0x06,
    0x07,
    0x08,
    0x09,
    0x0A,
    0x0B,
    0x0C,
    0x0D,
    0x0E,
    0x0F,
    0x10,
    0x11,
    0x12,
    0x13,
    0x14,
    0x15,
    0x16,
    0x17,
    0x18,
    0x19,
    0x1A,
    0x1B,
    0x1C,
    0x1D,
    0x1E,
    0x1F,
	0x21,
    0x22,
    0x23,
    0x25,
    0x26,
    0x27,
    0x28,
    0x29,
    0x2A,
    0x2B,
    0x2C,
    0x2D,
    0x2E,
    0x2F,
    0x30,
    0x31,
    0x32,
    0x33,
    0x34,
    0x35,
    0x36,
    0x37,
    0x38,
    0x39,
    0x3A,
    0x3B,
    0x3C,
    0x3D,
    0x3E,
    0x3F
}; 

int kairos_reg_counter(struct kairos_data* kairos, int port, int counter, u32* value)
{
    int result;
    u8 csel;
    u16 counterL;
    u16 counterH;
    u32 counterLong;

	//AG TODO
    if (counter >= sizeof(COUNTER_BASE_ADDRS))
    	return 0;

    // compute counter select address
    csel = COUNTER_BASE_ADDRS[counter] + (port * 0x40);

	mutex_lock(&kairos->buf_lock);

    result = _kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_CSEL, csel);
    if (result != 0)
    {
		mutex_unlock(&kairos->buf_lock);
        return result;
    }

    counterL = 0x0000;
    result = _kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_CRDL, &counterL);
    if (result != 0)
    {		
    	mutex_unlock(&kairos->buf_lock);
        return result;
    }

    counterH = 0x0000;
    result = _kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_CRDH, &counterH);
    if (result != 0)
    {
		mutex_unlock(&kairos->buf_lock);
        return result;
    }

	mutex_unlock(&kairos->buf_lock);

    counterLong = counterH;
    counterLong = (counterLong << 16) | counterL;

    *value = counterLong;

    return KAIROS_ERR_OK;
} 

int kairos_check_enabled(struct kairos_data* kairos)
{
    int i;
    u16 data;

    for (i=0; i<2; i++)
    {
        usleep_range(1000, 1000);

        data = 0x0001;
        if (KAIROS_ERR_OK != kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_TR_CTRL, &data))
        {
            return -1;
        }

        /* if everything is OK */
	    if (0 == (data & TSN_CTRL_TRANSIT_MASK))
            break;
    }

    if (0 == (data & TSN_CTRL_READY_MASK))
    {
		dev_info(&kairos->spi->dev, "%s, FPGA not ready [reg 5]: 0x%x\r\n", __func__, data);
    }

 	if (TSN_CTRL_TRANSIT_MASK == (data & TSN_CTRL_TRANSIT_MASK))
 	{
		dev_info(&kairos->spi->dev, "%s, FPGA transition error [reg 5]: 0x%x\r\n", __func__, data);
    }

    if (TSN_CTRL_SPEED_MISM_MASK == (data & TSN_CTRL_SPEED_MISM_MASK))
    {
		dev_info(&kairos->spi->dev, "%s, FPGA detected speed mismatch [reg 5]: 0x%x\r\n", __func__, data);
    }

    return 1;
} 

static const u8 TIMESTAMP_BASE_ADDRS[] = 
{
	0x20,
	0x1E,
	0x28,
	0x26
};

int kairos_read_timestamp(struct kairos_data* kairos, u8 port, u8 rx, ktime_t* ts)
{
	u16 secH, secM, secL;
	u16 nsecH, nsecL;

	u64 secs = 0;
	u64 nsecs = 0;

	u16* data[5] = { &secH, &secM, &secL, &nsecH, &nsecL };
	u16 tmp;

	u8 reg = TIMESTAMP_BASE_ADDRS[(port*2) + rx];

	int i;
	int result;

	mutex_lock(&kairos->buf_lock);

	// read status register
	_kairos_reg_read(kairos, KAIROS_MODULE_PTP, reg-1, &tmp);
	//dev_info(&kairos->spi->dev, "%s [reg %d]: 0x%04x\r\n", __func__, reg-1, tmp);

	if (tmp & 0x0004)
	{
		for (i=0; i<5; i++)
		{
			result = _kairos_reg_read(kairos, KAIROS_MODULE_PTP, reg, data[i]);
			if (result != KAIROS_ERR_OK)
			{
				mutex_unlock(&kairos->buf_lock);
				return result;
			}

			//dev_info(&kairos->spi->dev, "%s [reg %d]: 0x%04x\r\n", __func__, reg, *data[i]);
		}

		secs = secH;
		secs = (secs << 16) | secM;
		secs = (secs << 16) | secL;

		nsecs = nsecH;
		nsecs = (nsecs << 16) | nsecL;
	}
	else
	{
		dev_err(&kairos->spi->dev, "%s [reg %d]: timestamp not available (0x%04X)\r\n", __func__, reg-1, tmp);
	}

	mutex_unlock(&kairos->buf_lock);
	*ts = ktime_set(secs, nsecs);

	return KAIROS_ERR_OK;
}
