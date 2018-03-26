/* 2015(C) G. Pavoni Exor S.p.a.
 *
 * 2015(C) Marco Cavallini - KOAN sas - RS485 support - set in DT the RTS GPIO
 * Based on the patch by Aurelien Bouin
 * 
 *  Driver for Motorola IMX serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Author: Sascha Hauer <sascha@saschahauer.de>
 *  Copyright (C) 2004 Pengutronix
 *
 *  Copyright (C) 2009 emlix GmbH
 *  Author: Fabian Godehardt (added IrDA support for iMX)
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * [29-Mar-2005] Mike Lee
 * Added hardware handshake
 */

#if defined(CONFIG_SERIAL_IMX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

// #define SCNK_DRIVER  ----> (CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
// #define SCNK_USING_HRTIMER

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>

#include <asm/irq.h>
#include <linux/platform_data/serial-imx.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/plxx_manager.h>

#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
#include <linux/ktime.h>
#ifdef SCNK_USING_HRTIMER
#include <linux/hrtimer.h>
#endif
#endif

/* Register definitions */
#define URXD0 0x0  /* Receiver Register */
#define URTX0 0x40 /* Transmitter Register */
#define UCR1  0x80 /* Control Register 1 */
#define UCR2  0x84 /* Control Register 2 */
#define UCR3  0x88 /* Control Register 3 */
#define UCR4  0x8c /* Control Register 4 */
#define UFCR  0x90 /* FIFO Control Register */
#define USR1  0x94 /* Status Register 1 */
#define USR2  0x98 /* Status Register 2 */
#define UESC  0x9c /* Escape Character Register */
#define UTIM  0xa0 /* Escape Timer Register */
#define UBIR  0xa4 /* BRM Incremental Register */
#define UBMR  0xa8 /* BRM Modulator Register */
#define UBRC  0xac /* Baud Rate Count Register */
#define IMX21_ONEMS 0xb0 /* One Millisecond register */
#define IMX1_UTS 0xd0 /* UART Test Register on i.mx1 */
#define IMX21_UTS 0xb4 /* UART Test Register on all other i.mx*/

/* UART Control Register Bit Fields.*/
#define URXD_DUMMY_READ (1<<16)
#define URXD_CHARRDY	(1<<15)
#define URXD_ERR	(1<<14)
#define URXD_OVRRUN	(1<<13)
#define URXD_FRMERR	(1<<12)
#define URXD_BRK	(1<<11)
#define URXD_PRERR	(1<<10)
#define URXD_RX_DATA    (0xFF<<0)
#define UCR1_ADEN	(1<<15) /* Auto detect interrupt */
#define UCR1_ADBR	(1<<14) /* Auto detect baud rate */
#define UCR1_TRDYEN	(1<<13) /* Transmitter ready interrupt enable */
#define UCR1_IDEN	(1<<12) /* Idle condition interrupt */
#define UCR1_ICD_REG(x) (((x) & 3) << 10) /* idle condition detect */
#define UCR1_RRDYEN	(1<<9)	/* Recv ready interrupt enable */
#define UCR1_RDMAEN	(1<<8)	/* Recv ready DMA enable */
#define UCR1_IREN	(1<<7)	/* Infrared interface enable */
#define UCR1_TXMPTYEN	(1<<6)	/* Transimitter empty interrupt enable */
#define UCR1_RTSDEN	(1<<5)	/* RTS delta interrupt enable */
#define UCR1_SNDBRK	(1<<4)	/* Send break */
#define UCR1_TDMAEN	(1<<3)	/* Transmitter ready DMA enable */
#define IMX1_UCR1_UARTCLKEN (1<<2) /* UART clock enabled, i.mx1 only */
#define UCR1_ATDMAEN    (1<<2)  /* Aging DMA Timer Enable */
#define UCR1_DOZE	(1<<1)	/* Doze */
#define UCR1_UARTEN	(1<<0)	/* UART enabled */
#define UCR2_ESCI	(1<<15)	/* Escape seq interrupt enable */
#define UCR2_IRTS	(1<<14)	/* Ignore RTS pin */
#define UCR2_CTSC	(1<<13)	/* CTS pin control */
#define UCR2_CTS	(1<<12)	/* Clear to send */
#define UCR2_ESCEN	(1<<11)	/* Escape enable */
#define UCR2_PREN	(1<<8)	/* Parity enable */
#define UCR2_PROE	(1<<7)	/* Parity odd/even */
#define UCR2_STPB	(1<<6)	/* Stop */
#define UCR2_WS		(1<<5)	/* Word size */
#define UCR2_RTSEN	(1<<4)	/* Request to send interrupt enable */
#define UCR2_ATEN	(1<<3)	/* Aging Timer Enable */
#define UCR2_TXEN	(1<<2)	/* Transmitter enabled */
#define UCR2_RXEN	(1<<1)	/* Receiver enabled */
#define UCR2_SRST	(1<<0)	/* SW reset */
#define UCR3_DTREN	(1<<13) /* DTR interrupt enable */
#define UCR3_PARERREN	(1<<12) /* Parity enable */
#define UCR3_FRAERREN	(1<<11) /* Frame error interrupt enable */
#define UCR3_DSR	(1<<10) /* Data set ready */
#define UCR3_DCD	(1<<9)	/* Data carrier detect */
#define UCR3_RI		(1<<8)	/* Ring indicator */
#define UCR3_ADNIMP	(1<<7)	/* Autobaud Detection Not Improved */
#define UCR3_RXDSEN	(1<<6)	/* Receive status interrupt enable */
#define UCR3_AIRINTEN	(1<<5)	/* Async IR wake interrupt enable */
#define UCR3_AWAKEN	(1<<4)	/* Async wake interrupt enable */
#define IMX21_UCR3_RXDMUXSEL	(1<<2)	/* RXD Muxed Input Select */
#define UCR3_INVT	(1<<1)	/* Inverted Infrared transmission */
#define UCR3_BPEN	(1<<0)	/* Preset registers enable */
#define UCR4_CTSTL_SHF	10	/* CTS trigger level shift */
#define UCR4_CTSTL_MASK	0x3F	/* CTS trigger is 6 bits wide */
#define UCR4_INVR	(1<<9)	/* Inverted infrared reception */
#define UCR4_ENIRI	(1<<8)	/* Serial infrared interrupt enable */
#define UCR4_WKEN	(1<<7)	/* Wake interrupt enable */
#define UCR4_REF16	(1<<6)	/* Ref freq 16 MHz */
#define UCR4_IDDMAEN    (1<<6)  /* DMA IDLE Condition Detected */
#define UCR4_IRSC	(1<<5)	/* IR special case */
#define UCR4_TCEN	(1<<3)	/* Transmit complete interrupt enable */
#define UCR4_BKEN	(1<<2)	/* Break condition interrupt enable */
#define UCR4_OREN	(1<<1)	/* Receiver overrun interrupt enable */
#define UCR4_DREN	(1<<0)	/* Recv data ready interrupt enable */
#define UFCR_RXTL_SHF	0	/* Receiver trigger level shift */
#define UFCR_DCEDTE	(1<<6)	/* DCE/DTE mode select */
#define UFCR_RFDIV	(7<<7)	/* Reference freq divider mask */
#define UFCR_RFDIV_REG(x)	(((x) < 7 ? 6 - (x) : 6) << 7)
#define UFCR_TXTL_SHF	10	/* Transmitter trigger level shift */
#define USR1_PARITYERR	(1<<15) /* Parity error interrupt flag */
#define USR1_RTSS	(1<<14) /* RTS pin status */
#define USR1_TRDY	(1<<13) /* Transmitter ready interrupt/dma flag */
#define USR1_RTSD	(1<<12) /* RTS delta */
#define USR1_ESCF	(1<<11) /* Escape seq interrupt flag */
#define USR1_FRAMERR	(1<<10) /* Frame error interrupt flag */
#define USR1_RRDY	(1<<9)	 /* Receiver ready interrupt/dma flag */
#define USR1_TIMEOUT	(1<<7)	 /* Receive timeout interrupt status */
#define USR1_RXDS	 (1<<6)	 /* Receiver idle interrupt flag */
#define USR1_AIRINT	 (1<<5)	 /* Async IR wake interrupt flag */
#define USR1_AWAKE	 (1<<4)	 /* Aysnc wake interrupt flag */
#define USR2_ADET	 (1<<15) /* Auto baud rate detect complete */
#define USR2_TXFE	 (1<<14) /* Transmit buffer FIFO empty */
#define USR2_DTRF	 (1<<13) /* DTR edge interrupt flag */
#define USR2_IDLE	 (1<<12) /* Idle condition */
#define USR2_IRINT	 (1<<8)	 /* Serial infrared interrupt flag */
#define USR2_WAKE	 (1<<7)	 /* Wake */
#define USR2_RTSF	 (1<<4)	 /* RTS edge interrupt flag */
#define USR2_TXDC	 (1<<3)	 /* Transmitter complete */
#define USR2_BRCD	 (1<<2)	 /* Break condition */
#define USR2_ORE	(1<<1)	 /* Overrun error */
#define USR2_RDR	(1<<0)	 /* Recv data ready */
#define UTS_FRCPERR	(1<<13) /* Force parity error */
#define UTS_LOOP	(1<<12)	 /* Loop tx and rx */
#define UTS_TXEMPTY	 (1<<6)	 /* TxFIFO empty */
#define UTS_RXEMPTY	 (1<<5)	 /* RxFIFO empty */
#define UTS_TXFULL	 (1<<4)	 /* TxFIFO full */
#define UTS_RXFULL	 (1<<3)	 /* RxFIFO full */
#define UTS_SOFTRST	 (1<<0)	 /* Software reset */

/* We've been assigned a range on the "Low-density serial ports" major */
#define SERIAL_IMX_MAJOR	207
#define MINOR_START		16
#define DEV_NAME		"ttymxc"
#define DRIVER_VERSION		"1.1"

#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
#define REQ_RPT		0x00
#define REQ_LANG	0x06
#define RES_LANG	0x04
#define REQ_KURZ	0x0E
#define RES_KURZ	0x0C
#define REQ_STAT	0x12
#define RES_STAT	0x14
#define REQ_VARI	0x18
#define REQ_PROZ	0x1C

#define PROZ_INFO	0x80
#define PROZ_INFO_CRC	0xC1
#define PROZ_INFO_NOCRC	0x81
#define diag_eot_cnt (sport->SCNKdata.diag_cnt[0])
#define diag_req_cnt (sport->SCNKdata.diag_cnt[1])
#define diag_err_cnt (sport->SCNKdata.diag_cnt[2])
#define diag_tot_cnt (sport->SCNKdata.diag_cnt[3])
#define diag_sts_cnt (sport->SCNKdata.diag_cnt[4])
#define diag_prz_cnt (sport->SCNKdata.diag_cnt[5])
#define diag_fus_cnt (sport->SCNKdata.diag_cnt[6])
#define diag_yyy_cnt (sport->SCNKdata.diag_cnt[7])

struct s_SCNKparams {
	unsigned char unitID;
	unsigned char inBufLen;
	unsigned char outBufLen;
	unsigned short applTimeout;
	unsigned short manufID;
};

struct s_SCNKdata {
	unsigned char unitID;
	unsigned char inBufLen;
	unsigned char outBufLen;
	unsigned short SCNKstatus;
	unsigned char statMsg[5];
	unsigned char infoMsg[12];
	unsigned char outBufMsg1[84];
	unsigned char outBufMsg2[84];
	bool useTxBuf2;
	bool useCRC;
	unsigned int diag_cnt[8];
	int gapTime;
	unsigned char localBuf[UART_XMIT_SIZE];
	struct circ_buf txBuf;
	bool SCNKenabled;
	int expectedLen;
	int rxLen;
	unsigned char rxBuf[84];
	int lastTxLen;
	unsigned char lastTxBuf[84];
	unsigned char lastRecvJob;
	bool pendingReq;
	struct timespec lastCycle;
#ifdef SCNK_USING_HRTIMER
	struct hrtimer hrt;
#endif
};
#define SET_SCNK_MODE 0x54FF
#define GET_SCNK_DIAG 0x54FE
#define SET_SCNK_DIAG 0x54FD
#define SET_SCNK_PREQ 0x54FC
#define GET_SCNK_SREQ 0x54FB
#define TOG_SCNK_BAUD 0x54FA

//#define SCNK_DEBUG KERN_DEBUG

#endif
/*
 * This determines how often we check the modem status signals
 * for any change.  They generally aren't connected to an IRQ
 * so we have to poll them.  We also check immediately before
 * filling the TX fifo incase CTS has been dropped.
 */
#define MCTRL_TIMEOUT	(250*HZ/1000)

#define DRIVER_NAME "IMX-uart"

#define UART_NR 8

/* i.mx21 type uart runs on all i.mx except i.mx1 */
enum imx_uart_type {
	IMX1_UART,
	IMX21_UART,
	IMX6Q_UART,
};

/* device type dependent stuff */
struct imx_uart_data {
	unsigned uts_reg;
	enum imx_uart_type devtype;
};

struct imx_port {
	struct uart_port	port;
	struct timer_list	timer;
	unsigned int		old_status;
	unsigned int		have_rtscts:1;
	unsigned int		dte_mode:1;
	unsigned int		irda_inv_rx:1;
	unsigned int		irda_inv_tx:1;
	unsigned short		trcv_delay; /* transceiver delay */
	struct clk		*clk_ipg;
	struct clk		*clk_per;
	const struct imx_uart_data *devdata;

	/* DMA fields */
	unsigned int		dma_is_inited:1;
	unsigned int		dma_is_enabled:1;
	unsigned int		dma_is_rxing:1;
	unsigned int		dma_is_txing:1;
	struct dma_chan		*dma_chan_rx, *dma_chan_tx;
         struct scatterlist      rx_sgl, tx_sgl[2];
         void                    *rx_buf;
	unsigned int		tx_bytes;
	unsigned int		dma_tx_nents;
	wait_queue_head_t	dma_wait;
	int			rts_gpio;         /* GPIO used as tx_en line for RS485 operation */
	int			be15mode_gpio;    /* GPIO used as mode selection between RS485 (1) and RS422 (0) on BE15 carriers */
	int 			mode_gpio;        /* If a valid gpio is mapped here, it means we have a programmable RS485/RS232 phy */
	int                     rxen_gpio;        /* If a valid gpio is mapped here, we will use it for disabling the RX echo while in RS485 mode */
	unsigned int            is_plugin_module; /* If set, indicates the uart lines are routed to a plugin module slot, so control signals are handled accordingly */
	struct 			serial_rs485 rs485;
	struct platform_device* plugin1dev;
	struct platform_device* plugin2dev;
	int                     mode_two_lines_only;
#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
	struct s_SCNKparams SCNKparams;
	struct s_SCNKdata SCNKdata;
#endif
};

struct imx_port_ucrs {
	unsigned int	ucr1;
	unsigned int	ucr2;
	unsigned int	ucr3;
};

#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
static void addCrc(unsigned char b, unsigned short *crc)
{
	unsigned char cy;
	//bit0
	cy = (*crc & 1);
	*crc >>= 1;
	if ((b & 1) ^ cy)
		*crc ^= 0xA001;
	//bit1
	cy = (*crc & 1) << 1;
	*crc >>= 1;
	if ((b & 2) ^ cy)
		*crc ^= 0xA001;
	//bit2
	cy = (*crc & 1) << 2;
	*crc >>= 1;
	if ((b & 4) ^ cy)
		*crc ^= 0xA001;
	//bit3
	cy = (*crc & 1) << 3;
	*crc >>= 1;
	if ((b & 8) ^ cy)
		*crc ^= 0xA001;
	//bit4
	cy = (*crc & 1) << 4;
	*crc >>= 1;
	if ((b & 0x10) ^ cy)
		*crc ^= 0xA001;
	//bit5
	cy = (*crc & 1) << 5;
	*crc >>= 1;
	if ((b & 0x20) ^ cy)
		*crc ^= 0xA001;
	//bit6
	cy = (*crc & 1) << 6;
	*crc >>= 1;
	if ((b & 0x40) ^ cy)
		*crc ^= 0xA001;
	//bit7
	cy = (*crc & 1) << 7;
	*crc >>= 1;
	if ((b & 0x80) ^ cy)
		*crc ^= 0xA001;
}

static void setSCNKTxData(unsigned char *pMsg, int len, bool useCRC)
{
	int i;
	if (useCRC)
	{
		unsigned short crc = 0xffff;
		addCrc(1, &crc);
		addCrc(pMsg[2], &crc);
		for (i = 0; i < len-6; i++)
			addCrc(pMsg[3+i], &crc);
		pMsg[len-2] = crc >> 8;
		pMsg[len-3] = crc & 0xFF;
		for (i = 0; i < len-1; i++)
			pMsg[len-1] ^= pMsg[i];
	}
	else
		for (i = 0; i < len-3; i++)
			pMsg[len-3] ^= pMsg[i];
}				struct timespec now;


#ifdef SCNK_USING_HRTIMER
void startSCNKtx(struct imx_port *sport)
{
	//enable transmission after a delay
	unsigned long temp;
#ifdef SCNK_DEBUG
//	dev_dbg(sport->port.dev, "<<<<<<<<< SCNK HRTcallback for uart %d >>>>>>>>>>\n", sport->port.line);
#endif
	imx_rs485_start_tx(sport);
	/* enable transmitter and shifter empty irq */
	temp = readl(((struct uart_port *)sport)->membase + UCR4);
	temp |= UCR4_TCEN;
	writel(temp, ((struct uart_port *)sport)->membase + UCR4);
}

struct imx_port *sportArray[UART_NR];
enum hrtimer_restart hrtCallback_0(struct hrtimer *phrt){startSCNKtx(sportArray[0]);return HRTIMER_NORESTART;}
enum hrtimer_restart hrtCallback_1(struct hrtimer *phrt){startSCNKtx(sportArray[1]);return HRTIMER_NORESTART;}
enum hrtimer_restart hrtCallback_2(struct hrtimer *phrt){startSCNKtx(sportArray[2]);return HRTIMER_NORESTART;}
enum hrtimer_restart hrtCallback_3(struct hrtimer *phrt){startSCNKtx(sportArray[3]);return HRTIMER_NORESTART;}
enum hrtimer_restart hrtCallback_4(struct hrtimer *phrt){startSCNKtx(sportArray[4]);return HRTIMER_NORESTART;}
enum hrtimer_restart hrtCallback_5(struct hrtimer *phrt){startSCNKtx(sportArray[5]);return HRTIMER_NORESTART;}
enum hrtimer_restart hrtCallback_6(struct hrtimer *phrt){startSCNKtx(sportArray[6]);return HRTIMER_NORESTART;}
enum hrtimer_restart hrtCallback_7(struct hrtimer *phrt){startSCNKtx(sportArray[7]);return HRTIMER_NORESTART;}

//void *hrtCallBackArray[UART_NR] = {&hrtCallback_0,
enum hrtimer_restart (*hrtCallBackArray[UART_NR] )(struct hrtimer *) = {&hrtCallback_0,
								   &hrtCallback_1,
								   &hrtCallback_2,
								   &hrtCallback_3,
								   &hrtCallback_4,
								   &hrtCallback_5,
								   &hrtCallback_6,
								   &hrtCallback_7};
#endif
#endif

static void imx_rs485_stop_tx(struct imx_port *sport)
{
	if ((sport->rts_gpio >= 0) && (sport->rs485.flags & SER_RS485_ENABLED))
	  if ((sport->rs485.flags & SER_RS485_RTS_AFTER_SEND) == 0)
	    gpio_set_value(sport->rts_gpio, 0);
	
	if(sport->is_plugin_module == 1)
	  if ((sport->rts_gpio < 0) && (sport->rs485.flags & SER_RS485_ENABLED))
	    writel(readl(sport->port.membase + UCR2) & ~UCR2_CTS, sport->port.membase + UCR2);
	
	if ((sport->rs485.flags & SER_RS485_ENABLED) && !(sport->rs485.flags & SER_RS485_RX_DURING_TX)) 
	{
	  //RX enable by using the prg phy dedicated gpio pin
	  if (gpio_is_valid(sport->rxen_gpio)) 
	    gpio_set_value(sport->rxen_gpio, 1);
	}
}

static void imx_rs485_start_tx(struct imx_port *sport)
{
	if ((sport->rs485.flags & SER_RS485_ENABLED) && !(sport->rs485.flags & SER_RS485_RX_DURING_TX))
	{
	  //RX disable by using the prg phy dedicated gpio pin 
	  if (gpio_is_valid(sport->rxen_gpio)) 
	    gpio_set_value(sport->rxen_gpio, 0);
	}
  
	if ((sport->rts_gpio >= 0) && (sport->rs485.flags & SER_RS485_ENABLED))
		gpio_set_value(sport->rts_gpio, 1);
	
	if(sport->is_plugin_module == 1)
	  if ((sport->rts_gpio < 0) && (sport->rs485.flags & SER_RS485_ENABLED))
	    writel(readl(sport->port.membase + UCR2) | UCR2_CTS, sport->port.membase + UCR2);
	  
}

void imx_config_rs485(struct imx_port *sport)
{

    dev_dbg(sport->port.dev, "%s -> (MCK) \n", __func__);
	dev_dbg(sport->port.dev, "--------- Setting UART /dev/ttymxc%d mode...\n", sport->port.line);
	dev_dbg(sport->port.dev, "--------- SER_RS485_ENABLED=%d \n", (sport->rs485.flags & SER_RS485_ENABLED));

	if (sport->rts_gpio >= 0)
	{
  	  gpio_set_value(sport->rts_gpio, 0);
	  if (sport->rs485.flags & SER_RS485_ENABLED)
	  {
	    if(sport->rs485.flags & SER_RS485_RX_DURING_TX)
	    {
		dev_dbg(sport->port.dev, "Setting UART to RS422\n");
		if(sport->be15mode_gpio >= 0)
		  gpio_set_value(sport->be15mode_gpio, 0);
	    }
	    else
	    {
		dev_dbg(sport->port.dev, "Setting UART to RS485\n");
		if(sport->be15mode_gpio >= 0)
		  gpio_set_value(sport->be15mode_gpio, 1);
	    }
	  }
	  else
	  {
		dev_dbg(sport->port.dev, "Setting UART to RS232\n");
		/*
		* If we are in RS232 mode and we have a programmable phy, enable the TX if not yet done.
		*/
		if (gpio_is_valid(sport->mode_gpio))
		  gpio_set_value(sport->rts_gpio, 1);
	  }
	}
	
	// If we have a programmable phy, set the mode accordingly
	if (gpio_is_valid(sport->mode_gpio)) 
	{
	  if(sport->rs485.flags & SER_RS485_ENABLED)
	    gpio_set_value(sport->mode_gpio, 1);
	  else
	    gpio_set_value(sport->mode_gpio, 0);
	}

	//RX enable by using the prg phy dedicated gpio pin 
	if (gpio_is_valid(sport->rxen_gpio)) 
	  gpio_set_value(sport->rxen_gpio, 1);
	
	if (sport->have_rtscts) 
	{
		dev_dbg(sport->port.dev, "UART have RTS/CTS\n");
		if (sport->rs485.flags & SER_RS485_ENABLED)
		  writel(readl(sport->port.membase + UCR2) & ~UCR2_CTSC, sport->port.membase + UCR2);
		
		if(sport->is_plugin_module == 1)
		  if ((sport->rts_gpio < 0) && (sport->rs485.flags & SER_RS485_ENABLED))
		    writel(readl(sport->port.membase + UCR2) & ~UCR2_CTS, sport->port.membase + UCR2);
	} 
	else
	{
		dev_dbg(sport->port.dev, "UART does not have RTS/CTS... RS485 mode not possible\n");
		sport->rs485.flags &= ~SER_RS485_ENABLED;
	}
}

// (MCK)
static int imx_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct imx_port *sport = (struct imx_port *)port;

	switch (cmd) {
		case TIOCSRS485:
			if (copy_from_user(&(sport->rs485), (struct serial_rs485 *) arg, sizeof(struct serial_rs485)))
					return -EFAULT;
			imx_config_rs485(sport);

			//Set duplex mode for RS485/422 plugin modules, according with RS485 or RS422 mode set
			if(sport->is_plugin_module == 1)
			  if (sport->rs485.flags & SER_RS485_ENABLED)
				if (!gpio_is_valid(sport->rxen_gpio))
				{
				  unsigned int cmd;

				  if(sport->rs485.flags & SER_RS485_RX_DURING_TX)
				cmd = RS422_485_IF_SETFD;
				  else
				cmd = RS422_485_IF_SETHD;
#if defined(CONFIG_CONFIG_PLXX_MANAGER) || defined(CONFIG_CONFIG_PLXX_MANAGER_MODULE)
				  if(sport->plugin1dev)
				plxx_manager_sendcmd(sport->plugin1dev , cmd);
				  if(sport->plugin2dev)
				plxx_manager_sendcmd(sport->plugin2dev , cmd);
#endif
				}

			break;

		case TIOCGRS485:
			if (copy_to_user((struct serial_rs485 *) arg, &(sport->rs485), sizeof(struct serial_rs485)))
					return -EFAULT;
			dev_info(sport->port.dev, "Getting RS485 parameters for the device\n");
			break;

#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
		case SET_SCNK_MODE:
			if (copy_from_user(&(sport->SCNKparams), (struct s_SCNKparams *) arg, sizeof(sport->SCNKparams)))
				return -EFAULT;
			sport->SCNKdata.useTxBuf2 = false;
			sport->SCNKdata.useCRC = false;
			memset (sport->SCNKdata.outBufMsg1, 0, sizeof(sport->SCNKdata.outBufMsg1));
			sport->SCNKdata.outBufMsg1[0] = 0x01;
			sport->SCNKdata.outBufMsg1[1] = 0x18;
			sport->SCNKdata.outBufMsg1[2] = sport->SCNKparams.outBufLen;
			setSCNKTxData(sport->SCNKdata.outBufMsg1,sport->SCNKparams.outBufLen+6, false);
			memset (sport->SCNKdata.outBufMsg2, 0, sizeof(sport->SCNKdata.outBufMsg1));
			sport->SCNKdata.outBufMsg2[0] = 0x01;
			sport->SCNKdata.outBufMsg2[1] = 0x18;
			sport->SCNKdata.outBufMsg2[2] = sport->SCNKparams.outBufLen;
			setSCNKTxData(sport->SCNKdata.outBufMsg2,sport->SCNKparams.outBufLen+6, false);

			sport->SCNKdata.statMsg[0] = 0x01;
			sport->SCNKdata.statMsg[1] = 0x14;
			sport->SCNKdata.SCNKstatus = 0x8006;
			sport->SCNKdata.infoMsg[0] = 0x01;
			sport->SCNKdata.infoMsg[1] = 0x18;
			sport->SCNKdata.infoMsg[2] = 0x08;
			sport->SCNKdata.infoMsg[3] = sport->SCNKparams.manufID >> 8;
			sport->SCNKdata.infoMsg[4] = sport->SCNKparams.manufID & 0xFF;
			memset (sport->SCNKdata.diag_cnt, 0, sizeof(sport->SCNKdata.diag_cnt));
			sport->SCNKdata.txBuf.buf = sport->SCNKdata.localBuf;
			sport->SCNKdata.txBuf.head = sport->SCNKdata.txBuf.tail = 0;
			sport->SCNKdata.expectedLen = 3;
			sport->SCNKdata.rxLen = 0;
			sport->SCNKdata.lastRecvJob = 0;
			sport->SCNKdata.gapTime = 180000;
			sport->SCNKdata.pendingReq = false;
#ifdef SCNK_USING_HRTIMER
			sportArray[sport->port.line] = sport;
			hrtimer_init(&sport->SCNKdata.hrt, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
#ifdef SCNK_DEBUG
//			dev_dbg(sport->port.dev, SCNK_DEBUG "<<<<<<<<< SCNK HRTinit function = %X on uart %d >>>>>>>>>>\n", (unsigned int)(hrtCallBackArray[sport->port.line]), sport->port.line);
#endif
			sport->SCNKdata.hrt.function = hrtCallBackArray[sport->port.line];
#endif
			getrawmonotonic(&sport->SCNKdata.lastCycle);
			sport->SCNKdata.SCNKenabled = true;
#ifdef SCNK_DEBUG
			dev_dbg(sport->port.dev, "<<<<<<<<< SCNK Mode activated on port:%d unitID=%d inL:%d outL:%d MANUF:%X >>>>>>>>>>\n",
				   sport->port.line,
				   sport->SCNKparams.unitID,
				   sport->SCNKparams.inBufLen,
				   sport->SCNKparams.outBufLen,
				   sport->SCNKparams.manufID);
#endif
			break;

		case GET_SCNK_DIAG:
			if (copy_to_user((unsigned int *)arg, &(sport->SCNKdata.diag_cnt), sizeof(sport->SCNKdata.diag_cnt)))
				return -EFAULT;
			break;

		case SET_SCNK_DIAG:
			{
				unsigned int tmp[2];
				if (copy_from_user(tmp, (unsigned int*) arg, sizeof(tmp)))
					return -EFAULT;
				if (tmp[0] < (sizeof(sport->SCNKdata.diag_cnt)/sizeof(unsigned int)))
					sport->SCNKdata.diag_cnt[tmp[0]] = tmp[1];
			}
			break;
		case SET_SCNK_PREQ:
			sport->SCNKdata.pendingReq = true;
			break;
		case GET_SCNK_SREQ:
			if (copy_to_user((unsigned int *)arg, &(sport->SCNKdata.pendingReq), sizeof(sport->SCNKdata.pendingReq)))
				return -EFAULT;
			break;
		case TOG_SCNK_BAUD:
			{
				bool is375 = false;
				if (copy_from_user(&is375, (unsigned int*) arg, sizeof(is375)))
					return -EFAULT;
				if (is375)
					sport->SCNKdata.gapTime = 90000;
				else
					sport->SCNKdata.gapTime = 180000;
			}
			break;
#endif
		default:
			return -ENOIOCTLCMD;
	}
	return 0;
}

/********************************/

static struct imx_uart_data imx_uart_devdata[] = {
	[IMX1_UART] = {
		.uts_reg = IMX1_UTS,
		.devtype = IMX1_UART,
	},
	[IMX21_UART] = {
		.uts_reg = IMX21_UTS,
		.devtype = IMX21_UART,
	},
	[IMX6Q_UART] = {
		.uts_reg = IMX21_UTS,
		.devtype = IMX6Q_UART,
	},
};

 static const struct platform_device_id imx_uart_devtype[] = {
	{
		.name = "imx1-uart",
		.driver_data = (kernel_ulong_t) &imx_uart_devdata[IMX1_UART],
	}, {
		.name = "imx21-uart",
		.driver_data = (kernel_ulong_t) &imx_uart_devdata[IMX21_UART],
	}, {
		.name = "imx6q-uart",
		.driver_data = (kernel_ulong_t) &imx_uart_devdata[IMX6Q_UART],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, imx_uart_devtype);

 static const struct of_device_id imx_uart_dt_ids[] = {
	{ .compatible = "fsl,imx6q-uart", .data = &imx_uart_devdata[IMX6Q_UART], },
	{ .compatible = "fsl,imx1-uart", .data = &imx_uart_devdata[IMX1_UART], },
	{ .compatible = "fsl,imx21-uart", .data = &imx_uart_devdata[IMX21_UART], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_uart_dt_ids);

static inline unsigned uts_reg(struct imx_port *sport)
{
	return sport->devdata->uts_reg;
}

static inline int is_imx1_uart(struct imx_port *sport)
{
	return sport->devdata->devtype == IMX1_UART;
}

static inline int is_imx21_uart(struct imx_port *sport)
{
	return sport->devdata->devtype == IMX21_UART;
}

static inline int is_imx6q_uart(struct imx_port *sport)
{
	return sport->devdata->devtype == IMX6Q_UART;
}
/*
 * Save and restore functions for UCR1, UCR2 and UCR3 registers
 */
#if defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_SERIAL_IMX_CONSOLE)
static void imx_port_ucrs_save(struct uart_port *port,
			       struct imx_port_ucrs *ucr)
{
	/* save control registers */
	ucr->ucr1 = readl(port->membase + UCR1);
	ucr->ucr2 = readl(port->membase + UCR2);
	ucr->ucr3 = readl(port->membase + UCR3);
}

static void imx_port_ucrs_restore(struct uart_port *port,
				  struct imx_port_ucrs *ucr)
{
	/* restore control registers */
	writel(ucr->ucr1, port->membase + UCR1);
	writel(ucr->ucr2, port->membase + UCR2);
	writel(ucr->ucr3, port->membase + UCR3);
}
#endif

/*
 * Handle any change of modem status signal since we were last called.
 */
static void imx_mctrl_check(struct imx_port *sport)
{
	unsigned int status, changed;

	status = sport->port.ops->get_mctrl(&sport->port);
	changed = status ^ sport->old_status;

	if (changed == 0)
		return;

	sport->old_status = status;

	if (changed & TIOCM_RI)
		sport->port.icount.rng++;
	if (changed & TIOCM_DSR)
		sport->port.icount.dsr++;
	if (changed & TIOCM_CAR)
		uart_handle_dcd_change(&sport->port, status & TIOCM_CAR);
	if (changed & TIOCM_CTS)
		uart_handle_cts_change(&sport->port, status & TIOCM_CTS);

	wake_up_interruptible(&sport->port.state->port.delta_msr_wait);
}

/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void imx_timeout(unsigned long data)
{
	struct imx_port *sport = (struct imx_port *)data;
	unsigned long flags;

	if (sport->port.state) {
		spin_lock_irqsave(&sport->port.lock, flags);
		imx_mctrl_check(sport);
		spin_unlock_irqrestore(&sport->port.lock, flags);

		mod_timer(&sport->timer, jiffies + MCTRL_TIMEOUT);
	}
}

/*
 * interrupts disabled on entry
 */
static void imx_stop_tx(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long temp;
 
	/*
	 * We are maybe in the SMP context, so if the DMA TX thread is running
	 * on other cpu, we have to wait for it to finish.
	 */
	if (sport->dma_is_enabled && sport->dma_is_txing)
		return;

	temp = readl(port->membase + UCR1);
	writel(temp & ~(UCR1_TXMPTYEN | UCR1_TRDYEN), port->membase + UCR1);

	/* In RS485 mode, disable TX in shifter is empty (we are confident here the circular buffer to be empty) */
	if (sport->rs485.flags & SER_RS485_ENABLED &&
		readl(port->membase + USR2) & USR2_TXDC) {
		temp = readl(port->membase + UCR2);
		imx_rs485_stop_tx(sport);
		// Transmit complete interrupt disabled
		temp = readl(port->membase + UCR4);
		temp &= ~UCR4_TCEN;
		writel(temp, port->membase + UCR4);
	}
}

/*
 * interrupts disabled on entry
 */
static void imx_stop_rx(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long temp;

	if (sport->dma_is_enabled && sport->dma_is_rxing) {
		if (sport->port.suspended) {
			dmaengine_terminate_all(sport->dma_chan_rx);
			sport->dma_is_rxing = 0;
		} else {
			return;
		}
	}

	temp = readl(sport->port.membase + UCR2);
	writel(temp & ~UCR2_RXEN, sport->port.membase + UCR2);

	/* disable the `Receiver Ready Interrrupt` */
	temp = readl(sport->port.membase + UCR1);
	writel(temp & ~UCR1_RRDYEN, sport->port.membase + UCR1);
}

/*
 * Set the modem control timer to fire immediately.
 */
static void imx_enable_ms(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;

	mod_timer(&sport->timer, jiffies);
}

 static void imx_dma_tx(struct imx_port *sport);
static inline void imx_transmit_buffer(struct imx_port *sport)
{

	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long temp;
	struct tty_port *ttyport = &sport->port.state->port;
	static unsigned long txfullflag;

#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
	if (sport->SCNKdata.SCNKenabled)
	{
		xmit = &sport->SCNKdata.txBuf;
//#ifdef SCNK_DEBUG
#if 0
		if (xmit->head != xmit->tail)
		{
			int i;
			char tmps[256];
			char tmp[16];
			long int L = (xmit->head-xmit->tail) & (UART_XMIT_SIZE-1);
			dev_dbg(sport->port.dev, "<<<<<<<<< SCNK TX activated: len=%ld job:%X >>>>>>>>>>\n", L, xmit->buf[(xmit->head+3) & (UART_XMIT_SIZE-1)]);
			tmps[0] = 0;
			for (i=0; i<L; i++)
			{
				sprintf(tmp, "%02X,", xmit->buf[(xmit->tail+i) & (UART_XMIT_SIZE-1)]);
				strcat(tmps,tmp);
			}
			dev_dbg(sport->port.dev, "%s\n", tmps);

		}
		else
			dev_dbg(sport->port.dev, "<<<<<<<<< SCNK TX ended >>>>>>>>>>\n");
#endif
	}
#endif
	if (sport->port.x_char) {
		/* Send next char */
		writel(sport->port.x_char, sport->port.membase + URTX0);
                 sport->port.icount.tx++;
                 sport->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
		imx_stop_tx(&sport->port);
		return;
	}
	
	if (sport->dma_is_enabled) {
		/*
		* We've just sent a X-char Ensure the TX DMA is enabled
		* and the TX IRQ is disabled.
		**/
		temp = readl(sport->port.membase + UCR1);
		temp &= ~(UCR1_TXMPTYEN | UCR1_TRDYEN);
		if (sport->dma_is_txing) {
			temp |= UCR1_TDMAEN;
			writel(temp, sport->port.membase + UCR1);
		} else {
			writel(temp, sport->port.membase + UCR1);
			imx_dma_tx(sport);
		}
	}

	if((ttyport->low_latency) && ((txfullflag & UTS_TXFULL) != UTS_TXFULL) )
	{ //Handle the low latency option for REALTIME protocol
		if (!sport->dma_is_enabled)
		{ // Clear the IRTS flag to keep the TX stopped while feedint the TX buffer (if we are not refilling on the fly the current packet)
			temp = readl(sport->port.membase + UCR2);
			temp &= ~UCR2_IRTS;
			writel(temp, sport->port.membase + UCR2);
		}
	}
         
	txfullflag = 0;
	while (!uart_circ_empty(xmit) &&
		   !( (txfullflag = readl(sport->port.membase + uts_reg(sport))) & UTS_TXFULL)) {
		/* send xmit->buf[xmit->tail]
		 * out the port here */
		writel(xmit->buf[xmit->tail], sport->port.membase + URTX0);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		imx_stop_tx(&sport->port);
	
	if(ttyport->low_latency)
	{ //Handle the low latency option for MPI protocol
		if ((txfullflag & UTS_TXFULL) && (!sport->dma_is_enabled) )
		{ 	//If we exited the TXBUFF write loop because of TXFULL, it means we probably need to transmit a chunk which is bugger than the TX FIFO size
			//so we enable the TRDYEN interrupt in order to refill the TX FIFO buffer when the watermark level is reached.
			temp = readl(sport->port.membase + UCR1);
			writel(temp | UCR1_TRDYEN, sport->port.membase + UCR1);
		}

		if (!sport->dma_is_enabled)
		{ // Set the IRTS flag to ignore RTS and start transmission
			temp = readl(sport->port.membase + UCR2);
			temp |= UCR2_IRTS;
			writel(temp, sport->port.membase + UCR2);
		}
	}
}

static void dma_tx_callback(void *data)
{
	struct imx_port *sport = data;
	struct scatterlist *sgl = &sport->tx_sgl[0];
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;
         unsigned long temp;
 
         spin_lock_irqsave(&sport->port.lock, flags);
 
         dma_unmap_sg(sport->port.dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);
 
         temp = readl(sport->port.membase + UCR1);
         temp &= ~UCR1_TDMAEN;
         writel(temp, sport->port.membase + UCR1);
 
         /* update the stat */
         xmit->tail = (xmit->tail + sport->tx_bytes) & (UART_XMIT_SIZE - 1);
         sport->port.icount.tx += sport->tx_bytes;
 
         dev_dbg(sport->port.dev, "we finish the TX DMA.\n");
 
         sport->dma_is_txing = 0;
 
         spin_unlock_irqrestore(&sport->port.lock, flags);
 
         if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
                 uart_write_wakeup(&sport->port);
 
         if (waitqueue_active(&sport->dma_wait)) {
                 wake_up(&sport->dma_wait);
                 dev_dbg(sport->port.dev, "exit in %s.\n", __func__);
                 return;
         }
 
         spin_lock_irqsave(&sport->port.lock, flags);
         if (!uart_circ_empty(xmit) && !uart_tx_stopped(&sport->port))
                 imx_dma_tx(sport);
         spin_unlock_irqrestore(&sport->port.lock, flags);
 }

 static void imx_dma_tx(struct imx_port *sport)
 {
         struct circ_buf *xmit = &sport->port.state->xmit;
         struct scatterlist *sgl = sport->tx_sgl;
         struct dma_async_tx_descriptor *desc;
         struct dma_chan *chan = sport->dma_chan_tx;
         struct device *dev = sport->port.dev;
         unsigned long temp;
         int ret;
 
         if (sport->dma_is_txing)
                 return;
 
         sport->tx_bytes = uart_circ_chars_pending(xmit);
 
         if (xmit->tail < xmit->head) {
                 sport->dma_tx_nents = 1;
                 sg_init_one(sgl, xmit->buf + xmit->tail, sport->tx_bytes);
         } else {
                 sport->dma_tx_nents = 2;
                 sg_init_table(sgl, 2);
                 sg_set_buf(sgl, xmit->buf + xmit->tail,
                                 UART_XMIT_SIZE - xmit->tail);
                 sg_set_buf(sgl + 1, xmit->buf, xmit->head);
         }
 
         ret = dma_map_sg(dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);
         if (ret == 0) {
                 dev_err(dev, "DMA mapping error for TX.\n");
                 return;
         }
         desc = dmaengine_prep_slave_sg(chan, sgl, sport->dma_tx_nents,
                                         DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
         if (!desc) {
                 dma_unmap_sg(dev, sgl, sport->dma_tx_nents,
                              DMA_TO_DEVICE);
                 dev_err(dev, "We cannot prepare for the TX slave dma!\n");
                 return;
         }
         desc->callback = dma_tx_callback;
         desc->callback_param = sport;
 
         dev_dbg(dev, "TX: prepare to send %lu bytes by DMA.\n",
                         uart_circ_chars_pending(xmit));
 
         temp = readl(sport->port.membase + UCR1);
         temp |= UCR1_TDMAEN;
         writel(temp, sport->port.membase + UCR1);
 
         /* fire it */
         sport->dma_is_txing = 1;
         dmaengine_submit(desc);
         dma_async_issue_pending(chan);
         return;
 }
 
/*
 * interrupts disabled on entry
 */
static void imx_start_tx(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long temp;


#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
	if (sport->SCNKdata.SCNKenabled) {
		struct circ_buf *xmit = &sport->port.state->xmit;
		unsigned char *pBuf = (sport->SCNKdata.useTxBuf2)?
									sport->SCNKdata.outBufMsg1:
									sport->SCNKdata.outBufMsg2;
		unsigned char *pBuf1 = pBuf;
#ifdef SCNK_DEBUG
//		dev_dbg(sport->port.dev, "<<<<<<<<< SCNK new request to transmit: len=%ld, job=%X >>>>>>>>>>\n",
//			   (xmit->head-xmit->tail) & (UART_XMIT_SIZE-1),
//			   (unsigned int)(xmit->buf[(xmit->tail+3) & (UART_XMIT_SIZE-1)]));
#endif

		while (!uart_circ_empty(xmit)) {
			*pBuf1++ = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail+1) & (UART_XMIT_SIZE-1);
		}
		setSCNKTxData(pBuf, sport->SCNKparams.outBufLen+6, sport->SCNKdata.useCRC);
		sport->SCNKdata.useTxBuf2 = !sport->SCNKdata.useTxBuf2;
	}
	else
#endif
	{
		if (sport->rs485.flags & SER_RS485_ENABLED) {
			imx_rs485_start_tx(sport);
			/* enable transmitter and shifter empty irq */
			temp = readl(port->membase + UCR4);
			temp |= UCR4_TCEN;
			writel(temp, port->membase + UCR4);
		} else {
			/*
				* If we are in RS232 mode and we have a programmable phy, enable the TX if not yet done.
			*/
			if (gpio_is_valid(sport->mode_gpio))
				if (gpio_is_valid(sport->rts_gpio)) {
					if(sport->mode_two_lines_only) {
						gpio_set_value(sport->rts_gpio, 0);
					} else {
						gpio_set_value(sport->rts_gpio, 1);
					}
				}
		}

		if (!sport->dma_is_enabled) {
			temp = readl(sport->port.membase + UCR1);
			writel(temp | UCR1_TXMPTYEN, sport->port.membase + UCR1);
		}

		if (sport->dma_is_enabled) {
			if (sport->port.x_char) {
				/* We have X-char to send, so enable TX IRQ and
				 * disable TX DMA to let TX interrupt to send X-char */
				temp = readl(sport->port.membase + UCR1);
				temp &= ~UCR1_TDMAEN;
				temp |= UCR1_TXMPTYEN;
				writel(temp, sport->port.membase + UCR1);
				return;
			}

			if (!uart_circ_empty(&port->state->xmit) && !uart_tx_stopped(port))
				imx_dma_tx(sport);
			return;
		}
	}
}

static irqreturn_t imx_rtsint(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);

	writel(USR1_RTSD, sport->port.membase + USR1);
	val = readl(sport->port.membase + USR1) & USR1_RTSS;
	uart_handle_cts_change(&sport->port, !!val);
	wake_up_interruptible(&sport->port.state->port.delta_msr_wait);

	spin_unlock_irqrestore(&sport->port.lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t imx_txint(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	imx_transmit_buffer(sport);
	spin_unlock_irqrestore(&sport->port.lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t imx_rxint(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned int rx, flg, ignored = 0;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags, temp;

	spin_lock_irqsave(&sport->port.lock, flags);

	while (readl(sport->port.membase + USR2) & USR2_RDR) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		rx = readl(sport->port.membase + URXD0);

		temp = readl(sport->port.membase + USR2);
		if (temp & USR2_BRCD) {
			writel(USR2_BRCD, sport->port.membase + USR2);
			if (uart_handle_break(&sport->port))
				continue;
		}

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		if (unlikely(rx & URXD_ERR)) {
			if (rx & URXD_BRK)
				sport->port.icount.brk++;
			else if (rx & URXD_PRERR)
				sport->port.icount.parity++;
			else if (rx & URXD_FRMERR)
				sport->port.icount.frame++;
			if (rx & URXD_OVRRUN)
				sport->port.icount.overrun++;

			if (rx & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}

			rx &= (sport->port.read_status_mask | 0xFF);

			if (rx & URXD_BRK)
				flg = TTY_BREAK;
			else if (rx & URXD_PRERR)
				flg = TTY_PARITY;
			else if (rx & URXD_FRMERR)
				flg = TTY_FRAME;
			if (rx & URXD_OVRRUN)
				flg = TTY_OVERRUN;

#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
#endif
		}

		if (sport->port.ignore_status_mask & URXD_DUMMY_READ)
			goto out;

#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
		if (sport->SCNKdata.SCNKenabled) {
			//SCNK state machine
			unsigned char c;
			int i;
			bool bufferFull=false;
			
#ifdef SCNK_DEBUG
//		dev_dbg(sport->port.dev, "<<<<<<<<< SCNK byte received: %X %X >>>>>>>>>>\n", rx, flg);
#endif
			if (flg != TTY_NORMAL)
			{
				while (readl(sport->port.membase + USR2) & USR2_RDR)	//disregard the rest
					rx = readl(sport->port.membase + URXD0);
				sport->SCNKdata.expectedLen = 2;	//exit
				sport->SCNKdata.rxLen = 0;
			}
			else if (sport->SCNKdata.rxLen < sport->SCNKdata.expectedLen)
			{
				sport->SCNKdata.rxBuf[sport->SCNKdata.rxLen] = (unsigned char)rx;
				sport->SCNKdata.rxLen++;
				if (sport->SCNKdata.rxLen == 2)
				{
					switch((unsigned char)rx & 0x3F)
					{
						case REQ_RPT:
						case REQ_STAT:
							break;
						case REQ_PROZ:
							sport->SCNKdata.expectedLen = 4;
							break;
						case REQ_LANG:
							sport->SCNKdata.expectedLen = 9;	//IO Long
							break;
						case RES_LANG:
							sport->SCNKdata.expectedLen = 10;	//IO Long
							break;
						case REQ_KURZ:
						case RES_KURZ:
							sport->SCNKdata.expectedLen = 5;	//IO Short
							break;
						case RES_STAT:
							sport->SCNKdata.expectedLen = 5;	//STAT messages
							break;
						case REQ_VARI:
							// get length on next byte
							break;
						default:
							while (readl(sport->port.membase + USR2) & USR2_RDR)	//disregard the rest
								rx = readl(sport->port.membase + URXD0);
							sport->SCNKdata.expectedLen = 2;	//exit
							break;
					}
				}
				else if (sport->SCNKdata.rxLen == 3 && (sport->SCNKdata.rxBuf[1] & 0x3F) == REQ_VARI)
					sport->SCNKdata.expectedLen = sport->SCNKdata.rxBuf[2] + (sport->SCNKdata.useCRC?6:4);
			}
			if (sport->SCNKdata.rxLen >= sport->SCNKdata.expectedLen)
			{
#if 0
				struct timespec now;
				getrawmonotonic(&now);
				diag_yyy_cnt = (int)(((long long)now.tv_sec * 1000000 + now.tv_nsec/1000) -
											  ((long long)sport->SCNKdata.lastCycle.tv_sec * 1000000 + sport->SCNKdata.lastCycle.tv_nsec/1000) -
											  4000);	//500 +/- spam of 5ms cycle time
				sport->SCNKdata.lastCycle.tv_sec = now.tv_sec;
				sport->SCNKdata.lastCycle.tv_nsec = now.tv_nsec;
				if ((int)(diag_yyy_cnt) > (int)(diag_fus_cnt))
					diag_fus_cnt = diag_yyy_cnt;
				if (readl(sport->port.membase + USR2) & USR2_RDR)
					diag_prz_cnt++;
#endif
				diag_tot_cnt++;
				c = sport->SCNKdata.rxBuf[0];
				if (c == sport->SCNKparams.unitID)
				{
					unsigned char inSum = 0;
					diag_fus_cnt++;
					for (i = 0; i < sport->SCNKdata.rxLen; i++)
						inSum ^= sport->SCNKdata.rxBuf[i];
					if (inSum == 0)
					{
						int len = 0;
						bool newRXFrame = false;
						unsigned char *pSend = NULL;
						bool prev_CRC = sport->SCNKdata.useCRC;
#ifdef SCNK_DEBUG
//		dev_dbg(sport->port.dev, "<<<<<<<<< SCNK new message received: rxlen=%d, cmd=%X len=%d, job=%X >>>>>>>>>>\n",
//							sport->SCNKdata.rxLen, sport->SCNKdata.rxBuf[1], sport->SCNKdata.rxBuf[2], sport->SCNKdata.rxBuf[3]);
#endif
						c = sport->SCNKdata.rxBuf[1] & 0x3F;
						switch (c)
						{
							case REQ_RPT:
								len = sport->SCNKdata.lastTxLen;
								pSend = sport->SCNKdata.lastTxBuf;
								break;
							case REQ_STAT:
								if (sport->SCNKdata.rxLen == 3)
								{
									memcpy (&sport->SCNKdata.statMsg[2], &sport->SCNKdata.SCNKstatus, sizeof(sport->SCNKdata.SCNKstatus));
									pSend = sport->SCNKdata.statMsg;
									pSend[4] = pSend[0] ^ pSend[1] ^ pSend[2] ^ pSend[3];
									len = 5;
								}
								diag_sts_cnt++;
								break;
							case REQ_PROZ:
								if (sport->SCNKdata.rxLen == 4)
								{
									diag_prz_cnt++;
									c = sport->SCNKdata.rxBuf[2];
									switch (c)
									{
										case PROZ_INFO:
											pSend = sport->SCNKdata.infoMsg;
											pSend[6] = pSend[0] ^ pSend[1] ^ pSend[2] ^ pSend[3] ^ pSend[4] ^ pSend[5];
											len = 7;
											break;
										case PROZ_INFO_NOCRC:
											sport->SCNKdata.useCRC = false;
											pSend = sport->SCNKdata.infoMsg;
											sport->SCNKdata.infoMsg[11] = pSend[0] ^ pSend[1] ^ pSend[2] ^ pSend[3] ^ pSend[4] ^ pSend[5] ^ pSend[6] ^ pSend[7] ^ pSend[8] ^ pSend[9] ^ pSend[10];
											len = 12;
											break;
										case PROZ_INFO_CRC:
												dev_dbg(sport->port.dev, "<<<<<<<< SCNK CRC SET 1 >>>>>>>>\n");
											sport->SCNKdata.useCRC = true;
											pSend = sport->SCNKdata.infoMsg;
											sport->SCNKdata.infoMsg[11] = pSend[0] ^ pSend[1] ^ pSend[2] ^ pSend[3] ^ pSend[4] ^ pSend[5] ^ pSend[6] ^ pSend[7] ^ pSend[8] ^ pSend[9] ^ pSend[10];
											len = 12;
											break;
										default:
											diag_eot_cnt++;
											if (c & 2)
												sport->SCNKdata.SCNKstatus &= 0xDFFF;
											if (c & 4)
												sport->SCNKdata.SCNKstatus |= 0x2000;
											sport->SCNKdata.useCRC = (c & 0x40)?true:false;
											if (sport->SCNKdata.useCRC) 
												dev_dbg(sport->port.dev, "<<<<<<<< SCNK CRC SET 2 >>>>>>>>\n");

											memcpy (&sport->SCNKdata.statMsg[2], &sport->SCNKdata.SCNKstatus, sizeof(sport->SCNKdata.SCNKstatus));
											pSend = sport->SCNKdata.statMsg;
											pSend[4] = pSend[0] ^ pSend[1] ^ pSend[2] ^ pSend[3];
											len = 5;
											break;
									}
								}
								break;
							case REQ_VARI:
								c = sport->SCNKdata.rxBuf[3];
								if (c != 0 && c != sport->SCNKdata.lastRecvJob)	//new buffer
								{
									newRXFrame = true;
								}
								diag_req_cnt++;
								//send out buffer
								pSend = (sport->SCNKdata.useTxBuf2)?sport->SCNKdata.outBufMsg2:sport->SCNKdata.outBufMsg1;
								len = pSend[2]+((sport->SCNKdata.useCRC)?6:4);
#ifdef SCNK_DEBUG
//								if (sport->SCNKdata.pendingReq) 
//									dev_dbg(sport->port.dev, "SCNK: reset pending request flag\n");
#endif
								sport->SCNKdata.pendingReq = false;

								break;
							default:
								break;
						}
						//send anything prepared
						if (len)
						{
							//save for repetition
							if (pSend != sport->SCNKdata.lastTxBuf)
							{
								sport->SCNKdata.lastTxLen = len;
								memcpy(sport->SCNKdata.lastTxBuf, pSend, len);
							}
							//fill tx_buffer
							for (i=0; i<len; i++)
							{
								sport->SCNKdata.txBuf.buf[sport->SCNKdata.txBuf.head] = pSend[i];
								sport->SCNKdata.txBuf.head = (sport->SCNKdata.txBuf.head + 1) & (UART_XMIT_SIZE-1);
							}
							{
#ifdef SCNK_USING_HRTIMER
								ktime_t kt = ktime_set(0, sport->SCNKdata.gapTime);
#ifdef SCNK_DEBUG
//								dev_dbg(sport->port.dev, "<<<<<<<<< SCNK HRTstart should call %X after %d >>>>>>>>>>\n", (unsigned int)(sport->SCNKdata.hrt.function), sport->SCNKdata.gapTime);
#endif
								hrtimer_start( &sport->SCNKdata.hrt, kt, HRTIMER_MODE_REL );
#else
								unsigned long temp;
								udelay(sport->SCNKdata.gapTime / 1000); /*  */
								imx_rs485_start_tx(sport);
								/* enable transmitter and shifter empty irq */
								temp = readl(((struct uart_port *)sport)->membase + UCR4);
								temp |= UCR4_TCEN;
								writel(temp, ((struct uart_port *)sport)->membase + UCR4);
								imx_transmit_buffer(sport);
#endif
							}
						}
						if (newRXFrame)
						{
#ifdef SCNK_DEBUG
//		dev_dbg(sport->port.dev, "<<<<<<<<< SCNK new frame received: rxlen=%d, job=%X len=%d >>>>>>>>>>\n",
//							sport->SCNKdata.rxLen, sport->SCNKdata.rxBuf[3], sport->SCNKdata.rxBuf[2]);
#endif
							if(sport->SCNKdata.rxLen == sport->SCNKparams.inBufLen + (sport->SCNKdata.useCRC?6:4))
							{
								unsigned short crc = 0xffff;
								if (sport->SCNKdata.useCRC)
								{
									addCrc(sport->SCNKdata.rxBuf[0], &crc);
									addCrc(sport->SCNKparams.inBufLen, &crc);
									for (i = 0; i < sport->SCNKdata.rxBuf[2]; i++)
										addCrc(sport->SCNKdata.rxBuf[3+i], &crc);
								}
								if (!sport->SCNKdata.useCRC ||
									(((crc >> 8) == sport->SCNKdata.rxBuf[sport->SCNKparams.inBufLen+4]) &&
									 ((crc & 0xff) == sport->SCNKdata.rxBuf[sport->SCNKparams.inBufLen+3])))
								{
									//fill the data
									tty_insert_flip_string(port, sport->SCNKdata.rxBuf, sport->SCNKparams.inBufLen+6);
									bufferFull=true;	
									sport->SCNKdata.lastRecvJob = c;
								}
								else
									diag_err_cnt++;
							}
						}
						if (prev_CRC != sport->SCNKdata.useCRC)
						{
							pSend = (sport->SCNKdata.useTxBuf2)?sport->SCNKdata.outBufMsg2:sport->SCNKdata.outBufMsg1;
							setSCNKTxData(pSend, sport->SCNKparams.outBufLen+6, sport->SCNKdata.useCRC);
						}
					}	//XOR match
					else
					{
						diag_err_cnt++;
					}
				}	//unitID
				sport->SCNKdata.rxLen = 0;
				sport->SCNKdata.expectedLen = 3;
				spin_unlock_irqrestore(&sport->port.lock, flags);
				if (bufferFull)
					tty_flip_buffer_push(port);
				return IRQ_HANDLED;
			}
		}
		else
#endif
			tty_insert_flip_char(port, rx, flg);
	}

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);
	tty_flip_buffer_push(port);
	return IRQ_HANDLED;
}

static int start_rx_dma(struct imx_port *sport);
/*
 * If the RXFIFO is filled with some data, and then we
 * arise a DMA operation to receive them.
 */
static void imx_dma_rxint(struct imx_port *sport)
{
        unsigned long temp;
        unsigned long flags;

        spin_lock_irqsave(&sport->port.lock, flags);

        temp = readl(sport->port.membase + USR2);
        if ((temp & USR2_RDR) && !sport->dma_is_rxing) {
                sport->dma_is_rxing = 1;

                /* disable the `Recerver Ready Interrrupt` */
                temp = readl(sport->port.membase + UCR1);
                temp &= ~(UCR1_RRDYEN);
                writel(temp, sport->port.membase + UCR1);

                /* tell the DMA to receive the data. */
                start_rx_dma(sport);
        }

        spin_unlock_irqrestore(&sport->port.lock, flags);
}

static irqreturn_t imx_int(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned int sts;
	unsigned int sts2;

	sts = readl(sport->port.membase + USR1);
        sts2 = readl(sport->port.membase + USR2);

        if (sts & USR1_RRDY) {
                if (sport->dma_is_enabled)
                        imx_dma_rxint(sport);
                else
		imx_rxint(irq, dev_id);
	}

        if ((sts & USR1_TRDY &&
             readl(sport->port.membase + UCR1) & UCR1_TXMPTYEN) ||
            (sts2 & USR2_TXDC &&
             readl(sport->port.membase + UCR4) & UCR4_TCEN))
		imx_txint(irq, dev_id);

	if (sts & USR1_RTSD)
		imx_rtsint(irq, dev_id);

	if (sts & USR1_AWAKE)
		writel(USR1_AWAKE, sport->port.membase + USR1);

        if (sts2 & USR2_ORE) {
                dev_err(sport->port.dev, "Rx FIFO overrun\n");
                sport->port.icount.overrun++;
                writel(USR2_ORE, sport->port.membase + USR2);
        }

	return IRQ_HANDLED;
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int imx_tx_empty(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned int ret;

	ret = (readl(sport->port.membase + USR2) & USR2_TXDC) ?  TIOCSER_TEMT : 0;

	/* If the TX DMA is working, return 0. */
	if (sport->dma_is_enabled && sport->dma_is_txing)
		ret = 0;

	return ret;
}

/*
 * We have a modem side uart, so the meanings of RTS and CTS are inverted.
 */
static unsigned int imx_get_mctrl(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned int tmp = TIOCM_DSR | TIOCM_CAR;

	if (readl(sport->port.membase + USR1) & USR1_RTSS)
		tmp |= TIOCM_CTS;

	if (readl(sport->port.membase + UCR2) & UCR2_CTS)
		tmp |= TIOCM_RTS;

	if (readl(sport->port.membase + uts_reg(sport)) & UTS_LOOP)
		tmp |= TIOCM_LOOP;

	return tmp;
}

static void imx_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long temp;

        if (!(sport->rs485.flags & SER_RS485_ENABLED)) {
                temp = readl(sport->port.membase + UCR2);
                temp &= ~(UCR2_CTS);
	if (mctrl & TIOCM_RTS)
		temp |= UCR2_CTS;

	writel(temp, sport->port.membase + UCR2);
        }

	temp = readl(sport->port.membase + uts_reg(sport)) & ~UTS_LOOP;
	if (mctrl & TIOCM_LOOP)
		temp |= UTS_LOOP;
	writel(temp, sport->port.membase + uts_reg(sport));
}

/*
 * Interrupts always disabled.
 */
static void imx_break_ctl(struct uart_port *port, int break_state)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long flags, temp;

	spin_lock_irqsave(&sport->port.lock, flags);

	temp = readl(sport->port.membase + UCR1) & ~UCR1_SNDBRK;

	if (break_state != 0)
		temp |= UCR1_SNDBRK;

	writel(temp, sport->port.membase + UCR1);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

#define TXTL 4 /* reset default */
#define RXTL 1 /* For console port */
#define RXTL_UART 16 /* For uart */

static void imx_setup_ufcr(struct imx_port *sport, unsigned int mode)
{
        unsigned int val;

        /* set receiver / transmitter trigger level */
        val = readl(sport->port.membase + UFCR) & (UFCR_RFDIV | UFCR_DCEDTE);
        val |= TXTL << UFCR_TXTL_SHF | RXTL;
        writel(val, sport->port.membase + UFCR);
}

#define RX_BUF_SIZE	(PAGE_SIZE)

static void imx_rx_dma_done(struct imx_port *sport)
{
        unsigned long temp;
        unsigned long flags;

        spin_lock_irqsave(&sport->port.lock, flags);

        /* Enable this interrupt when the RXFIFO is empty. */
        temp = readl(sport->port.membase + UCR1);
        temp |= UCR1_RRDYEN;
        writel(temp, sport->port.membase + UCR1);

        sport->dma_is_rxing = 0;

        /* Is the shutdown waiting for us? */
        if (waitqueue_active(&sport->dma_wait))
                wake_up(&sport->dma_wait);

        spin_unlock_irqrestore(&sport->port.lock, flags);
}

/*
 * There are three kinds of RX DMA interrupts(such as in the MX6Q):
 *   [1] the RX DMA buffer is full.
 *   [2] the Aging timer expires(wait for 8 bytes long)
 *   [3] the Idle Condition Detect(enabled the UCR4_IDDMAEN).
 *
 * The [2] is trigger when a character was been sitting in the FIFO
 * meanwhile [3] can wait for 32 bytes long when the RX line is
 * on IDLE state and RxFIFO is empty.
 */
static void dma_rx_callback(void *data)
{
        struct imx_port *sport = data;
        struct dma_chan *chan = sport->dma_chan_rx;
        struct scatterlist *sgl = &sport->rx_sgl;
        struct tty_port *port = &sport->port.state->port;
        struct dma_tx_state state;
        enum dma_status status;
        unsigned int count;

        /* unmap it first */
        dma_unmap_sg(sport->port.dev, sgl, 1, DMA_FROM_DEVICE);

        status = dmaengine_tx_status(chan, (dma_cookie_t)0, &state);
        count = RX_BUF_SIZE - state.residue;

        if (readl(sport->port.membase + USR2) & USR2_IDLE) {
                /* In condition [3] the SDMA counted up too early */
                count--;

                writel(USR2_IDLE, sport->port.membase + USR2);
        }

        dev_dbg(sport->port.dev, "We get %d bytes.\n", count);

        if (count) {
                if (!(sport->port.ignore_status_mask & URXD_DUMMY_READ))
                        tty_insert_flip_string(port, sport->rx_buf, count);
                tty_flip_buffer_push(port);

                start_rx_dma(sport);
        } else if (readl(sport->port.membase + USR2) & USR2_RDR) {
                /*
                 * start rx_dma directly once data in RXFIFO, more efficient
                 * than before:
                 *      1. call imx_rx_dma_done to stop dma if no data received
                 *      2. wait next  RDR interrupt to start dma transfer.
                 */
                start_rx_dma(sport);
        } else {
                /*
                 * stop dma to prevent too many IDLE event trigged if no data
                 * in RXFIFO
                 */
                imx_rx_dma_done(sport);
        }
}

static int start_rx_dma(struct imx_port *sport)
{
        struct scatterlist *sgl = &sport->rx_sgl;
        struct dma_chan *chan = sport->dma_chan_rx;
        struct device *dev = sport->port.dev;
        struct dma_async_tx_descriptor *desc;
        int ret;

        sg_init_one(sgl, sport->rx_buf, RX_BUF_SIZE);
        ret = dma_map_sg(dev, sgl, 1, DMA_FROM_DEVICE);
        if (ret == 0) {
                dev_err(dev, "DMA mapping error for RX.\n");
                return -EINVAL;
        }
        desc = dmaengine_prep_slave_sg(chan, sgl, 1, DMA_DEV_TO_MEM,
                                        DMA_PREP_INTERRUPT);
        if (!desc) {
                dma_unmap_sg(dev, sgl, 1, DMA_FROM_DEVICE);
                dev_err(dev, "We cannot prepare for the RX slave dma!\n");
                return -EINVAL;
        }
        desc->callback = dma_rx_callback;
        desc->callback_param = sport;

        dev_dbg(dev, "RX: prepare for the DMA.\n");
        dmaengine_submit(desc);
        dma_async_issue_pending(chan);
        return 0;
}

static void imx_uart_dma_exit(struct imx_port *sport)
{
        if (sport->dma_chan_rx) {
                dma_release_channel(sport->dma_chan_rx);
                sport->dma_chan_rx = NULL;

                kfree(sport->rx_buf);
                sport->rx_buf = NULL;
        }

        if (sport->dma_chan_tx) {
                dma_release_channel(sport->dma_chan_tx);
                sport->dma_chan_tx = NULL;
        }

	sport->dma_is_inited = 0;
}

static int imx_uart_dma_init(struct imx_port *sport)
{
        struct dma_slave_config slave_config = {};
        struct device *dev = sport->port.dev;
        int ret;

         /* Prepare for RX : */
         sport->dma_chan_rx = dma_request_slave_channel(dev, "rx");
         if (!sport->dma_chan_rx) {
                 dev_dbg(dev, "cannot get the DMA channel.\n");
                 ret = -EINVAL;
                 goto err;
         }
 
         slave_config.direction = DMA_DEV_TO_MEM;
         slave_config.src_addr = sport->port.mapbase + URXD0;
         slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
         slave_config.src_maxburst = RXTL;
         ret = dmaengine_slave_config(sport->dma_chan_rx, &slave_config);
         if (ret) {
                 dev_err(dev, "error in RX dma configuration.\n");
                 goto err;
         }
 
         sport->rx_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
         if (!sport->rx_buf) {
                 ret = -ENOMEM;
                 goto err;
         }
 
	/* Prepare for TX : */
	sport->dma_chan_tx = dma_request_slave_channel(dev, "tx");
	if (!sport->dma_chan_tx) {
		dev_err(dev, "cannot get the TX DMA channel!\n");
		ret = -EINVAL;
		goto err;
	}

	slave_config.direction = DMA_MEM_TO_DEV;
	slave_config.dst_addr = sport->port.mapbase + URTX0;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	slave_config.dst_maxburst = TXTL;
	ret = dmaengine_slave_config(sport->dma_chan_tx, &slave_config);
	if (ret) {
		dev_err(dev, "error in TX dma configuration.");
		goto err;
	}

	sport->dma_is_inited = 1;

	return 0;
err:
	imx_uart_dma_exit(sport);
	return ret;
}

static void imx_enable_dma(struct imx_port *sport)
{
	unsigned long temp;

	init_waitqueue_head(&sport->dma_wait);

	/* set UCR1 */
	temp = readl(sport->port.membase + UCR1);
	temp |= UCR1_RDMAEN | UCR1_TDMAEN | UCR1_ATDMAEN |
		/* wait for 32 idle frames for IDDMA interrupt */
		UCR1_ICD_REG(3);
	writel(temp, sport->port.membase + UCR1);

	/* set UCR4 */
	temp = readl(sport->port.membase + UCR4);
	temp |= UCR4_IDDMAEN;
	writel(temp, sport->port.membase + UCR4);

	sport->dma_is_enabled = 1;
}

static void imx_disable_dma(struct imx_port *sport)
{
	unsigned long temp;

	/* clear UCR1 */
	temp = readl(sport->port.membase + UCR1);
	temp &= ~(UCR1_RDMAEN | UCR1_TDMAEN | UCR1_ATDMAEN);
	writel(temp, sport->port.membase + UCR1);

	/* clear UCR2 */
	temp = readl(sport->port.membase + UCR2);
	temp &= ~(UCR2_CTSC | UCR2_CTS);
	writel(temp, sport->port.membase + UCR2);

	/* clear UCR4 */
	temp = readl(sport->port.membase + UCR4);
	temp &= ~UCR4_IDDMAEN;
	writel(temp, sport->port.membase + UCR4);

	sport->dma_is_enabled = 0;
}

/* half the RX buffer size */
#define CTSTL 16

static int imx_startup(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;
	int retval, i;
	unsigned long flags, temp;

#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
	sport->SCNKdata.SCNKenabled = false;
    dev_info(sport->port.dev, "STARTUP UART %d\n", sport->port.line);
#endif
	retval = clk_prepare_enable(sport->clk_per);
	if (retval)
                 return retval;
	retval = clk_prepare_enable(sport->clk_ipg);
	if (retval) {
		clk_disable_unprepare(sport->clk_per);
                 return retval;
	}

	imx_setup_ufcr(sport, 0);

	/* disable the DREN bit (Data Ready interrupt enable) before
	 * requesting IRQs
	 */
	temp = readl(sport->port.membase + UCR4);

	/* set the trigger level for CTS */
	temp &= ~(UCR4_CTSTL_MASK << UCR4_CTSTL_SHF);
	temp |= CTSTL << UCR4_CTSTL_SHF;

	writel(temp & ~UCR4_DREN, sport->port.membase + UCR4);

         spin_lock_irqsave(&sport->port.lock, flags);
	/* Reset fifo's and state machines */
	i = 100;

	temp = readl(sport->port.membase + UCR2);
	temp &= ~UCR2_SRST;
	writel(temp, sport->port.membase + UCR2);

	while (!(readl(sport->port.membase + UCR2) & UCR2_SRST) && (--i > 0))
		udelay(1);

         /*
          * Finally, clear and enable interrupts
          */
         writel(USR1_RTSD, sport->port.membase + USR1);
         writel(USR2_ORE, sport->port.membase + USR2);
 
         temp = readl(sport->port.membase + UCR1);
         temp |= UCR1_RRDYEN | UCR1_RTSDEN | UCR1_UARTEN;
 
         writel(temp, sport->port.membase + UCR1);
 
         temp = readl(sport->port.membase + UCR4);
         temp |= UCR4_OREN;
         writel(temp, sport->port.membase + UCR4);
	temp = readl(sport->port.membase + UCR2);
	temp |= (UCR2_RXEN | UCR2_TXEN);
	if (!sport->have_rtscts)
		temp |= UCR2_IRTS;
	writel(temp, sport->port.membase + UCR2);

	if (!is_imx1_uart(sport)) {
		temp = readl(sport->port.membase + UCR3);
		temp |= IMX21_UCR3_RXDMUXSEL | UCR3_ADNIMP;
		writel(temp, sport->port.membase + UCR3);
	}
 

	/*
	 * Enable modem status interrupts
	 */
	imx_enable_ms(&sport->port);
	spin_unlock_irqrestore(&sport->port.lock, flags);
 
         return 0;
}

static void imx_shutdown(struct uart_port *port)
{
         struct imx_port *sport = (struct imx_port *)port;
         unsigned long temp;
         unsigned long flags;
 
#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
		 sport->SCNKdata.SCNKenabled = false;
		 dev_info(sport->port.dev, "SHUTDOWN UART %d\n", sport->port.line);
#endif
		 if (sport->dma_is_enabled) {
                 int ret;
 
                 /* We have to wait for the DMA to finish. */
                 ret = wait_event_interruptible(sport->dma_wait,
                         !sport->dma_is_rxing && !sport->dma_is_txing);
                 if (ret != 0) {
                         sport->dma_is_rxing = 0;
                         sport->dma_is_txing = 0;
                         dmaengine_terminate_all(sport->dma_chan_tx);
                         dmaengine_terminate_all(sport->dma_chan_rx);
                 }
                 spin_lock_irqsave(&sport->port.lock, flags);
                 imx_stop_tx(port);
                 imx_stop_rx(port);
                 imx_disable_dma(sport);
                 spin_unlock_irqrestore(&sport->port.lock, flags);
                 imx_uart_dma_exit(sport);
	}

	spin_lock_irqsave(&sport->port.lock, flags);
	temp = readl(sport->port.membase + UCR2);
	temp &= ~(UCR2_TXEN);
	writel(temp, sport->port.membase + UCR2);
	spin_unlock_irqrestore(&sport->port.lock, flags);
 
	/*
	 * Stop our timer.
	 */
	del_timer_sync(&sport->timer);

         /*
          * Disable all interrupts, port and break condition.
	 */

	spin_lock_irqsave(&sport->port.lock, flags);
	temp = readl(sport->port.membase + UCR1);
	temp &= ~(UCR1_TXMPTYEN | UCR1_RRDYEN | UCR1_RTSDEN | UCR1_UARTEN | UCR1_TRDYEN);

	writel(temp, sport->port.membase + UCR1);
	spin_unlock_irqrestore(&sport->port.lock, flags);

#ifdef TURNBACK_SERIAL_ON_STANDARD_RS232
	if(sport->rs485.flags & SER_RS485_ENABLED)
	{
		imx_rs485_stop_tx(sport);
		sport->rs485.flags &= ~SER_RS485_ENABLED;
		imx_config_rs485(sport);
	}
#endif
	
	clk_disable_unprepare(sport->clk_per);
	clk_disable_unprepare(sport->clk_ipg);
}

static void imx_flush_buffer(struct uart_port *port)
{
         struct imx_port *sport = (struct imx_port *)port;
         struct scatterlist *sgl = &sport->tx_sgl[0];
         unsigned long temp;
         int i = 100, ubir, ubmr, uts;
 
         if (!sport->dma_chan_tx)
                 return;
 
         sport->tx_bytes = 0;
         dmaengine_terminate_all(sport->dma_chan_tx);
         if (sport->dma_is_txing) {
                 dma_unmap_sg(sport->port.dev, sgl, sport->dma_tx_nents,
                              DMA_TO_DEVICE);
                 temp = readl(sport->port.membase + UCR1);
                 temp &= ~UCR1_TDMAEN;
                 writel(temp, sport->port.membase + UCR1);
                 sport->dma_is_txing = false;
         }
 
         /*
          * According to the Reference Manual description of the UART SRST bit:
          * "Reset the transmit and receive state machines,
          * all FIFOs and register USR1, USR2, UBIR, UBMR, UBRC, URXD, UTXD
          * and UTS[6-3]". As we don't need to restore the old values from
          * USR1, USR2, URXD, UTXD, only save/restore the other four registers
          */
         ubir = readl(sport->port.membase + UBIR);
         ubmr = readl(sport->port.membase + UBMR);
         uts = readl(sport->port.membase + IMX21_UTS);
 
         temp = readl(sport->port.membase + UCR2);
         temp &= ~UCR2_SRST;
         writel(temp, sport->port.membase + UCR2);
 
         while (!(readl(sport->port.membase + UCR2) & UCR2_SRST) && (--i > 0))
                 udelay(1);
 
         /* Restore the registers */
         writel(ubir, sport->port.membase + UBIR);
         writel(ubmr, sport->port.membase + UBMR);
         writel(uts, sport->port.membase + IMX21_UTS);
}

static void
imx_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	struct imx_port *sport = (struct imx_port *)port;
	unsigned long flags;
	unsigned int ucr2, old_ucr1, old_txrxen, baud, quot;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	unsigned int div, ufcr;
	unsigned long num, denom;
	uint64_t tdiv64;
	/*
	 * We only support CS7 and CS8.
	 */
	while ((termios->c_cflag & CSIZE) != CS7 &&
	       (termios->c_cflag & CSIZE) != CS8) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS8)
		ucr2 = UCR2_WS | UCR2_SRST | UCR2_IRTS;
	else
		ucr2 = UCR2_SRST | UCR2_IRTS;

	if (termios->c_cflag & CRTSCTS) {
		if (sport->have_rtscts) {
			ucr2 &= ~UCR2_IRTS;
			ucr2 |= UCR2_CTSC;
                         /* Can we enable the DMA support? */
                         if (is_imx6q_uart(sport) && !uart_console(port)
                                 && !sport->dma_is_inited)
                                 imx_uart_dma_init(sport);
		} else {
			termios->c_cflag &= ~CRTSCTS;
		}
	}

	if (termios->c_cflag & CSTOPB)
		ucr2 |= UCR2_STPB;
	if (termios->c_cflag & PARENB) {
		ucr2 |= UCR2_PREN;
		if (termios->c_cflag & PARODD)
			ucr2 |= UCR2_PROE;
	}

	del_timer_sync(&sport->timer);

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);
	quot = uart_get_divisor(port, baud);

	spin_lock_irqsave(&sport->port.lock, flags);

	sport->port.read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |= (URXD_FRMERR | URXD_PRERR);
	if (termios->c_iflag & (BRKINT | PARMRK))
		sport->port.read_status_mask |= URXD_BRK;

	/*
	 * Characters to ignore
	 */
	sport->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
                 sport->port.ignore_status_mask |= URXD_PRERR | URXD_FRMERR;
	if (termios->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |= URXD_BRK;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |= URXD_OVRRUN;
	}

         if ((termios->c_cflag & CREAD) == 0)
                 sport->port.ignore_status_mask |= URXD_DUMMY_READ;
	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * disable interrupts and drain transmitter
	 */
	old_ucr1 = readl(sport->port.membase + UCR1);
	writel(old_ucr1 & ~(UCR1_TXMPTYEN | UCR1_RRDYEN | UCR1_RTSDEN | UCR1_TRDYEN),
			sport->port.membase + UCR1);

	while (!(readl(sport->port.membase + USR2) & USR2_TXDC))
		barrier();

	/* then, disable everything */
	old_txrxen = readl(sport->port.membase + UCR2);
	writel(old_txrxen & ~(UCR2_TXEN | UCR2_RXEN),
			sport->port.membase + UCR2);
	old_txrxen &= (UCR2_CTS | UCR2_TXEN | UCR2_RXEN);
	/* custom-baudrate handling */
	div = sport->port.uartclk / (baud * 16);
	if (baud == 38400 && quot != div)
	  baud = sport->port.uartclk / (quot * 16);

	div = sport->port.uartclk / (baud * 16);
	if (div > 7)
		div = 7;
	if (!div)
		div = 1;

	rational_best_approximation(16 * div * baud, sport->port.uartclk,
		1 << 16, 1 << 16, &num, &denom);

	tdiv64 = sport->port.uartclk;
	tdiv64 *= num;
	do_div(tdiv64, denom * 16 * div);
	tty_termios_encode_baud_rate(termios,
				(speed_t)tdiv64, (speed_t)tdiv64);

	num -= 1;
	denom -= 1;

	ufcr = readl(sport->port.membase + UFCR);
	ufcr = (ufcr & (~UFCR_RFDIV)) | UFCR_RFDIV_REG(div);
	if (sport->dte_mode)
		ufcr |= UFCR_DCEDTE;
	writel(ufcr, sport->port.membase + UFCR);

	writel(num, sport->port.membase + UBIR);
	writel(denom, sport->port.membase + UBMR);

	if (!is_imx1_uart(sport))
		writel(sport->port.uartclk / div / 1000,
				sport->port.membase + IMX21_ONEMS);

	writel(old_ucr1, sport->port.membase + UCR1);

	/* set the parity, stop bits and data size */
	writel(ucr2 | old_txrxen, sport->port.membase + UCR2);

	if (UART_ENABLE_MS(&sport->port, termios->c_cflag))
		imx_enable_ms(&sport->port);

	if (sport->dma_is_inited && !sport->dma_is_enabled)
		imx_enable_dma(sport);

	if (sport->rs485.flags & SER_RS485_ENABLED)
		   imx_config_rs485(sport);
	
	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static const char *imx_type(struct uart_port *port)
{
	struct imx_port *sport = (struct imx_port *)port;

	return sport->port.type == PORT_IMX ? "IMX" : NULL;
}

static void imx_release_port(struct uart_port *port)
{
	/* nothing to do */
}

/*
 * Configure/autoconfigure the port.
 */
static void imx_config_port(struct uart_port *port, int flags)
{
	struct imx_port *sport = (struct imx_port *)port;

	if (flags & UART_CONFIG_TYPE)
		sport->port.type = PORT_IMX;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_IMX and PORT_UNKNOWN
 */
static int
imx_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct imx_port *sport = (struct imx_port *)port;
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_IMX)
		ret = -EINVAL;
	if (sport->port.irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;
	if (sport->port.uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if (sport->port.mapbase != (unsigned long)ser->iomem_base)
		ret = -EINVAL;
	if (sport->port.iobase != ser->port)
		ret = -EINVAL;

	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

#if defined(CONFIG_CONSOLE_POLL)
 static int imx_poll_init(struct uart_port *port)
 {
         struct imx_port *sport = (struct imx_port *)port;
         unsigned long flags;
         unsigned long temp;
         int retval;
 
         retval = clk_prepare_enable(sport->clk_ipg);
         if (retval)
                 return retval;
         retval = clk_prepare_enable(sport->clk_per);
         if (retval)
                 clk_disable_unprepare(sport->clk_ipg);
 
         imx_setup_ufcr(sport, 0);
 
         spin_lock_irqsave(&sport->port.lock, flags);
 
         temp = readl(sport->port.membase + UCR1);
         if (is_imx1_uart(sport))
                 temp |= IMX1_UCR1_UARTCLKEN;
         temp |= UCR1_UARTEN | UCR1_RRDYEN;
         temp &= ~(UCR1_TXMPTYEN | UCR1_RTSDEN | UCR1_TRDYEN);
         writel(temp, sport->port.membase + UCR1);
 
         temp = readl(sport->port.membase + UCR2);
         temp |= UCR2_RXEN;
         writel(temp, sport->port.membase + UCR2);
 
         spin_unlock_irqrestore(&sport->port.lock, flags);
 
         return 0;
 }
 
 static int imx_poll_get_char(struct uart_port *port)
 {
         if (!(readl_relaxed(port->membase + USR2) & USR2_RDR))
                 return NO_POLL_CHAR;
 
         return readl_relaxed(port->membase + URXD0) & URXD_RX_DATA;
 }
 
 static void imx_poll_put_char(struct uart_port *port, unsigned char c)
 {
         unsigned int status;
 
         /* drain */
         do {
                 status = readl_relaxed(port->membase + USR1);
         } while (~status & USR1_TRDY);
 
         /* write */
         writel_relaxed(c, port->membase + URTX0);
 
         /* flush */
         do {
                 status = readl_relaxed(port->membase + USR2);
         } while (~status & USR2_TXDC);
}
#endif

static struct uart_ops imx_pops = {
	.tx_empty	= imx_tx_empty,
	.set_mctrl	= imx_set_mctrl,
	.get_mctrl	= imx_get_mctrl,
	.stop_tx	= imx_stop_tx,
	.start_tx	= imx_start_tx,
	.stop_rx	= imx_stop_rx,
	.enable_ms	= imx_enable_ms,
	.break_ctl	= imx_break_ctl,
	.startup	= imx_startup,
	.shutdown	= imx_shutdown,
	.ioctl          = imx_ioctl,	
	.flush_buffer	= imx_flush_buffer,
	.set_termios	= imx_set_termios,
	.type		= imx_type,
	.config_port	= imx_config_port,
	.verify_port	= imx_verify_port,
	.release_port	= imx_release_port,
#if defined(CONFIG_CONSOLE_POLL)
	.poll_get_char  = imx_poll_get_char,
	.poll_put_char  = imx_poll_put_char,
#endif
};

static struct imx_port *imx_ports[UART_NR];

#ifdef CONFIG_SERIAL_IMX_CONSOLE
static void imx_console_putchar(struct uart_port *port, int ch)
{
	struct imx_port *sport = (struct imx_port *)port;

	while (readl(sport->port.membase + uts_reg(sport)) & UTS_TXFULL)
		barrier();

	writel(ch, sport->port.membase + URTX0);
}

/*
 * Interrupts are disabled on entering
 */
static void
imx_console_write(struct console *co, const char *s, unsigned int count)
{
	struct imx_port *sport = imx_ports[co->index];
	struct imx_port_ucrs old_ucr;
	unsigned int ucr1;
	unsigned long flags = 0;
	int locked = 1;
	int retval;

	retval = clk_enable(sport->clk_per);
	if (retval)
		return;
	retval = clk_enable(sport->clk_ipg);
	if (retval) {
		clk_disable(sport->clk_per);
		return;
	}

	if (sport->port.sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&sport->port.lock, flags);
	else
		spin_lock_irqsave(&sport->port.lock, flags);

	/*
	 *	First, save UCR1/2/3 and then disable interrupts
	 */
	imx_port_ucrs_save(&sport->port, &old_ucr);
	ucr1 = old_ucr.ucr1;

	if (is_imx1_uart(sport))
		ucr1 |= IMX1_UCR1_UARTCLKEN;
	ucr1 |= UCR1_UARTEN;
	ucr1 &= ~(UCR1_TXMPTYEN | UCR1_RRDYEN | UCR1_RTSDEN | UCR1_TRDYEN);

	writel(ucr1, sport->port.membase + UCR1);

	writel(old_ucr.ucr2 | UCR2_TXEN, sport->port.membase + UCR2);

	uart_console_write(&sport->port, s, count, imx_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore UCR1/2/3
	 */
	while (!(readl(sport->port.membase + USR2) & USR2_TXDC));

	imx_port_ucrs_restore(&sport->port, &old_ucr);

	if (locked)
		spin_unlock_irqrestore(&sport->port.lock, flags);

	clk_disable(sport->clk_ipg);
	clk_disable(sport->clk_per);
}

/*
 * If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init
imx_console_get_options(struct imx_port *sport, int *baud,
			   int *parity, int *bits)
{

	if (readl(sport->port.membase + UCR1) & UCR1_UARTEN) {
		/* ok, the port was enabled */
		unsigned int ucr2, ubir, ubmr, uartclk;
		unsigned int baud_raw;
		unsigned int ucfr_rfdiv;

		ucr2 = readl(sport->port.membase + UCR2);

		*parity = 'n';
		if (ucr2 & UCR2_PREN) {
			if (ucr2 & UCR2_PROE)
				*parity = 'o';
			else
				*parity = 'e';
		}

		if (ucr2 & UCR2_WS)
			*bits = 8;
		else
			*bits = 7;

		ubir = readl(sport->port.membase + UBIR) & 0xffff;
		ubmr = readl(sport->port.membase + UBMR) & 0xffff;

		ucfr_rfdiv = (readl(sport->port.membase + UFCR) & UFCR_RFDIV) >> 7;
		if (ucfr_rfdiv == 6)
			ucfr_rfdiv = 7;
		else
			ucfr_rfdiv = 6 - ucfr_rfdiv;

		uartclk = clk_get_rate(sport->clk_per);
		uartclk /= ucfr_rfdiv;

		{	/*
			 * The next code provides exact computation of
			 *   baud_raw = round(((uartclk/16) * (ubir + 1)) / (ubmr + 1))
			 * without need of float support or long long division,
			 * which would be required to prevent 32bit arithmetic overflow
			 */
			unsigned int mul = ubir + 1;
			unsigned int div = 16 * (ubmr + 1);
			unsigned int rem = uartclk % div;

			baud_raw = (uartclk / div) * mul;
			baud_raw += (rem * mul + div / 2) / div;
			*baud = (baud_raw + 50) / 100 * 100;
		}

		if (*baud != baud_raw)
			pr_info("Console IMX rounded baud rate from %d to %d\n",
				baud_raw, *baud);
	}
}

static int __init
imx_console_setup(struct console *co, char *options)
{
	struct imx_port *sport;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int retval;

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(imx_ports))
		co->index = 0;
	sport = imx_ports[co->index];
	if (sport == NULL)
		return -ENODEV;

	/* For setting the registers, we only need to enable the ipg clock. */
	retval = clk_prepare_enable(sport->clk_ipg);
	if (retval)
		goto error_console;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		imx_console_get_options(sport, &baud, &parity, &bits);

	imx_setup_ufcr(sport, 0);

	retval = uart_set_options(&sport->port, co, baud, parity, bits, flow);

	clk_disable(sport->clk_ipg);
	if (retval) {
		clk_unprepare(sport->clk_ipg);
		goto error_console;
	}

	retval = clk_prepare(sport->clk_per);
	if (retval)
		clk_disable_unprepare(sport->clk_ipg);

error_console:
	return retval;
}

static struct uart_driver imx_reg;
static struct console imx_console = {
	.name		= DEV_NAME,
	.write		= imx_console_write,
	.device		= uart_console_device,
	.setup		= imx_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &imx_reg,
};

#define IMX_CONSOLE	&imx_console
#else
#define IMX_CONSOLE	NULL
#endif

static struct uart_driver imx_reg = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
	.major          = SERIAL_IMX_MAJOR,
	.minor          = MINOR_START,
	.nr             = ARRAY_SIZE(imx_ports),
	.cons           = IMX_CONSOLE,
};

static int serial_imx_suspend(struct platform_device *dev, pm_message_t state)
{
	struct imx_port *sport = platform_get_drvdata(dev);
	unsigned int val;

	/* enable wakeup from i.MX UART */
	val = readl(sport->port.membase + UCR3);
		val |= UCR3_AWAKEN;
	writel(val, sport->port.membase + UCR3);

	uart_suspend_port(&imx_reg, &sport->port);

	return 0;
}

static int serial_imx_resume(struct platform_device *dev)
{
	struct imx_port *sport = platform_get_drvdata(dev);
	unsigned int val;

	/* disable wakeup from i.MX UART */
	val = readl(sport->port.membase + UCR3);
         val &= ~UCR3_AWAKEN;
	writel(val, sport->port.membase + UCR3);

	uart_resume_port(&imx_reg, &sport->port);

	return 0;
}

#ifdef CONFIG_OF
/*
 * This function returns 1 iff pdev isn't a device instatiated by dt, 0 iff it
 * could successfully get all information from dt or a negative errno.
 */
static int serial_imx_probe_dt(struct imx_port *sport,
		struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(imx_uart_dt_ids, &pdev->dev);
	int ret;
	struct device_node *plxxnp;

	if (!np)
		/* no device tree device */
		return 1;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		return ret;
	}
	sport->port.line = ret;

	if (of_get_property(np, "fsl,uart-has-rtscts", NULL))
		sport->have_rtscts = 1;

	sport->is_plugin_module = 0;
	if (of_get_property(np, "is-plugin-module", NULL))
		sport->is_plugin_module = 1;
	
	if (of_get_property(np, "fsl,dte-mode", NULL))
		sport->dte_mode = 1;
	
	sport->rs485.flags = 0;
	
	//Get rts-gpio line (used for tx enable 1=active)
	sport->rts_gpio = -EINVAL;
	ret = of_get_named_gpio(np, "rts-gpio", 0); 
	if (ret >= 0 && gpio_is_valid(ret)) 
	{
		dev_dbg(sport->port.dev, "Setting UART /dev/ttymxc%d with the pin %d as rts-gpio\n", sport->port.line, ret);
		sport->rts_gpio = ret;

		ret = gpio_request(sport->rts_gpio, "rts-gpio");
		if(ret < 0)
		  return ret;
		
		ret = gpio_direction_output(sport->rts_gpio, 0);
		if(ret < 0)
		  return ret;
	}

	//Get be15mode-gpio line (used for mode selection between RS485 and RS422 on BE15 carriers 0=RS422 1=RS485)
	sport->be15mode_gpio = -EINVAL;
	ret = of_get_named_gpio(np, "be15mode-gpio", 0); // IMX_GPIO_NR(3, 29);
	if (ret >= 0 && gpio_is_valid(ret)) 
	{
		dev_dbg(sport->port.dev, "Setting UART /dev/ttymxc%d with the pin %d as be15mode-gpio\n", sport->port.line, ret);
		sport->be15mode_gpio = ret;

		ret = gpio_request(sport->be15mode_gpio, "be15mode-gpio");
		if(ret < 0)
		  return ret;
		
		ret = gpio_direction_output(sport->be15mode_gpio, 0);
		if(ret < 0)
		  return ret;
	}
	
		
	// Get mode-gpio line, which is used to switch from RS485 <-> RS232 on programmable phys
	sport->mode_gpio = -EINVAL;
	ret = of_get_named_gpio(np, "mode-gpio", 0);
	if (ret >= 0 && gpio_is_valid(ret)) 
	{
		dev_dbg(sport->port.dev, "Setting UART /dev/ttymxc%d with the pin %d as mode-gpio\n", sport->port.line, ret);
		sport->mode_gpio = ret;

		ret = gpio_request(sport->mode_gpio, "mode-gpio");
		if(ret < 0)
		  return ret;
		
		ret = gpio_direction_output(sport->mode_gpio, 0);
		if(ret < 0)
		  return ret;
	}

	// Get rxen-gpio, which is used to enable/disable the rx on programmable phys
	sport->rxen_gpio = -EINVAL;
	ret = of_get_named_gpio(np, "rxen-gpio", 0);
	if (ret >= 0 && gpio_is_valid(ret)) 
	{
		dev_dbg(sport->port.dev, "Setting UART /dev/ttymxc%d with the pin %d as rxen-gpio\n", sport->port.line, ret);
		sport->rxen_gpio = ret;

		ret = gpio_request(sport->rxen_gpio, "rxen-gpio");
		if(ret < 0)
		  return ret;
		
		ret = gpio_direction_output(sport->rxen_gpio, 1);
		if(ret < 0)
		  return ret;
	}
	
	/* Get handle to plugin plxx manager drivers (if any) */
	sport->plugin1dev = NULL;
	sport->plugin2dev = NULL;
	
	plxxnp = of_parse_phandle(np, "plugin1", 0);
	if (plxxnp) 
	{
	  sport->plugin1dev = of_find_device_by_node(plxxnp);
	  of_node_put(plxxnp);
	}

	plxxnp = of_parse_phandle(np, "plugin2", 0);
	if (plxxnp) 
	{
	  sport->plugin2dev = of_find_device_by_node(plxxnp);
	  of_node_put(plxxnp);
	}
	
	sport->devdata = of_id->data;

	if (of_property_read_bool(np, "mode-two-lines-only"))
	{
	    sport->mode_two_lines_only = 1;
	    dev_dbg(sport->port.dev, "Setting UART /dev/ttymxc%d with two wires serial mode \n", sport->port.line );
	} else {
	    sport->mode_two_lines_only = 0;
	}
	return 0;
}
#else
static inline int serial_imx_probe_dt(struct imx_port *sport,
		struct platform_device *pdev)
{
	return 1;
}
#endif

static void serial_imx_probe_pdata(struct imx_port *sport,
		struct platform_device *pdev)
{
	struct imxuart_platform_data *pdata = dev_get_platdata(&pdev->dev);

	sport->port.line = pdev->id;
	sport->devdata = (struct imx_uart_data	*) pdev->id_entry->driver_data;

	if (!pdata)
		return;

	if (pdata->flags & IMXUART_HAVE_RTSCTS)
		sport->have_rtscts = 1;
}

static int serial_imx_probe(struct platform_device *pdev)
{
	struct imx_port *sport;
	void __iomem *base;
	int ret = 0;
	struct resource *res;
         int txirq, rxirq, rtsirq;

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	ret = serial_imx_probe_dt(sport, pdev);
	if (ret > 0)
		serial_imx_probe_pdata(sport, pdev);
	else if (ret < 0)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

        rxirq = platform_get_irq(pdev, 0);
        txirq = platform_get_irq(pdev, 1);
        rtsirq = platform_get_irq(pdev, 2);
	sport->port.dev = &pdev->dev;
	sport->port.mapbase = res->start;
	sport->port.membase = base;
	sport->port.type = PORT_IMX,
	sport->port.iotype = UPIO_MEM;
        sport->port.irq = rxirq;
	sport->port.fifosize = 32;
	sport->port.ops = &imx_pops;
	sport->port.flags = UPF_BOOT_AUTOCONF;
	init_timer(&sport->timer);
	sport->timer.function = imx_timeout;
	sport->timer.data     = (unsigned long)sport;

	sport->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(sport->clk_ipg)) {
		ret = PTR_ERR(sport->clk_ipg);
		dev_err(&pdev->dev, "failed to get ipg clk: %d\n", ret);
		return ret;
	}

	sport->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(sport->clk_per)) {
		ret = PTR_ERR(sport->clk_per);
		dev_err(&pdev->dev, "failed to get per clk: %d\n", ret);
		return ret;
	}

	sport->port.uartclk = clk_get_rate(sport->clk_per);

         /*
          * Allocate the IRQ(s) i.MX1 has three interrupts whereas later
          * chips only have one interrupt.
          */
         if (txirq > 0) {
                 ret = devm_request_irq(&pdev->dev, rxirq, imx_rxint, 0,
                                        dev_name(&pdev->dev), sport);
                 if (ret)
                         return ret;
 
                 ret = devm_request_irq(&pdev->dev, txirq, imx_txint, 0,
                                        dev_name(&pdev->dev), sport);
                 if (ret)
                         return ret;
         } else {
                 ret = devm_request_irq(&pdev->dev, rxirq, imx_int, 0,
                                        dev_name(&pdev->dev), sport);
                 if (ret)
                         return ret;
         }
	imx_ports[sport->port.line] = sport;

	platform_set_drvdata(pdev, sport);

	dev_info(&pdev->dev, "Serial port %d added with Exor-imx-uart V%s driver \n", sport->port.line, DRIVER_VERSION);
	return uart_add_one_port(&imx_reg, &sport->port);
}

static int serial_imx_remove(struct platform_device *pdev)
{
	struct imx_port *sport = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "Serial port %d removed\n", sport->port.line);
	if (gpio_is_valid(sport->mode_gpio))
		gpio_free(sport->mode_gpio);
	if (gpio_is_valid(sport->rts_gpio))
		gpio_free(sport->rts_gpio);
	if (gpio_is_valid(sport->rxen_gpio))
		gpio_free(sport->rxen_gpio);
	if (gpio_is_valid(sport->be15mode_gpio))
		gpio_free(sport->be15mode_gpio);
	return uart_remove_one_port(&imx_reg, &sport->port);
}

static struct platform_driver serial_imx_driver = {
	.probe		= serial_imx_probe,
	.remove		= serial_imx_remove,

	.suspend	= serial_imx_suspend,
	.resume		= serial_imx_resume,
	.id_table	= imx_uart_devtype,
	.driver		= {
	.name	= "imx-exor-uart",
	.owner	= THIS_MODULE,
	.of_match_table = imx_uart_dt_ids,
	},
};

static int __init imx_serial_init(void)
{
	int ret;

	pr_info("Serial: Exor custom\n");

	ret = uart_register_driver(&imx_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&serial_imx_driver);
	if (ret != 0)
		uart_unregister_driver(&imx_reg);

	return ret;
}

static void __exit imx_serial_exit(void)
{
	platform_driver_unregister(&serial_imx_driver);
	uart_unregister_driver(&imx_reg);
}

module_init(imx_serial_init);
module_exit(imx_serial_exit);

MODULE_AUTHOR("Sascha Hauer");
MODULE_AUTHOR("Giuseppe Migliorini");
MODULE_AUTHOR("Luigi Scagnet");
MODULE_DESCRIPTION("Exor S-K serial port driver based on IMX generic ");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-uart");