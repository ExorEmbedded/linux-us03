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
#define CONFIG_SERIAL_IMX_EXOR_UART_MODULE 1

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
#define EXOR_MPI
#define USE_WAIT_TX_EMPTY

#define RETRY_IN_SAME_TOKEN
//#define	MANAGE_RR_ANSWERS

#ifdef EXOR_MPI
#include "linux/types.h"
#include <linux/hrtimer.h>
#include "../../../include/linux/time.h"
#include "../../../arch/arm/include/asm/ftrace.h"
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

#ifdef EXOR_MPI

//????????????????????#define UART_OMAP_TLR		0x07	/* FIFO trigger level register */
//????????????????????#define UART_RXFIFO_LVL		0x19	/* number of chars in RX fifo */
//????????????????????#define OMAP_TX_FIFO_LVL	0x1A	/* number of chars in TX fifo */

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;

#define MPI_MASTER_STACK 1
#define MPI_SLAVE_STACK 0
#define BUF_LEN  512

#define SET_MPI_MODE 0x54EF
#define GET_MPI_DIAG 0x54EE
#define SET_MPI_DIAG 0x54ED
#define MPI_OPEN	 0x54EC
#define MPI_RUN		 0x54EB
#define MPI_IS_OPEN	 0x54EA
#define MPI_CLOSE	 0x54E9
#define SET_MPI_DATA 0x54E8
#define SET_MPI_REQ	 0x54E7
#define GET_MPI_RQST 0x54E6
#define GET_MPI_RESP 0X54E5

struct s_MPIparams
{
	const char *uart;
	unsigned char panelNode;
	unsigned char maxNode;
	unsigned short applTimeout;
	unsigned short guardTimeoutConstant;
	unsigned short guardTimeoutFactor;
	unsigned short ackTimeout;
	unsigned short fdlTimeout;
	unsigned short tokTimeout;
	unsigned short selfTokTimeout;
};

/* Frame Control Functions                                              */
#define  D_SD1          0x10           /* Start delimeter FDL STATUS    */
#define  D_SD2          0x68           /* Start delimeter SRD Request Data Transfer */
#define  D_SD3          0xA2           /* Start delimeter               */
#define  D_ED           0x16           /* End delimeter                 */
#define  D_SC           0xE5           /* Short acknowledge             */
#define  D_TOK          0xDC           /* Token frame     (SD4)         */

/* Frame Control Functions                                              */
#define FCFC_SDAL            0x03      /* Send Data with Acknowledge Low*/
#define FCFC_SRDL            0x0C      /* Send and Request Data Low     */
#define FCFC_RFDL            0x09      /* Request FDL status            */

/* Frame Control byte masks                                             */
#define FCMSK_FCB            0x20      /* FCB mask                      */
#define FCMSK_FCV            0x10      /* FCV mask                      */
#define FCMSK_ST             0x30      /* Station Type mask             */
#define FCMSK_FT             0x40      /* Frame Type mask               */

#define FCST_SLAVE           0x00      /* Slave station                 */
#define FCST_MSNR            0x10      /* Master not ready for ring     */
#define FCST_MSRD            0x20      /* Master ready for ring         */
#define FCST_MSIR            0x30      /* Master in for ring            */

/* Reply Frame Control bytes (FC)                                       */
#define RFCMSK_CT            0x0F      /* Mask for frame control resp   */
#define RFC_OK               0x00      /* Acknowledge positive          */
#define RFC_DL               0x08      /* Response FDL/FMA1/2 Data low  */
#define RFC_DH               0x0A      /* Response FDL/FMA1/2 Data high */
#define RFC_TTNAK            0x02      /* NAK no resource */
#define RFC_RS_NAK           0x03      /* NAK no service activated */

#define NR_MAX_STATIONS 127

// master line states
#define NONE        0
#define SKIP        1

/* Config Par ----------------------------*/
#define N_MACCHINE   8
#define N_EVPOST	 256

#define FSM_TOK      0
#define FSM_LOG      1
#define FSM_JB0      2
#define FSM_JOB      3
#define FSM_LGS      4
#define FSM_J0S      5
#define FSM_JBS      6
#define FSM_LGF      7

/* Log Activation Flags ------------------*/
#define EVENTLOG	       0
#if EVENTLOG
   #define SUBSET_EVENTLOG  0
#endif
#define FUNCTLOG         0
#define SHORT_FORM       1

/* Events --------------------------------*/
#define EV__GAP 99
#define EV__NULL 0
#define EV_BRC_ENABLE 1
#define EV_BRC_ACK 2
#define EV_BRC_TOK 3
#define EV_BRC_SD1 4
#define EV_BRC_SD2 5
#define EV_BRC_SDX 6
#define EV_BRC_EOTX 7
#define EV_BRC_APPL_TIMEOUT 8
#define EV_BRC_ACK_TIMEOUT 9
#define EV_BRC_SL_SESS_TIMEOUT 10
#define EV_BRC_STOP_SESS 11
#define EV_BRC_J0S_SESS_TIMEOUT 12
#define EV_BRC_JBS_SESS_TIMEOUT 13
#define EV_SES_ACK 14
#define EV_SES_SD2 15
#define EV_SES_ERR 16
#define EV_SES_SD1 17
#define EV_SES_RETRY 18
#define EV_LOG_RUN 19
#define EV_LOG_START 20
#define EV_LOG_OFF 21
#define EV_TOK_RUN 22
#define EV_TOK_ACTIVEIDLE 23
#define EV_TOK_FDLSTATUS 24
#define EV_TOK_WAITSESSRX 25
#define EV_TOK_FDLTIMEOUT 26
#define EV_TOK_TIMEOUT 27
#define EV_TOK_TXFRAME 28
#define EV_TOK_PASSTOKEN 29
#define EV_TOK_SELFTOKEN 30
#define EV_TOK_TIMEOUT_TOK 31
#define EV_TOK_SELFTOK_TIMEOUT 32
#define EV_TOK_NO_RING_ACTIVITY 33
#define EV_JB0_RUN 34
#define EV_JB0_SEND_REQ 35
#define EV_JOB_RUN 36
#define EV_JOB_SEND_REQ 37
#define EV_LGS_RUN 38
#define EV_LGS_SEND_SAPRSP 39
#define EV_LGS_SEND_ACKSAPRSPACK 40
#define EV_J0S_RUN 41
#define EV_J0S_SEND_JOBACK 42
#define EV_J0S_SEND_JOBRSP 43
#define EV_JBS_RUN 44
#define EV_JBS_SEND_JOBACK 45
#define EV_JBS_SEND_JOBRSP 46
#define EV_LGF_RUN 47
#define EV_LGF_SEND_LOGOFFACK 48
#define EV_TX_DELAY_DONE 49
#define N_EVENTI 50

/* States --------------------------------*/
#define _NULL 0
#define TOK_IDLE 1
#define TOK_LISTENTOKEN 2
#define TOK_ACTIVEIDLE 3
#define TOK_WAITRX 4
#define TOK_TOKEN_RETRY 5
#define TOK_WAITFDLSTATUS 6
#define TOK_WAITFDLSTATUS2 7
#define TOK_WAITSESSRX 8
#define TOK_SELFTOKEN 9
#define LOG_IDLE 10
#define LOG_RUN 11
#define LOG_WAITSAPREQACK 12
#define LOG_WAITSAPRSP 13
#define LOG_WAITSAPRSPACK 14
#define LOG_WAITACKSAPRSPACK 15
#define JB0_IDLE 16
#define JB0_RUN 17
#define JB0_WAITSHORTACK0 18
#define JB0_WAITJOBACK0 19
#define JB0_WAITJOBRESP0 20
#define JB0_WAITSHORTACK02 21
#define JOB_IDLE 22
#define JOB_RUN 23
#define JOB_WAITSHORTACK 24
#define JOB_WAITJOBACK 25
#define JOB_WAITJOBRESP 26
#define JOB_WAITSHORTACK2 27
#define LGS_IDLE 28
#define LGS_RUN 29
#define LGS_SEND_SAPRSP 30
#define LGS_ACKSAPRSP 31
#define LGS_SEND_ACKSAPRSPACK 32
#define J0S_IDLE 33
#define J0S_RUN 34
#define J0S_SEND_JOBACK 35
#define J0S_SEND_JOBRSP 36
#define J0S_WAIT_ACK_JOBRSP 37
#define JBS_IDLE 38
#define JBS_RUN 39
#define JBS_SEND_JOBACK 40
#define JBS_SEND_JOBRSP 41
#define JBS_WAIT_ACK_JOBRSP 42
#define LGF_IDLE 43
#define LGF_RUN 44
#define LGF_SENT_LOGOFF_ACK 45
#define N_STATI 46

#define WR 0
#define RD 1
#define NR_JOB_RETRY 3

#define JOBACTION_REQ_LEN0 31


typedef enum
{
   NO_ERROR       =  0x00, /* initial value, before comm. starts        */
   NOT_ACCEPTED   =  0x01, /* request refused                           */
   M_PROC_RUNNING,
   M_PROC_OK,
   RESPONSE_NAK,           /* repeated NACKs from slave                 */
   TIMEOUT_ERR,            /* Not getting polled by Master              */
   RESPONSE_ERR,           /* error in response from slave              */
   GEN_COMM_ERR,           /* general communication error               */
   TIMEOUT_ERR2,           /* no response from slave to data request    */
   TIMEOUT_ERR3,           /* Timed out while sending request to PLC.   */
   RESPONSE_NAK2,          /* NAK from PLC.                             */
   LINE_ERROR,             /* Bad baud rate, parity, data bits etc      */
   RESPONSE_ERR2,          /* ill formed response from PLC              */
   RESPONSE_XOFF,          /* Timed out while waiting for XON           */
   RESPONSE_CTS,           /* Timed out while waiting for CTS           */
   COMM_CTRL_BLOCKED,      /* Comm blocked by FrameCommControlBlocked   */
   DRIVER_ERR              /* error in protocol driver                  */
} sendType;


byte const JobActionReq0[JOBACTION_REQ_LEN0] =
   {D_SD2, 0x19, 0x19, D_SD2, 0, 0, 0x7C, 0, 0,  //SD2 LE LER SD2 DA SA FC DAE SAE
	0xF1, 0,                                     //F1 jobnr
	0x32, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,    //APPL FIX PART
	0x08, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x01,    //
	0x00, 0x03, 0x00, 0xF0,                      //
	0, D_ED};                                    //FCS ED

#define JOBACTION2_REQ_LEN0 33
byte const JobAction2Req0[JOBACTION2_REQ_LEN0] =
   {D_SD2, 0x1B, 0x1B, D_SD2, 0, 0, 0x5C, 0, 0,  //SD2 LE LER SD2 DA SA FC DAE SAE
	0xF1, 0,                                     //F1 jobnr
	0x32, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00,    //APPL FIX PART
	0x08, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00,    //
	0x00, 0x01, 0x00, 0x03, 0x00, 0xF0,          //
	0, D_ED};                                    //FCS ED

#define JOBACTION_REQ_LEN 37
byte const JobActionReq[JOBACTION_REQ_LEN] =
   {D_SD2, 0x1F, 0x1F, D_SD2, 0, 0, 0x7C, 0, 0,  //SD2 LE LER SD2 DA SA FC DAE SAE
	0xF1, 0,                                     //F1 jobnr
	0x32, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,    //APPL FIX PART
	0x0E, 0x00, 0x00, 0x04, 0x01, 0x12, 0x0a,    //
	0x10, 0x05, 0x00, 0x01, 0x00, 0x00, 0x83,    //
	0x00, 0x00, 0x00,                            //
	0, D_ED};                                    //FCS ED

#define JOBACTION_ACK_LEN 14
byte const JobActionAck[JOBACTION_ACK_LEN] =
   {D_SD2, 0x08, 0x08, D_SD2, 0, 0, 0x7C, 0, 0,  //SD2 LE LER SD2 DA SA FC DAE SAE
	0xB0, 0x01, 0,                               //BO 1 jobnr
	0, D_ED};                                    //FCS ED

#define LOGON_SAPREQ_LEN 23
byte const LogOnSAPReq[LOGON_SAPREQ_LEN] =
   {D_SD2, 0x11, 0x11, D_SD2, 0, 0, 0x6D, 0, 0, //SD2 LE LER SD2 DA SA FC DAE SAE
	0xE0, 4, 0, 0x80, 0, 2, 0, 2, 2, 0, 2, 0,    //FIX PART
	0, D_ED};                                    //FCS ED

#define LOGON_SAPRSP_LEN 23
byte const LogOnSAPRsp[LOGON_SAPRSP_LEN] =
   {D_SD2, 0x11, 0x11, D_SD2, 0, 0, 0x6C, 0, 0, //SD2 LE LER SD2 DA SA FC DAE SAE
	0xD0, 4, 0, 0x80, 0, 2, 0, 2, 1, 0, 1, 0,    //FIX PART
//    0xD0, 4, 0, 0x80, 0, 2, 0, 2, 2, 0, 2, 0,    //FIX PART
	0, D_ED};                                    //FCS ED

#define LOGOFF_ACK 12
byte const LogOffAck[LOGOFF_ACK] =
   {D_SD2, 0x06, 0x06, D_SD2, 0, 0, 0x6D, 0, 0, //SD2 LE LER SD2 DA SA FC DAE SAE
	0xC0,                                       //FIX PART
	0, D_ED};                                   //FCS ED

#define LOGON_SAPRESPACK_LEN 13
byte const LogOnSAPRespAck[LOGON_SAPRESPACK_LEN] =
   {D_SD2, 0x07, 0x07, D_SD2, 0, 0, 0x5C, 0, 0, //SD2 LE LER SD2 DA SA FC DAE SAE
	0x05, 0x01,                                  //FIX PART
	0, D_ED};                                    //FCS ED

typedef struct
{
   byte  IsActive;         // 0 - station not present
						   // 1 - station present
   byte  StationType;      // 0 - slave station
						   // 1 - Master not ready to enter token ring
						   // 2 - Master ready to enter token ring (Ready FC=0x20)
						   // 3 - Master in token ring (Active FC=0x30)
   byte  FCV;              // 0 - alternating function of FCB is invalid
						   // 1 - alternating function of FCB is valid
   byte  FCB;              // Alternates between 0 and 1 for each new
						   // action frame
   byte  Logged;           // 0/1 LoggedOn/LoggedOff
   byte  LogStatus;        // status of logging machine	//MG001
   byte  Job;              // Job counter
   byte  LogOn_SAE;        // LogOnSAE
   byte  LogOn_DAE;        // LogOnDAE
} sStationStatus;

#define MAX_LENGTH_SEND 270                   /* max frame data length  */
#define MAX_LENGTH_RECV 270                   /* max frame data length  */

#define PASS_TOKEN_REPLY 4
#define FDLSTATUS        0
#define PASSTOKEN        1

struct s_MPIdata
{

	struct imx_port *mpiSport;
	bool m_isOpen;
	struct hrtimer hrt;
	byte UltiUart1_TxTimeout;
	int UltiUart1_TxNunUSec;


	int generateEOTEvent;
	int shortACK_EOTEvent;

	// from action.c
	byte NumTokenRotations;          /* num token rotations          */
	byte TokenNotForMe;
	byte LastStation;

	byte SessReqPending;
	byte SessionStarted;
	byte JobRetry;
	byte LogSessReqPending;
	byte LgfSessReqPending;
	byte Job0SessReqPending;
	byte JobSessReqPending;

	byte LgsSessReqPending;
	byte J0sSessReqPending;
	byte JbsSessReqPending;

	byte MyOperFlag;
	byte WrMyBuff[256];
	byte FrameMyBuff[256];
	word WrMyBuffLength;
	byte FrameMyBuffLength;
	word MyLen;

	unsigned char countArray[7];
	unsigned char interruptsCount[3];

	unsigned char ev_queue[N_EVPOST];
	unsigned char ev_queue_rd;
	unsigned char ev_queue_wr;
	unsigned char queue_empty;
	unsigned char event;
	unsigned char tok_state;
	unsigned char lgf_state;

	#if MPI_MASTER_STACK
	byte LogOn_Retry;
	unsigned char log_state;
	unsigned char jb0_state;
	unsigned char job_state;
	#endif
	#if MPI_SLAVE_STACK
	unsigned char lgs_state;
	unsigned char j0s_state;
	unsigned char jbs_state;
	byte JbsOnSlave_Job;
	#endif

	signed char  FrameUniopNodeNum;       /* Node of UniOP in network      */
	byte  FrameSendBufferLen1;
	byte  FrameSendBufferLen;
	byte  FrameSendBuffer[MAX_LENGTH_SEND + 10];       /* TX buffer */
	byte  FrameSendBuffer1[MAX_LENGTH_SEND + 10];       /* TX buffer */

	word  FrameSessionSendLength;
	byte  FrameSessionSendBuffer[MAX_LENGTH_SEND + 10];     //MG001
	word  FrameSessionReceiveLength;
	byte  FrameSessionReceiveBuffer[MAX_LENGTH_RECV + 10];  //MG001

	byte ProcedureApplMStatus;                 /* procedure global status      */
	bool ProcedureApplMRequestPending;         /* Waiting to send PLC a request*/
	bool ProcedureApplMResponsePending;        /* Waiting for answer from PLC  */
	sStationStatus StationStatus[NR_MAX_STATIONS]; /* the station statuses    */
	byte LowestAddress;                        /* Station for next GAP         */
	signed char NextGAP;
	bool AreWeInRing;
	signed char NextStation;
	signed char MaxStationAddress;
	word applResponseTime;
	word guardTimeConstant;
	word guardTimeFactor;
	word ackGuardTime;
	word FDLTimeout;
	word tokTimeout;
	word selfTokTimeout;
	byte Source;
	byte Dest;
	byte FlowCtrl;
	byte MyFrameResponseBuff[MAX_LENGTH_RECV + 10];
	word MyFrameResponseBuffLen;
	byte MyFrameRequestBuff[MAX_LENGTH_SEND + 10];
	word FrameJobSessionLen;
	byte FrameJobSessionBuff[MAX_LENGTH_SEND + 10];
	byte ReadyForConfigMode;
	byte SlaveSession;

	byte GapUpdateFactor;
	byte RingActivityCounter;

	byte LogOn_DA;
	byte LogOff_DA;
	byte Last_DA;
	byte LogOn_DAE;
	byte LogOn_SAE;
	byte CurrentShortAck;
	byte Start_LogOn_SAE;

	byte Start_LogOn_DAE;
	byte LogOnSlave_DA;

	byte startStopTrace;
	byte traceStatus;
	byte tracePostStopCnt;

	byte PassTokenReply;
	byte FlagSendSelfToken;
	byte FlagPassToken;
	byte Sd1RespGuard;

	byte GapUpdateFactorCnt;
	signed char NxtStat;
	byte cnt;
	unsigned char m_taskBuf[BUF_LEN];
	int m_taskLen;
	int applTryCnt;
	bool MPIenabled;
	bool MPImode;
	int rxCnt;
	unsigned char mpiRxBuf[280];
//	int txCnt;
//	int txIdx;
//	unsigned char mpiTxBuf[280];
	unsigned char localBuf[UART_XMIT_SIZE];
	struct circ_buf mpiTxBuf;
};
void ev_move(struct s_MPIdata *pMPIdata, unsigned char ev);

#endif

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
	unsigned long txfullflag;
#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
	struct s_SCNKparams SCNKparams;
	struct s_SCNKdata SCNKdata;
#ifdef EXOR_MPI
	struct s_MPIdata MPIdata;
#endif
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

#ifdef EXOR_MPI
struct ValueName
{
	int value;
	const char *name;
};

// event names
struct ValueName event_name[] =
{
	{ EV__NULL, "EV__NULL" },
	{ EV_BRC_ENABLE, "EV_BRC_ENABLE" },
	{ EV_BRC_ACK, "EV_BRC_ACK" },
	{ EV_BRC_TOK, "EV_BRC_TOK" },
	{ EV_BRC_SD1, "EV_BRC_SD1" },
	{ EV_BRC_SD2, "EV_BRC_SD2" },
	{ EV_BRC_SDX, "EV_BRC_SDX" },
	{ EV_BRC_EOTX, "EV_BRC_EOTX" },
	{ EV_BRC_APPL_TIMEOUT, "EV_BRC_APPL_TIMEOUT" },
	{ EV_BRC_ACK_TIMEOUT, "EV_BRC_ACK_TIMEOUT" },
	{ EV_BRC_SL_SESS_TIMEOUT, "EV_BRC_SL_SESS_TIMEOUT" },
	{ EV_BRC_STOP_SESS, "EV_BRC_STOP_SESS" },
	{ EV_BRC_J0S_SESS_TIMEOUT, "EV_BRC_J0S_SESS_TIMEOUT" },
	{ EV_BRC_JBS_SESS_TIMEOUT, "EV_BRC_JBS_SESS_TIMEOUT" },
	{ EV_SES_ACK, "EV_SES_ACK" },
	{ EV_SES_SD2, "EV_SES_SD2" },
	{ EV_SES_ERR, "EV_SES_ERR" },
	{ EV_SES_SD1, "EV_SES_SD1" },
	{ EV_SES_RETRY, "EV_SES_RETRY" },
	{ EV_LOG_RUN, "EV_LOG_RUN" },
	{ EV_LOG_START, "EV_LOG_START" },
	{ EV_LOG_OFF, "EV_LOG_OFF" },
	{ EV_TOK_RUN, "EV_TOK_RUN" },
	{ EV_TOK_ACTIVEIDLE, "EV_TOK_ACTIVEIDLE" },
	{ EV_TOK_FDLSTATUS, "EV_TOK_FDLSTATUS" },
	{ EV_TOK_WAITSESSRX, "EV_TOK_WAITSESSRX" },
	{ EV_TOK_FDLTIMEOUT, "EV_TOK_FDLTIMEOUT" },
	{ EV_TOK_TIMEOUT, "EV_TOK_TIMEOUT" },
	{ EV_TOK_TXFRAME, "EV_TOK_TXFRAME" },
	{ EV_TOK_PASSTOKEN, "EV_TOK_PASSTOKEN" },
	{ EV_TOK_SELFTOKEN, "EV_TOK_SELFTOKEN" },
	{ EV_TOK_TIMEOUT_TOK, "EV_TOK_TIMEOUT_TOK" },
	{ EV_TOK_SELFTOK_TIMEOUT, "EV_TOK_SELFTOK_TIMEOUT" },
	{ EV_TOK_NO_RING_ACTIVITY, "EV_TOK_NO_RING_ACTIVITY" },
	{ EV_JB0_RUN, "EV_JB0_RUN" },
	{ EV_JB0_SEND_REQ, "EV_JB0_SEND_REQ" },
	{ EV_JOB_RUN, "EV_JOB_RUN" },
	{ EV_JOB_SEND_REQ, "EV_JOB_SEND_REQ" },
	{ EV_LGS_RUN, "EV_LGS_RUN" },
	{ EV_LGS_SEND_SAPRSP, "EV_LGS_SEND_SAPRSP" },
	{ EV_LGS_SEND_ACKSAPRSPACK, "EV_LGS_SEND_ACKSAPRSPACK" },
	{ EV_J0S_RUN, "EV_J0S_RUN" },
	{ EV_J0S_SEND_JOBACK, "EV_J0S_SEND_JOBACK" },
	{ EV_J0S_SEND_JOBRSP, "EV_J0S_SEND_JOBRSP" },
	{ EV_JBS_RUN, "EV_JBS_RUN" },
	{ EV_JBS_SEND_JOBACK, "EV_JBS_SEND_JOBACK" },
	{ EV_JBS_SEND_JOBRSP, "EV_JBS_SEND_JOBRSP" },
	{ EV_LGF_RUN, "EV_LGF_RUN" },
	{ EV_LGF_SEND_LOGOFFACK, "EV_LGF_SEND_LOGOFFACK" },
	{ EV_TX_DELAY_DONE, "EV_TX_DELAY_DONE" },
};
// state names
struct ValueName state_name[] =
{
{ _NULL, "_NULL" },
{ TOK_IDLE, "TOK_IDLE" },
{ TOK_LISTENTOKEN, "TOK_LISTENTOKEN" },
{ TOK_ACTIVEIDLE, "TOK_ACTIVEIDLE" },
{ TOK_WAITRX, "TOK_WAITRX" },
{ TOK_TOKEN_RETRY, "TOK_TOKEN_RETRY" },
{ TOK_WAITFDLSTATUS, "TOK_WAITFDLSTATUS" },
{ TOK_WAITFDLSTATUS2, "TOK_WAITFDLSTATUS2" },
{ TOK_WAITSESSRX, "TOK_WAITSESSRX" },
{ TOK_SELFTOKEN, "TOK_SELFTOKEN" },
{ LOG_IDLE, "LOG_IDLE" },
{ LOG_RUN, "LOG_RUN" },
{ LOG_WAITSAPREQACK, "LOG_WAITSAPREQACK" },
{ LOG_WAITSAPRSP, "LOG_WAITSAPRSP" },
{ LOG_WAITSAPRSPACK, "LOG_WAITSAPRSPACK" },
{ LOG_WAITACKSAPRSPACK, "LOG_WAITACKSAPRSPACK" },
{ JB0_IDLE, "JB0_IDLE" },
{ JB0_RUN, "JB0_RUN" },
{ JB0_WAITSHORTACK0, "JB0_WAITSHORTACK0" },
{ JB0_WAITJOBACK0, "JB0_WAITJOBACK0" },
{ JB0_WAITJOBRESP0, "JB0_WAITJOBRESP0" },
{ JB0_WAITSHORTACK02, "JB0_WAITSHORTACK02" },
{ JOB_IDLE, "JOB_IDLE" },
{ JOB_RUN, "JOB_RUN" },
{ JOB_WAITSHORTACK, "JOB_WAITSHORTACK" },
{ JOB_WAITJOBACK, "JOB_WAITJOBACK" },
{ JOB_WAITJOBRESP, "JOB_WAITJOBRESP" },
{ JOB_WAITSHORTACK2, "JOB_WAITSHORTACK2" },
{ LGS_IDLE, "LGS_IDLE" },
{ LGS_RUN, "LGS_RUN" },
{ LGS_SEND_SAPRSP, "LGS_SEND_SAPRSP" },
{ LGS_ACKSAPRSP, "LGS_ACKSAPRSP" },
{ LGS_SEND_ACKSAPRSPACK, "LGS_SEND_ACKSAPRSPACK" },
{ J0S_IDLE, "J0S_IDLE" },
{ J0S_RUN, "J0S_RUN" },
{ J0S_SEND_JOBACK, "J0S_SEND_JOBACK" },
{ J0S_SEND_JOBRSP, "J0S_SEND_JOBRSP" },
{ J0S_WAIT_ACK_JOBRSP, "J0S_WAIT_ACK_JOBRSP" },
{ JBS_IDLE, "JBS_IDLE" },
{ JBS_RUN, "JBS_RUN" },
{ JBS_SEND_JOBACK, "JBS_SEND_JOBACK" },
{ JBS_SEND_JOBRSP, "JBS_SEND_JOBRSP" },
{ JBS_WAIT_ACK_JOBRSP, "JBS_WAIT_ACK_JOBRSP" },
{ LGF_IDLE, "LGF_IDLE" },
{ LGF_RUN, "LGF_RUN" },
{ LGF_SENT_LOGOFF_ACK, "LGF_SENT_LOGOFF_ACK" },
};

#ifdef DEBUG_STATE
#include <sys/time.h>
#include <string>       // std::string
#include <sstream>      // std::stringstream
#include <fstream>      // std::stringstream
#include <deque>

static const char *eventName(int ev)
{
	for (int i=0; i<sizeof(event_name)/sizeof(ValueName); ++i)
	{
		if (event_name[i].value == ev)
			return event_name[i].name;
	}
	return "unknown";
};

static const char *stateName(int state)
{
	for (int i=0; i<sizeof(state_name)/sizeof(ValueName); ++i)
	{
		if (state_name[i].value == state)
			return state_name[i].name;
	}
	return "unknown";
};

class MsgContainer
{
	class UsTimer
	{
	public:
		typedef unsigned int Value;
		UsTimer()
		{
			timespec time;
			clock_gettime( CLOCK_MONOTONIC, &time );
			m_startUs = time.tv_sec * 1000000 + time.tv_nsec/1000;
		}
		Value getUs() const
		{
			timespec time;
			clock_gettime( CLOCK_MONOTONIC, &time );
			Value us = time.tv_sec * 1000000 + time.tv_nsec/1000;
			return us - m_startUs;
		}
	private:
		Value m_startUs;
	};

	struct DiagMsg
	{
		UsTimer::Value time;
		std::string type;
		std::string msg;

		std::ostream &dump(std::ostream &ostr) const
		{
			ostr << std::setfill('0') << std::setw(8) << time << " " << type << " = " << msg << "\n";
			return ostr;
		}
	};

public:
	MsgContainer(int maxSize, bool cycling, bool running)
		: m_maxSize(maxSize),
		  m_cycling(cycling),
		  m_running(running)
	{
	}

	void run()
	{
		m_running = true;
		append("setMode", "run");
	}

	void stop()
	{
		append("setMode", "stop");
		m_running = false;
	}

	void append(const char *type, const char *msg)
	{
		if (m_running)
		{
			if (m_cycling && (m_messages.size() >= m_maxSize))
			{
				// make new place
				m_messages.pop_front();
			}
			if (m_messages.size() < m_maxSize)
			{
				// add new item
				DiagMsg diagMsg;
				diagMsg.time = usTimer.getUs();
				diagMsg.type = type;
				diagMsg.msg = msg;
				m_messages.push_back(diagMsg);
			}
		}
	}

	void append(const char *type, int value)
	{
		std::stringstream stream;
		stream << value;
		std::string str = stream.str();
		append(type, str.c_str());
	}

	void append(const char *type, unsigned char *arr, int len)
	{
		std::stringstream stream;
		for (int i=0; i<len; ++i)
		{
			stream << " " << std::hex << std::setfill('0') << std::setw(2) << (int)arr[i];
		}
		std::string str = stream.str();
		append(type, str.c_str());
	}

	friend std::ostream &operator <<(std::ostream &ostr, const MsgContainer &container);

private:
	UsTimer usTimer;
	std::deque<DiagMsg> m_messages;
	int m_maxSize;
	bool m_cycling;
	bool m_running;
};

inline std::ostream &operator <<(std::ostream &ostr, const MsgContainer &container)
{
	for (std::deque<MsgContainer::DiagMsg>::const_iterator it = container.m_messages.begin(); it != container.m_messages.end(); ++it)
		it->dump(ostr);
	return ostr;
}

static MsgContainer msgContainer(10000, true, true);

static void dumpRun()
{
	msgContainer.run();
}

static void dumpStop()
{
	msgContainer.stop();
}

static void dumpEvent(int ev)
{
	msgContainer.append("EVENT", eventName(ev));
}

static void dumpTokState(int tokState)
{
	static int old = -1;
	if (old != tokState)
	{
		old = tokState;
		msgContainer.append("TOK_STATE", stateName(tokState));
	}
}

static void dumpLogState(int state)
{
	static int old = -1;
	if (old != state)
	{
		old = state;
		msgContainer.append("LOG_STATE", stateName(state));
	}
}

static void dumpJb0State(int state)
{
	static int old = -1;
	if (old != state)
	{
		old = state;
		msgContainer.append("JB0_STATE", stateName(state));
	}
}

static void dumpJobState(int state)
{
	static int old = -1;
	if (old != state)
	{
		old = state;
		msgContainer.append("JOB_STATE", stateName(state));
	}
}

static void dumpInteger(const char *name, int n)
{
	msgContainer.append(name, n);
}

static void dumpTimeout(int type, int value)
{
	std::stringstream stream;
	stream << "TIMEOUT_" << type;
	std::string str = stream.str();
	msgContainer.append(str.c_str(), value);
}

static void dumpArray(const char *type, unsigned char *arr, int len)
{
	msgContainer.append(type, arr, len);
}

static void dumpDebug(const char *debug)
{
	msgContainer.append("DEBUG", debug);
}

static void printDiagMessages()
{
	// std::cout << msgContainer;
	std::ofstream file("diag.txt", std::ofstream::out);
	if (file.good())
	{
		file << msgContainer;
		file.close();
	}
}

#else
#define dumpRun()
#define dumpStop()
#define dumpEvent(x)
#define dumpTokState(x)
#define dumpLogState(x)
#define dumpJb0State(x)
#define dumpJobState(x)
#define dumpInteger(x, y)
#define dumpTimeout(x, y)
#define dumpDebug(x)
#define dumpArray(x, y, z)
#define printDiagMessages()
#endif // DEBUG_STATE

static void CalcCheckSum(unsigned char *buf, int start, int fcsPos)
{
	unsigned char sum = 0;
	int i;
	for (i=start; i<fcsPos; ++i)
		sum += buf[i];
	buf[fcsPos] = sum;
}

static void RtlCopyMemory(unsigned char *dest, const unsigned char *src, int len)
{
	memcpy(dest, src, len);
}
void UltiUART1_StartTimer(struct s_MPIdata *pMPIdata, unsigned char ev, int timeoutUSec);
static void mpiReceiveFrame(struct s_MPIdata *pMPIdata, unsigned char *buf, unsigned int len);

enum hrtimer_restart hrtCallBack(struct hrtimer *phrt)
{
	struct s_MPIdata *pMPIdata = container_of(phrt, struct s_MPIdata, hrt);
	unsigned long flags=0;
//	struct imx_port *sport = container_of(pMPIdata, struct imx_port, MPIdata);
	spin_lock_irqsave(&pMPIdata->mpiSport->port.lock, flags);
	if (pMPIdata->UltiUart1_TxTimeout == EV__GAP) {
		if (pMPIdata->rxCnt) {
//			printk("MPI frame RX %d\n", pMPIdata->rxCnt);
			mpiReceiveFrame(pMPIdata, pMPIdata->mpiRxBuf, pMPIdata->rxCnt);
//			pMPIdata->rxCnt = 0;
		}
	}
	else
		ev_move(pMPIdata, pMPIdata->UltiUart1_TxTimeout);
	spin_unlock_irqrestore(&pMPIdata->mpiSport->port.lock, flags);
	return HRTIMER_NORESTART;
}

void MPIDriverInit(struct s_MPIdata *pMPIdata)
{
	hrtimer_init(&pMPIdata->hrt, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	pMPIdata->hrt.function = hrtCallBack;
	pMPIdata->MPIenabled = false;

	// initialize variables (previous globals)
	pMPIdata->NumTokenRotations = 0;
	pMPIdata->TokenNotForMe = 0;
	pMPIdata->LastStation = 0;

	pMPIdata->SessReqPending = false;
	pMPIdata->SessionStarted = false;
	pMPIdata->JobRetry = 0;
	pMPIdata->LogSessReqPending = false;
	pMPIdata->LgfSessReqPending = false;
	pMPIdata->Job0SessReqPending = false;
	pMPIdata->JobSessReqPending = false;

	pMPIdata->LgsSessReqPending = false;
	pMPIdata->J0sSessReqPending = false;
	pMPIdata->JbsSessReqPending = false;

	pMPIdata->MyOperFlag = 0;
	memset(pMPIdata->WrMyBuff, 0, sizeof(pMPIdata->WrMyBuff));
	memset(pMPIdata->FrameMyBuff, 0, sizeof(pMPIdata->FrameMyBuff));
	pMPIdata->WrMyBuffLength = 0;
	pMPIdata->FrameMyBuffLength = 0;
	pMPIdata->MyLen = 0;

	memset(pMPIdata->countArray, 0, sizeof(pMPIdata->countArray));
	memset(pMPIdata->interruptsCount, 0, sizeof(pMPIdata->interruptsCount));

	memset(pMPIdata->ev_queue, 0, sizeof(pMPIdata->ev_queue));
	pMPIdata->ev_queue_rd = 0;
	pMPIdata->ev_queue_wr = 0;
	pMPIdata->queue_empty = 1;
	pMPIdata->event = 0;
	pMPIdata->tok_state = TOK_IDLE;
	pMPIdata->lgf_state = LGF_IDLE;

#if MPI_MASTER_STACK
	pMPIdata->LogOn_Retry = 0;
	pMPIdata->log_state = LOG_IDLE;
	pMPIdata->jb0_state = JB0_IDLE;
	pMPIdata->job_state = JOB_IDLE;
#endif
#if MPI_SLAVE_STACK
	pMPIdata->lgs_state = LGS_IDLE;
	pMPIdata->j0s_state = J0S_IDLE;
	pMPIdata->jbs_state = JBS_IDLE;
	pMPIdata->JbsOnSlave_Job = 0;
#endif

	pMPIdata->FrameUniopNodeNum = 0;
	pMPIdata->FrameSendBufferLen1 = 0;
	pMPIdata->FrameSendBufferLen = 0;
	pMPIdata->FrameSessionSendLength = 0;
	pMPIdata->FrameSessionReceiveLength = 0;

	pMPIdata->ProcedureApplMStatus = 0;
	pMPIdata->ProcedureApplMRequestPending = 0;
	pMPIdata->ProcedureApplMResponsePending = 0;
	memset(pMPIdata->StationStatus, 0, sizeof(pMPIdata->StationStatus));
	pMPIdata->LowestAddress = 0;
	pMPIdata->NextGAP = 0;
	pMPIdata->AreWeInRing = 0;
	pMPIdata->NextStation = 0;
	pMPIdata->MaxStationAddress = 0;
	pMPIdata->applResponseTime = 0;
	pMPIdata->guardTimeConstant = 0;
	pMPIdata->guardTimeFactor = 0;
	pMPIdata->ackGuardTime = 0;
	pMPIdata->FDLTimeout = 0;
	pMPIdata->tokTimeout = 0;
	pMPIdata->selfTokTimeout = 0;
	pMPIdata->Source = 0;
	pMPIdata->Dest = 0;
	pMPIdata->FlowCtrl = 0;
	pMPIdata->MyFrameResponseBuffLen = 0;
	pMPIdata->FrameJobSessionLen = 0;
	pMPIdata->ReadyForConfigMode = 0;
	pMPIdata->SlaveSession = 0;

	pMPIdata->GapUpdateFactor = 0;
	pMPIdata->job_state = 0;
	pMPIdata->RingActivityCounter = 0;

	pMPIdata->LogOn_DA = 0;
	pMPIdata->LogOff_DA = 0;
	pMPIdata->Last_DA = 0;
	pMPIdata->LogOn_DAE = 0;
	pMPIdata->LogOn_SAE = 0;
	pMPIdata->CurrentShortAck = 0;
	pMPIdata->Start_LogOn_SAE = 0;

	pMPIdata->Start_LogOn_DAE = 0;
	pMPIdata->LogOnSlave_DA = 0;

#if MPI_SLAVE_STACK
	pMPIdata->lgs_state = 0;
	pMPIdata->j0s_state = 0;
	pMPIdata->jbs_state = 0;
#endif
	pMPIdata->job_state = 0;
	pMPIdata->log_state = 0;
	pMPIdata->lgf_state = 0;
	pMPIdata->jb0_state = 0;

	pMPIdata->startStopTrace = 0;
	pMPIdata->traceStatus = 0;
	pMPIdata->tracePostStopCnt = 0;

	pMPIdata->PassTokenReply    = 0;
	pMPIdata->FlagSendSelfToken = 0;
	pMPIdata->FlagPassToken     = 0;
	pMPIdata->Sd1RespGuard      = 0;

	pMPIdata->GapUpdateFactorCnt = 0;
	pMPIdata->NxtStat = 0;
	pMPIdata->cnt = 0;
	pMPIdata->generateEOTEvent = 0;
	pMPIdata->shortACK_EOTEvent = 0;
	pMPIdata->mpiTxBuf.buf = pMPIdata->localBuf;
	pMPIdata->mpiTxBuf.head = pMPIdata->mpiTxBuf.tail = 0;
	pMPIdata->MPImode = true;
}


void DBGReg(byte event, byte status, word param)
{
}
#define MAKEWORD(a, b)     ((word)(((byte)(a)) | ((word)((byte)(b))) << 8))
void ev_post(struct s_MPIdata *pMPIdata, unsigned char ev);

static void imx_start_tx(struct uart_port *port);
static inline void imx_transmit_buffer(struct imx_port *sport);
static void imx_rs485_start_tx(struct imx_port *sport);

static void sendData(struct s_MPIdata *pMPIdata, char * buf, unsigned int len)
{
	int i;
	struct imx_port *sport = pMPIdata->mpiSport;

//	printk("MPI frame TX %d\n", len);

	if (len == 1)
		pMPIdata->shortACK_EOTEvent = 1;

	for (i=0; i<len; i++)
	{
		sport->MPIdata.mpiTxBuf.buf[sport->MPIdata.mpiTxBuf.head] = buf[i];
		sport->MPIdata.mpiTxBuf.head = (sport->MPIdata.mpiTxBuf.head + 1) & (UART_XMIT_SIZE-1);
	}
	imx_start_tx(&sport->port);
}


void sendShortAck(struct s_MPIdata *pMPIdata)
{
	unsigned char c = D_SC;
	sendData(pMPIdata, &c, 1);
}

void UltiUART1_StartTimer(struct s_MPIdata *pMPIdata, unsigned char ev, int timeoutUSec)
{
	ktime_t kt = ktime_set(timeoutUSec / 1000000, (timeoutUSec % 1000000)*1000);
   pMPIdata->UltiUart1_TxTimeout = ev;
   pMPIdata->UltiUart1_TxNunUSec = timeoutUSec;
   hrtimer_start( &pMPIdata->hrt, kt, HRTIMER_MODE_REL );
}

void UltiUart1_StopTimer(struct s_MPIdata *pMPIdata)
{
	hrtimer_try_to_cancel(&pMPIdata->hrt);
	pMPIdata->UltiUart1_TxTimeout = EV__NULL;
}

/************************************************************************
*
* mInitEngine
*
*  This function Init the Stations and state machines status
*
* Parameters:
*  void
*
* Returns:
***********************************************************************/
void mInitEngine(struct s_MPIdata *pMPIdata)
{
   int i;
   // Initialise all the stations to inactive
   for (i = 0; i < NR_MAX_STATIONS; i++)
   {
	  pMPIdata->StationStatus[i].IsActive = 0;
	  pMPIdata->StationStatus[i].StationType = 0;
	  pMPIdata->StationStatus[i].FCV = 0;
	  pMPIdata->StationStatus[i].FCB = 1;
	  pMPIdata->StationStatus[i].Logged = 0;
	  pMPIdata->StationStatus[i].LogStatus = 0;   //MG001
	  pMPIdata->StationStatus[i].Job = 0;
	  pMPIdata->StationStatus[i].LogOn_SAE = pMPIdata->Start_LogOn_SAE++;
	  pMPIdata->StationStatus[i].LogOn_DAE = 0;
   }
   //reset all other state machines
   pMPIdata->lgf_state = LGF_IDLE;
   pMPIdata->log_state = LOG_IDLE;
   pMPIdata->jb0_state = JB0_IDLE;
   pMPIdata->job_state = JOB_IDLE;
#if MPI_SLAVE_STACK
   pMPIdata->lgs_state = LGS_IDLE;
   pMPIdata->j0s_state = J0S_IDLE;
   pMPIdata->jbs_state = JBS_IDLE;
#endif
   pMPIdata->tok_state = TOK_IDLE;
}

void Init_ev_queue(struct s_MPIdata *pMPIdata)
{
	memset(pMPIdata->ev_queue, 0, sizeof(pMPIdata->ev_queue));
	pMPIdata->ev_queue_rd = 0;
	pMPIdata->ev_queue_wr = 0;
	pMPIdata->queue_empty = 1;
}


// Post event
void ev_post(struct s_MPIdata *pMPIdata, unsigned char ev)
{
	pMPIdata->ev_queue[pMPIdata->ev_queue_rd] = ev;
	pMPIdata->ev_queue_rd = (pMPIdata->ev_queue_rd + 1) % N_EVPOST;
	pMPIdata->queue_empty = 0;
}

/************************************************************************
*
* ResetStations
*
*  This function reset the Stations Status
*
* Parameters:
*  void
*
* Returns:
***********************************************************************/
void ResetStations(struct s_MPIdata *pMPIdata)
{
byte i;

   for (i = 0; i < NR_MAX_STATIONS; i++)
   {
	  pMPIdata->StationStatus[i].IsActive = 0;
	  pMPIdata->StationStatus[i].StationType = 0;
	  pMPIdata->StationStatus[i].Job = 0;
	  pMPIdata->StationStatus[i].Logged = 0;
	  pMPIdata->StationStatus[i].LogOn_DAE = 0;
	  pMPIdata->StationStatus[i].LogStatus = 0; //MG001
   }
   pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
   pMPIdata->NextStation = -1;
   pMPIdata->AreWeInRing = false;
   pMPIdata->m_taskLen = 0;
   Init_ev_queue(pMPIdata);
}

#ifdef RETRY_IN_SAME_TOKEN
void ResetLastDAStation(struct s_MPIdata *pMPIdata)
{
	pMPIdata->StationStatus[pMPIdata->Last_DA].IsActive = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].StationType = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].Logged = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].LogStatus = 0; //MG001
}
#endif
/************************************************************************
*
* PrepareNextGAP
*
*  This function returns the next station address in the GAP between
*  us and the next active station. The NextGAP station is a static
*  variable that is incremented by this function.
*
* Parameters:
*  void
*
* Returns:
*  -1 - no GAP
*  else - next GAP station
***********************************************************************/
#define MON_DEBUG 0
signed char PrepareNextGAP(struct s_MPIdata *pMPIdata)
{
	int i;

   #if MON_DEBUG
	  putCh('a');
   #endif

   if (pMPIdata->NextGAP == pMPIdata->MaxStationAddress)
	  pMPIdata->NextGAP = -1;

   for (i = 0; i < pMPIdata->MaxStationAddress; i++)
   {
	  #if MON_DEBUG
		 putHexByte(pMPIdata->NextGAP);
	  #endif
	  if (pMPIdata->StationStatus[pMPIdata->NextGAP + 1].IsActive && (pMPIdata->StationStatus[pMPIdata->NextGAP + 1].StationType >= 2))
	  {
		 if ((pMPIdata->NextGAP + 1) == (pMPIdata->FrameUniopNodeNum + 1))
		 {
			#if MON_DEBUG
			   putCh('d');
			#endif
			pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
			return -1;
		 }
		 pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum + 1;
		 #if MON_DEBUG
			putCh('b');
		 #endif
		 return pMPIdata->NextGAP;
	  }
	  else
	  {
		 if ((pMPIdata->NextGAP + 1) > pMPIdata->MaxStationAddress)
		 {
			pMPIdata->NextGAP = -1;
			#if MON_DEBUG
			   putCh('e');
			#endif
		 }
		 else
		 {
			if ((pMPIdata->NextGAP + 1) == pMPIdata->FrameUniopNodeNum)
			{
			   pMPIdata->NextGAP++;
			   #if MON_DEBUG
				  putCh('f');
			   #endif
			}
			else
			{
			   #if MON_DEBUG
				  putCh('g');
			   #endif
			   pMPIdata->NextGAP += 1;
			   return pMPIdata->NextGAP;
			}
		 }
	  }
   }
   return -1;	//MG001
}

/************************************************************************
*
* GetNextActiveStation
*
*  This function checks the LAS to determine the next active station
*  relative to us
*
* Parameters:
*  void
*
* Returns:
*  next active station
************************************************************************/
signed char GetNextActiveStation(struct s_MPIdata *pMPIdata)
{
	signed char  i;

   // Search over the whole array of station addresses starting from
   // StartNode to find the next master station, i.e. a station
   // registered as In the token ring or ready for it
   i = pMPIdata->FrameUniopNodeNum + 1;
   while (i != pMPIdata->FrameUniopNodeNum)
   {
	  if (i > pMPIdata->MaxStationAddress)
	  {
		 if (pMPIdata->FrameUniopNodeNum == 0)
			break;
		 else
			i = 0;
	  }
	  if (pMPIdata->StationStatus[i].IsActive && (pMPIdata->StationStatus[i].StationType >= 2))
		 return i;
	  i++;
   }
   return pMPIdata->FrameUniopNodeNum;
}

#if MPI_MASTER_STACK
//MG003 Start
void mExitFromRing(struct s_MPIdata *pMPIdata)
{

   pMPIdata->SessReqPending     = false;
   pMPIdata->SessionStarted     = false;
   pMPIdata->LgsSessReqPending  = false;
   pMPIdata->Job0SessReqPending = false;
   pMPIdata->JobSessReqPending  = false;

//   ProcedureApplMStatus = TIMEOUT_ERR;
#ifdef FATAL_ERRORS
   pMPIdata->ProcedureApplMStatus = GEN_COMM_ERR;
   DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0001);
   pMPIdata->ProcedureApplMRequestPending  = false;
   pMPIdata->ProcedureApplMResponsePending = false;
#else
   if (pMPIdata->ProcedureApplMResponsePending)
	  pMPIdata->ProcedureApplMRequestPending = true;
#endif

   ResetStations(pMPIdata);

   //reset all other state machines
   pMPIdata->lgf_state = LGF_IDLE;
   pMPIdata->log_state = LOG_IDLE;
   pMPIdata->jb0_state = JB0_IDLE;
   pMPIdata->job_state = JOB_IDLE;
#if MPI_SLAVE_STACK
   pMPIdata->lgs_state = LGS_IDLE;
   pMPIdata->j0s_state = J0S_IDLE;
   pMPIdata->jbs_state = JBS_IDLE;
#endif
   DBGReg(242, pMPIdata->tok_state, TOK_IDLE);
   pMPIdata->tok_state = TOK_IDLE;
   dumpTokState(pMPIdata->tok_state);

   UltiUART1_StartTimer(pMPIdata, EV_TOK_RUN,1000*1000);
}
//MG003 End


/*----LogOn--------------------------------*/
#define MAX_LOGON_RETRY 5

void mLogSendSAPReqFrame(struct s_MPIdata *pMPIdata)
{
   RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, LogOnSAPReq, LOGON_SAPREQ_LEN);
   pMPIdata->FrameSessionSendBuffer[4] = pMPIdata->LogOn_DA | 0x80;
   pMPIdata->FrameSessionSendBuffer[5] = pMPIdata->FrameUniopNodeNum | 0x80;
   pMPIdata->FrameSessionSendBuffer[7] = pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_DAE;
   pMPIdata->FrameSessionSendBuffer[8] = pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_SAE;
   pMPIdata->FrameSessionSendLength = LOGON_SAPREQ_LEN;

   //Calcola FCS
   CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

   pMPIdata->LogSessReqPending = true;
   pMPIdata->SessReqPending    = true;
}

byte LogOn_RespFrameOk(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] != 0xD0)
   {
	  pMPIdata->StationStatus[pMPIdata->LogOn_DA].Job = 0;
	  return 0;
   }
   pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_DAE = pMPIdata->FrameSessionReceiveBuffer[8];
   return 1;
}

byte LogOn_AckRespFrameOk(struct s_MPIdata *pMPIdata)
{
   pMPIdata->StationStatus[pMPIdata->LogOn_DA].Logged = 1;
   pMPIdata->StationStatus[pMPIdata->LogOn_DA].Job   = 0;
   return 1;
}


unsigned char mLogAbortSession(struct s_MPIdata *pMPIdata)
{
   mExitFromRing(pMPIdata); //MG003
   return LOG_IDLE; //MG003
}


unsigned char mLogCheckLogOff(struct s_MPIdata *pMPIdata)
{
   //Exit from Ring!!!!
   if (pMPIdata->LogOn_Retry > MAX_LOGON_RETRY)
   {
	  pMPIdata->LogOn_Retry = 0;
	  ev_post(pMPIdata, EV_LOG_OFF);
	  pMPIdata->NumTokenRotations = 0;
	  pMPIdata->LowestAddress = 0xFF;
	  return LOG_RUN;
   }

   if (pMPIdata->LogSessReqPending && (pMPIdata->FrameSessionReceiveBuffer[9] == 0x80))
	  pMPIdata->LogOn_Retry++;

	mLogAbortSession(pMPIdata);
	return LOG_RUN;
}

unsigned char mLogSendRespAck(struct s_MPIdata *pMPIdata)
{
   if (LogOn_RespFrameOk(pMPIdata))
   {
	  //Send Short Ack
	  sendShortAck(pMPIdata);

	  pMPIdata->SessReqPending = true;

	  //Prepare next frame
	  pMPIdata->FrameSessionSendLength = LOGON_SAPRESPACK_LEN;
	  RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, LogOnSAPRespAck, LOGON_SAPRESPACK_LEN);
	  pMPIdata->FrameSessionSendBuffer[4] = pMPIdata->LogOn_DA | 0x80;
	  pMPIdata->FrameSessionSendBuffer[5] = pMPIdata->FrameUniopNodeNum | 0x80;
	  pMPIdata->FrameSessionSendBuffer[7] = pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_DAE;
	  pMPIdata->FrameSessionSendBuffer[8] = pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_SAE;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

	  return LOG_WAITSAPRSPACK;
   }
   return mLogCheckLogOff(pMPIdata);
}

unsigned char mLogWaitAckRespAck(struct s_MPIdata *pMPIdata)
{
	if (LogOn_AckRespFrameOk(pMPIdata))
	{
	  sendShortAck(pMPIdata);

	  //Success!!!
	  pMPIdata->LogSessReqPending = false;
	  pMPIdata->LogOn_Retry = 0;
	  return LOG_RUN;
	}
	return mLogCheckLogOff(pMPIdata);
}

/*----LogOff----------------------------*/
unsigned char mLgfCheckLogOff(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->SlaveSession)   //MG001
	  return LGF_RUN;  //MG001

   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0x80)
   {
	  mExitFromRing(pMPIdata); //MG003
	  return LGF_IDLE; //MG003

   }
   return LGF_RUN;
}

void mLgfSendLogOffAck(struct s_MPIdata *pMPIdata)
{
   pMPIdata->SessReqPending = true;
}

void mLgfAbortSession(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->LgfSessReqPending)
   {
	  pMPIdata->LgfSessReqPending = false;
	  pMPIdata->SessReqPending    = false;
	  pMPIdata->SessionStarted    = false;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].Logged = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].Job   = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].LogOn_DAE = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].LogStatus = 0;	//MG001
   }
}

unsigned char mLgfCheckSD1(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->Dest == pMPIdata->FrameUniopNodeNum)
   {
	  pMPIdata->LgfSessReqPending = false;
	  pMPIdata->SessReqPending    = false;
	  pMPIdata->SessionStarted    = false;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].Logged = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].Job   = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].LogOn_DAE = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].LogStatus = 0;	//MG001
	  return LGF_RUN;
   }
   return LGF_SENT_LOGOFF_ACK;
}

/*--- JobAction & JobAction0 common functions -------------*/
char JobAckFrameOk(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] != 0xB0)
   {
	  pMPIdata->StationStatus[pMPIdata->FrameSessionSendBuffer[4] & 0x7F].Job = 0;
	  return 0;
   }
   return 1;
}

/*----JobAction0----------------------------*/
void mJob0ApplicationResponseErr(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->Job0SessReqPending)
   {
	  pMPIdata->Job0SessReqPending = false;
	  pMPIdata->SessReqPending     = false;
	  pMPIdata->SessionStarted     = false;
	  pMPIdata->LogSessReqPending  = false;
	  pMPIdata->ProcedureApplMStatus = GEN_COMM_ERR;
	  DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0002);
	  pMPIdata->ProcedureApplMRequestPending  = false;
	  pMPIdata->ProcedureApplMResponsePending = false;
	  pMPIdata->StationStatus[pMPIdata->Last_DA].LogStatus = 1;
   }
}

void mJob0SendReq(struct s_MPIdata *pMPIdata)
{
   RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, JobActionReq0, JOBACTION_REQ_LEN0);
   pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
   pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
   pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
   pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
   pMPIdata->FrameSessionSendBuffer[10] = 0; //Job = 0
   pMPIdata->FrameSessionSendLength = JOBACTION_REQ_LEN0;

   //Calcola FCS
   CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

   pMPIdata->Job0SessReqPending = true;
   pMPIdata->SessReqPending = true;
}

unsigned char mJob0SendShortAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xB0)
   {
	  sendShortAck(pMPIdata);
	  return JB0_WAITJOBRESP0;
   }

   pMPIdata->StationStatus[pMPIdata->FrameSessionSendBuffer[4] & 0x7F].Job = 0;
   mJob0ApplicationResponseErr(pMPIdata);
   return JB0_RUN;
}

unsigned char mJob0SendJobAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xF1)
   {
	  sendShortAck(pMPIdata);

	  pMPIdata->FrameSessionSendBuffer[0]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[1]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[2]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[3]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
	  pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
	  pMPIdata->FrameSessionSendBuffer[6]  = 0x5C;
	  pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
	  pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
	  pMPIdata->FrameSessionSendBuffer[9]  = 0xB0;
	  pMPIdata->FrameSessionSendBuffer[10] = 0x01;
	  pMPIdata->FrameSessionSendBuffer[11] = 0;
	  pMPIdata->FrameSessionSendBuffer[13] = D_ED;
	  pMPIdata->FrameSessionSendLength = JOBACTION_ACK_LEN;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

	  pMPIdata->SessReqPending = true;
	  return JB0_WAITSHORTACK02;
   }

   mJob0ApplicationResponseErr(pMPIdata);
   return JB0_RUN;
}
void mJob0Ok(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 0xFF) pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 1;
   else pMPIdata->StationStatus[pMPIdata->Last_DA].Job++;

   pMPIdata->Job0SessReqPending = false;
}
/*----JobAction----------------------------*/
void mJobApplicationResponse(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->JobSessReqPending)
   {
	  pMPIdata->JobSessReqPending = false;
	  pMPIdata->SessReqPending    = false;
	  pMPIdata->SessionStarted    = false;
	  if (pMPIdata->MyFrameResponseBuff[14] != 0xFF)
	  {
		 pMPIdata->ProcedureApplMStatus = RESPONSE_NAK2;
		 DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0003);
	  }
	  else
		 pMPIdata->ProcedureApplMStatus = NO_ERROR;
	  pMPIdata->ProcedureApplMRequestPending  = false;
	  pMPIdata->ProcedureApplMResponsePending = false;
   }
}

void mJobApplicationResponseErr(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->JobSessReqPending)
   {
	  pMPIdata->Job0SessReqPending = false;
	  pMPIdata->SessReqPending     = false;
	  pMPIdata->SessionStarted     = false;
	  pMPIdata->LogSessReqPending  = false;
	  pMPIdata->ProcedureApplMStatus = GEN_COMM_ERR;
	  DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0004);
	  pMPIdata->ProcedureApplMRequestPending  = false;
	  pMPIdata->ProcedureApplMResponsePending = false;
   }
}

void mJobSendReq(struct s_MPIdata *pMPIdata)
{
   pMPIdata->FrameJobSessionBuff[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
   pMPIdata->FrameJobSessionBuff[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
   pMPIdata->FrameJobSessionBuff[10] = pMPIdata->StationStatus[pMPIdata->Last_DA].Job;

   //Calcola FCS
   CalcCheckSum(pMPIdata->FrameJobSessionBuff, 4, pMPIdata->FrameJobSessionLen - 2);
   pMPIdata->FrameJobSessionBuff[pMPIdata->FrameJobSessionLen-1] = D_ED;

   RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, pMPIdata->FrameJobSessionBuff, pMPIdata->FrameJobSessionLen);   //MG001
   pMPIdata->FrameSessionSendLength =pMPIdata-> FrameJobSessionLen;                               //MG001

   pMPIdata->JobSessReqPending = true;
   pMPIdata->SessReqPending    = true;
   pMPIdata->JobRetry = 0;
}

unsigned char  mJobRetryReq(struct s_MPIdata *pMPIdata)
{
   if (++pMPIdata->JobRetry > NR_JOB_RETRY)
   {
	  mJobApplicationResponseErr(pMPIdata);
	  return JOB_RUN;
   }
   else
   {
	  RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, pMPIdata->FrameJobSessionBuff, pMPIdata->FrameJobSessionLen);   //MG001
	  pMPIdata->FrameSessionSendLength =pMPIdata->FrameJobSessionLen;                               //MG001
	  pMPIdata->JobSessReqPending = true;
	  pMPIdata->SessReqPending    = true;
	  return JOB_WAITSHORTACK;
   }
}


unsigned char mJobSendShortAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0x80)  //MG001
	  return JOB_WAITJOBACK;                  //MG001

   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xB0)
   {
	  sendShortAck(pMPIdata);
	  return JOB_WAITJOBRESP;
   }
   DBGReg(241, pMPIdata->FrameSessionReceiveBuffer[9], 0);
   return JOB_WAITJOBACK;                  //MG001
}

unsigned char mJobSendJobAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0x80)  //MG001
	  return JOB_WAITJOBRESP;                 //MG001

   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xB0)
   {
	  sendShortAck(pMPIdata);
	  DBGReg(240, pMPIdata->FrameSessionReceiveBuffer[9], 0);
	  return JOB_WAITJOBRESP;
   }
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xF1)
   {
	  pMPIdata->MyFrameResponseBuffLen = pMPIdata->FrameSessionReceiveBuffer[1]-7;
	  RtlCopyMemory(pMPIdata->MyFrameResponseBuff, &pMPIdata->FrameSessionReceiveBuffer[11], pMPIdata->MyFrameResponseBuffLen);

	  sendShortAck(pMPIdata);

	  pMPIdata->FrameSessionSendBuffer[0]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[1]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[2]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[3]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
	  pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
	  pMPIdata->FrameSessionSendBuffer[6]  = 0x5C;
	  pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
	  pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
	  pMPIdata->FrameSessionSendBuffer[9]  = 0xB0;
	  pMPIdata->FrameSessionSendBuffer[10] = 0x01;
	  pMPIdata->FrameSessionSendBuffer[11] = pMPIdata->StationStatus[pMPIdata->Last_DA].Job;
	  pMPIdata->FrameSessionSendBuffer[13] = D_ED;
	  pMPIdata->FrameSessionSendLength = JOBACTION_ACK_LEN;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

	  if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 0xFF) pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 1;
	  else pMPIdata->StationStatus[pMPIdata->Last_DA].Job++;

	  pMPIdata->SessReqPending = true;
	  pMPIdata->JobRetry = 0;
	  return JOB_WAITSHORTACK2;
   }

   DBGReg(240, pMPIdata->FrameSessionReceiveBuffer[9], 0);

   return JOB_WAITJOBRESP;                 //MG001
}

unsigned char mJobRetryJobAck(struct s_MPIdata *pMPIdata)
{
   if (++pMPIdata->JobRetry > NR_JOB_RETRY)
   {
	  mJobApplicationResponseErr(pMPIdata);
	  return JOB_RUN;
   }
   else
   {
	  //retransmit previous Job number
	  if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 1)   pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 0xFF;
	  else                                   pMPIdata->StationStatus[pMPIdata->Last_DA].Job--;

	  pMPIdata->FrameSessionSendBuffer[0]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[1]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[2]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[3]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
	  pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
	  pMPIdata->FrameSessionSendBuffer[6]  = 0x5C;
	  pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
	  pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
	  pMPIdata->FrameSessionSendBuffer[9]  = 0xB0;
	  pMPIdata->FrameSessionSendBuffer[10] = 0x01;
	  pMPIdata->FrameSessionSendBuffer[11] = pMPIdata->StationStatus[pMPIdata->Last_DA].Job;
	  pMPIdata->FrameSessionSendBuffer[13] = D_ED;
	  pMPIdata->FrameSessionSendLength = JOBACTION_ACK_LEN;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

	  if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 0xFF) pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 1;
	  else pMPIdata->StationStatus[pMPIdata->Last_DA].Job++;

	  pMPIdata->SessReqPending = true;
	  return JOB_WAITSHORTACK2;
   }
}

void mJobStopSess(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->JobSessReqPending)
   {
	  pMPIdata->JobSessReqPending = false;
   }
}
#endif

/*----Token--------------------------------*/

//MG002 Start
void mTokFSMRestart(struct s_MPIdata *pMPIdata)
{
   #if MPI_MASTER_STACK
	  ev_post(pMPIdata, EV_LOG_RUN);
	  ev_post(pMPIdata, EV_JB0_RUN);
	  ev_post(pMPIdata, EV_JOB_RUN);
   #endif
   #if MPI_SLAVE_STACK
	  ev_post(pMPIdata, EV_LGS_RUN);
   #endif
   ev_post(pMPIdata, EV_LGF_RUN);
}
//MG002 End
void mTokListenTokenStartTimeout(struct s_MPIdata *pMPIdata)
{
   UltiUART1_StartTimer(pMPIdata, EV_TOK_TIMEOUT, (word)pMPIdata->FrameUniopNodeNum * pMPIdata->guardTimeFactor + pMPIdata->guardTimeConstant); //guard timeout
}

unsigned char mTokListenToken(struct s_MPIdata *pMPIdata)
{
   // Since SA is passing the token it must be an active master
   pMPIdata->StationStatus[pMPIdata->Source].StationType = 3; // Master in token ring

   // When we have listened to 2 complete token rotations
   // then we are ready to join the token ring
   if (2 <= pMPIdata->NumTokenRotations)
   {
	  mTokFSMRestart(pMPIdata); //MG002
	  return TOK_ACTIVEIDLE;
   }

   if (pMPIdata->LowestAddress > pMPIdata->Source)
   {
	  pMPIdata->LowestAddress = pMPIdata->Source;
	  pMPIdata->NumTokenRotations = 0;
   }

   if (pMPIdata->LowestAddress == pMPIdata->Dest)
	  pMPIdata->NumTokenRotations++;

   return TOK_LISTENTOKEN;
}

void mTokActiveIdle(struct s_MPIdata *pMPIdata)
{
   // Since SA is passing the token it must be an active master
   pMPIdata->StationStatus[pMPIdata->Source].StationType = 2; // Master Master ready to enter token ring (Ready FC=0x20)
   if (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest)
   {
	   dumpDebug("mTokActiveIdle");
	  // We have been addressed with an FDL status request
	  // If we are already in the token ring then tell the
	  // requester so, otherwise tell them that we are ready
	  // to join the token ring!
	  pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
	  if (pMPIdata->NextStation == (signed char)-1)
		 pMPIdata->NextStation = pMPIdata->Source;

	  pMPIdata->FrameSendBuffer[0] = D_SD1;
	  pMPIdata->FrameSendBuffer[1] = pMPIdata->Source;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBuffer[3] = (pMPIdata->AreWeInRing)? FCST_MSIR | RFC_OK : FCST_MSRD | RFC_OK;
	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSendBuffer, 1, 4);
	  pMPIdata->FrameSendBuffer[5] = D_ED;
	  pMPIdata->FrameSendBufferLen = 6;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

	  pMPIdata->FlagSendSelfToken = 0;
	  mTokFSMRestart(pMPIdata);  //MG002
	  return;
   }
}

unsigned char GotoActiveIdle(struct s_MPIdata *pMPIdata)
{
   pMPIdata->SessReqPending     = false;
   pMPIdata->SessionStarted     = false;
   pMPIdata->LgsSessReqPending  = false;
   pMPIdata->Job0SessReqPending = false;
   pMPIdata->JobSessReqPending  = false;

#ifdef FATAL_ERRORS
   pMPIdata->ProcedureApplMStatus = GEN_COMM_ERR;
   DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0005);
   pMPIdata->ProcedureApplMRequestPending  = false;
   pMPIdata->ProcedureApplMResponsePending = false;
#else
   if (pMPIdata->ProcedureApplMResponsePending)
	  pMPIdata->ProcedureApplMRequestPending = true;
#endif

   ResetStations(pMPIdata);

//MG001 start
   //reset all other state machines
   mLgfAbortSession(pMPIdata);
   pMPIdata->lgf_state = LGF_RUN;
   mLogAbortSession(pMPIdata);
   pMPIdata->log_state = LOG_RUN;
   mJob0ApplicationResponseErr(pMPIdata);
   pMPIdata->jb0_state = JB0_RUN;
   mJobApplicationResponseErr(pMPIdata);
   pMPIdata->job_state = JOB_RUN;
#if MPI_SLAVE_STACK
   mLgsAbortSession(pMPIdata);
   pMPIdata->lgs_state = LGS_RUN;
   mJ0sAbortSession(pMPIdata);
   pMPIdata->j0s_state = J0S_IDLE;
   mJbsAbortSession(pMPIdata);
   pMPIdata->jbs_state = JBS_IDLE;
#endif
   DBGReg(243, pMPIdata->tok_state, TOK_IDLE);
   UltiUART1_StartTimer(pMPIdata, EV_TOK_RUN,100*1000);
   return TOK_IDLE;
}
void StopTimeoutFDL(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->UltiUart1_TxTimeout == EV_TOK_FDLTIMEOUT)
   {
	  UltiUart1_StopTimer(pMPIdata);
   }
}

void mTokSD1Resp(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest)
   {
	  StopTimeoutFDL(pMPIdata);       //MG001
	  // Since SA is passing the token it must be an active master
	  pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->StationStatus[pMPIdata->Source].StationType = 2; // Master in token ring
	  pMPIdata->FlagSendSelfToken = 0;
	  pMPIdata->PassTokenReply++;
	  pMPIdata->NextStation = pMPIdata->Source;

	  // Pass the token onto the next active master
	  pMPIdata->FrameSendBuffer[0] = D_TOK;
	  pMPIdata->FrameSendBuffer[1] = pMPIdata->Source;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBufferLen = 3;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

	  return;
   }
}

byte FDLStatus(struct s_MPIdata *pMPIdata, signed char *MyNextGap)
{
   if ((*MyNextGap = PrepareNextGAP(pMPIdata)) != (signed char)-1)
   {
	  dumpDebug("FDLStatus");
	  pMPIdata->FrameSendBuffer[0] = D_SD1;
	  pMPIdata->FrameSendBuffer[1] = *MyNextGap;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBuffer[3] = FCFC_RFDL | FCMSK_FT;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSendBuffer, 1, 4);
	  pMPIdata->FrameSendBuffer[5] = D_ED;
	  pMPIdata->FrameSendBufferLen = 6;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

	  pMPIdata->PassTokenReply = 0;
	  return 1;
   }
   return 0;
}

byte PassToken(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->PassTokenReply < PASS_TOKEN_REPLY)
   {
	  // If the NextStation hasn't been calculated yet do it now
	  if (pMPIdata->NextStation == (signed char)-1)
		 pMPIdata->NextStation = pMPIdata->FrameUniopNodeNum;
	  else
		 pMPIdata->NextStation = GetNextActiveStation(pMPIdata);

	  // Pass the token onto the next active master
	  pMPIdata->FrameSendBuffer[0] = D_TOK;
	  pMPIdata->FrameSendBuffer[1] = pMPIdata->NextStation;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBufferLen = 3;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

	  pMPIdata->PassTokenReply++;
	  return 1;
   }

   //Reset station Status
   pMPIdata->StationStatus[(int)pMPIdata->NextStation].IsActive = 0;
   pMPIdata->StationStatus[(int)pMPIdata->NextStation].StationType = 0;
   pMPIdata->PassTokenReply = 0;

   // Try with the next station if exists
   pMPIdata->NextStation = GetNextActiveStation(pMPIdata);
   if ((pMPIdata->NextStation == (signed char)-1) || (pMPIdata->NextStation == pMPIdata->FrameUniopNodeNum)) return 0;

   // Pass the token onto the next active master
   pMPIdata->FrameSendBuffer[0] = D_TOK;
   pMPIdata->FrameSendBuffer[1] = pMPIdata->NextStation;
   pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
   pMPIdata->FrameSendBufferLen = 3;
   sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

   pMPIdata->PassTokenReply++;
   return 1;
}

unsigned char mTokTxFrame(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameUniopNodeNum != pMPIdata->Dest)
   {
	  if (pMPIdata->Dest == pMPIdata->Source)  //it is a self-token, the ring is broken
	  {
		  printk("MPI: TOK rec a SelfToken\n");
		 mExitFromRing(pMPIdata);
		 return TOK_IDLE;
	  }
	  //Too token not for me
	  if (pMPIdata->TokenNotForMe++ > 60)
	  {
		  printk("MPI: TOK rec too many not for me\n");
		 pMPIdata->TokenNotForMe = 0;
		 return GotoActiveIdle(pMPIdata);
	  }
	  mTokListenTokenStartTimeout(pMPIdata);
	  return TOK_WAITRX;
   }

   //disable any active timeout
   UltiUart1_StopTimer(pMPIdata);

   pMPIdata->PassTokenReply = 0;
   pMPIdata->AreWeInRing = true;
   pMPIdata->TokenNotForMe = 0;
   if (pMPIdata->GapUpdateFactorCnt <= pMPIdata->GapUpdateFactor)   //MG001
	  pMPIdata->GapUpdateFactorCnt++;                     //MG001

   if (pMPIdata->SessReqPending && !pMPIdata->SessionStarted)//Session Request Pending
   {
	  if (pMPIdata->LgfSessReqPending || pMPIdata->LgsSessReqPending || pMPIdata->J0sSessReqPending || pMPIdata->JbsSessReqPending)
	  {
		 pMPIdata->FrameSessionSendLength = pMPIdata->FrameSendBufferLen1;
		 sendData(pMPIdata, pMPIdata->FrameSendBuffer1, (byte)pMPIdata->FrameSessionSendLength);
	  }
	  else
		 sendData(pMPIdata, pMPIdata->FrameSessionSendBuffer, (byte)pMPIdata->FrameSessionSendLength);

	  pMPIdata->SessionStarted = true;
	  return TOK_WAITRX;
   }

   if (pMPIdata->GapUpdateFactorCnt > pMPIdata->GapUpdateFactor)  //MG001
   {
	  if (FDLStatus(pMPIdata, &pMPIdata->NxtStat))
	  {
		 if (pMPIdata->NxtStat == pMPIdata->NextStation - 1)
			pMPIdata->GapUpdateFactorCnt = 0;
		 return TOK_WAITRX;// Send an FDL Status request to the next GAP
	  }
   }
   if (PassToken(pMPIdata))
	   return TOK_WAITRX;// Pass Token
   return GotoActiveIdle(pMPIdata);
}
//MG001 end


void mTokStartTimeoutTok(struct s_MPIdata *pMPIdata)
{
   UltiUART1_StartTimer(pMPIdata, EV_TOK_TIMEOUT_TOK, pMPIdata->tokTimeout); //guard timeout
 //  dumpInteger("TOKEN TIMEOUT", tokTimeout);
}

void StopTimeoutTok(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->UltiUart1_TxTimeout == EV_TOK_TIMEOUT_TOK)
   {
	  pMPIdata->PassTokenReply = 0;
	  UltiUart1_StopTimer(pMPIdata);
   }
}
void mTokStartFDLTimeout(struct s_MPIdata *pMPIdata)
{
   #if EVENTLOG
	  UltiUART1_StartTimer(pMPIdata, EV_TOK_FDLTIMEOUT, 25*11);
   #else
	  UltiUART1_StartTimer(pMPIdata, EV_TOK_FDLTIMEOUT, pMPIdata->FDLTimeout);
   #endif
   pMPIdata->PassTokenReply = 0;
}
//MG001 start
void mTokStopTimeoutTok(struct s_MPIdata *pMPIdata)
{
   StopTimeoutTok(pMPIdata);
}
//MG001 end

void StopTimeoutAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->UltiUart1_TxTimeout == EV_BRC_ACK_TIMEOUT)
   {
	  UltiUart1_StopTimer(pMPIdata);
   }
}

void mTokStartTimeout10(struct s_MPIdata *pMPIdata)
{
   UltiUART1_StartTimer(pMPIdata, EV_TOK_TIMEOUT, (word)pMPIdata->FrameUniopNodeNum * pMPIdata->guardTimeFactor + pMPIdata->guardTimeConstant); //guard timeout
   pMPIdata->PassTokenReply = 0;
}

void mTokStartSelfTokTimeout(struct s_MPIdata *pMPIdata)
{
   UltiUART1_StartTimer(pMPIdata, EV_TOK_SELFTOK_TIMEOUT, pMPIdata->selfTokTimeout);  //timeout di funzionamento  //MG001
   pMPIdata->PassTokenReply = 0;
}


unsigned char mTokCheckReply(struct s_MPIdata *pMPIdata)
{
   //Sessione Slave - Su EOTX di ACK
   if (pMPIdata->ReadyForConfigMode == 2)
   {
	  pMPIdata->ReadyForConfigMode = 3;
	  return TOK_IDLE;
   }

   if (pMPIdata->LgfSessReqPending || pMPIdata->LgsSessReqPending || pMPIdata->J0sSessReqPending || pMPIdata->JbsSessReqPending) {
	  if (pMPIdata->SessReqPending && (pMPIdata->SessionStarted == 0)) {
		 pMPIdata->CurrentShortAck = SKIP;
		 if (PassToken(pMPIdata))
			return TOK_WAITRX;// Pass Token
		 return GotoActiveIdle(pMPIdata);
	  }
   }

   //Session Master
   if (pMPIdata->SessionStarted)
   {
	  pMPIdata->SessReqPending = false;
	  pMPIdata->SessionStarted = false;
	  UltiUART1_StartTimer(pMPIdata, EV_BRC_ACK_TIMEOUT, pMPIdata->ackGuardTime); //timeout di guardia
	  return TOK_WAITSESSRX;
   }

   if (pMPIdata->Sd1RespGuard)
   {
	  pMPIdata->Sd1RespGuard = 0;
	  return TOK_WAITRX;
   }

   //it's a pass token
   if (pMPIdata->PassTokenReply > 0)
   {
	  mTokStartTimeoutTok(pMPIdata);
	  return TOK_WAITRX;
   }
   if (pMPIdata->shortACK_EOTEvent)
   {
	   pMPIdata->shortACK_EOTEvent = 0;
	   mTokStartTimeoutTok(pMPIdata);
	   return TOK_WAITRX;
   }
   mTokStartFDLTimeout(pMPIdata);
   return TOK_WAITFDLSTATUS;
}

void mTokReloadTimer(struct s_MPIdata *pMPIdata)
{
}

unsigned char mTokWaitSessRx(struct s_MPIdata *pMPIdata, int stat)
{
   StopTimeoutAck(pMPIdata);
   if (stat)
   {
#ifdef RETRY_IN_SAME_TOKEN
	  if (++pMPIdata->JobRetry < NR_JOB_RETRY)
      {
         if (pMPIdata->job_state == JOB_WAITSHORTACK)
         {
           RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, pMPIdata->FrameJobSessionBuff, pMPIdata->FrameJobSessionLen);   //MG001
           pMPIdata->FrameSessionSendLength =pMPIdata->FrameJobSessionLen;                               //MG001
           pMPIdata->JobSessReqPending = true;
           pMPIdata->SessReqPending    = true;
         }
         else
         {
           //retransmit previous Job number
           if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 1)   
              pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 0xFF;
           else
              pMPIdata->StationStatus[pMPIdata->Last_DA].Job--;
           pMPIdata->FrameSessionSendBuffer[0]  = D_SD2;
           pMPIdata->FrameSessionSendBuffer[1]  = 0x08;
           pMPIdata->FrameSessionSendBuffer[2]  = 0x08;
           pMPIdata->FrameSessionSendBuffer[3]  = D_SD2;
           pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
           pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
           pMPIdata->FrameSessionSendBuffer[6]  = 0x5C;
           pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
           pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
           pMPIdata->FrameSessionSendBuffer[9]  = 0xB0;
           pMPIdata->FrameSessionSendBuffer[10] = 0x01;
           pMPIdata->FrameSessionSendBuffer[11] = pMPIdata->StationStatus[pMPIdata->Last_DA].Job;
           pMPIdata->FrameSessionSendBuffer[13] = D_ED;
           pMPIdata->FrameSessionSendLength = JOBACTION_ACK_LEN;
           //Calcola FCS
           CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);
           if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 0xFF) pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 1;
           else pMPIdata->StationStatus[pMPIdata->Last_DA].Job++;
           pMPIdata->SessReqPending = true;            
         }
         //resend
         return mTokTxFrame(pMPIdata);
      }
	  else {
		  ResetLastDAStation(pMPIdata);
		  ev_post(pMPIdata, EV_SES_ERR);
	  }
#else
	  ev_post(pMPIdata, EV_SES_RETRY);
#endif
   }
   else
	  ev_post(pMPIdata, EV_SES_ACK);

   //Pass Token
   {
	  if (PassToken(pMPIdata))
	  {
		 pMPIdata->FlagPassToken = 1;
		 return TOK_WAITSESSRX;
	  }
   }
   pMPIdata->FlagPassToken = 0;
   return GotoActiveIdle(pMPIdata);
}

#ifdef MANAGE_RR_ANSWERS
unsigned char mTokAnswerRR(struct s_MPIdata *pMPIdata)
{
   StopTimeoutAck(pMPIdata);
   ev_post(pMPIdata, EV_SES_RETRY);

   //Pass Token
   {
	  if (PassToken(pMPIdata))
	  {
		 pMPIdata->FlagPassToken = 1;
		 return TOK_WAITSESSRX;
	  }
   }
   pMPIdata->FlagPassToken = 0;
   return GotoActiveIdle(pMPIdata);
}
#endif

unsigned char mTokWaitSessRxSD1(struct s_MPIdata *pMPIdata)
{
   StopTimeoutAck(pMPIdata);

   if (pMPIdata->FrameSessionReceiveBuffer[3] == (FCST_MSIR | RFC_TTNAK) ||  /* Master in for ring + NAK no resource */
	   pMPIdata->FrameSessionReceiveBuffer[3] == (FCST_MSIR | RFC_RS_NAK))   /* Master in for ring + NAK no service activated */
   {
	   mExitFromRing(pMPIdata);
	   return TOK_IDLE;
   }

   ev_post(pMPIdata, EV_SES_ERR);
   //Pass Token
   if (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest)
   {
	  if (PassToken(pMPIdata))
	  {
		 pMPIdata->FlagPassToken = 1;
		 return TOK_WAITSESSRX;
	  }
   }
   pMPIdata->FlagPassToken = 0;
   return GotoActiveIdle(pMPIdata);
}
//MG001 end

unsigned char mTokCheckGuard(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FlagPassToken)
   {
	 pMPIdata->FlagPassToken = 0;
	 mTokStartTimeoutTok(pMPIdata);
	 return TOK_WAITRX;
   }
   return TOK_WAITSESSRX;
}

unsigned char mTokPassToken1(struct s_MPIdata *pMPIdata)
{
   if (PassToken(pMPIdata)) return TOK_TOKEN_RETRY;
   return GotoActiveIdle(pMPIdata);
}

unsigned char mTokPassToken(struct s_MPIdata *pMPIdata)
{
   if (PassToken(pMPIdata)) return TOK_WAITFDLSTATUS;
   return GotoActiveIdle(pMPIdata);
}

unsigned char mTokCheckFDLStatusResp(struct s_MPIdata *pMPIdata)
{
   if ((pMPIdata->Source == pMPIdata->NextGAP) && (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest))
   {
	  // Copy the station type from the StationType bits in the FC
	  // to our internal variable
	  pMPIdata->StationStatus[pMPIdata->Source].StationType =
		 (pMPIdata->FrameSessionReceiveBuffer[3] & FCMSK_ST) >> 4;

	  pMPIdata->NextStation = pMPIdata->Source;
	  pMPIdata->StationStatus[pMPIdata->Source].StationType = 3;

	  // Pass the token onto the next active master
	  pMPIdata->FrameSendBuffer[0] = D_TOK;
	  pMPIdata->FrameSendBuffer[1] = pMPIdata->NextStation;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBufferLen = 3;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);
	  pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;

	  pMPIdata->FlagSendSelfToken = 0;
	  pMPIdata->PassTokenReply++;

	  return TOK_WAITRX;
   }
   return GotoActiveIdle(pMPIdata);
}

void mTokSendSelfToken(struct s_MPIdata *pMPIdata)
{
   // Pass the token to ourselves
   pMPIdata->FrameSendBuffer[0] = D_TOK;
   pMPIdata->FrameSendBuffer[1] = pMPIdata->FrameUniopNodeNum;
   pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
   pMPIdata->FrameSendBufferLen = 3;
   sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

   pMPIdata->FlagSendSelfToken = 1;
   pMPIdata->NextStation = - 1;
}

unsigned char mTokSendFDLStatus(struct s_MPIdata *pMPIdata)
{
	signed char nxtgap;

   if (pMPIdata->cnt++ == 255)//to recovery error
   {
	  ResetStations(pMPIdata);
	  pMPIdata->cnt = 0;
   }
   // Send an FDL Status request to the next GAP
   if (FDLStatus(pMPIdata, &nxtgap))
		 return TOK_SELFTOKEN;
   return GotoActiveIdle(pMPIdata);
}

unsigned char mTokCheckResp(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FlagSendSelfToken)
   {
	  pMPIdata->FlagSendSelfToken = 0;
	  mTokStartSelfTokTimeout(pMPIdata);
	  return TOK_SELFTOKEN;
   }
   //Return from self token
   mTokFSMRestart(pMPIdata);         //MG002
   mTokStartTimeoutTok(pMPIdata);
   return TOK_WAITRX;
}

void mTokSelfToken(struct s_MPIdata *pMPIdata)
{
   ResetStations(pMPIdata);
   mTokStartFDLTimeout(pMPIdata);  //MG001
}

void mTokSendFDLStatusRsp(struct s_MPIdata *pMPIdata)
{
	if (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest)
	{
		if (pMPIdata->cnt++ > 5)//to avoid dead lock
		{
			pMPIdata->cnt = 0;
			mTokStartTimeout10(pMPIdata);
			return;
		}

		dumpDebug("mTokSendFDLStatusRsp");
		pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
		pMPIdata->FrameSendBuffer[0] = D_SD1;
		pMPIdata->FrameSendBuffer[1] = pMPIdata->Source;
		pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
		pMPIdata->FrameSendBuffer[3] = (pMPIdata->AreWeInRing)? FCST_MSIR | RFC_OK : FCST_MSRD | RFC_OK;
		pMPIdata->StationStatus[pMPIdata->Source].StationType = 3;

		//Calcola FCS
		CalcCheckSum(pMPIdata->FrameSendBuffer, 1, 4);
		pMPIdata->FrameSendBuffer[5] = D_ED;
		pMPIdata->FrameSendBufferLen = 6;
		sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

		pMPIdata->FlagSendSelfToken = 0;
		pMPIdata->Sd1RespGuard = 1;
		return;
	}
}

unsigned char mTokGotoActiveIdle(struct s_MPIdata *pMPIdata)
{
   return GotoActiveIdle(pMPIdata);
}

#if MPI_SLAVE_STACK
/************************************************************************
*
* PrepareSwitchToConfigFrames
*
*  This function builds the frames for manage the "switch to config" process
*
* Parameters:
*  void
*
* Returns:
*  0 - error
*  1 - ok
***********************************************************************/
byte PrepareSwitchToConfigFrames()
{
   //Prepare Response Frame : WrMyBuff
   if (FrameSessionReceiveBuffer[14+7] == 5)
   {//WR
	  RtlCopyMemory(WrMyBuff, &FrameSessionReceiveBuffer[11], 7);
	  //Prepare WR Ack
	  WrMyBuff[1]  = 0x03;
	  WrMyBuff[7] = 0x02;
	  WrMyBuff[8] = 0x00;
	  WrMyBuff[9] = 0x01;
	  WrMyBuff[10] = 0x00;
	  WrMyBuff[11] = 0x00;
	  WrMyBuff[12] = 0x05;
	  WrMyBuff[13] = 0x01;
	  WrMyBuff[14] = 0xFF;
	  FrameMyBuffLength = 19;
	  MyOperFlag = WR;
	  //Check if the cmd goto_config is ok and prepare answer
	  if (FrameSessionReceiveBuffer[32+4+7] == 0x01 ||
		  FrameSessionReceiveBuffer[32+4+7] == 0x02)
	  {
		 FrameMyBuff[0] = 0x81;
		 FrameMyBuff[1] = 0x01;
		 FrameMyBuff[2] = FrameSessionReceiveBuffer[32+2+7];
		 FrameMyBuff[3] = FrameSessionReceiveBuffer[32+3+7];
		 FrameMyBuff[4] = 0x80 | FrameSessionReceiveBuffer[32+4+7];
		 FrameMyBuff[5] = 0;
		 FrameMyBuff[6] = 0;
		 FrameMyBuff[7] = 1;

//MG001 start
		 if (FrameSessionReceiveBuffer[32+4+7] == 1)
			FrameMyBuff[8] = 1;
		 else
			FrameMyBuff[8] = FrameSessionReceiveBuffer[32+4+7+4];
//MG001 end
	  }
	  else
		 return 0; //error!!
   }
   if (FrameSessionReceiveBuffer[14+7] == 4)
   {//RD
	  MyOperFlag = 1;
	  WrMyBuffLength = MAKEWORD(FrameSessionReceiveBuffer[21+7], FrameSessionReceiveBuffer[20+7]);
	  FrameMyBuffLength = 22 + WrMyBuffLength;
	  //Prepare PDU-Frame Header
	  RtlCopyMemory(WrMyBuff, &FrameSessionReceiveBuffer[11], 7);
	  WrMyBuff[1] = 0x03;
	  WrMyBuff[7] = 0x02;
	  WrMyBuff[8] = 0x00;
	  WrMyBuff[9] = 0x0D;
	  WrMyBuff[10] = 0x00;
	  WrMyBuff[11] = 0x00;
	  WrMyBuff[12] = 0x04;
	  WrMyBuff[13] = 0x01;  //NrOfVars
	  WrMyBuff[14] = 0xFF;
	  WrMyBuff[15] = 0x04;
	  MyLen = MAKEWORD(FrameSessionReceiveBuffer[21+7], FrameSessionReceiveBuffer[20+7]);
	  MyLen = MyLen << 3;
	  WrMyBuff[16] = HIBYTE(MyLen);
	  WrMyBuff[17] = LOBYTE(MyLen);
//MG001      if (FrameMyBuff[4] == 0x82 && MyLen == 9*8)
	  if (FrameMyBuff[4] == 0x82 && FrameMyBuff[8] == 0 && MyLen == 9*8)   //MG001
		 ReadyForConfigMode = 1;
	  else
		 ReadyForConfigMode = 0;
   }
   return 1;
}

void InsertSwitchToConfigFrame(void)
{
   if (MyOperFlag == WR)
	  RtlCopyMemory(&FrameSendBuffer1[11], WrMyBuff, 19);
   else
   {
	  RtlCopyMemory(&WrMyBuff[18], FrameMyBuff, WrMyBuffLength);
	  RtlCopyMemory(&FrameSendBuffer1[11], WrMyBuff, 18 + WrMyBuffLength + 2);
   }
   FrameSendBufferLen1 = FrameSendBuffer1[1]+6;
}

/*----LogSlave---------------------------*/
unsigned char mLgsCheckSAPReqFrame()
{
   //Rx Frame Ok
   if (FrameSessionReceiveBuffer[9] == 0xE0)
   {
	  sendShortAck();

	  //Prepare next frame
	  FrameSendBufferLen1 = LOGON_SAPRSP_LEN;
	  RtlCopyMemory(FrameSendBuffer1, LogOnSAPRsp, LOGON_SAPRSP_LEN);
	  LogOnSlave_DA = FrameSessionReceiveBuffer[5] & 0x7F;
	  StationStatus[LogOnSlave_DA].LogOn_DAE = FrameSessionReceiveBuffer[8];
	  StationStatus[LogOnSlave_DA].LogOn_SAE = Start_LogOn_DAE;
	  FrameSendBuffer1[4] = LogOnSlave_DA | 0x80;
	  FrameSendBuffer1[5] = FrameUniopNodeNum | 0x80;
	  FrameSendBuffer1[7] = StationStatus[LogOnSlave_DA].LogOn_DAE;
	  FrameSendBuffer1[8] = StationStatus[LogOnSlave_DA].LogOn_SAE;

	  //Calcola FCS
	  CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

	  ev_post(EV_LGS_SEND_SAPRSP);
	  timer_start(EV_BRC_SL_SESS_TIMEOUT, 10000);

	  LgsSessReqPending = true;
	  SlaveSession = true;
	  ReadyForConfigMode = 0;

	  ev_post(EV_BRC_STOP_SESS); //Abort eventually JobSession
	  return LGS_SEND_SAPRSP;
   }
   return LGS_RUN;
}

void mLgsSendRequestedSap()
{
   //Because it's possible a JOB abort
   SessReqPending = true;
   SessionStarted = false;
}

unsigned char mLgsCheckAckSAPRspFrame()
{
   //Rx Frame Ok
   Source = FrameSessionReceiveBuffer[5] & 0x7f;
   if (Source == LogOnSlave_DA)
   {
	  if (FrameSessionReceiveBuffer[9] == 0x05) {
		 sendShortAck();

		 //Prepare next frame
		 FrameSendBufferLen1 = LOGON_SAPRESPACK_LEN;
		 RtlCopyMemory(FrameSendBuffer1, LogOnSAPRespAck, LOGON_SAPRESPACK_LEN);
		 FrameSendBuffer1[4] = LogOnSlave_DA | 0x80;
		 FrameSendBuffer1[5] = FrameUniopNodeNum | 0x80;
		 FrameSendBuffer1[7] = StationStatus[LogOnSlave_DA].LogOn_DAE;
		 FrameSendBuffer1[8] = StationStatus[LogOnSlave_DA].LogOn_SAE;

		 //Calcola FCS
		 CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

		 ev_post(EV_LGS_SEND_ACKSAPRSPACK);
		 return LGS_SEND_ACKSAPRSPACK;
	  }
	  mLgsAbortSession();
	  return LGS_RUN;
   }
   return LGS_ACKSAPRSP;
}

void mLgsSendAckSAPRspAck()
{
   SessReqPending = true;
   SessionStarted = false;
}

void mLgsLogOnSlaveOk()
{
   timer_stop(EV_BRC_SL_SESS_TIMEOUT); //MG002
   Start_LogOn_DAE++;
   LgsSessReqPending = false;
   StationStatus[LogOnSlave_DA].Logged = 1;
   StationStatus[LogOnSlave_DA].Job   = 0;
   ev_post(EV_J0S_RUN);
}

void mLgsAbortSession()
{
   if (LgsSessReqPending)
   {
	  LgsSessReqPending = false;
	  SessReqPending    = false;
	  StationStatus[LogOnSlave_DA].Logged = 0;
	  StationStatus[LogOnSlave_DA].Job   = 0;
	  StationStatus[LogOnSlave_DA].LogOn_DAE = 0;
	  StationStatus[LogOnSlave_DA].LogStatus = 0;	//MG001
   }
   SlaveSession = false;
}

/*----Job0Slave---------------------------*/
void mJ0sRun()
{
   timer_start(EV_BRC_J0S_SESS_TIMEOUT, 1000);
}

unsigned char mJ0sCheckJobReqFrame()
{
   if (!StationStatus[LogOnSlave_DA].Logged)    //MG001
	  return J0S_RUN;                           //MG001

   Source = FrameSessionReceiveBuffer[5] & 0x7f;   //MG001
   if (Source == LogOnSlave_DA)                    //MG001
   {                                               //MG001
	  //Rx Frame Ok
	  if ((FrameSessionReceiveBuffer[9] == 0xF1) &&
		  (FrameSessionReceiveBuffer[10] == 0x00))
	  {
		 sendShortAck();
		 //Prepare next frame
		 FrameSendBufferLen1 = JOBACTION_ACK_LEN;
		 RtlCopyMemory(FrameSendBuffer1, JobActionAck, JOBACTION_ACK_LEN);
		 FrameSendBuffer1[4] = LogOnSlave_DA | 0x80;
		 FrameSendBuffer1[5] = FrameUniopNodeNum | 0x80;
		 FrameSendBuffer1[7] = StationStatus[LogOnSlave_DA].LogOn_DAE;
		 FrameSendBuffer1[8] = StationStatus[LogOnSlave_DA].LogOn_SAE;

		 //Calcola FCS
		 CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

		 ev_post(EV_J0S_SEND_JOBACK);
		 J0sSessReqPending = true;
		 timer_start(EV_BRC_J0S_SESS_TIMEOUT, 1000);	//MG001
		 return J0S_SEND_JOBACK;
	  }
   }                                               //MG001
   return J0S_RUN;
}

void mJ0sSendJobAck()
{
   SessReqPending = true;
}

void mJ0sBuildJobRsp()
{
   RtlCopyMemory(FrameSendBuffer1, JobAction2Req0, JOBACTION2_REQ_LEN0);

   FrameSendBuffer1[4]  = LogOnSlave_DA | 0x80;
   FrameSendBuffer1[5]  = FrameUniopNodeNum | 0x80;
   FrameSendBuffer1[7]  = StationStatus[LogOnSlave_DA].LogOn_DAE;
   FrameSendBuffer1[8]  = StationStatus[LogOnSlave_DA].LogOn_SAE;
   FrameSendBufferLen1 = FrameSendBuffer1[1]+6;

   //Calcola FCS
   CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

   ev_post(EV_J0S_SEND_JOBRSP);
   timer_start(EV_BRC_J0S_SESS_TIMEOUT, 1000);	//MG001
}

void mJ0sSendJobRsp()
{
   SessReqPending = true;
}

void mJ0sAbortSession()
{
   if (J0sSessReqPending)
   {
	  J0sSessReqPending = false;
	  SessReqPending    = false;
	  SessionStarted    = false;
	  StationStatus[LogOnSlave_DA].Logged = 0;
	  StationStatus[LogOnSlave_DA].Job   = 0;
	  StationStatus[LogOnSlave_DA].LogOn_DAE = 0;
	  StationStatus[LogOnSlave_DA].LogStatus = 0;  //MG001
   }
   SlaveSession = false;
}

//MG001 void mJ0sSlaveOk(void)
unsigned char mJ0sSlaveOk()
{
   Source = FrameSessionReceiveBuffer[5] & 0x7f;   //MG001
   if (Source == LogOnSlave_DA)                    //MG001
   {                                               //MG001
	  J0sSessReqPending = false;
	  //Rx Frame Ok
	  if ((FrameSessionReceiveBuffer[9] == 0xB0) &&
		  (FrameSessionReceiveBuffer[10] == 0x01))
	  {
		 sendShortAck();

		 StationStatus[LogOnSlave_DA].Logged = 1;
		 StationStatus[LogOnSlave_DA].Job   = 1;
		 ev_post(EV_JBS_RUN);
		 timer_stop(EV_BRC_J0S_SESS_TIMEOUT);
		 return J0S_IDLE;                          //MG001
	  }
	  ev_post(EV_SES_ERR);
	  return J0S_IDLE;                             //MG001
   }                                               //MG001
   return J0S_WAIT_ACK_JOBRSP;                     //MG001
}

/*----JobSlave---------------------------*/
void mJbsRun()
{
   timer_start(EV_BRC_JBS_SESS_TIMEOUT, 1000);
}

unsigned char mJbsCheckJobReqFrame()
{
   if (!StationStatus[LogOnSlave_DA].Logged) return JBS_RUN;

   //Rx Frame Ok
   Source = FrameSessionReceiveBuffer[5] & 0x7f;
   if (Source == LogOnSlave_DA)
   {
	  if (FrameSessionReceiveBuffer[9] == 0xF1)
	  {
		 sendShortAck();

		 if (!PrepareSwitchToConfigFrames())
			return JBS_RUN;

		 //Prepare JobAck frame
		 FrameSendBufferLen1 = JOBACTION_ACK_LEN;
		 RtlCopyMemory(FrameSendBuffer1, JobActionAck, JOBACTION_ACK_LEN);

		 FrameSendBuffer1[4] = LogOnSlave_DA | 0x80;
		 FrameSendBuffer1[5] = FrameUniopNodeNum | 0x80;
		 FrameSendBuffer1[7] = StationStatus[LogOnSlave_DA].LogOn_DAE;
		 FrameSendBuffer1[8] = StationStatus[LogOnSlave_DA].LogOn_SAE;
		 JbsOnSlave_Job = FrameSessionReceiveBuffer[10];
		 FrameSendBuffer1[11] = JbsOnSlave_Job;

		 //Calcola FCS
		 CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

		 ev_post(EV_JBS_SEND_JOBACK);
		 JbsSessReqPending = true;
		 timer_start(EV_BRC_JBS_SESS_TIMEOUT, 1000);	//MG001
		 return JBS_SEND_JOBACK;
	  }
   }
   return JBS_RUN;
}

void mJbsSendJobAck()
{
   SessReqPending = true;
}

void mJbsBuildJobRsp()
{
   FrameSendBuffer1[0]  = D_SD2;
   FrameSendBuffer1[1]  = FrameMyBuffLength + 7;
   FrameSendBuffer1[2]  = FrameMyBuffLength + 7;
   FrameSendBuffer1[3]  = D_SD2;
   FrameSendBuffer1[4]  = LogOnSlave_DA | 0x80;
   FrameSendBuffer1[5]  = FrameUniopNodeNum | 0x80;
   FrameSendBuffer1[9]  = 0x5C;
   FrameSendBuffer1[7]  = StationStatus[LogOnSlave_DA].LogOn_DAE;
   FrameSendBuffer1[8]  = StationStatus[LogOnSlave_DA].LogOn_SAE;
   FrameSendBuffer1[9]  = 0xF1;
   FrameSendBuffer1[10] = JbsOnSlave_Job;

   InsertSwitchToConfigFrame();
   FrameSendBufferLen1 = FrameSendBuffer1[1]+6;

   //Calcola FCS
   CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);
   FrameSendBuffer1[FrameSendBufferLen1-1] = D_ED;

   ev_post(EV_JBS_SEND_JOBRSP);
   timer_start(EV_BRC_JBS_SESS_TIMEOUT, 1000);	//MG001
}

void mJbsSendJobRsp()
{
   SessReqPending = true;
}

void mJbsAbortSession()
{
   if (JbsSessReqPending)
   {
	  JbsSessReqPending = false;
	  SessReqPending    = false;
	  SessionStarted    = false;
	  StationStatus[LogOnSlave_DA].Logged = 0;
	  StationStatus[LogOnSlave_DA].Job   = 0;
	  StationStatus[LogOnSlave_DA].LogOn_DAE = 0;
	  StationStatus[LogOnSlave_DA].LogStatus = 0;	//MG001
   }
   SlaveSession = false;
}

unsigned char mJbsSlaveOk()
{
   Source = FrameSessionReceiveBuffer[5] & 0x7f;   //MG001
   if (Source == LogOnSlave_DA)                    //MG001
   {                                               //MG001
	  JbsSessReqPending = false;
	  //Rx Frame Ok
	  if ((FrameSessionReceiveBuffer[9] == 0xB0) &&
		  (FrameSessionReceiveBuffer[10] == 0x01))
	  {

		 sendShortAck();

		 if (ReadyForConfigMode == 1)
		 {
			ReadyForConfigMode = 2;
			timer_stop(EV_BRC_JBS_SESS_TIMEOUT);
			SlaveSession = false;
			//GOTO_CONFIG_MODE();         //switch to config mode!!!
			return JBS_IDLE;
		 }
		 return JBS_RUN;
	  }
	  ev_post(EV_SES_ERR);
	  SlaveSession = false;
   }                                               //MG001
   return JBS_WAIT_ACK_JOBRSP;
}

#endif // MPI_SLAVE_STACK

// Get Event
unsigned char ev_get(struct s_MPIdata *pMPIdata)
{
  unsigned char ev;

	ev = pMPIdata->ev_queue[pMPIdata->ev_queue_wr];
	pMPIdata->ev_queue_wr = (pMPIdata->ev_queue_wr + 1) % N_EVPOST;
	if (pMPIdata->ev_queue_wr == pMPIdata->ev_queue_rd) pMPIdata->queue_empty = 1;
	dumpEvent(ev);
	return ev;
}

// Move The Machines

void Token(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
unsigned char last_state = pMPIdata->tok_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->tok_state)
	{
		case TOK_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE:
					next_state = TOK_IDLE; break;
				case EV_TOK_RUN:
					mTokListenTokenStartTimeout(pMPIdata);next_state = TOK_LISTENTOKEN; break;
			} break;
		case TOK_LISTENTOKEN:
			switch (pMPIdata->event) {
				case EV_BRC_TOK:
					printk("MPI: TOK rec in TOK_LISTENTOKEN\n");
					mTokListenTokenStartTimeout(pMPIdata);next_state = mTokListenToken(pMPIdata); break;//MG001
				case EV_BRC_SD1:
					mTokListenTokenStartTimeout(pMPIdata);next_state = mTokListenToken(pMPIdata); break;//MG001
				case EV_TOK_TIMEOUT:
					mTokSendSelfToken(pMPIdata);next_state = TOK_LISTENTOKEN; break;
				case EV_BRC_EOTX:
					mTokStartSelfTokTimeout(pMPIdata);next_state = TOK_SELFTOKEN; break;
			} break;
		case TOK_SELFTOKEN:
			switch (pMPIdata->event) {
				case EV_TOK_SELFTOK_TIMEOUT : next_state = mTokSendFDLStatus(pMPIdata); break;
				case EV_BRC_EOTX : mTokStartFDLTimeout(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break;
				case EV_BRC_TOK :
					printk("MPI: TOK rec in TOK_SELFTOKEN\n");
					mTokFSMRestart(pMPIdata);next_state = TOK_ACTIVEIDLE; break;//MG002
			} break;
		case TOK_WAITFDLSTATUS2:
			switch (pMPIdata->event) {
				case EV_TOK_FDLTIMEOUT : mTokSendSelfToken(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break;
				case EV_BRC_EOTX : next_state = mTokCheckResp(pMPIdata); break;
				case EV_BRC_SD1 : mTokSD1Resp(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break;
				case EV_BRC_TOK :
					printk("MPI: TOK rec in TOK_WAITFDLSTATUS2\n");
					mTokFSMRestart(pMPIdata);next_state = TOK_ACTIVEIDLE; break;//MG002
			} break;
		case TOK_ACTIVEIDLE:
			switch (pMPIdata->event) {
				case EV_BRC_SD1:
					mTokActiveIdle(pMPIdata);next_state = TOK_ACTIVEIDLE; break;
				case EV_BRC_EOTX:
					next_state = TOK_WAITRX; break; //MG001
				case EV_BRC_TOK:
					printk("MPI: TOK rec in TOK_ACTIVEIDLE\n");
					mTokStartTimeout10(pMPIdata);next_state = TOK_ACTIVEIDLE; break;   //MG001
				case EV_SES_SD2:
					mTokStartTimeout10(pMPIdata);next_state = TOK_ACTIVEIDLE; break;   //MG001
				case EV_TOK_TIMEOUT:
					mTokSelfToken(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break; //MG002
			} break;
		case TOK_TOKEN_RETRY:
			switch (pMPIdata->event) {
				case EV_BRC_EOTX : mTokStartTimeoutTok(pMPIdata);next_state = TOK_WAITRX; break;
				case EV_BRC_TOK :
					printk("MPI: TOK rec in TOK_TOKEN_RETRY\n");
					next_state = TOK_ACTIVEIDLE; break;
			} break;
		case TOK_WAITRX:
			switch (pMPIdata->event) {
				case EV_BRC_SD1 : mTokListenTokenStartTimeout(pMPIdata);mTokSendFDLStatusRsp(pMPIdata);next_state = TOK_WAITRX; break;   //MG001
				case EV_SES_SD2 : mTokListenTokenStartTimeout(pMPIdata);next_state = TOK_WAITRX; break;        //MG001
				case EV_BRC_TOK : next_state = mTokTxFrame(pMPIdata); break;
				case EV_BRC_EOTX : next_state = mTokCheckReply(pMPIdata); break;
				case EV_TOK_TIMEOUT_TOK : next_state = mTokPassToken1(pMPIdata); break;
				case EV_TOK_TIMEOUT : mTokSelfToken(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break; //MG002
			} break;
		case TOK_WAITFDLSTATUS:
			switch (pMPIdata->event) {
				case EV_TOK_FDLTIMEOUT : next_state = mTokPassToken(pMPIdata); break;
				case EV_BRC_EOTX : mTokStartTimeoutTok(pMPIdata);next_state = TOK_WAITRX; break;
				case EV_BRC_SD1 : next_state = mTokCheckFDLStatusResp(pMPIdata); break;
				case EV_BRC_TOK :
					printk("MPI: TOK rec in TOK_WAIT_FDLSTATUS\n");
					next_state = mTokListenToken(pMPIdata); break;
			} break;
		case TOK_WAITSESSRX:
			switch (pMPIdata->event) {
				case EV_BRC_ACK : next_state = mTokWaitSessRx(pMPIdata, 0); break;
				case EV_BRC_SD1 : next_state = mTokWaitSessRxSD1(pMPIdata); break;
				case EV_BRC_EOTX : next_state = mTokCheckGuard(pMPIdata); break;
				case EV_BRC_ACK_TIMEOUT: next_state = mTokWaitSessRx(pMPIdata, 1);  break;
#ifdef MANAGE_RR_ANSWERS
				case EV_BRC_SDX: next_state = mTokAnswerRR(pMPIdata);  break;
#endif
			} break;
	}
	if (next_state != _NULL)
	{
		#if EVENTLOG
		  #if SUBSET_EVENTLOG
			if (msgEventsLog[event]) {
		  #endif
		   #if SHORT_FORM
				dbg_byte = tok_state+LOG_STATE_OFFSET;
				UltiUART2_FifoWrite(&dbg_byte, 1);
				dbg_byte = next_state+LOG_STATE_OFFSET;
				  UltiUART2_FifoWrite(&dbg_byte, 1);
			#else
					sprintf(s, "(%s)->(%s)", msgStates[tok_state], msgStates[next_state]);
				   DbgStrPrint(s);
			#endif
			#if SUBSET_EVENTLOG
			 }
		#endif
		#endif
		pMPIdata->tok_state = next_state;
		DBGReg(pMPIdata->event, last_state, pMPIdata->tok_state);
	}
	dumpTokState(pMPIdata->tok_state);
}

void LogOff(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
unsigned char last_state = pMPIdata->lgf_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->lgf_state) {
		case LGF_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE : next_state = LGF_IDLE; break;
				case EV_LGF_RUN : next_state = LGF_RUN; break;
			} break;
		case LGF_RUN:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLgfCheckLogOff(pMPIdata); break;
			} break;
		case LGF_SENT_LOGOFF_ACK:
			switch (pMPIdata->event) {
				case EV_LGF_SEND_LOGOFFACK : mLgfSendLogOffAck(pMPIdata);next_state = LGF_SENT_LOGOFF_ACK; break;
//MG002				case EV_BRC_SD1 : next_state = mLgfCheckSD1(); break;
				case EV_SES_ACK : mLgfAbortSession(pMPIdata);next_state = LGF_RUN; break;
				case EV_SES_ERR : mLgfAbortSession(pMPIdata);next_state = LGF_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mLgfAbortSession(pMPIdata);next_state = LGF_RUN; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			DebugPrint(lgf_state+LOG_STATE_OFFSET);DebugPrint(next_state+LOG_STATE_OFFSET);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[lgf_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	pMPIdata->lgf_state = next_state;
	DBGReg(pMPIdata->event, last_state, pMPIdata->lgf_state);
	}
}


#if MPI_MASTER_STACK
void Log(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
unsigned char last_state = pMPIdata->log_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->log_state) {
		case LOG_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE : next_state = LOG_IDLE; break;
				case EV_LOG_RUN : next_state = LOG_RUN; break;
			} break;
		case LOG_RUN:
			switch (pMPIdata->event) {
				case EV_LOG_START : mLogSendSAPReqFrame(pMPIdata);next_state = LOG_WAITSAPREQACK; break;
				//case EV_SES_SD2 : next_state = LOG_RUN; break;
			} break;
		case LOG_WAITSAPREQACK:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLogCheckLogOff(pMPIdata); break;
				case EV_SES_ACK : next_state = LOG_WAITSAPRSP; break;
				//MG003 case EV_SES_ERR : mLogAbortSession();next_state = LOG_RUN; break;
				//MG003 case EV_BRC_APPL_TIMEOUT : mLogAbortSession();next_state = LOG_RUN; break;
				case EV_SES_ERR : next_state = mLogAbortSession(pMPIdata); break;//MG003
				case EV_BRC_APPL_TIMEOUT : next_state = mLogAbortSession(pMPIdata); break;//MG003
			} break;
		case LOG_WAITSAPRSP:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLogSendRespAck(pMPIdata); break;
				//MG003 case EV_BRC_APPL_TIMEOUT : mLogAbortSession();next_state = LOG_RUN; break;
				case EV_BRC_APPL_TIMEOUT : next_state = mLogAbortSession(pMPIdata); break;//MG003
			} break;
		case LOG_WAITSAPRSPACK:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLogCheckLogOff(pMPIdata); break;
				case EV_SES_ACK : next_state = LOG_WAITACKSAPRSPACK; break;
				//MG003 case EV_SES_ERR : mLogAbortSession();next_state = LOG_RUN  ; break;
				//MG003 case EV_BRC_APPL_TIMEOUT : mLogAbortSession();next_state = LOG_RUN ; break;
				case EV_SES_ERR : next_state = mLogAbortSession(pMPIdata); break;//MG003
				case EV_BRC_APPL_TIMEOUT : next_state = mLogAbortSession(pMPIdata); break;//MG003
			} break;
		case LOG_WAITACKSAPRSPACK:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLogWaitAckRespAck(pMPIdata); break;
				//MG003 case EV_SES_ERR : mLogAbortSession();next_state = LOG_RUN         ; break;
				//MG003 case EV_BRC_APPL_TIMEOUT : mLogAbortSession();next_state = LOG_RUN  ; break;
				case EV_SES_ERR : next_state = mLogAbortSession(pMPIdata); break;//MG003
				case EV_BRC_APPL_TIMEOUT : next_state = mLogAbortSession(pMPIdata);break;//MG003
			} break;
	}
	if (next_state != _NULL) {
   #if EVENTLOG
		#if SUBSET_EVENTLOG
		if (msgEventsLog[event]) {
		#endif
	   #if SHORT_FORM
			dbg_byte = log_state+LOG_STATE_OFFSET;
			UltiUART2_FifoWrite(&dbg_byte, 1);
			dbg_byte = next_state+LOG_STATE_OFFSET;
			  UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[log_state], msgStates[next_state]);
			   DbgStrPrint(s);
		#endif
		#if SUBSET_EVENTLOG
		 }
		#endif
	#endif
		pMPIdata->log_state = next_state;
		dumpLogState(pMPIdata->log_state);
		DBGReg(pMPIdata->event, last_state, pMPIdata->log_state);
   }
}

void Job0(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
unsigned char last_state = pMPIdata->jb0_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->jb0_state) {
		case JB0_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE : next_state = JB0_IDLE; break;
				case EV_JB0_RUN : next_state = JB0_RUN; break;
			} break;
		case JB0_RUN:
			switch (pMPIdata->event) {
				case EV_JB0_SEND_REQ : mJob0SendReq(pMPIdata);next_state = JB0_WAITSHORTACK0; break;
			} break;
		case JB0_WAITSHORTACK0:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_ACK : next_state = JB0_WAITJOBACK0; break;
				case EV_SES_ERR : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
			} break;
		case JB0_WAITJOBACK0:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_SD2 : next_state = mJob0SendShortAck(pMPIdata); break;
				case EV_SES_ERR : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
			} break;
		case JB0_WAITJOBRESP0:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_SD2 : next_state = mJob0SendJobAck(pMPIdata); break;
				case EV_SES_ERR : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
			} break;
		case JB0_WAITSHORTACK02:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_ACK : mJob0Ok(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_ERR : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			dbg_byte = jb0_state+LOG_STATE_OFFSET;
			UltiUART2_FifoWrite(&dbg_byte, 1);
			dbg_byte = next_state+LOG_STATE_OFFSET;
			  UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[jb0_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	pMPIdata->jb0_state = next_state;
	dumpJb0State(pMPIdata->jb0_state);
	DBGReg(pMPIdata->event, last_state, pMPIdata->jb0_state);
	}
}

void Job(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
unsigned char last_state = pMPIdata->job_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->job_state) {
		case JOB_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE : next_state = JOB_IDLE; break;
				case EV_JOB_RUN : next_state = JOB_RUN; break;
			} break;
		case JOB_RUN:
			switch (pMPIdata->event) {
				case EV_JOB_SEND_REQ : mJobSendReq(pMPIdata);next_state = JOB_WAITSHORTACK; break;
			} break;
		case JOB_WAITSHORTACK:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_ACK : next_state = JOB_WAITJOBACK; break;
				case EV_SES_ERR : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_STOP_SESS : mJobStopSess(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_RETRY : next_state = mJobRetryReq(pMPIdata); break;
			} break;
		case JOB_WAITJOBACK:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_SD2 : next_state = mJobSendShortAck(pMPIdata); break;
				case EV_SES_ERR : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_STOP_SESS : mJobStopSess(pMPIdata);next_state = JOB_RUN; break;
			} break;
		case JOB_WAITJOBRESP:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_SD2 : next_state = mJobSendJobAck(pMPIdata); break;
				case EV_SES_ERR : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_STOP_SESS : mJobStopSess(pMPIdata);next_state = JOB_RUN; break;
			} break;
		case JOB_WAITSHORTACK2:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_ACK : mJobApplicationResponse(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_ERR : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_STOP_SESS : mJobStopSess(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_RETRY : next_state = mJobRetryJobAck(pMPIdata); break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			dbg_byte = job_state+LOG_STATE_OFFSET;
			UltiUART2_FifoWrite(&dbg_byte, 1);
			dbg_byte = next_state+LOG_STATE_OFFSET;
			  UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[job_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
		pMPIdata->job_state = next_state;
		dumpJobState(pMPIdata->job_state);
		DBGReg(pMPIdata->event, last_state, pMPIdata->job_state);
	}
}
#endif

#if MPI_SLAVE_STACK
void LogSlave()
{
unsigned char next_state = _NULL;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (lgs_state) {
		case LGS_IDLE:
			switch (event) {
				case EV_BRC_ENABLE : next_state = LGS_IDLE; break;
				case EV_LGS_RUN : next_state = LGS_RUN; break;
			} break;
		case LGS_RUN:
			switch (event) {
				case EV_SES_SD2 : next_state = mLgsCheckSAPReqFrame(); break;
				case EV_BRC_SL_SESS_TIMEOUT : mLgsAbortSession();next_state = LGS_RUN; break;
			} break;
		case LGS_SEND_SAPRSP:
			switch (event) {
				case EV_SES_ERR : mLgsAbortSession();next_state = LGS_RUN; break;
				case EV_LGS_SEND_SAPRSP : mLgsSendRequestedSap();next_state = LGS_SEND_SAPRSP; break;
				case EV_SES_ACK : next_state = LGS_ACKSAPRSP; break;
				case EV_BRC_SL_SESS_TIMEOUT : mLgsAbortSession();next_state = LGS_RUN; break;
			} break;
		case LGS_ACKSAPRSP:
			switch (event) {
				case EV_SES_SD2 : next_state = mLgsCheckAckSAPRspFrame(); break;
				case EV_BRC_SL_SESS_TIMEOUT : mLgsAbortSession();next_state = LGS_RUN; break;
			} break;
		case LGS_SEND_ACKSAPRSPACK:
			switch (event) {
				case EV_BRC_SL_SESS_TIMEOUT : mLgsAbortSession();next_state = LGS_RUN; break;
				case EV_SES_ERR : mLgsAbortSession();next_state = LGS_RUN; break;
				case EV_LGS_SEND_ACKSAPRSPACK : mLgsSendAckSAPRspAck();next_state = LGS_SEND_ACKSAPRSPACK; break;
				case EV_SES_ACK : mLgsLogOnSlaveOk();next_state = LGS_RUN; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			DebugPrint(lgs_state+LOG_STATE_OFFSET);DebugPrint(next_state+LOG_STATE_OFFSET);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[lgs_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	lgs_state = next_state;
	}
}

void Job0Slave()
{
unsigned char next_state = _NULL;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (j0s_state) {
		case J0S_IDLE:
			switch (event) {
				case EV_BRC_ENABLE : next_state = J0S_IDLE; break;
				case EV_J0S_RUN : mJ0sRun();next_state = J0S_RUN; break;
			} break;
		case J0S_RUN:
			switch (event) {
				case EV_SES_SD2 : next_state = mJ0sCheckJobReqFrame(); break;
				case EV_BRC_J0S_SESS_TIMEOUT : mJ0sAbortSession();next_state = J0S_IDLE; break;
			} break;
		case J0S_SEND_JOBACK:
			switch (event) {
				case EV_SES_ERR : mJ0sAbortSession();next_state = J0S_IDLE; break;
				cavoidse EV_J0S_SEND_JOBACK : mJ0sSendJobAck();next_state = J0S_SEND_JOBACK; break;
				case EV_SES_ACK : mJ0sBuildJobRsp();next_state = J0S_SEND_JOBRSP; break;
				case EV_BRC_J0S_SESS_TIMEOUT : mJ0sAbortSession();next_state = J0S_IDLE; break;
			} break;
		case J0S_SEND_JOBRSP:
			switch (event) {
				case EV_SES_ERR : mJ0sAbortSession();next_state = J0S_IDLE; break;
				case EV_J0S_SEND_JOBRSP : mJ0sSendJobRsp();next_state = J0S_SEND_JOBRSP; break;
				case EV_SES_ACK : next_state = J0S_WAIT_ACK_JOBRSP; break;
				case EV_BRC_J0S_SESS_TIMEOUT : mJ0sAbortSession();next_state = J0S_IDLE; break;
			} break;
		case J0S_WAIT_ACK_JOBRSP:
			switch (event) {
//MG001				case EV_SES_SD2 : mJ0sSlaveOk();next_state = J0S_IDLE; break;
				case EV_SES_SD2 : next_state = mJ0sSlaveOk();break;   //MG001
				case EV_SES_ERR : mJ0sAbortSession();next_state = J0S_IDLE; break;
				case EV_BRC_J0S_SESS_TIMEOUT : mJ0sAbortSession();next_state = J0S_IDLE; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			dbg_byte = j0s_state+LOG_STATE_OFFSET;
			UltiUART2_FifoWrite(&dbg_byte, 1);
			dbg_byte = next_state+LOG_STATE_OFFSET;
			  UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[j0s_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	j0s_state = next_state;
	}
}

void JobSlave()
{
unsigned char next_state = _NULL;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (jbs_state) {
		case JBS_IDLE:
			switch (event) {
				case EV_BRC_ENABLE : next_state = JBS_IDLE; break;
				case EV_JBS_RUN : mJbsRun();next_state = JBS_RUN; break;
			} break;
		case JBS_RUN:
			switch (event) {
				case EV_SES_SD2 : next_state = mJbsCheckJobReqFrame(); break;
				case EV_BRC_JBS_SESS_TIMEOUT : mJbsAbortSession();next_state = JBS_IDLE; break;
			} break;
		case JBS_SEND_JOBACK:
			switch (event) {
				case EV_SES_ERR : mJbsAbortSession();next_state = JBS_IDLE; break;
				case EV_JBS_SEND_JOBACK : mJbsSendJobAck();next_state = JBS_SEND_JOBACK; break;
				case EV_SES_ACK : mJbsBuildJobRsp();next_state = JBS_SEND_JOBRSP; break;
				case EV_BRC_JBS_SESS_TIMEOUT : mJbsAbortSession();next_state = JBS_IDLE; break;
			} break;
		case JBS_SEND_JOBRSP:
			switch (event) {
				case EV_SES_ERR : mJbsAbortSession();next_state = JBS_IDLE; break;
				case EV_JBS_SEND_JOBRSP : mJbsSendJobRsp();next_state = JBS_SEND_JOBRSP; break;
				case EV_SES_ACK : next_state = JBS_WAIT_ACK_JOBRSP; break;
				case EV_BRC_JBS_SESS_TIMEOUT : mJbsAbortSession();next_state = JBS_IDLE; break;
			} break;
		case JBS_WAIT_ACK_JOBRSP:
			switch (event) {
				case EV_SES_SD2 : next_state = mJbsSlaveOk(); break;
				case EV_SES_ERR : mJbsAbortSession();next_state = JBS_IDLE; break;
				case EV_BRC_JBS_SESS_TIMEOUT : mJbsAbortSession();next_state = JBS_IDLE; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			DebugPrint(jbs_state+LOG_STATE_OFFSET);DebugPrint(next_state+LOG_STATE_OFFSET);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[jbs_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	jbs_state = next_state;
	}
}



// Post event
void ev_post(unsigned char ev)
{
	ev_queue[ev_queue_rd] = ev;
	ev_queue_rd = (ev_queue_rd + 1) % N_EVPOST;
	queue_empty = 0;
}
#endif
// Get Event

void ev_move(struct s_MPIdata *pMPIdata, unsigned char ev)
{
#if EVENTLOG
   #if !SHORT_FORM
	  char s[20];
   #endif
#endif

   ev_post(pMPIdata, ev);
   pMPIdata->event = ev_get(pMPIdata);
	while ( pMPIdata->event )
	{
	  #if EVENTLOG
		#if SUBSET_EVENTLOG
			   if (msgEventsLog[event])
			   {
		 #endif
		 #if SHORT_FORM
				  dbg_byte = event + LOG_EVENT_OFFSET;
					UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
					sprintf(s, "\n\r%s:", msgEvents[event]);
				 DbgStrPrint(s);
	   #endif
		#if SUBSET_EVENTLOG
			   }
		 #endif
		#endif

	  Token(pMPIdata);
	  LogOff(pMPIdata);

	  #if MPI_SLAVE_STACK
		 LogSlave();
	  #endif

	  #if MPI_MASTER_STACK
		 Job(pMPIdata);
		 Log(pMPIdata);
		 Job0(pMPIdata);
	  #endif

	  #if MPI_SLAVE_STACK
		 Job0Slave(pMPIdata);
		 JobSlave(pMPIdata);
	  #endif

	  if (pMPIdata->queue_empty)
	  {
		 return;
	  }
	  pMPIdata->event = ev_get(pMPIdata);
   }
}

#if 0	//^^???
void setRxFifoThrForMPI(struct imx_port *sport, int status)
{
	unsigned int TLRtmp;
	/* FIFOs and DMA Settings */

	/* FCR can be changed only when the
	 * baud clock is not running
	 * DLL_REG and DLH_REG set to 0.
	 */

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_DLL, 0);
	serial_out(up, UART_DLM, 0);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	sport->efr = serial_in(up, UART_EFR) & ~UART_EFR_ECB;
	sport->efr &= ~UART_EFR_SCD;
	serial_out(up, UART_EFR, sport->efr | UART_EFR_ECB);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	sport->mcr = serial_in(up, UART_MCR) & ~UART_MCR_TCRTLR;
	serial_out(up, UART_MCR, sport->mcr | UART_MCR_TCRTLR);
	/* FIFO ENABLE, DMA MODE */

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	TLRtmp = serial_in(up, UART_OMAP_TLR) & 0x0f;
	serial_out(up, UART_OMAP_TLR, ((status)?0x8F:0) | TLRtmp);	//rxFIFOth=33, txFIFth=60

	/* Reset UART_MCR_TCRTLR: this must be done with the EFR_ECB bit set */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_MCR, sport->mcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, sport->efr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
}
#endif

void MPIDriverOpen(struct s_MPIdata *pMPIdata, struct s_MPIparams init)
{
	if (pMPIdata->m_isOpen)
	{
		pMPIdata->FrameUniopNodeNum = init.panelNode;	//????????????? do it in open
		ResetStations(pMPIdata);
	}
	else
	{
		pMPIdata->FrameUniopNodeNum = init.panelNode;	//????????????? do it in open
		pMPIdata->MaxStationAddress = init.maxNode;
		pMPIdata->applResponseTime = init.applTimeout;
		pMPIdata->guardTimeConstant = init.guardTimeoutConstant;
		pMPIdata->guardTimeFactor = init.guardTimeoutFactor;
		pMPIdata->ackGuardTime = init.ackTimeout;
		pMPIdata->FDLTimeout = init.fdlTimeout;
		pMPIdata->tokTimeout = init.tokTimeout;
		pMPIdata->selfTokTimeout = init.selfTokTimeout;
		pMPIdata->SlaveSession = false;
		pMPIdata->ProcedureApplMRequestPending  = false;
		pMPIdata->ProcedureApplMResponsePending = false;

		pMPIdata->LowestAddress = 0xFF;
		pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
		pMPIdata->NextStation = -1;
		pMPIdata->AreWeInRing = false;
		pMPIdata->ReadyForConfigMode = 0;
		pMPIdata->GapUpdateFactor = 5;
		pMPIdata->Start_LogOn_SAE = 20;
		pMPIdata->Start_LogOn_DAE = 20;
		mInitEngine(pMPIdata);
		mTokStartTimeout10(pMPIdata);
		Init_ev_queue(pMPIdata);
		ev_move(pMPIdata, EV_BRC_ENABLE);
		pMPIdata->m_taskLen = 0;
		pMPIdata->m_isOpen = true;
		pMPIdata->MPIenabled = true;
		pMPIdata->rxCnt = 0;
	}
}

static void mpiReceiveFrame(struct s_MPIdata *pMPIdata, unsigned char *buf, unsigned int len)
{
	unsigned char l;
	StopTimeoutTok(pMPIdata);
	while (len) {
		switch (buf[0])
		{
			case D_SD1:
				if (len < 6) {
					len = 0;
					break;
				}
				pMPIdata->FrameSessionReceiveLength = 6;
				memcpy(pMPIdata->FrameSessionReceiveBuffer, buf, 6);
				pMPIdata->Dest = buf[1];
				pMPIdata->Source = buf[2];
				DBGReg(255, D_SD1, MAKEWORD(pMPIdata->Source, pMPIdata->Dest));
				pMPIdata->FlowCtrl = buf[3];
				pMPIdata->StationStatus[pMPIdata->Source].IsActive = 1;
	#ifdef MANAGE_RR_ANSWERS
				if (pMPIdata->Dest == pMPIdata->FrameUniopNodeNum && (pMPIdata->FlowCtrl & 0x4F) == 2)
					ev_move(pMPIdata, EV_BRC_SDX);
				else
	#endif
				ev_move(pMPIdata, EV_BRC_SD1);
				len -= 6;
				buf += 6;
				pMPIdata->rxCnt -= 6;
				break;
			case D_SD2:
				l =  buf[1]+6;
				if (len < l) {
					len = 0;
					break;
				}
				pMPIdata->FrameSessionReceiveLength = l;
				memcpy(pMPIdata->FrameSessionReceiveBuffer, buf, l);
				pMPIdata->FlowCtrl = buf[6];
				pMPIdata->Dest = buf[4] & 0x7F;
				pMPIdata->Source = buf[5] & 0x7F;
#if 0
				{
					byte AM_JobNr = pMPIdata->FrameSessionReceiveBuffer[9] == 0xB0 ? pMPIdata->FrameSessionReceiveBuffer[11] : pMPIdata->FrameSessionReceiveBuffer[10];
					DBGReg(255, D_SD2, MAKEWORD(((pMPIdata->Source << 4) | (pMPIdata->Dest & 0x0F)),
												((pMPIdata->FrameSessionReceiveBuffer[9] & 0xF0) |
												(AM_JobNr & 0x0F))));
				}
#endif
				len -= l;
				buf += l;
				pMPIdata->rxCnt -= l;
				pMPIdata->StationStatus[pMPIdata->Source].IsActive = 1;
				if (pMPIdata->Dest != pMPIdata->FrameUniopNodeNum)
				{
					if (pMPIdata->tok_state == TOK_WAITRX ||         //MG001
						pMPIdata->tok_state == TOK_LISTENTOKEN ||    //MG001
						pMPIdata->tok_state == TOK_ACTIVEIDLE)       //MG001
						mTokListenTokenStartTimeout(pMPIdata);      //MG001
					break;
				}
	#ifdef MANAGE_RR_ANSWERS
				if ((pMPIdata->FlowCtrl & 0x4F) == 2)
					ev_move(pMPIdata, EV_BRC_SDX);
				else
	#endif
					ev_move(pMPIdata, EV_SES_SD2);
				break;
			case D_TOK:
				if (len < 3) {
					len = 0;
					break;
				}
				pMPIdata->Dest   = buf[1] & 0x7F;                        //MG002
				pMPIdata->Source = buf[2] & 0x7F;                        //MG002
				len -= 3;
				buf += 3;
				pMPIdata->rxCnt -= 3;
				DBGReg(255, D_TOK, MAKEWORD(pMPIdata->Source, pMPIdata->Dest));
				if ((pMPIdata->Source > NR_MAX_STATIONS - 1) || (pMPIdata->Dest > NR_MAX_STATIONS - 1)) {  //MG002
					break;                                                            //MG002
				}
				pMPIdata->StationStatus[pMPIdata->Source].IsActive = 1;
				pMPIdata->StationStatus[pMPIdata->Source].StationType = 3;
				ev_move(pMPIdata, EV_BRC_TOK);
				break;
			case D_SC:
				DBGReg(255, D_SC, 0);
				len -= 1;
				buf += 1;
				pMPIdata->rxCnt -= 1;
				if (pMPIdata->tok_state == TOK_WAITSESSRX)
					ev_move(pMPIdata, EV_BRC_ACK); // consider ack only if I sent SD2 req
				break;
			default:
				len--;
				buf++;
				pMPIdata->rxCnt--;
				break;
		} //switch(FrameSessionReceiveBuffer[0])
	}
	if (pMPIdata->rxCnt) {
		memcpy(pMPIdata->mpiRxBuf, buf, pMPIdata->rxCnt);
	}
}

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

#ifdef EXOR_MPI
static int serial_ioctl_mpi(struct imx_port *sport, unsigned int cmd, unsigned long arg)
{
	unsigned long flags = 0;
	int ret = 0;
	switch (cmd) {
		case SET_MPI_MODE:
			if (!sport->SCNKdata.SCNKenabled) {
				spin_lock_irqsave(&sport->port.lock, flags);
				sport->MPIdata.mpiSport = sport;
				MPIDriverInit(&sport->MPIdata);
				spin_unlock_irqrestore(&sport->port.lock, flags);
			}	
			break;
		case MPI_OPEN:
			if (!sport->SCNKdata.SCNKenabled) {
				struct s_MPIparams init;
				if (copy_from_user(&init, (struct s_MPIparams *) arg, sizeof(init)))
					ret = -EFAULT;
				else {
					spin_lock_irqsave(&sport->port.lock, flags);
					MPIDriverOpen(&sport->MPIdata, init);
					spin_unlock_irqrestore(&sport->port.lock, flags);
				}
			}
			break;

		case MPI_RUN:
			{
				spin_lock_irqsave(&sport->port.lock, flags);
				ev_move(&sport->MPIdata, EV_TOK_RUN);					//do it in run
				spin_unlock_irqrestore(&sport->port.lock, flags);
			}
			break;

		case MPI_IS_OPEN:
			if (copy_to_user((unsigned int *)arg, &sport->MPIdata.m_isOpen, sizeof(sport->MPIdata.m_isOpen)))
				ret = -EFAULT;
			break;

		case GET_MPI_DIAG:
//			if (copy_to_user((unsigned int *)arg, &(sport->SCNKdata.diag_cnt), sizeof(sport->SCNKdata.diag_cnt)))
				ret = -EFAULT;
			break;

		case SET_MPI_DIAG:
			{
				unsigned int tmp[2];
				if (copy_from_user(tmp, (unsigned int*) arg, sizeof(tmp)))
					ret = -EFAULT;
			}
			break;

		case MPI_CLOSE:
			spin_lock_irqsave(&sport->port.lock, flags);
			mExitFromRing(&sport->MPIdata);
			UltiUart1_StopTimer(&sport->MPIdata);
			usleep_range(10000,11000);
			sport->MPIdata.m_isOpen = false;
			sport->MPIdata.MPIenabled = false;
			sport->MPIdata.MPImode = false;
			spin_unlock_irqrestore(&sport->port.lock, flags);
			break;

		case SET_MPI_DATA:
			{
				unsigned int tmp;
				if (copy_from_user(&tmp, (unsigned int *) arg, sizeof(tmp)))
					ret = -EFAULT;
				else {
					sport->MPIdata.FrameJobSessionLen = tmp;
					if (copy_from_user(sport->MPIdata.FrameJobSessionBuff, (unsigned char*) arg + sizeof(tmp), tmp))
						ret = -EFAULT;
				}
			}
			break;

		case SET_MPI_REQ:
			{
				int tmp;
				if (copy_from_user(&tmp, (int *) arg, sizeof(tmp)))
					ret = -EFAULT;
				else {
					spin_lock_irqsave(&sport->port.lock, flags);
					sport->MPIdata.ProcedureApplMRequestPending = tmp;
					sport->MPIdata.ProcedureApplMResponsePending = tmp;
					sport->MPIdata.ProcedureApplMStatus = M_PROC_RUNNING;
					sport->MPIdata.applTryCnt = sport->MPIdata.applResponseTime / 5; //init timeout counting
					spin_unlock_irqrestore(&sport->port.lock, flags);
				}
			}
			break;
		case GET_MPI_RQST:
			{
				unsigned char plcIndex;
				if (copy_from_user(&plcIndex, (unsigned char *) arg, sizeof(plcIndex)))
					ret = -EFAULT;
				else {
					byte tmp;
					spin_lock_irqsave(&sport->port.lock, flags);
					if (sport->MPIdata.ProcedureApplMRequestPending)
					{
						// Check whether there really was no traffic on the line at all !!!!!
						if (sport->MPIdata.AreWeInRing && sport->MPIdata.StationStatus[plcIndex].IsActive &&
							sport->MPIdata.StationStatus[plcIndex].StationType >= 2)
						{
							if (sport->MPIdata.SlaveSession == false)
							{
								if (!sport->MPIdata.StationStatus[plcIndex].Logged)
								{
									if (sport->MPIdata.StationStatus[plcIndex].LogStatus == 0)
									{
										sport->MPIdata.LogOn_DA = plcIndex;
										ev_post(&sport->MPIdata, EV_LOG_START);
										sport->MPIdata.StationStatus[plcIndex].LogStatus = 1;
									}
								}
								else
								{
									sport->MPIdata.Last_DA = plcIndex;
									if (sport->MPIdata.StationStatus[sport->MPIdata.Last_DA].Job == 0)
									{
										if (sport->MPIdata.StationStatus[plcIndex].LogStatus == 1)
										{
											ev_post(&sport->MPIdata, EV_JB0_SEND_REQ);
											sport->MPIdata.StationStatus[plcIndex].LogStatus = 2;
										}
									}
									else
									{
										ev_post(&sport->MPIdata, EV_JOB_SEND_REQ);
										sport->MPIdata.ProcedureApplMRequestPending = false;
									}
								}
							}
						}
					}

					/* Comm sequence is over when ProcedureMResponsePending == false     */
					if (sport->MPIdata.ProcedureApplMResponsePending)
					{
						// check application timeout
						if (--sport->MPIdata.applTryCnt == 0)
						{
							sport->MPIdata.ProcedureApplMStatus = TIMEOUT_ERR;
							DBGReg(239, sport->MPIdata.ProcedureApplMStatus, 0x0006);
							sport->MPIdata.ProcedureApplMRequestPending = false;
							sport->MPIdata.ProcedureApplMResponsePending = false;
							ev_post(&sport->MPIdata, EV_BRC_APPL_TIMEOUT);
						}
					}

					if (NO_ERROR == sport->MPIdata.ProcedureApplMStatus) {
						sport->MPIdata.ProcedureApplMStatus = M_PROC_OK;
					}
					tmp = sport->MPIdata.ProcedureApplMStatus;
					spin_unlock_irqrestore(&sport->port.lock, flags);
					if (copy_to_user((unsigned char *)arg, &tmp, sizeof(sport->MPIdata.ProcedureApplMStatus)))
						ret = -EFAULT;
				}
			}
			break;
		case GET_MPI_RESP:
			{
				unsigned int len = sport->MPIdata.MyFrameResponseBuffLen;
				if (copy_to_user((unsigned char *)arg, &len, sizeof(len)))
					ret = -EFAULT;
				else if (copy_to_user((unsigned char *)arg+sizeof(len), &sport->MPIdata.MyFrameResponseBuff, len))
					ret = -EFAULT;
			}
			break;
		default:
			ret = -ENOIOCTLCMD;
			break;
	}
	return ret;
}
#endif

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
			if ( arg == 0 || sport->MPIdata.MPIenabled){
				sport->SCNKdata.SCNKenabled = false;
				break;
			}
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
			sport->port.read_status_mask |= (URXD_FRMERR | URXD_PRERR);
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
#ifdef EXOR_MPI
			return serial_ioctl_mpi(sport, cmd, arg);
#else
			return -ENOIOCTLCMD;
#endif			
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
#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
#ifdef EXOR_MPI
		if (sport->MPIdata.MPIenabled)
			ev_move(&sport->MPIdata, EV_BRC_EOTX);
#endif
#endif
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

#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
	if (sport->SCNKdata.SCNKenabled)
	{
		xmit = &sport->SCNKdata.txBuf;
	}
#ifdef EXOR_MPI
	else if(sport->MPIdata.MPIenabled) {
		xmit = &sport->MPIdata.mpiTxBuf;
	}
#endif
#endif
//	printk("ITB %X,%X,%02X,%d\n",readl(sport->port.membase + USR2),txfullflag & UTS_TXFULL, xmit->buf[xmit->tail], uart_circ_chars_pending(xmit));
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

	if((ttyport->low_latency) && ((sport->txfullflag & UTS_TXFULL) != UTS_TXFULL) )
	{ //Handle the low latency option for REALTIME protocol
		if (!sport->dma_is_enabled)
		{ // Clear the IRTS flag to keep the TX stopped while feedint the TX buffer (if we are not refilling on the fly the current packet)
			temp = readl(sport->port.membase + UCR2);
			temp &= ~UCR2_IRTS;
			writel(temp, sport->port.membase + UCR2);
		}
	}
         
	sport->txfullflag = 0;
	while (!uart_circ_empty(xmit) &&
		   !( (sport->txfullflag = readl(sport->port.membase + uts_reg(sport))) & UTS_TXFULL)) {
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
	
//	printk("<<<< imx_transmit_buffer %d, %d\n",ttyport->low_latency, txfullflag & UTS_TXFULL);
	if(ttyport->low_latency)
	{ //Handle the low latency option for MPI protocol
		if ((sport->txfullflag & UTS_TXFULL) && (!sport->dma_is_enabled) )
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
//	printk("<<<< imx_txint %02X\n",readl(sport->port.membase + USR2));
	imx_transmit_buffer(sport);
	spin_unlock_irqrestore(&sport->port.lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t imx_rxint(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned int rx, flg;
#if !(defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE))
	unsigned int ignored = 0;
#endif
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

//			printk(KERN_ALERT ">>>> WRONG Char2 %d %X %X\n", sport->port.read_status_mask, rx, rx & (sport->port.read_status_mask | 0xFF));

#if !(defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE))
			if (rx & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}
#endif
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
				sport->SCNKdata.expectedLen = 3;	//exit
				sport->SCNKdata.rxLen = 0;
//				printk(KERN_ALERT ">>>> WRONG Char %d \n", sport->SCNKdata.rxLen);
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
							sport->SCNKdata.expectedLen = 3;	//exit
							sport->SCNKdata.rxLen = 0;
//							printk(KERN_ALERT ">>>> WRONG REQ %X \n", rx);
							break;
					}
				}
				else if (sport->SCNKdata.rxLen == 3 && (sport->SCNKdata.rxBuf[1] & 0x3F) == REQ_VARI)
				{
					if (sport->SCNKdata.rxBuf[2] < 6 || sport->SCNKdata.rxBuf[2] > 78)
					{
						while (readl(sport->port.membase + USR2) & USR2_RDR)	//disregard the rest
							rx = readl(sport->port.membase + URXD0);
						sport->SCNKdata.expectedLen = 3;	//exit
						sport->SCNKdata.rxLen = 0;
//						printk(KERN_ALERT ">>>> WRONG Len %d \n", sport->SCNKdata.rxBuf[2]);
					}
					else
						sport->SCNKdata.expectedLen = sport->SCNKdata.rxBuf[2] + (sport->SCNKdata.useCRC?6:4);
				}
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
#ifdef EXOR_MPI
		else if (sport->MPIdata.MPIenabled) {
			if (flg != TTY_NORMAL){
				while (readl(sport->port.membase + USR2) & USR2_RDR)	//disregard the rest
					rx = readl(sport->port.membase + URXD0);
				sport->MPIdata.rxCnt = 0;
				printk(KERN_ALERT ">>>> WRONG Char %d \n", sport->MPIdata.rxCnt);
			}
			else {
//				printk("RX:%02X\n", (unsigned char)rx);
				if (sport->MPIdata.rxCnt < 280)
					sport->MPIdata.mpiRxBuf[sport->MPIdata.rxCnt++] = (unsigned char)rx;
				else
					printk("frame RX overflow\n");

				if (sport->MPIdata.UltiUart1_TxTimeout != EV__NULL && sport->MPIdata.UltiUart1_TxTimeout != EV_TOK_RUN){
					UltiUART1_StartTimer(&sport->MPIdata, EV__GAP, 160);	//160 usec is the gap
				}
			}
		}
#endif
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
#ifdef EXOR_MPI
	sport->MPIdata.MPIenabled = false;
#endif	
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
		imx_rs485_stop_tx(sport); 
#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
		 sport->SCNKdata.SCNKenabled = false;
#ifdef EXOR_MPI
		if (sport->MPIdata.MPIenabled)
		{
			hrtimer_try_to_cancel(&sport->MPIdata.hrt);
			sport->MPIdata.m_isOpen = false;
			sport->MPIdata.MPIenabled = false;
			sport->MPIdata.MPImode = false;
		}
#endif
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

static void imx_set_termios(struct uart_port *port, struct ktermios *termios,
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
#if defined(CONFIG_SERIAL_IMX_EXOR_UART) || defined(CONFIG_SERIAL_IMX_EXOR_UART_MODULE)
	if (termios->c_iflag & INPCK || sport->SCNKdata.SCNKenabled || sport->MPIdata.MPIenabled)
		sport->port.read_status_mask |= (URXD_FRMERR | URXD_PRERR);
#else
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |= (URXD_FRMERR | URXD_PRERR);
#endif
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
static int imx_verify_port(struct uart_port *port, struct serial_struct *ser)
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
static void imx_console_write(struct console *co, const char *s, unsigned int count)
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
