/*
 *  dviplugin.h 
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
#ifndef _DVIPLUGINIMPL_H
#define _DVIPLUGINIMPL_H

#define MAXRESVAL (2048)
#define MAXCOORDVALUE (999)

#define SHOW_WCE_INPUT  0
#define SHOW_DVI_INPUT  1
#define SHOW_DVI_INPUT2 2

#define DVI_CVIINPUTISVALID (0x519)

/*
 * FPGA cores base addresses
 */
#define WCE_CVI_BASE        (0xc1000440)
#define WCE_CVI3_BASE       (0xc1000480)
#define WCE_CVI2_BASE       (0xc10004C0)
#define WCE_SCALER_BASE     (0xc1000E00) 
#define ADP_CVO_BASE        (0xc1000800)
#define DVI_CVI_BASE        (0xc1000400)
#define DVI_CVO_BASE        (0xc1000000)
#define MISCCTRLREG         (0xc0000800)
#define DVIINPUTSTATREG     (0xc0000810)
#define DATACTRL_REG        (0xc0000810) /*This is the same register, with different meaning when written*/
#define DVISELOUTREG        (0xc0000820)
#define VIDIN_ALPHA_BASE    (0xc1000500) 
#define VIDIN_SCALER_BASE   (0xc1000600) 
#define DVIPLLBASE          (0xc0000000)
#define ADR_HIDDEV_BASE     (0xa0004000)
#define DVO_ALPHA_BASE      (0xc1000C00)
#define DVO_FRAMER_BASE     (0xc1000D00)
#define DVO_TP_BASE         (0xc1000D00)

#define FBADDR_WCE          (0x00000000)
#define FBADDR_DVIIN        (0x20000000)

/*
 * FractionalPLL reconfig. core registers
 */
#define PLL_MODE            (0x00 << 2)
#define PLL_STATUS          (0x01 << 2)
#define PLL_START           (0x02 << 2)
#define PLL_NCOUNT          (0x03 << 2)
#define PLL_MCOUNT          (0x04 << 2)
#define PLL_CCOUNT          (0x05 << 2)
#define PLL_PHASE           (0x06 << 2)
#define PLL_KFRAC           (0x07 << 2)
#define PLL_BANDWIDTH       (0x08 << 2)
#define PLL_CHARGEPUMP      (0x09 << 2)
#define PLL_VCODIV          (0x1C << 2)


/*
 * Clocked Video IN II registers
 */
#define CVI_CONTROL         (0x00 << 2)
#define CVI_STATUS          (0x01 << 2)
#define CVI_SAMPLECOUNT     (0x04 << 2)
#define CVI_LINECOUNT       (0x05 << 2)

/*
 * Clocked Video OUT II registers
 */
#define CVO_CONTROL         (0x00 << 2)
#define CVO_STATUS          (0x01 << 2)
#define CVO_MODEX_CTRL      (0x05 << 2)
#define CVO_MODEX_XRES      (0x06 << 2)
#define CVO_MODEX_VRES      (0x07 << 2)
#define CVO_MODEX_HSFP      (0x09 << 2)
#define CVO_MODEX_HSW       (0x0a << 2)
#define CVO_MODEX_HSSUM     (0x0b << 2)
#define CVO_MODEX_VSFP      (0x0c << 2)
#define CVO_MODEX_VSW       (0x0d << 2)
#define CVO_MODEX_VSSUM     (0x0e << 2)
#define CVO_MODEX_VALID     (0x1c << 2)

/*
 * ALPHA BLENDING registers
 */
#define ALPHA_CONTROL       (0x00 << 2)
#define ALPHA_STATUS        (0x01 << 2)
#define ALPHA_L1_X          (0x02 << 2)
#define ALPHA_L1_Y          (0x03 << 2)
#define ALPHA_L1_ENA        (0x04 << 2)
#define ALPHA_L2_X          (0x05 << 2)
#define ALPHA_L2_Y          (0x06 << 2)
#define ALPHA_L2_ENA        (0x07 << 2)

/*
 * SCALER registers
 */
#define SCALER_CONTROL      (0x00 << 2)
#define SCALER_STATUS       (0x01 << 2)
#define SCALER_WIDTH        (0x03 << 2)
#define SCALER_HEIGHT       (0x04 << 2)

/*
 * Frame reader core
 */
#define FRAMER_CONTROL (0x00 << 2)
#define FRAMER_STATUS  (0x01 << 2)
#define FRAMER_BA      (0x04 << 2)
#define FRAMER_WORDS   (0x05 << 2)
#define FRAMER_SCYCP   (0x06 << 2)
#define FRAMER_WIDTH   (0x08 << 2)
#define FRAMER_HEIGHT  (0x09 << 2)

/*
 * Clipper core
 */
#define CLIPPER_CONTROL (0x00 << 2)
#define CLIPPER_X       (0x03 << 2)
#define CLIPPER_WIDTH   (0x04 << 2)
#define CLIPPER_Y       (0x05 << 2)
#define CLIPPER_HEIGHT  (0x06 << 2)

/* 
 * Test pattern gen II
 */
#define TP_CONTROL (0x00 << 2)
#define TP_WIDTH   (0x03 << 2)
#define TP_HEIGHT  (0x04 << 2)
#define TP_R       (0x05 << 2)
#define TP_G       (0x06 << 2)
#define TP_B       (0x07 << 2)

/*
 * Data control register
 */
#define DCTRL_VSYNC_EN  (0x1 << 0)
#define DCTRL_VSYNC_INV (0x1 << 1)
#define DCTRL_HSYNC_EN  (0x1 << 2)
#define DCTRL_HSYNC_INV (0x1 << 3)
#define DCTRL_BLANK_EN  (0x1 << 4)
#define DCTRL_BLANK_INV (0x1 << 5)
#define DCTRL_VCLK_INV  (0x1 << 6)

/*
 * HIDDEV Controller registers
 */

#define REG_RESET       (ADR_HIDDEV_BASE + 0x014)       // bit    name       default    description
#define RESET_MASK      0x0001                          //  0     reset         1       = 1 under reset,     = 0 dma engine active
#define HID_MASK        0x0002                          //  1     hid           0       = 1 hid device mode, = 0 debug mode

#define REG_HID_STATUS  (ADR_HIDDEV_BASE + 0x020)       // bit    name       default    description
#define EMPTY_MASK      0x8000                          //  15    empty         1       = 1 Fifo empty
#define FULL_MASK       0x4000                          //  14    full          0       = 0 Fifo full
#define FIFO_ST_MASK    0x3FFF                          // 0..13  fifo_count    0       = number of bytes stored in Fifo

#define REG_HID_DATA    (ADR_HIDDEV_BASE + 0x01C)       // bit    name       default    description

#define REG_HID_CTL     (ADR_HIDDEV_BASE + 0x024)       // bit    name       default    description
#define COMMIT_MASK     0x0001                          //  0     commit        0       = 1 enable fifo transfer
#define AUTO_MASK       0x0002                          //  1     auto          0       = 1 enable auto-commit

/* Data structure holding the video input configuration
 */
struct vidin_config
{
	u32 enabled; //0=Video input disabled 1=Video input enabled (visible only over transparent color 0x0001)
	u32 xoffs;   //X offset of the video input image [pixels]
	u32 yoffs;   //Y offset of the video input image [pixels]
	u32 width;   //Width of the video input image [pixels]
	u32 height;  //Height of the video input image [pixels]
};

/* Data structure holding the dvi plugin instance params
 */
struct dviplugin_data {
	struct device       *dev;
	struct mutex        lock;
	struct i2c_adapter  *i2cadap;           //I2C adapter to be used for accessing the i2c devices of the module
	struct t_DisplayParams     DVIparams;   //DVI parameters/timings
	struct t_DisplayParams     ADPparams;   //ADP parameters/timings used for local display (if used/present). All 0s if not used.
	struct spi_device   *spi;               //Pointer to spi device
	u32                 adp_cvo_enabled;    //Flag indicating if the corresponding core should be actually enabled
	u32                 wce_cvi_enabled;    //Flag indicating if the corresponding core should be actually enabled
	u32                 dvi_cvo_enabled;    //Flag indicating if the corresponding core should be actually enabled
	u32                 dvi_cvi_enabled;    //Flag indicating if the corresponding core should be actually enabled
	u32                 fexiting;           //Flag indicating if the driver is being unloaded
	u32                 seldviout;          //Flag indicating which video source has to be routed to the DVI output port
	u32                 DVIinStatus;        //Indicates the DVI input presence (0 : No signal, !=0 : Signal present, LSW=XRES MSW=YRES)
	u32                 adp_dualLVDS;       //LVDS out configuration for local display/ADP (0x00=single LVDS, 0x02=Dual LVDS) 
	struct vidin_config vidin;              //Video input configuration
	// Polling thread stuff
	struct task_struct* auto_update;
	struct completion   auto_update_stop;
	// Emulated mouse thread stuff
	struct task_struct* mouse_update;
	struct completion   mouse_update_stop;
	int                 mouseevent_x;
	int                 mouseevent_y;
	int                 mouseevent_lbutt;
	int                 mouseevent_rbutt;
	int	            mouseevent_flag;
	wait_queue_head_t   send_wq;
};

#if 0
typedef struct 
{
	HANDLE              hthread;            //Monitor thread handle
	VIDIN_CONFIG        vidin;              //Video input configuration
	HANDLE              mousethread;        //Thread for forwarding filtered mouse/touch events
	HANDLE              mousemovedevent;    //Event for signalling a new mouse/touch event 
	DVI_MOUSEEVNT       mouseevent;         //Last received Mouse/touch event for touch redirection
} DVI_DEVICE;
#endif

/*
 * Data structure for programming the fractions PLL
 */
typedef struct 
{
	u32 f_Khz;
	u32 pll_mode;
	u32 pll_ncount;
	u32 pll_mcount;
	u32 pll_ccount;      
	u32 pll_phase;        
	u32 pll_kfrac;        
	u32 pll_bandwidth;    
	u32 pll_chargepump;   
	u32 pll_vcodiv;       
} PLL_DATA;

/*
 * Function declarations
 */
int DVIwriteReg(struct spi_device *spi, u32 regaddr, u32 value);     //Write a 32 bit register at the specified (32 bit) address
u32 DVIreadReg(struct spi_device *spi, u32 regaddr);                 //Read a 32 bit register from the specified (32 bit) address

int pca9557_init(struct i2c_adapter  *adap);
int pca9557_set_out(struct i2c_adapter  *adap, unsigned char n);
int pca9557_clr_out(struct i2c_adapter  *adap, unsigned char n);

int TFP410_CheckPresence(struct i2c_adapter  *adap);
int TFP410_Configure(struct i2c_adapter  *adap);
int TFP410_Stop(struct i2c_adapter  *adap);

int adv7611_CheckPresence(struct i2c_adapter  *adap);
int adv7611_Configure(struct i2c_adapter  *adap);

int InitFPGACores(struct dviplugin_data *pDevice);
int DVI_Configureswitch(struct dviplugin_data *pDevice);
int VIDIN_Configure(struct dviplugin_data* pDevice);

int monitor_thread(void *p);
int InitDVI_CVI(struct dviplugin_data *pDevice);
int InitDVI_CVO(struct dviplugin_data *pDevice);

void Redirect2USBDEvice(struct dviplugin_data* pDevice);
int  mouse_thread(void* p);                                          //Mouse thread for forwarding touch/mouse events
#endif