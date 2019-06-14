#ifndef KAIROS_H
#define KAIROS_H

#include <linux/device.h>
#include <linux/netdevice.h>
#include <net/dsa.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

// PTP support
#include <linux/ptp_clock_kernel.h>

#include "kairos-regs.h"

//#define VERBOSE

#define KAIROS_ERR_OK			0

/* shadow registers */
#define TSN_SHADOW_CNTRL_STATUS 		0
#define TSN_SHADOW_TGDCTRL				1
#define TSN_SHADOW_FDB_AGING			2
#define TSN_SHADOW_SWFG					3
#define TSN_SHADOW_PLCTRL				4
#define TSN_SHADOW_PLMASKS				5
#define TSN_SHADOW_PER_XSEL				6
#define TSN_SHADOW_PER_PORTCONFIG 		7
#define TSN_SHADOW_PER_PRIOCONFIG 		8
#define TSN_SHADOW_PER_PORTPRIOCONFIG 	9
#define TSN_SHADOW_PER_VLANIDCONFIG		10
#define TSN_SHADOW_PER_VLANMEMBER		11
#define TSN_SHADOW_LAST 				12

#define TSN_FPGA_PER_XSEL_PRIO_MASK						0x0007
#define TSN_FPGA_PER_XSEL_PRIO_MAX_VALUE				7
#define TSN_FPGA_PER_XSEL_PORT_MAX_VALUE				3

#define KAIROS_MAX_PORTS		2
#define N_EXT_TS				1	

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
	u8				*tx_buffer;
	u8				*rx_buffer;
	u32				speed_hz;

	/* PTP management thread */
	struct task_struct* ptp_thread;

	struct dsa_switch* ds;
	int sw_addr;

	u16 shadow_regs[TSN_SHADOW_LAST];
	u16 shadow_max_sdu[TSN_FPGA_PER_XSEL_PORT_MAX_VALUE+1][TSN_FPGA_PER_XSEL_PRIO_MAX_VALUE+1];

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


int kairos_reg_read(struct kairos_data* kairos, u8 module, u8 reg, u16* data);
int kairos_reg_write(struct kairos_data* kairos, u8 module, u8 reg, u16 data);

int kairos_check_enabled(struct kairos_data* kairos);

void kairos_build_date(struct kairos_data* kairos);
void kairos_fw_release(struct kairos_data* kairos);
u16 kairos_status(struct kairos_data* kairos);

int kairos_ptp_init(struct kairos_data* kairos);
int kairos_switch_init(struct kairos_data* kairos);


/*
 * Counter
 */
#define KAIROS_RFLT			0
#define KAIROS_ROC			1
#define KAIROS_RVTG			2
#define KAIROS_RBD			3
#define KAIROS_ROLD			4
#define KAIROS_RDA_UC		5
#define KAIROS_RDA_MC		6
#define KAIROS_RDA_BC		7
#define KAIROS_RSZ_L64		8
#define KAIROS_RSZ_64 		9
#define KAIROS_RSZ_L127		10
#define KAIROS_RSZ_L255		11
#define KAIROS_RSZ_L511 	12
#define KAIROS_RSZ_L1023	13
#define KAIROS_RSZ_L1518	14
#define KAIROS_RSZ_G1518	15
#define KAIROS_TTDQ0		16
#define KAIROS_TTDQ1		17
#define KAIROS_TTDQ2		18
#define KAIROS_TTDQ3		19
#define KAIROS_TTDQ4		20
#define KAIROS_TTDQ5		21
#define KAIROS_TTDQ6		22
#define KAIROS_TTDQ7		23
#define KAIROS_RPR0			24
#define KAIROS_RPR1			25
#define KAIROS_RPR2			26
#define KAIROS_RPR3			27
#define KAIROS_RPR4			28
#define KAIROS_RPR5			29
#define KAIROS_RPR6			30
#define KAIROS_RPR7			31
#define KAIROS_TOC			32
#define KAIROS_TVTG			33
#define KAIROS_TBD 			34
#define KAIROS_TDA_UC		35
#define KAIROS_TDA_MC		36
#define KAIROS_TDA_BC		37
#define KAIROS_TSZ_L64		38
#define KAIROS_TSZ_64 		39
#define KAIROS_TSZ_L127		40
#define KAIROS_TSZ_L255		41
#define KAIROS_TSZ_L511 	42
#define KAIROS_TSZ_L1023	43
#define KAIROS_TSZ_L1518	44
#define KAIROS_TSZ_G1518	45
#define KAIROS_TPO0			46
#define KAIROS_TPO1			47
#define KAIROS_TPO2   		48
#define KAIROS_TPO3			49
#define KAIROS_TPO4			50
#define KAIROS_TPO5			51
#define KAIROS_TPO6			52
#define KAIROS_TPO7			53
#define KAIROS_TPR0			54
#define KAIROS_TPR1			55
#define KAIROS_TPR2			56
#define KAIROS_TPR3			57
#define KAIROS_TPR4			58
#define KAIROS_TPR5			59
#define KAIROS_TPR6			60
#define KAIROS_TPR7			61
#define KAIROS_NUM_COUNTERS 62

int kairos_reg_counter(struct kairos_data* kairos, int port, int counter, u32* value);

#endif

