#ifndef _REGISTERS_H_
#define _REGISTERS_H_


#define KAIROS_MODULE_GENERAL	0x01
#define KAIROS_MODULE_TSN		0x02
#define KAIROS_MODULE_PTP		0x03


#define KAIROS_REG_GR_MODID    0x00
#define KAIROS_REG_GR_RELLB    0x01
#define KAIROS_REG_GR_RELHB    0x02
#define KAIROS_REG_GR_BDTLB    0x03
#define KAIROS_REG_GR_BDTHB    0x04
#define KAIROS_REG_GR_CTRL     0x05
#define KAIROS_REG_GR_PORTEN   0x06
#define KAIROS_REG_GR_LINK     0x12
#define KAIROS_REG_GR_GPIO_OUT 0x18
#define KAIROS_REG_GR_GPIO_IN  0x19
#define KAIROS_REG_GR_GPIO_TRIS 0x1A
#define KAIROS_REG_GR_GPIOA_OUT 0x1B
#define KAIROS_REG_GR_GPIOA_IN  0x1C
#define KAIROS_REG_GR_GPIOA_TRIS 0x1D
#define KAIROS_REG_GR_GPIOB_OUT 0x1F
#define KAIROS_REG_GR_GPIOB_IN  0x20
#define KAIROS_REG_GR_GPIOB_TRIS 0x21
#define KAIROS_REG_GR_SMI_CTRL 0x23
#define KAIROS_REG_GR_SMI_CMD  0x24
#define KAIROS_REG_GR_SMI_DAT  0x25

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


#define KAIROS_REG_HR_MODID_C      0x00
#define KAIROS_REG_HR_REL_L_C      0x01
#define KAIROS_REG_HR_REL_H_C      0x02
#define KAIROS_REG_HR_BLD_L_C      0x03
#define KAIROS_REG_HR_BLD_H_C      0x04
#define KAIROS_REG_TR_CTRL         0x05
#define KAIROS_REG_TR_GCLDAT       0xC4
#define KAIROS_REG_TR_GCLTIL       0xC5
#define KAIROS_REG_TR_GCLTIH       0xC6
#define KAIROS_REG_TR_GCLCMD       0xC7
#define KAIROS_REG_HR_CSEL         0x8D
#define KAIROS_REG_HR_CRDL         0x8E
#define KAIROS_REG_HR_CRDH         0x8F
#define KAIROS_REG_HR_SWTRC_CFG    0x90
#define KAIROS_REG_HR_SWTRC0       0x91
#define KAIROS_REG_HR_SWTRC1       0x92
#define KAIROS_REG_HR_PFREE        0x93
#define KAIROS_REG_HR_MFREE        0x94
#define KAIROS_REG_HR_GATEMON0     0x95
#define KAIROS_REG_HR_GATEMON1     0x96
#define KAIROS_REG_HR_FDBAGE       0x97
#define KAIROS_REG_HR_FDBMAX       0x98
#define KAIROS_REG_HR_FDBRDL       0x99
#define KAIROS_REG_HR_FDBRDM       0x9A
#define KAIROS_REG_HR_FDBRDH       0x9B
#define KAIROS_REG_HR_FDBMDRD      0x9C
#define KAIROS_REG_HR_FDBWRL       0x9D
#define KAIROS_REG_HR_FDBWRM       0x9E
#define KAIROS_REG_HR_FDBWRH       0x9F
#define KAIROS_REG_HR_FDBWRM0      0xA0
#define KAIROS_REG_HR_FDBWRCMD     0xA2
#define KAIROS_REG_HR_SWCFG        0xA3
#define KAIROS_REG_HR_PSEL         0xA6
#define KAIROS_REG_HR_PTCFG        0xA7
#define KAIROS_REG_HR_PRCFG        0xA8
#define KAIROS_REG_HR_PTPRCFG      0xA9
#define KAIROS_REG_HR_VIDCFG       0xAA
#define KAIROS_REG_HR_VIDMBRCFG    0xAB
#define KAIROS_REG_HR_FEABITS0     0xAC
#define KAIROS_REG_HR_LIMITS0      0xAF
#define KAIROS_REG_TR_QTRACK       0xB1
#define KAIROS_REG_TR_TGDVER       0xB3
#define KAIROS_REG_TR_TGDCTRL      0xB5
#define KAIROS_REG_TR_TGDSTAT0     0xB6
#define KAIROS_REG_TR_TGDSTAT1     0xB7
#define KAIROS_REG_TR_ESTWRL       0xB8
#define KAIROS_REG_TR_ESTWRH       0xB9
#define KAIROS_REG_TR_ESTCMD       0xBA
#define KAIROS_REG_TR_EETWRL       0xBB
#define KAIROS_REG_TR_EETWRH       0xBC
#define KAIROS_REG_TR_EETCMD       0xBD
#define KAIROS_REG_TR_CTWRL        0xBE
#define KAIROS_REG_TR_CTWRH        0xBF
#define KAIROS_REG_TR_CTSUBNS      0xC0
#define KAIROS_REG_TR_LCNSL        0xC1
#define KAIROS_REG_TR_LCNSH        0xC2
#define KAIROS_REG_TR_LCS          0xC3
#define KAIROS_REG_TR_GCLDAT       0xC4
#define KAIROS_REG_TR_GCLTIL       0xC5
#define KAIROS_REG_TR_GCLTIH       0xC6
#define KAIROS_REG_TR_GCLCMD       0xC7
#define KAIROS_REG_TR_PLSTAT       0xCD
#define KAIROS_REG_TR_PLRD0        0xCE
#define KAIROS_REG_TR_PLRD1        0xCF
#define KAIROS_REG_TR_PLRD2        0xD0
#define KAIROS_REG_TR_PLCTRL       0xD1
#define KAIROS_REG_TR_PLMASKS      0xD2
#define KAIROS_REG_TR_ITSCTL       0xD5

/* Register: Module ID */
#define TSN_TSN_FUNCTYPE						0x0013
#define TSN_TSN_ARCH_DEMO						0x0000
#define TSN_TSN_CAP_QBV							0x0010
#define TSN_TSN_CAP_QBU							0x0020
#define TSN_TSN_CAP_QCH							0x0040
#define TSN_TSN_CAP_CB							0x0080

/* Register: Control Status */
#define TSN_CTRL_GIGA_MASK                      0x0010
#define TSN_CTRL_READY_MASK						0x4000
#define TSN_CTRL_TRANSIT_MASK					0x2000
#define TSN_CTRL_SPEED_MISM_MASK				0x1000

/* switching configuration (reg A3) */
#define	TSN_SWITCH_CFG_LAS_MODE 				0x3000
#define	TSN_SWITCH_CFG_LAS_MODE_OFFSET			12
#define	TSN_SWITCH_CFG_FDB_AGE_EN				0x0020
#define	TSN_SWITCH_CFG_FDB_LRN_EN				0x0010
#define	TSN_SWITCH_CFG_ALWAYS_OBT				0x0200
#define	TSN_SWITCH_CFG_VLAN_UNAWARE_EN			0x0400
#define	TSN_SWITCH_CFG_CT_EN					0x0800
#define	TSN_SWITCH_CFG_GATEMON_STATE_EN			0x8000

/* Port selection reg 0xA6 */
#define TSN_PER_XSEL_PORT_MASK                  0x0030
#define TSN_PER_XSEL_PORT_OFFSET                4
#define TSN_PER_XSEL_PRIO_MASK					0x0007
#define TSN_PER_XSEL_PRIO_MAX_VALUE				7
#define TSN_PER_XSEL_PORT_MAX_VALUE				3

/* Per port priority (reg 0xA9) */
#define	TSN_PER_PORTPRIO_MAXSDU_MASK			0x07FF
#define	TSN_PER_PORTPRIO_QTRACK_MASK			0x8000
#define	TSN_MAXSDU_DEFVAL						(1536)

/* FDB meta data */
#define TSN_FDB_META_REPRIO						0x8000
#define TSN_FDB_META_REPRIO_MASK				0x7000
#define TSN_FDB_META_REPRIO_OFFSET				12
#define	TSN_FDB_META_STATIC						0x0800
#define TSN_FDB_META_PASSBLOCKED				0x0200
#define TSN_FDB_META_OBT						0x0100
#define TSN_FDB_META_NO_FLAGS					0x0000
#define TSN_FDB_META_PORT_MASK					0x000F
#define TSN_FDB_META_PORT_CASCADING				0x0001
#define TSN_FDB_META_PORT_TUNNEL				0x0002
#define TSN_FDB_META_PORT_PORT1					0x0004
#define TSN_FDB_META_PORT_PORT2					0x0008

/* FDB command */
#define TSN_FDB_CMD_DEL							0x0200
#define TSN_FDB_CMD_IDX_MASK					0x01FF

#define TSN_FDB_PMASK_CASCADING					0x0001
#define TSN_FDB_PMASK_TUNNEL					0x0002
#define TSN_FDB_PMASK_PORT1						0x0004
#define TSN_FDB_PMASK_PORT2						0x0008
#define TSN_FDB_PMASK_ALL						0x000F
#define TSN_FDB_PMASK_MASK						0x000F
#define TSN_FDB_PMASK_NO_PORTS					0x0000

/* TGD Stat 0 */
#define TSN_SCHEDULE_PNTR						0xFF00
#define TSN_OPER_GATE_STATES					0x00FF

/* TGD Stat 1 */
#define TSN_SCHED_B                             0x0001
#define TSN_ESC_ARMED                           0x0002
#define TSN_EEC_ARMED                           0x0004
#define TSN_CYCLE_ARMED                         0x0010

/* Features bits 0 (0xAC) */
#define TSN_FEATUREBITS_MULT				 	32
#define TSN_FEATUREBITS_FDBBINS_MASK			0x00F0

/* Switch trace configuration (reg 0x90) */
#define TSN_SWITCH_TRACE_FRONTPORT2_BIT         0x0008
#define TSN_SWITCH_TRACE_FRONTPORT1_BIT         0x0004
#define TSN_SWITCH_TRACE_TUNNEL_BIT             0x0002
#define TSN_SWITCH_TRACE_CASCADING_BIT          0x0001

/* switch trace bits (reg 0x91) */
#define TSN_SWITCH_TRACE_BIT_OVERSIZE			0x0001
#define TSN_SWITCH_TRACE_BIT_UNDERSIZE          0x0002
#define TSN_SWITCH_TRACE_BIT_L1L2BAD			0x0004
#define TSN_SWITCH_TRACE_BIT_NOPTR              0x0008
#define TSN_SWITCH_TRACE_BIT_QFULL  			0x0010
#define TSN_SWITCH_TRACE_BIT_FIFOFULL           0x0020
#define TSN_SWITCH_TRACE_BIT_MAXSDU_EXC			0x0040
#define TSN_SWITCH_TRACE_BIT_INGRFLT            0x0080
#define TSN_SWITCH_TRACE_BIT_EGRFLT				0x0100
#define TSN_SWITCH_TRACE_BIT_VTAGFLT            0x0200
#define TSN_SWITCH_TRACE_BIT_BLOCKED			0x0400
#define TSN_SWITCH_TRACE_BIT_OBT				0x0800
#define TSN_SWITCH_TRACE_BIT_DSTDOWN			0x1000
#define TSN_SWITCH_TRACE_BIT_MATCH				0x2000
#define TSN_SWITCH_TRACE_BIT_MEMLIMIT			0x4000
#define TSN_SWITCH_TRACE_BIT_SRCDOWN            0x8000

/* TSN Control register (0xB5) */
#define TSN_ADMIN_GATE_STATE					0xFF00
#define TSN_CYCLE_SNAP							0x0010
#define TSN_GATE_EN                             0x0001


#define KAIROS_REG_PTP_SETTINGS 0x09
#define KAIROS_REG_PTP_STATUS1 	0x0B
#define KAIROS_REG_PTP_STATUS	0x0C
#define KAIROS_REG_PTP_CLK_RD	0x0D
#define KAIROS_REG_PTP_CLK_WR	0x0E
#define KAIROS_REG_PTP_OFFSET	0x0F
#define KAIROS_REG_PTP_DRIFT	0x10
#define KAIROS_REG_PTP_FREE_DATA 0x12
#define KAIROS_REG_PTP_SYNC_DATA 0x16

#define REG_PR_SS_FREE_DATA     0x12
#define REG_PR_SS_SYNT_DATA     0x14
#define REG_PR_SS_SYNC_DATA     0x16
#define REG_PR_SS_DRAC_DATA     0x18
#define REG_PR_TS_RX_P1_DATA    0x1E
#define REG_PR_TS_TX_P1_DATA    0x20
#define REG_PR_TS_RX_P2_DATA    0x26
#define REG_PR_TS_TX_P2_DATA    0x28

#endif
