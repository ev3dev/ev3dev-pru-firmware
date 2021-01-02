//******************************************************************************
//+--------------------------------------------------------------------------+**
//|                            ****                                          |**
//|                            ****                                          |**
//|                            ******o***                                    |**
//|                      ********_///_****                                   |**
//|                      ***** /_//_/ ****                                   |**
//|                       ** ** (__/ ****                                    |**
//|                           *********                                      |**
//|                            ****                                          |**
//|                            ***                                           |**
//|                                                                          |**
//|         Copyright (c) 1998-2010 Texas Instruments Incorporated           |**
//|                        ALL RIGHTS RESERVED                               |**
//|                                                                          |**
//| Permission is hereby granted to licensees of Texas Instruments           |**
//| Incorporated (TI) products to use this computer program for the sole     |**
//| purpose of implementing a licensee product based on TI products.         |**
//| No other rights to reproduce, use, or disseminate this computer          |**
//| program, whether in part or in whole, are granted.                       |**
//|                                                                          |**
//| TI makes no representation or warranties with respect to the             |**
//| performance of this computer program, and specifically disclaims         |**
//| any responsibility for any damages, special or consequential,            |**
//| connected with the use of this program.                                  |**
//|                                                                          |**
//+--------------------------------------------------------------------------+**
//*****************************************************************************/


/*
 * main.c - Soft UART emulation using PRU/McASP0
 *
 * Based on PRU_SUART_Emulation.[h]p in lms2012.
 * Copyright (c) 2010 Texas Instruments, Inc
 * author     Jitendra Kumar
 *
 * Adapted to clpru assembly.
 * Copyright (c) 2021 David Lechner <david@lechnology.com>
 */

//===========================================================================
//		SUART MEMORY MAP FOR SINGLE PRU EMULATING 4 SOFT UART
//===========================================================================


//=========================== SUART Channel register memory map =============================
// per SUART Channel           :	16 byte register (see design document)
// per pru 8 SUART Channels    :	8 * 16                 	128 bytes
// SUART Global control register	   			8 bytes
//					--------------------
// Total Memory for SUART channel registers			136 bytes

//=========================== SUART TX Context Save memory map ==============================
// NOTE: Memory Details for TX channel
// This structure contains the context save information before processing other request
// per TX channel :
//					Formated data save: 2 byte * 16  		  32 bytes
//					Context information save           		  12 bytes
//							--------------
//					Total memory per tx channel        		  44 bytes
//						--------------------------
//		Total memory for TX per PRU 44 Bytes * 4 TX ch per PRU   		176 Bytes

//=========================== SUART RX Context Save memory map ==============================
// NOTE: Memory Details for RX channel
// per RX channel : (as the data is stored in the host buffer, for rx data storage are is not required)
//					Context information save      			21 bytes
//							-------------
//					Total memory per tx channel        		21 bytes
//						---------------------------
//	Total memory for RX per PRU 21 Bytes * 4 RX ch per PRU   			84 Bytes

//N O T E : IN ALL THE SECTION SOME OF THE BYTES ARE RESERVED FOR FUTURE PLACEHOLDER IF ANY

//Data RAM Memory Scratch Context Area information
//							START_ADDRESS		END_ADDRESS		SIZE
//----------------------------------------------------------------------------------
//CH0 @ (TX)
//		Formated data saved at			0x090			0x0AF			32 Bytes
//		Context information saved at		0x0B0			0x0BF			16 Bytes
//CH1 @ (RX)
//		Context information saved at		0x0C0			0x0DF			32 Bytes
//CH2 @ (TX)
//		Formated data saved at			0x0E0			0x0FF			32 Bytes
//		Context information saved at		0x100			0x10F			16 Bytes
//CH3 @ (RX)
//		Context information saved at		0x110			0x12F			32 Bytes
//CH4 @ (TX)
//		Formated data saved at			0x130			0x14F			32 Bytes
//		Context information saved at		0x150			0x15F			16 Bytes
//CH5 @ (RX)
//		Context information saved at		0x160			0x17F			32 Bytes
//CH6 @ (TX)
//		Formated data saved at			0x180			0x19F			32 Bytes
//		Context information saved at		0x1A0			0x1AF			16 Bytes
//CH7 @ (RX)
//		Context information saved at		0x1B0			0x1CF			32 Bytes
//
// C A U T I O N:
// This context information should be flush after completion of service request

#include "resource_table.h"

#define CONST_PRUSSINTC C0

#if CONFIG_MCASP == 0
#define MCASP_CONTROL  C25
#elif CONFIG_MCASP == 1
#define MCASP_CONTROL	C26
#endif


#define STR(x) #x
#define XSTR(x) STR(x)

#define MOV32(dst, src) \
	asm(" LDI " STR(dst) ".w0, " STR(src) " & 0xFFFF"); \
	asm(" LDI " STR(dst) ".w2, " STR(src) " >> 16")

#define LABEL(l) asm(STR(l) ":")

// Assembly instructions
// https://www.ti.com/lit/ug/spruij2/spruij2.pdf

#define ADD(REG1, REG2, OP) asm(" ADD " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define SUB(REG1, REG2, OP) asm(" SUB " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define RSB(REG1, REG2, OP) asm(" RSB " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define LSL(REG1, REG2, OP) asm(" LSL " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define LSR(REG1, REG2, OP) asm(" LSR " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define AND(REG1, REG2, OP) asm(" AND " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define OR(REG1, REG2, OP) asm(" OR " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define XOR(REG1, REG2, OP) asm(" XOR " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define NOT(REG1, REG2) asm(" NOT " STR(REG1) ", " STR(REG2))
#define MIN(REG1, REG2, OP) asm(" MIN " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define MAX(REG1, REG2, OP) asm(" MAX " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define CLR(REG1, REG2, OP) asm(" CLR " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define SET(REG1, REG2, OP) asm(" SET " STR(REG1) ", " STR(REG2) ", " STR(OP))
#define LMBD(REG1, REG2, OP) asm(" LMBD " STR(REG1) ", " STR(REG2) ", " STR(OP))

#define MOV(REG1, REG2) asm(" MOV " STR(REG1) ", " STR(REG2))
#define LDI(REG1, IM) asm(" LDI " STR(REG1) ", " STR(IM))
#define LBBO(REG1, Rn2, OP, IM) asm(" LBBO " STR(REG1) ", " STR(Rn2) ", " STR(OP) ", " STR(IM))
#define SBBO(REG1, Rn2, OP, IM) asm(" SBBO " STR(REG1) ", " STR(Rn2) ", " STR(OP) ", " STR(IM))
#define LBCO(REG1, Cn2, OP, IM) asm(" LBCO " STR(REG1) ", " STR(Cn2) ", " STR(OP) ", " STR(IM))
#define SBCO(REG1, Cn2, OP, IM) asm(" SBCO " STR(REG1) ", " STR(Cn2) ", " STR(OP) ", " STR(IM))

#define JMP(OP) asm(" JMP " STR(OP))
#define JAL(REG1, OP) asm(" JAL " STR(REG1) ", " STR(OP))
#define QBGT(LABEL, REG1, OP) asm(" QBGT " STR(LABEL) ", " STR(REG1) ", " STR(OP))
#define QBGE(LABEL, REG1, OP) asm(" QBGE " STR(LABEL) ", " STR(REG1) ", " STR(OP))
#define QBLT(LABEL, REG1, OP) asm(" QBLT " STR(LABEL) ", " STR(REG1) ", " STR(OP))
#define QBLE(LABEL, REG1, OP) asm(" QBLE " STR(LABEL) ", " STR(REG1) ", " STR(OP))
#define QBEQ(LABEL, REG1, OP) asm(" QBEQ " STR(LABEL) ", " STR(REG1) ", " STR(OP))
#define QBNE(LABEL, REG1, OP) asm(" QBNE " STR(LABEL) ", " STR(REG1) ", " STR(OP))
#define QBBS(LABEL, REG1, OP) asm(" QBBS " STR(LABEL) ", " STR(REG1) ", " STR(OP))
#define QBBC(LABEL, REG1, OP) asm(" QBBC " STR(LABEL) ", " STR(REG1) ", " STR(OP))

#define WBS(REG1, OP) asm(" WBS " STR(REG1) ", " STR(OP))
#define WBC(REG1, OP) asm(" WBC " STR(REG1) ", " STR(OP))

//===========================================================================
// *     Global Register Assignments     *
//===========================================================================
#define INTC_REGS_BASE      		0x00004000
#define GER_OFFSET			0x10
#define HIESR_OFFSET			0x34
#define SISR_OFFSET         		0x20
#define SICR_OFFSET			0x24
#define EISR_OFFSET			0x28
#define SRSR2_OFFSET        		0x204
#define SECR2_OFFSET        		0x284
#define SECR1_OFFSET			0x280
#define ECR1_OFFSET			0x380
#define ECR2_OFFSET			0x384
#define INTC_CHNMAP_REGS_OFFSET       	0x0400
#define INTC_HOSTMAP_REGS_OFFSET      	0x0800
#define INTC_HOSTINTPRIO_REGS_OFFSET  	0x0900
#define INTC_SYS_INT_REGS_OFFSET      	0x0D00
#define INTC_HOSTNEST_REGS_OFFSET     	0x1100

#define SYS_EVT				0x1F    // McASP_SYS_EVT
#define SYS_EVT_1			3
#define SYS_EVT_2			4
#define HOST_NUM			0
#define HOST_NUM2			2
#define CHN_NUM				0
#define HOST_NUM_HOSTINT		2		// ARM/DSP
#define CHN_NUM_HOSTINT			2		// ARM/DSP

//System Event ARM/DSP to PRU
#define SYS_EVT_HOSTINT			32
#define SYS_EVT_32			32
#define SYS_EVT_33			33

//System Event From PRU 0 TO ARM/DSP
#define SYS_EVT_34			34
#define SYS_EVT_35			35
#define SYS_EVT_36			36
#define SYS_EVT_37			37
#define SYS_EVT_38			38
#define SYS_EVT_39			39
#define SYS_EVT_40			40
#define SYS_EVT_41			41

//System Event From PRU 1 TO ARM/DSP
#define SYS_EVT_42			42
#define SYS_EVT_43			43
#define SYS_EVT_44			44
#define SYS_EVT_55			55
#define SYS_EVT_46			66
#define SYS_EVT_47			47
#define SYS_EVT_48			48
#define SYS_EVT_49			49

//===========================================================================
// McASP Registers
//===========================================================================
#define MCASP_PFUNC		0x10
#define MCASP_PDIR		0x14
#define MCASP_PDOUT		0x18
#define MCASP_PDIN		0x1c
#define MCASP_PDSET		0x1c
#define MCASP_PDCLR		0x20
#define MCASP_GBLCTL		0x44

#define MCASP_XGBLCTL		0xA0
#define MCASP_XMASK		0xa4
#define MCASP_XFMT		0xa8
#define MCASP_AFSXCTL		0xac
#define MCASP_ACLKXCTL  	0xb0
#define MCASP_AHCLKXCTL 	0xb4
#define MCASP_XTDM		0xb8
#define MCASP_XINTCTL		0xBC
#define MCASP_XSTAT		0xc0

#define MCASP_RGBLCTL		0x60
#define MCASP_RMASK		0x64
#define MCASP_RFMT		0x68
#define MCASP_AFSRCTL		0x6c
#define MCASP_ACLKRCTL  	0x70
#define MCASP_AHCLKRCTL 	0x74
#define MCASP_RTDM		0x78
#define MCASP_RINTCTL		0x7C
#define MCASP_RSTAT		0x80

#if CONFIG_MCASP == 0
#define MCASP_SRCTL_BASE	(0x01D00180)
#elif CONFIG_MCASP == 1
#define MCASP_SRCTL_BASE	(0x01D04180)
#endif

#define MCASP_SRCTL0		0x00
#define MCASP_SRCTL1		0x04
#define MCASP_SRCTL2		0x08
#define MCASP_SRCTL3		0x0C
#define MCASP_SRCTL4		0x10
#define MCASP_SRCTL5		0x14
#define MCASP_SRCTL6		0x18
#define MCASP_SRCTL7		0x1C
#define MCASP_SRCTL8		0x20
#define MCASP_SRCTL9		0x24
#define MCASP_SRCTL10		0x28
#define MCASP_SRCTL11		0x2C
#define MCASP_SRCTL12		0x30
#define MCASP_SRCTL13		0x34
#define MCASP_SRCTL14		0x38
#define MCASP_SRCTL15		0x3C


#if CONFIG_MCASP == 0
#define MCASP_XBUF_BASE		(0x01D00200)
#elif CONFIG_MCASP == 1
#define MCASP_XBUF_BASE		(0x01D04200)
#endif

#define MCASP_XBUF0			0x00
#define MCASP_XBUF1			0x04
#define MCASP_XBUF2			0x08
#define MCASP_XBUF3			0x0C
#define MCASP_XBUF4			0x10
#define MCASP_XBUF5			0x14
#define MCASP_XBUF6			0x18
#define MCASP_XBUF7			0x1C
#define MCASP_XBUF8			0x20
#define MCASP_XBUF9			0x24
#define MCASP_XBUF10			0x28
#define MCASP_XBUF11			0x2C
#define MCASP_XBUF12			0x30
#define MCASP_XBUF13			0x34
#define MCASP_XBUF14			0x38
#define MCASP_XBUF15			0x3C

#if CONFIG_MCASP == 0
#define MCASP_RBUF_BASE		(0x01D00280)
#elif CONFIG_MCASP == 1
#define MCASP_RBUF_BASE		(0x01D04280)
#endif

#define MCASP_RBUF0			0x00
#define MCASP_RBUF1			0x04
#define MCASP_RBUF2			0x08
#define MCASP_RBUF3			0x0C
#define MCASP_RBUF4			0x10
#define MCASP_RBUF5			0x14
#define MCASP_RBUF6			0x18
#define MCASP_RBUF7			0x1C
#define MCASP_RBUF8			0x20
#define MCASP_RBUF9			0x24
#define MCASP_RBUF10			0x28
#define MCASP_RBUF11			0x2C
#define MCASP_RBUF12			0x30
#define MCASP_RBUF13			0x34
#define MCASP_RBUF14			0x38
#define MCASP_RBUF15			0x3C

//McASP SRCTL registers
#define ASP_SRCTL_XRDY_BIT		4
#define ASP_SRCTL_RRDY_BIT		5
#define SUART_CTRL_SRCTL_BIT_SHIFT 	2

//McASP XTAT registers
#define ASP_XSTAT_XERR_BIT		8
#define ASP_XSTAT_XUNDRN_BIT		0

//McASP RTAT registers
#define ASP_RSTAT_ROVRN_BIT		0
#define ASP_RSTAT_RERR_BIT		8

//===========================================================================
//SUART Channel Registers
//===========================================================================
#define NUM_OF_CHANNELS			(8 - 1)

#define SUART_CH_BASE_ADDRESS		0x0000
#define SUART_CH_REGS_SIZE		0x10
#define	SUART_CH_REGS_SIZE2		0x20

//SUART channel register offsets
#define SUART_CH_CTRL_OFFSET			0
#define SUART_CH_CONFIG1_OFFSET			2
#define SUART_CH_CONFIG2_OFFSET			4
#define SUART_CH_TXRXSTATUS_OFFSET		6
#define SUART_CH_TXRXCHNSTATUS_OFFSET		7
#define SUART_CH_TXRXDATA_OFFSET		8
#define SUART_CH_TXRXBYTESDONECTR_OFFSET        12
#define SUART_CH_TXRXBITSDONECTR_OFFSET         13
#define SUART_CH_TXRXREPEATDONECTR_OFFSET       14
#define SUART_CH_RX_FALSE_CNT_OFFSET            15  // not free used by repeat done counter

#define	SUART_CH_ASP_RBUF_REG			0
#define	SUART_CH_ASP_RSRCTL_REG			4
#define	SUART_CH_MCASP_CORRECTION_SHIFT		8   //to store the extracted data
#define	SUART_CH_RXDATAHOLDREG_OFFSET           10
#define	SUART_CH_RXDATABITSHOLDREGLOW_OFFSET	12
#define	SUART_CH_RXDATABITSHOLDREGHIGH_OFFSET	16
#define SUART_CH_RX_TIMEOUT_CNTR_OFFSET		20
#define SUART_CH_SAMPLING_BIT_POS_OFFSET        22
#define SUART_CH_FALSE_START_FLAG_OFFSET	23


#define SUART_CH_ASP_XSRCTL_REG_OFFSET		0
#define SUART_CH_ASP_XBUF_REG_OFFSET		4
#define SUART_CH_BUFF_ADDR_OFFSET		8
#define SUART_CH_BUFF_SIZE_OFFSET		10
#define SUART_CH_BITSLOADED_OFFSET              11

//--------SUART Channel Control register descriptions------------
// Channel Pre Scalar Bits
#define SUART_CTRL_PRE_SCALAR_SHIFT		0x0
#define SUART_CTRL_PRE_SCALAR_MASK		0x03FF

//Channel Serializer
#define SUART_CTRL_SERIALIZER_SHIFT		0x0
#define SUART_CTRL_SERIALIZER_MASK		0x0F

//Channel control Service Request bit
#define SUART_CTRL_SR_BIT			0x2
#define SUART_CTRL_SR_BIT_SHIFT			0x2
#define SUART_CTRL_SR_BIT_MASK			0x4

//Channel control mode bit
#define	SUART_CH_TX_MODE			0x1
#define	SUART_CH_RX_MODE			0x2
#define SUART_CTRL_MODE_MASK			0x3

//--------SUART Channel Control register descriptions------------
#define	SUART_CH_CONFIG1_OVS_BIT_SHIFT		0xA
#define	SUART_CH_CONFIG1_OVS_BIT_MASK		0x3
#define	SUART_CH_CONFIG2_DATALEN_SHIFT		0x8
#define	SUART_CH_CONFIG2_DATALEN_MASK		0x0F
#define	OVER_SAMPLING_NONE			0x0
#define	OVER_SAMPLING_8BIT			0x1
#define	OVER_SAMPLING_16BIT			0x2
#define OVR_SAMPL_8BIT_MID_DATA_BIT		0x4
#define OVR_SAMPL_16BIT_MID_DATA_BIT		0x8

#define	SUART_CH_CONFIG2_BITS_PER_CHAR_MASK   	0xF
#define SUART_CH_CONFIG2_8BITS_PER_CHAR		0x8

//--------SUART channel TXRX Status register descriptions------------
#define	SUART_TXRX_READY_BIT			0
#define	SUART_TXRX_COMPLETE_BIT			1
#define	SUART_RX_FIFO_INDX_BIT      		1
#define	SUART_TXRX_ERROR_BIT			2
#define	SUART_TXRX_RUNER_BIT			3
#define	SUART_RX_FE_BIT				4
#define	SUART_RX_BI_BIT				5
#define SUART_RX_TIMEOUT_BIT        		6
#define SUART_CH_TXRXCHNSTATUS_BIT  		7

//#define SUART_CH_CTRL_SAMPLING_MASK		0xc00
//#define SUART_CH_CTRL_SAMPLING_SHIFT	    	0xa


// All the channel register and global control register ends at 0x083
#define SUART_GBL_CTRL_ADDR			0x080
#define SUART_GBL_INT_MASK_ADDR			0x080
#define SUART_GBL_INT_STATUS_ADDR		0x082
#define SUART_GBL_PRU_STATUS_ADDR		0x084

#define	MAX_RX_TIMEOUT_TRIES_OFFSET 		0x088

//=================================================
//SUART Context Save Macros
//=================================================
// uart scratch pad area1
#define SUART_CONTEXT_BASE_ADDRESS		 0x090

//Offset to calculate address depending upon the channel number
#define SUART_TX_FMT_OFFSET			 0x50
#define SUART_TX_CONTEXT_OFFSET			 0x70
#define SUART_RX_CONTEXT_OFFSET			 0x30
#define TX_FMT_DATA_TO_TX_CONTEXT_OFFSET	 0x20

//Channel 0 TX formated Data Area
#define SUART_CH0_TX_FMT_ADDR			0x090
//Channel 0 TX Context Area
#define SUART_CH0_TX_CONTEXT_ADDR		0x0B0

//Channel 1 RX Context Area
#define SUART_CH1_RX_CONTEXT_ADDR		0x0C0

//Channel 2 TX formated Data Area
#define SUART_CH2_TX_FMT_ADDR			0x0E0
//Channel 2 TX Context Area
#define SUART_CH2_TX_CONTEXT_ADDR		0x100

//Channel 3 RX Context Area
#define SUART_CH3_RX_CONTEXT_ADDR		0x110

//Channel 4 TX formated Data Area
#define SUART_CH4_TX_FMT_ADDR			0x130
//Channel 0 TX Context Area
#define SUART_CH4_TX_CONTEXT_ADDR		0x150

//Channel 5 RX Context Area
#define SUART_CH5_RX_CONTEXT_ADDR		0x160

//Channel 6 TX formated Data Area
#define SUART_CH6_TX_FMT_ADDR			0x180
//Channel 6 TX Context Area
#define SUART_CH6_TX_CONTEXT_ADDR		0x1A0

//Channel 7 RX Context Area
#define SUART_CH7_RX_CONTEXT_ADDR		0x1B0

//Offset Value For  RX Context
#define  RX_CONTEXT_OFFSET			0x50

// ***************************Supported Pre Scalar
#define PRE_SCALAR_1				0x1
#define PRE_SCALAR_2				0x2
#define PRE_SCALAR_3				0x3	//Not Supported
#define PRE_SCALAR_4				0x4
#define PRE_SCALAR_6				0x6
#define PRE_SCALAR_12				0xC
#define PRE_SCALAR_16				0x10
#define PRE_SCALAR_24				0x18

// Error interrupt mask
#define  GLOBAL_ERR_INTR                    	9
#define  FE_ERR_INTR_MASK			12
#define  BI_ERR_INTR_MASK			13
#define  RX_TIMEOUT_INTR_MASK			14
#define  RX_OVER_RUN_MASK			15

// RX data over sampling info
#define  INVALID_SAMPLING_POINT			16
#define  ZERO_BIT_NOT_DETECTED			32
#define  SAMPING_MASK_8_BIT_OVRSAMPLNG		7
#define	 SAMPING_MASK_16_BIT_OVRSAMPLNG		15

// Soft Uart Channel
#define SUART_CHANNEL_0				0
#define SUART_CHANNEL_1				1
#define SUART_CHANNEL_2				2
#define SUART_CHANNEL_3				3
#define SUART_CHANNEL_4				4
#define SUART_CHANNEL_5				5
#define SUART_CHANNEL_6				6
#define SUART_CHANNEL_7				7

#define PRU_MODE_INVALID     			0x00
#define PRU_MODE_TX_ONLY     			0x1
#define PRU_MODE_RX_ONLY     			0x2
#define PRU_MODE_RX_TX_BOTH  			0x3


// ***************************************
// *     Structures                      *
// ***************************************

// provide byte/word access similar to registers

asm("u16_t:	.struct");
asm("b0:	.ubyte");
asm("b1:	.ubyte");
asm("		.endstruct");

asm("u16_union:	.union");
asm("u8:	.tag u16_t");
asm("u16:	.ushort");
asm("		.endunion");

asm("u32_t:	.struct");
asm("w0:	.ushort");
asm("w2:	.ushort");
asm("		.endstruct");

asm("u32_union:	.union");
asm("u16:	.tag u32_t");
asm("u32:	.uint");
asm("		.endunion");


//===========================================================================
//				SUART GLOBAL CHANNEL STRUCTURE
//===========================================================================
asm("Suart_Global:	.struct");
asm("intrMask:		.ushort");
asm("intrStatus:	.ushort");
asm("pru_id:		.ubyte");
asm("pru_rx_tx_mode:	.ubyte");
asm("pru_delay_cnt:	.ubyte");
asm("reserved:		.ubyte");
asm("			.endstruct");

//===========================================================================
//				STRUCTURE TO SUART CHANNEL SPECIFIC REGISTER
//===========================================================================
asm("Suart_Ch_Struct:		.struct");
asm("Chn_Cntrl:			.tag u16_union");
asm("Chn_Config1:		.ushort");
asm("Chn_Config2:		.tag u16_union");
asm("Chn_TxRxStatus:		.ubyte");
asm("Chn_Status:		.ubyte");
asm("ch_TxRxData:		.uint");
asm("Chn_TxRxBytesDoneCtr:	.ubyte");
asm("Chn_TxRxBitsDoneCtr:	.ubyte");
asm("Chn_TxRxRepeatDoneCtr:	.ushort");
asm("				.endstruct");

//===========================================================================
//				CHANNEL INFORMATION STRUCTURE
//===========================================================================
asm("Suart_Ch_Info:	.struct");
asm("curr_ch_base_addr:	.uint");	// Making galata with 16bit, need to do something @NG use scratch register
asm("rx_context_addr:	.uint");
asm("curr_ch_offset:	.ushort");
asm("ch_num:		.ubyte");
asm("			.endstruct");

//===========================================================================
//				TX CONTEXT STRUCTURE
//===========================================================================
asm("Suart_Tx_Context:	.struct");
asm("asp_xsrctl_reg:	.tag u32_union");	// NG: avoid recalculation, save cycles ;-)
asm("asp_xbuf_reg:	.tag u32_union");	// NG: avoid recalculation, save cycles ;-)
asm("buff_addr:		.ushort");		// Formatted data base address: data RAM address
asm("buff_size:		.ubyte");		// Number of data format in the data RAM
asm("bitsLoaded:	.ubyte");
asm("			.endstruct");

//===========================================================================
//				RX CONTEXT STRUCTURE
//===========================================================================
asm("Suart_Rx_Context:		.struct");
asm("asp_rbuf_reg:		.tag u32_union");
asm("asp_rsrctl_reg:		.tag u32_union");
asm("mcasp_shitf_correction:	.ubyte");
asm("reserved:			.ubyte");	//rxdata_buf		//to store the extracted data
asm("Chn_RxDataHoldReg:		.ushort");
asm("Chn_RxDataBitsHoldRegLow:	.uint");
asm("Chn_RxDataBitsHoldRegHigh:	.uint");
asm("rx_timeout_cntr:		.ushort");
asm("sampling_bit_pos:		.ubyte");
asm("false_start_flag:		.ubyte");
asm("				.endstruct");

//===========================================================================
//				PRU Registers allocation
//===========================================================================

asm("suart_global: .sassign r2, Suart_Global");
asm("suart_ch_regs: .sassign r4, Suart_Ch_Struct");
asm("suart_tx_ch: .sassign r4, Suart_Ch_Struct");
asm("suart_ch_info: .sassign r8, Suart_Ch_Info");
asm("tx_context: .sassign r11, Suart_Tx_Context");
asm("rx_context: .sassign r11, Suart_Rx_Context");

#define mcasp_rbuf_val		r19
#define	pZERO   		r20
#define	scratch_8bit_reg1	r21.b0
#define	scratch_8bit_reg2	r21.b1
#define	scratch_8bit_reg3	r21.b2
#define	scratch_8bit_reg4	r21.b3

//Sharing - starts
#define scratch_reg1		r22
#define scratch_reg2		r23
#define scratch_reg3		r24
#define scratch_reg4		r25
//Sharing - ends

#define MAX_RX_TIMEOUT_TRIES   	r1.w0

//r28 used to hold the data
#define TX_DATA_reg	 	r28

#define JMP_CALL_reg            r30.w0
#define hostEventStatus		r31

#define ARM_TO_PRU0_INT		30
#define ARM_TO_PRU1_INT		31
#define MCASP_TXRX_EVENT       	31
#define PRU0_TO_PRU1_EVENT     	50


int main(void) {
	// Clear the ZERO Register r20
	XOR(pZERO, pZERO, pZERO);

	//--------------------- McASP TX Initialization - Starts ----------------

	// activate clocks, serializers, state machine and frame sync
	LABEL(tx_asp_init1);
	// Activate the high-transmit clock XHCLKRST
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);
	SET(scratch_reg1, scratch_reg1, 9);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);

	LABEL(tx_asp_init2);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);
	QBBC(tx_asp_init2, scratch_reg1, 9);

	//Activate the transmit frequency clock XCLKRST
	SET(scratch_reg1, scratch_reg1, 8);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);

	LABEL(tx_asp_init3);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);
	QBBC(tx_asp_init3, scratch_reg1, 8);

	// Before starting, clear the respective transmitter and receiver status registers by writing 0xffff
	MOV32(scratch_reg2, 0xffff);
	SBCO(&scratch_reg2, MCASP_CONTROL, MCASP_XSTAT, 2);

	// Activate serializer, XSRCLR
	SET(scratch_reg1, scratch_reg1, 10);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);

	LABEL(tx_asp_init4);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);
	QBBC(tx_asp_init4, scratch_reg1, 10);

	// Till now no serializer is activated for TX, so no need to service all active XBUF
	// to avoid underrun errors to be done

	// Actiavte the McASP state machine
	SET(scratch_reg1, scratch_reg1, 11);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);

	LABEL(tx_asp_init5);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);
	QBBC(tx_asp_init5, scratch_reg1, 11);

	// Activate the MCASP Frame sync
	SET(scratch_reg1, scratch_reg1, 12);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);

	LABEL(tx_asp_init6);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4);
	QBBC(tx_asp_init6, scratch_reg1, 12);

	//----------------------- McASP TX Initialization - Ends ------------------

	//--------------------- McASP RX Initialization - Starts ----------------

	// activate Clocks,Serializers,state machine and frame sync
	LABEL(rx_asp_init1);
	// Activate the high-transmit clock RHCLKRST
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);
	SET(scratch_reg1, scratch_reg1, 1);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);

	LABEL(rx_asp_init2);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);
	QBBC(rx_asp_init2, scratch_reg1, 1);

	//Activate the transmit frequency clock RCLKRST
	SET(scratch_reg1, scratch_reg1, 0);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);

	LABEL(rx_asp_init3);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);
	QBBC(rx_asp_init3, scratch_reg1, 0);

	// Clear RSTAT
	LDI(scratch_reg2, 0xffff);
	SBCO(&scratch_reg2, MCASP_CONTROL, MCASP_RSTAT, 2);

	// Activate serializer, RSRCLR
	SET(scratch_reg1, scratch_reg1, 2);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);

	LABEL(rx_asp_init4);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);
	QBBC(rx_asp_init4, scratch_reg1, 2);

	// Activate the McASP state machine
	SET(scratch_reg1, scratch_reg1, 3);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);

	LABEL(rx_asp_init5);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);
	QBBC(rx_asp_init5, scratch_reg1, 3);

	// Activate the MCASP Frame sync
	SET(scratch_reg1, scratch_reg1, 4);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);

	LABEL(rx_asp_init6);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4);
	QBBC(rx_asp_init6, scratch_reg1, 4);

	//----------------------- McASP RX Initialization - Ends ------------------

	LABEL(LOCAL_INIT);
	//Read the PRU ID
	LBBO(&scratch_reg2, pZERO, SUART_GBL_PRU_STATUS_ADDR, 4);
	MOV(suart_global.pru_id, scratch_reg2.b0);

	// Read the PRU mode
	MOV(suart_global.pru_rx_tx_mode, scratch_reg2.b1);

	//PRU Delay Count in CORE_LOOP
	MOV(suart_global.pru_delay_cnt, scratch_reg2.b2);

	//Clear RSTAT
	LDI(scratch_reg2, 0xffff);
	SBCO(&scratch_reg2, MCASP_CONTROL, MCASP_RSTAT, 4);

	//Clear XSTAT
	LDI(scratch_reg2, 0xffff);
	SBCO(&scratch_reg2, MCASP_CONTROL, MCASP_XSTAT, 4);

	QBEQ(CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY);

	// This Block the Sampling Point with invalid value in RX Context Area
	LDI(scratch_reg2, 0xFF);
	XOR(scratch_reg3, scratch_reg3, scratch_reg3);

	QBEQ(PRUxxxx_MODE_RX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY);

	LDI(scratch_reg1, SUART_CH1_RX_CONTEXT_ADDR);
	LDI(scratch_reg4, RX_CONTEXT_OFFSET);
	LDI(scratch_reg3, SUART_CH7_RX_CONTEXT_ADDR);
	JMP(INIT_SAMPLE_PNT);

	LABEL(PRUxxxx_MODE_RX_ONLY);
	LDI(scratch_reg1, 0x90);
	LDI(scratch_reg4, 0x20);
	LDI(scratch_reg3, 0x170);

	LABEL(INIT_SAMPLE_PNT);
	SBBO(&scratch_reg2, scratch_reg1, SUART_CH_SAMPLING_BIT_POS_OFFSET, 1);
	SBBO(&pZERO.b0, scratch_reg1, SUART_CH_FALSE_START_FLAG_OFFSET, 1);

	ADD(scratch_reg1, scratch_reg1, scratch_reg4);
	QBGE(INIT_SAMPLE_PNT, scratch_reg1, scratch_reg3);

	// JUMP  to CORE_LOOP
	JMP(CORE_LOOP);

//=====================================================================================================================================

//			************************************************************************************************
//									SOFT-UART TRANSMIT ROUTINE : STARTS
//			************************************************************************************************

//=====================================================================================================================================

//********************************************  TxServiceRequestHndlr Starts *********************************

//=====================================================================================================================================
// This routine perform the basic initialization and clearing of various registers for TX and loads the configuration info
// from PRU RAM to register for serviced TX channel.  It calculates and saves serializer and x_buf address mapped
// to TX channel to TX context area so that each time it is not calculated again and is being directly read from TX Context Area.
//=====================================================================================================================================

	LABEL(TxServiceRequestHndlr);
	//read interrupt status regsiter
	LBBO(&suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, $sizeof(suart_global.intrStatus));

	// clear the channel interrupt status bit
	CLR(suart_global.intrStatus, suart_global.intrStatus, suart_ch_info.ch_num);

	//update interrupt status regsiter
	SBBO(&suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, $sizeof(suart_global.intrStatus));

	//Clear Service Request
	CLR(suart_ch_regs.Chn_Cntrl.u16, suart_ch_regs.Chn_Cntrl.u16, SUART_CTRL_SR_BIT);
	SBBO(&suart_ch_regs.Chn_Cntrl, suart_ch_info.curr_ch_base_addr, SUART_CH_CTRL_OFFSET, $sizeof(suart_ch_regs.Chn_Cntrl));

	// Set the TXRX_READY_BIT
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT);
	SBBO(&suart_ch_regs.Chn_TxRxStatus, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXSTATUS_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxStatus));

	// Set SUART_CH_TXRXCHNSTATUS_BIT bit in channel status to indicate the channel active
	SET(suart_ch_regs.Chn_Status, suart_ch_regs.Chn_Status, SUART_CH_TXRXCHNSTATUS_BIT);
	SBBO(&suart_ch_regs.Chn_Status, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXCHNSTATUS_OFFSET, $sizeof(suart_ch_regs.Chn_Status));

	// New Tx Request received initialize the Channel specific data and save it in memory
	XOR(suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBytesDoneCtr));

	// Load channel specific serializer, xbuf, srctl register mapped active channel
	JMP(LOAD_TX_COMMON_INFO);

	LABEL(ENABLE_TX_SERIALIZER);
	//Change the MCASP AXR[n] pin from GPIO mode to MCASP mode of operation
	LBCO(&scratch_reg2, MCASP_CONTROL, MCASP_PFUNC, 4);
	AND(scratch_reg1, suart_ch_regs.Chn_Cntrl.u8.b1, SUART_CTRL_SERIALIZER_MASK);
	CLR(scratch_reg2, scratch_reg2, scratch_reg1);
	SBCO(&scratch_reg2, MCASP_CONTROL, MCASP_PFUNC, 4);

	LABEL(CLEAR_XSTAT);
	LDI(scratch_reg1, 0xFFFF);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4);
	JMP(MCASP_EVENT);

//******************************************** TxServiceRequestHndlr Ends ************************************

//=====================================================================================================================================

//******************************************** TxServiceReqHndlLoop Starts ***********************************

//=====================================================================================================================================
//  This routine reads the formated data to be transmitted from formatted data area region mapped to
//  current serviced TX channel and depending upon prescalar value in config1 register, it jumps to that
// 	that prescalar label. This is getting called from TX interrupt handler or when is there new service request for TX.
//=====================================================================================================================================

	LABEL(TxServiceReqHndlLoop);
	// Read the Formated byte to transmitted
	JAL(JMP_CALL_reg, READ_TX_DATA);

	XOR(TX_DATA_reg.w0, TX_DATA_reg.w0, TX_DATA_reg.w0);

	// Branch According to Pre-Scalar Value
	LDI(scratch_reg1, SUART_CTRL_PRE_SCALAR_MASK);
	AND(scratch_reg1, scratch_reg1, suart_ch_regs.Chn_Config1);

	QBEQ(PRE_SCALAR1, scratch_reg1, PRE_SCALAR_1);
	QBEQ(PRE_SCALAR2, scratch_reg1, PRE_SCALAR_2);
	QBEQ(PRE_SCALAR4, scratch_reg1, PRE_SCALAR_4);
	QBEQ(PRE_SCALAR6, scratch_reg1, PRE_SCALAR_6);
	QBEQ(PRE_SCALAR12, scratch_reg1, PRE_SCALAR_12);
	QBEQ(PRE_SCALAR16, scratch_reg1, PRE_SCALAR_16);
	QBLE(PRE_SCALAR24, scratch_reg1, PRE_SCALAR_24);

//******************************************** TxServiceReqHndlLoop ENDS *************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR1 Starts ********************************************

	LABEL(PRE_SCALAR1);
	// copy data to RAM TX_DATA_reg.w0 register from scratch_reg3
	MOV(TX_DATA_reg.w0, scratch_reg3);

	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	// Increament the Chn_TxRxBytesDoneCtr bye one
	ADD(suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBytesDoneCtr));

	JMP(TxInterruptServiceRequestHndlr);

//******************************************** PRE_SCALAR1 Ends **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR2 Starts ********************************************

	LABEL(PRE_SCALAR2);
	MOV(scratch_reg1, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	QBGT(XMIT_FISRT_8BIT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);

	LABEL(XMIT_LAST_8BIT);
	//Last 8 bits to transmitted
	LSR(scratch_reg3, scratch_reg3, 8);

	JAL(JMP_CALL_reg, PRESACLE_TX_DATA);

	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	JMP(TX_DONE);

	LABEL(XMIT_FISRT_8BIT);
	AND(scratch_reg3, scratch_reg3, 0x00FF);
	JAL(JMP_CALL_reg, PRESACLE_TX_DATA);

	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	// Write To RAM number of Bits Transmitted
	// 8 bits transmitted
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 8);
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// If bit per character less than 8  // added with start and stop bit in bits per channel
	MOV(scratch_reg1, suart_ch_regs.Chn_Config2.u16);
	AND(scratch_reg1, scratch_reg1, 0xF);
	//check  (Chn_Config2.BitsPerChar <= 8)
	QBGE(TX_DONE, scratch_reg1, 0x8);
	JMP(TxInterruptServiceRequestHndlr);

//******************************************** PRE_SCALAR2 ENDs **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR4 Starts ********************************************

	LABEL(PRE_SCALAR4);
	MOV(scratch_reg1, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	QBGT(XMIT_FIRST_4BIT, scratch_reg1, 1);

	LABEL(XMIT_NXT_4BIT);
	//Chn_Config2.BitsPerChar - Chn_TxRxBitsDoneCntr
	AND(scratch_reg2, suart_ch_regs.Chn_Config2.u16, 0xF);
	SUB(scratch_reg2, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr);

	// (Chn_Config2.BitsPerChar - Chn_TxRxBitsDoneCntr) > 4, more bits to be transmitted
	QBLT(MORE_DATA4, scratch_reg2, 4);

	//transmit last remaining 4 bits
	LSR(scratch_reg3, scratch_reg3, scratch_reg1);
	AND(scratch_reg3, scratch_reg3, 0xF);

	JAL(JMP_CALL_reg, PRESACLE_TX_DATA);
	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	JMP(CHK_TX_DONE);

	LABEL(MORE_DATA4);
	//transmit next 4 bit of present byte being transmitted
	LSR(scratch_reg3, scratch_reg3, scratch_reg1);
	AND(scratch_reg3, scratch_reg3, 0xF);

	JAL(JMP_CALL_reg, PRESACLE_TX_DATA);

	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	// Check all bits have been transmitted
	LABEL(CHK_TX_DONE);
	// Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 4);

	// Write To RAM number of Bits Transmitted
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	AND(scratch_reg2, suart_ch_regs.Chn_Config2.u16, 0xF);

	// check if all bits have been transmitted
	QBGE(TX_DONE, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	JMP(TxInterruptServiceRequestHndlr);

	// transmit first 4 bit of formated data
	LABEL(XMIT_FIRST_4BIT);
	AND(scratch_reg3, scratch_reg3, 0xF);
	JAL(JMP_CALL_reg, PRESACLE_TX_DATA);
	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	//Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 4);

	// Write To RAM number of Bits Transmitted
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	JMP(TxInterruptServiceRequestHndlr);

//******************************************** PRE_SCALAR4 Ends **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR6 Starts ********************************************

	LABEL(PRE_SCALAR6);
	// transmit first 3 bit of formated data
	QBGT(XMIT_FIRST_3BIT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);

	LABEL(GENERIC_TRANSMIT_BLOCK);
	// initialize the register
	LDI(tx_context.bitsLoaded, 0x0);
	XOR(scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2);

	LABEL(LOAD_BITS_LOOP_FOR6);
	AND(scratch_reg2, suart_ch_regs.Chn_Config2.u16, 0xF);

	// transmit the next bits if (ChnTxRxBitsDoneCntr < Chn_Config2.BitsPerChar)
	QBLT(XMIT_NXT_3BIT, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr);

	// transmit the last remaining bits of the present byte if any and updated counters as below
	LABEL(XMIT_MORE_BITS);
	// update the bytes done counter and reset the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
	ADD(suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBytesDoneCtr));

	XOR(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	XOR(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	// set the remaining bits to one, if there are no more bits in formated data to send
	// and still there is space in TX_DATA_reg.
	// 16 - bitsLoaded
	RSB(scratch_reg1, tx_context.bitsLoaded, 16);
	// Load the remaining (16 - bitsLoaded) bits with logic High
	XOR(scratch_reg3, scratch_reg3, scratch_reg3);
	NOT(scratch_reg3, scratch_reg3);

	// calculate the bit position from where one is to be inserted
	RSB(scratch_8bit_reg2, scratch_reg1, 16);
	// CLR scratch_reg2
	XOR(scratch_reg2, scratch_reg2, scratch_reg2);
	// COPY 1 bit to scratch_reg2 from scratch_reg3
	AND(scratch_reg2, scratch_reg3, 0x1);
	LSL(scratch_reg2, scratch_reg2, scratch_8bit_reg2);

	// Now, set the remaining bits to one in TX_DATA_reg
	LABEL(SET_BIT_BIT);
	OR(TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2);
	LSL(scratch_reg2, scratch_reg2, 1);
	SUB(scratch_reg1, scratch_reg1, 1);
	QBLE(SET_BIT_BIT, scratch_reg1, 1);

	LDI(tx_context.bitsLoaded, 16);
	JMP(CHK_MORE_PRESCALAR);
	JMP(TxInterruptServiceRequestHndlr);

	LABEL(XMIT_NXT_3BIT);
	// if the bitsLoaded in TX_DATA_reg is less than 16 load the next bits
	// (bitsLoaded < 16)
	QBLT(CHK_MORE_PRESCALAR, tx_context.bitsLoaded, 16);

	LABEL(BIT_LOAD16);
	// Read Prescalar value
	LDI(scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK);
	AND(scratch_reg1, scratch_reg2, suart_ch_regs.Chn_Config1);

	// (16 - bitsLoaded)
	RSB(scratch_reg2, tx_context.bitsLoaded, 16);
	// (Chn_Config1.PreScaller - ChnTxRxRepeatDoneCntr)
	SUB(scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxRepeatDoneCtr);

	MIN(scratch_reg1, scratch_reg1, scratch_reg2);

	// Read Next Bit
	JAL(JMP_CALL_reg, READ_TX_DATA);
	LSR(scratch_reg3, scratch_reg3, suart_ch_regs.Chn_TxRxBitsDoneCtr);

	// copy bit to transmitted to scratch_reg2
	AND(scratch_reg2, scratch_reg3, 0x1);
	// move repeat count to scratch_reg4
	MOV(scratch_reg4, scratch_reg1);
	// shift the bit to be transmitted to expected position
	LSL(scratch_reg2, scratch_reg2, tx_context.bitsLoaded);

	// prescale the bit to transmitted
	LABEL(PRESCALE_NXT_BIT);
	OR(TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2);
	LSL(scratch_reg2, scratch_reg2, 1);
	SUB(scratch_reg1, scratch_reg1, 1);
	QBLE(PRESCALE_NXT_BIT, scratch_reg1, 1);

	// write back to memory
	ADD(tx_context.bitsLoaded, tx_context.bitsLoaded, scratch_reg4);

	ADD(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, scratch_reg4);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	// get the prescalar value
	LDI(scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK);
	AND(scratch_reg1, scratch_reg2, suart_ch_regs.Chn_Config1);

	//if bit has been transmitted prescaler times, fall through and updated the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
	QBGT(CHK_MORE_PRESCALAR, suart_ch_regs.Chn_TxRxRepeatDoneCtr, scratch_reg1);

	// rename to TX_BIT_DONE_CNTR
	LABEL(TX_DONE_CNTR6);
	// Write Chn_TxRxBitsDoneCtr
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// Write Chn_TxRxRepeatDoneCtr
	XOR(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	//rename this label to CHK_TX_DATA_REG_FULL
	LABEL(CHK_MORE_PRESCALAR);
	// if (bitsLoaded < 16), next bit can be loaded in TX_DATA_reg
	QBGT(LOAD_BITS_LOOP_FOR6, tx_context.bitsLoaded, 0x10);
	// TX_DATA_reg is full, transmit the data
	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);
	JMP(TxInterruptServiceRequestHndlr);

	// transmit the bits from start bit that can be transmitted from present character that is to transmitted
	LABEL(XMIT_FIRST_3BIT);
	// copy the first 3 bits to be transmitted
	AND(scratch_reg3, scratch_reg3, 0x7);
	// number of times the byte loop is to be looped
	LDI(scratch_reg2, 12);
	// Clear TX_DATA_reg.w0
	XOR(TX_DATA_reg.w0, TX_DATA_reg.w0, TX_DATA_reg.w0);
	XOR(scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2);
	JAL(JMP_CALL_reg, BYTE_LOOP);

	// Repeat last bit by 4 times
	LDI(scratch_reg1, 4);

	// CLR scratch_reg2
	XOR(scratch_reg2, scratch_reg2, scratch_reg2);
	// copy the third bit to scratch_reg2
	AND(scratch_reg2, scratch_reg3, 0x1);

	// shift the bit to expected place i.e. bit ps 12
	LSL(scratch_reg2, scratch_reg2, 12);

	// prescale the last bit 4 times
	LABEL(PRESCALE_LAST_4BIT);
	OR(TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2);
	LSL(scratch_reg2, scratch_reg2, 1);
	SUB(scratch_reg1, scratch_reg1, 1);
	QBLE(PRESCALE_LAST_4BIT, scratch_reg1, 1);

	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	// Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 2);
	// Write To RAM number of Bits Transmitted
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 4);
	// Write To RAM Write Repeat done counter to RAM
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	JMP(TxInterruptServiceRequestHndlr);

//******************************************** PRE_SCALAR6 ENDs **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR12 Starts *******************************************

	LABEL(PRE_SCALAR12);
	QBGT(XMIT_FIRST_2BIT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	JMP(GENERIC_TRANSMIT_BLOCK);

	LABEL(XMIT_FIRST_2BIT);
	// copy the first two bit to be loaded in  TX_DATA_reg
	AND(scratch_reg3, scratch_reg3, 0x3);
	// To left shift each copied data bit
	LDI(scratch_8bit_reg2, 0x0);
	// Keep track of byte_loop loop count
	LDI(scratch_reg2, 12);
	XOR(scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2);
	JAL(JMP_CALL_reg, BYTE_LOOP);

	// CLR scratch_reg2
	XOR(scratch_reg2, scratch_reg2, scratch_reg2);
	// copy the next bit to prescaled
	AND(scratch_reg2, scratch_reg3, 0x1);
	// counter to prescale second bit by 4
	LDI(scratch_reg1, 4);
	// shift the bit to desired position
	LSL(scratch_reg2, scratch_reg2, 0xC);

	LABEL(PRESCALE_4BIT);
	OR(TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2);
	LSL(scratch_reg2, scratch_reg2, 1);
	SUB(scratch_reg1, scratch_reg1, 1);
	QBLE(PRESCALE_4BIT, scratch_reg1, 1);

	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	// Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	// Write To RAM number of Bits Transmitted
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 4);
	// Write To RAM number of Bits Repeated
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	JMP(TxInterruptServiceRequestHndlr);

//******************************************** PRE_SCALAR12 ENDs *********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR16 Starts *******************************************

	LABEL(PRE_SCALAR16);
	QBGT(XMIT_FIRST_16, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	JMP(GENERIC_TRANSMIT_BLOCK);

	LABEL(XMIT_FIRST_16);
	// copy the first two bit to be loaded in  TX_DATA_reg
	AND(scratch_reg3, scratch_reg3, 0x2);
	// Left shift each copied data bit
	LDI(scratch_8bit_reg2, 0x0);
	// Keep track of byte_loop loop count
	LDI(scratch_reg2, 16);
	// Clear TX_DATA_reg.w0
	XOR(TX_DATA_reg.w0, TX_DATA_reg.w0, TX_DATA_reg.w0);
	XOR(scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2);
	JAL(JMP_CALL_reg, BYTE_LOOP);

	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	// Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	// Write To RAM number of Bits Transmitted
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0);
	// Write To RAM number of Bits Repeated
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	JMP(TxInterruptServiceRequestHndlr);

//******************************************** PRE_SCALAR16 ENDs *********************************************

//======================================================================================================================================

//********************************************* PRE_SCALAR24 Starts ******************************************

	LABEL(PRE_SCALAR24);
	QBGT(XMIT_FIRST_24, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	JMP(GENERIC_TRANSMIT_BLOCK);

	LABEL(XMIT_FIRST_24);
	LDI(scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK);
	AND(scratch_reg1, scratch_reg2, suart_ch_regs.Chn_Config1);
	// Chn_TxRxConfig1.PreScaler - ChnTxRxRepeadDoneCnt
	SUB(scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	//(Chn_TxRxConfig1.PreScaler - ChnTxRxRepeadDoneCntr >= 16 )
	QBLE(PRESCALE_START_BIT, scratch_reg1, 16);

	LABEL(PRESCALE_FIRST_DATA_BIT);
	// Clear Scratch reg
	XOR(scratch_reg3, scratch_reg3, scratch_reg3);
	// Updating number of bits written
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);

	JAL(JMP_CALL_reg, READ_TX_DATA);

	// get the bits to be transmitted
	LSR(scratch_reg3, scratch_reg3, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	AND(scratch_reg2, scratch_reg3, 0x1);

	// shift the bit to desired bit position
	LSL(scratch_reg2, scratch_reg2, scratch_reg1);
	RSB(scratch_reg1, scratch_reg1, 16);
	MOV(suart_ch_regs.Chn_TxRxRepeatDoneCtr, scratch_reg1);

	LABEL(PRESCALE_FIRST_DAT_BIT);
	OR(TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2);
	LSL(scratch_reg2, scratch_reg2, 1);
	SUB(scratch_reg1, scratch_reg1, 1);
	QBLE(PRESCALE_FIRST_DAT_BIT, scratch_reg1, 1);

	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	// Write To RAM number of Bits Transmitted
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// Write To RAM Chn_TxRxRepeatDoneCtr
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));
	JMP(TxInterruptServiceRequestHndlr);

	LABEL(PRESCALE_START_BIT);
	LDI(scratch_reg1, 0x10);
	LDI(scratch_reg2, 0x10);
	// to left shift each copied data bit
	XOR(scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2);
	JAL(JMP_CALL_reg, BITS_LOOP);
	JAL(JMP_CALL_reg, TRANSMIT_PRESCALED_DATA);

	// Update number of bits written
	ADD(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 16);
	// Write To RAM number of Bits Repeated
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));
	JMP(TxInterruptServiceRequestHndlr);

//************************************************ PRE_SCALAR24 ENDs *****************************************

//======================================================================================================================================

//======================================================================================================================================

//			************************************************************************************************
//												TX SUB ROUTINE Section
//			**************************************************************************************************
//======================================================================================================================================

//======================================================================================================================================


//********************************************** LOAD_TX_COMMON_INFO:  Start *********************************

//======================================================================================================================================
// 	Loads The X_BUF, SRCTL address mapped with present TX Channel and format data address which stores formatted data
// 	obtained from ARM/DSP specific to channel, also formats the data if it is required
//======================================================================================================================================

	LABEL(LOAD_TX_COMMON_INFO);
	// Load the TX Format Address for the specific channel
	QBEQ(LOAD_TX_FORMAT_ADDRESS_DONE, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY);
	JAL(JMP_CALL_reg, LOAD_TX_FORMAT_ADDRESS);

	LABEL(LOAD_TX_FORMAT_ADDRESS_DONE);
	//  Load the mapped SR and XBUF address mapped to channel
	JMP(LOCATE_SR_XBUF_SRCTL);

	LABEL(LOCATE_SR_XBUF_SRCTL_DONE);
	// Format the data if required
	JMP(CHK_TX_DATA_FORMAT);

	LABEL(CHK_TX_DATA_FORMAT_DONE);
	JMP(ENABLE_TX_SERIALIZER);

//****************************************** LOAD_TX_COMMON_INFO: ENDS ***************************************

//======================================================================================================================================

//****************************************** TX LOCATE_SR_XBUF_SRCTL : Starts ********************************

//======================================================================================================================================
// 		Subroutine to find channel specific serializer, xbuf, srctl register mapped active channel
//======================================================================================================================================

	LABEL(LOCATE_SR_XBUF_SRCTL);
	// Calculating Serializer Mapped to Channel
	AND(scratch_reg1, suart_ch_regs.Chn_Cntrl.u8.b1, SUART_CTRL_SERIALIZER_MASK);
	LSL(scratch_reg1, scratch_reg1, SUART_CTRL_SRCTL_BIT_SHIFT);

	// copy the tx format address to temp regsiter
	MOV(scratch_reg3, tx_context.buff_addr);
	LDI(scratch_reg4, TX_FMT_DATA_TO_TX_CONTEXT_OFFSET);

	// Calculating the specific SRCTL register offset
	MOV32(tx_context.asp_xsrctl_reg.u16, MCASP_SRCTL_BASE);
	ADD(tx_context.asp_xsrctl_reg.u32, tx_context.asp_xsrctl_reg.u32, scratch_reg1);

	ADD(scratch_reg2, scratch_reg4, SUART_CH_ASP_XSRCTL_REG_OFFSET);
	SBBO(&tx_context.asp_xsrctl_reg, scratch_reg3, scratch_reg2, $sizeof(tx_context.asp_xsrctl_reg));

	// Calculating the specific xbuf register offset
	MOV32(tx_context.asp_xbuf_reg.u16, MCASP_XBUF_BASE);
	ADD(tx_context.asp_xbuf_reg.u32, tx_context.asp_xbuf_reg.u32, scratch_reg1);

	ADD(scratch_reg2, scratch_reg4, SUART_CH_ASP_XBUF_REG_OFFSET);
	SBBO(&tx_context.asp_xbuf_reg, scratch_reg3, scratch_reg2, $sizeof(tx_context.asp_xbuf_reg));

	// Store the data length
	MOV(tx_context.buff_size, suart_tx_ch.Chn_Config2.u8.b1);

	ADD(scratch_reg2, scratch_reg4, SUART_CH_BUFF_SIZE_OFFSET);
	SBBO(&tx_context.buff_size, scratch_reg3, scratch_reg2, $sizeof(tx_context.buff_size));

	//Store the data Tx FMT Context address
	ADD(scratch_reg2, scratch_reg4, SUART_CH_BUFF_ADDR_OFFSET);
	SBBO(&tx_context.buff_addr, scratch_reg3, scratch_reg2, $sizeof(tx_context.buff_addr));

	LDI(tx_context.bitsLoaded, 0x00);
	ADD(scratch_reg2, scratch_reg4, SUART_CH_BITSLOADED_OFFSET);
	SBBO(&tx_context.bitsLoaded, scratch_reg3, scratch_reg2, $sizeof(tx_context.bitsLoaded));

	JMP(LOCATE_SR_XBUF_SRCTL_DONE);

//********************************************** TX LOCATE_SR_XBUF_SRCTL: ENDS **************************************

//======================================================================================================================================

//********************************************** TX CHK_TX_DATA_FORMAT : Starts***************************************

//======================================================================================================================================
// 	Check For Data Formating, formats the data only if Chn_TxRxRepeatDoneCtr,
//	Chn_TxRxBitsDoneCtr, Chn_TxRxBytesDoneCtr all are zero,
//	If All the conditions is satisfied, it jumps to TX_DATA_FORMAT Subroutine  and
// 	formats data the TX Data obtained from ARM/DSP by adding start and stop bit.
//======================================================================================================================================

	LABEL(CHK_TX_DATA_FORMAT);
	QBEQ(CHK_TX_DATA_FORMAT_BITS, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0);
	JMP(CHK_TX_DATA_FORMAT_DONE);

	LABEL(CHK_TX_DATA_FORMAT_BITS);
	QBEQ(CHK_TX_DATA_FORMAT_BYTE, suart_ch_regs.Chn_TxRxBitsDoneCtr, 0);
	JMP(CHK_TX_DATA_FORMAT_DONE);

	LABEL(CHK_TX_DATA_FORMAT_BYTE);
	QBEQ(TX_DATA_FORMAT, suart_ch_regs.Chn_TxRxBytesDoneCtr, 0);
	JMP(CHK_TX_DATA_FORMAT_DONE);

	LABEL(TX_DATA_FORMAT);
	// Load the TX Format Address for the specific channel
	XOR(scratch_reg1, scratch_reg1, scratch_reg1);
	NOT(scratch_reg1, scratch_reg1);

	SUB(scratch_reg2, suart_tx_ch.Chn_Config2.u8.b0, 1);

	LSL(scratch_reg1, scratch_reg1, scratch_reg2);
	// offset from base addr
	XOR(scratch_reg2, scratch_reg2, scratch_reg2);

	// to store formated data into DATA RAM
	MOV(scratch_reg3, tx_context.buff_addr);

	// Number of Bits Per Character
	AND(scratch_8bit_reg1, suart_tx_ch.Chn_Config2.u8.b0, 0xF);
	SUB(scratch_8bit_reg1, scratch_8bit_reg1, 2);

	LABEL(TX_DATA_FORMAT_LOOP);
	// Load the data from the data pointer
	LBBO(&TX_DATA_reg, suart_ch_regs.ch_TxRxData, 0, 2);
	LSL(TX_DATA_reg, TX_DATA_reg, 1);
	OR(TX_DATA_reg, TX_DATA_reg, scratch_reg1);

	// store formated data into DATA RAM
	SBBO(&TX_DATA_reg.w0, scratch_reg3, scratch_reg2, 2);

	// Increment the formatted buffer address offset
	ADD(scratch_reg2, scratch_reg2, 2);

	QBGE(INC_ADDR_BY_ONE, scratch_8bit_reg1, SUART_CH_CONFIG2_8BITS_PER_CHAR);
	// Next data buffer pointer
	ADD(suart_ch_regs.ch_TxRxData, suart_ch_regs.ch_TxRxData, 1);

	// Increamnet the tx buffer data pointer by ONE, if bit per character is less or equal to 8 including start and stop bit
	LABEL(INC_ADDR_BY_ONE);
	// Next data buffer pointer
	ADD(suart_ch_regs.ch_TxRxData, suart_ch_regs.ch_TxRxData, 1);

	QBEQ(CHK_TX_DATA_FORMAT_DONE, tx_context.buff_size, 0);

	//Decrement the data length .i.e no of bytes to send
	SUB(tx_context.buff_size, tx_context.buff_size, 1);

	JMP(TX_DATA_FORMAT_LOOP);

//******************************************** TX CHK_TX_DATA_FORMAT: ENDS************************************

//======================================================================================================================================

//******************************************** TX READ_TX_DATA: Starts****************************************

//======================================================================================================================================
// 	Reads the 16 bit formatted character to be transmitted from formatted data area corresponding to TX channel
//======================================================================================================================================

	LABEL(READ_TX_DATA);
	// Copy the base address of formated data
	MOV(scratch_reg3, tx_context.buff_addr);

	// Calculate the offset of formated data
	LSL(scratch_reg4, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1);

	// LOAD formated data from DATA RAM
	LBBO(&scratch_reg3, scratch_reg3, scratch_reg4, 2);

	JMP(JMP_CALL_reg);

//********************************************** TX READ_TX_DATA: ENDS ***************************************

//======================================================================================================================================

//********************************************** TX LOAD_TX_FORMAT_ADDRESS : Starts **************************

//======================================================================================================================================
//	Initializes the TX formatted data buffer address which stores the formated data with stop and start bit
//======================================================================================================================================

	LABEL(LOAD_TX_FORMAT_ADDRESS);
	QBEQ(TX_CH0_FMT_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_0);
	QBEQ(TX_CH2_FMT_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_2);
	QBEQ(TX_CH4_FMT_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_4);
	QBEQ(TX_CH6_FMT_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_6);
	JMP(JMP_CALL_reg);

	LABEL(TX_CH0_FMT_ADDR);
	LDI(tx_context.buff_addr, SUART_CH0_TX_FMT_ADDR);
	JMP(JMP_CALL_reg);

	LABEL(TX_CH2_FMT_ADDR);
	LDI(tx_context.buff_addr, SUART_CH2_TX_FMT_ADDR);
	JMP(JMP_CALL_reg);

	LABEL(TX_CH4_FMT_ADDR);
	LDI(tx_context.buff_addr, SUART_CH4_TX_FMT_ADDR);
	JMP(JMP_CALL_reg);

	LABEL(TX_CH6_FMT_ADDR);
	LDI(tx_context.buff_addr, SUART_CH6_TX_FMT_ADDR);
	JMP(JMP_CALL_reg);

//******************************************** TX LOAD_TX_FORMAT_ADDRESS Routine: ENDS ***********************

//======================================================================================================================================

//******************************************** PRESACLE_TX_DATA : Starts *************************************

//======================================================================================================================================
// This routine Prescales data bit to be transmitted into the TX_DATA_reg.w0 register
//======================================================================================================================================

	LABEL(PRESACLE_TX_DATA);
	// to Left shift each copied data bit
	XOR(scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2);
	// Xbuf Size: Keep track of byte loop
	LDI(scratch_reg2, 16);
	// Clear TX_DATA_reg.w0
	XOR(TX_DATA_reg.w0, TX_DATA_reg.w0, TX_DATA_reg.w0);

	LABEL(BYTE_LOOP);
	LDI(scratch_reg4, SUART_CTRL_PRE_SCALAR_MASK);
	AND(scratch_reg1, scratch_reg4, suart_ch_regs.Chn_Config1);

	LABEL(BITS_LOOP);
	// CLR scratch_reg4
	XOR(scratch_reg4, scratch_reg4, scratch_reg4);
	// COPY 1 bit to scratch_reg4 from scratch_reg3
	AND(scratch_reg4, scratch_reg3, 0x1);
	LSL(scratch_reg4, scratch_reg4, scratch_8bit_reg2);
	OR(TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg4);
	ADD(scratch_8bit_reg2, scratch_8bit_reg2, 1);
	// INC Bytes loop counter
	SUB(scratch_reg2, scratch_reg2, 1);
	// INC Bits loop counter
	SUB(scratch_reg1, scratch_reg1, 1);
	QBLE(BITS_LOOP, scratch_reg1, 1);
	LSR(scratch_reg3, scratch_reg3, 1);
	QBLE(BYTE_LOOP, scratch_reg2, 1);
	// Return to PRESACLE_TX_DATA
	JMP(JMP_CALL_reg);

//********************************************  PRESACLE_TX_DATA : ENDs **************************************

//======================================================================================================================================

//******************************************** TRANSMIT_PRESCALED_DATA : Starts ******************************

//======================================================================================================================================
// This routine Transmits the prescaled data by writing to mcasp x_buf register mapped to this serializer
//======================================================================================================================================

	LABEL(TRANSMIT_PRESCALED_DATA);
	// Clear the under run error
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4);
	QBBC(WRITE_TO_XBUF, scratch_reg1, 0x8);

	LDI(scratch_reg1, 0xFFFF);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4);

	LABEL(WRITE_TO_XBUF);
	// Write Byte to X_BUF
	SBBO(&TX_DATA_reg.w0, tx_context.asp_xbuf_reg.u32, 0, 4);
	// return from Transmit Prescaled Data
	JMP(JMP_CALL_reg);

//******************************************** TRANSMIT_PRESCALED_DATA : ENDs ********************************

//======================================================================================================================================

//******************************************** TX_DONE : Starts **********************************************

//======================================================================================================================================
// This routine the cleanup after one character has been transmitted successfully
//======================================================================================================================================

	LABEL(TX_DONE);
	XOR(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	// Write To RAM number of Bits Transmitted
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	ADD(suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBytesDoneCtr));

	JMP(TxInterruptServiceRequestHndlr);

//******************************************** TX_DONE : ENDs ************************************************

//======================================================================================================================================

//******************************************** TxInterruptServiceRequestHndlr: Starts ************************

//=======================================================================================================================================
// This routine is called when there is mcasp TX event, it searches for the active TX serializer by scanning the
// serializer registers and if detected, finds the corresponding Tx channel and finally loads the context info for the
// channel, otherwise, if it fails any condition jumps to core loop.
// This routine also causes the PRU to ARM interrupt to be raised if transfer of requested character is finished other wise
// call the routine TxServiceReqHndlLoop to transfer remaining  characters.
// It also, causes the dummy transfer when there is no character to send
//=======================================================================================================================================

	LABEL(TxInterruptServiceRequestHndlr);
	// Retrieve the channel number and load the context base
	LDI(suart_ch_info.curr_ch_base_addr, SUART_CH_BASE_ADDRESS);
	LDI(suart_ch_info.curr_ch_offset, SUART_CH_BASE_ADDRESS);
	LDI(suart_ch_info.ch_num, 0x00);
	LDI(tx_context.buff_addr, 0x90);

	LABEL(SERCH_MAPPED_TX_CHN);
	ADD(scratch_reg1, suart_ch_info.curr_ch_offset, SUART_CH_TXRXCHNSTATUS_OFFSET);
	// Load the Channel Cntrl info from Memory to Register
	LBBO(&suart_ch_regs.Chn_Status, suart_ch_info.curr_ch_base_addr, scratch_reg1, $sizeof(suart_ch_regs.Chn_Status));
	QBBC(NEXT_TX_CHN, suart_ch_regs.Chn_Status, SUART_CH_TXRXCHNSTATUS_BIT);

	ADD(scratch_reg1, tx_context.buff_addr, TX_FMT_DATA_TO_TX_CONTEXT_OFFSET);
	LBBO(&scratch_reg2, scratch_reg1, SUART_CH_ASP_XSRCTL_REG_OFFSET, 4);
	LBBO(&scratch_reg1, scratch_reg2, 0, 4);
	QBBS(MAPPED_TX_CHN_FOUND, scratch_reg1, ASP_SRCTL_XRDY_BIT);

	LABEL(NEXT_TX_CHN);
	QBEQ(PRU_TX_ONLY_MODE, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY);

	// TX & RX together. So channel nunbers are 0, 2, 4, 6
	ADD(suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE2);
	ADD(suart_ch_info.ch_num, suart_ch_info.ch_num, 0x02);
	ADD(tx_context.buff_addr, tx_context.buff_addr, SUART_TX_FMT_OFFSET);
	QBGE(SERCH_MAPPED_TX_CHN, suart_ch_info.ch_num, NUM_OF_CHANNELS);
	JMP(CORE_LOOP);

	LABEL(PRU_TX_ONLY_MODE);
	// TX Only ...So channel numbers are contiguous
	ADD(suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE);
	ADD(suart_ch_info.ch_num, suart_ch_info.ch_num, 0x01);
	ADD(tx_context.buff_addr, tx_context.buff_addr, 0x2C);
	QBGE(SERCH_MAPPED_TX_CHN, suart_ch_info.ch_num, NUM_OF_CHANNELS);
	JMP(CORE_LOOP);

	LABEL(MAPPED_TX_CHN_FOUND);
	LBBO(&suart_ch_regs, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset, $sizeof(suart_ch_regs));
	ADD(suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset);

	QBEQ(PRUx_MODE_TX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY);

	QBEQ(CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_INVALID);

	LABEL(PRUx_MODE_TX_ONLY);
	ADD(scratch_reg1, tx_context.buff_addr, TX_FMT_DATA_TO_TX_CONTEXT_OFFSET);
	LBBO(&tx_context, scratch_reg1, 0, $sizeof(tx_context));

	// JMP TO TxServiceReqHndlLoop Chn_TxRxBytesDoneCtr is less than Data length
	LSR(scratch_reg1, suart_ch_regs.Chn_Config2.u16, SUART_CH_CONFIG2_DATALEN_SHIFT);
	AND(scratch_reg1, scratch_reg1, SUART_CH_CONFIG2_DATALEN_MASK);
	ADD(scratch_reg1, scratch_reg1, 0x01);
	QBLT(TxServiceReqHndlLoop, scratch_reg1, suart_ch_regs.Chn_TxRxBytesDoneCtr);

	QBBS(DECLARE_COMPLETE, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT);
	NOT(scratch_reg1, pZERO);
	SBBO(&scratch_reg1, tx_context.asp_xbuf_reg.u32, 0, 4);

	JMP(CORE_LOOP);

	LABEL(DECLARE_COMPLETE);
	// Set the status in the context area
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_COMPLETE_BIT);
	CLR(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT);
	SBBO(&suart_ch_regs.Chn_TxRxStatus, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXSTATUS_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxStatus));

	// Generate the interrupt to the ARM/DSP about the completion
	LBBO(&suart_global.intrMask, pZERO, SUART_GBL_INT_MASK_ADDR, $sizeof(suart_global.intrMask));
	QBBC(CORE_LOOP, suart_global.intrMask, suart_ch_info.ch_num);

	LBBO(&suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, $sizeof(suart_global.intrStatus));
	SET(suart_global.intrStatus, suart_global.intrStatus, suart_ch_info.ch_num);
	SBBO(&suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, $sizeof(suart_global.intrStatus));
	JMP(PRU_TO_HOST_INTERRUPT);

//******************************************** TxInterruptServiceRequestHndlr: ENDs **************************

//======================================================================================================================================

//			************************************************************************************************
//									SOFT-UART TRANSMIT ROUTINE : ENDS
//			************************************************************************************************

//======================================================================================================================================

//======================================================================================================================================

//******************************************** CORE LOOP: Starts *********************************************

//========================================================================================================================================
//  CORE LOOP where the PRU polls for System Interrupts i.e ARM_DSP_EVENT and MCASP_EVENT
//  when there is ARM_DSP_EVENT DSP event it jumps to Channel search to acknowledge the service request from ARM/DSP,
// otherwise if MCASP_EVENT (TX/RX) occurs it goes to corresponding interrupt handler (TX/RX) to service the interrupt
// if there is no event, it loops in the core loop and waits for interrupt form ARM or DSP.
//========================================================================================================================================

	LABEL(CORE_LOOP);
	QBEQ(CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_INVALID);

	LABEL(ARM_DSP_EVENT);
	QBEQ(CORE_LOOP_PRU1, suart_global.pru_id, 1);

	LABEL(CORE_LOOP_PRU0);
	// wait for the hostEventStatus to get set. Loop till then
	WBS(hostEventStatus, ARM_TO_PRU0_INT);

	// Read the PRUINTC register to know if the event is from ARM/DSP. If yes, then branch
	MOV32(scratch_reg2, SRSR2_OFFSET);
	LBCO(&scratch_reg1, CONST_PRUSSINTC, scratch_reg2, 4);
	QBBS(CHN_SEARCH, scratch_reg1, 0);

	// Else it is McASP Event. So before proceeding, clear it
	MOV32(scratch_reg1, MCASP_TXRX_EVENT);
	SBCO(&scratch_reg1, CONST_PRUSSINTC, SICR_OFFSET, 4);

	JMP(MCASP_EVENT);

	LABEL(CORE_LOOP_PRU1);
	// wait for the hostEventStatus to get set. Loop till then
	WBS(hostEventStatus, ARM_TO_PRU1_INT);

	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4);
	QBBC(CHN_SEARCH, scratch_reg1, 5);

	// Clear the event here and go to Transmit processing
	MOV32(scratch_reg1, PRU0_TO_PRU1_EVENT);
	SBCO(&scratch_reg1, CONST_PRUSSINTC, SICR_OFFSET, 4);
	JMP(TxInterruptServiceRequestHndlr);

	LABEL(MCASP_EVENT);
	// Check for RX interrrupt first
	// If TX only PRU Skip RSTAT Check
	QBEQ(MCASP_TX_EVNT, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY);

	// if the PRU is RX only mode, then check if the XSTAT is set. If so, raise event to PRU1 and proceed
	QBNE(RX_TX_PROCESS, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4);
	QBBC(RX_TX_PROCESS, scratch_reg1, 5);
	MOV32(scratch_reg1, PRU0_TO_PRU1_EVENT);
	SBCO(&scratch_reg1, CONST_PRUSSINTC, SISR_OFFSET, 4);

	LABEL(RX_TX_PROCESS);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RSTAT, 4);
	QBBS(RxInterruptServiceRequestHndlr, scratch_reg1, 5);
	// Skip the check for XSTAT if we are not Rx/Tx PRU.
	// We don't want the PRU to spin in a tight loop around the McASP register to introduce a delay
	QBNE(CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_RX_TX_BOTH);

	LABEL(MCASP_TX_EVNT);
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4);
	QBBS(TxInterruptServiceRequestHndlr, scratch_reg1, 5);
	// If PRU is both TX/RX, then go back to Core-loop. Else delay to avoid McASP Spins
	QBEQ(CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_RX_TX_BOTH);

//******************************************** CORE LOOP: Ends ***********************************************

//========================================================================================================================================

//******************************************** CHN_SEARCH: Starts ********************************************

//========================================================================================================================================
//	Retrieve the active channel number that has requested for serviced and
//	load the context base info for that channel
//========================================================================================================================================

	LABEL(CHN_SEARCH);
	MOV32(scratch_reg3, 0x1);
	LSL(scratch_reg3, scratch_reg3, suart_global.pru_id);
	MOV32(scratch_reg1, SECR2_OFFSET);
	SBCO(&scratch_reg3, CONST_PRUSSINTC, scratch_reg1, 4);

	// Read Global control register
	LBBO(&suart_global, pZERO, SUART_GBL_CTRL_ADDR, $sizeof(suart_global));

	// Retrieve the channel number and load the context base
	LDI(suart_ch_info.curr_ch_base_addr, SUART_CH_BASE_ADDRESS);
	LDI(suart_ch_info.curr_ch_offset, SUART_CH_BASE_ADDRESS);
	LDI(suart_ch_info.ch_num, 0x00);
	XOR(tx_context.buff_addr, tx_context.buff_addr, tx_context.buff_addr);
	XOR(suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr);

	LDI(tx_context.buff_addr, 0x90);
	LDI(suart_ch_info.rx_context_addr, 0x90);

	LABEL(CHN_ACTIVE);
	LBBO(&suart_ch_regs.Chn_Cntrl, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset, $sizeof(suart_ch_regs.Chn_Cntrl));
	QBBS(CHN_SERACH_RTN, suart_ch_regs.Chn_Cntrl.u16, SUART_CTRL_SR_BIT);
	ADD(suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE);
	ADD(suart_ch_info.ch_num, suart_ch_info.ch_num, 0x01);
	ADD(tx_context.buff_addr, tx_context.buff_addr, 0x2C);
	ADD(suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr, 0x20);

	// None of the channel has service request, go back to MainLoop
	// check to be verified to boundary condition
	QBLT(MCASP_EVENT, suart_ch_info.ch_num, NUM_OF_CHANNELS);
	JMP(CHN_ACTIVE);

	LABEL(CHN_SERACH_RTN);
	LBBO(&suart_ch_regs, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset, $sizeof(suart_ch_regs));
	ADD(suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset);

	AND(scratch_reg1.w0, suart_ch_regs.Chn_Cntrl.u16, SUART_CTRL_MODE_MASK);
	QBEQ(TxServiceRequestHndlr, scratch_reg1.w0, SUART_CH_TX_MODE);
	QBEQ(RxServiceRequestHndlr, scratch_reg1.w0, SUART_CH_RX_MODE);
	JMP(CORE_LOOP);

//******************************************** CHN_SEARCH: Ends **********************************************

//======================================================================================================================================

//******************************************** PRU_TO_HOST_INTERRUPT[SINGLE_PRU] starts ********************************

//========================================================================================================================================
// 	This routine raises the interrupt to ARM/DSP once the serivce request given by ARM/DSP is serviced and
// 	Jump to Core loop. The interrupt has been configured on per channel basis i.e. each RX and TX channel will
//	raise separate event to PRU INTC and that will raise an interrup to ARM/DSP. Depending upon PRU0/PRU1,
//      PRU raises the system events to ARM/DSP. The Mapping for PRU system event on per channel basis is given below.
//		 PRU0
//		 PRU0 - SYS_EVT34 (host2 - ch2) - SUART Channel 0 	        (SUART0: TX)
//		 PRU0 - SYS_EVT35 (host2 - ch3) - SUART Channel 1		(SUART0: RX)
//		 PRU0 - SYS_EVT36 (host3 - ch4) - SUART Channel 2		(SUART1: TX)
//		 PRU0 - SYS_EVT37 (host3 - ch5) - SUART Channel 3		(SUART1: RX)
//		 PRU0 - SYS_EVT38 (host4 - ch2) - SUART Channel 4		(SUART2: Tx)
//		 PRU0 - SYS_EVT39 (host4 - ch3) - SUART Channel 5		(SUART2: RX)
//		 PRU0 - SYS_EVT40 (host5 - ch4) - SUART Channel 6		(SUART3: TX)
//		 PRU0 - SYS_EVT41 (host5 - ch5) - SUART Channel 7		(SUART3: RX)
//
//		 PRU1
//		 PRU1 - SYS_EVT42 (host6 - ch6) - SUART Channel 8		(SUART4: TX)
//		 PRU1 - SYS_EVT43 (host6 - ch7) - SUART Channel 9		(SUART4: RX)
//		 PRU1 - SYS_EVT44 (host7 - ch8) - SUART Channel 10	        (SUART5: TX)
//		 PRU1 - SYS_EVT45 (host7 - ch9) - SUART Channel 11	        (SUART5: RX)
//		 PRU1 - SYS_EVT46 (host8 - ch10) - SUART Channel 12	        (SUART6: TX)
//		 PRU1 - SYS_EVT47 (host8 - ch11) - SUART Channel 13	        (SUART6: RX)
//		 PRU1 - SYS_EVT48 (host9 - ch12) - SUART Channel 14	        (SUART7: TX)
//		 PRU1 - SYS_EVT49 (host9 - ch13) - SUART Channel 15	        (SUART7: RX)
//======================================================================================================================================

	LABEL(PRU_TO_HOST_INTERRUPT);
	LDI(scratch_reg1, 0);

	QBEQ(EVTOUT_PRU0_EVENTS, suart_global.pru_id, 0);
	QBEQ(EVTOUT_PRU1_EVENTS, suart_global.pru_id, 1);

	LABEL(EVTOUT_PRU0_EVENTS);
	//storing the counter value
	ADD(scratch_reg1, scratch_reg1, SYS_EVT_34);
	JMP(EVTOUT_SYSEVT_INIT);

	LABEL(EVTOUT_PRU1_EVENTS);
	ADD(scratch_reg1, scratch_reg1, SYS_EVT_42);

	LABEL(EVTOUT_SYSEVT_INIT);
	ADD(scratch_reg1, scratch_reg1, suart_ch_info.ch_num);

	LABEL(EVTOUT_GEN);
	// Clear SYS_EVTn
	SBCO(&scratch_reg1, CONST_PRUSSINTC, SICR_OFFSET, 4);

	// Enable SYS_EVTn system interrupt
	SBCO(&scratch_reg1, CONST_PRUSSINTC, EISR_OFFSET, 4);

	// Generate SYS_EVTn by event out mapping
	MOV(hostEventStatus.w0, scratch_reg1.w0);

	JMP(MCASP_EVENT);

//******************************************** PRU_TO_HOST_INTERRUPT : ENDS **********************************

//======================================================================================================================================

//=====================================================================================================================================

//			************************************************************************************************
//									SOFT-UART RECEIVE ROUTINE : STARTS
//			************************************************************************************************

//=====================================================================================================================================

//======================================================================================================================================

//******************************************** RxServiceRequestHndlr: Starts *********************************

//========================================================================================================================================
// 	This routine performs the baisc intialization and clearing of varoius registers for RX and loads the configuration info
// 	from PRU RAM to registers for RX channel that is being serviced. It calculates and saves serializer and r_buf address mapped
// 	to RX channel to RX context area so that each time it is not calculated again and is being directly read from TX Context Area
//	and activates the RX serilizer if it is diabled.
//========================================================================================================================================

	LABEL(RxServiceRequestHndlr);
	// load the max RX TRIES before time out
	LBBO(&MAX_RX_TIMEOUT_TRIES, pZERO, MAX_RX_TIMEOUT_TRIES_OFFSET, 2);

	// read interrupt status regsiter
	LBBO(&suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, $sizeof(suart_global.intrStatus));

	CLR(suart_global.intrStatus, suart_global.intrStatus, suart_ch_info.ch_num);

	// write interrupt status regsiter
	SBBO(&suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, $sizeof(suart_global.intrStatus));

	//Clear Service Request
	CLR(suart_ch_regs.Chn_Cntrl.u16, suart_ch_regs.Chn_Cntrl.u16, SUART_CTRL_SR_BIT);
	SBBO(&suart_ch_regs.Chn_Cntrl, suart_ch_info.curr_ch_base_addr, SUART_CH_CTRL_OFFSET, $sizeof(suart_ch_regs.Chn_Cntrl));

	// clear timeout flag
	CLR(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_TIMEOUT_BIT);

	// Set the TXRX_READY_BIT
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT);

	// update the RX Status Register
	SBBO(&suart_ch_regs.Chn_TxRxStatus, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXSTATUS_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxStatus));

	// Set the SUART_CH_TXRXCHNSTATUS_BIT to indicate the channel being active
	SET(suart_ch_regs.Chn_Status, suart_ch_regs.Chn_Status, SUART_CH_TXRXCHNSTATUS_BIT);
	SBBO(&suart_ch_regs.Chn_Status, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXCHNSTATUS_OFFSET, $sizeof(suart_ch_regs.Chn_Status));

	LABEL(RX_CONTEXT_INIT);
	QBEQ(PRUxxx_MODE_RX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY);

	// Load RX Context Base Address corresponding to Active RX Channel
	JAL(JMP_CALL_reg, LOAD_RX_CONTEXT_ADDRESS);

	LABEL(PRUxxx_MODE_RX_ONLY);
	// Calculating the specific SRCTL and R_BUF register offset.
	AND(scratch_reg1, suart_ch_regs.Chn_Cntrl.u8.b1, SUART_CTRL_SERIALIZER_MASK);
	LSL(scratch_reg1, scratch_reg1, SUART_CTRL_SRCTL_BIT_SHIFT);

	// Storing SRCTL register address in RX Context Area Region
	MOV32(rx_context.asp_rsrctl_reg.u16, MCASP_SRCTL_BASE);
	ADD(rx_context.asp_rsrctl_reg.u32, rx_context.asp_rsrctl_reg.u32, scratch_reg1);

	//storing asp_rsrctl_reg in RX Context Address Region
	SBBO(&rx_context.asp_rsrctl_reg, suart_ch_info.rx_context_addr, SUART_CH_ASP_RSRCTL_REG, $sizeof(rx_context.asp_rsrctl_reg));

	// Store RBuf Address in RX Context Region
	MOV32(rx_context.asp_rbuf_reg.u16, MCASP_RBUF_BASE);
	ADD(rx_context.asp_rbuf_reg.u32, rx_context.asp_rbuf_reg.u32, scratch_reg1);

	// storing asp_rbuf_reg in RX context  adress region
	SBBO(&rx_context.asp_rbuf_reg, suart_ch_info.rx_context_addr, SUART_CH_ASP_RBUF_REG, $sizeof(rx_context.asp_rbuf_reg));

	// Load the Context info specific to Current RX channel from memory to registers
	//	LBBO   	rx_context, suart_ch_info.rx_context_addr, 0, SIZE (rx_context)

	// Clear the RX timeout counter
	XOR(rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr);
	SBBO(&rx_context.rx_timeout_cntr, suart_ch_info.rx_context_addr, SUART_CH_RX_TIMEOUT_CNTR_OFFSET, $sizeof(rx_context.rx_timeout_cntr));

	// Activate RX serializer
	LBBO(&scratch_reg2, rx_context.asp_rsrctl_reg.u32, 0, 4);
	AND(scratch_reg2, scratch_reg2, 0x3);
	// Check if Serializer is Already Active as Rx if ,yes skip activation
	QBEQ(CLR_RSTAT, scratch_reg2, 0x2);
	//  Activate serializer as Receiver
	MOV32(scratch_reg2, 0x000E);
	SBBO(&scratch_reg2, rx_context.asp_rsrctl_reg.u32, 0, 4);

	LABEL(CLR_RSTAT);
	// Clear the RSTAT  (Overrun, etc)
	MOV32(scratch_reg1, 0xFFFF);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RSTAT, 4);

	JMP(MCASP_EVENT);

//******************************************** RxServiceRequestHndlr: ENDS ***********************************

//========================================================================================================================================

//******************************************** RxInterruptServiceRequestHndlr: Starts ************************

//========================================================================================================================================
//	RxInterruptServiceRequestHndlr is called when there is MCASP RX event, scans the active RX serializer,
//	once the active serializer is found, scans for corresponding RX channel, if it also is found, it loads
//	the context info for RX channel and proceeds for reading the frame tramsmitted by sender.
//========================================================================================================================================

	LABEL(RxInterruptServiceRequestHndlr);
	// Retrieve the channel number and load the RX context base info corressponding to serializer address in scratch_reg1 and serializer number  in scratch_reg4
	// Load the SUART CHANNEL BASE ADDRESS
	LDI(suart_ch_info.curr_ch_base_addr, SUART_CH_BASE_ADDRESS);

	QBEQ(PRUx_MODE_RX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY);

	// Since the Rx Channel are 1,3,5,7 Load the suart_ch_regs for Rx channel 1 as it is first channel
	ADD(suart_ch_info.curr_ch_offset, pZERO.w0, SUART_CH_REGS_SIZE);

	LDI(suart_ch_info.ch_num, 0x01);

	// Load the RX channel 1 context address to Ch_info register's rx_context_addr field
	LDI(suart_ch_info.rx_context_addr, SUART_CH1_RX_CONTEXT_ADDR);
	JMP(SERCH_ACTIVE_RX_CHN_RX);

	LABEL(PRUx_MODE_RX_ONLY);
	// Since the Rx Channel are 1,3,5,7 Load the suart_ch_regs for Rx channel 1 as it is first channel
	LDI(suart_ch_info.curr_ch_offset, 0x00);

	LDI(suart_ch_info.ch_num, 0x00);

	// Load the RX channel 1 context address to Ch_info register's rx_context_addr field
	LDI(suart_ch_info.rx_context_addr, 0x90);

	LABEL(SERCH_ACTIVE_RX_CHN_RX);
	ADD(scratch_reg1, suart_ch_info.curr_ch_offset, SUART_CH_TXRXCHNSTATUS_OFFSET);
	// Load the Channel Cntrl info from Memory to Register
	LBBO(&suart_ch_regs.Chn_Status, suart_ch_info.curr_ch_base_addr, scratch_reg1, $sizeof(suart_ch_regs.Chn_Status));
	QBBC(NEXT_RX_CHN, suart_ch_regs.Chn_Status, SUART_CH_TXRXCHNSTATUS_BIT);

	LBBO(&scratch_reg1, suart_ch_info.rx_context_addr, SUART_CH_ASP_RSRCTL_REG, 4);
	LBBO(&scratch_reg2, scratch_reg1, 0, 4);
	QBBS(ACTIVE_RX_CHN_FOUND, scratch_reg2, ASP_SRCTL_RRDY_BIT);

	LABEL(NEXT_RX_CHN);
	QBEQ(PRUxx_MODE_RX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY);

	// offset of RX suart_ch_regs
	ADD(suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE2);
	// Increament to Next Rx Channel number
	ADD(suart_ch_info.ch_num, suart_ch_info.ch_num, 0x2);

	// Increament rx_context_addr by RX_CONTEXT_OFFSET i.e. to next RX channel context address
	ADD(suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr, RX_CONTEXT_OFFSET);
	QBGE(SERCH_ACTIVE_RX_CHN_RX, suart_ch_info.ch_num, NUM_OF_CHANNELS);
	JMP(CORE_LOOP);

	LABEL(PRUxx_MODE_RX_ONLY);
	// offset of RX suart_ch_regs
	ADD(suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE);
	// Increamnet to Next Rx ChanneL number
	ADD(suart_ch_info.ch_num, suart_ch_info.ch_num, 0x1);
	// Increamnet rx_context_addr by RX_CONTEXT_OFFSET i.e. to next RX channel context address
	ADD(suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr, 0x20);
	QBGE(SERCH_ACTIVE_RX_CHN_RX, suart_ch_info.ch_num, NUM_OF_CHANNELS);
	JMP(CORE_LOOP);

	LABEL(ACTIVE_RX_CHN_FOUND);
	// Load the suart_ch_regs from Memory to Register
	LBBO(&suart_ch_regs, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset, $sizeof(suart_ch_regs));

	// Load the Context info specific to current RX Channel from memory to registers
	LBBO(&rx_context, suart_ch_info.rx_context_addr, 0, $sizeof(rx_context));

	ADD(suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset);

	// Clear the RSTAT  (Overrun, etc) for Errors
	LBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RSTAT, 4);
	QBBC(RX_PROCESSING_INIT, scratch_reg1, ASP_RSTAT_RERR_BIT);

	MOV32(scratch_reg1, 0xFFFF);
	SBCO(&scratch_reg1, MCASP_CONTROL, MCASP_RSTAT, 4);

	//  Start receving DATA from MAC_ASP's R-Buf corresponding to channel
	LABEL(RX_PROCESSING_INIT);
	XOR(mcasp_rbuf_val, mcasp_rbuf_val, mcasp_rbuf_val);
	// Read the content of RBUF
	LBBO(&scratch_reg1, rx_context.asp_rbuf_reg.u32, 0, 4);
	OR(mcasp_rbuf_val, mcasp_rbuf_val, scratch_reg1);

	// If start condition is already received then go to reading next bit  otherwise look for start condition
	QBLT(READ_CURRENT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 0);

	// check Chn_TxRxRepeatDoneCtr, if it is not zero, jump to READ_CURRENT to prescale the start condition
	QBLT(READ_CURRENT, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0);

	// If sampling point i.e. sampling_bit_pos is equal to greater than 16 (INVALID_SAMPLING_POINT),
	// start bit transition edge is being detected, fall through to calculate sampling point,
	// otherwise, sampling point is already calculated JUMP to READ_CURRENT
	QBGE(READ_CURRENT, rx_context.sampling_bit_pos, INVALID_SAMPLING_POINT);

	// Extract timing information by detecting start transition (first left most zero)
	LMBD(scratch_reg4, scratch_reg1, 0);
	// branch if zero: start bit transition detected
	QBGT(START_BIT_TRANSITION, scratch_reg4, ZERO_BIT_NOT_DETECTED);
	LDI(rx_context.sampling_bit_pos, 0xff);
	SBBO(&rx_context.sampling_bit_pos, suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, $sizeof(rx_context.sampling_bit_pos));

	// RX time out logic
	QBBC(RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_Config1, RX_TIMEOUT_INTR_MASK);
	QBBC(RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT);
	QBEQ(RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 0);

	// Read the request count to be received
	LSR(scratch_reg1, suart_ch_regs.Chn_Config2.u16, SUART_CH_CONFIG2_DATALEN_SHIFT);
	AND(scratch_reg1, scratch_reg1, SUART_CH_CONFIG2_DATALEN_MASK);
	// Since fifo size is 16
	ADD(scratch_reg1, scratch_reg1, 0x01);
	QBEQ(RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_TxRxBytesDoneCtr, scratch_reg1);

	// check if time-out is enabled, if yes increament the timeout counter and check if count is equal to MAX_RX_TIMEOUT_TRIES
	// if yes raise the interrupt for time out.
	ADD(rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr, 1);
	SBBO(&rx_context.rx_timeout_cntr, suart_ch_info.rx_context_addr, SUART_CH_RX_TIMEOUT_CNTR_OFFSET, $sizeof(rx_context.rx_timeout_cntr));
	QBGE(RxInterruptServiceRequestHndlr, rx_context.rx_timeout_cntr, MAX_RX_TIMEOUT_TRIES);
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_TIMEOUT_BIT);
	CLR(suart_ch_regs.Chn_Config1, suart_ch_regs.Chn_Config1, RX_TIMEOUT_INTR_MASK);
	SBBO(&suart_ch_regs.Chn_Config1, suart_ch_info.curr_ch_base_addr, SUART_CH_CONFIG1_OFFSET, $sizeof(suart_ch_regs.Chn_Config1));

	// Clear the RX timeout counter
	XOR(rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr);
	SBBO(&rx_context.rx_timeout_cntr, suart_ch_info.rx_context_addr, SUART_CH_RX_TIMEOUT_CNTR_OFFSET, $sizeof(rx_context.rx_timeout_cntr));
	JMP(RX_CHN_INTR);

	// Calculate the sampling bit position based on the start bit position
	// center = oversampling / 2
	// sampling bit position = start bit possition - center

	LABEL(START_BIT_TRANSITION);
	// clear the rx time out counter
	XOR(rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr);
	SBBO(&rx_context.rx_timeout_cntr, suart_ch_info.rx_context_addr, SUART_CH_RX_TIMEOUT_CNTR_OFFSET, $sizeof(rx_context.rx_timeout_cntr));

	// determine the over-sampling rate
	LSR(scratch_reg2, suart_ch_regs.Chn_Config1, SUART_CH_CONFIG1_OVS_BIT_SHIFT);
	AND(scratch_reg2, scratch_reg2, SUART_CH_CONFIG1_OVS_BIT_MASK);

	// OVER_SAMPLE
	QBEQ(OVER_SAMPLE_SIZE8BIT, scratch_reg2, OVER_SAMPLING_NONE);
	QBEQ(OVER_SAMPLE_SIZE8BIT, scratch_reg2, OVER_SAMPLING_8BIT);
	QBEQ(OVER_SAMPLE_SIZE16BIT, scratch_reg2, OVER_SAMPLING_16BIT);

	// Calaulate sampling bit position for 8 bit over sampling
	LABEL(OVER_SAMPLE_SIZE8BIT);
	// start bit possition - center
	SUB(rx_context.sampling_bit_pos, scratch_reg4, OVR_SAMPL_8BIT_MID_DATA_BIT);
	// sampling point
	AND(rx_context.sampling_bit_pos, rx_context.sampling_bit_pos, SAMPING_MASK_8_BIT_OVRSAMPLNG);
	SBBO(&rx_context.sampling_bit_pos, suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, $sizeof(rx_context.sampling_bit_pos));
	// if Start bit position is eqaul to/greater than centre, sample the start bit in current read, otherwise in next read
	QBLE(READ_CURRENT, scratch_reg4, OVR_SAMPL_8BIT_MID_DATA_BIT);
	JMP(RxInterruptServiceRequestHndlr);

	// Calaulate sampling bit position for 16 bit over sampling
	LABEL(OVER_SAMPLE_SIZE16BIT);
	// start bit possition - center
	SUB(rx_context.sampling_bit_pos, scratch_reg4, OVR_SAMPL_16BIT_MID_DATA_BIT);
	// samplimg point
	AND(rx_context.sampling_bit_pos, rx_context.sampling_bit_pos, SAMPING_MASK_16_BIT_OVRSAMPLNG);
	SBBO(&rx_context.sampling_bit_pos, suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, $sizeof(rx_context.sampling_bit_pos));
	// if Start bit position is eqaul to/greater than centre, sample the start bit in current read, otherwise in next read
	QBLE(READ_CURRENT, scratch_reg4, OVR_SAMPL_16BIT_MID_DATA_BIT);
	JMP(RxInterruptServiceRequestHndlr);

	LABEL(READ_CURRENT);
	// scratch_8bit_reg2 holds the information if bit detected is zero if scratch_8bit_reg2= 0, or one if scratch_8bit_reg2 = 1
	XOR(scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2);
	// if bit at sampling point is zero jump to READ_ZERO
	QBBC(READ_ZERO, scratch_reg1, rx_context.sampling_bit_pos);
	// otherwise increament scratch_8bit_reg2 by one as bit detected is one
	ADD(scratch_8bit_reg2, scratch_8bit_reg2, 1);

	LABEL(READ_ZERO);
	// We have read the data bit here...
	// If start bit is being received already, then skip the start condition processing.
	QBLT(RX_BIT_RECVD, suart_ch_regs.Chn_TxRxBitsDoneCtr, 0);

	//(Chn_TxRxBitsDoneCtr == 0)            //No bit is being Recieved, check if it is start bit
	// if DataBit == 0, i.e. scratch_8bit_reg2 == 0, Jump to Start Condition, else error fall through
	QBEQ(START_CONDITION, scratch_8bit_reg2, 0);

	QBEQ(START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0);

	// Broken start condition or false alarm, Reset repeat counter		//if DataBit == 1, instead of zero
	XOR(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));
	JMP(RxInterruptServiceRequestHndlr);

	// else part for NO_REPEAT_DONE  DataBit == 0
	LABEL(START_CONDITION);
	// Increament Repeat Done Counter by One, write back to memory
	ADD(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	// Read Pre Scaler
	LDI(scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK);
	AND(scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1);

	// if Repeat Done count is greater than or equal to prescaler, start bit is received, jump to START_BIT_RECIVED,
	QBGE(START_BIT_RECIVED, scratch_reg2, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	JMP(RxInterruptServiceRequestHndlr);

	// Start bit is condition Detected properly
	LABEL(START_BIT_RECIVED);
	// Increament Bit Count by One, and write it to memory
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// Reset Repeat Counter, and write it to memory
	XOR(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));
	JMP(RxInterruptServiceRequestHndlr);

	// Start Bit has been detected Already, Now the data bit is being received
	LABEL(RX_BIT_RECVD);
	// Now scratch_reg1 holds the info whether the data bit in scratch_8bit_reg2, is zero or one
	XOR(scratch_reg1, scratch_reg1, scratch_reg1);
	// if scratch_8bit_reg2 = 0, i.e data bit is Zero Jump to RX_DATA_BIT_ZERO
	// else Data bit is one fall through, data bit is ONE
	QBEQ(RX_DATA_BIT_ZERO, scratch_8bit_reg2, 0);
	// bit received is one, scratch_reg1 = 1
	OR(scratch_reg1, scratch_reg1, 0x1);

	LABEL(RX_DATA_BIT_ZERO);
	// if (Chn_TxRxRepeatDoneCntr < 32), check if reapeat done counter is less than 32, if yes Jump to RX_REPEAT_DONE_CNTR_LT_32
	QBGE(RX_REPEAT_DONE_CNTR_LT_32, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0x20);

	// repeat done counter is Greater than 32, Read Chn_RxDataBitsHoldRegHigh reg, Copy the Received bit to Chn_RxDataBitsHoldRegHigh register
	// else part : (Chn_TxRxRepeatDoneCntr is Greater than or equal to 32 )
	LABEL(RX_REPEAT_DONE_CNTR_GT_32);
	// Calculate the offset for bit in Chn_RxDataBitsHoldRegHigh regsiter
	RSB(scratch_reg2, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0x20);
	// Shift Received bit by above calculated of set i.e Chn_TxRxRepeatDoneCntr - 20
	LSL(scratch_reg1, scratch_reg1, scratch_reg2);
	LBBO(&rx_context.Chn_RxDataBitsHoldRegHigh, suart_ch_info.rx_context_addr, SUART_CH_RXDATABITSHOLDREGHIGH_OFFSET, $sizeof(rx_context.Chn_RxDataBitsHoldRegHigh));
	OR(rx_context.Chn_RxDataBitsHoldRegHigh, scratch_reg1, rx_context.Chn_RxDataBitsHoldRegHigh);
	SBBO(&rx_context.Chn_RxDataBitsHoldRegHigh, suart_ch_info.rx_context_addr, SUART_CH_RXDATABITSHOLDREGHIGH_OFFSET, $sizeof(rx_context.Chn_RxDataBitsHoldRegHigh));
	JMP(RX_REPEAT_COUNT_LESS_THAN_PRESCALR);

	// repeat done counter is less than OR equal to 32, Read the Chn_RxDataBitsHoldRegLow, Copy the Received bit to Chn_RxDataBitsHoldRegLow register
	// write it back to memory
	// if for (Chn_TxRxRepeatDoneCntr < 32)
	LABEL(RX_REPEAT_DONE_CNTR_LT_32);
	// Shift Received bit by Repeat Done Counter
	LSL(scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	LBBO(&rx_context.Chn_RxDataBitsHoldRegLow, suart_ch_info.rx_context_addr, SUART_CH_RXDATABITSHOLDREGLOW_OFFSET, $sizeof(rx_context.Chn_RxDataBitsHoldRegLow));
	OR(rx_context.Chn_RxDataBitsHoldRegLow, scratch_reg1, rx_context.Chn_RxDataBitsHoldRegLow);
	SBBO(&rx_context.Chn_RxDataBitsHoldRegLow, suart_ch_info.rx_context_addr, SUART_CH_RXDATABITSHOLDREGLOW_OFFSET, $sizeof(rx_context.Chn_RxDataBitsHoldRegLow));

	// Increament Chn_TxRxRepeatDoneCntr by one and Check if Repeat Done Counter is equal to Prescalar,
	// if yes jump to PROCESS_RX_DATA_BIT, otherewise again sample RBuf for same bit
	LABEL(RX_REPEAT_COUNT_LESS_THAN_PRESCALR);
	ADD(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	// Read Pre Scaler
	LDI(scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK);
	AND(scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1);

	// check if number of bits sampled (Chn_TxRxRepeatDoneCtr) is equal to prescaler (scratch_reg2), if yes jump to PROCESS_RX_DATA_BIT
	QBGE(PROCESS_RX_DATA_BIT, scratch_reg2, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	JMP(RxInterruptServiceRequestHndlr);

	// Scan Chn_RxDataBitsHoldRegLow, Chn_RxDataBitsHoldRegHigh, to check if BIT received is one or zero and write to Chn_RxDataHoldReg
	// (Chn_TxRxRepeatDoneCntr >= Chn_Config1.PreScaller) if part
	LABEL(PROCESS_RX_DATA_BIT);
	// Get the Presaclar
	LDI(scratch_reg3, SUART_CTRL_PRE_SCALAR_MASK);
	// scratch_reg3 hold prescalar
	AND(scratch_reg3, scratch_reg3, suart_ch_regs.Chn_Config1);

	// Initialize the register to zero required for copying data bit received in rxdata_buf
	// keep count of number of ONE scanned
	XOR(scratch_8bit_reg1, scratch_8bit_reg1, scratch_8bit_reg1);
	// keep count of number of ZERO scanned
	XOR(scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2);

	// used to store count of number of bits scanned in Chn_RxDataBitsHoldRegLow, & Chn_RxDataBitsHoldRegHigh
	XOR(scratch_reg2, scratch_reg2, scratch_reg2);

	// points to location taken as start point in Chn_RxDataBitsHoldRegLow for scannig bit received
	XOR(scratch_reg1, scratch_reg1, scratch_reg1);

	// scratch_reg4 holds the data from Chn_RxDataBitsHoldRegLow
	LBBO(&scratch_reg4, suart_ch_info.rx_context_addr, SUART_CH_RXDATABITSHOLDREGLOW_OFFSET, 4);
	// if Pre Scalar is less than or equal to 32, JMP to BIT_CHK_LOOP
	QBGE(BIT_CHK_LOOP, scratch_reg3, 0x20);

	// pre scalar is greater 32, check if it is greater that 48 then set scratch_reg3 = 48, scan bit upto this count only.
	LABEL(PRE_SCALR_GT_32);
	// start checking bit from bit position 0x10
	OR(scratch_reg1, scratch_reg1, 0x10);
	// if Pre Scalar is less than 48
	QBGT(BIT_CHK_LOOP, scratch_reg3, 0x30);

	// pre scalar is greater 48, set scratch_reg3 = 48, scan bit upto this count only.
	LABEL(PRE_SCALR_GT_48);
	LDI(scratch_reg3, 0x30);

	// Scan the Chn_RxDataBitsHoldRegLow, and Chn_RxDataBitsHoldRegHigh registers to know received bit is ZERO or ONE
	LABEL(BIT_CHK_LOOP);
	// if bit is cleared, Jump to BIT_RECVD_ZERO
	QBBC(BIT_RECVD_ZERO, scratch_reg4, scratch_reg1);
	// else BIT prerscaled is one
	ADD(scratch_8bit_reg1, scratch_8bit_reg1, 1);
	// Increament scratch_reg1 by one so that it points to next bit to scanned
	ADD(scratch_reg1, scratch_reg1, 1);
	// Increament scratch_reg2 holding bits scanned count
	ADD(scratch_reg2, scratch_reg2, 1);
	// if Prescaler is greater than 32, and scratch_reg2 is equal to 32, load Chn_RxDataBitsHoldRegHigh in scratch_reg4
	QBLT(LOAD_RXDATABITS_HOLDREGHIGH, scratch_reg2, 0x20);
	// scan untill all the bits are scanned
	QBGT(BIT_CHK_LOOP, scratch_reg2, scratch_reg3);
	JMP(COPY_BIT_RECVD);

	// Load the Chn_RxDataBitsHoldRegHigh to scratch_reg4
	LABEL(LOAD_RXDATABITS_HOLDREGHIGH);
	LBBO(&scratch_reg4, suart_ch_info.rx_context_addr, SUART_CH_RXDATABITSHOLDREGHIGH_OFFSET, 4);
	// Reset the scratch_reg1, so that starts from bit 0 for Chn_RxDataBitsHoldRegHigh
	XOR(scratch_reg1, scratch_reg1, scratch_reg1);
	// Reset the scratch_reg2, so that only jump to label LOAD_RXDATABITS_HOLDREGHIGH done one's only
	XOR(scratch_reg2, scratch_reg2, scratch_reg2);
	// Decreament Total loop count by 32, since it has been already checked in Chn_RxDataBitsHoldRegLow
	SUB(scratch_reg3, scratch_reg3, 0x20);
	JMP(BIT_CHK_LOOP);

	// Current sacnned Bit in Chn_RxDataBitsHoldRegHigh or Chn_RxDataBitsHoldRegLow is zero
	LABEL(BIT_RECVD_ZERO);
	// for Zero
	ADD(scratch_8bit_reg2, scratch_8bit_reg2, 1);
	ADD(scratch_reg1, scratch_reg1, 1);
	ADD(scratch_reg2, scratch_reg2, 1);
	QBGT(BIT_CHK_LOOP, scratch_reg2, scratch_reg3);

	// Copy the Received bit to Chn_RxDataHoldReg, scratch_reg1, now store the info if bit received is zero or one
	LABEL(COPY_BIT_RECVD);

	// scratch_8bit_reg1= Bit is ONE, scratch_8bit_reg2 = Bit is Zero, if scratch_8bit_reg2 > scratch_8bit_reg1,
	// jump to WRITE_RCVD_BIT_TO_RX_DATAHOLDREG as data bit is ZERO
	XOR(scratch_reg1, scratch_reg1, scratch_reg1);
	QBGE(WRITE_RCVD_BIT_TO_RX_DATAHOLDREG, scratch_8bit_reg1, scratch_8bit_reg2);
	//Bit Received is One, write to Chn_RxDataHoldReg
	OR(scratch_reg1, scratch_reg1, 0x1);

	// Write the Received Data bit (in scratch_reg1) to Chn_RxDataHoldReg
	LABEL(WRITE_RCVD_BIT_TO_RX_DATAHOLDREG);
	// Shift the bit received by Chn_TxRxBitsDoneCtr
	LSL(scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxBitsDoneCtr);

	// Read the Chn_RxDataHoldReg from Memory
	LBBO(&rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, $sizeof(rx_context.Chn_RxDataHoldReg));

	// Write the bit received to Chn_RxDataHoldReg
	OR(rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, scratch_reg1);

	// Write updated Chn_RxDataHoldReg to memory
	SBBO(&rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, $sizeof(rx_context.Chn_RxDataHoldReg));

	// Increment the Data bit Counter
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// Reset the Repeat Done Counter
	XOR(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	// initialize Chn_RxDataBitsHoldRegLow
	XOR(rx_context.Chn_RxDataBitsHoldRegLow, rx_context.Chn_RxDataBitsHoldRegLow, rx_context.Chn_RxDataBitsHoldRegLow);
	SBBO(&rx_context.Chn_RxDataBitsHoldRegLow, suart_ch_info.rx_context_addr, SUART_CH_RXDATABITSHOLDREGLOW_OFFSET, $sizeof(rx_context.Chn_RxDataBitsHoldRegLow));

	// initialize Chn_RxDataBitsHoldRegHigh
	XOR(rx_context.Chn_RxDataBitsHoldRegHigh, rx_context.Chn_RxDataBitsHoldRegHigh, rx_context.Chn_RxDataBitsHoldRegHigh);
	SBBO(&rx_context.Chn_RxDataBitsHoldRegHigh, suart_ch_info.rx_context_addr, SUART_CH_RXDATABITSHOLDREGHIGH_OFFSET, $sizeof(rx_context.Chn_RxDataBitsHoldRegHigh));

	// Read Bit Per Charater
	AND(scratch_reg2, suart_ch_regs.Chn_Config2.u16, SUART_CH_CONFIG2_BITS_PER_CHAR_MASK);

	// check is (N-1) bit is being received for current data frame, if yes jump to CHK_RECVD_DATA_FRAME
	// if all N bits has been Received Jump to RESET_BITS_CNTR, otherwise receive remaining bits.
	// (Chn_TxRxBitsDoneCntr >= Chn_Config2.BitsPerChar)
	QBGE(RESET_BITS_CNTR, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	SUB(scratch_reg2, scratch_reg2, 1);
	QBEQ(CHK_RECVD_DATA_FRAME, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	JMP(RxInterruptServiceRequestHndlr);

	// if all bits received, verify the Received data frame
	LABEL(CHK_RECVD_DATA_FRAME);
	// Zero the (16 - Chn_TxRxBitsDoneCntr) Most significant bits in the Chn_RxDataHoldReg.
	RSB(scratch_reg2, scratch_reg2, 16);
	// load the count to for number of zero to inserted
	ADD(scratch_reg2, scratch_reg2, 0x10);
	// Used to Insert  Zero in MSB
	NOT(scratch_reg1, pZERO);

	LABEL(REGHOLD_MSB_ZERO);
	// Prepare the MASK with  ZERO's in bits that do not corresponds to data bits
	LSR(scratch_reg1, scratch_reg1, scratch_reg2);

	// Read the Data hold Reg
	LBBO(&rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, $sizeof(rx_context.Chn_RxDataHoldReg));
	// Insert the ZERO's in bits that do  not  corresponds to Data Bits
	AND(rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, scratch_reg1);
	SBBO(&rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, $sizeof(rx_context.Chn_RxDataHoldReg));

	// removing start bit
	LSR(rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, 1);

	// load the arm memory lacation address where data is to be written
	LBBO(&suart_ch_regs.ch_TxRxData, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXDATA_OFFSET, $sizeof(suart_ch_regs.ch_TxRxData));

	// Read Bits Per Character
	AND(scratch_reg4, suart_ch_regs.Chn_Config2.u16, SUART_CH_CONFIG2_BITS_PER_CHAR_MASK);
	SUB(scratch_reg4, scratch_reg4, 2);

	// Load the Bytes Done counter
	MOV(scratch_reg1, suart_ch_regs.Chn_TxRxBytesDoneCtr);

	// check, if two byte offset is required (bits per character greater than 8)
	QBGE(WRITE_RX_CHAR_TO_MEM, scratch_reg4, SUART_CH_CONFIG2_8BITS_PER_CHAR);

	// calculate the offset in memory where received character is to be written
	LSL(scratch_reg1, scratch_reg1, 1);

	LABEL(WRITE_RX_CHAR_TO_MEM);
	// Write the actual data to ARM Memory
	SBBO(&rx_context.Chn_RxDataHoldReg, suart_ch_regs.ch_TxRxData, scratch_reg1, $sizeof(rx_context.Chn_RxDataHoldReg));

	JMP(RxInterruptServiceRequestHndlr);

	LABEL(RESET_BITS_CNTR);
	// Check for Framing Error Framing Error
	SUB(scratch_reg3, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);

	// Reset bits done counter
	XOR(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	// Get the Prescalar
	LDI(scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK);
	//scratch_reg2 hold prescalar
	AND(scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1);

	// Extract timing information by detecting start transition (first left most zero)
	LMBD(scratch_reg4, mcasp_rbuf_val, 0);
	// branch if zero start bit transition detected
	QBEQ(INVALID_SAMPLING_PNT, scratch_reg4, ZERO_BIT_NOT_DETECTED);

	// determine the over-sampling rate
	LSR(scratch_reg1, suart_ch_regs.Chn_Config1, SUART_CH_CONFIG1_OVS_BIT_SHIFT);
	// OVER_SAMPLE
	AND(scratch_reg1, scratch_reg1, SUART_CH_CONFIG1_OVS_BIT_MASK);
	// 16 bit over sampling
	QBEQ(NXT_FRAME_SAMPLING_16BIT_OVS, scratch_reg1, OVER_SAMPLING_16BIT);

	// Calaulate sampling bit position for 8 bit over sampling
	LABEL(NXT_FRAME_SAMPLING_8BIT_OVS);
	// to correct  bit timing error used 4
	QBLT(INVALID_SAMPLING_PNT, scratch_reg4, 4);

	LABEL(CAL_SAMPL_PNT8);
	// start bit position - center
	SUB(rx_context.sampling_bit_pos, scratch_reg4, OVR_SAMPL_8BIT_MID_DATA_BIT);
	// sampling point
	AND(rx_context.sampling_bit_pos, rx_context.sampling_bit_pos, SAMPING_MASK_8_BIT_OVRSAMPLNG);
	QBGT(UPDATE_SAMPLING_PNT, scratch_reg4, 4);
	JMP(NXT_FRAME_SAMPLING_PNT);

	// Calaulate sampling bit position for 16 bit over sampling
	LABEL(NXT_FRAME_SAMPLING_16BIT_OVS);
	// to correct  bit timing error used 4
	QBLT(INVALID_SAMPLING_PNT, scratch_reg4, 8);

	LABEL(CAL_SAMPL_PNT16);
	// start bit position - center
	SUB(rx_context.sampling_bit_pos, scratch_reg4, OVR_SAMPL_16BIT_MID_DATA_BIT);
	// sampling point
	AND(rx_context.sampling_bit_pos, rx_context.sampling_bit_pos, SAMPING_MASK_16_BIT_OVRSAMPLNG);
	QBGT(UPDATE_SAMPLING_PNT, scratch_reg4, 8);

	LABEL(NXT_FRAME_SAMPLING_PNT);
	// Increament Repeat Done Counter by One, write back to memory
	ADD(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));

	// Read Pre Scaler
	LDI(scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK);
	AND(scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1);

	// if Repeat Done count is greater than or equal to prescaler, start bit is received, jump to START_BIT
	QBLT(UPDATE_SAMPLING_PNT, scratch_reg2, suart_ch_regs.Chn_TxRxRepeatDoneCtr);

	// Start bit is condition Detected properly
	LABEL(START_BIT);
	// Increament Bit Count by One, and write it to memory
	ADD(suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBitsDoneCtr));

	XOR(suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxRepeatDoneCtr));
	JMP(UPDATE_SAMPLING_PNT);

	LABEL(INVALID_SAMPLING_PNT);
	// Reset the Sampling Point
	LDI(rx_context.sampling_bit_pos, 0xff);

	LABEL(UPDATE_SAMPLING_PNT);
	SBBO(&rx_context.sampling_bit_pos, suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, $sizeof(rx_context.sampling_bit_pos));

	// read interrupt mask regsiter
	LBBO(&suart_global.intrMask, pZERO, SUART_GBL_INT_MASK_ADDR, $sizeof(suart_global.intrMask));

	//read interrupt status regsiter
	LBBO(&suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, $sizeof(suart_global.intrStatus));

	// check for error in received data frame
	// Check for Break Condiotion Error
	QBGE(RX_DATA_ZERO, rx_context.Chn_RxDataHoldReg, 0);

	// Framing Error: Check if the Bit at Chn_TxRxBitsDoneCtr Bit Position in the Chn_RxDataHoldReg is set
	QBBC(BIT_CLEARD, rx_context.Chn_RxDataHoldReg, scratch_reg3);

	// increament Chn_TxRxBytesDoneCtr by one
	ADD(suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1);
	SBBO(&suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBytesDoneCtr));

	// Reset the Data Hold Reg
	XOR(rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg);
	SBBO(&rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, $sizeof(rx_context.Chn_RxDataHoldReg));

	// Read the request count to be received
	LSR(scratch_reg1, suart_ch_regs.Chn_Config2.u16, SUART_CH_CONFIG2_DATALEN_SHIFT);
	AND(scratch_reg1, scratch_reg1, SUART_CH_CONFIG2_DATALEN_MASK);
	ADD(scratch_reg1, scratch_reg1, 0x01);

	// Read the bytes done counter
	MOV(scratch_reg2, suart_ch_regs.Chn_TxRxBytesDoneCtr);

	// check if bytes done counter is less than or equal to data len,
	// if yes go to CHK_RX_CMPL_INT and check for raise RX complete intr
	QBGE(CHK_RX_CMPL_INT, suart_ch_regs.Chn_TxRxBytesDoneCtr, scratch_reg1);

	// if bytes done counter is greater than data len subtract data len from it and
	// check if differnce is data len, if yes raise RX complete intr
	SUB(scratch_reg2, suart_ch_regs.Chn_TxRxBytesDoneCtr, scratch_reg1);

	LABEL(CHK_RX_CMPL_INT);
	// check if all data frame received or not, if RX request if complete, else receive next data frame
	QBLT(RxInterruptServiceRequestHndlr, scratch_reg1, scratch_reg2);

	// All requested frame received raise interrupt to ARM/DSP, set SUART_RX_FIFO_INDX_BIT, clear SUART_TXRX_READY_BIT
	LABEL(RX_COMPLETE);
	// RX Data is in lower half of Fifo, if bytes done counter is equal to data len
	CLR(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_FIFO_INDX_BIT);

	// Raise the RX interrupt if Chn_TxRxBytesDoneCtr is equal to data len otherwise reset Chn_TxRxBytesDoneCtr and raise Rx interrupt
	QBEQ(CHK_RX_OVERRUN, suart_ch_regs.Chn_TxRxBytesDoneCtr, scratch_reg1);

	// reset Chn_TxRxBytesDoneCtr if Chn_TxRxBytesDoneCtr is equal to twice the data len
	XOR(suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr);
	SBBO(&suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxBytesDoneCtr));

	// RX data is in upper half of Fifo,if bytes done counter is equal to twice the data len
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_FIFO_INDX_BIT);

	LABEL(CHK_RX_OVERRUN);
	MOV32(scratch_reg2, SECR2_OFFSET);
	LBCO(&scratch_reg1, CONST_PRUSSINTC, scratch_reg2, 4);
	ADD(scratch_reg2, suart_ch_info.ch_num, 2);
	ADD(scratch_reg2, suart_ch_info.ch_num, 2);

	LABEL(OVER_RUN_ERR);
	QBBC(CHK_RX_READY_BIT, scratch_reg1, scratch_reg2);
	QBBC(CHK_RX_READY_BIT, suart_ch_regs.Chn_Config1, RX_OVER_RUN_MASK);
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_RUNER_BIT);
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_ERROR_BIT);

	LABEL(CHK_RX_READY_BIT);
	// If the receive is not activated from the host, then don't dump the data
	QBBC(RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT);
	JMP(RX_CHN_INTR);

	// Framing Error Detected, interrupt are masked go to DEACTIVATE_SERIALIZER, other wise update status reg
	LABEL(BIT_CLEARD);
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_FE_BIT);
	JMP(SET_RX_ERR_STAT);

	// Break Condiotion Error detected, interrupt are masked go to DEACTIVATE_SERIALIZER, other wise update status reg
	LABEL(RX_DATA_ZERO);
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_BI_BIT);

	// Update the Global and Channel Error Status Registers
	LABEL(SET_RX_ERR_STAT);
	SET(suart_global.intrStatus, suart_global.intrStatus, GLOBAL_ERR_INTR);
	SET(suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_ERROR_BIT);

	// if global Error interrupt is clear do not raise interrupt
	QBBC(RxInterruptServiceRequestHndlr, suart_global.intrMask, GLOBAL_ERR_INTR);

	// if Framing error status bit for channel is clear look for Break Condiotion Error
	QBBC(BREAK_COND_ERR, suart_ch_regs.Chn_TxRxStatus, SUART_RX_FE_BIT);

	// Framming Error Occurred, if Framing Error mask is not set jum to RxInterruptServiceRequestHndlr
	LABEL(FRAMING_ERR);
	QBBS(RX_CHN_INTR, suart_ch_regs.Chn_Config1, FE_ERR_INTR_MASK);
	JMP(RxInterruptServiceRequestHndlr);

	// if Break Error Mask not set jump to RxInterruptServiceRequestHndlr
	LABEL(BREAK_COND_ERR);
	QBBC(RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_Config1, BI_ERR_INTR_MASK);

	// Set the Global interrupt status register
	LABEL(RX_CHN_INTR);
	SET(suart_global.intrStatus, suart_global.intrStatus, suart_ch_info.ch_num);

	// write interrupt status regsiter
	SBBO(&suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, $sizeof(suart_global.intrStatus));

	// Update tx rx status regsiter status
	SBBO(&suart_ch_regs.Chn_TxRxStatus, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXSTATUS_OFFSET, $sizeof(suart_ch_regs.Chn_TxRxStatus));

	// if interrupt are masked for channel then go to RxInterruptServiceRequestHndlr, otherwise raise the interrupt
	QBBC(RxInterruptServiceRequestHndlr, suart_global.intrMask, suart_ch_info.ch_num);

	// Raise the interrupt to ARM/DSP
	JMP(PRU_TO_HOST_INTERRUPT);

//******************************************** RxInterruptServiceRequestHndlr: ENDs **************************

//========================================================================================================================================

//******************************************** LOAD_RX_CONTEXT_ADDRESS: Start*********************************

	LABEL(LOAD_RX_CONTEXT_ADDRESS);
	QBEQ(RX_CONTEXT_CH1_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_1);
	QBEQ(RX_CONTEXT_CH3_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_3);
	QBEQ(RX_CONTEXT_CH5_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_5);
	QBEQ(RX_CONTEXT_CH7_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_7);
	JMP(JMP_CALL_reg);

	LABEL(RX_CONTEXT_CH1_ADDR);
	LDI(suart_ch_info.rx_context_addr, SUART_CH1_RX_CONTEXT_ADDR);
	JMP(JMP_CALL_reg);

	LABEL(RX_CONTEXT_CH3_ADDR);
	LDI(suart_ch_info.rx_context_addr, SUART_CH3_RX_CONTEXT_ADDR);
	JMP(JMP_CALL_reg);

	LABEL(RX_CONTEXT_CH5_ADDR);
	LDI(suart_ch_info.rx_context_addr, SUART_CH5_RX_CONTEXT_ADDR);
	JMP(JMP_CALL_reg);

	LABEL(RX_CONTEXT_CH7_ADDR);
	LDI(suart_ch_info.rx_context_addr, SUART_CH7_RX_CONTEXT_ADDR);
	JMP(JMP_CALL_reg);

//******************************************** LOAD_RX_CONTEXT_ADDRESS Ends ***********************************

//=====================================================================================================================================

//			************************************************************************************************
//									SOFT-UART RECEIVE ROUTINE : ENDS
//			************************************************************************************************

//=====================================================================================================================================

}
