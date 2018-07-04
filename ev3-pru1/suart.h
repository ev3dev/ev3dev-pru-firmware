//
// Based on: suart_pru_emu.hp
//  (C) Copyright 2010, Texas Instruments, Inc


//===========================================================================
//	SUART MEMORY MAP FOR SINGLE PRU EMULATING 4 SOFT UART
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
//						START_ADDRESS	END_ADDRESS	SIZE
//----------------------------------------------------------------------------------
//CH0 @ (TX)
//		Formated data saved at		0x090		0x0AF		32 Bytes
//		Context information saved at	0x0B0		0x0BF		16 Bytes
//CH1 @ (RX)
//		Context information saved at	0x0C0		0x0DF		32 Bytes
//CH2 @ (TX)
//		Formated data saved at		0x0E0		0x0FF		32 Bytes
//		Context information saved at	0x100		0x10F		16 Bytes
//CH3 @ (RX)
//		Context information saved at	0x110		0x12F		32 Bytes
//CH4 @ (TX)
//		Formated data saved at		0x130		0x14F		32 Bytes
//		Context information saved at	0x150		0x15F		16 Bytes
//CH5 @ (RX)
//		Context information saved at	0x160		0x17F		32 Bytes
//CH6 @ (TX)
//		Formated data saved at		0x180		0x19F		32 Bytes
//		Context information saved at	0x1A0		0x1AF		16 Bytes
//CH7 @ (RX)
//		Context information saved at	0x1B0		0x1CF		32 Bytes
//
// C A U T I O N:
// This context information should be flush after completion of service request

#ifndef _PRU_SUART_H_
#define _PRU_SUART_H_

//===========================================================================
// *     Global Register Assignments     *
//===========================================================================
#define INTC_REGS_BASE      		0x00004000
#define GER_OFFSET			0x10
#define HIESR_OFFSET			0x34
#define SISR_OFFSET         		0x20
#define SICR_OFFSET			0x24
#define EISR_OFFSET			0x28
#define SRSR1_OFFSET                    0x200
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

//McASP SRCTL registers
#define ASP_SRCTL_XRDY_BIT		4
#define ASP_SRCTL_RRDY_BIT		5
#define SUART_CTRL_SRCTL_BIT_SHIFT 	2

//===========================================================================
//SUART Channel Registers
//===========================================================================
/* TODO: rename to MAX_CHANNEL or fix conditional expressions */
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

//Channel control stop bits
#define SUART_CTRL_STOPBIT_MASK                 0x1
#define SUART_CTRL_STOPBIT_SHIFT                0x7

//--------SUART Channel Control register descriptions------------
#define	SUART_CH_CONFIG1_OVS_BIT_SHIFT		0xA
#define	SUART_CH_CONFIG1_OVS_BIT_MASK		0x3
#define SUART_CH_CONFIG2_RX_PARITY_VAL_SHIFT	0xC
#define SUART_CH_CONFIG2_RX_PARITY_VAL_MASK     0xF
#define	SUART_CH_CONFIG2_DATALEN_SHIFT		0x8
#define	SUART_CH_CONFIG2_DATALEN_MASK		0x0F
#define	OVER_SAMPLING_NONE			0x0
#define	OVER_SAMPLING_8BIT			0x1
#define	OVER_SAMPLING_16BIT			0x2
#define OVR_SAMPL_8BIT_MID_DATA_BIT		0x4
#define OVR_SAMPL_16BIT_MID_DATA_BIT		0x8

#define SUART_CH_CONFIG2_PARITY_SHIFT           0x6
#define SUART_CH_CONFIG2_PARITY_MASK            0x3

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
#define SUART_RX_PE_BIT                         7
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
#define  RX_OVER_RUN_MASK                       15
#define  PE_ERR_INTR_MASK                       0

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

//===========================================================================
//		SUART GLOBAL CHANNEL STRUCTURE
//===========================================================================

typedef struct {
	uint16_t intrMask;
	uint16_t intrStatus;
	uint8_t pru_id;
	uint8_t pru_rx_tx_mode;
	uint8_t pru_delay_cnt;
	uint8_t reserved;
} Suart_Global;


//===========================================================================
//		STRUCTURE TO SUART CHANNEL SPECIFIC REGISTER
//===========================================================================

typedef struct {
	uint16_t Chn_Cntrl;
	uint16_t Chn_Config1;
	uint16_t Chn_Config2;
	uint8_t  Chn_TxRxStatus;
	uint8_t  Chn_Status;
	uint32_t ch_TxRxData;
	uint8_t  Chn_TxRxBytesDoneCtr;
	uint8_t  Chn_TxRxBitsDoneCtr;
	uint16_t Chn_TxRxRepeatDoneCtr;
} Suart_Ch_Struct;

//===========================================================================
//		CHANNEL INFORMATION STRUCTURE
//===========================================================================

typedef struct {
	uint32_t curr_ch_base_addr;
	uint32_t rx_context_addr;
	uint16_t curr_ch_offset;
	uint8_t  ch_num;
} Suart_Ch_Info;

//===========================================================================
//		TX CONTEXT STRUCTURE
//===========================================================================

typedef struct {
	uint32_t asp_xsrctl_reg;		//NG: avoid recalculation, save cycles ;-)
	uint32_t asp_xbuf_reg;		//NG: avoid recalculation, save cycles ;-)
	uint16_t buff_addr;		 //Formatted data base address: data RAM address
	uint8_t  buff_size;		//Number of data format in the data RAM
	uint8_t  bitsLoaded;
} Suart_Tx_Context;

//===========================================================================
//		RX CONTEXT STRUCTURE
//===========================================================================

typedef struct {
	uint32_t asp_rbuf_reg;
	uint32_t asp_rsrctl_reg;
	uint8_t  reserved0;
	uint8_t  reserved1;     //rxdata_buf		//to store the extracted data
	uint16_t Chn_RxDataHoldReg;
	uint32_t Chn_RxDataBitsHoldRegLow;
	uint32_t Chn_RxDataBitsHoldRegHigh;
	uint16_t rx_timeout_cntr;
	uint8_t  sampling_bit_pos;
	uint8_t  false_start_flag;
} Suart_Rx_Context;


#define ARM_TO_PRU0_INT		30
#define ARM_TO_PRU1_INT		31
#define MCASP_TXRX_EVENT       	31
#define PRU0_TO_PRU1_EVENT     	50

#endif // _PRU_SUART_H_
