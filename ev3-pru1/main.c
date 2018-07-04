/*
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>

#include <pru_rpmsg.h>
#include <pru_virtio_config.h>

#include <am18xx/pru_ctrl.h>
#include <am18xx/pru_intc.h>
#include <am18xx/sys_gpio.h>
#include <am18xx/sys_mcasp.h>
#include <am18xx/sys_timer.h>

#include "suart.h"
#include "resource_table.h"


// system events to/from ARM

// PRU0
//#define HOST_INT	(uint32_t)(1U << 30)	// host interrupt 0
//#define EVENT_FROM_ARM	32
//#define EVENT_TO_ARM	33

// PRU1
#define HOST_INT	(uint32_t)(1U << 31)	// host interrupt 1
#define EVENT_FROM_ARM	34
#define EVENT_TO_ARM	35

volatile uint32_t register __R31;

// Based on: suart_pru_emu.p
// (C) Copyright 2010, Texas Instruments, Inc

// .origin 0
// .entrypoint SUART_EMULATION

//#include "PRU_SUART_Emulation.hp"


//======================================================================================================================================

//****************************************** TX LOCATE_SR_XBUF_SRCTL : Starts ********************************

//======================================================================================================================================
// 		Subroutine to find channel specific serializer, xbuf, srctl register mapped active channel
//======================================================================================================================================
//LOCATE_SR_XBUF_SRCTL:
void locate_sr_xbuf_srctl(Suart_Ch_Struct *suart_ch_regs, Suart_Ch_Struct *suart_tx_ch, Suart_Tx_Context *tx_context)
{
	uint32_t scratch_reg1;
	uint32_t scratch_reg2;
	uint32_t scratch_reg3;
	uint32_t scratch_reg4;

    	// Calculating Serializer Mapped to Channel
//	AND 	scratch_reg1, suart_ch_regs.Chn_Cntrl.b1, SUART_CTRL_SERIALIZER_MASK
//	LSL 	scratch_reg1, scratch_reg1, SUART_CTRL_SRCTL_BIT_SHIFT
	scratch_reg1 = ((uint8_t *)&suart_ch_regs->Chn_Cntrl)[1] & SUART_CTRL_SERIALIZER_MASK;
	scratch_reg1 <<= SUART_CTRL_SRCTL_BIT_SHIFT;

	// copy the tx format address to temp register
//	MOV   	scratch_reg3, tx_context.buff_addr
//	MOV     scratch_reg4, TX_FMT_DATA_TO_TX_CONTEXT_OFFSET
	scratch_reg3 = tx_context->buff_addr;
	scratch_reg4 = TX_FMT_DATA_TO_TX_CONTEXT_OFFSET;

	//Calculating the specific SRCTL register offset .
//	MOV32  	tx_context.asp_xsrctl_reg, MCASP_SRCTL_BASE
//	ADD 	tx_context.asp_xsrctl_reg, tx_context.asp_xsrctl_reg, scratch_reg1
	tx_context->asp_xsrctl_reg = (uint32_t)&MCASP0.SRCTL0;
	tx_context->asp_xsrctl_reg += scratch_reg1;

//	ADD     scratch_reg2, scratch_reg4, SUART_CH_ASP_XSRCTL_REG_OFFSET
//	SBBO    tx_context.asp_xsrctl_reg, scratch_reg3, scratch_reg2, SIZE (tx_context.asp_xsrctl_reg)
	scratch_reg2 = scratch_reg4 + SUART_CH_ASP_XSRCTL_REG_OFFSET;
	*(uint32_t *)(scratch_reg3 + scratch_reg2) = tx_context->asp_xsrctl_reg;

	//Calculating the specific xbuf register offset .
//	MOV32	tx_context.asp_xbuf_reg, MCASP_XBUF_BASE
//	ADD	    tx_context.asp_xbuf_reg, tx_context.asp_xbuf_reg, scratch_reg1
	tx_context->asp_xsrctl_reg = (uint32_t)&MCASP0.XBUF0;
	tx_context->asp_xsrctl_reg += scratch_reg1;

//	ADD     scratch_reg2, scratch_reg4, SUART_CH_ASP_XBUF_REG_OFFSET
//	SBBO    tx_context.asp_xbuf_reg, scratch_reg3, scratch_reg2, SIZE (tx_context.asp_xbuf_reg)
	scratch_reg2 = scratch_reg4 + SUART_CH_ASP_XBUF_REG_OFFSET;
	*(uint32_t *)(scratch_reg3 + scratch_reg2) = tx_context->asp_xbuf_reg;

	//Store the data length
//	MOV 	tx_context.buff_size, suart_tx_ch.Chn_Config2.b1
	tx_context->buff_size = ((uint8_t *)&suart_tx_ch->Chn_Config2)[1];

//	ADD     scratch_reg2, scratch_reg4, SUART_CH_BUFF_SIZE_OFFSET
//	SBBO    tx_context.buff_size, scratch_reg3, scratch_reg2, SIZE (tx_context.buff_size)
	scratch_reg2 = scratch_reg4 + SUART_CH_BUFF_SIZE_OFFSET;
	*(uint8_t *)(scratch_reg3 + scratch_reg2) = tx_context->buff_size;

	//Store the data Tx FMT Context address
//	ADD     scratch_reg2, scratch_reg4, SUART_CH_BUFF_ADDR_OFFSET
//	SBBO    tx_context.buff_addr, scratch_reg3, scratch_reg2, SIZE (tx_context.buff_addr)
	scratch_reg2 = scratch_reg4 + SUART_CH_BUFF_ADDR_OFFSET;
	*(uint8_t *)(scratch_reg3 + scratch_reg2) = tx_context->buff_size;

//	LDI 	tx_context.bitsLoaded, 0x00
//	ADD     scratch_reg2, scratch_reg4, SUART_CH_BITSLOADED_OFFSET
//	SBBO    tx_context.bitsLoaded, scratch_reg3, scratch_reg2, SIZE (tx_context.bitsLoaded)
	tx_context->bitsLoaded = 0;
	scratch_reg2 = scratch_reg4 + SUART_CH_BITSLOADED_OFFSET;
	*(uint8_t *)(scratch_reg3 + scratch_reg2) = tx_context->bitsLoaded;

//	JMP	LOCATE_SR_XBUF_SRCTL_DONE
}
//********************************************** TX LOCATE_SR_XBUF_SRCTL: ENDS **************************************

//======================================================================================================================================

//********************************************** TX CHK_TX_DATA_FORMAT : Starts***************************************

//======================================================================================================================================
// 	Check For Data Formating, formats the data only if Chn_TxRxRepeatDoneCtr,
//	Chn_TxRxBitsDoneCtr, Chn_TxRxBytesDoneCtr all are zero,
//  If All the conditions is satisfied, it jumps to TX_DATA_FORMAT Subroutine  and
// 	formats data the TX Data obtained from ARM/DSP by adding start and stop bit.
//======================================================================================================================================
//CHK_TX_DATA_FORMAT:
void chk_tx_data_format(Suart_Ch_Struct *suart_ch_regs, Suart_Ch_Struct *suart_tx_ch, Suart_Tx_Context *tx_context, uint32_t *TX_DATA_reg)
{
	uint32_t scratch_reg1;
	uint32_t scratch_reg2;
	uint32_t scratch_reg3;
	uint32_t scratch_reg4;
	uint32_t scratch_reg5;
	uint32_t scratch_reg6;
	uint32_t scratch_reg7;
	uint8_t scratch_8bit_reg1;

//	QBEQ    CHK_TX_DATA_FORMAT_BITS, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0
	if (suart_ch_regs->Chn_TxRxRepeatDoneCtr) {
//	JMP		CHK_TX_DATA_FORMAT_DONE
		return;

//CHK_TX_DATA_FORMAT_BITS:
	}
//	QBEQ    CHK_TX_DATA_FORMAT_BYTE, suart_ch_regs.Chn_TxRxBitsDoneCtr, 0
	if (suart_ch_regs->Chn_TxRxBitsDoneCtr) {
//	JMP		CHK_TX_DATA_FORMAT_DONE
		return;

//CHK_TX_DATA_FORMAT_BYTE:
	}
//	QBEQ    TX_DATA_FORMAT, suart_ch_regs.Chn_TxRxBytesDoneCtr, 0
	if (suart_ch_regs->Chn_TxRxBytesDoneCtr) {
//	JMP		CHK_TX_DATA_FORMAT_DONE
		return;

//TX_DATA_FORMAT:
	}
	// Load the TX Format Address for the specific channel
//	XOR 	scratch_reg1, scratch_reg1, scratch_reg1
//	NOT 	scratch_reg1, scratch_reg1
	scratch_reg1 = ~0;

//	SUB 	scratch_reg2, suart_tx_ch.Chn_Config2.b0, 1
	scratch_reg2 = ((uint8_t *)&suart_tx_ch->Chn_Config2)[0] - 1;

//	LSL	    scratch_reg1, scratch_reg1, scratch_reg2
//	XOR 	scratch_reg2, scratch_reg2, scratch_reg2  // offset from base addr
	scratch_reg1 <<= scratch_reg2;
	scratch_reg2 = 0;

	// to store formated data into DATA RAM
//	MOV	    scratch_reg3, tx_context.buff_addr
	scratch_reg3 = tx_context->buff_addr;

	//Number of Bits Per Character
//	AND  	scratch_8bit_reg1, suart_ch_regs.Chn_Config2.b0, 0xF
//   	SUB     scratch_8bit_reg1, scratch_8bit_reg1, 2
	scratch_8bit_reg1 = ((uint8_t *)&suart_tx_ch->Chn_Config2)[0] & 0xf;

        //Check stop bit value
//        LSR     scratch_reg4, suart_tx_ch.Chn_Cntrl, SUART_CTRL_STOPBIT_SHIFT
	scratch_reg4 = suart_tx_ch->Chn_Cntrl >> SUART_CTRL_STOPBIT_SHIFT;
//        QBBC    CHECK_PARITY_VAL, scratch_reg4, 0
	if (scratch_reg4 & (1U << 0)) {
//        SUB     scratch_8bit_reg1, scratch_8bit_reg1, 1
//        LSR     scratch_reg1, scratch_reg1, 1
		scratch_8bit_reg1--;
		scratch_reg1 >>= 1;

//CHECK_PARITY_VAL:
	}
        //Load parity value
//        LSR     scratch_reg4, suart_tx_ch.Chn_Config2, SUART_CH_CONFIG2_PARITY_SHIFT
	scratch_reg4 = suart_tx_ch->Chn_Config2 >> SUART_CH_CONFIG2_PARITY_SHIFT;
//        AND     scratch_reg4, scratch_reg4, SUART_CH_CONFIG2_PARITY_MASK
	scratch_reg4 &= SUART_CH_CONFIG2_PARITY_MASK;
//        QBEQ    TX_DATA_FORMAT_LOOP, scratch_reg4, 0
	if (!scratch_reg4) {
//        SUB     scratch_8bit_reg1, scratch_8bit_reg1, 1
		scratch_8bit_reg1--;

//TX_DATA_FORMAT_LOOP:
	}
	do {
	//Load the data from the data pointer
//	LD16	TX_DATA_reg, suart_tx_ch.ch_TXRXData
		*TX_DATA_reg = suart_tx_ch->ch_TxRxData;
//        QBEQ    FORMAT_START_STOP, scratch_reg4, 0
		if (!scratch_reg4) {

//        AND     scratch_reg6, TX_DATA_reg, 0x1
//        MOV     scratch_reg7, TX_DATA_reg
//        MOV     scratch_reg5, scratch_8bit_reg1
			scratch_reg6 = *TX_DATA_reg & 0x1;
			scratch_reg7 = *TX_DATA_reg;
			scratch_reg5 = scratch_8bit_reg1;

//PARITY_CALC:
			do {
//        LSR     scratch_reg7, scratch_reg7, 1
				scratch_reg7 >>= 1;
//        XOR     scratch_reg6, scratch_reg7, scratch_reg6
				scratch_reg6 ^= scratch_reg7;
//        SUB     scratch_reg5, scratch_reg5, 1
				scratch_reg5--;
//        QBLT    PARITY_CALC, scratch_reg5, 1
			} while (scratch_reg5 > 1);
//        QBEQ    EVEN_PARITY, scratch_reg4, 2
			if (scratch_reg4 != 2) {
//        NOT     scratch_reg6, scratch_reg6
				scratch_reg6 = ~scratch_reg6;

//EVEN_PARITY:
			}
//        AND     scratch_reg6, scratch_reg6, 0x1
			scratch_reg6 &= 0x1;
//        QBEQ    SET_PARITY, scratch_reg6, 1
			if (scratch_reg6 != 1) {
//        CLR     TX_DATA_reg, TX_DATA_reg, scratch_8bit_reg1
				*TX_DATA_reg &= ~(1U << scratch_8bit_reg1);
//        JMP     FORMAT_START_STOP

//SET_PARITY:
			} else {
//        SET     TX_DATA_reg, TX_DATA_reg, scratch_8bit_reg1
				*TX_DATA_reg |= (1U << scratch_8bit_reg1);

//FORMAT_START_STOP:
			}
		}
//	LSL	    TX_DATA_reg, TX_DATA_reg, 1
//	OR	    TX_DATA_reg, TX_DATA_reg, scratch_reg1
		*TX_DATA_reg <<= 1;
		*TX_DATA_reg |= scratch_reg1;

	//store formated data into DATA RAM
//	SBBO	TX_DATA_reg.w0, scratch_reg3, scratch_reg2, 2
		*(uint16_t *)(scratch_reg3 + scratch_reg2) = *TX_DATA_reg;

	//Increment the formatted buffer address offset
//	ADD 	scratch_reg2, scratch_reg2, 2
		scratch_reg2 += 2;


//	QBGE  	INC_ADDR_BY_ONE, scratch_8bit_reg1, SUART_CH_CONFIG2_8BITS_PER_CHAR //0x8
		if (scratch_8bit_reg1 > SUART_CH_CONFIG2_8BITS_PER_CHAR) {
	//Next data buffer pointer
//	ADD		suart_tx_ch.ch_TXRXData, suart_tx_ch.ch_TXRXData, 1
			suart_tx_ch->ch_TxRxData++;

// Increment the tx buffer data pointer by ONE, if bit per character is less or equal to 8 including start and stop bit
//INC_ADDR_BY_ONE:
		}
	//Next data buffer pointer
//	ADD	    suart_tx_ch.ch_TXRXData, suart_tx_ch.ch_TXRXData, 1
		suart_tx_ch->ch_TxRxData++;

//	QBEQ	CHK_TX_DATA_FORMAT_DONE, tx_context.buff_size, 0
		if (tx_context->buff_size == 0) return;

	//Decrement the data length .i.e no of bytes to send
//	SUB	    tx_context.buff_size, tx_context.buff_size, 1
		tx_context->buff_size--;

//	JMP		TX_DATA_FORMAT_LOOP
	} while (true);
}
//******************************************** TX CHK_TX_DATA_FORMAT: ENDS************************************

//======================================================================================================================================

//******************************************** TX READ_TX_DATA: Starts****************************************

//======================================================================================================================================
// 	Reads the 16 bit formatted character to be transmitted from formatted data area corresponding to TX channel
//======================================================================================================================================

//READ_TX_DATA:
uint32_t read_tx_data(Suart_Ch_Struct *suart_ch_regs, Suart_Tx_Context *tx_context) {
	uint32_t scratch_reg3;
	uint32_t scratch_reg4;

	// Copy the base address of formated data
//	MOV	    scratch_reg3, tx_context.buff_addr
	scratch_reg3 = tx_context->buff_addr;

	// Calculate the offset of formated data
//	LSL 	scratch_reg4, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1
	scratch_reg4 = suart_ch_regs->Chn_TxRxBytesDoneCtr << 1;

	//LOAD formated data from DATA RAM
//	LBBO 	scratch_reg3, scratch_reg3, scratch_reg4, 2
	*(uint16_t *)scratch_reg3 = *(uint16_t *)(scratch_reg3 + scratch_reg4);

	return scratch_reg3;
//.ret
}

//********************************************** TX READ_TX_DATA: ENDS ***************************************

//======================================================================================================================================

//********************************************** TX LOAD_TX_FORMAT_ADDRESS : Starts **************************

//======================================================================================================================================
//	Initializes the TX formatted data buffer address which stores the formated data with stop and start bit
//======================================================================================================================================

//LOAD_TX_FORMAT_ADDRESS:
void load_tx_format_address(Suart_Ch_Info *suart_ch_info, Suart_Tx_Context *tx_context)
{
//	QBEQ	TX_CH0_FMT_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_0
//	QBEQ	TX_CH2_FMT_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_2
//	QBEQ	TX_CH4_FMT_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_4
//	QBEQ	TX_CH6_FMT_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_6
	switch (suart_ch_info->ch_num) {
//.ret

//TX_CH0_FMT_ADDR:
	case SUART_CHANNEL_0:
//	LDI 	tx_context.buff_addr, SUART_CH0_TX_FMT_ADDR
		tx_context->buff_addr = SUART_CH0_TX_FMT_ADDR;
//.ret
		break;


//TX_CH2_FMT_ADDR:
	case SUART_CHANNEL_2:
//	LDI 	tx_context.buff_addr, SUART_CH2_TX_FMT_ADDR
		tx_context->buff_addr = SUART_CH2_TX_FMT_ADDR;
//.ret
		break;


//TX_CH4_FMT_ADDR:
	case SUART_CHANNEL_4:
//	LDI 	tx_context.buff_addr, SUART_CH4_TX_FMT_ADDR
		tx_context->buff_addr = SUART_CH4_TX_FMT_ADDR;
//.ret
		break;


//TX_CH6_FMT_ADDR:
	case SUART_CHANNEL_6:
//	LDI 	tx_context.buff_addr, SUART_CH6_TX_FMT_ADDR
		tx_context->buff_addr = SUART_CH6_TX_FMT_ADDR;
//.ret
		break;
	}
}
//******************************************** TX LOAD_TX_FORMAT_ADDRESS Routine: ENDS ***********************

//======================================================================================================================================

//******************************************** PRESACLE_TX_DATA : Starts *************************************

//======================================================================================================================================
// This routine Prescales data bit to be transmitted into the TX_DATA_reg.w0 register
//======================================================================================================================================

void byte_loop(Suart_Ch_Struct *suart_ch_regs, uint32_t scratch_reg2, uint32_t scratch_reg3, uint8_t scratch_8bit_reg2, uint32_t *TX_DATA_reg)
{
	uint32_t scratch_reg1;
	uint32_t scratch_reg4;

	do {
//	MOV 	scratch_reg4, SUART_CTRL_PRE_SCALAR_MASK
//	AND		scratch_reg1, scratch_reg4, suart_ch_regs.Chn_Config1
		scratch_reg4 = SUART_CTRL_PRE_SCALAR_MASK;
		scratch_reg1 = scratch_reg4 & suart_ch_regs->Chn_Config1;

BITS_LOOP:
		do {
//	AND     scratch_reg4, scratch_reg3,0x1					// COPY 1 bit to scratch_reg4 from scratch_reg3
			scratch_reg4 = scratch_reg3 & 0x1;
//	LSL 	scratch_reg4, scratch_reg4, scratch_8bit_reg2
			scratch_reg4 <<= scratch_8bit_reg2;
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg4
			*TX_DATA_reg |= scratch_reg4;
//	ADD	    scratch_8bit_reg2, scratch_8bit_reg2, 1
			scratch_8bit_reg2++;
//	SUB     scratch_reg2, scratch_reg2, 1  				//INC Bytes loop counter
			scratch_reg2--;
//	SUB     scratch_reg1, scratch_reg1, 1			//INC Bits loop counter
			scratch_reg1--;
//	QBLE    BITS_LOOP, scratch_reg1, 1
		} while (scratch_reg1 >= 1);
//	LSR	    scratch_reg3, scratch_reg3, 1
		scratch_reg3 >>= 1;
//	QBLE    BYTE_LOOP, scratch_reg2, 1
	} while (scratch_reg2 >= 1);
}

//PRESACLE_TX_DATA:
void prescale_tx_data(Suart_Ch_Struct *suart_ch_regs, uint32_t scratch_reg3, uint32_t *TX_DATA_reg)
{
	uint32_t scratch_reg2;
	uint8_t scratch_8bit_reg2;

//	XOR 	scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2       //to Left shift each copied data bit
//	MOV 	scratch_reg2, 16        						//Xbuf Size: Keep track of byte loop
//	XOR 	TX_DATA_reg.w0,TX_DATA_reg.w0,TX_DATA_reg.w0    //Clear TX_DATA_reg.w0
	scratch_8bit_reg2 = 0;
	scratch_reg2 = 16;
	*TX_DATA_reg = 0;

	byte_loop(suart_ch_regs, scratch_reg2, scratch_reg3, scratch_8bit_reg2, TX_DATA_reg);
//.ret  // Return to PRESACLE_TX_DATA
}

//********************************************  PRESACLE_TX_DATA : ENDs **************************************

//======================================================================================================================================

//******************************************** TRANSMIT_PRESCALED_DATA : Starts ******************************

//======================================================================================================================================
// This routine Transmits the prescaled data by writing to mcasp x_buf register mapped to this serializer
//======================================================================================================================================

//TRANSMIT_PRESCALED_DATA:
void transmit_prescaled_data(Suart_Tx_Context *tx_context, uint32_t *TX_DATA_reg)
{
	// Clear the under run error
//	LBCO	scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4
//	QBBC	WRITE_TO_XBUF, scratch_reg1, 0x8
	if (MCASP0.XSTAT_bit.XERR) {

//	LDI		scratch_reg1, 0xFFFF
//	SBCO	scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4
		MCASP0.XSTAT = 0xffff;

//WRITE_TO_XBUF:
	}
	// Write Byte to X_BUF
//	SBBO  	TX_DATA_reg.w0, tx_context.asp_xbuf_reg, #00, 4
	tx_context->asp_xbuf_reg = *TX_DATA_reg;

//.ret   // return from Transmit Prescaled Data
}
//******************************************** TRANSMIT_PRESCALED_DATA : ENDs ********************************

//========================================================================================================================================

//******************************************** LOAD_RX_CONTEXT_ADDRESS: Start*********************************
//LOAD_RX_CONTEXT_ADDRESS:
void load_rx_context_address(Suart_Ch_Info *suart_ch_info)
{
//	QBEQ	RX_CONTEXT_CH1_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_1
//	QBEQ	RX_CONTEXT_CH3_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_3
//	QBEQ	RX_CONTEXT_CH5_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_5
//	QBEQ	RX_CONTEXT_CH7_ADDR, suart_ch_info.ch_num, SUART_CHANNEL_7
	switch (suart_ch_info->ch_num) {

//.ret
	default:
		break;

//RX_CONTEXT_CH1_ADDR:
	case SUART_CHANNEL_1:
//	LDI 	suart_ch_info.rx_context_addr, SUART_CH1_RX_CONTEXT_ADDR
		suart_ch_info->rx_context_addr = SUART_CH1_RX_CONTEXT_ADDR;
//.ret
		break;

//RX_CONTEXT_CH3_ADDR:
	case SUART_CHANNEL_3:
//	LDI 	suart_ch_info.rx_context_addr, SUART_CH3_RX_CONTEXT_ADDR
		suart_ch_info->rx_context_addr = SUART_CH3_RX_CONTEXT_ADDR;
//.ret
		break;

//RX_CONTEXT_CH5_ADDR:
	case SUART_CHANNEL_5:
//	LDI 	suart_ch_info.rx_context_addr, SUART_CH5_RX_CONTEXT_ADDR
		suart_ch_info->rx_context_addr = SUART_CH5_RX_CONTEXT_ADDR;
//.ret
		break;

//RX_CONTEXT_CH7_ADDR:
	case SUART_CHANNEL_7:
//	LDI 	suart_ch_info.rx_context_addr, SUART_CH7_RX_CONTEXT_ADDR
		suart_ch_info->rx_context_addr = SUART_CH7_RX_CONTEXT_ADDR;
//.ret
		break;
	}
}
//******************************************** LOAD_RX_CONTEXT_ADDRESS Ends ***********************************


// SUART_EMULATION:
int main(void) {
	uint16_t MAX_RX_TIMEOUT_TRIES;
	Suart_Global suart_global;
	Suart_Ch_Struct *suart_ch_regs;
	Suart_Ch_Struct *suart_tx_ch;
	Suart_Ch_Info *suart_ch_info;
	Suart_Tx_Context *tx_context;
	Suart_Rx_Context *rx_context;
	uint8_t scratch_8bit_reg2;
	uint32_t scratch_reg1;
	uint32_t scratch_reg2;
	uint32_t scratch_reg3;
	uint32_t scratch_reg4;
	uint32_t scratch_reg5;
	uint32_t scratch_reg6;
	uint32_t scratch_reg7;
	uint32_t mcasp_rbuf_val;
	uint32_t TX_DATA_reg;

	// Clear the ZERO Register r20
	// XOR    pZERO, pZERO, pZERO
//	pZERO = 0;

//--------------------- McASP TX Initialization - Starts ----------------
//activate Clocks, Serializers, state machine and frame sync
// tx_asp_init1:
    //Activate the high-transmit clock XHCLKRST
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
//     SET       scratch_reg1, BIT_9
//     SBCO   	  scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
	MCASP0.XGBLCTL_bit.XHCLKRST = 1;

// tx_asp_init2:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
//     QBBC      tx_asp_init2, scratch_reg1, BIT_9
	while (!MCASP0.XGBLCTL_bit.XHCLKRST) { }

    //Activate the transmit frequency clock XCLKRST
//     SET       scratch_reg1, BIT_8
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
	MCASP0.XGBLCTL_bit.XCLKRST = 1;

// tx_asp_init3:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
//     QBBC      tx_asp_init3, scratch_reg1, BIT_8
	while (!MCASP0.XGBLCTL_bit.XCLKRST) { }

    //Before starting, clear the respective transmitter and receiver status registers by writing 0xffff
//     MOV32     scratch_reg2, 0xffff
//     SBCO      scratch_reg2, MCASP_CONTROL, MCASP_XSTAT, 2
	MCASP0.XSTAT = 0xffff;

    // Activate serializer, XSRCLR
//     SET       scratch_reg1, BIT_10
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
	MCASP0.XGBLCTL_bit.XSRCLR = 1;

// tx_asp_init4:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
//     QBBC      tx_asp_init4, scratch_reg1, BIT_10
	while (!MCASP0.XGBLCTL_bit.XSRCLR) { }

    // Till now no serializer is activated for TX, so no need to service all active XBUF
    // to avoid underrun errors to be done

    // Activate the McASP state machine
//     SET       scratch_reg1, BIT_11
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
	MCASP0.XGBLCTL_bit.XSMRST = 1;

// tx_asp_init5:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
//     QBBC      tx_asp_init5, scratch_reg1, BIT_11
	while (!MCASP0.XGBLCTL_bit.XSMRST) { }

    // Activate the MCASP Frame sync
//     SET       scratch_reg1, BIT_12
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
	MCASP0.XGBLCTL_bit.XFRST = 1;

// tx_asp_init6:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_XGBLCTL, 4
//     QBBC      tx_asp_init6, scratch_reg1, BIT_12
	while (!MCASP0.XGBLCTL_bit.XFRST) { }

//----------------------- McASP TX Initialization - Ends ------------------

//--------------------- McASP RX Initialization - Starts ----------------

//activate Clocks,Serializers,state machine and frame sync
// rx_asp_init1:
    //Activate the high-transmit clock RHCLKRST
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
//     SET       scratch_reg1, BIT_1
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
	MCASP0.RGBLCTL_bit.RHCLKRST = 1;

// rx_asp_init2:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
//     QBBC      rx_asp_init2, scratch_reg1, BIT_1
	while (!MCASP0.RGBLCTL_bit.RHCLKRST) { }

    //Activate the transmit frequency clock RCLKRST
//     SET       scratch_reg1, BIT_0
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
	MCASP0.RGBLCTL_bit.RCLKRST = 1;

// rx_asp_init3:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
//     QBBC      rx_asp_init3, scratch_reg1, BIT_0
	while (!MCASP0.RGBLCTL_bit.RCLKRST) { }

	//Clear RSTAT
//     LDI       scratch_reg2, 0xffff
//     SBCO      scratch_reg2, MCASP_CONTROL, MCASP_RSTAT, 2
	MCASP0.RSTAT = 0xffff;

    // Activate serializer, RSRCLR
//     SET       scratch_reg1, BIT_2
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
	MCASP0.RGBLCTL_bit.RSRCLR = 1;

// rx_asp_init4:
//    LBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
//    QBBC      rx_asp_init4, scratch_reg1, BIT_2
	while (!MCASP0.RGBLCTL_bit.RSRCLR) { }

    // Activate the McASP state machine
//     SET       scratch_reg1, BIT_3
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
	MCASP0.RGBLCTL_bit.RSMRST = 1;

// rx_asp_init5:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
//     QBBC      rx_asp_init5, scratch_reg1, BIT_3
	while (!MCASP0.RGBLCTL_bit.RSMRST) { }

    // Activate the MCASP Frame sync
//     SET       scratch_reg1, BIT_4
//     SBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
	MCASP0.RGBLCTL_bit.RFRST = 1;

// rx_asp_init6:
//     LBCO      scratch_reg1, MCASP_CONTROL, MCASP_RGBLCTL, 4
//     QBBC      rx_asp_init6, scratch_reg1, BIT_4
	while (!MCASP0.RGBLCTL_bit.RFRST) { }

//----------------------- McASP RX Initialization - Ends ------------------

// LOCAL_INIT:
	//Read the PRU ID
	// LBBO	scratch_reg2, pZERO, SUART_GBL_PRU_STATUS_ADDR, 4
	// MOV 	suart_global.pru_id, scratch_reg2.b0
	suart_global.pru_id = ((uint8_t *)SUART_GBL_PRU_STATUS_ADDR)[0];

	// Read the PRU mode
//	MOV		suart_global.pru_rx_tx_mode, scratch_reg2.b1
	suart_global.pru_rx_tx_mode = ((uint8_t *)SUART_GBL_PRU_STATUS_ADDR)[1];

        //PRU Delay Count in CORE_LOOP
//        MOV     suart_global.pru_delay_cnt, scratch_reg2.b2
	suart_global.pru_delay_cnt = ((uint8_t *)SUART_GBL_PRU_STATUS_ADDR)[2];

	//Clear RSTAT
//	LDI     scratch_reg2, 0xffff
//   	SBCO    scratch_reg2, MCASP_CONTROL, MCASP_RSTAT, 4
	MCASP0.RSTAT = 0xffff;

	//Clear XSTAT
//	LDI     scratch_reg2, 0xffff
//   	SBCO    scratch_reg2, MCASP_CONTROL, MCASP_XSTAT, 4
	MCASP0.XSTAT = 0xffff;

//	QBEQ   CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY
	if (suart_global.pru_rx_tx_mode == PRU_MODE_TX_ONLY) goto CORE_LOOP;

        // This Block the Sampling Point with invalid value in RX Context Area
//        LDI    scratch_reg2, 0xFF
	scratch_reg2 = 0xff;

//	QBEQ   PRUxxxx_MODE_RX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY
	if (suart_global.pru_rx_tx_mode != PRU_MODE_RX_ONLY) {

//	LDI    scratch_reg1, SUART_CH1_RX_CONTEXT_ADDR
//	LDI    scratch_reg4, RX_CONTEXT_OFFSET
//	LDI    scratch_reg3, 0x1B0
//	JMP	   INIT_SAMPLE_PNT
		scratch_reg1 = SUART_CH1_RX_CONTEXT_ADDR;
		scratch_reg4 = RX_CONTEXT_OFFSET;
		scratch_reg3 = 0x1B0;
	}

//PRUxxxx_MODE_RX_ONLY:
//	LDI    scratch_reg1, 0x90
//	LDI    scratch_reg4, 0x20
//	LDI    scratch_reg3, 0x170
	else {
		scratch_reg1 = 0x90;
		scratch_reg4 = 0x20;
		scratch_reg3 = 0x170;
	}

//INIT_SAMPLE_PNT:
//	SBBO   scratch_reg2, scratch_reg1, SUART_CH_SAMPLING_BIT_POS_OFFSET, 1
//	SBBO   pZERO.b0, scratch_reg1, SUART_CH_FALSE_START_FLAG_OFFSET, 1
	do {
		*(uint8_t *)(scratch_reg1 + SUART_CH_SAMPLING_BIT_POS_OFFSET) = scratch_reg2;
		*(uint8_t *)(scratch_reg1 + SUART_CH_FALSE_START_FLAG_OFFSET) = 0;

//        ADD    scratch_reg1, scratch_reg1, scratch_reg4
//        QBGE   INIT_SAMPLE_PNT, scratch_reg1, scratch_reg3
		scratch_reg1 += scratch_reg4;
	} while (scratch_reg1 <= scratch_reg3);


	// JUMP  to CORE_LOOP
//	JMP    CORE_LOOP
	goto CORE_LOOP;

//=====================================================================================================================================

//		************************************************************************************************
//						SOFT-UART TRANSMIT ROUTINE : STARTS
//		************************************************************************************************

//=====================================================================================================================================

//********************************************  TxServiceRequestHndlr Starts *********************************

//=====================================================================================================================================
// This routine perform the basic initialization and clearing of various registers for TX and loads the configuration info
// from PRU RAM to register for serviced TX channel.  It calculates and saves serializer and x_buf address mapped
// to TX channel to TX context area so that each time it is not calculated again and is being directly read from TX Context Area.
//=====================================================================================================================================

TxServiceRequestHndlr:
	//read interrupt status register
//	LBBO    suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, 2
	suart_global.intrStatus = *(uint16_t *)SUART_GBL_INT_STATUS_ADDR;

	// clear the channel interrupt status bit
//	CLR	    suart_global.intrStatus, suart_global.intrStatus, suart_ch_info.ch_num
	suart_global.intrStatus &= ~(1U << suart_ch_info->ch_num);

	//update interrupt status register
//	SBBO    suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, 2
	*(uint16_t *)SUART_GBL_INT_STATUS_ADDR = suart_global.intrStatus;

	//Clear Service Request
//	CLR  	suart_ch_regs.Chn_Cntrl, SUART_CTRL_SR_BIT // initialized by driver
//	SBBO 	suart_ch_regs.Chn_Cntrl,suart_ch_info.curr_ch_base_addr, SUART_CH_CTRL_OFFSET, SIZE (suart_ch_regs.Chn_Cntrl)
	suart_ch_regs->Chn_Cntrl &= ~(1U << SUART_CTRL_SR_BIT);
	*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_CTRL_OFFSET) = suart_ch_regs->Chn_Cntrl;

	// Set the TXRX_READY_BIT
//	SET  	suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT  //(Initialized by driver)
//	SBBO 	suart_ch_regs.Chn_TxRxStatus, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXSTATUS_OFFSET, SIZE (suart_ch_regs.Chn_TxRxStatus)
	suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_TXRX_READY_BIT);
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXSTATUS_OFFSET) = suart_ch_regs->Chn_TxRxStatus;

	// Set SUART_CH_TXRXCHNSTATUS_BIT bit in channel status to indicate the channel active
//	SET     suart_ch_regs.Chn_Status, SUART_CH_TXRXCHNSTATUS_BIT
//	SBBO 	suart_ch_regs.Chn_Status, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXCHNSTATUS_OFFSET, SIZE (suart_ch_regs.Chn_Status)
	suart_ch_regs->Chn_Status |= (1U << SUART_CH_TXRXCHNSTATUS_BIT);
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXCHNSTATUS_OFFSET) = suart_ch_regs->Chn_Status;

	// New Tx Request received initialize the Channel specific data and save it in memory
//	XOR 	suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBytesDoneCtr)
	suart_ch_regs->Chn_TxRxBytesDoneCtr = 0;
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBYTESDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBytesDoneCtr;

	// Load channel specific serializer, xbuf, srctl register mapped active channel
//	JMP 	LOAD_TX_COMMON_INFO
	goto LOAD_TX_COMMON_INFO;

ENABLE_TX_SERIALIZER:
        //Change the MCASP AXR[n] pin from GPIO mode to MCASP mode of operation
//        LBCO    scratch_reg2, MCASP_CONTROL, MCASP_PFUNC, 4
//        AND     scratch_reg1, suart_ch_regs.Chn_Cntrl.b1, SUART_CTRL_SERIALIZER_MASK
//        CLR     scratch_reg2, scratch_reg1
//        SBCO    scratch_reg2, MCASP_CONTROL, MCASP_PFUNC, 4
	MCASP0.PFUNC &= ~(1U << (((uint8_t *)&suart_ch_regs->Chn_Cntrl)[1] & SUART_CTRL_SERIALIZER_MASK));

//CLEAR_XSTAT:
//        LDI     scratch_reg1, 0xFFFF
//        SBCO    scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4
//        JMP     MCASP_EVENT
	MCASP0.XSTAT = 0xffff;
	goto MCASP_EVENT;

//******************************************** TxServiceRequestHndlr Ends ************************************

//=====================================================================================================================================

//******************************************** TxServiceReqHndlLoop Starts ***********************************

//=====================================================================================================================================
//  This routine reads the formated data to be transmitted from formatted data area region mapped to
//  current serviced TX channel and depending upon prescalar value in config1 register, it jumps to that
// 	that prescalar label. This is getting called from TX interrupt handler or when is there new service request for TX.
//=====================================================================================================================================

TxServiceReqHndlLoop:
//        QBBC    DUMMY_TX, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT
	// Read the Formated byte to transmitted
//	CALL	READ_TX_DATA
	if (!(suart_ch_regs->Chn_TxRxStatus & (1U << SUART_TXRX_READY_BIT))) goto DUMMY_TX;
	scratch_reg3 = read_tx_data(suart_ch_regs, tx_context);

//	XOR 	TX_DATA_reg.w0,TX_DATA_reg.w0,TX_DATA_reg.w0
	TX_DATA_reg = 0;

        // Branch According to Pre-Scalar Value
//	MOV		scratch_reg1, SUART_CTRL_PRE_SCALAR_MASK
//	AND 	scratch_reg1, scratch_reg1, suart_ch_regs.Chn_Config1
	scratch_reg1 = SUART_CTRL_PRE_SCALAR_MASK;
	scratch_reg1 &= suart_ch_regs->Chn_Config1;

//   	QBEQ 	PRE_SCALAR1, scratch_reg1, PRE_SCALAR_1
//        QBEQ 	PRE_SCALAR2, scratch_reg1, PRE_SCALAR_2
//        QBEQ    PRE_SCALAR3, scratch_reg1, PRE_SCALAR_3
//	QBEQ 	PRE_SCALAR4, scratch_reg1, PRE_SCALAR_4
//	QBEQ 	PRE_SCALAR6, scratch_reg1, PRE_SCALAR_6
//	QBEQ 	PRE_SCALAR12, scratch_reg1, PRE_SCALAR_12
//	QBEQ 	PRE_SCALAR16, scratch_reg1, PRE_SCALAR_16
//	QBLE 	PRE_SCALAR24, scratch_reg1, PRE_SCALAR_24
	switch (scratch_reg1) {

//******************************************** TxServiceReqHndlLoop ENDS *************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR1 Starts ********************************************
//PRE_SCALAR1:
	case PRE_SCALAR_1:
	// copy data to RAM TX_DATA_reg.w0 register from scratch_reg3
//	MOV		TX_DATA_reg.w0, scratch_reg3
		TX_DATA_reg = scratch_reg3;

//	CALL    TRANSMIT_PRESCALED_DATA
		transmit_prescaled_data(tx_context, &TX_DATA_reg);

	// Increment the Chn_TxRxBytesDoneCtr bye one
//	ADD 	suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1
//	SBBO 	suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBytesDoneCtr)
		suart_ch_regs->Chn_TxRxBytesDoneCtr += 1;
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBYTESDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBytesDoneCtr;

//	JMP     TxInterruptServiceRequestHndlr
		goto TxInterruptServiceRequestHndlr;
//******************************************** PRE_SCALAR1 Ends **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR2 Starts ********************************************
//PRE_SCALAR2:
	case PRE_SCALAR_2:
//	MOV 	scratch_reg1, suart_ch_regs.Chn_TxRxBitsDoneCtr
//	QBGT	XMIT_FISRT_8BIT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
		scratch_reg1 = suart_ch_regs->Chn_TxRxBitsDoneCtr;
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr >= 1) {

//XMIT_LAST_8BIT:
//	LSR     scratch_reg3, scratch_reg3, 8  //Last 8 bits to transmitted
			scratch_reg3 >>= 8;

//	CALL    PRESACLE_TX_DATA
			prescale_tx_data(suart_ch_regs, scratch_reg3, &TX_DATA_reg);

//	CALL 	TRANSMIT_PRESCALED_DATA
			transmit_prescaled_data(tx_context, &TX_DATA_reg);

//	JMP     TX_DONE
			goto TX_DONE;

//XMIT_FISRT_8BIT:
		}
//	AND 	scratch_reg3, scratch_reg3, 0x00FF
//	CALL	PRESACLE_TX_DATA
		scratch_reg3 &= 0xff;
		prescale_tx_data(suart_ch_regs, scratch_reg3, &TX_DATA_reg);

//	CALL    TRANSMIT_PRESCALED_DATA
		transmit_prescaled_data(tx_context, &TX_DATA_reg);

	// Write To RAM number of Bits Transmitted
//	ADD		suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 8  	//8 bits transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
		suart_ch_regs->Chn_TxRxBitsDoneCtr += 8;
		*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

	// If bit per character less than 8  // added with start and stop bit in bits per channel

//        AND     scratch_reg1, suart_ch_regs.Chn_Config2, 0xF
//	QBGE  	TX_DONE, scratch_reg1, 0x8    //check  (Chn_Config2.BitsPerChar <= 8)
//	JMP     TxInterruptServiceRequestHndlr
		scratch_reg1 = suart_ch_regs->Chn_Config2 & 0xf;
		if (scratch_reg1 <= 8) goto TX_DONE;
		goto TxInterruptServiceRequestHndlr;

//******************************************** PRE_SCALAR2 ENDs **********************************************

//******************************************** PRE_SCALAR3 Starts ********************************************
//PRE_SCALAR3:
	case PRE_SCALAR_3:
//	QBGT 	XMIT_FIRST_6BIT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1   // transmit first 6 bit of formated data
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr >= 1) {

//GENERIC_TRANSMIT_BLOCK3:
	// initialize bitsLoaded register
        //  keeps track of how many bits have been placed into the TX_DATA register
//	MOV     tx_context.bitsLoaded, 0x0
//	//XOR     scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2
			tx_context->bitsLoaded = 0;

//LOAD_BITS_LOOP_FOR3:
			do {
        // Load bitsPerChar
//	AND 	scratch_reg2, suart_ch_regs.Chn_Config2, 0xF
				scratch_reg2 = suart_ch_regs->Chn_Config2 & 0xf;

        // transmit the next bits if ( BitsDoneCntr < BitsPerChar )
//	QBLT 	XMIT_NXT_5BIT, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr
				if (scratch_reg2 <= suart_ch_regs->Chn_TxRxBitsDoneCtr) {

	// transmit the last remaining bits of the present byte
//XMIT_MORE_BITS3:
	// update the bytes done counter and reset the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
//	ADD 	suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1
//	SBBO    suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBytesDoneCtr)
					suart_ch_regs->Chn_TxRxBytesDoneCtr++;
					*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBYTESDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBytesDoneCtr;

//	XOR	    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
					suart_ch_regs->Chn_TxRxBitsDoneCtr = 0;
					*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	XOR	    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
					suart_ch_regs->Chn_TxRxRepeatDoneCtr = 0;
					*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

	// set the remaining bits to one, if there are no more bits in formated data to send
	// and still there is space in TX_DATA_reg.
//	RSB     scratch_reg1, tx_context.bitsLoaded, 0x10	   	//16 - bitsLoaded
//	//XOR     scratch_reg3, scratch_reg3, scratch_reg3		// Load the remaining (16 - bitsLoaded) bits with logic High
//	//NOT   	scratch_reg3, scratch_reg3
//        //NOTE make sr3 all 1's without 2 commands
//        LDI     scratch_reg3, 0xFFFF //CHANGE
					scratch_reg1 = 10 - tx_context->bitsLoaded;
					scratch_reg3 = 0xffff;

//	RSB 	scratch_8bit_reg2, scratch_reg1, 16			//calculate the bit position from where one is to be inserted
					scratch_8bit_reg2 = 16 - scratch_reg1;

        // NOTE
//	//XOR 	scratch_reg2, scratch_reg2, scratch_reg2		//CLR scratch_reg2
//	//AND     scratch_reg2, scratch_reg3,0x1				// COPY 1 bit to scratch_reg2 from scratch_reg3
//        AND scratch_reg2, scratch_reg3,0x1 //CHANGE
					scratch_reg2 = scratch_reg3 & 0x1;

//	LSL 	scratch_reg2, scratch_reg2, scratch_8bit_reg2
					scratch_reg2 <<= scratch_8bit_reg2;

	// Now, set the remaining bits to one in TX_DATA_reg
//SET_BIT_BIT3:
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2
//	LSL 	scratch_reg2, scratch_reg2, 1
//	SUB     scratch_reg1, scratch_reg1, 1
//	QBLE    SET_BIT_BIT, scratch_reg1, 1
					TX_DATA_reg |= scratch_reg2;
					scratch_reg2 <<= 1;
					scratch_reg1--;
					if (scratch_reg1 >= 1) goto SET_BIT_BIT;

//   	MOV 	tx_context.bitsLoaded, 16
//	JMP	CHK_MORE_PRESCALAR3
					tx_context->bitsLoaded = 16;
					continue;

//XMIT_NXT_5BIT:
				}
	// if the bitsLoaded in TX_DATA_reg is less than 16, load the next bits (BIT_LOAD16)
//	QBLT 	CHK_MORE_PRESCALAR3, tx_context.bitsLoaded, 0x10
				if (tx_context->bitsLoaded > 0x10) continue;

        // if( bitsLoaded < 16 )
//BIT_LOAD16_3:
	// Read Prescalar value
//	MOV    	scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//	AND    	scratch_reg1, scratch_reg2, suart_ch_regs.Chn_Config1
				scratch_reg1 = suart_ch_regs->Chn_Config1 & SUART_CTRL_PRE_SCALAR_MASK;

//	RSB	    scratch_reg2, tx_context.bitsLoaded, 16    //(16 - bitsLoaded)
//	SUB	    scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxRepeatDoneCtr //(Chn_Config1.PreScaller - ChnTxRxRepeatDoneCntr)
				scratch_reg2 = 16 - tx_context->bitsLoaded;
				scratch_reg1 -= suart_ch_regs->Chn_TxRxRepeatDoneCtr;

        // sr1 = MIN( #Needed_Repeats, (16 - bitsLoaded) )
//	MIN    	scratch_reg1, scratch_reg1, scratch_reg2
				scratch_reg1 = scratch_reg1 < scratch_reg2 ? scratch_reg1 : scratch_reg2;

	// Get formatted TX data into sr3
//	CALL	READ_TX_DATA
				scratch_reg3 = read_tx_data(suart_ch_regs, tx_context);

        // Shift off the bits that have been fully repeated and transmitted
//	LSR 	scratch_reg3, scratch_reg3, suart_ch_regs.Chn_TxRxBitsDoneCtr
				scratch_reg3 >>= suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	AND 	scratch_reg2, scratch_reg3, 0x1		// copy bit to transmitted to scratch_reg2
//	MOV     scratch_reg4, scratch_reg1         	// move repeat count to scratch_reg4
//	LSL 	scratch_reg2, scratch_reg2, tx_context.bitsLoaded   // shift the bit to be transmitted to expected position
				scratch_reg2 = scratch_reg3 * 0x1;
				scratch_reg4 = scratch_reg1;
				scratch_reg2 <<= tx_context->bitsLoaded;

	// prescale the bit to transmitted
//PRESCALE_NXT_BIT3:
				do {
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2
					TX_DATA_reg |= 	scratch_reg2;
//	LSL 	scratch_reg2, scratch_reg2, 1
					scratch_reg2 <<= 1;
//	SUB     scratch_reg1, scratch_reg1, 1
					scratch_reg1--;
//	QBLE    PRESCALE_NXT_BIT3, scratch_reg1, 1
				} while (scratch_reg1 >= 1);

	// write back to memory
//	ADD 	tx_context.bitsLoaded, tx_context.bitsLoaded, scratch_reg4
				tx_context->bitsLoaded &= scratch_reg4;

//	ADD 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, scratch_reg4
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
				suart_ch_regs->Chn_TxRxRepeatDoneCtr += scratch_reg4;
				*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

	// get the prescalar value
//	MOV    scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//	AND    scratch_reg1, scratch_reg2, suart_ch_regs.Chn_Config1
				scratch_reg1 = suart_ch_regs->Chn_Config1 & SUART_CTRL_PRE_SCALAR_MASK;

	//if bit has been transmitted prescaler times, fall through and updated the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
//	QBGT    CHK_MORE_PRESCALAR3, suart_ch_regs.Chn_TxRxRepeatDoneCtr, scratch_reg1
				if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= scratch_reg1) {

        //if (Chn_Config1.PreScaller == ChnTxRxRepeatDoneCntr)

//TX_BIT_DONE_CNTR:  // NOTE::nothing calls this function. Just runs into
	// Write Chn_TxRxBitsDoneCtr
//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
					suart_ch_regs->Chn_TxRxBitsDoneCtr++;
					*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

	// Write Chn_TxRxRepeatDoneCtr
//	XOR 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
					suart_ch_regs->Chn_TxRxRepeatDoneCtr = 0;
					*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;
				}
//CHK_MORE_PRESCALAR3:   //rename this label to CHK_TX_DATA_REG_FULL
//	QBGT 	LOAD_BITS_LOOP_FOR3, tx_context.bitsLoaded, 0x10
			} while (tx_context->bitsLoaded < 10);

        //if (bitsLoaded = 16),  TX_DATA_reg full
//	CALL 	TRANSMIT_PRESCALED_DATA				  // TX_DATA_reg is full, transmit the data
//	JMP     TxInterruptServiceRequestHndlr
			transmit_prescaled_data(tx_context, &TX_DATA_reg);
			goto TxInterruptServiceRequestHndlr;

	// transmit the bits from start bit that can be transmitted from present character that is to transmitted
//XMIT_FIRST_6BIT:
		}
//	AND 	scratch_reg3, scratch_reg3, 0x3F			// copy the first 6 bits to be transmitted
//	MOV 	scratch_reg2, 15        			//number of times the byte loop is to be looped
//                                                                // 5 bits are repeated 3 times = 15bits
//                                                                // the 6th bit is placed once at last slot
		scratch_reg3 &= 0x3f;
		scratch_reg2 = 15;

//	XOR 	TX_DATA_reg.w0,TX_DATA_reg.w0,TX_DATA_reg.w0    //Clear TX_DATA_reg.w0
//        XOR     scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2 //Clear scartch_8bit_reg2
//	CALL	BYTE_LOOP
		TX_DATA_reg = 0;
		scratch_8bit_reg2 = 0;
		byte_loop(suart_ch_regs, scratch_reg2, scratch_reg3, scratch_8bit_reg2, &TX_DATA_reg);

//	MOV	    scratch_reg1, 0x1  				// Repeat last bit by 1 time
//                                                                // NOTE: OPTIMIZE THIS FOR NO REPEAT LATER
		scratch_reg1 = 1;

//	//XOR 	scratch_reg2, scratch_reg2, scratch_reg2	//CLR scratch_reg2 //CHANGE
//	AND     scratch_reg2, scratch_reg3, 0x1			//copy the sixth bit to scratch_reg2
		scratch_reg2 = scratch_reg3 & 0x1;

//	LSL 	scratch_reg2, scratch_reg2, 0xf      	//shift the bit to expected place i.e. bit pos 15
		scratch_reg2 <<= 0xf;

	// prescale the last bit 1 times (NOTE: CHANGE LATER)
//XMIT_LAST_1BIT:
		do {
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2
			TX_DATA_reg |= scratch_reg2;
//	LSL 	scratch_reg2, scratch_reg2, 1
			scratch_reg2 <<= 1;
//	SUB     scratch_reg1, scratch_reg1, 1
			scratch_reg1--;
//	QBLE    XMIT_LAST_1BIT, scratch_reg1, 1
		} while (scratch_reg1 >= 1);

//	CALL    TRANSMIT_PRESCALED_DATA
		transmit_prescaled_data(tx_context, &TX_DATA_reg);

//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 5    //Updating number of bits written
//                                                                                           // 5 were completely written, 1 partial
//	// Write To RAM number of Bits Transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
		suart_ch_regs->Chn_TxRxBitsDoneCtr += 5;
		*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	ADD 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1 //Updating number of bits written
//                                                                                            // #times last incomplete bit was written
//	// Write To RAM Write Repeat done counter to RAM
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
		suart_ch_regs->Chn_TxRxRepeatDoneCtr += 1;
		*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

//	JMP     TxInterruptServiceRequestHndlr
		goto TxInterruptServiceRequestHndlr;

//******************************************** PRE_SCALAR3 ENDs **********************************************
//======================================================================================================================================

//******************************************** PRE_SCALAR4 Starts ********************************************
//PRE_SCALAR4:
	case PRE_SCALAR_4:


//        QBGT 	XMIT_FIRST_4BIT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr >= 1) {

//XMIT_NXT_4BIT:
//	AND     scratch_reg2, suart_ch_regs.Chn_Config2, 0xF
//	SUB     scratch_reg2, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr    //Chn_Config2.BitsPerChar - Chn_TxRxBitsDoneCntr
			scratch_reg2 &= 0xf;
			scratch_reg2 -= suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	QBLT    MORE_DATA4, scratch_reg2, 4   //(Chn_Config2.BitsPerChar - Chn_TxRxBitsDoneCntr) > 4, more bits to be transmitted
			if (scratch_reg2 <= 4) {

	//transmit last remaining 4 bits
//	LSR		scratch_reg3, scratch_reg3, scratch_reg1
//	AND		scratch_reg3, scratch_reg3, 0xF
				scratch_reg3 >>= scratch_reg1;
				scratch_reg3 &= 0xf;

//	CALL	PRESACLE_TX_DATA
//        CALL    TRANSMIT_PRESCALED_DATA
				prescale_tx_data(suart_ch_regs, scratch_reg3, &TX_DATA_reg);
				transmit_prescaled_data(tx_context, &TX_DATA_reg);

//	JMP 	CHK_TX_DONE

//MORE_DATA4:			//transmit next 4 bit of present byte being transmitted
			} else {
//	LSR		scratch_reg3, scratch_reg3, scratch_reg1
//	AND		scratch_reg3, scratch_reg3, 0xF
				scratch_reg3 >>= scratch_reg1;
				scratch_reg3 &= 0xf;

//	CALL	PRESACLE_TX_DATA
				prescale_tx_data(suart_ch_regs, scratch_reg3, &TX_DATA_reg);

// 	CALL    TRANSMIT_PRESCALED_DATA
				transmit_prescaled_data(tx_context, &TX_DATA_reg);

	// Check all bits have been transmitted
//CHK_TX_DONE:
			}
	//Updating number of bits written
//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 4
			suart_ch_regs->Chn_TxRxBitsDoneCtr += 4;

	// Write To RAM number of Bits Transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
			*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	AND 	scratch_reg2, suart_ch_regs.Chn_Config2, 0xF
			scratch_reg2 &= 0xf;

	// check if all bits have been transmitted
//	QBGE    TX_DONE, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr
//	JMP     TxInterruptServiceRequestHndlr
			if (scratch_reg2 <= suart_ch_regs->Chn_TxRxBitsDoneCtr) goto TX_DONE;
			goto TxInterruptServiceRequestHndlr;

//XMIT_FIRST_4BIT:   // transmit first 4 bit of formated data
	}
//	AND		scratch_reg3, scratch_reg3, 0xF
//	CALL	PRESACLE_TX_DATA
//   	CALL    TRANSMIT_PRESCALED_DATA
		scratch_reg3 &= 0xf;
		prescale_tx_data(suart_ch_regs, scratch_reg3, &TX_DATA_reg);
		transmit_prescaled_data(tx_context, &TX_DATA_reg);

	//Updating number of bits written
//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 4
		suart_ch_regs->Chn_TxRxBitsDoneCtr += 4;

	// Write To RAM number of Bits Transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
		*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	JMP     TxInterruptServiceRequestHndlr
		goto TxInterruptServiceRequestHndlr;
//******************************************** PRE_SCALAR4 Ends **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR6 Starts ********************************************
//PRE_SCALAR6:
	case PRE_SCALAR_6:
//	QBGT 	XMIT_FIRST_3BIT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1   // transmit first 3 bit of formated data
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr >= 1) {

GENERIC_TRANSMIT_BLOCK:
	//initialize the register
//	MOV     tx_context.bitsLoaded, 0x0
//	XOR     scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2
			tx_context->bitsLoaded = 0;
			scratch_8bit_reg2 = 0;

//LOAD_BITS_LOOP_FOR6:
			do {
//	AND 	scratch_reg2, suart_ch_regs.Chn_Config2, 0xF
				scratch_reg2 = suart_ch_regs->Chn_Config2 & 0xf;

//	QBLT 	XMIT_NXT_3BIT, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr    // transmit the next bits if (ChnTxRxBitsDoneCntr < Chn_Config2.BitsPerChar)
				if (scratch_reg2 <= suart_ch_regs->Chn_TxRxBitsDoneCtr) {

	// transmit the last remaining bits of the present byte if any and updated counters as below
//XMIT_MORE_BITS:
	// update the bytes done counter and reset the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
//	ADD 	suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1
//	SBBO    suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBytesDoneCtr)
					suart_ch_regs->Chn_TxRxBytesDoneCtr++;
					*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBYTESDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBytesDoneCtr;

//	XOR	    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
					suart_ch_regs->Chn_TxRxBitsDoneCtr = 0;
					*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	XOR	    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
					suart_ch_regs->Chn_TxRxRepeatDoneCtr = 0;
					*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

	// set the remaining bits to one, if there are no more bits in formated data to send
	// and still there is space in TX_DATA_reg.
//	RSB     scratch_reg1, tx_context.bitsLoaded, 0x10	   	//16 - bitsLoaded
					scratch_reg1 = 0x10 - tx_context->bitsLoaded;


//        LDI     scratch_reg3, 0xFFFF
					scratch_reg3 = 0xffff;

//	RSB 	scratch_8bit_reg2, scratch_reg1, 16			//calculate the bit position from where one is to be inserted
//	AND     scratch_reg2, scratch_reg3,0x1				// COPY 1 bit to scratch_reg2 from scratch_reg3
//	LSL 	scratch_reg2, scratch_reg2, scratch_8bit_reg2
					scratch_8bit_reg2 = 16 - scratch_reg1;
					scratch_reg2 = scratch_reg3 & 0x1;
					scratch_reg2 <<= scratch_8bit_reg2;

	// Now, set the remaining bits to one in TX_DATA_reg
SET_BIT_BIT:
					do {
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2
						TX_DATA_reg |= scratch_reg2;
//	LSL 	scratch_reg2, scratch_reg2, 1
						scratch_reg2 <<= 1;
//	SUB     scratch_reg1, scratch_reg1, 1
						scratch_reg1--;
//	QBLE    SET_BIT_BIT, scratch_reg1, 1
					} while (scratch_reg1 >= 1);

//   	MOV 	tx_context.bitsLoaded, 16
					tx_context->bitsLoaded = 16;
//	JMP	    CHK_MORE_PRESCALAR

//XMIT_NXT_3BIT:
				}
	//if the bitsLoaded in TX_DATA_reg is less than 16 load the next bits
//	QBLT 	CHK_MORE_PRESCALAR, tx_context.bitsLoaded, 0x10  //(bitsLoaded < 16)
				else if (tx_context->bitsLoaded <= 0x10) {

//BIT_LOAD16:
	// Read Prescalar value
//	MOV    	scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//	AND    	scratch_reg1, scratch_reg2, suart_ch_regs.Chn_Config1
					scratch_reg1 = suart_ch_regs->Chn_Config1 & SUART_CTRL_PRE_SCALAR_MASK;

//	RSB	    scratch_reg2, tx_context.bitsLoaded, 16    //(16 - bitsLoaded)
//	SUB	    scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxRepeatDoneCtr //(Chn_Config1.PreScaller - ChnTxRxRepeatDoneCntr)
					scratch_reg2 = 16 - tx_context->bitsLoaded;
					scratch_reg1-= suart_ch_regs->Chn_TxRxRepeatDoneCtr;

//	MIN    	scratch_reg1, scratch_reg1, scratch_reg2  // repeat count = scratch_reg1, min of
					scratch_reg1 = scratch_reg1 < scratch_reg2 ? scratch_reg1 : scratch_reg2;

	// Read Next Bit
//	CALL	READ_TX_DATA
//	LSR 	scratch_reg3, scratch_reg3, suart_ch_regs.Chn_TxRxBitsDoneCtr
					scratch_reg3 = read_tx_data(suart_ch_regs, tx_context);
					scratch_reg3 >>= suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	AND 	scratch_reg2, scratch_reg3, 0x1		// copy bit to transmitted to scratch_reg2
//	MOV     scratch_reg4, scratch_reg1         	// move repeat count to scratch_reg4
//	LSL 	scratch_reg2, scratch_reg2, tx_context.bitsLoaded   // shift the bit to be transmitted to expected position
					scratch_reg2 = scratch_reg3 & 0x1;
					scratch_reg4 = scratch_reg1;
					scratch_reg2 <<= tx_context->bitsLoaded;

	// prescale the bit to transmitted
//PRESCALE_NXT_BIT:
					do {
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2
						TX_DATA_reg |= scratch_reg2;
//	LSL 	scratch_reg2, scratch_reg2, 1
						scratch_reg2 <<= 1;
//	SUB     scratch_reg1, scratch_reg1, 1
						scratch_reg1--;
//	QBLE    PRESCALE_NXT_BIT, scratch_reg1, 1
					} while (scratch_reg1 >= 1);

	// write back to memory
//	ADD 	tx_context.bitsLoaded, tx_context.bitsLoaded, scratch_reg4
					tx_context->bitsLoaded += scratch_reg4;

//	ADD 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, scratch_reg4
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
					suart_ch_regs->Chn_TxRxRepeatDoneCtr += scratch_reg4;
					*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

	// get the prescalar value
//	MOV    scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//	AND    scratch_reg1, scratch_reg2, suart_ch_regs.Chn_Config1
					scratch_reg1 = suart_ch_regs->Chn_Config1 & SUART_CTRL_PRE_SCALAR_MASK;

	//if bit has been transmitted prescaler times, fall through and updated the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
//	QBGT    CHK_MORE_PRESCALAR, suart_ch_regs.Chn_TxRxRepeatDoneCtr, scratch_reg1   //if (Chn_Config1.PreScaller == ChnTxRxRepeatDoneCntr)
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= scratch_reg1) {

//TX_DONE_CNTR6:  // rename to TX_BIT_DONE_CNTR
	// Write Chn_TxRxBitsDoneCtr
//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
						suart_ch_regs->Chn_TxRxBitsDoneCtr++;
						*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

	// Write Chn_TxRxRepeatDoneCtr
//	XOR 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
						suart_ch_regs->Chn_TxRxRepeatDoneCtr = 0;
						*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

//CHK_MORE_PRESCALAR:   //rename this label to CHK_TX_DATA_REG_FULL
					}
				}
//	QBGT 	LOAD_BITS_LOOP_FOR6, tx_context.bitsLoaded, 0x10  //if (bitsLoaded < 16), next bit can be loaded in TX_DATA_reg
			} while (tx_context->bitsLoaded < 16);
//	CALL 	TRANSMIT_PRESCALED_DATA				  // TX_DATA_reg is full, transmit the data
//	JMP     TxInterruptServiceRequestHndlr
			transmit_prescaled_data(tx_context, &TX_DATA_reg);
			goto TxInterruptServiceRequestHndlr;

	// transmit the bits from start bit that can be transmitted from present character that is to transmitted
//XMIT_FIRST_3BIT:
		}
//	AND 	scratch_reg3, scratch_reg3, 0x7			// copy the first 3 bits to be transmitted
//	MOV 	scratch_reg2, 12        			//number of times the byte loop is to be looped
//	XOR 	TX_DATA_reg.w0,TX_DATA_reg.w0,TX_DATA_reg.w0    //Clear TX_DATA_reg.w0
//        XOR     scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2
//	CALL	BYTE_LOOP
		scratch_reg3 &= 0x7;
		scratch_reg2 = 12;
		TX_DATA_reg = 0;
		scratch_8bit_reg2 = 0;
		byte_loop(suart_ch_regs, scratch_reg2, scratch_reg3, scratch_8bit_reg2, &TX_DATA_reg);

//	MOV	    scratch_reg1, 0x4  				// Repeat last bit by 4 times
		scratch_reg1 = 0x4;

//	AND     scratch_reg2, scratch_reg3, 0x1			//copy the third bit to scratch_reg2
		scratch_reg2 = scratch_reg3 & 0x1;

//	LSL 	scratch_reg2, scratch_reg2, 0xc      	//shift the bit to expected place i.e. bit ps 12
		scratch_reg2 <<= 0xc;

	// prescale the last bit 4 times
//PRESCALE_LAST_4BIT:
		do {
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2
			TX_DATA_reg |= scratch_reg2;
//	LSL 	scratch_reg2, scratch_reg2, 1
			scratch_reg2 <<= 1;
//	SUB     scratch_reg1, scratch_reg1, 1
			scratch_reg1--;
//	QBLE    PRESCALE_LAST_4BIT, scratch_reg1, 1
		} while (scratch_reg1 >= 1);

//	CALL    TRANSMIT_PRESCALED_DATA
		transmit_prescaled_data(tx_context, &TX_DATA_reg);

//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 2    //Updating number of bits written
	// Write To RAM number of Bits Transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
		suart_ch_regs->Chn_TxRxBitsDoneCtr += 2;
		*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	ADD 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 4 //Updating number of bits written
	// Write To RAM Write Repeat done counter to RAM
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
		suart_ch_regs->Chn_TxRxRepeatDoneCtr += 4;
		*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

//	JMP     TxInterruptServiceRequestHndlr
		goto TxInterruptServiceRequestHndlr;
//******************************************** PRE_SCALAR6 ENDs **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR12 Starts *******************************************
//PRE_SCALAR12:
	case PRE_SCALAR_12:
//	QBGT 	XMIT_FIRST_2BIT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr >= 1) {
//	JMP 	GENERIC_TRANSMIT_BLOCK
			goto GENERIC_TRANSMIT_BLOCK;

//XMIT_FIRST_2BIT:
		}
//	AND 	scratch_reg3, scratch_reg3, 0x3		// copy the first two bit to be loaded in  TX_DATA_reg
		scratch_reg3 *= 0x3;
//	MOV 	scratch_8bit_reg2, 0x0      	 	// To Left shift each copied data bit
		scratch_8bit_reg2 = 0x0;
//	MOV 	scratch_reg2, 12        		//Keep track of byte_loop loop count
		scratch_reg2 = 12;
//	XOR     scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2
		scratch_8bit_reg2 = 0;
//	CALL	BYTE_LOOP
		byte_loop(suart_ch_regs, scratch_reg2, scratch_reg3, scratch_8bit_reg2, &TX_DATA_reg);

//	AND     scratch_reg2, scratch_reg2, 0x1				//copy the next bit to prescaled
		scratch_reg2 = scratch_reg2 & 0x1;
//	MOV	    scratch_reg1, 4						//counter to prescale second bit by 4
		scratch_reg1 = 4;
//	LSL 	scratch_reg2, scratch_reg2, 0xC				//shift the bit to desired position
		scratch_reg2 <<= 0xc;
//PRESCALE_4BIT:
		do {
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2
			TX_DATA_reg |= scratch_reg2;
//	LSL 	scratch_reg2, scratch_reg2, 1
			scratch_reg2 <<= 1;
//	SUB     scratch_reg1, scratch_reg1, 1
			scratch_reg1--;
//	QBLE    PRESCALE_4BIT, scratch_reg1, 1
		} while (scratch_reg1 >= 1);

//   	CALL 	TRANSMIT_PRESCALED_DATA
		transmit_prescaled_data(tx_context, &TX_DATA_reg);

//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1    //Updating number of bits written
	// Write To RAM number of Bits Transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
		suart_ch_regs->Chn_TxRxBitsDoneCtr++;
		*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	ADD 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 4 //Updating number of bits written
	// Write To RAM number of Bits Repeated
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
		suart_ch_regs->Chn_TxRxRepeatDoneCtr += 4;
		*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

//	JMP     TxInterruptServiceRequestHndlr
		goto TxInterruptServiceRequestHndlr;
//******************************************** PRE_SCALAR12 ENDs *********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR16 Starts *******************************************
//PRE_SCALAR16:
	case PRE_SCALAR_16:
//	QBGT 	XMIT_FIRST_16, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr >= 1) {
//	JMP 	GENERIC_TRANSMIT_BLOCK
			goto GENERIC_TRANSMIT_BLOCK;

//XMIT_FIRST_16:
		}
//	AND 	scratch_reg3, scratch_reg3, 0x2		//// copy the first two bit to be loaded in  TX_DATA_reg
		scratch_reg3 &= 0x2;
//	MOV 	scratch_8bit_reg2, 0x0       		//Left shift each copied data bit
		scratch_8bit_reg2 = 0x0;
//	MOV 	scratch_reg2, 16       			//Keep track of byte_loop loop count
		scratch_reg2 = 16;
//	XOR 	TX_DATA_reg.w0,TX_DATA_reg.w0,TX_DATA_reg.w0    //Clear TX_DATA_reg.w0
		TX_DATA_reg = 0;
//        XOR     scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2
		scratch_8bit_reg2 = 0;
//	CALL	BYTE_LOOP
		byte_loop(suart_ch_regs, scratch_reg2, scratch_reg3, scratch_8bit_reg2, &TX_DATA_reg);

//	CALL 	TRANSMIT_PRESCALED_DATA
		transmit_prescaled_data(tx_context, &TX_DATA_reg);

//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1   //Updating number of bits written
	// Write To RAM number of Bits Transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
		suart_ch_regs->Chn_TxRxBitsDoneCtr++;
		*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	ADD 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0 //Updating number of bits written
	// Write To RAM number of Bits Repeated
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
		suart_ch_regs->Chn_TxRxRepeatDoneCtr += 0;
		*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

//	JMP     TxInterruptServiceRequestHndlr
		goto TxInterruptServiceRequestHndlr;
//******************************************** PRE_SCALAR16 ENDs *********************************************

//======================================================================================================================================

//********************************************* PRE_SCALAR24 Starts ******************************************
//PRE_SCALAR24:
	case PRE_SCALAR_24:
//	QBGT 	XMIT_FIRST_24, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr >= 1) {
//	JMP 	GENERIC_TRANSMIT_BLOCK
			goto GENERIC_TRANSMIT_BLOCK;

//XMIT_FIRST_24:
		}
//	MOV    scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//	AND    scratch_reg1, scratch_reg2, suart_ch_regs.Chn_Config1
		scratch_reg1 = suart_ch_regs->Chn_Config1 & SUART_CTRL_PRE_SCALAR_MASK;
//	SUB    scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxRepeatDoneCtr //Chn_TxRxConfig1.PreScaler - ChnTxRxRepeadDoneCnt
		scratch_reg1 -= suart_ch_regs->Chn_TxRxRepeatDoneCtr;
//	QBLE 	PRESCALE_START_BIT, scratch_reg1, 16		//(Chn_TxRxConfig1.PreScaler - ChnTxRxRepeadDoneCntr >= 16 )
		if (scratch_reg1 < 16) {

//PRESCALE_FIRST_DATA_BIT:

//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1   //Updating number of bits written
			suart_ch_regs->Chn_TxRxBitsDoneCtr++;

//	CALL	READ_TX_DATA
			scratch_reg3 = read_tx_data(suart_ch_regs, tx_context);

	//get the bits to transmitted
//	LSR     scratch_reg3, scratch_reg3, suart_ch_regs.Chn_TxRxBitsDoneCtr
//	AND     scratch_reg2, scratch_reg3, 0x1
			scratch_reg3 >>= suart_ch_regs->Chn_TxRxBitsDoneCtr;
			scratch_reg2 = scratch_reg3 & 0x1;

	//shift the bit to desired bit position
//	LSL     scratch_reg2, scratch_reg2, scratch_reg1
//	RSB 	scratch_reg1,  scratch_reg1, 16
//	MOV		suart_ch_regs.Chn_TxRxRepeatDoneCtr, scratch_reg1
			scratch_reg2 <<= scratch_reg1;
			scratch_reg1 = 16 - scratch_reg1;
			suart_ch_regs->Chn_TxRxRepeatDoneCtr = scratch_reg1;

//PRESCALE_FIRST_DAT_BIT:
			do {
//	OR      TX_DATA_reg.w0, TX_DATA_reg.w0, scratch_reg2
				TX_DATA_reg |= scratch_reg2;
//	LSL 	scratch_reg2, scratch_reg2, 1
				scratch_reg2 <<= 1;
//	SUB     scratch_reg1, scratch_reg1, 1
				scratch_reg1--;
//	QBLE    PRESCALE_FIRST_DAT_BIT, scratch_reg1, 1
			} while (scratch_reg1 >= 1);


//	CALL 	TRANSMIT_PRESCALED_DATA
			transmit_prescaled_data(tx_context, &TX_DATA_reg);

	// Write To RAM number of Bits Transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
			*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

	// Write To RAM Chn_TxRxRepeatDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
			*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;
//	JMP     TxInterruptServiceRequestHndlr
			goto TxInterruptServiceRequestHndlr;

//PRESCALE_START_BIT:
		}
//	MOV 	scratch_reg1, 0x10
//	MOV 	scratch_reg2, 0x10
//	XOR     scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2       //to Left shift each copied data bit
//	CALL 	BITS_LOOP
//	CALL 	TRANSMIT_PRESCALED_DATA
		scratch_reg1 = 0x10;
		scratch_reg2 = 0x10;
		scratch_8bit_reg2 = 0;
		bits_loop();
		transmit_prescaled_data(tx_context, &TX_DATA_reg);

//	ADD 	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 16 // Update number of bits written
	// Write To RAM number of Bits Repeated
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
//	JMP     TxInterruptServiceRequestHndlr
		suart_ch_regs->Chn_TxRxRepeatDoneCtr += 16;
		*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;
		goto TxInterruptServiceRequestHndlr;

//************************************************ PRE_SCALAR24 ENDs *****************************************
	}
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

LOAD_TX_COMMON_INFO:
	// Load the TX Format Address for the specific channel
//	QBEQ    LOAD_TX_FORMAT_ADDRESS_DONE, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY
	if (suart_global.pru_rx_tx_mode != PRU_MODE_TX_ONLY) {
//	CALL  	LOAD_TX_FORMAT_ADDRESS
		load_tx_format_address(suart_ch_info, tx_context);

//LOAD_TX_FORMAT_ADDRESS_DONE:
	}
	//  Load the mapped SR and XBUF address mapped to channel
//	JMP 	LOCATE_SR_XBUF_SRCTL
	locate_sr_xbuf_srctl(suart_ch_regs, suart_tx_ch, tx_context);

//LOCATE_SR_XBUF_SRCTL_DONE:
	// Format the data if required
//	JMP    	CHK_TX_DATA_FORMAT
	chk_tx_data_format(suart_ch_regs, suart_tx_ch, tx_context, &TX_DATA_reg);

//CHK_TX_DATA_FORMAT_DONE:
//	JMP     ENABLE_TX_SERIALIZER
	goto ENABLE_TX_SERIALIZER;

//****************************************** LOAD_TX_COMMON_INFO: ENDS ***************************************

//======================================================================================================================================

//******************************************** TX_DONE : Starts **********************************************

//======================================================================================================================================
// This routine the cleanup after one character has been transmitted successfully
//======================================================================================================================================

TX_DONE:
//	XOR    	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr
	// Write To RAM number of Bits Transmitted
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
	suart_ch_regs->Chn_TxRxBitsDoneCtr = 0;
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//	ADD 	suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1
//	SBBO    suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBytesDoneCtr)
	suart_ch_regs->Chn_TxRxBytesDoneCtr++;
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBYTESDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBytesDoneCtr;

//	JMP     TxInterruptServiceRequestHndlr
	goto TxInterruptServiceRequestHndlr;

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

TxInterruptServiceRequestHndlr:
	//Retrieve the channel number and load the context base;
//	MOV 	suart_ch_info.curr_ch_base_addr, SUART_CH_BASE_ADDRESS
//	MOV  	suart_ch_info.curr_ch_offset, SUART_CH_BASE_ADDRESS
//	MOV	suart_ch_info.ch_num, 0x00
//	MOV     tx_context.buff_addr, 0x90
	suart_ch_info->curr_ch_base_addr = SUART_CH_BASE_ADDRESS;
	suart_ch_info->curr_ch_offset = SUART_CH_BASE_ADDRESS;
	suart_ch_info->ch_num = 0;
	tx_context->buff_addr = 0x90;

//SERCH_MAPPED_TX_CHN:
	do {
//        ADD     scratch_reg1, suart_ch_info.curr_ch_offset, SUART_CH_TXRXCHNSTATUS_OFFSET
		scratch_reg1 = suart_ch_info->curr_ch_offset + SUART_CH_TXRXCHNSTATUS_OFFSET;
//        LBBO    suart_ch_regs.Chn_Status, suart_ch_info.curr_ch_base_addr, scratch_reg1, SIZE (suart_ch_regs.Chn_Status)  //Load the Channel Cntrl info from Memory to Register
		suart_ch_regs->Chn_Status = *(uint8_t *)(suart_ch_info->curr_ch_base_addr + scratch_reg1);
//        QBBC    NEXT_TX_CHN, suart_ch_regs.Chn_Status, SUART_CH_TXRXCHNSTATUS_BIT
		if (suart_ch_regs->Chn_Status & (1U << SUART_CH_TXRXCHNSTATUS_BIT)) {

//        ADD     scratch_reg1, tx_context.buff_addr, TX_FMT_DATA_TO_TX_CONTEXT_OFFSET
			scratch_reg1 = tx_context->buff_addr + TX_FMT_DATA_TO_TX_CONTEXT_OFFSET;
//        LBBO    scratch_reg2, scratch_reg1, SUART_CH_ASP_XSRCTL_REG_OFFSET, 4
			scratch_reg2 = *(uint32_t *)(scratch_reg1 + SUART_CH_ASP_XSRCTL_REG_OFFSET);
//        LBBO    scratch_reg1, scratch_reg2, #00, 4
			scratch_reg1 = *(uint32_t *)(scratch_reg2);
//        QBBS    MAPPED_TX_CHN_FOUND, scratch_reg1, ASP_SRCTL_XRDY_BIT
			if (scratch_reg1 & (1U << ASP_SRCTL_XRDY_BIT)) goto MAPPED_TX_CHN_FOUND;

//NEXT_TX_CHN:
		}
//        QBEQ    PRU_TX_ONLY_MODE, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY
		if (suart_global.pru_rx_tx_mode != PRU_MODE_TX_ONLY) {

        //TX & RX together. So channel numbers are 0, 2, 4, 6
//        ADD    suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE2
			suart_ch_info->curr_ch_offset += SUART_CH_REGS_SIZE2;
//        ADD    suart_ch_info.ch_num, suart_ch_info.ch_num, 0x02
			suart_ch_info->ch_num += 2;
//        ADD    tx_context.buff_addr, tx_context.buff_addr, SUART_TX_FMT_OFFSET
			tx_context->buff_addr += SUART_TX_FMT_OFFSET;
//        QBGE   SERCH_MAPPED_TX_CHN, suart_ch_info.ch_num, NUM_OF_CHANNELS
			if (suart_ch_info->ch_num <= NUM_OF_CHANNELS) continue;
//        JMP    CORE_LOOP
			goto CORE_LOOP;
//PRU_TX_ONLY_MODE:
		}
        //TX Only ...So channel numbers are contiguous
//        ADD    suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE
		suart_ch_info->curr_ch_offset += SUART_CH_REGS_SIZE;
//        ADD    suart_ch_info.ch_num, suart_ch_info.ch_num, 0x01
		suart_ch_info->ch_num++;
//        ADD	   tx_context.buff_addr, tx_context.buff_addr, 0x2C
		tx_context->buff_addr += 0x2c;
//        QBGE   SERCH_MAPPED_TX_CHN, suart_ch_info.ch_num, NUM_OF_CHANNELS
		if (suart_ch_info->ch_num <= NUM_OF_CHANNELS) continue;
//        JMP    CORE_LOOP
		goto CORE_LOOP;
	} while (true);
MAPPED_TX_CHN_FOUND:

//	LBBO 	suart_ch_regs, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset, SIZE (suart_ch_regs)
//        ADD     suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset
	suart_ch_regs = (void *)(suart_ch_info->curr_ch_base_addr + suart_ch_info->curr_ch_offset);
	suart_ch_info->curr_ch_base_addr += suart_ch_info->curr_ch_offset;

//	QBEQ     PRUx_MODE_TX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY
	if (suart_global.pru_rx_tx_mode != PRU_MODE_TX_ONLY) {

//	QBEQ     CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_INVALID
		if (suart_global.pru_rx_tx_mode == PRU_MODE_INVALID) goto CORE_LOOP;

//PRUx_MODE_TX_ONLY:
		}
//	ADD     scratch_reg1, tx_context.buff_addr, TX_FMT_DATA_TO_TX_CONTEXT_OFFSET
//	LBBO    tx_context,   scratch_reg1, #00, SIZE (tx_context)
	scratch_reg1 = tx_context->buff_addr + TX_FMT_DATA_TO_TX_CONTEXT_OFFSET;
	tx_context = (void *)tx_context->buff_addr;

	// JMP TO TxServiceReqHndlLoop Chn_TxRxBytesDoneCtr is less than Data length
//	LSR 	scratch_reg1, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_DATALEN_SHIFT // 0x8
//	AND	    scratch_reg1, scratch_reg1, SUART_CH_CONFIG2_DATALEN_MASK	//0x0F
//	ADD	    scratch_reg1, scratch_reg1, 0x01
//	QBLT 	TxServiceReqHndlLoop, scratch_reg1, suart_ch_regs.Chn_TxRxBytesDoneCtr
	scratch_reg1 = suart_ch_regs->Chn_Config2 >> SUART_CH_CONFIG2_DATALEN_SHIFT;
	scratch_reg1 &= SUART_CH_CONFIG2_DATALEN_MASK;
	scratch_reg1++;
	if (scratch_reg1 > suart_ch_regs->Chn_TxRxBytesDoneCtr) goto TxServiceReqHndlLoop;

//	QBBS    DECLARE_COMPLETE, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT
	if (!(suart_ch_regs->Chn_TxRxStatus & (1U << SUART_TXRX_READY_BIT))) {
DUMMY_TX:
//	NOT	    scratch_reg1, pZERO
//  	SBBO    scratch_reg1, tx_context.asp_xbuf_reg, 00, 4
		scratch_reg1 = ~0;
		tx_context->asp_xbuf_reg = scratch_reg1;

//	JMP     CORE_LOOP
		goto CORE_LOOP;
//DECLARE_COMPLETE:
	}
	// Set the status in the context area
//	SET    suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_COMPLETE_BIT
//   	CLR    suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT
//        SBBO   suart_ch_regs.Chn_TxRxStatus, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXSTATUS_OFFSET, SIZE (suart_ch_regs.Chn_TxRxStatus)
	suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_TXRX_COMPLETE_BIT);
	suart_ch_regs->Chn_TxRxStatus &= ~(1U << SUART_TXRX_READY_BIT);
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXSTATUS_OFFSET) = suart_ch_regs->Chn_TxRxStatus;

	// Generate the interrupt to the ARM/DSP about the completion
//	LBBO    suart_global.intrMask, pZERO, SUART_GBL_INT_MASK_ADDR, 2
//        QBBC    CORE_LOOP, suart_global.intrMask, suart_ch_info.ch_num
	suart_global.intrMask = *(uint16_t *)(0 + SUART_GBL_INT_MASK_ADDR);
	if (!(suart_global.intrMask & (1U << suart_ch_info->ch_num))) goto CORE_LOOP;

//	LBBO    suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, 2
//	SET	suart_global.intrStatus, suart_global.intrStatus, suart_ch_info.ch_num
//	SBBO    suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, 2
//	JMP     PRU_TO_HOST_INTERRUPT
	suart_global.intrStatus = *(uint16_t *)(0 + SUART_GBL_INT_STATUS_ADDR);
	suart_global.intrStatus |= (1U << suart_ch_info->ch_num);
	*(uint16_t *)(0 + SUART_GBL_INT_STATUS_ADDR) = suart_global.intrStatus;
	goto PRU_TO_HOST_INTERRUPT;

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

CORE_LOOP:
//	QBEQ    CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_INVALID
	if (suart_global.pru_rx_tx_mode == PRU_MODE_INVALID) goto CORE_LOOP;

//ARM_DSP_EVENT:
//	QBEQ    CORE_LOOP_PRU1, suart_global.pru_id, 1
	if (suart_global.pru_id != 1) {

//CORE_LOOP_PRU0:
        // wait for the hostEventStatus to get set. Loop till then
//        WBS     hostEventStatus, ARM_TO_PRU0_INT
		while (!(__R31 & (1U << ARM_TO_PRU0_INT))) { }

        // Read the PRUINTC register to know if the event is from ARM/DSP. If yes, then branch
//        MOV   scratch_reg2, SRSR2_OFFSET
//        LBCO    scratch_reg1, CONST_PRUSSINTC, scratch_reg2, 4
//        QBBS    CHN_SEARCH, scratch_reg1, 0
		if (PRU_INTC.STATESETINT1 & (1U << 0)) goto CHN_SEARCH;

        // Else it is McASP Event. So before proceeding, clear it
//        MOV   scratch_reg1, MCASP_TXRX_EVENT
//        SBCO    scratch_reg1, CONST_PRUSSINTC, SICR_OFFSET, 4
		PRU_INTC.STATIDXCLR = MCASP_TXRX_EVENT;

//        JMP	MCASP_EVENT
	} else {
//CORE_LOOP_PRU1:
        // wait for the hostEventStatus to get set. Loop till then
//        WBS     hostEventStatus, ARM_TO_PRU1_INT
//	LBCO 	scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4
//	QBBC    CHN_SEARCH, scratch_reg1, BIT_5
		while (!(__R31 & (1U << ARM_TO_PRU1_INT))) { }
		if (!MCASP0.XSTAT_bit.XDATA) goto CHN_SEARCH;

	// Clear the event here and go to Transmit processing
//        MOV   scratch_reg1, PRU0_TO_PRU1_EVENT
//        SBCO    scratch_reg1, CONST_PRUSSINTC, SICR_OFFSET, 4
//        JMP     TxInterruptServiceRequestHndlr
		PRU_INTC.STATIDXCLR = PRU0_TO_PRU1_EVENT;
		goto TxInterruptServiceRequestHndlr;
	}
MCASP_EVENT:
        // Check for RX interrupt first
        // If TX only PRU Skip RSTAT Check
//	QBEQ    MCASP_TX_EVNT, suart_global.pru_rx_tx_mode, PRU_MODE_TX_ONLY
	if (suart_global.pru_rx_tx_mode != PRU_MODE_TX_ONLY) {

	// if the PRU is RX only mode, then check if the XSTAT is set. If so, raise event to PRU1 and proceed
//	QBNE    RX_TX_PROCESS, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY
		if (suart_global.pru_rx_tx_mode == PRU_MODE_RX_ONLY) {
//	LBCO 	scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4
//	QBBC    RX_TX_PROCESS, scratch_reg1, BIT_5
			if (MCASP0.XSTAT_bit.XDATA) {
//        MOV   scratch_reg1, PRU0_TO_PRU1_EVENT
//        SBCO    scratch_reg1, CONST_PRUSSINTC, SISR_OFFSET, 4
				PRU_INTC.STATIDXSET = PRU0_TO_PRU1_EVENT;

//RX_TX_PROCESS:
			}
		}
//	LBCO 	scratch_reg1, MCASP_CONTROL, MCASP_RSTAT, 4
//	QBBS    RxInterruptServiceRequestHndlr, scratch_reg1, BIT_5
		if (MCASP0.RSTAT_bit.RDATA) goto RxInterruptServiceRequestHndlr;

        // Skip the check for XSTAT if we are not Rx/Tx PRU.
	// We don't want the PRU to spin in a tight loop around the McASP register to introduce a delay
//	QBNE    CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_RX_TX_BOTH
		if (suart_global.pru_rx_tx_mode != PRU_MODE_RX_TX_BOTH) goto CORE_LOOP;

//MCASP_TX_EVNT:
	}
//	LBCO 	scratch_reg1, MCASP_CONTROL, MCASP_XSTAT, 4
//	QBBS    TxInterruptServiceRequestHndlr, scratch_reg1, BIT_5
	if (MCASP0.XSTAT_bit.XDATA) goto TxInterruptServiceRequestHndlr;

	// If PRU is both TX/RX, then go back to Core-loop. Else delay to avoid McASP Spins
//	QBEQ    CORE_LOOP, suart_global.pru_rx_tx_mode, PRU_MODE_RX_TX_BOTH
	if (suart_global.pru_rx_tx_mode == PRU_MODE_RX_TX_BOTH) goto CORE_LOOP;

//******************************************** CORE LOOP: Ends ***********************************************

//========================================================================================================================================

//******************************************** CHN_SEARCH: Starts ********************************************

//========================================================================================================================================
//	Retrieve the active channel number that has requested for serviced and
//	load the context base info for that channel
//========================================================================================================================================
CHN_SEARCH:
//        MOV   regVal, 0x00000001
//        LSL     regVal, regVal, suart_global.pru_id
//	MOV   scratch_reg1, SECR2_OFFSET
//	SBCO    regVal, CONST_PRUSSINTC, scratch_reg1, 4
	PRU_INTC.STATCLRINT1 = (1U << suart_global.pru_id);

	//Read Global control register
//	LBBO	suart_global, pZERO, SUART_GBL_CTRL_ADDR, SIZE(suart_global)
	suart_global = *(Suart_Global *)(0 + SUART_GBL_CTRL_ADDR);

	//Retrieve the channel number and load the context base;
//	LDI 	suart_ch_info.curr_ch_base_addr, SUART_CH_BASE_ADDRESS
//	LDI  	suart_ch_info.curr_ch_offset, SUART_CH_BASE_ADDRESS
//	LDI	    suart_ch_info.ch_num, 0x00
//	XOR		tx_context.buff_addr, tx_context.buff_addr, tx_context.buff_addr
//	XOR     suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr
	suart_ch_info->curr_ch_base_addr = SUART_CH_BASE_ADDRESS;
	suart_ch_info->curr_ch_offset = SUART_CH_BASE_ADDRESS;
	suart_ch_info->ch_num = 0;
	tx_context->buff_addr = 0;
	suart_ch_info->rx_context_addr = 0;

//	MOV		tx_context.buff_addr, 0x90
//	MOV     suart_ch_info.rx_context_addr, 0x90
	tx_context->buff_addr = 0x90;
	suart_ch_info->rx_context_addr = 0x90;

//CHN_ACTIVE:
	do {
//	LBBO 	suart_ch_regs.Chn_Cntrl, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset, SIZE (suart_ch_regs.Chn_Cntrl)
//	QBBS 	CHN_SERACH_RTN, suart_ch_regs.Chn_Cntrl, SUART_CTRL_SR_BIT
//	ADD  	suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE
//	ADD  	suart_ch_info.ch_num, suart_ch_info.ch_num, 0x01
//	ADD		tx_context.buff_addr, tx_context.buff_addr, 0x2C
//	ADD		suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr, 0x20
		suart_ch_regs->Chn_Cntrl = *(uint16_t *)(suart_ch_info->curr_ch_base_addr + suart_ch_info->curr_ch_offset);
		if (suart_ch_regs->Chn_Cntrl & (1U << SUART_CTRL_SR_BIT)) break;
		suart_ch_info->curr_ch_offset += SUART_CH_REGS_SIZE;
		suart_ch_info->ch_num++;
		tx_context->buff_addr += 0x2c;
		suart_ch_info->rx_context_addr += 0x20;

	//None of the channel has service request, go back to MainLoop
//	QBLT 	MCASP_EVENT, suart_ch_info.ch_num, NUM_OF_CHANNELS  //check to be verified to boundary condition
		if (suart_ch_info->ch_num > NUM_OF_CHANNELS) goto MCASP_EVENT;
//	JMP 	CHN_ACTIVE
	} while (true);

//CHN_SERACH_RTN:
//	LBBO 	suart_ch_regs, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset, SIZE (suart_ch_regs)
//        ADD     suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset
	suart_ch_regs = (void *)(suart_ch_info->curr_ch_base_addr + suart_ch_info->curr_ch_offset);

//	AND     scratch_reg1.w0, suart_ch_regs.Chn_Cntrl, SUART_CTRL_MODE_MASK
//	QBEQ    TxServiceRequestHndlr, scratch_reg1.w0, SUART_CH_TX_MODE
//	QBEQ    RxServiceRequestHndlr, scratch_reg1.w0, SUART_CH_RX_MODE
//	JMP     CORE_LOOP
	scratch_reg1 = suart_ch_regs->Chn_Cntrl & SUART_CTRL_MODE_MASK;
	if (scratch_reg1 == SUART_CH_TX_MODE) goto TxServiceRequestHndlr;
	if (scratch_reg1 == SUART_CH_RX_MODE) goto RxServiceRequestHndlr;
	goto CORE_LOOP;

//******************************************** CHN_SEARCH: Ends **********************************************

//======================================================================================================================================

//******************************************** PRU_TO_HOST_INTERRUPT[SINGLE_PRU] starts ********************************

//========================================================================================================================================
// 	This routine raises the interrupt to ARM/DSP once the service request given by ARM/DSP is serviced and
// 	Jump to Core loop. The interrupt has been configured on per channel basis i.e. each RX and TX channel will
//	raise separate event to PRU INTC and that will raise an interrupt to ARM/DSP. Depending upon PRU0/PRU1,
//      PRU raises the system events to ARM/DSP. The Mapping for PRU system event on per channel basis is given below.
	// PRU0
	// PRU0 - SYS_EVT34 (host2 - ch2) - SUART Channel 0 	        (SUART0: TX)
	// PRU0 - SYS_EVT35 (host2 - ch3) - SUART Channel 1		(SUART0: RX)
	// PRU0 - SYS_EVT36 (host3 - ch4) - SUART Channel 2		(SUART1: TX)
	// PRU0 - SYS_EVT37 (host3 - ch5) - SUART Channel 3		(SUART1: RX)
	// PRU0 - SYS_EVT38 (host4 - ch2) - SUART Channel 4		(SUART2: Tx)
	// PRU0 - SYS_EVT39 (host4 - ch3) - SUART Channel 5		(SUART2: RX)
	// PRU0 - SYS_EVT40 (host5 - ch4) - SUART Channel 6		(SUART3: TX)
	// PRU0 - SYS_EVT41 (host5 - ch5) - SUART Channel 7		(SUART3: RX)

	// PRU1
	// PRU1 - SYS_EVT42 (host6 - ch6) - SUART Channel 8		(SUART4: TX)
	// PRU1 - SYS_EVT43 (host6 - ch7) - SUART Channel 9		(SUART4: RX)
	// PRU1 - SYS_EVT44 (host7 - ch8) - SUART Channel 10	        (SUART5: TX)
	// PRU1 - SYS_EVT45 (host7 - ch9) - SUART Channel 11	        (SUART5: RX)
	// PRU1 - SYS_EVT46 (host8 - ch10) - SUART Channel 12	        (SUART6: TX)
	// PRU1 - SYS_EVT47 (host8 - ch11) - SUART Channel 13	        (SUART6: RX)
	// PRU1 - SYS_EVT48 (host9 - ch12) - SUART Channel 14	        (SUART7: TX)
	// PRU1 - SYS_EVT49 (host9 - ch13) - SUART Channel 15	        (SUART7: RX)
//======================================================================================================================================
PRU_TO_HOST_INTERRUPT:
//	LDI	scratch_reg1, 0
	scratch_reg1 = 0;

//	QBEQ	EVTOUT_PRU0_EVENTS, suart_global.pru_id, 0
//	QBEQ	EVTOUT_PRU1_EVENTS, suart_global.pru_id, 1
	switch (suart_global.pru_id) {

//EVTOUT_PRU0_EVENTS:
	case 0:
	//storing the counter value
//	ADD	scratch_reg1, scratch_reg1, SYS_EVT_34
//	JMP	EVTOUT_SYSEVT_INIT
		scratch_reg1 += SYS_EVT_34;
		break;

//EVTOUT_PRU1_EVENTS:
	case 1:
//	ADD	scratch_reg1, scratch_reg1, SYS_EVT_42
		scratch_reg1 += SYS_EVT_42;
		break;
	}
//EVTOUT_SYSEVT_INIT:
//	ADD     scratch_reg1, scratch_reg1, suart_ch_info.ch_num
	scratch_reg1 += suart_ch_info->ch_num;

//EVTOUT_GEN:
   	//Clear SYS_EVTn
//	SBCO    scratch_reg1, CONST_PRUSSINTC, SICR_OFFSET, 4
	PRU_INTC.STATIDXCLR = scratch_reg1;

	// Enable SYS_EVTn system interrupt
//	SBCO    scratch_reg1, CONST_PRUSSINTC, EISR_OFFSET,  4
	PRU_INTC.ENIDXSET = scratch_reg1;

        //Generate SYS_EVTn by event out mapping
//   	MOV 	r31.w0, scratch_reg1.w0
	__R31 = scratch_reg1;

//	JMP  	MCASP_EVENT
	goto MCASP_EVENT;

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
// 	This routine performs the basic initialization and clearing of various registers for RX and loads the configuration info
// 	from PRU RAM to registers for RX channel that is being serviced. It calculates and saves serializer and r_buf address mapped
// 	to RX channel to RX context area so that each time it is not calculated again and is being directly read from TX Context Area
//	and activates the RX serializer if it is disabled.
//========================================================================================================================================

RxServiceRequestHndlr:
	// load the max RX TRIES before time out
//	LBBO    MAX_RX_TIMEOUT_TRIES, pZERO, MAX_RX_TIMEOUT_TRIES_OFFSET, 2
	MAX_RX_TIMEOUT_TRIES = *(uint16_t *)(0 + MAX_RX_TIMEOUT_TRIES_OFFSET);

	//read interrupt status register
//        LBBO    suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, 2
	suart_global.intrStatus = *(uint16_t *)(0 + SUART_GBL_INT_STATUS_ADDR);

//	CLR	    suart_global.intrStatus, suart_global.intrStatus, suart_ch_info.ch_num
	suart_global.intrStatus &= !(1U << suart_ch_info->ch_num);

	//write interrupt status register
//   	SBBO    suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, 2
	*(uint16_t *)(0 + SUART_GBL_INT_STATUS_ADDR) = suart_global.intrStatus;

	//Clear Service Request
//	CLR  	suart_ch_regs.Chn_Cntrl, SUART_CTRL_SR_BIT
//	SBBO 	suart_ch_regs.Chn_Cntrl,suart_ch_info.curr_ch_base_addr, SUART_CH_CTRL_OFFSET, SIZE (suart_ch_regs.Chn_Cntrl)
	suart_ch_regs->Chn_Cntrl &= ~(1U << SUART_CTRL_SR_BIT);
	*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_CTRL_OFFSET) = suart_ch_regs->Chn_Cntrl;

	// clear timeout flag
//	CLR    suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_TIMEOUT_BIT
	suart_ch_regs->Chn_TxRxStatus &= ~(1U << SUART_RX_TIMEOUT_BIT);

	// Set the TXRX_READY_BIT
//	SET  	suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT ;
	suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_TXRX_READY_BIT);

	// update the RX Status Register
//	SBBO 	suart_ch_regs.Chn_TxRxStatus, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXSTATUS_OFFSET, SIZE (suart_ch_regs.Chn_TxRxStatus)
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXSTATUS_OFFSET) = suart_ch_regs->Chn_TxRxStatus;

        // Set the SUART_CH_TXRXCHNSTATUS_BIT to indicate the channel being active
//        SET     suart_ch_regs.Chn_Status, SUART_CH_TXRXCHNSTATUS_BIT
//        SBBO 	suart_ch_regs.Chn_Status, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXCHNSTATUS_OFFSET, SIZE (suart_ch_regs.Chn_Status)
	suart_ch_regs->Chn_Status |= (1U << SUART_CH_TXRXCHNSTATUS_BIT);
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXCHNSTATUS_OFFSET) = suart_ch_regs->Chn_Status;

//RX_CONTEXT_INIT:

//	QBEQ     PRUxxx_MODE_RX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY
	if (suart_global.pru_rx_tx_mode != PRU_MODE_RX_ONLY) {

        // Load RX Context Base Address corresponding to Active RX Channel
//	CALL 	LOAD_RX_CONTEXT_ADDRESS
		load_rx_context_address(suart_ch_info);
	}
//PRUxxx_MODE_RX_ONLY:
	//Calculating the specific SRCTL and R_BUF register offset.
//	AND 	scratch_reg1, suart_ch_regs.Chn_Cntrl.b1, SUART_CTRL_SERIALIZER_MASK
//	LSL 	scratch_reg1, scratch_reg1, SUART_CTRL_SRCTL_BIT_SHIFT
	scratch_reg1 = ((uint8_t *)suart_ch_regs->Chn_Cntrl)[1] & SUART_CTRL_SERIALIZER_MASK;
	scratch_reg1 <<= SUART_CTRL_SRCTL_BIT_SHIFT;

	// Storing SRCTL register address in RX Context Area Region
//	MOV32  	rx_context.asp_rsrctl_reg, MCASP_SRCTL_BASE
//	ADD 	rx_context.asp_rsrctl_reg, rx_context.asp_rsrctl_reg, scratch_reg1
	rx_context->asp_rsrctl_reg = (uint32_t)&MCASP0.SRCTL0;
	rx_context->asp_rsrctl_reg += scratch_reg1;

	//storing asp_rsrctl_reg in RX Context Address Region
//	SBBO    rx_context.asp_rsrctl_reg, suart_ch_info.rx_context_addr, SUART_CH_ASP_RSRCTL_REG, SIZE (rx_context.asp_rsrctl_reg)
	*(uint32_t *)(suart_ch_info->rx_context_addr + SUART_CH_ASP_RSRCTL_REG) = rx_context->asp_rsrctl_reg;

	//Store RBuf Address in RX Context Region
//	MOV32  	rx_context.asp_rbuf_reg, MCASP_RBUF_BASE
//	ADD 	rx_context.asp_rbuf_reg, rx_context.asp_rbuf_reg, scratch_reg1
	rx_context->asp_rbuf_reg = (uint32_t)&MCASP0.RBUF0;
	rx_context->asp_rbuf_reg += scratch_reg1;

	// storing asp_rbuf_reg in RX context  address region
//	SBBO    rx_context.asp_rbuf_reg, suart_ch_info.rx_context_addr, SUART_CH_ASP_RBUF_REG, SIZE (rx_context.asp_rbuf_reg)
	*(uint32_t *)(suart_ch_info->rx_context_addr + SUART_CH_ASP_RBUF_REG) = rx_context->asp_rbuf_reg;

	// Load the Context info specific to Current RX channel from memory to registers
////	LBBO   	rx_context,  suart_ch_info.rx_context_addr, #00, SIZE (rx_context)

	// Clear the RX timeout counter
//	XOR     rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr
//	SBBO    rx_context.rx_timeout_cntr, suart_ch_info.rx_context_addr, SUART_CH_RX_TIMEOUT_CNTR_OFFSET, SIZE(rx_context.rx_timeout_cntr)
	rx_context->rx_timeout_cntr = 0;
	*(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RX_TIMEOUT_CNTR_OFFSET) = rx_context->rx_timeout_cntr;

	// Activate RX serializer,
//	LBBO	scratch_reg2, rx_context.asp_rsrctl_reg, #00, 4
//	AND     scratch_reg2, scratch_reg2, 0x3
//	QBEQ    CLR_RSTAT, scratch_reg2, 0x2           // Check if Serializer is Already Active as Rx if ,yes skip activation
//	MOV   scratch_reg2, 0x000E 					  //  Activate serializer as Receiver
//	SBBO 	scratch_reg2, rx_context.asp_rsrctl_reg, #00, 4
	scratch_reg2 = *(uint32_t *)(rx_context->asp_rsrctl_reg);
	scratch_reg2 &= 0x3;
	if (scratch_reg2 != 2) {
		scratch_reg2 = 0xe;
		*(uint32_t *)(rx_context->asp_rsrctl_reg) = scratch_reg2;
	}
//CLR_RSTAT:
	//Clear the RSTAT  (Overrun, etc)
//	MOV32 	scratch_reg1, 0xFFFF
//	SBCO  	scratch_reg1, MCASP_CONTROL, MCASP_RSTAT, 4
	MCASP0.RSTAT = 0xffff;

//	JMP     MCASP_EVENT
	goto MCASP_EVENT;

//******************************************** RxServiceRequestHndlr: ENDS ***********************************

//========================================================================================================================================

//******************************************** RxInterruptServiceRequestHndlr: Starts ************************

//========================================================================================================================================
//	RxInterruptServiceRequestHndlr is called when there is MCASP RX event, scans the active RX serializer,
//	once the active serializer is found, scans for corresponding RX channel, if it also is found, it loads
//	the context info for RX channel and proceeds for reading the frame transmitted by sender.
//========================================================================================================================================

RxInterruptServiceRequestHndlr:
	//Retrieve the channel number and load the RX context base info corresponding to serializer address in scratch_reg1 and serializer number  in scratch_reg4
//	LDI 	suart_ch_info.curr_ch_base_addr, SUART_CH_BASE_ADDRESS 			//Load the SUART CHANNEL BASE ADDRESS
	suart_ch_info->curr_ch_base_addr = SUART_CH_BASE_ADDRESS;

//	QBEQ     PRUx_MODE_RX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY
	if (suart_global.pru_rx_tx_mode != PRU_MODE_RX_ONLY) {

	//Since the Rx Channel are 1,3,5,7 Load the suart_ch_regs for Rx channel 1 as it is first channel
//	ADD     suart_ch_info.curr_ch_offset, pZERO.w0, SUART_CH_REGS_SIZE
		suart_ch_info->curr_ch_offset = 0 + SUART_CH_REGS_SIZE;

//	LDI	    suart_ch_info.ch_num, 0x01
		suart_ch_info->ch_num = 1;

	// Load the RX channel 1 context address to Ch_info register's rx_context_addr field
//	LDI 	suart_ch_info.rx_context_addr, SUART_CH1_RX_CONTEXT_ADDR
//	JMP     SERCH_ACTIVE_RX_CHN_RX
		suart_ch_info->rx_context_addr = SUART_CH1_RX_CONTEXT_ADDR;

	} else {
//PRUx_MODE_RX_ONLY:
	//Since the Rx Channel are 1,3,5,7 Load the suart_ch_regs for Rx channel 1 as it is first channel
//	LDI     suart_ch_info.curr_ch_offset, 0x00
		suart_ch_info->curr_ch_offset = 0;

//	LDI	    suart_ch_info.ch_num, 0x00
		suart_ch_info->ch_num = 0;

	// Load the RX channel 1 context address to Ch_info register's rx_context_addr field
//	LDI 	suart_ch_info.rx_context_addr, 0x90
		suart_ch_info->rx_context_addr = 0x90;
	}
	// Search for the channel corresponding to serializer number in scratch_reg4
//SERCH_ACTIVE_RX_CHN_RX:
	do {
//        ADD     scratch_reg1, suart_ch_info.curr_ch_offset, SUART_CH_TXRXCHNSTATUS_OFFSET
//	LBBO 	suart_ch_regs.Chn_Status, suart_ch_info.curr_ch_base_addr, scratch_reg1, SIZE (suart_ch_regs.Chn_Status)  //Load the Channel Cntrl info from Memory to Register
//        QBBC    NEXT_RX_CHN, suart_ch_regs.Chn_Status, SUART_CH_TXRXCHNSTATUS_BIT
		scratch_reg1 = suart_ch_info->curr_ch_offset + SUART_CH_TXRXCHNSTATUS_OFFSET;
		suart_ch_regs->Chn_Status = *(uint8_t *)(suart_ch_info->curr_ch_base_addr + scratch_reg1);
		if (suart_ch_regs->Chn_Status & (1U << SUART_CH_TXRXCHNSTATUS_BIT)) {

//        LBBO    scratch_reg1, suart_ch_info.rx_context_addr, SUART_CH_ASP_RSRCTL_REG, 4
//        LBBO    scratch_reg2, scratch_reg1, #0, 4
//	QBBS    ACTIVE_RX_CHN_FOUND, scratch_reg2, ASP_SRCTL_RRDY_BIT
			scratch_reg1 = *(uint32_t *)(suart_ch_info->rx_context_addr + SUART_CH_ASP_RSRCTL_REG);
			scratch_reg2 = *(uint32_t *)(scratch_reg1);
			if (scratch_reg2 & (1U << ASP_SRCTL_RRDY_BIT)) break;
		}
//NEXT_RX_CHN:
//	QBEQ    PRUxx_MODE_RX_ONLY, suart_global.pru_rx_tx_mode, PRU_MODE_RX_ONLY
		if (suart_global.pru_rx_tx_mode != PRU_MODE_RX_ONLY) {

//	ADD  	suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE2 //offset of RX suart_ch_regs
//	ADD  	suart_ch_info.ch_num, suart_ch_info.ch_num, 0x2			// Increment to Next Rx ChanneL number
			suart_ch_info->curr_ch_offset += SUART_CH_REGS_SIZE2;
			suart_ch_info->ch_num += 2;

//	ADD     suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr, RX_CONTEXT_OFFSET  // Increment rx_context_addr by RX_CONTEXT_OFFSET i.e. to next RX channel context address
//	QBGE 	SERCH_ACTIVE_RX_CHN_RX, suart_ch_info.ch_num, NUM_OF_CHANNELS
//	JMP     CORE_LOOP  			//JMP to CORE_LOOP to if no Active RX channel found
			suart_ch_info->rx_context_addr += RX_CONTEXT_OFFSET;
			if (suart_ch_info->ch_num <= NUM_OF_CHANNELS) continue;
			goto CORE_LOOP;
		}
//PRUxx_MODE_RX_ONLY:
//	ADD  	suart_ch_info.curr_ch_offset, suart_ch_info.curr_ch_offset, SUART_CH_REGS_SIZE //offset of RX suart_ch_regs
//	ADD  	suart_ch_info.ch_num, suart_ch_info.ch_num, 0x1			// Increment to Next Rx ChanneL number
//	ADD     suart_ch_info.rx_context_addr, suart_ch_info.rx_context_addr, 0x20  // Increment rx_context_addr by RX_CONTEXT_OFFSET i.e. to next RX channel context address
//	QBGE 	SERCH_ACTIVE_RX_CHN_RX, suart_ch_info.ch_num, NUM_OF_CHANNELS
//	JMP     CORE_LOOP  			//JMP to CORE_LOOP to if no Active RX channel found
		suart_ch_info->curr_ch_offset += SUART_CH_REGS_SIZE;
		suart_ch_info->ch_num++;
		suart_ch_info->rx_context_addr += 0x20;
		if (suart_ch_info->ch_num <= NUM_OF_CHANNELS) continue;
		goto CORE_LOOP;
	} while (true);
//ACTIVE_RX_CHN_FOUND:
//	LBBO 	suart_ch_regs, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset, SIZE (suart_ch_regs)  //Load the suart_ch_regs from Memory to Register
	suart_ch_regs = (void *)(suart_ch_info->curr_ch_base_addr + suart_ch_info->curr_ch_offset);

        // Load the Context info specific to current RX Channel from memory to registers
//	LBBO   	rx_context,  suart_ch_info.rx_context_addr, #00, SIZE (rx_context)
	rx_context = (void *)(suart_ch_info->rx_context_addr);

//        ADD    suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_base_addr, suart_ch_info.curr_ch_offset
	suart_ch_info->curr_ch_base_addr += suart_ch_info->curr_ch_offset;

	//Clear the RSTAT  (Overrun, etc) for Errors
//	LBCO  	scratch_reg1, MCASP_CONTROL, MCASP_RSTAT, 4
//	QBBC    RX_PROCESSING_INIT, scratch_reg1, ASP_RSTAT_RERR_BIT
	if (MCASP0.RSTAT_bit.RERR) {

//	MOV32 	scratch_reg1, 0xFFFF
//	SBCO  	scratch_reg1, MCASP_CONTROL, MCASP_RSTAT, 4
		MCASP0.RSTAT = 0xffff;

	}
//  Start receiving DATA from MAC_ASP's R-Buf corresponding to channel
//RX_PROCESSING_INIT:
//        XOR    mcasp_rbuf_val, mcasp_rbuf_val, mcasp_rbuf_val
  	// Read the content of RBUF
//	LBBO	scratch_reg1, rx_context.asp_rbuf_reg, #00, 4
//        OR      mcasp_rbuf_val, mcasp_rbuf_val, scratch_reg1
	mcasp_rbuf_val = 0;
	scratch_reg1 = *(uint32_t *)(rx_context->asp_rbuf_reg);
	mcasp_rbuf_val |= scratch_reg1;

	do {
	// If start condition is already received then go to reading next bit  otherwise look for start condition
//	QBLT 	READ_CURRENT, suart_ch_regs.Chn_TxRxBitsDoneCtr, 0
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr > 0) break;

	//check Chn_TxRxRepeatDoneCtr, if it is not zero, jump to READ_CURRENT to prescale the start condition
//	QBLT 	READ_CURRENT, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0
		if (suart_ch_regs->Chn_TxRxRepeatDoneCtr > 0) break;

	// If sampling point i.e. sampling_bit_pos is equal to greater than 16 (INVALID_SAMPLING_POINT),
	// start bit transition edge is being detected, fall through to calculate sampling point,
	// otherwise, sampling point is already calculated JUMP to READ_CURRENT
//        QBGE    READ_CURRENT, rx_context.sampling_bit_pos, INVALID_SAMPLING_POINT     //
		if (rx_context->sampling_bit_pos <= INVALID_SAMPLING_POINT) break;

        // Extract timing information by detecting start transition (first left most zero)
//        LMBD    scratch_reg4, scratch_reg1, 0
//        QBGT    START_BIT_TRANSITION, scratch_reg4, ZERO_BIT_NOT_DETECTED // branch if zero: start bit transition detected
//        MOV	    rx_context.sampling_bit_pos, 0xff
//        SBBO    rx_context.sampling_bit_pos,  suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, SIZE (rx_context.sampling_bit_pos)
		scratch_reg4 = __lmbd(scratch_reg1, 0);
		if (scratch_reg4 >= ZERO_BIT_NOT_DETECTED) {
			rx_context->sampling_bit_pos = 0xff;
			*(uint8_t *)(suart_ch_info->rx_context_addr + SUART_CH_SAMPLING_BIT_POS_OFFSET) = rx_context->sampling_bit_pos;

	// RX time out logic
//	QBBC   RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_Config1, RX_TIMEOUT_INTR_MASK
//	QBBC   RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT
//        QBEQ   RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 0
			if (!(suart_ch_regs->Chn_Config1 & (1U << RX_TIMEOUT_INTR_MASK))) goto RxInterruptServiceRequestHndlr;
			if (!(suart_ch_regs->Chn_TxRxStatus & (1U << SUART_TXRX_READY_BIT))) goto RxInterruptServiceRequestHndlr;
			if (!suart_ch_regs->Chn_TxRxBytesDoneCtr) goto RxInterruptServiceRequestHndlr;

	// Read the request count to be received
//	LSR 	scratch_reg1, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_DATALEN_SHIFT //0x8
//	AND	    scratch_reg1, scratch_reg1, SUART_CH_CONFIG2_DATALEN_MASK	//0x0F
//	ADD	    scratch_reg1, scratch_reg1, 0x01			//Since fifo size is 16
//	QBEQ   RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_TxRxBytesDoneCtr, scratch_reg1
			scratch_reg1 = suart_ch_regs->Chn_Config2 >> SUART_CH_CONFIG2_DATALEN_SHIFT;
			scratch_reg1 &= SUART_CH_CONFIG2_DATALEN_MASK;
			scratch_reg1++;
			if (suart_ch_regs->Chn_TxRxBytesDoneCtr == scratch_reg1) goto RxInterruptServiceRequestHndlr;

	// check if time-out is enabled, if yes increment the timeout counter and check if count is equal to MAX_RX_TIMEOUT_TRIES
	// if yes raise the interrupt for time out.
//	ADD    rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr, 1
//	SBBO   rx_context.rx_timeout_cntr, suart_ch_info.rx_context_addr, SUART_CH_RX_TIMEOUT_CNTR_OFFSET, SIZE(rx_context.rx_timeout_cntr)
//	QBGE   RxInterruptServiceRequestHndlr, rx_context.rx_timeout_cntr, MAX_RX_TIMEOUT_TRIES
//	SET    suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_TIMEOUT_BIT
//	CLR    suart_ch_regs.Chn_Config1, suart_ch_regs.Chn_Config1, RX_TIMEOUT_INTR_MASK
//	SBBO   suart_ch_regs.Chn_Config1, suart_ch_info.curr_ch_base_addr, SUART_CH_CONFIG1_OFFSET, SIZE (suart_ch_regs.Chn_Config1)
			rx_context->rx_timeout_cntr++;
			*(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RX_TIMEOUT_CNTR_OFFSET) = rx_context->rx_timeout_cntr;
			if (rx_context->rx_timeout_cntr <= MAX_RX_TIMEOUT_TRIES) goto RxInterruptServiceRequestHndlr;
			suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_RX_TIMEOUT_BIT);
			suart_ch_regs->Chn_Config1 &= ~(1U << RX_TIMEOUT_INTR_MASK);
			*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_CONFIG1_OFFSET) = suart_ch_regs->Chn_Config1;

	// Clear the RX timeout counter
//	XOR     rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr
//	SBBO    rx_context.rx_timeout_cntr, suart_ch_info.rx_context_addr, SUART_CH_RX_TIMEOUT_CNTR_OFFSET, SIZE(rx_context.rx_timeout_cntr)
//	JMP    RX_CHN_INTR
			rx_context->rx_timeout_cntr = 0;
			*(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RX_TIMEOUT_CNTR_OFFSET) = rx_context->rx_timeout_cntr;
			goto RX_CHN_INTR;

	// Calculate the sampling bit position based on the start bit position
	// center = oversampling / 2
	// sampling bit position = start bit position - center
		}
//START_BIT_TRANSITION:
    // clear the rx time out counter
//	XOR     rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr, rx_context.rx_timeout_cntr
//	SBBO    rx_context.rx_timeout_cntr, suart_ch_info.rx_context_addr, SUART_CH_RX_TIMEOUT_CNTR_OFFSET, SIZE(rx_context.rx_timeout_cntr)
		rx_context->rx_timeout_cntr = 0;
		*(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RX_TIMEOUT_CNTR_OFFSET) = rx_context->rx_timeout_cntr;

	// determine the over-sampling rate
//	LSR     scratch_reg2, suart_ch_regs.Chn_Config1, SUART_CH_CONFIG1_OVS_BIT_SHIFT //0xA            //Right Shift by 10
//	AND     scratch_reg2, scratch_reg2, SUART_CH_CONFIG1_OVS_BIT_MASK  //0x3
		scratch_reg2 = suart_ch_regs->Chn_Config1 >> SUART_CH_CONFIG1_OVS_BIT_SHIFT;
		scratch_reg2 &= SUART_CH_CONFIG1_OVS_BIT_MASK;

         // Read Pre Scaler
//        MOV     scratch_reg3, SUART_CTRL_PRE_SCALAR_MASK
//        AND     scratch_reg3, scratch_reg3, suart_ch_regs.Chn_Config1
		scratch_reg3 = SUART_CTRL_PRE_SCALAR_MASK;
		scratch_reg3 &= SUART_CTRL_PRE_SCALAR_MASK;

	//OVER_SAMPLE
//	QBEQ	OVER_SAMPLE_SIZE8BIT, scratch_reg2, OVER_SAMPLING_NONE		// No over Sampling
//	QBEQ	OVER_SAMPLE_SIZE8BIT, scratch_reg2, OVER_SAMPLING_8BIT  	// 8 bit over sampling
//	QBEQ	OVER_SAMPLE_SIZE16BIT, scratch_reg2, OVER_SAMPLING_16BIT	// 16 bit over sampling
		switch (scratch_reg2) {

	// Calculate sampling bit position for 8 bit over sampling
//OVER_SAMPLE_SIZE8BIT:
		case OVER_SAMPLING_NONE:
		case OVER_SAMPLING_8BIT:
		default:
        // if pre-scalar is 2 or 4, sample at end of frame rather than midpoint
//        QBEQ    PS_2_4_SAMPLE_POINT_OS8, scratch_reg3, 2
//        QBEQ    PS_2_4_SAMPLE_POINT_OS8, scratch_reg3, 4
//	SUB 	rx_context.sampling_bit_pos, scratch_reg4, OVR_SAMPL_8BIT_MID_DATA_BIT  //4						//start bit position - center
//	AND	rx_context.sampling_bit_pos, rx_context.sampling_bit_pos, SAMPING_MASK_8_BIT_OVRSAMPLNG	// sampling point
//	SBBO    rx_context.sampling_bit_pos,  suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, SIZE (rx_context.sampling_bit_pos)
//	QBLE	READ_CURRENT, scratch_reg4, OVR_SAMPL_8BIT_MID_DATA_BIT //4         //if Start bit position is equal to/greater than center, sample the start bit in current read, otherwise in next read
//	JMP	RxInterruptServiceRequestHndlr
			if (scratch_reg3 != 2) {
				if (scratch_reg3 != 4) {
					rx_context->sampling_bit_pos = scratch_reg4 - OVR_SAMPL_8BIT_MID_DATA_BIT;
					rx_context->sampling_bit_pos &= SAMPING_MASK_8_BIT_OVRSAMPLNG;
					*(uint8_t *)(suart_ch_info->rx_context_addr + SUART_CH_SAMPLING_BIT_POS_OFFSET) = rx_context->sampling_bit_pos;
					if (scratch_reg4 >= OVR_SAMPL_8BIT_MID_DATA_BIT) break;
					goto RxInterruptServiceRequestHndlr;
				}
			}
//PS_2_4_SAMPLE_POINT_OS8:
//        ADD     rx_context.sampling_bit_pos, scratch_reg4, 1  //4                                             //start bit position - center
//        AND     rx_context.sampling_bit_pos, rx_context.sampling_bit_pos, SAMPING_MASK_8_BIT_OVRSAMPLNG // sampling point
//        SBBO    rx_context.sampling_bit_pos,  suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, SIZE (rx_context.sampling_bit_pos)
//        QBEQ    READ_CURRENT, scratch_reg4, 7 //4         //if Start bit position is equal to/greater than center, sample the start bit in current read, otherwise in next read
//        JMP     RxInterruptServiceRequestHndlr
			rx_context->sampling_bit_pos = scratch_reg4 + 1;
			rx_context->sampling_bit_pos &= SAMPING_MASK_8_BIT_OVRSAMPLNG;
			*(uint8_t *)(suart_ch_info->rx_context_addr + SUART_CH_SAMPLING_BIT_POS_OFFSET) = rx_context->sampling_bit_pos;
			if (scratch_reg4 >= 7) break;
			goto RxInterruptServiceRequestHndlr;

	// Calculate sampling bit position for 16 bit over sampling
//OVER_SAMPLE_SIZE16BIT:
		case OVER_SAMPLING_16BIT:
        // if pre-scalar is 2 or 4, sample at end of frame rather than midpoint
//        QBEQ    PS_2_4_SAMPLE_POINT_OS16, scratch_reg3, 2
//        QBEQ    PS_2_4_SAMPLE_POINT_OS16, scratch_reg3, 4
//	SUB	rx_context.sampling_bit_pos, scratch_reg4, OVR_SAMPL_16BIT_MID_DATA_BIT // start bit position - center
//	AND	rx_context.sampling_bit_pos,  rx_context.sampling_bit_pos, SAMPING_MASK_16_BIT_OVRSAMPLNG		//sampling point
//	SBBO    rx_context.sampling_bit_pos,  suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, SIZE (rx_context.sampling_bit_pos)
//	QBLE	READ_CURRENT, scratch_reg4, OVR_SAMPL_16BIT_MID_DATA_BIT  //if Start bit position is equal to/greater than center, sample the start bit in current read, otherwise in next read
//	JMP	RxInterruptServiceRequestHndlr
			if (scratch_reg3 != 2) {
				if (scratch_reg3 != 4) {
					rx_context->sampling_bit_pos = scratch_reg4 - OVR_SAMPL_16BIT_MID_DATA_BIT;
					rx_context->sampling_bit_pos &= SAMPING_MASK_16_BIT_OVRSAMPLNG;
					*(uint8_t *)(suart_ch_info->rx_context_addr + SUART_CH_SAMPLING_BIT_POS_OFFSET) = rx_context->sampling_bit_pos;
					if (scratch_reg4 >= OVR_SAMPL_16BIT_MID_DATA_BIT) break;
					goto RxInterruptServiceRequestHndlr;
				}
			}

//PS_2_4_SAMPLE_POINT_OS16:
//        ADD     rx_context.sampling_bit_pos, scratch_reg4, 1 // start bit position + 1
//        AND     rx_context.sampling_bit_pos,  rx_context.sampling_bit_pos, SAMPING_MASK_16_BIT_OVRSAMPLNG               //sampling point
//        SBBO    rx_context.sampling_bit_pos,  suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, SIZE (rx_context.sampling_bit_pos)
//        QBEQ    READ_CURRENT, scratch_reg4, 15  //if Start bit position is equal to first bit in frame, sample the start bit in current read, otherwise in next read
//        JMP     RxInterruptServiceRequestHndlr
			rx_context->sampling_bit_pos = scratch_reg4 + 1;
			rx_context->sampling_bit_pos &= SAMPING_MASK_16_BIT_OVRSAMPLNG;
			*(uint8_t *)(suart_ch_info->rx_context_addr + SUART_CH_SAMPLING_BIT_POS_OFFSET) = rx_context->sampling_bit_pos;
			if (scratch_reg4 >= 15) break;
			goto RxInterruptServiceRequestHndlr;
		}
	} while (true);
//READ_CURRENT:
	// scratch_8bit_reg2 holds the information if bit detected is zero if scratch_8bit_reg2= 0, or one if scratch_8bit_reg2 = 1
//	XOR	scratch_8bit_reg2, scratch_8bit_reg2, scratch_8bit_reg2
//	QBBC  	READ_ZERO, scratch_reg1, rx_context.sampling_bit_pos	 //if bit at sampling point is zero jump to READ_ZERO
//	ADD	scratch_8bit_reg2, scratch_8bit_reg2, 1	                // otherwise increment scratch_8bit_reg2 by one as bit detected is one
	scratch_8bit_reg2 = 0;
	if (scratch_reg1 & (1U << rx_context->sampling_bit_pos)) {
		scratch_8bit_reg2++;
	}
//READ_ZERO:
	// We have read the data bit here...
	// If start bit is being received already, then skip the start condition processing.
//	QBLT 	RX_BIT_RECVD, suart_ch_regs.Chn_TxRxBitsDoneCtr, 0
	if (suart_ch_regs->Chn_TxRxBitsDoneCtr > 0) {

		do {
	//(Chn_TxRxBitsDoneCtr == 0)            //No bit is being Received, check if it is start bit
	// if DataBit == 0, i.e. scratch_8bit_reg2 == 0, Jump to Start Condition, else error fall through
//	QBEQ	START_CONDITION, scratch_8bit_reg2, 0
			if (scratch_8bit_reg2 == 0) break;

//	QBEQ 	START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 0   // to be checked with new logic if required or not
			if (suart_ch_regs->Chn_TxRxRepeatDoneCtr == 0) break;

        // Read Pre Scaler
//        MOV     scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//        AND     scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1
			scratch_reg2 = SUART_CTRL_PRE_SCALAR_MASK;
			scratch_reg2 &= suart_ch_regs->Chn_Config1;

//        QBBS    BROKEN_START, scratch_reg2, 0
			if (!(scratch_reg2 & (1 << 0))) {
//PS_2_START:
//        QBNE    PS_4_START, scratch_reg2, 2
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1
//        JMP     BROKEN_START
				if (scratch_reg2 == 2) {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 1) break;
				}
//PS_4_START:
//        QBNE    PS_6_START, scratch_reg2, 4
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 3
//        JMP     BROKEN_START
				else if (scratch_reg2 == 4) {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 3) break;
				}
//PS_6_START:
//        QBNE    PS_12_START, scratch_reg2, 6
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 5
//        JMP     BROKEN_START
				else if (scratch_reg2 == 6) {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 5) break;
				}
//PS_12_START:
//        QBNE    PS_24_START, scratch_reg2, 12
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 10
//        JMP     BROKEN_START
				else if (scratch_reg2 == 12) {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 10) break;
				}
//PS_24_START:
//        QBNE    PS_48_START, scratch_reg2, 24
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 21
//        JMP     BROKEN_START
				else if (scratch_reg2 == 24) {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 21) break;
				}
//PS_48_START:
//        QBNE    PS_96_START, scratch_reg2, 48
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 45
//        JMP     BROKEN_START
				else if (scratch_reg2 == 48) {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 45) break;
				}
//PS_96_START:
//        QBNE    PS_192_START, scratch_reg2, 96
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 91
//        JMP     BROKEN_START
				else if (scratch_reg2 == 96) {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 91) break;
				}
//PS_192_START:
//        QBNE    PS_384_START, scratch_reg2.b0, 0x80
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 182
//        JMP     BROKEN_START
				else if (scratch_reg2 == 192) {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 182) break;
				}
//PS_384_START:
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 255
//        QBLE    START_CONDITION, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 110
				else {
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 255) break;
					if (suart_ch_regs->Chn_TxRxRepeatDoneCtr >= 110) break;
				}
			}
//BROKEN_START:
	// Broken start condition or false alarm, Reset repeat counter		//if DataBit == 1, instead of zero
//	XOR     suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
//	JMP     RxInterruptServiceRequestHndlr
			suart_ch_regs->Chn_TxRxRepeatDoneCtr = 0;
			*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;
			goto RxInterruptServiceRequestHndlr;
		} while (true);
//START_CONDITION:              //else part for NO_REPEAT_DONE  DataBit == 0
	// Increment Repeat Done Counter by One, write back to memory
//	ADD	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
		suart_ch_regs->Chn_TxRxRepeatDoneCtr++;
		*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

	// Read Pre Scaler
//	MOV     scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//	AND     scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1
		scratch_reg2 = SUART_CTRL_PRE_SCALAR_MASK;
		scratch_reg2 &= suart_ch_regs->Chn_Config1;

	// if Repeat Done count is greater than or equal to prescaler, start bit is received, jump to START_BIT_RECIVED,
//	QBGE 	START_BIT_RECIVED, scratch_reg2, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//	JMP	RxInterruptServiceRequestHndlr
		if (scratch_reg2 > suart_ch_regs->Chn_TxRxRepeatDoneCtr) {
			goto RxInterruptServiceRequestHndlr;
		}
	// Start bit is condition Detected properly
//START_BIT_RECIVED:
   	 // Increment Bit Count by One, and write it to memory
//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
	// Reset Repeat Counter, and write it to memory
//	XOR     suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
//	JMP	RxInterruptServiceRequestHndlr
		suart_ch_regs->Chn_TxRxBitsDoneCtr++;
		*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;
		suart_ch_regs->Chn_TxRxRepeatDoneCtr = 0;
		*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;
		goto RxInterruptServiceRequestHndlr;
	}
	// Start Bit has been detected Already, Now the data bit is being received
//RX_BIT_RECVD:
        // scratch_reg1 = repeated data bit
        // scratch_reg2 = Pre Scalar value
        // scratch_reg3 = 50% value of Pre Scalar
        // scratch_reg4 = bits per character

//        XOR     scratch_reg1, scratch_reg1, scratch_reg1
//        XOR     scratch_reg3, scratch_reg3, scratch_reg3
	scratch_reg1 = 0;
	scratch_reg3 = 0;

        // Read Pre Scaler
//        MOV     scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//        AND     scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1
	scratch_reg2 = SUART_CTRL_PRE_SCALAR_MASK;
	scratch_reg2 &= suart_ch_regs->Chn_Config1;

        // Check if Pre Scalar = 1 -- yes, sample each McASP Frame
//        QBEQ    SAMPLE_RX_BIT, scratch_reg2, 1
        // Check if Pre Scalar != 3 -- yes, divide by 2 and check if within stop bit
//        QBNE    SET_SAMPLE_BIT, scratch_reg2, 3
//        LDI     scratch_reg3, 1
//        JMP     SAMPLE_RX_BIT
	if (scratch_reg2 != 1) {
		if (scratch_reg2 == 3) {
			scratch_reg3 = 1;
		}
		else {
//SET_SAMPLE_BIT:
        // Check if Pre Scalar > 12 -- yes, check if within stop bit; no, jump to STORE_RXDATABITS_HOLDREGLOW_PS_LT_64
//        MOV     scratch_reg3, scratch_reg2      // Copy PS to scratch_reg3
//        LSR     scratch_reg3, scratch_reg3, 1   // Divide PS by 2
//        SUB     scratch_reg3, scratch_reg3, 1
			scratch_reg3 = scratch_reg2;
			scratch_reg3 >>= 1;
			scratch_reg3--;
		}
	}
//SAMPLE_RX_BIT:
        // Check if repeated bit is the bit to be sampled
//        QBNE    CHECK_REPEATDONE_CTR, scratch_reg3, suart_ch_regs.Chn_TxRxRepeatDoneCtr
	if (suart_ch_regs->Chn_TxRxRepeatDoneCtr == scratch_reg3) {

        // Check if data bit is zero
//        QBEQ    RX_DATA_BIT_ZERO, scratch_8bit_reg2, 0
//        OR      scratch_reg1, scratch_reg1, 0x1                 // bit received is one, scratch_reg1 = 1
		if (scratch_8bit_reg2) {
			scratch_reg1 |= 0x1;
		}
//RX_DATA_BIT_ZERO:
        // Shift the bit received by Chn_TxRxBitsDoneCtr
//        LSL     scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxBitsDoneCtr
		scratch_reg1 <<= suart_ch_regs->Chn_TxRxBitsDoneCtr;

        //Read the Chn_RxDataHoldReg from Memory
//        LBBO    rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, SIZE (rx_context.Chn_RxDataHoldReg)
		rx_context->Chn_RxDataHoldReg = *(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RXDATAHOLDREG_OFFSET);

        //Write the bit received to Chn_RxDataHoldReg
//        OR      rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, scratch_reg1
		rx_context->Chn_RxDataHoldReg |= scratch_reg1;

        // Write updated Chn_RxDataHoldReg to memory
//        SBBO    rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, SIZE (rx_context.Chn_RxDataHoldReg)
		*(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RXDATAHOLDREG_OFFSET) = rx_context->Chn_RxDataHoldReg;

        // Increment Chn_TxRxRepeatDoneCtr
//        ADD     suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1
//        SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
		suart_ch_regs->Chn_TxRxRepeatDoneCtr++;
		*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

        // If pre-scalar 1, skip stop bit check and increment received counters
//        QBEQ    INCR_RCVD_CTRS, scratch_reg2, 1
		if (scratch_reg2 == 1) goto INCR_RCVD_CTRS;
//CHECK_BITDONE_CTR:
        // Read Bit Per Character and check if current sample is in stop bit
//        AND     scratch_reg4, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_BITS_PER_CHAR_MASK //0xF
//        SUB     scratch_reg4, scratch_reg4, 1
//        QBLE    INCR_RCVD_CTRS, suart_ch_regs.Chn_TxRxBitsDoneCtr, scratch_reg4
//        JMP     RxInterruptServiceRequestHndlr
		scratch_reg4 = suart_ch_regs->Chn_Config2 & SUART_CH_CONFIG2_BITS_PER_CHAR_MASK;
		scratch_reg4--;
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr >= scratch_reg4) goto INCR_RCVD_CTRS;
		goto RxInterruptServiceRequestHndlr;
	}
//CHECK_REPEATDONE_CTR:
        // Increment RepeatDoneCtr
//        ADD     suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1
//        SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
	suart_ch_regs->Chn_TxRxRepeatDoneCtr++;
	*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

////        QBLT    RX_LT_REPEATDONE, scratch_reg3, suart_ch_regs.Chn_TxRxRepeatDoneCtr

	// check if number of bits sampled (Chn_TxRxRepeatDoneCtr) is equal to prescaler (scratch_reg2)
//	QBGE 	INCR_RCVD_CTRS, scratch_reg2, suart_ch_regs.Chn_TxRxRepeatDoneCtr  //(Chn_TxRxRepeatDoneCntr >= Chn_Config1.PreScaller)
////RX_LT_REPEATDONE:
//	JMP	RxInterruptServiceRequestHndlr
	if (scratch_reg2 > suart_ch_regs->Chn_TxRxRepeatDoneCtr) {
		goto RxInterruptServiceRequestHndlr;
	}
	// Increment Receive Counters and store to memory
INCR_RCVD_CTRS:
        // Increment the Data bit Counter
//	ADD 	suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
	suart_ch_regs->Chn_TxRxBitsDoneCtr++;
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

	// Reset the Repeat Done Counter
//	XOR	suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
	suart_ch_regs->Chn_TxRxRepeatDoneCtr = 0;
	*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

	// Read Bit Per Character
//	AND    	scratch_reg2, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_BITS_PER_CHAR_MASK //0xF
	scratch_reg2 = suart_ch_regs->Chn_Config2 & SUART_CH_CONFIG2_BITS_PER_CHAR_MASK;

	// check is (N-1) bit is being received for current data frame, if yes jump to CHK_RECVD_DATA_FRAME
        // if all N bits has been Received Jump to RESET_BITS_CNTR, otherwise receive remaining bits.
        // otherwise, check if parity enabled
//        QBGE    RESET_BITS_CNTR, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr        //(Chn_TxRxBitsDoneCntr >= Chn_Config2.BitsPerChar)
//        SUB     scratch_reg2, scratch_reg2, 1
//        QBEQ    CHK_RECVD_DATA_FRAME, scratch_reg2, suart_ch_regs.Chn_TxRxBitsDoneCtr
	if (scratch_reg2 > suart_ch_regs->Chn_TxRxBitsDoneCtr) {
		scratch_reg2--;
		if (suart_ch_regs->Chn_TxRxBitsDoneCtr != scratch_reg2) {
        // If parity not enabled (none = 0), continue receiving remaining bits.
        // Otherwise, add bit value to bit value summation for use in parity check.
//        LSR     scratch_reg4, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_PARITY_SHIFT
//        AND     scratch_reg4, scratch_reg4, SUART_CH_CONFIG2_PARITY_MASK
//        QBEQ    RxInterruptServiceRequestHndlr, scratch_reg4, 0
			scratch_reg4 = suart_ch_regs->Chn_Config2 >> SUART_CH_CONFIG2_PARITY_SHIFT;
			scratch_reg4 &= SUART_CH_CONFIG2_PARITY_MASK;
			if (scratch_reg4 == 0) goto RxInterruptServiceRequestHndlr;

//RECEIVED_BIT_SUMMATION:
        // Check received bit value. If equals 0, continue receiving remaining bits.
//        SUB     suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
//        LBBO    scratch_reg1, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, SIZE (rx_context.Chn_RxDataHoldReg)
			suart_ch_regs->Chn_TxRxBitsDoneCtr--;
			scratch_reg1 = *(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RXDATAHOLDREG_OFFSET);

//        LSR     scratch_reg1, scratch_reg1, suart_ch_regs.Chn_TxRxBitsDoneCtr
//        QBEQ    RxInterruptServiceRequestHndlr, scratch_reg1, 0
			scratch_reg1 >>= suart_ch_regs->Chn_TxRxBitsDoneCtr;
			if (scratch_reg1 == 0) goto RxInterruptServiceRequestHndlr;

        // Load summed received bits (scratch_regs4) and increment summation by 1
//        LSR     scratch_reg4, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_RX_PARITY_VAL_SHIFT // 0xC
//        AND     scratch_reg4, scratch_reg4, SUART_CH_CONFIG2_RX_PARITY_VAL_MASK  // 0xF
//        ADD     scratch_reg4, scratch_reg4, 1
			scratch_reg4 = suart_ch_regs->Chn_Config2 >> SUART_CH_CONFIG2_RX_PARITY_VAL_SHIFT;
			scratch_reg4 &= SUART_CH_CONFIG2_RX_PARITY_VAL_MASK;
			scratch_reg4++;

        // Store updated summed received bits
//        AND     suart_ch_regs.Chn_Config2.b1, suart_ch_regs.Chn_Config2.b1, 0x0F
//        LSL     scratch_reg4, scratch_reg4, 4
//        OR      suart_ch_regs.Chn_Config2.b1, suart_ch_regs.Chn_Config2.b1, scratch_reg4
//        SBBO    suart_ch_regs.Chn_Config2.b1 , suart_ch_info.curr_ch_base_addr, 5, 2
			((uint8_t *)&suart_ch_regs->Chn_Config2)[1] &= 0xf;
			scratch_reg4 <<= 4;
			((uint8_t *)&suart_ch_regs->Chn_Config2)[1] |= scratch_reg4;
			*(uint8_t *)(suart_ch_info->curr_ch_base_addr + 5) = ((uint8_t *)&suart_ch_regs->Chn_Config2)[1];

        // Continue receiving remaining bits
//        JMP     RxInterruptServiceRequestHndlr
			goto RxInterruptServiceRequestHndlr;
		}
	// if all bits received, verify the Received data frame
//CHK_RECVD_DATA_FRAME:
	//Zero the (16 - Chn_TxRxBitsDoneCntr) Most significant bits in the Chn_RxDataHoldReg.
//	RSB     scratch_reg2, scratch_reg2, 16
//	ADD     scratch_reg2, scratch_reg2, 0x10           // load the count to for number of zero to inserted
//	NOT     scratch_reg1, pZERO  			// Used to Insert  Zero in MSB
		scratch_reg2 = 16 - scratch_reg2;
		scratch_reg2 += 0x10;
		scratch_reg1 = ~0;

//REGHOLD_MSB_ZERO:
	// Prepare the MASK with  ZERO's in bits that do not corresponds to data bits
//	LSR     scratch_reg1, scratch_reg1, scratch_reg2
		scratch_reg1 >>= scratch_reg2;

	// Read the Data hold Reg
//	LBBO    rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, SIZE (rx_context.Chn_RxDataHoldReg)
	// Insert the ZERO's in bits that do  not  corresponds to Data Bits
//	AND  	rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, scratch_reg1
//	SBBO    rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, SIZE (rx_context.Chn_RxDataHoldReg)
		rx_context->Chn_RxDataHoldReg = *(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RXDATAHOLDREG_OFFSET);
		rx_context->Chn_RxDataHoldReg &= scratch_reg1;
		*(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RXDATAHOLDREG_OFFSET) = rx_context->Chn_RxDataHoldReg;

        // removing start bit
//        LSR     rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, 1  // removing start bit
		rx_context->Chn_RxDataHoldReg >>= 1;

        // load the arm memory location address where data is to be written
//        LBBO    suart_ch_regs.ch_TxRxData, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXDATA_OFFSET, SIZE (suart_ch_regs.ch_TxRxData)
		suart_ch_regs->ch_TxRxData = *(uint32_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXDATA_OFFSET);

        // Read Bits Per Character (scratch_reg4)
//        AND     scratch_reg4, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_BITS_PER_CHAR_MASK //0xF
//        SUB     scratch_reg4, scratch_reg4, 2
		scratch_reg4 = suart_ch_regs->Chn_Config2 & SUART_CH_CONFIG2_BITS_PER_CHAR_MASK;
		scratch_reg4 -= 2;

        // Check parity setting (scratch_reg7).  If parity = none (0), branch to BYTES_DONE_CHECK
//        LSR     scratch_reg7, suart_tx_ch.Chn_Config2, SUART_CH_CONFIG2_PARITY_SHIFT
//        AND     scratch_reg7, scratch_reg7, SUART_CH_CONFIG2_PARITY_MASK
//        QBEQ    BYTES_DONE_CHECK, scratch_reg7, 0
		scratch_reg7 = suart_tx_ch->Chn_Config2 >> SUART_CH_CONFIG2_PARITY_SHIFT;
		scratch_reg7 &= SUART_CH_CONFIG2_PARITY_MASK;
		if (!scratch_reg7) {

//PARITY_CHECK:
        // Calculate the data bpc by subtracting 1 (parity bit) from scratch_reg4: the BPC - 2 (BPC - start - stop bits).
//        SUB     scratch_reg4, scratch_reg4, 1
			scratch_reg4--;

        // Load the received parity bit (Chn_RxDataHoldReg >> data bpc)
//        LSR     scratch_reg5, rx_context.Chn_RxDataHoldReg, scratch_reg4
			scratch_reg5 = rx_context->Chn_RxDataHoldReg >> scratch_reg4;

        // Load the calculated parity (or summed received bit) value.
        // If odd parity mode, inverse calculated parity.
//        LSR     scratch_reg6, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_RX_PARITY_VAL_SHIFT // 0xC
//        QBNE    PARITY_COMPARE, scratch_reg7, 1
//        NOT     scratch_reg6, scratch_reg6
			scratch_reg6 = suart_ch_regs->Chn_Config2 >> SUART_CH_CONFIG2_RX_PARITY_VAL_SHIFT;
			if (scratch_reg7 == 1) {
				scratch_reg6 = ~scratch_reg6;
			}
//PARITY_COMPARE:
        // Compare received and calculated parity bits.  If different (=1), handle parity error.  Otherwise, no error.
//        AND     scratch_reg6, scratch_reg6, 1
//        XOR     scratch_reg5, scratch_reg5, scratch_reg6
//        QBEQ    CLEAR_PARITY_BIT, scratch_reg5, 0
			scratch_reg6 &= 1;
			scratch_reg5 ^= scratch_reg6;
			if (scratch_reg5) {
//PARITY_ERROR:
        // Set PE bit in TXRX Status
//        SET     suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_PE_BIT
				suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_RX_PE_BIT);
			}
//CLEAR_PARITY_BIT:
        // removing parity bit
//        CLR    rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, scratch_reg4
			rx_context->Chn_RxDataHoldReg &= ~(1U << scratch_reg4);
		}
//BYTES_DONE_CHECK:
        // Load the Bytes Done counter
//        MOV     scratch_reg1, suart_ch_regs.Chn_TxRxBytesDoneCtr
		scratch_reg1 = suart_ch_regs->Chn_TxRxBytesDoneCtr;

        // check, if two byte offset is required (bits per character greater than 8)
//        QBGE    WRITE_RX_CHAR_TO_MEM, scratch_reg4, SUART_CH_CONFIG2_8BITS_PER_CHAR //8
		if (scratch_reg4 > SUART_CH_CONFIG2_8BITS_PER_CHAR) {

        // calculate the offset in memory where received character is to be written
//        LSL     scratch_reg1, scratch_reg1, 1
			scratch_reg1 <<= 1;
		}
//WRITE_RX_CHAR_TO_MEM:
        // Write the actual data to ARM Memory
//        SBBO    rx_context.Chn_RxDataHoldReg, suart_ch_regs.ch_TxRxData, scratch_reg1, SIZE (rx_context.Chn_RxDataHoldReg)
		*(uint16_t *)(suart_ch_regs->ch_TxRxData + scratch_reg1) = rx_context->Chn_RxDataHoldReg;

        // clear parity summation value
//        AND     suart_ch_regs.Chn_Config2.b1, suart_ch_regs.Chn_Config2.b1, 0x0F
//        SBBO    suart_ch_regs.Chn_Config2.b1 , suart_ch_info.curr_ch_base_addr, 5, 2
		((uint8_t *)&suart_ch_regs->Chn_Config2)[1] &= 0xf;
		*(uint8_t *)(suart_ch_info->curr_ch_base_addr + 5) = ((uint8_t *)&suart_ch_regs->Chn_Config2)[1];


//        JMP     RxInterruptServiceRequestHndlr
		goto RxInterruptServiceRequestHndlr;
	}
//RESET_BITS_CNTR:
	// Check for Framing Error Framing Error
//	SUB     scratch_reg3, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
	scratch_reg3 = suart_ch_regs->Chn_TxRxBitsDoneCtr - 1;

	// Reset bits done counter
//	XOR     suart_ch_regs.Chn_TxRxBitsDoneCtr,suart_ch_regs.Chn_TxRxBitsDoneCtr,suart_ch_regs.Chn_TxRxBitsDoneCtr
//	SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
	suart_ch_regs->Chn_TxRxBitsDoneCtr = 0;
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

	// Get the Presaclar
//	MOV 	scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//	AND     scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1   //scratch_reg2 hold prescalar
	scratch_reg2 = SUART_CTRL_PRE_SCALAR_MASK;
	scratch_reg2 &= suart_ch_regs->Chn_Config1;

        // Extract timing information by detecting start transition (first left most zero)
//        LMBD    scratch_reg4, mcasp_rbuf_val, 0
//        QBEQ    INVALID_SAMPLING_PNT, scratch_reg4, ZERO_BIT_NOT_DETECTED   // branch if zero start bit transition detected
        // determine the over-sampling rate
//        LSR     scratch_reg1, suart_ch_regs.Chn_Config1, SUART_CH_CONFIG1_OVS_BIT_SHIFT //0xA            //Right Shift by 10
//        AND     scratch_reg1, scratch_reg1, SUART_CH_CONFIG1_OVS_BIT_MASK  //0x3                                //OVER_SAMPLE
//        QBEQ    NXT_FRAME_SAMPLING_16BIT_OVS, scratch_reg1, OVER_SAMPLING_16BIT // 16 bit over sampling
	scratch_reg4 = __lmbd(mcasp_rbuf_val, 0);
	if (scratch_reg4 == ZERO_BIT_NOT_DETECTED) goto INVALID_SAMPLING_PNT;
	scratch_reg1 = suart_ch_regs->Chn_Config1 >> SUART_CH_CONFIG1_OVS_BIT_SHIFT;
	scratch_reg1 &= SUART_CH_CONFIG1_OVS_BIT_MASK;
	if (scratch_reg1 != OVER_SAMPLING_16BIT) {

    // Calculate sampling bit position for 8 bit over sampling
//NXT_FRAME_SAMPLING_8BIT_OVS:
//        QBLT    INVALID_SAMPLING_PNT, scratch_reg4, rx_context.sampling_bit_pos // to correct  bit timing error used 4
		if (scratch_reg4 > rx_context->sampling_bit_pos) goto INVALID_SAMPLING_PNT;

//CAL_SAMPL_PNT8:
//        SUB     rx_context.sampling_bit_pos, scratch_reg4, OVR_SAMPL_8BIT_MID_DATA_BIT    //start bit position - center
//        AND     rx_context.sampling_bit_pos, rx_context.sampling_bit_pos, SAMPING_MASK_8_BIT_OVRSAMPLNG // sampling point
//        QBGT    UPDATE_SAMPLING_PNT, scratch_reg4, 4
//        JMP     NXT_FRAME_SAMPLING_PNT
		rx_context->sampling_bit_pos = scratch_reg4 - OVR_SAMPL_8BIT_MID_DATA_BIT;
		rx_context->sampling_bit_pos &= SAMPING_MASK_8_BIT_OVRSAMPLNG;
		if (scratch_reg4 < 4) goto UPDATE_SAMPLING_PNT;
	} else {
    // Calculate sampling bit position for 16 bit over sampling
//NXT_FRAME_SAMPLING_16BIT_OVS:
//        QBLT    INVALID_SAMPLING_PNT, scratch_reg4, rx_context.sampling_bit_pos // to correct  bit timing error used 4
		if (scratch_reg4 > rx_context->sampling_bit_pos) goto INVALID_SAMPLING_PNT;

//CAL_SAMPL_PNT16:
//        SUB     rx_context.sampling_bit_pos, scratch_reg4, OVR_SAMPL_16BIT_MID_DATA_BIT     //start bit position - center
//        AND     rx_context.sampling_bit_pos,  rx_context.sampling_bit_pos, SAMPING_MASK_16_BIT_OVRSAMPLNG  //samplimg point
//        QBGT    UPDATE_SAMPLING_PNT, scratch_reg4, 8
			rx_context->sampling_bit_pos = scratch_reg4 - OVR_SAMPL_16BIT_MID_DATA_BIT;
			rx_context->sampling_bit_pos &= SAMPING_MASK_16_BIT_OVRSAMPLNG;
			if (scratch_reg4 < 8) goto UPDATE_SAMPLING_PNT;
	}
//NXT_FRAME_SAMPLING_PNT:
        // Increment Repeat Done Counter by One, write back to memory
//        ADD     suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, 1
//        SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
	suart_ch_regs->Chn_TxRxRepeatDoneCtr++;
	*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;

        // Read Pre Scaler
//        MOV     scratch_reg2, SUART_CTRL_PRE_SCALAR_MASK
//        AND     scratch_reg2, scratch_reg2, suart_ch_regs.Chn_Config1
	scratch_reg2 = SUART_CTRL_PRE_SCALAR_MASK;
	scratch_reg2 &= suart_ch_regs->Chn_Config1;

        // if Repeat Done count is greater than or equal to prescaler, start bit is received, jump to START_BIT
//        QBLT   UPDATE_SAMPLING_PNT, scratch_reg2, suart_ch_regs.Chn_TxRxRepeatDoneCtr
	if (scratch_reg2 > suart_ch_regs->Chn_TxRxRepeatDoneCtr) goto UPDATE_SAMPLING_PNT;

        // Start bit is condition Detected properly
//START_BIT:
        // Increment Bit Count by One, and write it to memory
//        ADD     suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_regs.Chn_TxRxBitsDoneCtr, 1
//        SBBO    suart_ch_regs.Chn_TxRxBitsDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBITSDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBitsDoneCtr)
	suart_ch_regs->Chn_TxRxBitsDoneCtr++;
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBITSDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBitsDoneCtr;

//        XOR     suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_regs.Chn_TxRxRepeatDoneCtr
//        SBBO    suart_ch_regs.Chn_TxRxRepeatDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXREPEATDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxRepeatDoneCtr)
//        JMP     UPDATE_SAMPLING_PNT
	suart_ch_regs->Chn_TxRxRepeatDoneCtr = 0;
	*(uint16_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXREPEATDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxRepeatDoneCtr;
	goto UPDATE_SAMPLING_PNT;

INVALID_SAMPLING_PNT:
	// Reset the Sampling Point
//	MOV     rx_context.sampling_bit_pos, 0xff
	rx_context->sampling_bit_pos = 0xff;

UPDATE_SAMPLING_PNT:
//        SBBO    rx_context.sampling_bit_pos,  suart_ch_info.rx_context_addr, SUART_CH_SAMPLING_BIT_POS_OFFSET, SIZE (rx_context.sampling_bit_pos)
	*(uint8_t *)(suart_ch_info->rx_context_addr + SUART_CH_SAMPLING_BIT_POS_OFFSET) = rx_context->sampling_bit_pos;

        //Increment Chn_TxRxBytesDoneCtr by one,
//        ADD     suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, 1
//        SBBO    suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBytesDoneCtr)
	suart_ch_regs->Chn_TxRxBytesDoneCtr++;
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBYTESDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBytesDoneCtr;

	//read interrupt mask register
//   	LBBO    suart_global.intrMask, pZERO, SUART_GBL_INT_MASK_ADDR,  2
	suart_global.intrMask = *(uint16_t *)(0 + SUART_GBL_INT_MASK_ADDR);

	//read interrupt status register
//        LBBO    suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, 2
	suart_global.intrStatus = *(uint16_t *)(0 + SUART_GBL_INT_STATUS_ADDR);

	// check for error in received data frame
	// Check for Break Condition Error
//	QBGE    RX_DATA_ZERO, rx_context.Chn_RxDataHoldReg, 0
	if (rx_context->Chn_RxDataHoldReg > 0) {

	// Framing Error: Check if the Bit at Chn_TxRxBitsDoneCtr Bit Position in the Chn_RxDataHoldReg is set;
//   	QBBC    BIT_CLEARD, rx_context.Chn_RxDataHoldReg, scratch_reg3
		if (rx_context->Chn_RxDataHoldReg & (1U << scratch_reg3)) {

	// Reset the Data Hold Reg
//	XOR     rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg, rx_context.Chn_RxDataHoldReg
//	SBBO    rx_context.Chn_RxDataHoldReg, suart_ch_info.rx_context_addr, SUART_CH_RXDATAHOLDREG_OFFSET, SIZE (rx_context.Chn_RxDataHoldReg)
			rx_context->Chn_RxDataHoldReg = 0;
			*(uint16_t *)(suart_ch_info->rx_context_addr + SUART_CH_RXDATAHOLDREG_OFFSET) = rx_context->Chn_RxDataHoldReg;

        // check if parity error set.  (PE bit in TXRX Status)
        // branch to SET_RX_ERR_STAT
//        QBBS    SET_RX_ERR_STAT, suart_ch_regs.Chn_TxRxStatus, SUART_RX_PE_BIT
			if (suart_ch_regs->Chn_TxRxStatus & SUART_RX_PE_BIT) goto SET_RX_ERR_STAT;

	// Read the request count to be received
//	LSR 	scratch_reg1, suart_ch_regs.Chn_Config2, SUART_CH_CONFIG2_DATALEN_SHIFT // 0x8
//	AND     scratch_reg1, scratch_reg1, SUART_CH_CONFIG2_DATALEN_MASK //0x0F
//	ADD     scratch_reg1, scratch_reg1, 0x01
			scratch_reg1 = suart_ch_regs->Chn_Config2 >> SUART_CH_CONFIG2_DATALEN_SHIFT;
			scratch_reg1 &= SUART_CH_CONFIG2_DATALEN_MASK;
			scratch_reg1++;

	// Read the bytes done counter
//	MOV     scratch_reg2, suart_ch_regs.Chn_TxRxBytesDoneCtr
			scratch_reg2 = suart_ch_regs->Chn_TxRxBytesDoneCtr;

	// check if bytes done counter is less than or equal to data len,
	// if yes go to CHK_RX_CMPL_INT and check for raise RX complete intr
//	QBGE    CHK_RX_CMPL_INT, suart_ch_regs.Chn_TxRxBytesDoneCtr, scratch_reg1
			if (suart_ch_regs->Chn_TxRxBytesDoneCtr > scratch_reg1) {

	// if bytes done counter is greater than data len subtract data len from it and
	// check if difference is data len, if yes raise RX complete intr
//	SUB    scratch_reg2, suart_ch_regs.Chn_TxRxBytesDoneCtr, scratch_reg1
				scratch_reg2 = suart_ch_regs->Chn_TxRxBytesDoneCtr - scratch_reg1;
			}
//CHK_RX_CMPL_INT:
	// check if all data frame received or not, if RX request if complete, else receive next data frame
//	QBLT   	RxInterruptServiceRequestHndlr, scratch_reg1, scratch_reg2
			if (scratch_reg1 > scratch_reg2) goto RxInterruptServiceRequestHndlr;

	// All requested frame received raise interrupt to ARM/DSP, set SUART_RX_FIFO_INDX_BIT, clear SUART_TXRX_READY_BIT
//RX_COMPLETE:
	// RX Data is in lower half of FIFO, if bytes done counter is equal to data len
//	CLR    suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_FIFO_INDX_BIT
			suart_ch_regs->Chn_TxRxStatus &= ~(1U << SUART_RX_FIFO_INDX_BIT);

	// Raise the RX interrupt if Chn_TxRxBytesDoneCtr is equal to data len otherwise reset Chn_TxRxBytesDoneCtr and raise Rx interrupt
//	QBEQ   CHK_RX_OVERRUN, suart_ch_regs.Chn_TxRxBytesDoneCtr, scratch_reg1
			if (suart_ch_regs->Chn_TxRxBytesDoneCtr != scratch_reg1) {

	// reset Chn_TxRxBytesDoneCtr if Chn_TxRxBytesDoneCtr is equal to twice the data len
//	XOR    suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_regs.Chn_TxRxBytesDoneCtr
//	SBBO   suart_ch_regs.Chn_TxRxBytesDoneCtr, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXBYTESDONECTR_OFFSET, SIZE (suart_ch_regs.Chn_TxRxBytesDoneCtr)
				suart_ch_regs->Chn_TxRxBytesDoneCtr = 0;
				*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXBYTESDONECTR_OFFSET) = suart_ch_regs->Chn_TxRxBytesDoneCtr;

	// RX data is in upper half of Fifo,if bytes done counter is equal to twice the data len
//	SET    suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_FIFO_INDX_BIT
				suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_RX_FIFO_INDX_BIT);
			}
//CHK_RX_OVERRUN:
//        MOV32   scratch_reg2, SECR2_OFFSET
//        LBCO    scratch_reg1, CONST_PRUSSINTC, scratch_reg2, 4
//        ADD     scratch_reg2,  suart_ch_info.ch_num, 2
//        ADD     scratch_reg2,  suart_ch_info.ch_num, 2
			scratch_reg1 = PRU_INTC.STATCLRINT1;
			scratch_reg2 = suart_ch_info->ch_num + 2;

//OVER_RUN_ERR:
//        QBBC    CHK_RX_READY_BIT, scratch_reg1, scratch_reg2
//        QBBC    CHK_RX_READY_BIT, suart_ch_regs.Chn_Config1, RX_OVER_RUN_MASK
//        SET     suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_RUNER_BIT
//        SET     suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_ERROR_BIT
			if (scratch_reg1 & (1U << scratch_reg2)) {
				if (suart_ch_regs->Chn_Config1 & (1U << RX_OVER_RUN_MASK)) {
					suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_TXRX_RUNER_BIT);
				}
			}
//CHK_RX_READY_BIT:
	// If the receive is not activated from the host, then don't dump the data
//	QBBC   RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_READY_BIT
			if (!(suart_ch_regs->Chn_TxRxStatus & (1U << SUART_TXRX_READY_BIT))) goto RxInterruptServiceRequestHndlr;

//        JMP    RX_CHN_INTR
			goto RX_CHN_INTR;
		}
        // Framing Error Detected, interrupt are masked go to DEACTIVATE_SERIALIZER, other wise update status reg
//BIT_CLEARD:
//	SET     suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_FE_BIT
//        JMP     SET_RX_ERR_STAT
		suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_RX_FE_BIT);
		goto SET_RX_ERR_STAT;
	}
        //Break Condition Error detected, interrupt are masked go to DEACTIVATE_SERIALIZER, other wise update status reg
//RX_DATA_ZERO:
//	SET     suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_RX_BI_BIT
	suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_RX_BI_BIT);

        // Update the Global and Channel Error Status Registers
SET_RX_ERR_STAT:
//	SET	    suart_global.intrStatus, suart_global.intrStatus, GLOBAL_ERR_INTR
//	SET     suart_ch_regs.Chn_TxRxStatus, suart_ch_regs.Chn_TxRxStatus, SUART_TXRX_ERROR_BIT
	suart_global.intrStatus |= (1U << GLOBAL_ERR_INTR);
	suart_ch_regs->Chn_TxRxStatus |= (1U << SUART_TXRX_ERROR_BIT);

        // if global Error interrupt is clear do not raise interrupt
//	QBBC    RxInterruptServiceRequestHndlr, suart_global.intrMask, GLOBAL_ERR_INTR
	if (!(suart_global.intrMask & (1U << GLOBAL_ERR_INTR))) goto RxInterruptServiceRequestHndlr;

        // change command above to branch to parity error check
        // if parity error is not available, may need to branch to RxInterruptServiceRequestHndlr

        // if Framing error status bit for channel is clear look for Parity Error
//        QBBS    FRAMING_ERR, suart_ch_regs.Chn_TxRxStatus, SUART_RX_FE_BIT
	if (!(suart_ch_regs->Chn_TxRxStatus & (1U << SUART_RX_FE_BIT))) {
        // if Parity error status bit for channel is clear look for Break Condition Error
//        QBBC    BREAK_COND_ERR, suart_ch_regs.Chn_TxRxStatus, SUART_RX_PE_BIT
		if (!(suart_ch_regs->Chn_TxRxStatus & (1U << SUART_RX_PE_BIT))) goto BREAK_COND_ERR;
//PARITY_ERR:
//        QBBS    RX_CHN_INTR, suart_ch_regs.Chn_Status, PE_ERR_INTR_MASK
//        JMP     RxInterruptServiceRequestHndlr
		if (suart_ch_regs->Chn_Status & (1U << PE_ERR_INTR_MASK)) goto RX_CHN_INTR;
		goto RxInterruptServiceRequestHndlr;
	}
//FRAMING_ERR:
//        QBBS    RX_CHN_INTR, suart_ch_regs.Chn_Config1, FE_ERR_INTR_MASK
//        JMP     RxInterruptServiceRequestHndlr
	if (suart_ch_regs->Chn_Config1 & (1U << FE_ERR_INTR_MASK)) goto RX_CHN_INTR;
	goto RxInterruptServiceRequestHndlr;

        // if Break Error Mask not set jump to RxInterruptServiceRequestHndlr
BREAK_COND_ERR:
//        QBBC    RxInterruptServiceRequestHndlr, suart_ch_regs.Chn_Config1, BI_ERR_INTR_MASK
	if (!(suart_ch_regs->Chn_Config1 & (1U << BI_ERR_INTR_MASK))) goto RxInterruptServiceRequestHndlr;

//***************

	// Set the Global interrupt status register
RX_CHN_INTR:
//	SET	    suart_global.intrStatus, suart_global.intrStatus, suart_ch_info.ch_num
	suart_global.intrStatus |= (1U << suart_ch_info->ch_num);

	//write interrupt status register
//   	SBBO    suart_global.intrStatus, pZERO, SUART_GBL_INT_STATUS_ADDR, 2
	*(uint16_t *)(0 + SUART_GBL_INT_STATUS_ADDR) = suart_global.intrStatus;

	//Update tx rx status register status
//	SBBO    suart_ch_regs.Chn_TxRxStatus, suart_ch_info.curr_ch_base_addr, SUART_CH_TXRXSTATUS_OFFSET, SIZE (suart_ch_regs.Chn_TxRxStatus)
	*(uint8_t *)(suart_ch_info->curr_ch_base_addr + SUART_CH_TXRXSTATUS_OFFSET) = suart_ch_regs->Chn_TxRxStatus;

	// if interrupt are masked for channel then go to RxInterruptServiceRequestHndlr, otherwise raise the interrupt
//	QBBC    RxInterruptServiceRequestHndlr, suart_global.intrMask, suart_ch_info.ch_num
	if (!(suart_global.intrMask & (1U << suart_ch_info->ch_num))) goto RxInterruptServiceRequestHndlr;

	// Raise the interrupt to ARM/DSP
//	JMP     PRU_TO_HOST_INTERRUPT
	goto PRU_TO_HOST_INTERRUPT;

//******************************************** RxInterruptServiceRequestHndlr: ENDs **************************

//=====================================================================================================================================

//			************************************************************************************************
//									SOFT-UART RECEIVE ROUTINE : ENDS
//			************************************************************************************************

//=====================================================================================================================================

}
