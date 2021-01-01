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
 * Based on PRU_SUART_Emulation.p in lms2012.
 */

#include <stdbool.h>
#include <stdint.h>

#include <pru_rpmsg.h>
#include <pru_virtio_config.h>

#include <am18xx/pru_ctrl.h>
#include <am18xx/pru_intc.h>
#include <am18xx/sys_gpio.h>
#include <am18xx/sys_timer.h>


int main(void) {
	asm (
			"SUART_EMULATION:                                                    \n"
			// Clear the ZERO Register r20
			"    XOR      r20, r20, r20                                          \n"

			//--------------------- McASP TX Initialization - Starts ----------------

			// activate clocks, serializers, state machine and frame sync
			"tx_asp_init1:                                                       \n"
			// Activate the high-transmit clock XHCLKRST
			"    LBCO     &r22, C25, 0xA0, 4                                     \n"
			"    SET      r22, r22, 9                                            \n"
			"    SBCO     &r22, C25, 0xA0, 4                                     \n"

			"tx_asp_init2:                                                       \n"
			"    LBCO     &r22, C25, 0xA0, 4                                     \n"
			"    QBBC     tx_asp_init2, r22, 9                                   \n"

			//Activate the transmit frequency clock XCLKRST
			"    SET      r22, r22, 8                                            \n"
			"    SBCO     &r22, C25, 0xA0, 4                                     \n"

			"tx_asp_init3:                                                       \n"
			"    LBCO     &r22, C25, 0xA0, 4                                     \n"
			"    QBBC     tx_asp_init3, r22, 8                                   \n"

			// Before starting, clear the respective transmitter and receiver status registers by writing 0xffff
			"    LDI      r23.w0, 0xffff & 0xFFFF                                \n"
			"    LDI      r23.w2, 0xffff >> 16                                   \n"
			"    SBCO     &r23, C25, 0xc0, 2                                     \n"

			// Activate serializer, XSRCLR
			"    SET      r22, r22, 10                                           \n"
			"    SBCO     &r22, C25, 0xA0, 4                                     \n"

			"tx_asp_init4:                                                       \n"
			"    LBCO     &r22, C25, 0xA0, 4                                     \n"
			"    QBBC     tx_asp_init4, r22, 10                                  \n"

			// Till now no serializer is activated for TX, so no need to service all active XBUF
			// to avoid underrun errors to be done

			// Actiavte the McASP state machine
			"    SET      r22, r22, 11                                           \n"
			"    SBCO     &r22, C25, 0xA0, 4                                     \n"

			"tx_asp_init5:                                                       \n"
			"    LBCO     &r22, C25, 0xA0, 4                                     \n"
			"    QBBC     tx_asp_init5, r22, 11                                  \n"

			// Activate the MCASP Frame sync
			"    SET      r22, r22, 12                                           \n"
			"    SBCO     &r22, C25, 0xA0, 4                                     \n"

			"tx_asp_init6:                                                       \n"
			"    LBCO     &r22, C25, 0xA0, 4                                     \n"
			"    QBBC     tx_asp_init6, r22, 12                                  \n"

			//----------------------- McASP TX Initialization - Ends ------------------

			//--------------------- McASP RX Initialization - Starts ----------------

			// activate Clocks,Serializers,state machine and frame sync
			"rx_asp_init1:                                                       \n"
			// Activate the high-transmit clock RHCLKRST
			"    LBCO     &r22, C25, 0x60, 4                                     \n"
			"    SET      r22, r22, 1                                            \n"
			"    SBCO     &r22, C25, 0x60, 4                                     \n"

			"rx_asp_init2:                                                       \n"
			"    LBCO     &r22, C25, 0x60, 4                                     \n"
			"    QBBC     rx_asp_init2, r22, 1                                   \n"

			//Activate the transmit frequency clock RCLKRST
			"    SET      r22, r22, 0                                            \n"
			"    SBCO     &r22, C25, 0x60, 4                                     \n"

			"rx_asp_init3:                                                       \n"
			"    LBCO     &r22, C25, 0x60, 4                                     \n"
			"    QBBC     rx_asp_init3, r22, 0                                   \n"

			// Clear RSTAT
			"    LDI      r23, 0xffff                                            \n"
			"    SBCO     &r23, C25, 0x80, 2                                     \n"

			// Activate serializer, RSRCLR
			"    SET      r22, r22, 2                                            \n"
			"    SBCO     &r22, C25, 0x60, 4                                     \n"

			"rx_asp_init4:                                                       \n"
			"    LBCO     &r22, C25, 0x60, 4                                     \n"
			"    QBBC     rx_asp_init4, r22, 2                                   \n"

			// Activate the McASP state machine
			"    SET      r22, r22, 3                                            \n"
			"    SBCO     &r22, C25, 0x60, 4                                     \n"

			"rx_asp_init5:                                                       \n"
			"    LBCO     &r22, C25, 0x60, 4                                     \n"
			"    QBBC     rx_asp_init5, r22, 3                                   \n"

			// Activate the MCASP Frame sync
			"    SET      r22, r22, 4                                            \n"
			"    SBCO     &r22, C25, 0x60, 4                                     \n"

			"rx_asp_init6:                                                       \n"
			"    LBCO     &r22, C25, 0x60, 4                                     \n"
			"    QBBC     rx_asp_init6, r22, 4                                   \n"

			//----------------------- McASP RX Initialization - Ends ------------------

			"LOCAL_INIT:                                                         \n"
			//Read the PRU ID
			"    LBBO     &r23, r20, 0x084, 4                                    \n"
			"    MOV      R3.b0, r23.b0                                          \n"

			// Read the PRU mode
			"    MOV      R3.b1, r23.b1                                          \n"

			//PRU Delay Count in CORE_LOOP
			"    MOV      R3.b2, r23.b2                                          \n"

			//Clear RSTAT
			"    LDI      r23, 0xffff                                            \n"
			"    SBCO     &r23, C25, 0x80, 4                                     \n"

			//Clear XSTAT
			"    LDI      r23, 0xffff                                            \n"
			"    SBCO     &r23, C25, 0xc0, 4                                     \n"

			"    QBEQ     CORE_LOOP, R3.b1, 0x1                                  \n"

			// This Block the Sampling Point with invalid value in RX Context Area
			"    LDI      r23, 0xFF                                              \n"
			"    XOR      r24, r24, r24                                          \n"

			"    QBEQ     PRUxxxx_MODE_RX_ONLY, R3.b1, 0x2                       \n"

			"    LDI      r22, 0x0C0                                             \n"
			"    LDI      r25, 0x50                                              \n"
			"    LDI      r24, 0x1B0                                             \n"
			"    JMP      INIT_SAMPLE_PNT                                        \n"

			"PRUxxxx_MODE_RX_ONLY:                                               \n"
			"    LDI      r22, 0x90                                              \n"
			"    LDI      r25, 0x20                                              \n"
			"    LDI      r24, 0x170                                             \n"

			"INIT_SAMPLE_PNT:                                                    \n"
			"    SBBO     &r23, r22, 22, 1                                       \n"
			"    SBBO     &r20.b0, r22, 23, 1                                    \n"

			"    ADD      r22, r22, r25                                          \n"
			"    QBGE     INIT_SAMPLE_PNT, r22, r24                              \n"

			// JUMP  to CORE_LOOP
			"    JMP      CORE_LOOP                                              \n"


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


			"TxServiceRequestHndlr:                                              \n"
			//read interrupt status regsiter
			"    LBBO     &R2.w2, r20, 0x082, 2                                  \n"

			// clear the channel interrupt status bit
			"    CLR      R2.w2, R2.w2, R10.b2                                   \n"

			//update interrupt status regsiter
			"    SBBO     &R2.w2, r20, 0x082, 2                                  \n"

			//Clear Service Request
			"    CLR      R4.w0, R4.w0, 0x2                                      \n"
			"    SBBO     &R4.w0, R8, 0,  2                                      \n"

			// Set the TXRX_READY_BIT
			"    SET      R5.b2, R5.b2, 0                                        \n"
			"    SBBO     &R5.b2, R8, 6,  1                                      \n"

			// Set SUART_CH_TXRXCHNSTATUS_BIT bit in channel status to indicate the channel active
			"    SET      R5.b3, R5.b3, 7                                        \n"
			"    SBBO     &R5.b3, R8, 7,  1                                      \n"

			// New Tx Request received initialize the Channel specific data and save it in memory
			"    XOR      R7.b0, R7.b0, R7.b0                                    \n"
			"    SBBO     &R7.b0, R8, 12,  1                                     \n"

			// Load channel specific serializer, xbuf, srctl register mapped active channel
			"    JMP      LOAD_TX_COMMON_INFO                                    \n"

			"ENABLE_TX_SERIALIZER:                                               \n"
			//Change the MCASP AXR[n] pin from GPIO mode to MCASP mode of operation
			"    LBCO     &r23, C25, 0x10, 4                                     \n"
			"    AND      r22, R4.b1, 0x0F                                       \n"
			"    CLR      r23, r23, r22                                          \n"
			"    SBCO     &r23, C25, 0x10, 4                                     \n"

			"CLEAR_XSTAT:                                                        \n"
			"    LDI      r22, 0xFFFF                                            \n"
			"    SBCO     &r22, C25, 0xc0, 4                                     \n"
			"    JMP      MCASP_EVENT                                            \n"

//******************************************** TxServiceRequestHndlr Ends ************************************

//=====================================================================================================================================

//******************************************** TxServiceReqHndlLoop Starts ***********************************

//=====================================================================================================================================
//  This routine reads the formated data to be transmitted from formatted data area region mapped to
//  current serviced TX channel and depending upon prescalar value in config1 register, it jumps to that
// 	that prescalar label. This is getting called from TX interrupt handler or when is there new service request for TX.
//=====================================================================================================================================

			"TxServiceReqHndlLoop:                                               \n"
			// Read the Formated byte to transmitted
			"    JAL      r30.w0, READ_TX_DATA                                   \n"

			"    XOR      r28.w0, r28.w0, r28.w0                                 \n"

			// Branch According to Pre-Scalar Value
			"    LDI      r22, 0x03FF                                            \n"
			"    AND      r22, r22, R4.w2                                        \n"

			"    QBEQ     PRE_SCALAR1, r22, 0x1                                  \n"
			"    QBEQ     PRE_SCALAR2, r22, 0x2                                  \n"
			"    QBEQ     PRE_SCALAR4, r22, 0x4                                  \n"
			"    QBEQ     PRE_SCALAR6, r22, 0x6                                  \n"
			"    QBEQ     PRE_SCALAR12, r22, 0xC                                 \n"
			"    QBEQ     PRE_SCALAR16, r22, 0x10                                \n"
			"    QBLE     PRE_SCALAR24, r22, 0x18                                \n"

//******************************************** TxServiceReqHndlLoop ENDS *************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR1 Starts ********************************************

			"PRE_SCALAR1:                                                        \n"
			// copy data to RAM TX_DATA_reg.w0 register from scratch_reg3
			"    MOV      r28.w0, r24                                            \n"

			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			// Increament the Chn_TxRxBytesDoneCtr bye one
			"    ADD      R7.b0, R7.b0, 1                                        \n"
			"    SBBO     &R7.b0, R8, 12,  1                                     \n"

			"    JMP      TxInterruptServiceRequestHndlr                         \n"

//******************************************** PRE_SCALAR1 Ends **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR2 Starts ********************************************

			"PRE_SCALAR2:                                                        \n"
			"    MOV      r22, R7.b1                                             \n"
			"    QBGT     XMIT_FISRT_8BIT, R7.b1, 1                              \n"

			"XMIT_LAST_8BIT:                                                     \n"
			//Last 8 bits to transmitted
			"    LSR      r24, r24, 8                                            \n"

			"    JAL      r30.w0, PRESACLE_TX_DATA                               \n"

			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			"    JMP      TX_DONE                                                \n"

			"XMIT_FISRT_8BIT:                                                    \n"
			"    AND      r24, r24, 0x00FF                                       \n"
			"    JAL      r30.w0, PRESACLE_TX_DATA                               \n"

			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			// Write To RAM number of Bits Transmitted
			// 8 bits transmitted
			"    ADD      R7.b1, R7.b1, 8                                        \n"
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// If bit per character less than 8  // added with start and stop bit in bits per channel
			"    MOV      r22, R5.w0                                             \n"
			"    AND      r22, r22, 0xF                                          \n"
			//check  (Chn_Config2.BitsPerChar <= 8)
			"    QBGE     TX_DONE, r22, 0x8                                      \n"
			"    JMP      TxInterruptServiceRequestHndlr                         \n"

//******************************************** PRE_SCALAR2 ENDs **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR4 Starts ********************************************

			"PRE_SCALAR4:                                                        \n"
			"    MOV      r22, R7.b1                                             \n"
			"    QBGT     XMIT_FIRST_4BIT, r22, 1                                \n"

			"XMIT_NXT_4BIT:                                                      \n"
			//Chn_Config2.BitsPerChar - Chn_TxRxBitsDoneCntr
			"    AND      r23, R5.w0, 0xF                                        \n"
			"    SUB      r23, r23, R7.b1                                        \n"

			// (Chn_Config2.BitsPerChar - Chn_TxRxBitsDoneCntr) > 4, more bits to be transmitted
			"    QBLT     MORE_DATA4, r23, 4                                     \n"

			//transmit last remaining 4 bits
			"    LSR      r24, r24, r22                                          \n"
			"    AND      r24, r24, 0xF                                          \n"

			"    JAL      r30.w0, PRESACLE_TX_DATA                               \n"
			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			"    JMP      CHK_TX_DONE                                            \n"

			"MORE_DATA4:                                                         \n"
			//transmit next 4 bit of present byte being transmitted
			"    LSR      r24, r24, r22                                          \n"
			"    AND      r24, r24, 0xF                                          \n"

			"    JAL      r30.w0, PRESACLE_TX_DATA                               \n"

			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			// Check all bits have been transmitted
			"CHK_TX_DONE:                                                        \n"
			// Updating number of bits written
			"    ADD      R7.b1, R7.b1, 4                                        \n"

			// Write To RAM number of Bits Transmitted
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			"    AND      r23, R5.w0, 0xF                                        \n"

			// check if all bits have been transmitted
			"    QBGE     TX_DONE, r23, R7.b1                                    \n"
			"    JMP      TxInterruptServiceRequestHndlr                         \n"

			// transmit first 4 bit of formated data
			"XMIT_FIRST_4BIT:                                                    \n"
			"    AND      r24, r24, 0xF                                          \n"
			"    JAL      r30.w0, PRESACLE_TX_DATA                               \n"
			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			//Updating number of bits written
			"    ADD      R7.b1, R7.b1, 4                                        \n"

			// Write To RAM number of Bits Transmitted
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			"    JMP      TxInterruptServiceRequestHndlr                         \n"

//******************************************** PRE_SCALAR4 Ends **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR6 Starts ********************************************

			"PRE_SCALAR6:                                                        \n"
			// transmit first 3 bit of formated data
			"    QBGT     XMIT_FIRST_3BIT, R7.b1, 1                              \n"

			"GENERIC_TRANSMIT_BLOCK:                                             \n"
			// initialize the register
			"    LDI      R13.b3, 0x0                                            \n"
			"    XOR      r21.b1, r21.b1, r21.b1                                 \n"

			"LOAD_BITS_LOOP_FOR6:                                                \n"
			"    AND      r23, R5.w0, 0xF                                        \n"

			// transmit the next bits if (ChnTxRxBitsDoneCntr < Chn_Config2.BitsPerChar)
			"    QBLT     XMIT_NXT_3BIT, r23, R7.b1                              \n"

			// transmit the last remaining bits of the present byte if any and updated counters as below
			"XMIT_MORE_BITS:                                                     \n"
			// update the bytes done counter and reset the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
			"    ADD      R7.b0, R7.b0, 1                                        \n"
			"    SBBO     &R7.b0, R8, 12,  1                                     \n"

			"    XOR      R7.b1, R7.b1, R7.b1                                    \n"
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			"    XOR      R7.w2, R7.w2, R7.w2                                    \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			// set the remaining bits to one, if there are no more bits in formated data to send
			// and still there is space in TX_DATA_reg.
			// 16 - bitsLoaded
			"    RSB      r22, R13.b3, 16                                        \n"
			// Load the remaining (16 - bitsLoaded) bits with logic High
			"    XOR      r24, r24, r24                                          \n"
			"    NOT      r24, r24                                               \n"

			// calculate the bit position from where one is to be inserted
			"    RSB      r21.b1, r22, 16                                        \n"
			// CLR scratch_reg2
			"    XOR      r23, r23, r23                                          \n"
			// COPY 1 bit to scratch_reg2 from scratch_reg3
			"    AND      r23, r24, 0x1                                          \n"
			"    LSL      r23, r23, r21.b1                                       \n"

			// Now, set the remaining bits to one in TX_DATA_reg
			"SET_BIT_BIT:                                                        \n"
			"    OR       r28.w0, r28.w0, r23                                    \n"
			"    LSL      r23, r23, 1                                            \n"
			"    SUB      r22, r22, 1                                            \n"
			"    QBLE     SET_BIT_BIT, r22, 1                                    \n"

			"    LDI      R13.b3, 16                                             \n"
			"    JMP      CHK_MORE_PRESCALAR                                     \n"
			"    JMP      TxInterruptServiceRequestHndlr                         \n"

			"XMIT_NXT_3BIT:                                                      \n"
			// if the bitsLoaded in TX_DATA_reg is less than 16 load the next bits
			// (bitsLoaded < 16)
			"    QBLT     CHK_MORE_PRESCALAR, R13.b3, 16                         \n"

			"BIT_LOAD16:                                                         \n"
			// Read Prescalar value
			"    LDI      r23, 0x03FF                                            \n"
			"    AND      r22, r23, R4.w2                                        \n"

			// (16 - bitsLoaded)
			"    RSB      r23, R13.b3, 16                                        \n"
			// (Chn_Config1.PreScaller - ChnTxRxRepeatDoneCntr)
			"    SUB      r22, r22, R7.w2                                        \n"
			"    MIN      r22, r22, r23                                          \n"

			// Read Next Bit
			"    JAL      r30.w0, READ_TX_DATA                                   \n"
			"    LSR      r24, r24, R7.b1                                        \n"

			// copy bit to transmitted to scratch_reg2
			"    AND      r23, r24, 0x1                                          \n"
			// move repeat count to scratch_reg4
			"    MOV      r25, r22                                               \n"
			// shift the bit to be transmitted to expected position
			"    LSL      r23, r23, R13.b3                                       \n"

			// prescale the bit to transmitted
			"PRESCALE_NXT_BIT:                                                   \n"
			"    OR       r28.w0, r28.w0, r23                                    \n"
			"    LSL      r23, r23, 1                                            \n"
			"    SUB      r22, r22, 1                                            \n"
			"    QBLE     PRESCALE_NXT_BIT, r22, 1                               \n"

			// write back to memory
			"    ADD      R13.b3, R13.b3, r25                                    \n"

			"    ADD      R7.w2, R7.w2, r25                                      \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			// get the prescalar value
			"    LDI      r23, 0x03FF                                            \n"
			"    AND      r22, r23, R4.w2                                        \n"

			//if bit has been transmitted prescaler times, fall through and updated the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
			"    QBGT     CHK_MORE_PRESCALAR, R7.w2, r22                         \n"

			// rename to TX_BIT_DONE_CNTR
			"TX_DONE_CNTR6:                                                      \n"
			// Write Chn_TxRxBitsDoneCtr
			"    ADD      R7.b1, R7.b1, 1                                        \n"
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// Write Chn_TxRxRepeatDoneCtr
			"    XOR      R7.w2, R7.w2, R7.w2                                    \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			//rename this label to CHK_TX_DATA_REG_FULL
			"CHK_MORE_PRESCALAR:                                                 \n"
			// if (bitsLoaded < 16), next bit can be loaded in TX_DATA_reg
			"    QBGT     LOAD_BITS_LOOP_FOR6, R13.b3, 0x10                      \n"
			// TX_DATA_reg is full, transmit the data
			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"
			"    JMP      TxInterruptServiceRequestHndlr                         \n"

			// transmit the bits from start bit that can be transmitted from present character that is to transmitted
			"XMIT_FIRST_3BIT:                                                    \n"
			// copy the first 3 bits to be transmitted
			"    AND      r24, r24, 0x7                                          \n"
			// number of times the byte loop is to be looped
			"    LDI      r23, 12                                                \n"
			// Clear TX_DATA_reg.w0
			"    XOR      r28.w0, r28.w0, r28.w0                                 \n"
			"    XOR      r21.b1, r21.b1, r21.b1                                 \n"
			"    JAL      r30.w0, BYTE_LOOP                                      \n"

			// Repeat last bit by 4 times
			"    LDI      r22, 0x4                                               \n"

			// CLR scratch_reg2
			"    XOR      r23, r23, r23                                          \n"
			// copy the third bit to scratch_reg2
			"    AND      r23, r24, 0x1                                          \n"

			// shift the bit to expected place i.e. bit ps 12
			"    LSL      r23, r23, 0xc                                          \n"

			// prescale the last bit 4 times
			"PRESCALE_LAST_4BIT:                                                 \n"
			"    OR       r28.w0, r28.w0, r23                                    \n"
			"    LSL      r23, r23, 1                                            \n"
			"    SUB      r22, r22, 1                                            \n"
			"    QBLE     PRESCALE_LAST_4BIT, r22, 1                             \n"

			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			// Updating number of bits written
			"    ADD      R7.b1, R7.b1, 2                                        \n"
			// Write To RAM number of Bits Transmitted
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// Updating number of bits written
			"    ADD      R7.w2, R7.w2, 4                                        \n"
			// Write To RAM Write Repeat done counter to RAM
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			"    JMP      TxInterruptServiceRequestHndlr                         \n"

//******************************************** PRE_SCALAR6 ENDs **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR12 Starts *******************************************

			"PRE_SCALAR12:                                                       \n"
			"    QBGT     XMIT_FIRST_2BIT, R7.b1, 1                              \n"
			"    JMP      GENERIC_TRANSMIT_BLOCK                                 \n"

			"XMIT_FIRST_2BIT:                                                    \n"
			// copy the first two bit to be loaded in  TX_DATA_reg
			"    AND      r24, r24, 0x3                                          \n"
			// To left shift each copied data bit
			"    LDI      r21.b1, 0x0                                            \n"
			// Keep track of byte_loop loop count
			"    LDI      r23, 12                                                \n"
			"    XOR      r21.b1, r21.b1, r21.b1                                 \n"
			"    JAL      r30.w0, BYTE_LOOP                                      \n"

			// CLR scratch_reg2
			"    XOR      r23, r23, r23                                          \n"
			// copy the next bit to prescaled
			"    AND      r23, r24, 0x1                                          \n"
			// counter to prescale second bit by 4
			"    LDI      r22, 4                                                 \n"
			// shift the bit to desired position
			"    LSL      r23, r23, 0xC                                          \n"

			"PRESCALE_4BIT:                                                      \n"
			"    OR       r28.w0, r28.w0, r23                                    \n"
			"    LSL      r23, r23, 1                                            \n"
			"    SUB      r22, r22, 1                                            \n"
			"    QBLE     PRESCALE_4BIT, r22, 1                                  \n"

			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			// Updating number of bits written
			"    ADD      R7.b1, R7.b1, 1                                        \n"
			// Write To RAM number of Bits Transmitted
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// Updating number of bits written
			"    ADD      R7.w2, R7.w2, 4                                        \n"
			// Write To RAM number of Bits Repeated
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			"    JMP      TxInterruptServiceRequestHndlr                         \n"

//******************************************** PRE_SCALAR12 ENDs *********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR16 Starts *******************************************

			"PRE_SCALAR16:                                                       \n"
			"    QBGT     XMIT_FIRST_16, R7.b1, 1                                \n"
			"    JMP      GENERIC_TRANSMIT_BLOCK                                 \n"

			"XMIT_FIRST_16:                                                      \n"
			// copy the first two bit to be loaded in  TX_DATA_reg
			"    AND      r24, r24, 0x2                                          \n"
			// Left shift each copied data bit
			"    LDI      r21.b1, 0x0                                            \n"
			// Keep track of byte_loop loop count
			"    LDI      r23, 16                                                \n"
			// Clear TX_DATA_reg.w0
			"    XOR      r28.w0, r28.w0, r28.w0                                 \n"
			"    XOR      r21.b1, r21.b1, r21.b1                                 \n"
			"    JAL      r30.w0, BYTE_LOOP                                      \n"

			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			// Updating number of bits written
			"    ADD      R7.b1, R7.b1, 1                                        \n"
			// Write To RAM number of Bits Transmitted
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// Updating number of bits written
			"    ADD      R7.w2, R7.w2, 0                                        \n"
			// Write To RAM number of Bits Repeated
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			"    JMP      TxInterruptServiceRequestHndlr                         \n"

//******************************************** PRE_SCALAR16 ENDs *********************************************

//======================================================================================================================================

//********************************************* PRE_SCALAR24 Starts ******************************************

			"PRE_SCALAR24:                                                       \n"
			"    QBGT     XMIT_FIRST_24, R7.b1, 1                                \n"
			"    JMP      GENERIC_TRANSMIT_BLOCK                                 \n"

			"XMIT_FIRST_24:                                                      \n"
			"    LDI      r23, 0x03FF                                            \n"
			"    AND      r22, r23, R4.w2                                        \n"
			// Chn_TxRxConfig1.PreScaler - ChnTxRxRepeadDoneCnt
			"    SUB      r22, r22, R7.w2                                        \n"
			//(Chn_TxRxConfig1.PreScaler - ChnTxRxRepeadDoneCntr >= 16 )
			"    QBLE     PRESCALE_START_BIT, r22, 16                            \n"

			"PRESCALE_FIRST_DATA_BIT:                                            \n"
			// Clear Scratch reg
			"    XOR      r24, r24, r24                                          \n"
			// Updating number of bits written
			"    ADD      R7.b1, R7.b1, 1                                        \n"

			"    JAL      r30.w0, READ_TX_DATA                                   \n"

			// get the bits to be transmitted
			"    LSR      r24, r24, R7.b1                                        \n"
			"    AND      r23, r24, 0x1                                          \n"

			// shift the bit to desired bit position
			"    LSL      r23, r23, r22                                          \n"
			"    RSB      r22, r22, 16                                           \n"
			"    MOV      R7.w2, r22                                             \n"

			"PRESCALE_FIRST_DAT_BIT:                                             \n"
			"    OR       r28.w0, r28.w0, r23                                    \n"
			"    LSL      r23, r23, 1                                            \n"
			"    SUB      r22, r22, 1                                            \n"
			"    QBLE     PRESCALE_FIRST_DAT_BIT, r22, 1                         \n"

			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			// Write To RAM number of Bits Transmitted
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// Write To RAM Chn_TxRxRepeatDoneCtr
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"
			"    JMP      TxInterruptServiceRequestHndlr                         \n"

			"PRESCALE_START_BIT:                                                 \n"
			"    LDI      r22, 0x10                                              \n"
			"    LDI      r23, 0x10                                              \n"
			// to left shift each copied data bit
			"    XOR      r21.b1, r21.b1, r21.b1                                 \n"
			"    JAL      r30.w0, BITS_LOOP                                      \n"
			"    JAL      r30.w0, TRANSMIT_PRESCALED_DATA                        \n"

			// Update number of bits written
			"    ADD      R7.w2, R7.w2, 16                                       \n"
			// Write To RAM number of Bits Repeated
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"
			"    JMP      TxInterruptServiceRequestHndlr                         \n"

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

			"LOAD_TX_COMMON_INFO:                                                \n"
			// Load the TX Format Address for the specific channel
			"    QBEQ     LOAD_TX_FORMAT_ADDRESS_DONE, R3.b1, 0x1                \n"
			"    JAL      r30.w0, LOAD_TX_FORMAT_ADDRESS                         \n"

			"LOAD_TX_FORMAT_ADDRESS_DONE:                                        \n"
			//  Load the mapped SR and XBUF address mapped to channel
			"    JMP      LOCATE_SR_XBUF_SRCTL                                   \n"

			"LOCATE_SR_XBUF_SRCTL_DONE:                                          \n"
			// Format the data if required
			"    JMP      CHK_TX_DATA_FORMAT                                     \n"

			"CHK_TX_DATA_FORMAT_DONE:                                            \n"
			"    JMP      ENABLE_TX_SERIALIZER                                   \n"

//****************************************** LOAD_TX_COMMON_INFO: ENDS ***************************************

//======================================================================================================================================

//****************************************** TX LOCATE_SR_XBUF_SRCTL : Starts ********************************

//======================================================================================================================================
// 		Subroutine to find channel specific serializer, xbuf, srctl register mapped active channel
//======================================================================================================================================

			"LOCATE_SR_XBUF_SRCTL:                                               \n"
			// Calculating Serializer Mapped to Channel
			"    AND      r22, R4.b1, 0x0F                                       \n"
			"    LSL      r22, r22, 2                                            \n"

			// copy the tx format address to temp regsiter
			"    MOV      r24, R13.w0                                            \n"
			"    LDI      r25, 0x20                                              \n"

			// Calculating the specific SRCTL register offset
			"    LDI      R11.w0, (0x01D00180) & 0xFFFF                          \n"
			"    LDI      R11.w2, (0x01D00180) >> 16                             \n"
			"    ADD      R11, R11, r22                                          \n"

			"    ADD      r23, r25, 0                                            \n"
			"    SBBO     &R11, r24, r23,  4                                     \n"

			// Calculating the specific xbuf register offset
			"    LDI      R12.w0, (0x01D00200) & 0xFFFF                          \n"
			"    LDI      R12.w2, (0x01D00200) >> 16                             \n"
			"    ADD      R12, R12, r22                                          \n"

			"    ADD      r23, r25, 4                                            \n"
			"    SBBO     &R12, r24, r23,  4                                     \n"

			// Store the data length
			"    MOV      R13.b2, R5.b1                                          \n"

			"    ADD      r23, r25, 10                                           \n"
			"    SBBO     &R13.b2, r24, r23,  1                                  \n"

			//Store the data Tx FMT Context address
			"    ADD      r23, r25, 8                                            \n"
			"    SBBO     &R13.w0, r24, r23,  2                                  \n"

			"    LDI      R13.b3, 0x00                                           \n"
			"    ADD      r23, r25, 11                                           \n"
			"    SBBO     &R13.b3, r24, r23,  1                                  \n"

			"    JMP      LOCATE_SR_XBUF_SRCTL_DONE                              \n"

//********************************************** TX LOCATE_SR_XBUF_SRCTL: ENDS **************************************

//======================================================================================================================================

//********************************************** TX CHK_TX_DATA_FORMAT : Starts***************************************

//======================================================================================================================================
// 	Check For Data Formating, formats the data only if Chn_TxRxRepeatDoneCtr,
//	Chn_TxRxBitsDoneCtr, Chn_TxRxBytesDoneCtr all are zero,
//	If All the conditions is satisfied, it jumps to TX_DATA_FORMAT Subroutine  and
// 	formats data the TX Data obtained from ARM/DSP by adding start and stop bit.
//======================================================================================================================================

			"CHK_TX_DATA_FORMAT:                                                 \n"
			"    QBEQ     CHK_TX_DATA_FORMAT_BITS, R7.w2, 0                      \n"
			"    JMP      CHK_TX_DATA_FORMAT_DONE                                \n"

			"CHK_TX_DATA_FORMAT_BITS:                                            \n"
			"    QBEQ     CHK_TX_DATA_FORMAT_BYTE, R7.b1, 0                      \n"
			"    JMP      CHK_TX_DATA_FORMAT_DONE                                \n"

			"CHK_TX_DATA_FORMAT_BYTE:                                            \n"
			"    QBEQ     TX_DATA_FORMAT, R7.b0, 0                               \n"
			"    JMP      CHK_TX_DATA_FORMAT_DONE                                \n"

			"TX_DATA_FORMAT:                                                     \n"
			// Load the TX Format Address for the specific channel
			"    XOR      r22, r22, r22                                          \n"
			"    NOT      r22, r22                                               \n"

			"    SUB      r23, R5.b0, 1                                          \n"

			"    LSL      r22, r22, r23                                          \n"
			// offset from base addr
			"    XOR      r23, r23, r23                                          \n"

			// to store formated data into DATA RAM
			"    MOV      r24, R13.w0                                            \n"

			// Number of Bits Per Character
			"    AND      r21.b0, R5.b0, 0xF                                     \n"
			"    SUB      r21.b0, r21.b0, 2                                      \n"

			"TX_DATA_FORMAT_LOOP:                                                \n"
			// Load the data from the data pointer
			"    LBBO     &r28, R6, 0x00, 2                                      \n"
			"    LSL      r28, r28, 1                                            \n"
			"    OR       r28, r28, r22                                          \n"

			// store formated data into DATA RAM
			"    SBBO     &r28.w0, r24, r23, 2                                   \n"

			// Increment the formatted buffer address offset
			"    ADD      r23, r23, 2                                            \n"

			"    QBGE     INC_ADDR_BY_ONE, r21.b0, 0x8                           \n"
			// Next data buffer pointer
			"    ADD      R6, R6, 1                                              \n"

			// Increamnet the tx buffer data pointer by ONE, if bit per character is less or equal to 8 including start and stop bit
			"INC_ADDR_BY_ONE:                                                    \n"
			// Next data buffer pointer
			"    ADD      R6, R6, 1                                              \n"

			"    QBEQ     CHK_TX_DATA_FORMAT_DONE, R13.b2, 0                     \n"

			//Decrement the data length .i.e no of bytes to send
			"    SUB      R13.b2, R13.b2, 1                                      \n"

			"    JMP      TX_DATA_FORMAT_LOOP                                    \n"

//******************************************** TX CHK_TX_DATA_FORMAT: ENDS************************************

//======================================================================================================================================

//******************************************** TX READ_TX_DATA: Starts****************************************

//======================================================================================================================================
// 	Reads the 16 bit formatted character to be transmitted from formatted data area corresponding to TX channel
//======================================================================================================================================

			"READ_TX_DATA:                                                       \n"
			// Copy the base address of formated data
			"    MOV      r24, R13.w0                                            \n"

			// Calculate the offset of formated data
			"    LSL      r25, R7.b0, 1                                          \n"

			// LOAD formated data from DATA RAM
			"    LBBO     &r24, r24, r25, 2                                      \n"

			"    jmp      r30.w0                                                 \n"

//********************************************** TX READ_TX_DATA: ENDS ***************************************

//======================================================================================================================================

//********************************************** TX LOAD_TX_FORMAT_ADDRESS : Starts **************************

//======================================================================================================================================
//	Initializes the TX formatted data buffer address which stores the formated data with stop and start bit
//======================================================================================================================================

			"LOAD_TX_FORMAT_ADDRESS:                                             \n"
			"    QBEQ     TX_CH0_FMT_ADDR, R10.b2, 0                             \n"
			"    QBEQ     TX_CH2_FMT_ADDR, R10.b2, 2                             \n"
			"    QBEQ     TX_CH4_FMT_ADDR, R10.b2, 4                             \n"
			"    QBEQ     TX_CH6_FMT_ADDR, R10.b2, 6                             \n"
			"    jmp      r30.w0                                                 \n"

			"TX_CH0_FMT_ADDR:                                                    \n"
			"    LDI      R13.w0, 0x090                                          \n"
			"    jmp      r30.w0                                                 \n"

			"TX_CH2_FMT_ADDR:                                                    \n"
			"    LDI      R13.w0, 0x0E0                                          \n"
			"    jmp      r30.w0                                                 \n"

			"TX_CH4_FMT_ADDR:                                                    \n"
			"    LDI      R13.w0, 0x130                                          \n"
			"    jmp      r30.w0                                                 \n"

			"TX_CH6_FMT_ADDR:                                                    \n"
			"    LDI      R13.w0, 0x180                                          \n"
			"    jmp      r30.w0                                                 \n"

//******************************************** TX LOAD_TX_FORMAT_ADDRESS Routine: ENDS ***********************

//======================================================================================================================================

//******************************************** PRESACLE_TX_DATA : Starts *************************************

//======================================================================================================================================
// This routine Prescales data bit to be transmitted into the TX_DATA_reg.w0 register
//======================================================================================================================================

			"PRESACLE_TX_DATA:                                                   \n"
			// to Left shift each copied data bit
			"    XOR      r21.b1, r21.b1, r21.b1                                 \n"
			// Xbuf Size: Keep track of byte loop
			"    LDI      r23, 16                                                \n"
			// Clear TX_DATA_reg.w0
			"    XOR      r28.w0, r28.w0, r28.w0                                 \n"

			"BYTE_LOOP:                                                          \n"
			"    LDI      r25, 0x03FF                                            \n"
			"    AND      r22, r25, R4.w2                                        \n"

			"BITS_LOOP:                                                          \n"
			// CLR scratch_reg4
			"    XOR      r25, r25, r25                                          \n"
			// COPY 1 bit to scratch_reg4 from scratch_reg3
			"    AND      r25, r24, 0x1                                          \n"
			"    LSL      r25, r25, r21.b1                                       \n"
			"    OR       r28.w0, r28.w0, r25                                    \n"
			"    ADD      r21.b1, r21.b1, 1                                      \n"
			// INC Bytes loop counter
			"    SUB      r23, r23, 1                                            \n"
			// INC Bits loop counter
			"    SUB      r22, r22, 1                                            \n"
			"    QBLE     BITS_LOOP, r22, 1                                      \n"
			"    LSR      r24, r24, 1                                            \n"
			"    QBLE     BYTE_LOOP, r23, 1                                      \n"
			// Return to PRESACLE_TX_DATA
			"    jmp      r30.w0                                                 \n"

//********************************************  PRESACLE_TX_DATA : ENDs **************************************

//======================================================================================================================================

//******************************************** TRANSMIT_PRESCALED_DATA : Starts ******************************

//======================================================================================================================================
// This routine Transmits the prescaled data by writing to mcasp x_buf register mapped to this serializer
//======================================================================================================================================

			"TRANSMIT_PRESCALED_DATA:                                            \n"
			// Clear the under run error
			"    LBCO     &r22, C25, 0xc0, 4                                     \n"
			"    QBBC     WRITE_TO_XBUF, r22, 0x8                                \n"

			"    LDI      r22, 0xFFFF                                            \n"
			"    SBCO     &r22, C25, 0xc0, 4                                     \n"

			"WRITE_TO_XBUF:                                                      \n"
			// Write Byte to X_BUF
			"    SBBO     &r28.w0, R12, 00, 4                                    \n"
			// return from Transmit Prescaled Data
			"    jmp      r30.w0                                                 \n"

//******************************************** TRANSMIT_PRESCALED_DATA : ENDs ********************************

//======================================================================================================================================

//******************************************** TX_DONE : Starts **********************************************

//======================================================================================================================================
// This routine the cleanup after one character has been transmitted successfully
//======================================================================================================================================

			"TX_DONE:                                                            \n"
			"    XOR      R7.b1, R7.b1, R7.b1                                    \n"
			// Write To RAM number of Bits Transmitted
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			"    ADD      R7.b0, R7.b0, 1                                        \n"
			"    SBBO     &R7.b0, R8, 12,  1                                     \n"

			"    JMP      TxInterruptServiceRequestHndlr                         \n"

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

			"TxInterruptServiceRequestHndlr:                                     \n"
			// Retrieve the channel number and load the context base
			"    LDI      R8, 0x0000                                             \n"
			"    LDI      R10.w0, 0x0000                                         \n"
			"    LDI      R10.b2, 0x00                                           \n"
			"    LDI      R13.w0, 0x90                                           \n"

			"SERCH_MAPPED_TX_CHN:                                                \n"
			"    ADD      r22, R10.w0, 7                                         \n"
			// Load the Channel Cntrl info from Memory to Register
			"    LBBO     &R5.b3, R8, r22,  1                                    \n"
			"    QBBC     NEXT_TX_CHN, R5.b3, 7                                  \n"

			"    ADD      r22, R13.w0, 0x20                                      \n"
			"    LBBO     &r23, r22, 0, 4                                        \n"
			"    LBBO     &r22, r23, 0, 4                                        \n"
			"    QBBS     MAPPED_TX_CHN_FOUND, r22, 4                            \n"

			"NEXT_TX_CHN:                                                        \n"
			"    QBEQ     PRU_TX_ONLY_MODE, R3.b1, 0x1                           \n"

			// TX & RX together. So channel nunbers are 0, 2, 4, 6
			"    ADD      R10.w0, R10.w0, 0x20                                   \n"
			"    ADD      R10.b2, R10.b2, 0x02                                   \n"
			"    ADD      R13.w0, R13.w0, 0x50                                   \n"
			"    QBGE     SERCH_MAPPED_TX_CHN, R10.b2, (8 - 1)                   \n"
			"    JMP      CORE_LOOP                                              \n"

			"PRU_TX_ONLY_MODE:                                                   \n"
			// TX Only ...So channel numbers are contiguous
			"    ADD      R10.w0, R10.w0, 0x10                                   \n"
			"    ADD      R10.b2, R10.b2, 0x01                                   \n"
			"    ADD      R13.w0, R13.w0, 0x2C                                   \n"
			"    QBGE     SERCH_MAPPED_TX_CHN, R10.b2, (8 - 1)                   \n"
			"    JMP      CORE_LOOP                                              \n"

			"MAPPED_TX_CHN_FOUND:                                                \n"
			"    LBBO     &R4.w0, R8, R10.w0,  16                                \n"
			"    ADD      R8, R8, R10.w0                                         \n"

			"    QBEQ     PRUx_MODE_TX_ONLY, R3.b1, 0x1                          \n"

			"    QBEQ     CORE_LOOP, R3.b1, 0x00                                 \n"

			"PRUx_MODE_TX_ONLY:                                                  \n"
			"    ADD      r22, R13.w0, 0x20                                      \n"
			"    LBBO     &R11, r22, 0, 12                                       \n"

			// JMP TO TxServiceReqHndlLoop Chn_TxRxBytesDoneCtr is less than Data length
			"    LSR      r22, R5.w0, 0x8                                        \n"
			"    AND      r22, r22, 0x0F                                         \n"
			"    ADD      r22, r22, 0x01                                         \n"
			"    QBLT     TxServiceReqHndlLoop, r22, R7.b0                       \n"

			"    QBBS     DECLARE_COMPLETE, R5.b2, 0                             \n"
			"    NOT      r22, r20                                               \n"
			"    SBBO     &r22, R12, 00, 4                                       \n"

			"    JMP      CORE_LOOP                                              \n"

			"DECLARE_COMPLETE:                                                   \n"
			// Set the status in the context area
			"    SET      R5.b2, R5.b2, 1                                        \n"
			"    CLR      R5.b2, R5.b2, 0                                        \n"
			"    SBBO     &R5.b2, R8, 6,  1                                      \n"

			// Generate the interrupt to the ARM/DSP about the completion
			"    LBBO     &R2.w0, r20, 0x080, 2                                  \n"
			"    QBBC     CORE_LOOP, R2.w0, R10.b2                               \n"

			"    LBBO     &R2.w2, r20, 0x082, 2                                  \n"
			"    SET      R2.w2, R2.w2, R10.b2                                   \n"
			"    SBBO     &R2.w2, r20, 0x082, 2                                  \n"
			"    JMP      PRU_TO_HOST_INTERRUPT                                  \n"

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

			"CORE_LOOP:                                                          \n"
			"    QBEQ     CORE_LOOP, R3.b1, 0x00                                 \n"

			"ARM_DSP_EVENT:                                                      \n"
			"    QBEQ     CORE_LOOP_PRU1, R3.b0, 1                               \n"

			"CORE_LOOP_PRU0:                                                     \n"
			// wait for the hostEventStatus to get set. Loop till then
			"    WBS      r31, 30                                                \n"

			// Read the PRUINTC register to know if the event is from ARM/DSP. If yes, then branch
			"    LDI      r23.w0, 0x204 & 0xFFFF                                 \n"
			"    LDI      r23.w2, 0x204 >> 16                                    \n"
			"    LBCO     &r22, C0, r23, 4                                       \n"
			"    QBBS     CHN_SEARCH, r22, 0                                     \n"

			// Else it is McASP Event. So before proceeding, clear it
			"    LDI      r22.w0, 31 & 0xFFFF                                    \n"
			"    LDI      r22.w2, 31 >> 16                                       \n"
			"    SBCO     &r22, C0, 0x24, 4                                      \n"

			"    JMP      MCASP_EVENT                                            \n"

			"CORE_LOOP_PRU1:                                                     \n"
			// wait for the hostEventStatus to get set. Loop till then
			"    WBS      r31, 31                                                \n"

			"    LBCO     &r22, C25, 0xc0, 4                                     \n"
			"    QBBC     CHN_SEARCH, r22, 5                                     \n"

			// Clear the event here and go to Transmit processing
			"    LDI      r22.w0, 50 & 0xFFFF                                    \n"
			"    LDI      r22.w2, 50 >> 16                                       \n"
			"    SBCO     &r22, C0, 0x24, 4                                      \n"
			"    JMP      TxInterruptServiceRequestHndlr                         \n"

			"MCASP_EVENT:                                                        \n"
			// Check for RX interrrupt first
			// If TX only PRU Skip RSTAT Check
			"    QBEQ     MCASP_TX_EVNT, R3.b1, 0x1                              \n"

			// if the PRU is RX only mode, then check if the XSTAT is set. If so, raise event to PRU1 and proceed
			"    QBNE     RX_TX_PROCESS, R3.b1, 0x2                              \n"
			"    LBCO     &r22, C25, 0xc0, 4                                     \n"
			"    QBBC     RX_TX_PROCESS, r22, 5                                  \n"
			"    LDI      r22.w0, 50 & 0xFFFF                                    \n"
			"    LDI      r22.w2, 50 >> 16                                       \n"
			"    SBCO     &r22, C0, 0x20, 4                                      \n"

			"RX_TX_PROCESS:                                                      \n"
			"    LBCO     &r22, C25, 0x80, 4                                     \n"
			"    QBBS     RxInterruptServiceRequestHndlr, r22, 5                 \n"
			// Skip the check for XSTAT if we are not Rx/Tx PRU.
			// We don't want the PRU to spin in a tight loop around the McASP register to introduce a delay
			"    QBNE     CORE_LOOP, R3.b1, 0x3                                  \n"

			"MCASP_TX_EVNT:                                                      \n"
			"    LBCO     &r22, C25, 0xc0, 4                                     \n"
			"    QBBS     TxInterruptServiceRequestHndlr, r22, 5                 \n"
			// If PRU is both TX/RX, then go back to Core-loop. Else delay to avoid McASP Spins
			"    QBEQ     CORE_LOOP, R3.b1, 0x3                                  \n"

//******************************************** CORE LOOP: Ends ***********************************************

//========================================================================================================================================

//******************************************** CHN_SEARCH: Starts ********************************************

//========================================================================================================================================
//	Retrieve the active channel number that has requested for serviced and
//	load the context base info for that channel
//========================================================================================================================================

			"CHN_SEARCH:                                                         \n"
			"    LDI      r24.w0, 0x00000001 & 0xFFFF                            \n"
			"    LDI      r24.w2, 0x00000001 >> 16                               \n"
			"    LSL      r24, r24, R3.b0                                        \n"
			"    LDI      r22.w0, 0x284 & 0xFFFF                                 \n"
			"    LDI      r22.w2, 0x284 >> 16                                    \n"
			"    SBCO     &r24, C0, r22, 4                                       \n"

			// Read Global control register
			"    LBBO     &R2.w0, r20, 0x080,  8                                 \n"

			// Retrieve the channel number and load the context base
			"    LDI      R8, 0x0000                                             \n"
			"    LDI      R10.w0, 0x0000                                         \n"
			"    LDI      R10.b2, 0x00                                           \n"
			"    XOR      R13.w0, R13.w0, R13.w0                                 \n"
			"    XOR      R9, R9, R9                                             \n"

			"    LDI      R13.w0, 0x90                                           \n"
			"    LDI      R9, 0x90                                               \n"

			"CHN_ACTIVE:                                                         \n"
			"    LBBO     &R4.w0, R8, R10.w0,  2                                 \n"
			"    QBBS     CHN_SERACH_RTN, R4.w0, 0x2                             \n"
			"    ADD      R10.w0, R10.w0, 0x10                                   \n"
			"    ADD      R10.b2, R10.b2, 0x01                                   \n"
			"    ADD      R13.w0, R13.w0, 0x2C                                   \n"
			"    ADD      R9, R9, 0x20                                           \n"

			// None of the channel has service request, go back to MainLoop
			// check to be verified to boundary condition
			"    QBLT     MCASP_EVENT, R10.b2, (8 - 1)                           \n"
			"    JMP      CHN_ACTIVE                                             \n"

			"CHN_SERACH_RTN:                                                     \n"
			"    LBBO     &R4.w0, R8, R10.w0,  16                                \n"
			"    ADD      R8, R8, R10.w0                                         \n"

			"    AND      r22.w0, R4.w0, 0x3                                     \n"
			"    QBEQ     TxServiceRequestHndlr, r22.w0, 0x1                     \n"
			"    QBEQ     RxServiceRequestHndlr, r22.w0, 0x2                     \n"
			"    JMP      CORE_LOOP                                              \n"

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

			"PRU_TO_HOST_INTERRUPT:                                              \n"
			"    LDI      r22, 0                                                 \n"

			"    QBEQ     EVTOUT_PRU0_EVENTS, R3.b0, 0                           \n"
			"    QBEQ     EVTOUT_PRU1_EVENTS, R3.b0, 1                           \n"

			"EVTOUT_PRU0_EVENTS:                                                 \n"
			//storing the counter value
			"    ADD      r22, r22, 34                                           \n"
			"    JMP      EVTOUT_SYSEVT_INIT                                     \n"

			"EVTOUT_PRU1_EVENTS:                                                 \n"
			"    ADD      r22, r22, 42                                           \n"

			"EVTOUT_SYSEVT_INIT:                                                 \n"
			"    ADD      r22, r22, R10.b2                                       \n"

			"EVTOUT_GEN:                                                         \n"
			// Clear SYS_EVTn
			"    SBCO     &r22, C0, 0x24, 4                                      \n"

			// Enable SYS_EVTn system interrupt
			"    SBCO     &r22, C0, 0x28, 4                                      \n"

			// Generate SYS_EVTn by event out mapping
			"    MOV      r31.w0, r22.w0                                         \n"

			"    JMP      MCASP_EVENT                                            \n"

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

			"RxServiceRequestHndlr:                                              \n"
			// load the max RX TRIES before time out
			"    LBBO     &r1.w0, r20, 0x088, 2                                  \n"

			// read interrupt status regsiter
			"    LBBO     &R2.w2, r20, 0x082, 2                                  \n"

			"    CLR      R2.w2, R2.w2, R10.b2                                   \n"

			// write interrupt status regsiter
			"    SBBO     &R2.w2, r20, 0x082, 2                                  \n"

			//Clear Service Request
			"    CLR      R4.w0, R4.w0, 0x2                                      \n"
			"    SBBO     &R4.w0, R8, 0,  2                                      \n"

			// clear timeout flag
			"    CLR      R5.b2, R5.b2, 6                                        \n"

			// Set the TXRX_READY_BIT
			"    SET      R5.b2, R5.b2, 0                                        \n"

			// update the RX Status Register
			"    SBBO     &R5.b2, R8, 6,  1                                      \n"

			// Set the SUART_CH_TXRXCHNSTATUS_BIT to indicate the channel being active
			"    SET      R5.b3, R5.b3, 7                                        \n"
			"    SBBO     &R5.b3, R8, 7,  1                                      \n"

			"RX_CONTEXT_INIT:                                                    \n"
			"    QBEQ     PRUxxx_MODE_RX_ONLY, R3.b1, 0x2                        \n"

			// Load RX Context Base Address corresponding to Active RX Channel
			"    JAL      r30.w0, LOAD_RX_CONTEXT_ADDRESS                        \n"

			"PRUxxx_MODE_RX_ONLY:                                                \n"
			// Calculating the specific SRCTL and R_BUF register offset.
			"    AND      r22, R4.b1, 0x0F                                       \n"
			"    LSL      r22, r22, 2                                            \n"
			// Storing SRCTL register address in RX Context Area Region
			"    LDI      R12.w0, (0x01D00180) & 0xFFFF                          \n"
			"    LDI      R12.w2, (0x01D00180) >> 16                             \n"
			"    ADD      R12, R12, r22                                          \n"

			//storing asp_rsrctl_reg in RX Context Address Region
			"    SBBO     &R12, R9, 4,  4                                        \n"

			// Store RBuf Address in RX Context Region
			"    LDI      R11.w0, (0x01D00280) & 0xFFFF                          \n"
			"    LDI      R11.w2, (0x01D00280) >> 16                             \n"
			"    ADD      R11, R11, r22                                          \n"

			// storing asp_rbuf_reg in RX context  adress region
			"    SBBO     &R11, R9, 0,  4                                        \n"

			// Load the Context info specific to Current RX channel from memory to registers
			//	LBBO   	rx_context,  suart_ch_info.rx_context_addr, #00, SIZE (rx_context)

			// Clear the RX timeout counter
			"    XOR      R16.w0, R16.w0, R16.w0                                 \n"
			"    SBBO     &R16.w0, R9, 20,  2                                    \n"

			// Activate RX serializer
			"    LBBO     &r23, R12, 0, 4                                        \n"
			"    AND      r23, r23, 0x3                                          \n"
			// Check if Serializer is Already Active as Rx if ,yes skip activation
			"    QBEQ     CLR_RSTAT, r23, 0x2                                    \n"
			//  Activate serializer as Receiver
			"    LDI      r23.w0, 0x000E & 0xFFFF                                \n"
			"    LDI      r23.w2, 0x000E >> 16                                   \n"
			"    SBBO     &r23, R12, 0, 4                                        \n"

			"CLR_RSTAT:                                                          \n"
			// Clear the RSTAT  (Overrun, etc)
			"    LDI      r22.w0, 0xFFFF & 0xFFFF                                \n"
			"    LDI      r22.w2, 0xFFFF >> 16                                   \n"
			"    SBCO     &r22, C25, 0x80, 4                                     \n"

			"    JMP      MCASP_EVENT                                            \n"

//******************************************** RxServiceRequestHndlr: ENDS ***********************************

//========================================================================================================================================

//******************************************** RxInterruptServiceRequestHndlr: Starts ************************

//========================================================================================================================================
//	RxInterruptServiceRequestHndlr is called when there is MCASP RX event, scans the active RX serializer,
//	once the active serializer is found, scans for corresponding RX channel, if it also is found, it loads
//	the context info for RX channel and proceeds for reading the frame tramsmitted by sender.
//========================================================================================================================================

			"RxInterruptServiceRequestHndlr:                                     \n"
			// Retrieve the channel number and load the RX context base info corressponding to serializer address in scratch_reg1 and serializer number  in scratch_reg4
			// Load the SUART CHANNEL BASE ADDRESS
			"    LDI      R8, 0x0000                                             \n"

			"    QBEQ     PRUx_MODE_RX_ONLY, R3.b1, 0x2                          \n"

			// Since the Rx Channel are 1,3,5,7 Load the suart_ch_regs for Rx channel 1 as it is first channel
			"    ADD      R10.w0, r20.w0, 0x10                                   \n"

			"    LDI      R10.b2, 0x01                                           \n"

			// Load the RX channel 1 context address to Ch_info register's rx_context_addr field
			"    LDI      R9, 0x0C0                                              \n"
			"    JMP      SERCH_ACTIVE_RX_CHN_RX                                 \n"

			"PRUx_MODE_RX_ONLY:                                                  \n"
			// Since the Rx Channel are 1,3,5,7 Load the suart_ch_regs for Rx channel 1 as it is first channel
			"    LDI      R10.w0, 0x00                                           \n"

			"    LDI      R10.b2, 0x00                                           \n"

			// Load the RX channel 1 context address to Ch_info register's rx_context_addr field
			"    LDI      R9, 0x90                                               \n"

			"SERCH_ACTIVE_RX_CHN_RX:                                             \n"
			"    ADD      r22, R10.w0, 7                                         \n"
			// Load the Channel Cntrl info from Memory to Register
			"    LBBO     &R5.b3, R8, r22,  1                                    \n"
			"    QBBC     NEXT_RX_CHN, R5.b3, 7                                  \n"

			"    LBBO     &r22, R9, 4, 4                                         \n"
			"    LBBO     &r23, r22, 0, 4                                        \n"
			"    QBBS     ACTIVE_RX_CHN_FOUND, r23, 5                            \n"

			"NEXT_RX_CHN:                                                        \n"
			"    QBEQ     PRUxx_MODE_RX_ONLY, R3.b1, 0x2                         \n"

			// offset of RX suart_ch_regs
			"    ADD      R10.w0, R10.w0, 0x20                                   \n"
			// Increament to Next Rx Channel number
			"    ADD      R10.b2, R10.b2, 0x2                                    \n"

			// Increament rx_context_addr by RX_CONTEXT_OFFSET i.e. to next RX channel context address
			"    ADD      R9, R9, 0x50                                           \n"
			"    QBGE     SERCH_ACTIVE_RX_CHN_RX, R10.b2, (8 - 1)                \n"
			"    JMP      CORE_LOOP                                              \n"

			"PRUxx_MODE_RX_ONLY:                                                 \n"
			// offset of RX suart_ch_regs
			"    ADD      R10.w0, R10.w0, 0x10                                   \n"
			// Increamnet to Next Rx ChanneL number
			"    ADD      R10.b2, R10.b2, 0x1                                    \n"
			// Increamnet rx_context_addr by RX_CONTEXT_OFFSET i.e. to next RX channel context address
			"    ADD      R9, R9, 0x20                                           \n"
			"    QBGE     SERCH_ACTIVE_RX_CHN_RX, R10.b2, (8 - 1)                \n"
			"    JMP      CORE_LOOP                                              \n"

			"ACTIVE_RX_CHN_FOUND:                                                \n"
			// Load the suart_ch_regs from Memory to Register
			"    LBBO     &R4.w0, R8, R10.w0,  16                                \n"

			// Load the Context info specific to current RX Channel from memory to registers
			"    LBBO     &R11, R9, 0, 24                                        \n"

			"    ADD      R8, R8, R10.w0                                         \n"

			// Clear the RSTAT  (Overrun, etc) for Errors
			"    LBCO     &r22, C25, 0x80, 4                                     \n"
			"    QBBC     RX_PROCESSING_INIT, r22, 8                             \n"

			"    LDI      r22.w0, 0xFFFF & 0xFFFF                                \n"
			"    LDI      r22.w2, 0xFFFF >> 16                                   \n"
			"    SBCO     &r22, C25, 0x80, 4                                     \n"

			//  Start receving DATA from MAC_ASP's R-Buf corresponding to channel
			"RX_PROCESSING_INIT:                                                 \n"
			"    XOR      r19, r19, r19                                          \n"
			// Read the content of RBUF
			"    LBBO     &r22, R11, 0, 4                                        \n"
			"    OR       r19, r19, r22                                          \n"

			// If start condition is already received then go to reading next bit  otherwise look for start condition
			"    QBLT     READ_CURRENT, R7.b1, 0                                 \n"

			// check Chn_TxRxRepeatDoneCtr, if it is not zero, jump to READ_CURRENT to prescale the start condition
			"    QBLT     READ_CURRENT, R7.w2, 0                                 \n"

			// If sampling point i.e. sampling_bit_pos is equal to greater than 16 (INVALID_SAMPLING_POINT),
			// start bit transition edge is being detected, fall through to calculate sampling point,
			// otherwise, sampling point is already calculated JUMP to READ_CURRENT
			"    QBGE     READ_CURRENT, R16.b2, 16                               \n"

			// Extract timing information by detecting start transition (first left most zero)
			"    LMBD     r25, r22, 0                                            \n"
			// branch if zero: start bit transition detected
			"    QBGT     START_BIT_TRANSITION, r25, 32                          \n"
			"    LDI      R16.b2, 0xff                                           \n"
			"    SBBO     &R16.b2, R9, 22,  1                                    \n"

			// RX time out logic
			"    QBBC     RxInterruptServiceRequestHndlr, R4.w2, 14              \n"
			"    QBBC     RxInterruptServiceRequestHndlr, R5.b2, 0               \n"
			"    QBEQ     RxInterruptServiceRequestHndlr, R7.b0, 0               \n"

			// Read the request count to be received
			"    LSR      r22, R5.w0, 0x8                                        \n"
			"    AND      r22, r22, 0x0F                                         \n"
			// Since fifo size is 16
			"    ADD      r22, r22, 0x01                                         \n"
			"    QBEQ     RxInterruptServiceRequestHndlr, R7.b0, r22             \n"

			// check if time-out is enabled, if yes increament the timeout counter and check if count is equal to MAX_RX_TIMEOUT_TRIES
			// if yes raise the interrupt for time out.
			"    ADD      R16.w0, R16.w0, 1                                      \n"
			"    SBBO     &R16.w0, R9, 20,  2                                    \n"
			"    QBGE     RxInterruptServiceRequestHndlr, R16.w0, r1.w0          \n"
			"    SET      R5.b2, R5.b2, 6                                        \n"
			"    CLR      R4.w2, R4.w2, 14                                       \n"
			"    SBBO     &R4.w2, R8, 2,  2                                      \n"

			// Clear the RX timeout counter
			"    XOR      R16.w0, R16.w0, R16.w0                                 \n"
			"    SBBO     &R16.w0, R9, 20,  2                                    \n"
			"    JMP      RX_CHN_INTR                                            \n"

			// Calculate the sampling bit position based on the start bit position
			// center = oversampling / 2
			// sampling bit position = start bit possition - center

			"START_BIT_TRANSITION:                                               \n"
    			// clear the rx time out counter
			"    XOR      R16.w0, R16.w0, R16.w0                                 \n"
			"    SBBO     &R16.w0, R9, 20,  2                                    \n"

			// determine the over-sampling rate
			"    LSR      r23, R4.w2, 10                                         \n"
			"    AND      r23, r23, 0x3                                          \n"

			// OVER_SAMPLE
			"    QBEQ     OVER_SAMPLE_SIZE8BIT, r23, 0x0                         \n"
			"    QBEQ     OVER_SAMPLE_SIZE8BIT, r23, 0x1                         \n"
			"    QBEQ     OVER_SAMPLE_SIZE16BIT, r23, 0x2                        \n"

			// Calaulate sampling bit position for 8 bit over sampling
			"OVER_SAMPLE_SIZE8BIT:                                               \n"
			// start bit possition - center
			"    SUB      R16.b2, r25, 0x4                                       \n"
			// sampling point
			"    AND      R16.b2, R16.b2, 7                                      \n"
			"    SBBO     &R16.b2, R9, 22,  1                                    \n"
			// if Start bit position is eqaul to/greater than centre, sample the start bit in current read, otherwise in next read
			"    QBLE     READ_CURRENT, r25, 0x4                                 \n"
			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			// Calaulate sampling bit position for 16 bit over sampling
			"OVER_SAMPLE_SIZE16BIT:                                              \n"
			// start bit possition - center
			"    SUB      R16.b2, r25, 0x8                                       \n"
			// samplimg point
			"    AND      R16.b2, R16.b2, 15                                     \n"
			"    SBBO     &R16.b2, R9, 22,  1                                    \n"
			// if Start bit position is eqaul to/greater than centre, sample the start bit in current read, otherwise in next read
			"    QBLE     READ_CURRENT, r25, 0x8                                 \n"
			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			"READ_CURRENT:                                                       \n"
			// scratch_8bit_reg2 holds the information if bit detected is zero if scratch_8bit_reg2= 0, or one if scratch_8bit_reg2 = 1
			"    XOR      r21.b1, r21.b1, r21.b1                                 \n"
			// if bit at sampling point is zero jump to READ_ZERO
			"    QBBC     READ_ZERO, r22, R16.b2                                 \n"
			// otherwise increament scratch_8bit_reg2 by one as bit detected is one
			"    ADD      r21.b1, r21.b1, 1                                      \n"

			"READ_ZERO:                                                          \n"
			// We have read the data bit here...
			// If start bit is being received already, then skip the start condition processing.
			"    QBLT     RX_BIT_RECVD, R7.b1, 0                                 \n"

			//(Chn_TxRxBitsDoneCtr == 0)            //No bit is being Recieved, check if it is start bit
			// if DataBit == 0, i.e. scratch_8bit_reg2 == 0, Jump to Start Condition, else error fall through
			"    QBEQ     START_CONDITION, r21.b1, 0                             \n"

			"    QBEQ     START_CONDITION, R7.w2, 0                              \n"

			// Broken start condition or false alarm, Reset repeat counter		//if DataBit == 1, instead of zero
			"    XOR      R7.w2, R7.w2, R7.w2                                    \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"
			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			// else part for NO_REPEAT_DONE  DataBit == 0
			"START_CONDITION:                                                    \n"
			// Increament Repeat Done Counter by One, write back to memory
			"    ADD      R7.w2, R7.w2, 1                                        \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			// Read Pre Scaler
			"    LDI      r23, 0x03FF                                            \n"
			"    AND      r23, r23, R4.w2                                        \n"

			// if Repeat Done count is greater than or equal to prescaler, start bit is received, jump to START_BIT_RECIVED,
			"    QBGE     START_BIT_RECIVED, r23, R7.w2                          \n"
			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			// Start bit is condition Detected properly
			"START_BIT_RECIVED:                                                  \n"
			// Increament Bit Count by One, and write it to memory
			"    ADD      R7.b1, R7.b1, 1                                        \n"
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// Reset Repeat Counter, and write it to memory
			"    XOR      R7.w2, R7.w2, R7.w2                                    \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"
			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			// Start Bit has been detected Already, Now the data bit is being received
			"RX_BIT_RECVD:                                                       \n"
			// Now scratch_reg1 holds the info whether the data bit in scratch_8bit_reg2, is zero or one
			"    XOR      r22, r22, r22                                          \n"
			// if scratch_8bit_reg2 = 0, i.e data bit is Zero Jump to RX_DATA_BIT_ZERO
			// else Data bit is one fall through, data bit is ONE
			"    QBEQ     RX_DATA_BIT_ZERO, r21.b1, 0                            \n"
			// bit received is one, scratch_reg1 = 1
			"    OR       r22, r22, 0x1                                          \n"

			"RX_DATA_BIT_ZERO:                                                   \n"
			// if (Chn_TxRxRepeatDoneCntr < 32), check if reapeat done counter is less than 32, if yes Jump to RX_REPEAT_DONE_CNTR_LT_32
			"    QBGE     RX_REPEAT_DONE_CNTR_LT_32, R7.w2, 0x20                 \n"

			// repeat done counter is Greater than 32, Read Chn_RxDataBitsHoldRegHigh reg, Copy the Received bit to Chn_RxDataBitsHoldRegHigh register
			// else part : (Chn_TxRxRepeatDoneCntr is Greater than or equal to 32 )
			"RX_REPEAT_DONE_CNTR_GT_32:                                          \n"
			// Calculate the offset for bit in Chn_RxDataBitsHoldRegHigh regsiter
			"    RSB      r23, R7.w2, 0x20                                       \n"
			// Shift Received bit by above calculated of set i.e Chn_TxRxRepeatDoneCntr - 20
			"    LSL      r22, r22, r23                                          \n"
			"    LBBO     &R15, R9, 16,  4                                       \n"
			"    OR       R15, r22, R15                                          \n"
			"    SBBO     &R15, R9, 16,  4                                       \n"
			"    JMP      RX_REPEAT_COUNT_LESS_THAN_PRESCALR                     \n"

			// repeat done counter is less than OR equal to 32, Read the Chn_RxDataBitsHoldRegLow, Copy the Received bit to Chn_RxDataBitsHoldRegLow register
			// write it back to memory
			// if for (Chn_TxRxRepeatDoneCntr < 32)
			"RX_REPEAT_DONE_CNTR_LT_32:                                          \n"
			// Shift Received bit by Repeat Done Counter
			"    LSL      r22, r22, R7.w2                                        \n"
			"    LBBO     &R14, R9, 12,  4                                       \n"
			"    OR       R14, r22, R14                                          \n"
			"    SBBO     &R14, R9, 12,  4                                       \n"

			// Increament Chn_TxRxRepeatDoneCntr by one and Check if Repeat Done Counter is equal to Prescalar,
			// if yes jump to PROCESS_RX_DATA_BIT, otherewise again sample RBuf for same bit
			"RX_REPEAT_COUNT_LESS_THAN_PRESCALR:                                 \n"
			"    ADD      R7.w2, R7.w2, 1                                        \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			// Read Pre Scaler
			"    LDI      r23, 0x03FF                                            \n"
			"    AND      r23, r23, R4.w2                                        \n"

			// check if number of bits sampled (Chn_TxRxRepeatDoneCtr) is equal to prescaler (scratch_reg2), if yes jump to PROCESS_RX_DATA_BIT
			"    QBGE     PROCESS_RX_DATA_BIT, r23, R7.w2                        \n"
			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			// Scan Chn_RxDataBitsHoldRegLow, Chn_RxDataBitsHoldRegHigh, to check if BIT received is one or zero and write to Chn_RxDataHoldReg
			// (Chn_TxRxRepeatDoneCntr >= Chn_Config1.PreScaller) if part
			"PROCESS_RX_DATA_BIT:                                                \n"
			// Get the Presaclar
			"    LDI      r24, 0x03FF                                            \n"
			// scratch_reg3 hold prescalar
			"    AND      r24, r24, R4.w2                                        \n"

			// Initialize the register to zero required for copying data bit received in rxdata_buf
			// keep count of number of ONE scanned
			"    XOR      r21.b0, r21.b0, r21.b0                                 \n"
			// keep count of number of ZERO scanned
			"    XOR      r21.b1, r21.b1, r21.b1                                 \n"

			// used to store count of number of bits scanned in Chn_RxDataBitsHoldRegLow, & Chn_RxDataBitsHoldRegHigh
			"    XOR      r23, r23, r23                                          \n"

			// points to location taken as start point in Chn_RxDataBitsHoldRegLow for scannig bit received
			"    XOR      r22, r22, r22                                          \n"

			// scratch_reg4 holds the data from Chn_RxDataBitsHoldRegLow
			"    LBBO     &r25, R9, 12,  4                                       \n"
			// if Pre Scalar is less than or equal to 32, JMP to BIT_CHK_LOOP
			"    QBGE     BIT_CHK_LOOP, r24, 0x20                                \n"

			// pre scalar is greater 32, check if it is greater that 48 then set scratch_reg3 = 48, scan bit upto this count only.
			"PRE_SCALR_GT_32:                                                    \n"
			// start checking bit from bit position 0x10
			"    OR       r22, r22, 0x10                                         \n"
			// if Pre Scalar is less than 48
			"    QBGT     BIT_CHK_LOOP, r24, 0x30                                \n"

			// pre scalar is greater 48,  set scratch_reg3 = 48, scan bit upto this count only.
			"PRE_SCALR_GT_48:                                                    \n"
			"    LDI      r24, 0x30                                              \n"

			// Scan the Chn_RxDataBitsHoldRegLow, and Chn_RxDataBitsHoldRegHigh registers to know received bit is ZERO or ONE
			"BIT_CHK_LOOP:                                                       \n"
			// if bit is cleared, Jump to BIT_RECVD_ZERO
			"    QBBC     BIT_RECVD_ZERO, r25, r22                               \n"
			// else BIT prerscaled is one
			"    ADD      r21.b0, r21.b0, 1                                      \n"
			// Increament scratch_reg1 by one so that it points to next bit to scanned
			"    ADD      r22, r22, 1                                            \n"
			// Increament scratch_reg2 holding bits scanned count
			"    ADD      r23, r23, 1                                            \n"
			// if Prescaler is greater than 32, and scratch_reg2 is equal to 32, load Chn_RxDataBitsHoldRegHigh in scratch_reg4
			"    QBLT     LOAD_RXDATABITS_HOLDREGHIGH, r23, 0x20                 \n"
			// scan untill all the bits are scanned
			"    QBGT     BIT_CHK_LOOP, r23, r24                                 \n"
			"    JMP      COPY_BIT_RECVD                                         \n"

			// Load the Chn_RxDataBitsHoldRegHigh to scratch_reg4
			"LOAD_RXDATABITS_HOLDREGHIGH:                                        \n"
			"    LBBO     &r25, R9, 16,  4                                       \n"
			// Reset the scratch_reg1, so that starts from bit 0 for Chn_RxDataBitsHoldRegHigh
			"    XOR      r22, r22, r22                                          \n"
			// Reset the scratch_reg2, so that only jump to label LOAD_RXDATABITS_HOLDREGHIGH done one's only
			"    XOR      r23, r23, r23                                          \n"
			// Decreament Total loop count by 32, since it has been already checked in Chn_RxDataBitsHoldRegLow
			"    SUB      r24, r24, 0x20                                         \n"
			"    JMP      BIT_CHK_LOOP                                           \n"

			// Current sacnned Bit in Chn_RxDataBitsHoldRegHigh or Chn_RxDataBitsHoldRegLow is zero
			"BIT_RECVD_ZERO:                                                     \n"
			// for Zero
			"    ADD      r21.b1, r21.b1, 1                                      \n"
			"    ADD      r22, r22, 1                                            \n"
			"    ADD      r23, r23, 1                                            \n"
			"    QBGT     BIT_CHK_LOOP, r23, r24                                 \n"

			// Copy the Received bit to Chn_RxDataHoldReg, scratch_reg1, now store the info if bit received is zero or one
			"COPY_BIT_RECVD:                                                     \n"

			// scratch_8bit_reg1= Bit is ONE, scratch_8bit_reg2 = Bit is Zero, if scratch_8bit_reg2 > scratch_8bit_reg1,
			// jump to WRITE_RCVD_BIT_TO_RX_DATAHOLDREG as data bit is ZERO
			"    XOR      r22, r22, r22                                          \n"
			"    QBGE     WRITE_RCVD_BIT_TO_RX_DATAHOLDREG, r21.b0, r21.b1       \n"
			//Bit Received is One,  write to Chn_RxDataHoldReg
			"    OR       r22, r22, 0x1                                          \n"

			// Write the Received Data bit (in scratch_reg1) to Chn_RxDataHoldReg
			"WRITE_RCVD_BIT_TO_RX_DATAHOLDREG:                                   \n"
			// Shift the bit received by Chn_TxRxBitsDoneCtr
			"    LSL      r22, r22, R7.b1                                        \n"

			// Read the Chn_RxDataHoldReg from Memory
			"    LBBO     &R13.w2, R9, 10,  2                                    \n"

			// Write the bit received to Chn_RxDataHoldReg
			"    OR       R13.w2, R13.w2, r22                                    \n"

			// Write updated Chn_RxDataHoldReg to memory
			"    SBBO     &R13.w2, R9, 10,  2                                    \n"

			// Increment the Data bit Counter
			"    ADD      R7.b1, R7.b1, 1                                        \n"
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// Reset the Repeat Done Counter
			"    XOR      R7.w2, R7.w2, R7.w2                                    \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			// initialize Chn_RxDataBitsHoldRegLow
			"    XOR      R14, R14, R14                                          \n"
			"    SBBO     &R14, R9, 12,  4                                       \n"

			// initialize Chn_RxDataBitsHoldRegHigh
			"    XOR      R15, R15, R15                                          \n"
			"    SBBO     &R15, R9, 16,  4                                       \n"

			// Read Bit Per Charater
			"    AND      r23, R5.w0, 0xF                                        \n"

			// check is (N-1) bit is being received for current data frame, if yes jump to CHK_RECVD_DATA_FRAME
			// if all N bits has been Received Jump to RESET_BITS_CNTR, otherwise receive remaining bits.
			// (Chn_TxRxBitsDoneCntr >= Chn_Config2.BitsPerChar)
			"    QBGE     RESET_BITS_CNTR, r23, R7.b1                            \n"
			"    SUB      r23, r23, 1                                            \n"
			"    QBEQ     CHK_RECVD_DATA_FRAME, r23, R7.b1                       \n"
			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			// if all bits received, verify the Received data frame
			"CHK_RECVD_DATA_FRAME:                                               \n"
			// Zero the (16 - Chn_TxRxBitsDoneCntr) Most significant bits in the Chn_RxDataHoldReg.
			"    RSB      r23, r23, 16                                           \n"
			// load the count to for number of zero to inserted
			"    ADD      r23, r23, 0x10                                         \n"
			// Used to Insert  Zero in MSB
			"    NOT      r22, r20                                               \n"

			"REGHOLD_MSB_ZERO:                                                   \n"
			// Prepare the MASK with  ZERO's in bits that do not corresponds to data bits
			"    LSR      r22, r22, r23                                          \n"

			// Read the Data hold Reg
			"    LBBO     &R13.w2, R9, 10,  2                                    \n"
			// Insert the ZERO's in bits that do  not  corresponds to Data Bits
			"    AND      R13.w2, R13.w2, r22                                    \n"
			"    SBBO     &R13.w2, R9, 10,  2                                    \n"

			// removing start bit
			"    LSR      R13.w2, R13.w2, 1                                      \n"

			// load the arm memory lacation address where data is to be written
			"    LBBO     &R6, R8, 8,  4                                         \n"

			// Read Bits Per Character
			"    AND      r25, R5.w0, 0xF                                        \n"
			"    SUB      r25, r25, 2                                            \n"

			// Load the Bytes Done counter
			"    MOV      r22, R7.b0                                             \n"

			// check, if two byte offset is required (bits per character greater than 8)
			"    QBGE     WRITE_RX_CHAR_TO_MEM, r25, 0x8                         \n"

			// calculate the offset in memory where received character is to be written
			"    LSL      r22, r22, 1                                            \n"

			"WRITE_RX_CHAR_TO_MEM:                                               \n"
			// Write the actual data to ARM Memory
			"    SBBO     &R13.w2, R6, r22,  2                                   \n"

			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			"RESET_BITS_CNTR:                                                    \n"
			// Check for Framing Error Framing Error
			"    SUB      r24, R7.b1, 1                                          \n"

			// Reset bits done counter
			"    XOR      R7.b1, R7.b1, R7.b1                                    \n"
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			// Get the Prescalar
			"    LDI      r23, 0x03FF                                            \n"
			//scratch_reg2 hold prescalar
			"    AND      r23, r23, R4.w2                                        \n"

			// Extract timing information by detecting start transition (first left most zero)
			"    LMBD     r25, r19, 0                                            \n"
			// branch if zero start bit transition detected
			"    QBEQ     INVALID_SAMPLING_PNT, r25, 32                          \n"

			// determine the over-sampling rate
			"    LSR      r22, R4.w2, 10                                         \n"
			// OVER_SAMPLE
			"    AND      r22, r22, 0x3                                          \n"
			// 16 bit over sampling
			"    QBEQ     NXT_FRAME_SAMPLING_16BIT_OVS, r22, 0x2                 \n"

			// Calaulate sampling bit position for 8 bit over sampling
			"NXT_FRAME_SAMPLING_8BIT_OVS:                                        \n"
			// to correct  bit timing error used 4
			"    QBLT     INVALID_SAMPLING_PNT, r25, 4                           \n"

			"CAL_SAMPL_PNT8:                                                     \n"
			// start bit position - center
			"    SUB      R16.b2, r25, 0x4                                       \n"
			// sampling point
			"    AND      R16.b2, R16.b2, 7                                      \n"
			"    QBGT     UPDATE_SAMPLING_PNT, r25, 4                            \n"
			"    JMP      NXT_FRAME_SAMPLING_PNT                                 \n"

			// Calaulate sampling bit position for 16 bit over sampling
			"NXT_FRAME_SAMPLING_16BIT_OVS:                                       \n"
			// to correct  bit timing error used 4
			"    QBLT     INVALID_SAMPLING_PNT, r25, 8                           \n"

			"CAL_SAMPL_PNT16:                                                    \n"
			// start bit position - center
			"    SUB      R16.b2, r25, 0x8                                       \n"
			// sampling point
			"    AND      R16.b2, R16.b2, 15                                     \n"
			"    QBGT     UPDATE_SAMPLING_PNT, r25, 8                            \n"

			"NXT_FRAME_SAMPLING_PNT:                                             \n"
			// Increament Repeat Done Counter by One, write back to memory
			"    ADD      R7.w2, R7.w2, 1                                        \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"

			// Read Pre Scaler
			"    LDI      r23, 0x03FF                                            \n"
			"    AND      r23, r23, R4.w2                                        \n"

			// if Repeat Done count is greater than or equal to prescaler, start bit is received, jump to START_BIT
			"    QBLT     UPDATE_SAMPLING_PNT, r23, R7.w2                        \n"

			// Start bit is condition Detected properly
			"START_BIT:                                                          \n"
			// Increament Bit Count by One, and write it to memory
			"    ADD      R7.b1, R7.b1, 1                                        \n"
			"    SBBO     &R7.b1, R8, 13,  1                                     \n"

			"    XOR      R7.w2, R7.w2, R7.w2                                    \n"
			"    SBBO     &R7.w2, R8, 14,  2                                     \n"
			"    JMP      UPDATE_SAMPLING_PNT                                    \n"

			"INVALID_SAMPLING_PNT:                                               \n"
			// Reset the Sampling Point
			"    LDI      R16.b2, 0xff                                           \n"

			"UPDATE_SAMPLING_PNT:                                                \n"
			"    SBBO     &R16.b2, R9, 22,  1                                    \n"

			// read interrupt mask regsiter
			"    LBBO     &R2.w0, r20, 0x080, 2                                  \n"

			//read interrupt status regsiter
			"    LBBO     &R2.w2, r20, 0x082, 2                                  \n"

			// check for error in received data frame
			// Check for Break Condiotion Error
			"    QBGE     RX_DATA_ZERO, R13.w2, 0                                \n"

			// Framing Error: Check if the Bit at Chn_TxRxBitsDoneCtr Bit Position in the Chn_RxDataHoldReg is set
			"    QBBC     BIT_CLEARD, R13.w2, r24                                \n"

			// increament Chn_TxRxBytesDoneCtr by one
			"    ADD      R7.b0, R7.b0, 1                                        \n"
			"    SBBO     &R7.b0, R8, 12,  1                                     \n"

			// Reset the Data Hold Reg
			"    XOR      R13.w2, R13.w2, R13.w2                                 \n"
			"    SBBO     &R13.w2, R9, 10,  2                                    \n"

			// Read the request count to be received
			"    LSR      r22, R5.w0, 0x8                                        \n"
			"    AND      r22, r22, 0x0F                                         \n"
			"    ADD      r22, r22, 0x01                                         \n"

			// Read the bytes done counter
			"    MOV      r23, R7.b0                                             \n"

			// check if bytes done counter is less than or equal to data len,
			// if yes go to CHK_RX_CMPL_INT and check for raise RX complete intr
			"    QBGE     CHK_RX_CMPL_INT, R7.b0, r22                            \n"

			// if bytes done counter is greater than data len subtract data len from it and
			// check if differnce is data len, if yes raise RX complete intr
			"    SUB      r23, R7.b0, r22                                        \n"

			"CHK_RX_CMPL_INT:                                                    \n"
			// check if all data frame received or not, if RX request if complete, else receive next data frame
			"    QBLT     RxInterruptServiceRequestHndlr, r22, r23               \n"

			// All requested frame received raise interrupt to ARM/DSP, set SUART_RX_FIFO_INDX_BIT, clear SUART_TXRX_READY_BIT
			"RX_COMPLETE:                                                        \n"
			// RX Data is in lower half of Fifo, if bytes done counter is equal to data len
			"    CLR      R5.b2, R5.b2, 1                                        \n"

			// Raise the RX interrupt if Chn_TxRxBytesDoneCtr is equal to data len otherwise reset Chn_TxRxBytesDoneCtr and raise Rx interrupt
			"    QBEQ     CHK_RX_OVERRUN, R7.b0, r22                             \n"

			// reset Chn_TxRxBytesDoneCtr if Chn_TxRxBytesDoneCtr is equal to twice the data len
			"    XOR      R7.b0, R7.b0, R7.b0                                    \n"
			"    SBBO     &R7.b0, R8, 12,  1                                     \n"

			// RX data is in upper half of Fifo,if bytes done counter is equal to twice the data len
			"    SET      R5.b2, R5.b2, 1                                        \n"

			"CHK_RX_OVERRUN:                                                     \n"
			"    LDI      r23.w0, 0x284 & 0xFFFF                                 \n"
			"    LDI      r23.w2, 0x284 >> 16                                    \n"
			"    LBCO     &r22, C0, r23, 4                                       \n"
			"    ADD      r23, R10.b2, 2                                         \n"
			"    ADD      r23, R10.b2, 2                                         \n"

			"OVER_RUN_ERR:                                                       \n"
			"    QBBC     CHK_RX_READY_BIT, r22, r23                             \n"
			"    QBBC     CHK_RX_READY_BIT, R4.w2, 15                            \n"
			"    SET      R5.b2, R5.b2, 3                                        \n"
			"    SET      R5.b2, R5.b2, 2                                        \n"

			"CHK_RX_READY_BIT:                                                   \n"
			// If the receive is not activated from the host, then don't dump the data
			"    QBBC     RxInterruptServiceRequestHndlr, R5.b2, 0               \n"
			"    JMP      RX_CHN_INTR                                            \n"

			// Framing Error Detected, interrupt are masked go to DEACTIVATE_SERIALIZER, other wise update status reg
			"BIT_CLEARD:                                                         \n"
			"    SET      R5.b2, R5.b2, 4                                        \n"
			"    JMP      SET_RX_ERR_STAT                                        \n"

			// Break Condiotion Error detected, interrupt are masked go to DEACTIVATE_SERIALIZER, other wise update status reg
			"RX_DATA_ZERO:                                                       \n"
			"    SET      R5.b2, R5.b2, 5                                        \n"

			// Update the Global and Channel Error Status Registers
			"SET_RX_ERR_STAT:                                                    \n"
			"    SET      R2.w2, R2.w2, 9                                        \n"
			"    SET      R5.b2, R5.b2, 2                                        \n"

			// if global Error interrupt is clear do not raise interrupt
			"    QBBC     RxInterruptServiceRequestHndlr, R2.w0, 9               \n"

			// if Framing error status bit for channel is clear look for Break Condiotion Error
			"    QBBC     BREAK_COND_ERR, R5.b2, 4                               \n"

			// Framming Error Occurred, if Framing Error mask is not set jum to RxInterruptServiceRequestHndlr
			"FRAMING_ERR:                                                        \n"
			"    QBBS     RX_CHN_INTR, R4.w2, 12                                 \n"
			"    JMP      RxInterruptServiceRequestHndlr                         \n"

			// if Break Error Mask not set jump to RxInterruptServiceRequestHndlr
			"BREAK_COND_ERR:                                                     \n"
			"    QBBC     RxInterruptServiceRequestHndlr, R4.w2, 13              \n"

			// Set the Global interrupt status register
			"RX_CHN_INTR:                                                        \n"
			"    SET      R2.w2, R2.w2, R10.b2                                   \n"

			// write interrupt status regsiter
			"    SBBO     &R2.w2, r20, 0x082, 2                                  \n"

			// Update tx rx status regsiter status
			"    SBBO     &R5.b2, R8, 6,  1                                      \n"

			// if interrupt are masked for channel then go to RxInterruptServiceRequestHndlr, otherwise raise the interrupt
			"    QBBC     RxInterruptServiceRequestHndlr, R2.w0, R10.b2          \n"

			// Raise the interrupt to ARM/DSP
			"    JMP      PRU_TO_HOST_INTERRUPT                                  \n"

//******************************************** RxInterruptServiceRequestHndlr: ENDs **************************

//========================================================================================================================================

//******************************************** LOAD_RX_CONTEXT_ADDRESS: Start*********************************

			"LOAD_RX_CONTEXT_ADDRESS:                                            \n"
			"    QBEQ     RX_CONTEXT_CH1_ADDR, R10.b2, 1                         \n"
			"    QBEQ     RX_CONTEXT_CH3_ADDR, R10.b2, 3                         \n"
			"    QBEQ     RX_CONTEXT_CH5_ADDR, R10.b2, 5                         \n"
			"    QBEQ     RX_CONTEXT_CH7_ADDR, R10.b2, 7                         \n"
			"    jmp      r30.w0                                                 \n"

			"RX_CONTEXT_CH1_ADDR:                                                \n"
			"    LDI      R9, 0x0C0                                              \n"
			"    jmp      r30.w0                                                 \n"

			"RX_CONTEXT_CH3_ADDR:                                                \n"
			"    LDI      R9, 0x110                                              \n"
			"    jmp      r30.w0                                                 \n"

			"RX_CONTEXT_CH5_ADDR:                                                \n"
			"    LDI      R9, 0x160                                              \n"
			"    jmp      r30.w0                                                 \n"

			"RX_CONTEXT_CH7_ADDR:                                                \n"
			"    LDI      R9, 0x1B0                                              \n"
			"    jmp      r30.w0                                                 \n"

//******************************************** LOAD_RX_CONTEXT_ADDRESS Ends ***********************************

//=====================================================================================================================================

//			************************************************************************************************
//									SOFT-UART RECEIVE ROUTINE : ENDS
//			************************************************************************************************

//=====================================================================================================================================

	);
}
