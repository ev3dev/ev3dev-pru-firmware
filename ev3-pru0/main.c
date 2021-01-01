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
	// Clear the ZERO Register r20
	asm(" XOR      r20, r20, r20");

	//--------------------- McASP TX Initialization - Starts ----------------

	// activate clocks, serializers, state machine and frame sync
	asm("tx_asp_init1:");
	// Activate the high-transmit clock XHCLKRST
	asm(" LBCO     &r22, C25, 0xA0, 4");
	asm(" SET      r22, r22, 9");
	asm(" SBCO     &r22, C25, 0xA0, 4");

	asm("tx_asp_init2:");
	asm(" LBCO     &r22, C25, 0xA0, 4");
	asm(" QBBC     tx_asp_init2, r22, 9");

	//Activate the transmit frequency clock XCLKRST
	asm(" SET      r22, r22, 8");
	asm(" SBCO     &r22, C25, 0xA0, 4");

	asm("tx_asp_init3:");
	asm(" LBCO     &r22, C25, 0xA0, 4");
	asm(" QBBC     tx_asp_init3, r22, 8");

	// Before starting, clear the respective transmitter and receiver status registers by writing 0xffff
	asm(" LDI      r23.w0, 0xffff & 0xFFFF");
	asm(" LDI      r23.w2, 0xffff >> 16");
	asm(" SBCO     &r23, C25, 0xc0, 2");

	// Activate serializer, XSRCLR
	asm(" SET      r22, r22, 10");
	asm(" SBCO     &r22, C25, 0xA0, 4");

	asm("tx_asp_init4:");
	asm(" LBCO     &r22, C25, 0xA0, 4");
	asm(" QBBC     tx_asp_init4, r22, 10");

	// Till now no serializer is activated for TX, so no need to service all active XBUF
	// to avoid underrun errors to be done

	// Actiavte the McASP state machine
	asm(" SET      r22, r22, 11");
	asm(" SBCO     &r22, C25, 0xA0, 4");

	asm("tx_asp_init5:");
	asm(" LBCO     &r22, C25, 0xA0, 4");
	asm(" QBBC     tx_asp_init5, r22, 11");

	// Activate the MCASP Frame sync
	asm(" SET      r22, r22, 12");
	asm(" SBCO     &r22, C25, 0xA0, 4");

	asm("tx_asp_init6:");
	asm(" LBCO     &r22, C25, 0xA0, 4");
	asm(" QBBC     tx_asp_init6, r22, 12");

	//----------------------- McASP TX Initialization - Ends ------------------

	//--------------------- McASP RX Initialization - Starts ----------------

	// activate Clocks,Serializers,state machine and frame sync
	asm("rx_asp_init1:");
	// Activate the high-transmit clock RHCLKRST
	asm(" LBCO     &r22, C25, 0x60, 4");
	asm(" SET      r22, r22, 1");
	asm(" SBCO     &r22, C25, 0x60, 4");

	asm("rx_asp_init2:");
	asm(" LBCO     &r22, C25, 0x60, 4");
	asm(" QBBC     rx_asp_init2, r22, 1");

	//Activate the transmit frequency clock RCLKRST
	asm(" SET      r22, r22, 0");
	asm(" SBCO     &r22, C25, 0x60, 4");

	asm("rx_asp_init3:");
	asm(" LBCO     &r22, C25, 0x60, 4");
	asm(" QBBC     rx_asp_init3, r22, 0");

	// Clear RSTAT
	asm(" LDI      r23, 0xffff");
	asm(" SBCO     &r23, C25, 0x80, 2");

	// Activate serializer, RSRCLR
	asm(" SET      r22, r22, 2");
	asm(" SBCO     &r22, C25, 0x60, 4");

	asm("rx_asp_init4:");
	asm(" LBCO     &r22, C25, 0x60, 4");
	asm(" QBBC     rx_asp_init4, r22, 2");

	// Activate the McASP state machine
	asm(" SET      r22, r22, 3");
	asm(" SBCO     &r22, C25, 0x60, 4");

	asm("rx_asp_init5:");
	asm(" LBCO     &r22, C25, 0x60, 4");
	asm(" QBBC     rx_asp_init5, r22, 3");

	// Activate the MCASP Frame sync
	asm(" SET      r22, r22, 4");
	asm(" SBCO     &r22, C25, 0x60, 4");

	asm("rx_asp_init6:");
	asm(" LBCO     &r22, C25, 0x60, 4");
	asm(" QBBC     rx_asp_init6, r22, 4");

	//----------------------- McASP RX Initialization - Ends ------------------

	asm("LOCAL_INIT:");
	//Read the PRU ID
	asm(" LBBO     &r23, r20, 0x084, 4");
	asm(" MOV      R3.b0, r23.b0");

	// Read the PRU mode
	asm(" MOV      R3.b1, r23.b1");

	//PRU Delay Count in CORE_LOOP
	asm(" MOV      R3.b2, r23.b2");

	//Clear RSTAT
	asm(" LDI      r23, 0xffff");
	asm(" SBCO     &r23, C25, 0x80, 4");

	//Clear XSTAT
	asm(" LDI      r23, 0xffff");
	asm(" SBCO     &r23, C25, 0xc0, 4");

	asm(" QBEQ     CORE_LOOP, R3.b1, 0x1");

	// This Block the Sampling Point with invalid value in RX Context Area
	asm(" LDI      r23, 0xFF");
	asm(" XOR      r24, r24, r24");

	asm(" QBEQ     PRUxxxx_MODE_RX_ONLY, R3.b1, 0x2");

	asm(" LDI      r22, 0x0C0");
	asm(" LDI      r25, 0x50");
	asm(" LDI      r24, 0x1B0");
	asm(" JMP      INIT_SAMPLE_PNT");

	asm("PRUxxxx_MODE_RX_ONLY:");
	asm(" LDI      r22, 0x90");
	asm(" LDI      r25, 0x20");
	asm(" LDI      r24, 0x170");

	asm("INIT_SAMPLE_PNT:");
	asm(" SBBO     &r23, r22, 22, 1");
	asm(" SBBO     &r20.b0, r22, 23, 1");

	asm(" ADD      r22, r22, r25");
	asm(" QBGE     INIT_SAMPLE_PNT, r22, r24");

	// JUMP  to CORE_LOOP
	asm(" JMP      CORE_LOOP");

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

	asm("TxServiceRequestHndlr:");
	//read interrupt status regsiter
	asm(" LBBO     &R2.w2, r20, 0x082, 2");

	// clear the channel interrupt status bit
	asm(" CLR      R2.w2, R2.w2, R10.b2");

	//update interrupt status regsiter
	asm(" SBBO     &R2.w2, r20, 0x082, 2");

	//Clear Service Request
	asm(" CLR      R4.w0, R4.w0, 0x2");
	asm(" SBBO     &R4.w0, R8, 0,  2");

	// Set the TXRX_READY_BIT
	asm(" SET      R5.b2, R5.b2, 0");
	asm(" SBBO     &R5.b2, R8, 6,  1");

	// Set SUART_CH_TXRXCHNSTATUS_BIT bit in channel status to indicate the channel active
	asm(" SET      R5.b3, R5.b3, 7");
	asm(" SBBO     &R5.b3, R8, 7,  1");

	// New Tx Request received initialize the Channel specific data and save it in memory
	asm(" XOR      R7.b0, R7.b0, R7.b0");
	asm(" SBBO     &R7.b0, R8, 12,  1");

	// Load channel specific serializer, xbuf, srctl register mapped active channel
	asm(" JMP      LOAD_TX_COMMON_INFO");

	asm("ENABLE_TX_SERIALIZER:");
	//Change the MCASP AXR[n] pin from GPIO mode to MCASP mode of operation
	asm(" LBCO     &r23, C25, 0x10, 4");
	asm(" AND      r22, R4.b1, 0x0F");
	asm(" CLR      r23, r23, r22");
	asm(" SBCO     &r23, C25, 0x10, 4");

	asm("CLEAR_XSTAT:");
	asm(" LDI      r22, 0xFFFF");
	asm(" SBCO     &r22, C25, 0xc0, 4");
	asm(" JMP      MCASP_EVENT");

//******************************************** TxServiceRequestHndlr Ends ************************************

//=====================================================================================================================================

//******************************************** TxServiceReqHndlLoop Starts ***********************************

//=====================================================================================================================================
//  This routine reads the formated data to be transmitted from formatted data area region mapped to
//  current serviced TX channel and depending upon prescalar value in config1 register, it jumps to that
// 	that prescalar label. This is getting called from TX interrupt handler or when is there new service request for TX.
//=====================================================================================================================================

	asm("TxServiceReqHndlLoop:");
	// Read the Formated byte to transmitted
	asm(" JAL      r30.w0, READ_TX_DATA");

	asm(" XOR      r28.w0, r28.w0, r28.w0");

	// Branch According to Pre-Scalar Value
	asm(" LDI      r22, 0x03FF");
	asm(" AND      r22, r22, R4.w2");

	asm(" QBEQ     PRE_SCALAR1, r22, 0x1");
	asm(" QBEQ     PRE_SCALAR2, r22, 0x2");
	asm(" QBEQ     PRE_SCALAR4, r22, 0x4");
	asm(" QBEQ     PRE_SCALAR6, r22, 0x6");
	asm(" QBEQ     PRE_SCALAR12, r22, 0xC");
	asm(" QBEQ     PRE_SCALAR16, r22, 0x10");
	asm(" QBLE     PRE_SCALAR24, r22, 0x18");

//******************************************** TxServiceReqHndlLoop ENDS *************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR1 Starts ********************************************

	asm("PRE_SCALAR1:");
	// copy data to RAM TX_DATA_reg.w0 register from scratch_reg3
	asm(" MOV      r28.w0, r24");

	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	// Increament the Chn_TxRxBytesDoneCtr bye one
	asm(" ADD      R7.b0, R7.b0, 1");
	asm(" SBBO     &R7.b0, R8, 12,  1");

	asm(" JMP      TxInterruptServiceRequestHndlr");

//******************************************** PRE_SCALAR1 Ends **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR2 Starts ********************************************

	asm("PRE_SCALAR2:");
	asm(" MOV      r22, R7.b1");
	asm(" QBGT     XMIT_FISRT_8BIT, R7.b1, 1");

	asm("XMIT_LAST_8BIT:");
	//Last 8 bits to transmitted
	asm(" LSR      r24, r24, 8");

	asm(" JAL      r30.w0, PRESACLE_TX_DATA");

	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	asm(" JMP      TX_DONE");

	asm("XMIT_FISRT_8BIT:");
	asm(" AND      r24, r24, 0x00FF");
	asm(" JAL      r30.w0, PRESACLE_TX_DATA");

	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	// Write To RAM number of Bits Transmitted
	// 8 bits transmitted
	asm(" ADD      R7.b1, R7.b1, 8");
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// If bit per character less than 8  // added with start and stop bit in bits per channel
	asm(" MOV      r22, R5.w0");
	asm(" AND      r22, r22, 0xF");
	//check  (Chn_Config2.BitsPerChar <= 8)
	asm(" QBGE     TX_DONE, r22, 0x8");
	asm(" JMP      TxInterruptServiceRequestHndlr");

//******************************************** PRE_SCALAR2 ENDs **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR4 Starts ********************************************

	asm("PRE_SCALAR4:");
	asm(" MOV      r22, R7.b1");
	asm(" QBGT     XMIT_FIRST_4BIT, r22, 1");

	asm("XMIT_NXT_4BIT:");
	//Chn_Config2.BitsPerChar - Chn_TxRxBitsDoneCntr
	asm(" AND      r23, R5.w0, 0xF");
	asm(" SUB      r23, r23, R7.b1");

	// (Chn_Config2.BitsPerChar - Chn_TxRxBitsDoneCntr) > 4, more bits to be transmitted
	asm(" QBLT     MORE_DATA4, r23, 4");

	//transmit last remaining 4 bits
	asm(" LSR      r24, r24, r22");
	asm(" AND      r24, r24, 0xF");

	asm(" JAL      r30.w0, PRESACLE_TX_DATA");
	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	asm(" JMP      CHK_TX_DONE");

	asm("MORE_DATA4:");
	//transmit next 4 bit of present byte being transmitted
	asm(" LSR      r24, r24, r22");
	asm(" AND      r24, r24, 0xF");

	asm(" JAL      r30.w0, PRESACLE_TX_DATA");

	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	// Check all bits have been transmitted
	asm("CHK_TX_DONE:");
	// Updating number of bits written
	asm(" ADD      R7.b1, R7.b1, 4");

	// Write To RAM number of Bits Transmitted
	asm(" SBBO     &R7.b1, R8, 13,  1");

	asm(" AND      r23, R5.w0, 0xF");

	// check if all bits have been transmitted
	asm(" QBGE     TX_DONE, r23, R7.b1");
	asm(" JMP      TxInterruptServiceRequestHndlr");

	// transmit first 4 bit of formated data
	asm("XMIT_FIRST_4BIT:");
	asm(" AND      r24, r24, 0xF");
	asm(" JAL      r30.w0, PRESACLE_TX_DATA");
	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	//Updating number of bits written
	asm(" ADD      R7.b1, R7.b1, 4");

	// Write To RAM number of Bits Transmitted
	asm(" SBBO     &R7.b1, R8, 13,  1");

	asm(" JMP      TxInterruptServiceRequestHndlr");

//******************************************** PRE_SCALAR4 Ends **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR6 Starts ********************************************

	asm("PRE_SCALAR6:");
	// transmit first 3 bit of formated data
	asm(" QBGT     XMIT_FIRST_3BIT, R7.b1, 1");

	asm("GENERIC_TRANSMIT_BLOCK:");
	// initialize the register
	asm(" LDI      R13.b3, 0x0");
	asm(" XOR      r21.b1, r21.b1, r21.b1");

	asm("LOAD_BITS_LOOP_FOR6:");
	asm(" AND      r23, R5.w0, 0xF");

	// transmit the next bits if (ChnTxRxBitsDoneCntr < Chn_Config2.BitsPerChar)
	asm(" QBLT     XMIT_NXT_3BIT, r23, R7.b1");

	// transmit the last remaining bits of the present byte if any and updated counters as below
	asm("XMIT_MORE_BITS:");
	// update the bytes done counter and reset the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
	asm(" ADD      R7.b0, R7.b0, 1");
	asm(" SBBO     &R7.b0, R8, 12,  1");

	asm(" XOR      R7.b1, R7.b1, R7.b1");
	asm(" SBBO     &R7.b1, R8, 13,  1");

	asm(" XOR      R7.w2, R7.w2, R7.w2");
	asm(" SBBO     &R7.w2, R8, 14,  2");

	// set the remaining bits to one, if there are no more bits in formated data to send
	// and still there is space in TX_DATA_reg.
	// 16 - bitsLoaded
	asm(" RSB      r22, R13.b3, 16");
	// Load the remaining (16 - bitsLoaded) bits with logic High
	asm(" XOR      r24, r24, r24");
	asm(" NOT      r24, r24");

	// calculate the bit position from where one is to be inserted
	asm(" RSB      r21.b1, r22, 16");
	// CLR scratch_reg2
	asm(" XOR      r23, r23, r23");
	// COPY 1 bit to scratch_reg2 from scratch_reg3
	asm(" AND      r23, r24, 0x1");
	asm(" LSL      r23, r23, r21.b1");

	// Now, set the remaining bits to one in TX_DATA_reg
	asm("SET_BIT_BIT:");
	asm(" OR       r28.w0, r28.w0, r23");
	asm(" LSL      r23, r23, 1");
	asm(" SUB      r22, r22, 1");
	asm(" QBLE     SET_BIT_BIT, r22, 1");

	asm(" LDI      R13.b3, 16");
	asm(" JMP      CHK_MORE_PRESCALAR");
	asm(" JMP      TxInterruptServiceRequestHndlr");

	asm("XMIT_NXT_3BIT:");
	// if the bitsLoaded in TX_DATA_reg is less than 16 load the next bits
	// (bitsLoaded < 16)
	asm(" QBLT     CHK_MORE_PRESCALAR, R13.b3, 16");

	asm("BIT_LOAD16:");
	// Read Prescalar value
	asm(" LDI      r23, 0x03FF");
	asm(" AND      r22, r23, R4.w2");

	// (16 - bitsLoaded)
	asm(" RSB      r23, R13.b3, 16");
	// (Chn_Config1.PreScaller - ChnTxRxRepeatDoneCntr)
	asm(" SUB      r22, r22, R7.w2");
	asm(" MIN      r22, r22, r23");

	// Read Next Bit
	asm(" JAL      r30.w0, READ_TX_DATA");
	asm(" LSR      r24, r24, R7.b1");

	// copy bit to transmitted to scratch_reg2
	asm(" AND      r23, r24, 0x1");
	// move repeat count to scratch_reg4
	asm(" MOV      r25, r22");
	// shift the bit to be transmitted to expected position
	asm(" LSL      r23, r23, R13.b3");

	// prescale the bit to transmitted
	asm("PRESCALE_NXT_BIT:");
	asm(" OR       r28.w0, r28.w0, r23");
	asm(" LSL      r23, r23, 1");
	asm(" SUB      r22, r22, 1");
	asm(" QBLE     PRESCALE_NXT_BIT, r22, 1");

	// write back to memory
	asm(" ADD      R13.b3, R13.b3, r25");

	asm(" ADD      R7.w2, R7.w2, r25");
	asm(" SBBO     &R7.w2, R8, 14,  2");

	// get the prescalar value
	asm(" LDI      r23, 0x03FF");
	asm(" AND      r22, r23, R4.w2");

	//if bit has been transmitted prescaler times, fall through and updated the Chn_TxRxBitsDoneCtr and Chn_TxRxRepeatDoneCtr
	asm(" QBGT     CHK_MORE_PRESCALAR, R7.w2, r22");

	// rename to TX_BIT_DONE_CNTR
	asm("TX_DONE_CNTR6:");
	// Write Chn_TxRxBitsDoneCtr
	asm(" ADD      R7.b1, R7.b1, 1");
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// Write Chn_TxRxRepeatDoneCtr
	asm(" XOR      R7.w2, R7.w2, R7.w2");
	asm(" SBBO     &R7.w2, R8, 14,  2");

	//rename this label to CHK_TX_DATA_REG_FULL
	asm("CHK_MORE_PRESCALAR:");
	// if (bitsLoaded < 16), next bit can be loaded in TX_DATA_reg
	asm(" QBGT     LOAD_BITS_LOOP_FOR6, R13.b3, 0x10");
	// TX_DATA_reg is full, transmit the data
	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");
	asm(" JMP      TxInterruptServiceRequestHndlr");

	// transmit the bits from start bit that can be transmitted from present character that is to transmitted
	asm("XMIT_FIRST_3BIT:");
	// copy the first 3 bits to be transmitted
	asm(" AND      r24, r24, 0x7");
	// number of times the byte loop is to be looped
	asm(" LDI      r23, 12");
	// Clear TX_DATA_reg.w0
	asm(" XOR      r28.w0, r28.w0, r28.w0");
	asm(" XOR      r21.b1, r21.b1, r21.b1");
	asm(" JAL      r30.w0, BYTE_LOOP");

	// Repeat last bit by 4 times
	asm(" LDI      r22, 0x4");

	// CLR scratch_reg2
	asm(" XOR      r23, r23, r23");
	// copy the third bit to scratch_reg2
	asm(" AND      r23, r24, 0x1");

	// shift the bit to expected place i.e. bit ps 12
	asm(" LSL      r23, r23, 0xc");

	// prescale the last bit 4 times
	asm("PRESCALE_LAST_4BIT:");
	asm(" OR       r28.w0, r28.w0, r23");
	asm(" LSL      r23, r23, 1");
	asm(" SUB      r22, r22, 1");
	asm(" QBLE     PRESCALE_LAST_4BIT, r22, 1");

	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	// Updating number of bits written
	asm(" ADD      R7.b1, R7.b1, 2");
	// Write To RAM number of Bits Transmitted
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// Updating number of bits written
	asm(" ADD      R7.w2, R7.w2, 4");
	// Write To RAM Write Repeat done counter to RAM
	asm(" SBBO     &R7.w2, R8, 14,  2");

	asm(" JMP      TxInterruptServiceRequestHndlr");

//******************************************** PRE_SCALAR6 ENDs **********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR12 Starts *******************************************

	asm("PRE_SCALAR12:");
	asm(" QBGT     XMIT_FIRST_2BIT, R7.b1, 1");
	asm(" JMP      GENERIC_TRANSMIT_BLOCK");

	asm("XMIT_FIRST_2BIT:");
	// copy the first two bit to be loaded in  TX_DATA_reg
	asm(" AND      r24, r24, 0x3");
	// To left shift each copied data bit
	asm(" LDI      r21.b1, 0x0");
	// Keep track of byte_loop loop count
	asm(" LDI      r23, 12");
	asm(" XOR      r21.b1, r21.b1, r21.b1");
	asm(" JAL      r30.w0, BYTE_LOOP");

	// CLR scratch_reg2
	asm(" XOR      r23, r23, r23");
	// copy the next bit to prescaled
	asm(" AND      r23, r24, 0x1");
	// counter to prescale second bit by 4
	asm(" LDI      r22, 4");
	// shift the bit to desired position
	asm(" LSL      r23, r23, 0xC");

	asm("PRESCALE_4BIT:");
	asm(" OR       r28.w0, r28.w0, r23");
	asm(" LSL      r23, r23, 1");
	asm(" SUB      r22, r22, 1");
	asm(" QBLE     PRESCALE_4BIT, r22, 1");

	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	// Updating number of bits written
	asm(" ADD      R7.b1, R7.b1, 1");
	// Write To RAM number of Bits Transmitted
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// Updating number of bits written
	asm(" ADD      R7.w2, R7.w2, 4");
	// Write To RAM number of Bits Repeated
	asm(" SBBO     &R7.w2, R8, 14,  2");

	asm(" JMP      TxInterruptServiceRequestHndlr");

//******************************************** PRE_SCALAR12 ENDs *********************************************

//======================================================================================================================================

//******************************************** PRE_SCALAR16 Starts *******************************************

	asm("PRE_SCALAR16:");
	asm(" QBGT     XMIT_FIRST_16, R7.b1, 1");
	asm(" JMP      GENERIC_TRANSMIT_BLOCK");

	asm("XMIT_FIRST_16:");
	// copy the first two bit to be loaded in  TX_DATA_reg
	asm(" AND      r24, r24, 0x2");
	// Left shift each copied data bit
	asm(" LDI      r21.b1, 0x0");
	// Keep track of byte_loop loop count
	asm(" LDI      r23, 16");
	// Clear TX_DATA_reg.w0
	asm(" XOR      r28.w0, r28.w0, r28.w0");
	asm(" XOR      r21.b1, r21.b1, r21.b1");
	asm(" JAL      r30.w0, BYTE_LOOP");

	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	// Updating number of bits written
	asm(" ADD      R7.b1, R7.b1, 1");
	// Write To RAM number of Bits Transmitted
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// Updating number of bits written
	asm(" ADD      R7.w2, R7.w2, 0");
	// Write To RAM number of Bits Repeated
	asm(" SBBO     &R7.w2, R8, 14,  2");

	asm(" JMP      TxInterruptServiceRequestHndlr");

//******************************************** PRE_SCALAR16 ENDs *********************************************

//======================================================================================================================================

//********************************************* PRE_SCALAR24 Starts ******************************************

	asm("PRE_SCALAR24:");
	asm(" QBGT     XMIT_FIRST_24, R7.b1, 1");
	asm(" JMP      GENERIC_TRANSMIT_BLOCK");

	asm("XMIT_FIRST_24:");
	asm(" LDI      r23, 0x03FF");
	asm(" AND      r22, r23, R4.w2");
	// Chn_TxRxConfig1.PreScaler - ChnTxRxRepeadDoneCnt
	asm(" SUB      r22, r22, R7.w2");
	//(Chn_TxRxConfig1.PreScaler - ChnTxRxRepeadDoneCntr >= 16 )
	asm(" QBLE     PRESCALE_START_BIT, r22, 16");

	asm("PRESCALE_FIRST_DATA_BIT:");
	// Clear Scratch reg
	asm(" XOR      r24, r24, r24");
	// Updating number of bits written
	asm(" ADD      R7.b1, R7.b1, 1");

	asm(" JAL      r30.w0, READ_TX_DATA");

	// get the bits to be transmitted
	asm(" LSR      r24, r24, R7.b1");
	asm(" AND      r23, r24, 0x1");

	// shift the bit to desired bit position
	asm(" LSL      r23, r23, r22");
	asm(" RSB      r22, r22, 16");
	asm(" MOV      R7.w2, r22");

	asm("PRESCALE_FIRST_DAT_BIT:");
	asm(" OR       r28.w0, r28.w0, r23");
	asm(" LSL      r23, r23, 1");
	asm(" SUB      r22, r22, 1");
	asm(" QBLE     PRESCALE_FIRST_DAT_BIT, r22, 1");

	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	// Write To RAM number of Bits Transmitted
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// Write To RAM Chn_TxRxRepeatDoneCtr
	asm(" SBBO     &R7.w2, R8, 14,  2");
	asm(" JMP      TxInterruptServiceRequestHndlr");

	asm("PRESCALE_START_BIT:");
	asm(" LDI      r22, 0x10");
	asm(" LDI      r23, 0x10");
	// to left shift each copied data bit
	asm(" XOR      r21.b1, r21.b1, r21.b1");
	asm(" JAL      r30.w0, BITS_LOOP");
	asm(" JAL      r30.w0, TRANSMIT_PRESCALED_DATA");

	// Update number of bits written
	asm(" ADD      R7.w2, R7.w2, 16");
	// Write To RAM number of Bits Repeated
	asm(" SBBO     &R7.w2, R8, 14,  2");
	asm(" JMP      TxInterruptServiceRequestHndlr");

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

	asm("LOAD_TX_COMMON_INFO:");
	// Load the TX Format Address for the specific channel
	asm(" QBEQ     LOAD_TX_FORMAT_ADDRESS_DONE, R3.b1, 0x1");
	asm(" JAL      r30.w0, LOAD_TX_FORMAT_ADDRESS");

	asm("LOAD_TX_FORMAT_ADDRESS_DONE:");
	//  Load the mapped SR and XBUF address mapped to channel
	asm(" JMP      LOCATE_SR_XBUF_SRCTL");

	asm("LOCATE_SR_XBUF_SRCTL_DONE:");
	// Format the data if required
	asm(" JMP      CHK_TX_DATA_FORMAT");

	asm("CHK_TX_DATA_FORMAT_DONE:");
	asm(" JMP      ENABLE_TX_SERIALIZER");

//****************************************** LOAD_TX_COMMON_INFO: ENDS ***************************************

//======================================================================================================================================

//****************************************** TX LOCATE_SR_XBUF_SRCTL : Starts ********************************

//======================================================================================================================================
// 		Subroutine to find channel specific serializer, xbuf, srctl register mapped active channel
//======================================================================================================================================

	asm("LOCATE_SR_XBUF_SRCTL:");
	// Calculating Serializer Mapped to Channel
	asm(" AND      r22, R4.b1, 0x0F");
	asm(" LSL      r22, r22, 2");

	// copy the tx format address to temp regsiter
	asm(" MOV      r24, R13.w0");
	asm(" LDI      r25, 0x20");

	// Calculating the specific SRCTL register offset
	asm(" LDI      R11.w0, (0x01D00180) & 0xFFFF");
	asm(" LDI      R11.w2, (0x01D00180) >> 16");
	asm(" ADD      R11, R11, r22");

	asm(" ADD      r23, r25, 0");
	asm(" SBBO     &R11, r24, r23,  4");

	// Calculating the specific xbuf register offset
	asm(" LDI      R12.w0, (0x01D00200) & 0xFFFF");
	asm(" LDI      R12.w2, (0x01D00200) >> 16");
	asm(" ADD      R12, R12, r22");

	asm(" ADD      r23, r25, 4");
	asm(" SBBO     &R12, r24, r23,  4");

	// Store the data length
	asm(" MOV      R13.b2, R5.b1");

	asm(" ADD      r23, r25, 10");
	asm(" SBBO     &R13.b2, r24, r23,  1");

	//Store the data Tx FMT Context address
	asm(" ADD      r23, r25, 8");
	asm(" SBBO     &R13.w0, r24, r23,  2");

	asm(" LDI      R13.b3, 0x00");
	asm(" ADD      r23, r25, 11");
	asm(" SBBO     &R13.b3, r24, r23,  1");

	asm(" JMP      LOCATE_SR_XBUF_SRCTL_DONE");

//********************************************** TX LOCATE_SR_XBUF_SRCTL: ENDS **************************************

//======================================================================================================================================

//********************************************** TX CHK_TX_DATA_FORMAT : Starts***************************************

//======================================================================================================================================
// 	Check For Data Formating, formats the data only if Chn_TxRxRepeatDoneCtr,
//	Chn_TxRxBitsDoneCtr, Chn_TxRxBytesDoneCtr all are zero,
//	If All the conditions is satisfied, it jumps to TX_DATA_FORMAT Subroutine  and
// 	formats data the TX Data obtained from ARM/DSP by adding start and stop bit.
//======================================================================================================================================

	asm("CHK_TX_DATA_FORMAT:");
	asm(" QBEQ     CHK_TX_DATA_FORMAT_BITS, R7.w2, 0");
	asm(" JMP      CHK_TX_DATA_FORMAT_DONE");

	asm("CHK_TX_DATA_FORMAT_BITS:");
	asm(" QBEQ     CHK_TX_DATA_FORMAT_BYTE, R7.b1, 0");
	asm(" JMP      CHK_TX_DATA_FORMAT_DONE");

	asm("CHK_TX_DATA_FORMAT_BYTE:");
	asm(" QBEQ     TX_DATA_FORMAT, R7.b0, 0");
	asm(" JMP      CHK_TX_DATA_FORMAT_DONE");

	asm("TX_DATA_FORMAT:");
	// Load the TX Format Address for the specific channel
	asm(" XOR      r22, r22, r22");
	asm(" NOT      r22, r22");

	asm(" SUB      r23, R5.b0, 1");

	asm(" LSL      r22, r22, r23");
	// offset from base addr
	asm(" XOR      r23, r23, r23");

	// to store formated data into DATA RAM
	asm(" MOV      r24, R13.w0");

	// Number of Bits Per Character
	asm(" AND      r21.b0, R5.b0, 0xF");
	asm(" SUB      r21.b0, r21.b0, 2");

	asm("TX_DATA_FORMAT_LOOP:");
	// Load the data from the data pointer
	asm(" LBBO     &r28, R6, 0x00, 2");
	asm(" LSL      r28, r28, 1");
	asm(" OR       r28, r28, r22");

	// store formated data into DATA RAM
	asm(" SBBO     &r28.w0, r24, r23, 2");

	// Increment the formatted buffer address offset
	asm(" ADD      r23, r23, 2");

	asm(" QBGE     INC_ADDR_BY_ONE, r21.b0, 0x8");
	// Next data buffer pointer
	asm(" ADD      R6, R6, 1");

	// Increamnet the tx buffer data pointer by ONE, if bit per character is less or equal to 8 including start and stop bit
	asm("INC_ADDR_BY_ONE:");
	// Next data buffer pointer
	asm(" ADD      R6, R6, 1");

	asm(" QBEQ     CHK_TX_DATA_FORMAT_DONE, R13.b2, 0");

	//Decrement the data length .i.e no of bytes to send
	asm(" SUB      R13.b2, R13.b2, 1");

	asm(" JMP      TX_DATA_FORMAT_LOOP");

//******************************************** TX CHK_TX_DATA_FORMAT: ENDS************************************

//======================================================================================================================================

//******************************************** TX READ_TX_DATA: Starts****************************************

//======================================================================================================================================
// 	Reads the 16 bit formatted character to be transmitted from formatted data area corresponding to TX channel
//======================================================================================================================================

	asm("READ_TX_DATA:");
	// Copy the base address of formated data
	asm(" MOV      r24, R13.w0");

	// Calculate the offset of formated data
	asm(" LSL      r25, R7.b0, 1");

	// LOAD formated data from DATA RAM
	asm(" LBBO     &r24, r24, r25, 2");

	asm(" jmp      r30.w0");

//********************************************** TX READ_TX_DATA: ENDS ***************************************

//======================================================================================================================================

//********************************************** TX LOAD_TX_FORMAT_ADDRESS : Starts **************************

//======================================================================================================================================
//	Initializes the TX formatted data buffer address which stores the formated data with stop and start bit
//======================================================================================================================================

	asm("LOAD_TX_FORMAT_ADDRESS:");
	asm(" QBEQ     TX_CH0_FMT_ADDR, R10.b2, 0");
	asm(" QBEQ     TX_CH2_FMT_ADDR, R10.b2, 2");
	asm(" QBEQ     TX_CH4_FMT_ADDR, R10.b2, 4");
	asm(" QBEQ     TX_CH6_FMT_ADDR, R10.b2, 6");
	asm(" jmp      r30.w0");

	asm("TX_CH0_FMT_ADDR:");
	asm(" LDI      R13.w0, 0x090");
	asm(" jmp      r30.w0");

	asm("TX_CH2_FMT_ADDR:");
	asm(" LDI      R13.w0, 0x0E0");
	asm(" jmp      r30.w0");

	asm("TX_CH4_FMT_ADDR:");
	asm(" LDI      R13.w0, 0x130");
	asm(" jmp      r30.w0");

	asm("TX_CH6_FMT_ADDR:");
	asm(" LDI      R13.w0, 0x180");
	asm(" jmp      r30.w0");

//******************************************** TX LOAD_TX_FORMAT_ADDRESS Routine: ENDS ***********************

//======================================================================================================================================

//******************************************** PRESACLE_TX_DATA : Starts *************************************

//======================================================================================================================================
// This routine Prescales data bit to be transmitted into the TX_DATA_reg.w0 register
//======================================================================================================================================

	asm("PRESACLE_TX_DATA:");
	// to Left shift each copied data bit
	asm(" XOR      r21.b1, r21.b1, r21.b1");
	// Xbuf Size: Keep track of byte loop
	asm(" LDI      r23, 16");
	// Clear TX_DATA_reg.w0
	asm(" XOR      r28.w0, r28.w0, r28.w0");

	asm("BYTE_LOOP:");
	asm(" LDI      r25, 0x03FF");
	asm(" AND      r22, r25, R4.w2");

	asm("BITS_LOOP:");
	// CLR scratch_reg4
	asm(" XOR      r25, r25, r25");
	// COPY 1 bit to scratch_reg4 from scratch_reg3
	asm(" AND      r25, r24, 0x1");
	asm(" LSL      r25, r25, r21.b1");
	asm(" OR       r28.w0, r28.w0, r25");
	asm(" ADD      r21.b1, r21.b1, 1");
	// INC Bytes loop counter
	asm(" SUB      r23, r23, 1");
	// INC Bits loop counter
	asm(" SUB      r22, r22, 1");
	asm(" QBLE     BITS_LOOP, r22, 1");
	asm(" LSR      r24, r24, 1");
	asm(" QBLE     BYTE_LOOP, r23, 1");
	// Return to PRESACLE_TX_DATA
	asm(" jmp      r30.w0");

//********************************************  PRESACLE_TX_DATA : ENDs **************************************

//======================================================================================================================================

//******************************************** TRANSMIT_PRESCALED_DATA : Starts ******************************

//======================================================================================================================================
// This routine Transmits the prescaled data by writing to mcasp x_buf register mapped to this serializer
//======================================================================================================================================

	asm("TRANSMIT_PRESCALED_DATA:");
	// Clear the under run error
	asm(" LBCO     &r22, C25, 0xc0, 4");
	asm(" QBBC     WRITE_TO_XBUF, r22, 0x8");

	asm(" LDI      r22, 0xFFFF");
	asm(" SBCO     &r22, C25, 0xc0, 4");

	asm("WRITE_TO_XBUF:");
	// Write Byte to X_BUF
	asm(" SBBO     &r28.w0, R12, 00, 4");
	// return from Transmit Prescaled Data
	asm(" jmp      r30.w0");

//******************************************** TRANSMIT_PRESCALED_DATA : ENDs ********************************

//======================================================================================================================================

//******************************************** TX_DONE : Starts **********************************************

//======================================================================================================================================
// This routine the cleanup after one character has been transmitted successfully
//======================================================================================================================================

	asm("TX_DONE:");
	asm(" XOR      R7.b1, R7.b1, R7.b1");
	// Write To RAM number of Bits Transmitted
	asm(" SBBO     &R7.b1, R8, 13,  1");

	asm(" ADD      R7.b0, R7.b0, 1");
	asm(" SBBO     &R7.b0, R8, 12,  1");

	asm(" JMP      TxInterruptServiceRequestHndlr");

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

	asm("TxInterruptServiceRequestHndlr:");
	// Retrieve the channel number and load the context base
	asm(" LDI      R8, 0x0000");
	asm(" LDI      R10.w0, 0x0000");
	asm(" LDI      R10.b2, 0x00");
	asm(" LDI      R13.w0, 0x90");

	asm("SERCH_MAPPED_TX_CHN:");
	asm(" ADD      r22, R10.w0, 7");
	// Load the Channel Cntrl info from Memory to Register
	asm(" LBBO     &R5.b3, R8, r22,  1");
	asm(" QBBC     NEXT_TX_CHN, R5.b3, 7");

	asm(" ADD      r22, R13.w0, 0x20");
	asm(" LBBO     &r23, r22, 0, 4");
	asm(" LBBO     &r22, r23, 0, 4");
	asm(" QBBS     MAPPED_TX_CHN_FOUND, r22, 4");

	asm("NEXT_TX_CHN:");
	asm(" QBEQ     PRU_TX_ONLY_MODE, R3.b1, 0x1");

	// TX & RX together. So channel nunbers are 0, 2, 4, 6
	asm(" ADD      R10.w0, R10.w0, 0x20");
	asm(" ADD      R10.b2, R10.b2, 0x02");
	asm(" ADD      R13.w0, R13.w0, 0x50");
	asm(" QBGE     SERCH_MAPPED_TX_CHN, R10.b2, (8 - 1)");
	asm(" JMP      CORE_LOOP");

	asm("PRU_TX_ONLY_MODE:");
	// TX Only ...So channel numbers are contiguous
	asm(" ADD      R10.w0, R10.w0, 0x10");
	asm(" ADD      R10.b2, R10.b2, 0x01");
	asm(" ADD      R13.w0, R13.w0, 0x2C");
	asm(" QBGE     SERCH_MAPPED_TX_CHN, R10.b2, (8 - 1)");
	asm(" JMP      CORE_LOOP");

	asm("MAPPED_TX_CHN_FOUND:");
	asm(" LBBO     &R4.w0, R8, R10.w0,  16");
	asm(" ADD      R8, R8, R10.w0");

	asm(" QBEQ     PRUx_MODE_TX_ONLY, R3.b1, 0x1");

	asm(" QBEQ     CORE_LOOP, R3.b1, 0x00");

	asm("PRUx_MODE_TX_ONLY:");
	asm(" ADD      r22, R13.w0, 0x20");
	asm(" LBBO     &R11, r22, 0, 12");

	// JMP TO TxServiceReqHndlLoop Chn_TxRxBytesDoneCtr is less than Data length
	asm(" LSR      r22, R5.w0, 0x8");
	asm(" AND      r22, r22, 0x0F");
	asm(" ADD      r22, r22, 0x01");
	asm(" QBLT     TxServiceReqHndlLoop, r22, R7.b0");

	asm(" QBBS     DECLARE_COMPLETE, R5.b2, 0");
	asm(" NOT      r22, r20");
	asm(" SBBO     &r22, R12, 00, 4");

	asm(" JMP      CORE_LOOP");

	asm("DECLARE_COMPLETE:");
	// Set the status in the context area
	asm(" SET      R5.b2, R5.b2, 1");
	asm(" CLR      R5.b2, R5.b2, 0");
	asm(" SBBO     &R5.b2, R8, 6,  1");

	// Generate the interrupt to the ARM/DSP about the completion
	asm(" LBBO     &R2.w0, r20, 0x080, 2");
	asm(" QBBC     CORE_LOOP, R2.w0, R10.b2");

	asm(" LBBO     &R2.w2, r20, 0x082, 2");
	asm(" SET      R2.w2, R2.w2, R10.b2");
	asm(" SBBO     &R2.w2, r20, 0x082, 2");
	asm(" JMP      PRU_TO_HOST_INTERRUPT");

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

	asm("CORE_LOOP:");
	asm(" QBEQ     CORE_LOOP, R3.b1, 0x00");

	asm("ARM_DSP_EVENT:");
	asm(" QBEQ     CORE_LOOP_PRU1, R3.b0, 1");

	asm("CORE_LOOP_PRU0:");
	// wait for the hostEventStatus to get set. Loop till then
	asm(" WBS      r31, 30");

	// Read the PRUINTC register to know if the event is from ARM/DSP. If yes, then branch
	asm(" LDI      r23.w0, 0x204 & 0xFFFF");
	asm(" LDI      r23.w2, 0x204 >> 16");
	asm(" LBCO     &r22, C0, r23, 4");
	asm(" QBBS     CHN_SEARCH, r22, 0");

	// Else it is McASP Event. So before proceeding, clear it
	asm(" LDI      r22.w0, 31 & 0xFFFF");
	asm(" LDI      r22.w2, 31 >> 16");
	asm(" SBCO     &r22, C0, 0x24, 4");

	asm(" JMP      MCASP_EVENT");

	asm("CORE_LOOP_PRU1:");
	// wait for the hostEventStatus to get set. Loop till then
	asm(" WBS      r31, 31");

	asm(" LBCO     &r22, C25, 0xc0, 4");
	asm(" QBBC     CHN_SEARCH, r22, 5");

	// Clear the event here and go to Transmit processing
	asm(" LDI      r22.w0, 50 & 0xFFFF");
	asm(" LDI      r22.w2, 50 >> 16");
	asm(" SBCO     &r22, C0, 0x24, 4");
	asm(" JMP      TxInterruptServiceRequestHndlr");

	asm("MCASP_EVENT:");
	// Check for RX interrrupt first
	// If TX only PRU Skip RSTAT Check
	asm(" QBEQ     MCASP_TX_EVNT, R3.b1, 0x1");

	// if the PRU is RX only mode, then check if the XSTAT is set. If so, raise event to PRU1 and proceed
	asm(" QBNE     RX_TX_PROCESS, R3.b1, 0x2");
	asm(" LBCO     &r22, C25, 0xc0, 4");
	asm(" QBBC     RX_TX_PROCESS, r22, 5");
	asm(" LDI      r22.w0, 50 & 0xFFFF");
	asm(" LDI      r22.w2, 50 >> 16");
	asm(" SBCO     &r22, C0, 0x20, 4");

	asm("RX_TX_PROCESS:");
	asm(" LBCO     &r22, C25, 0x80, 4");
	asm(" QBBS     RxInterruptServiceRequestHndlr, r22, 5");
	// Skip the check for XSTAT if we are not Rx/Tx PRU.
	// We don't want the PRU to spin in a tight loop around the McASP register to introduce a delay
	asm(" QBNE     CORE_LOOP, R3.b1, 0x3");

	asm("MCASP_TX_EVNT:");
	asm(" LBCO     &r22, C25, 0xc0, 4");
	asm(" QBBS     TxInterruptServiceRequestHndlr, r22, 5");
	// If PRU is both TX/RX, then go back to Core-loop. Else delay to avoid McASP Spins
	asm(" QBEQ     CORE_LOOP, R3.b1, 0x3");

//******************************************** CORE LOOP: Ends ***********************************************

//========================================================================================================================================

//******************************************** CHN_SEARCH: Starts ********************************************

//========================================================================================================================================
//	Retrieve the active channel number that has requested for serviced and
//	load the context base info for that channel
//========================================================================================================================================

	asm("CHN_SEARCH:");
	asm(" LDI      r24.w0, 0x00000001 & 0xFFFF");
	asm(" LDI      r24.w2, 0x00000001 >> 16");
	asm(" LSL      r24, r24, R3.b0");
	asm(" LDI      r22.w0, 0x284 & 0xFFFF");
	asm(" LDI      r22.w2, 0x284 >> 16");
	asm(" SBCO     &r24, C0, r22, 4");

	// Read Global control register
	asm(" LBBO     &R2.w0, r20, 0x080,  8");

	// Retrieve the channel number and load the context base
	asm(" LDI      R8, 0x0000");
	asm(" LDI      R10.w0, 0x0000");
	asm(" LDI      R10.b2, 0x00");
	asm(" XOR      R13.w0, R13.w0, R13.w0");
	asm(" XOR      R9, R9, R9");

	asm(" LDI      R13.w0, 0x90");
	asm(" LDI      R9, 0x90");

	asm("CHN_ACTIVE:");
	asm(" LBBO     &R4.w0, R8, R10.w0,  2");
	asm(" QBBS     CHN_SERACH_RTN, R4.w0, 0x2");
	asm(" ADD      R10.w0, R10.w0, 0x10");
	asm(" ADD      R10.b2, R10.b2, 0x01");
	asm(" ADD      R13.w0, R13.w0, 0x2C");
	asm(" ADD      R9, R9, 0x20");

	// None of the channel has service request, go back to MainLoop
	// check to be verified to boundary condition
	asm(" QBLT     MCASP_EVENT, R10.b2, (8 - 1)");
	asm(" JMP      CHN_ACTIVE");

	asm("CHN_SERACH_RTN:");
	asm(" LBBO     &R4.w0, R8, R10.w0,  16");
	asm(" ADD      R8, R8, R10.w0");

	asm(" AND      r22.w0, R4.w0, 0x3");
	asm(" QBEQ     TxServiceRequestHndlr, r22.w0, 0x1");
	asm(" QBEQ     RxServiceRequestHndlr, r22.w0, 0x2");
	asm(" JMP      CORE_LOOP");

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

	asm("PRU_TO_HOST_INTERRUPT:");
	asm(" LDI      r22, 0");

	asm(" QBEQ     EVTOUT_PRU0_EVENTS, R3.b0, 0");
	asm(" QBEQ     EVTOUT_PRU1_EVENTS, R3.b0, 1");

	asm("EVTOUT_PRU0_EVENTS:");
	//storing the counter value
	asm(" ADD      r22, r22, 34");
	asm(" JMP      EVTOUT_SYSEVT_INIT");

	asm("EVTOUT_PRU1_EVENTS:");
	asm(" ADD      r22, r22, 42");

	asm("EVTOUT_SYSEVT_INIT:");
	asm(" ADD      r22, r22, R10.b2");

	asm("EVTOUT_GEN:");
	// Clear SYS_EVTn
	asm(" SBCO     &r22, C0, 0x24, 4");

	// Enable SYS_EVTn system interrupt
	asm(" SBCO     &r22, C0, 0x28, 4");

	// Generate SYS_EVTn by event out mapping
	asm(" MOV      r31.w0, r22.w0");

	asm(" JMP      MCASP_EVENT");

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

	asm("RxServiceRequestHndlr:");
	// load the max RX TRIES before time out
	asm(" LBBO     &r1.w0, r20, 0x088, 2");

	// read interrupt status regsiter
	asm(" LBBO     &R2.w2, r20, 0x082, 2");

	asm(" CLR      R2.w2, R2.w2, R10.b2");

	// write interrupt status regsiter
	asm(" SBBO     &R2.w2, r20, 0x082, 2");

	//Clear Service Request
	asm(" CLR      R4.w0, R4.w0, 0x2");
	asm(" SBBO     &R4.w0, R8, 0,  2");

	// clear timeout flag
	asm(" CLR      R5.b2, R5.b2, 6");

	// Set the TXRX_READY_BIT
	asm(" SET      R5.b2, R5.b2, 0");

	// update the RX Status Register
	asm(" SBBO     &R5.b2, R8, 6,  1");

	// Set the SUART_CH_TXRXCHNSTATUS_BIT to indicate the channel being active
	asm(" SET      R5.b3, R5.b3, 7");
	asm(" SBBO     &R5.b3, R8, 7,  1");

	asm("RX_CONTEXT_INIT:");
	asm(" QBEQ     PRUxxx_MODE_RX_ONLY, R3.b1, 0x2");

	// Load RX Context Base Address corresponding to Active RX Channel
	asm(" JAL      r30.w0, LOAD_RX_CONTEXT_ADDRESS");

	asm("PRUxxx_MODE_RX_ONLY:");
	// Calculating the specific SRCTL and R_BUF register offset.
	asm(" AND      r22, R4.b1, 0x0F");
	asm(" LSL      r22, r22, 2");
	// Storing SRCTL register address in RX Context Area Region
	asm(" LDI      R12.w0, (0x01D00180) & 0xFFFF");
	asm(" LDI      R12.w2, (0x01D00180) >> 16");
	asm(" ADD      R12, R12, r22");

	//storing asp_rsrctl_reg in RX Context Address Region
	asm(" SBBO     &R12, R9, 4,  4");

	// Store RBuf Address in RX Context Region
	asm(" LDI      R11.w0, (0x01D00280) & 0xFFFF");
	asm(" LDI      R11.w2, (0x01D00280) >> 16");
	asm(" ADD      R11, R11, r22");

	// storing asp_rbuf_reg in RX context  adress region
	asm(" SBBO     &R11, R9, 0,  4");

	// Load the Context info specific to Current RX channel from memory to registers
	//	LBBO   	rx_context,  suart_ch_info.rx_context_addr, #00, SIZE (rx_context)

	// Clear the RX timeout counter
	asm(" XOR      R16.w0, R16.w0, R16.w0");
	asm(" SBBO     &R16.w0, R9, 20,  2");

	// Activate RX serializer
	asm(" LBBO     &r23, R12, 0, 4");
	asm(" AND      r23, r23, 0x3");
	// Check if Serializer is Already Active as Rx if ,yes skip activation
	asm(" QBEQ     CLR_RSTAT, r23, 0x2");
	//  Activate serializer as Receiver
	asm(" LDI      r23.w0, 0x000E & 0xFFFF");
	asm(" LDI      r23.w2, 0x000E >> 16");
	asm(" SBBO     &r23, R12, 0, 4");

	asm("CLR_RSTAT:");
	// Clear the RSTAT  (Overrun, etc)
	asm(" LDI      r22.w0, 0xFFFF & 0xFFFF");
	asm(" LDI      r22.w2, 0xFFFF >> 16");
	asm(" SBCO     &r22, C25, 0x80, 4");

	asm(" JMP      MCASP_EVENT");

//******************************************** RxServiceRequestHndlr: ENDS ***********************************

//========================================================================================================================================

//******************************************** RxInterruptServiceRequestHndlr: Starts ************************

//========================================================================================================================================
//	RxInterruptServiceRequestHndlr is called when there is MCASP RX event, scans the active RX serializer,
//	once the active serializer is found, scans for corresponding RX channel, if it also is found, it loads
//	the context info for RX channel and proceeds for reading the frame tramsmitted by sender.
//========================================================================================================================================

	asm("RxInterruptServiceRequestHndlr:");
	// Retrieve the channel number and load the RX context base info corressponding to serializer address in scratch_reg1 and serializer number  in scratch_reg4
	// Load the SUART CHANNEL BASE ADDRESS
	asm(" LDI      R8, 0x0000");

	asm(" QBEQ     PRUx_MODE_RX_ONLY, R3.b1, 0x2");

	// Since the Rx Channel are 1,3,5,7 Load the suart_ch_regs for Rx channel 1 as it is first channel
	asm(" ADD      R10.w0, r20.w0, 0x10");

	asm(" LDI      R10.b2, 0x01");

	// Load the RX channel 1 context address to Ch_info register's rx_context_addr field
	asm(" LDI      R9, 0x0C0");
	asm(" JMP      SERCH_ACTIVE_RX_CHN_RX");

	asm("PRUx_MODE_RX_ONLY:");
	// Since the Rx Channel are 1,3,5,7 Load the suart_ch_regs for Rx channel 1 as it is first channel
	asm(" LDI      R10.w0, 0x00");

	asm(" LDI      R10.b2, 0x00");

	// Load the RX channel 1 context address to Ch_info register's rx_context_addr field
	asm(" LDI      R9, 0x90");

	asm("SERCH_ACTIVE_RX_CHN_RX:");
	asm(" ADD      r22, R10.w0, 7");
	// Load the Channel Cntrl info from Memory to Register
	asm(" LBBO     &R5.b3, R8, r22,  1");
	asm(" QBBC     NEXT_RX_CHN, R5.b3, 7");

	asm(" LBBO     &r22, R9, 4, 4");
	asm(" LBBO     &r23, r22, 0, 4");
	asm(" QBBS     ACTIVE_RX_CHN_FOUND, r23, 5");

	asm("NEXT_RX_CHN:");
	asm(" QBEQ     PRUxx_MODE_RX_ONLY, R3.b1, 0x2");

	// offset of RX suart_ch_regs
	asm(" ADD      R10.w0, R10.w0, 0x20");
	// Increament to Next Rx Channel number
	asm(" ADD      R10.b2, R10.b2, 0x2");

	// Increament rx_context_addr by RX_CONTEXT_OFFSET i.e. to next RX channel context address
	asm(" ADD      R9, R9, 0x50");
	asm(" QBGE     SERCH_ACTIVE_RX_CHN_RX, R10.b2, (8 - 1)");
	asm(" JMP      CORE_LOOP");

	asm("PRUxx_MODE_RX_ONLY:");
	// offset of RX suart_ch_regs
	asm(" ADD      R10.w0, R10.w0, 0x10");
	// Increamnet to Next Rx ChanneL number
	asm(" ADD      R10.b2, R10.b2, 0x1");
	// Increamnet rx_context_addr by RX_CONTEXT_OFFSET i.e. to next RX channel context address
	asm(" ADD      R9, R9, 0x20");
	asm(" QBGE     SERCH_ACTIVE_RX_CHN_RX, R10.b2, (8 - 1)");
	asm(" JMP      CORE_LOOP");

	asm("ACTIVE_RX_CHN_FOUND:");
	// Load the suart_ch_regs from Memory to Register
	asm(" LBBO     &R4.w0, R8, R10.w0,  16");

	// Load the Context info specific to current RX Channel from memory to registers
	asm(" LBBO     &R11, R9, 0, 24");

	asm(" ADD      R8, R8, R10.w0");

	// Clear the RSTAT  (Overrun, etc) for Errors
	asm(" LBCO     &r22, C25, 0x80, 4");
	asm(" QBBC     RX_PROCESSING_INIT, r22, 8");

	asm(" LDI      r22.w0, 0xFFFF & 0xFFFF");
	asm(" LDI      r22.w2, 0xFFFF >> 16");
	asm(" SBCO     &r22, C25, 0x80, 4");

	//  Start receving DATA from MAC_ASP's R-Buf corresponding to channel
	asm("RX_PROCESSING_INIT:");
	asm(" XOR      r19, r19, r19");
	// Read the content of RBUF
	asm(" LBBO     &r22, R11, 0, 4");
	asm(" OR       r19, r19, r22");

	// If start condition is already received then go to reading next bit  otherwise look for start condition
	asm(" QBLT     READ_CURRENT, R7.b1, 0");

	// check Chn_TxRxRepeatDoneCtr, if it is not zero, jump to READ_CURRENT to prescale the start condition
	asm(" QBLT     READ_CURRENT, R7.w2, 0");

	// If sampling point i.e. sampling_bit_pos is equal to greater than 16 (INVALID_SAMPLING_POINT),
	// start bit transition edge is being detected, fall through to calculate sampling point,
	// otherwise, sampling point is already calculated JUMP to READ_CURRENT
	asm(" QBGE     READ_CURRENT, R16.b2, 16");

	// Extract timing information by detecting start transition (first left most zero)
	asm(" LMBD     r25, r22, 0");
	// branch if zero: start bit transition detected
	asm(" QBGT     START_BIT_TRANSITION, r25, 32");
	asm(" LDI      R16.b2, 0xff");
	asm(" SBBO     &R16.b2, R9, 22,  1");

	// RX time out logic
	asm(" QBBC     RxInterruptServiceRequestHndlr, R4.w2, 14");
	asm(" QBBC     RxInterruptServiceRequestHndlr, R5.b2, 0");
	asm(" QBEQ     RxInterruptServiceRequestHndlr, R7.b0, 0");

	// Read the request count to be received
	asm(" LSR      r22, R5.w0, 0x8");
	asm(" AND      r22, r22, 0x0F");
	// Since fifo size is 16
	asm(" ADD      r22, r22, 0x01");
	asm(" QBEQ     RxInterruptServiceRequestHndlr, R7.b0, r22");

	// check if time-out is enabled, if yes increament the timeout counter and check if count is equal to MAX_RX_TIMEOUT_TRIES
	// if yes raise the interrupt for time out.
	asm(" ADD      R16.w0, R16.w0, 1");
	asm(" SBBO     &R16.w0, R9, 20,  2");
	asm(" QBGE     RxInterruptServiceRequestHndlr, R16.w0, r1.w0");
	asm(" SET      R5.b2, R5.b2, 6");
	asm(" CLR      R4.w2, R4.w2, 14");
	asm(" SBBO     &R4.w2, R8, 2,  2");

	// Clear the RX timeout counter
	asm(" XOR      R16.w0, R16.w0, R16.w0");
	asm(" SBBO     &R16.w0, R9, 20,  2");
	asm(" JMP      RX_CHN_INTR");

	// Calculate the sampling bit position based on the start bit position
	// center = oversampling / 2
	// sampling bit position = start bit possition - center

	asm("START_BIT_TRANSITION:");
	// clear the rx time out counter
	asm(" XOR      R16.w0, R16.w0, R16.w0");
	asm(" SBBO     &R16.w0, R9, 20,  2");

	// determine the over-sampling rate
	asm(" LSR      r23, R4.w2, 10");
	asm(" AND      r23, r23, 0x3");

	// OVER_SAMPLE
	asm(" QBEQ     OVER_SAMPLE_SIZE8BIT, r23, 0x0");
	asm(" QBEQ     OVER_SAMPLE_SIZE8BIT, r23, 0x1");
	asm(" QBEQ     OVER_SAMPLE_SIZE16BIT, r23, 0x2");

	// Calaulate sampling bit position for 8 bit over sampling
	asm("OVER_SAMPLE_SIZE8BIT:");
	// start bit possition - center
	asm(" SUB      R16.b2, r25, 0x4");
	// sampling point
	asm(" AND      R16.b2, R16.b2, 7");
	asm(" SBBO     &R16.b2, R9, 22,  1");
	// if Start bit position is eqaul to/greater than centre, sample the start bit in current read, otherwise in next read
	asm(" QBLE     READ_CURRENT, r25, 0x4");
	asm(" JMP      RxInterruptServiceRequestHndlr");

	// Calaulate sampling bit position for 16 bit over sampling
	asm("OVER_SAMPLE_SIZE16BIT:");
	// start bit possition - center
	asm(" SUB      R16.b2, r25, 0x8");
	// samplimg point
	asm(" AND      R16.b2, R16.b2, 15");
	asm(" SBBO     &R16.b2, R9, 22,  1");
	// if Start bit position is eqaul to/greater than centre, sample the start bit in current read, otherwise in next read
	asm(" QBLE     READ_CURRENT, r25, 0x8");
	asm(" JMP      RxInterruptServiceRequestHndlr");

	asm("READ_CURRENT:");
	// scratch_8bit_reg2 holds the information if bit detected is zero if scratch_8bit_reg2= 0, or one if scratch_8bit_reg2 = 1
	asm(" XOR      r21.b1, r21.b1, r21.b1");
	// if bit at sampling point is zero jump to READ_ZERO
	asm(" QBBC     READ_ZERO, r22, R16.b2");
	// otherwise increament scratch_8bit_reg2 by one as bit detected is one
	asm(" ADD      r21.b1, r21.b1, 1");

	asm("READ_ZERO:");
	// We have read the data bit here...
	// If start bit is being received already, then skip the start condition processing.
	asm(" QBLT     RX_BIT_RECVD, R7.b1, 0");

	//(Chn_TxRxBitsDoneCtr == 0)            //No bit is being Recieved, check if it is start bit
	// if DataBit == 0, i.e. scratch_8bit_reg2 == 0, Jump to Start Condition, else error fall through
	asm(" QBEQ     START_CONDITION, r21.b1, 0");

	asm(" QBEQ     START_CONDITION, R7.w2, 0");

	// Broken start condition or false alarm, Reset repeat counter		//if DataBit == 1, instead of zero
	asm(" XOR      R7.w2, R7.w2, R7.w2");
	asm(" SBBO     &R7.w2, R8, 14,  2");
	asm(" JMP      RxInterruptServiceRequestHndlr");

	// else part for NO_REPEAT_DONE  DataBit == 0
	asm("START_CONDITION:");
	// Increament Repeat Done Counter by One, write back to memory
	asm(" ADD      R7.w2, R7.w2, 1");
	asm(" SBBO     &R7.w2, R8, 14,  2");

	// Read Pre Scaler
	asm(" LDI      r23, 0x03FF");
	asm(" AND      r23, r23, R4.w2");

	// if Repeat Done count is greater than or equal to prescaler, start bit is received, jump to START_BIT_RECIVED,
	asm(" QBGE     START_BIT_RECIVED, r23, R7.w2");
	asm(" JMP      RxInterruptServiceRequestHndlr");

	// Start bit is condition Detected properly
	asm("START_BIT_RECIVED:");
	// Increament Bit Count by One, and write it to memory
	asm(" ADD      R7.b1, R7.b1, 1");
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// Reset Repeat Counter, and write it to memory
	asm(" XOR      R7.w2, R7.w2, R7.w2");
	asm(" SBBO     &R7.w2, R8, 14,  2");
	asm(" JMP      RxInterruptServiceRequestHndlr");

	// Start Bit has been detected Already, Now the data bit is being received
	asm("RX_BIT_RECVD:");
	// Now scratch_reg1 holds the info whether the data bit in scratch_8bit_reg2, is zero or one
	asm(" XOR      r22, r22, r22");
	// if scratch_8bit_reg2 = 0, i.e data bit is Zero Jump to RX_DATA_BIT_ZERO
	// else Data bit is one fall through, data bit is ONE
	asm(" QBEQ     RX_DATA_BIT_ZERO, r21.b1, 0");
	// bit received is one, scratch_reg1 = 1
	asm(" OR       r22, r22, 0x1");

	asm("RX_DATA_BIT_ZERO:");
	// if (Chn_TxRxRepeatDoneCntr < 32), check if reapeat done counter is less than 32, if yes Jump to RX_REPEAT_DONE_CNTR_LT_32
	asm(" QBGE     RX_REPEAT_DONE_CNTR_LT_32, R7.w2, 0x20");

	// repeat done counter is Greater than 32, Read Chn_RxDataBitsHoldRegHigh reg, Copy the Received bit to Chn_RxDataBitsHoldRegHigh register
	// else part : (Chn_TxRxRepeatDoneCntr is Greater than or equal to 32 )
	asm("RX_REPEAT_DONE_CNTR_GT_32:");
	// Calculate the offset for bit in Chn_RxDataBitsHoldRegHigh regsiter
	asm(" RSB      r23, R7.w2, 0x20");
	// Shift Received bit by above calculated of set i.e Chn_TxRxRepeatDoneCntr - 20
	asm(" LSL      r22, r22, r23");
	asm(" LBBO     &R15, R9, 16,  4");
	asm(" OR       R15, r22, R15");
	asm(" SBBO     &R15, R9, 16,  4");
	asm(" JMP      RX_REPEAT_COUNT_LESS_THAN_PRESCALR");

	// repeat done counter is less than OR equal to 32, Read the Chn_RxDataBitsHoldRegLow, Copy the Received bit to Chn_RxDataBitsHoldRegLow register
	// write it back to memory
	// if for (Chn_TxRxRepeatDoneCntr < 32)
	asm("RX_REPEAT_DONE_CNTR_LT_32:");
	// Shift Received bit by Repeat Done Counter
	asm(" LSL      r22, r22, R7.w2");
	asm(" LBBO     &R14, R9, 12,  4");
	asm(" OR       R14, r22, R14");
	asm(" SBBO     &R14, R9, 12,  4");

	// Increament Chn_TxRxRepeatDoneCntr by one and Check if Repeat Done Counter is equal to Prescalar,
	// if yes jump to PROCESS_RX_DATA_BIT, otherewise again sample RBuf for same bit
	asm("RX_REPEAT_COUNT_LESS_THAN_PRESCALR:");
	asm(" ADD      R7.w2, R7.w2, 1");
	asm(" SBBO     &R7.w2, R8, 14,  2");

	// Read Pre Scaler
	asm(" LDI      r23, 0x03FF");
	asm(" AND      r23, r23, R4.w2");

	// check if number of bits sampled (Chn_TxRxRepeatDoneCtr) is equal to prescaler (scratch_reg2), if yes jump to PROCESS_RX_DATA_BIT
	asm(" QBGE     PROCESS_RX_DATA_BIT, r23, R7.w2");
	asm(" JMP      RxInterruptServiceRequestHndlr");

	// Scan Chn_RxDataBitsHoldRegLow, Chn_RxDataBitsHoldRegHigh, to check if BIT received is one or zero and write to Chn_RxDataHoldReg
	// (Chn_TxRxRepeatDoneCntr >= Chn_Config1.PreScaller) if part
	asm("PROCESS_RX_DATA_BIT:");
	// Get the Presaclar
	asm(" LDI      r24, 0x03FF");
	// scratch_reg3 hold prescalar
	asm(" AND      r24, r24, R4.w2");

	// Initialize the register to zero required for copying data bit received in rxdata_buf
	// keep count of number of ONE scanned
	asm(" XOR      r21.b0, r21.b0, r21.b0");
	// keep count of number of ZERO scanned
	asm(" XOR      r21.b1, r21.b1, r21.b1");

	// used to store count of number of bits scanned in Chn_RxDataBitsHoldRegLow, & Chn_RxDataBitsHoldRegHigh
	asm(" XOR      r23, r23, r23");

	// points to location taken as start point in Chn_RxDataBitsHoldRegLow for scannig bit received
	asm(" XOR      r22, r22, r22");

	// scratch_reg4 holds the data from Chn_RxDataBitsHoldRegLow
	asm(" LBBO     &r25, R9, 12,  4");
	// if Pre Scalar is less than or equal to 32, JMP to BIT_CHK_LOOP
	asm(" QBGE     BIT_CHK_LOOP, r24, 0x20");

	// pre scalar is greater 32, check if it is greater that 48 then set scratch_reg3 = 48, scan bit upto this count only.
	asm("PRE_SCALR_GT_32:");
	// start checking bit from bit position 0x10
	asm(" OR       r22, r22, 0x10");
	// if Pre Scalar is less than 48
	asm(" QBGT     BIT_CHK_LOOP, r24, 0x30");

	// pre scalar is greater 48,  set scratch_reg3 = 48, scan bit upto this count only.
	asm("PRE_SCALR_GT_48:");
	asm(" LDI      r24, 0x30");

	// Scan the Chn_RxDataBitsHoldRegLow, and Chn_RxDataBitsHoldRegHigh registers to know received bit is ZERO or ONE
	asm("BIT_CHK_LOOP:");
	// if bit is cleared, Jump to BIT_RECVD_ZERO
	asm(" QBBC     BIT_RECVD_ZERO, r25, r22");
	// else BIT prerscaled is one
	asm(" ADD      r21.b0, r21.b0, 1");
	// Increament scratch_reg1 by one so that it points to next bit to scanned
	asm(" ADD      r22, r22, 1");
	// Increament scratch_reg2 holding bits scanned count
	asm(" ADD      r23, r23, 1");
	// if Prescaler is greater than 32, and scratch_reg2 is equal to 32, load Chn_RxDataBitsHoldRegHigh in scratch_reg4
	asm(" QBLT     LOAD_RXDATABITS_HOLDREGHIGH, r23, 0x20");
	// scan untill all the bits are scanned
	asm(" QBGT     BIT_CHK_LOOP, r23, r24");
	asm(" JMP      COPY_BIT_RECVD");

	// Load the Chn_RxDataBitsHoldRegHigh to scratch_reg4
	asm("LOAD_RXDATABITS_HOLDREGHIGH:");
	asm(" LBBO     &r25, R9, 16,  4");
	// Reset the scratch_reg1, so that starts from bit 0 for Chn_RxDataBitsHoldRegHigh
	asm(" XOR      r22, r22, r22");
	// Reset the scratch_reg2, so that only jump to label LOAD_RXDATABITS_HOLDREGHIGH done one's only
	asm(" XOR      r23, r23, r23");
	// Decreament Total loop count by 32, since it has been already checked in Chn_RxDataBitsHoldRegLow
	asm(" SUB      r24, r24, 0x20");
	asm(" JMP      BIT_CHK_LOOP");

	// Current sacnned Bit in Chn_RxDataBitsHoldRegHigh or Chn_RxDataBitsHoldRegLow is zero
	asm("BIT_RECVD_ZERO:");
	// for Zero
	asm(" ADD      r21.b1, r21.b1, 1");
	asm(" ADD      r22, r22, 1");
	asm(" ADD      r23, r23, 1");
	asm(" QBGT     BIT_CHK_LOOP, r23, r24");

	// Copy the Received bit to Chn_RxDataHoldReg, scratch_reg1, now store the info if bit received is zero or one
	asm("COPY_BIT_RECVD:");

	// scratch_8bit_reg1= Bit is ONE, scratch_8bit_reg2 = Bit is Zero, if scratch_8bit_reg2 > scratch_8bit_reg1,
	// jump to WRITE_RCVD_BIT_TO_RX_DATAHOLDREG as data bit is ZERO
	asm(" XOR      r22, r22, r22");
	asm(" QBGE     WRITE_RCVD_BIT_TO_RX_DATAHOLDREG, r21.b0, r21.b1");
	//Bit Received is One,  write to Chn_RxDataHoldReg
	asm(" OR       r22, r22, 0x1");

	// Write the Received Data bit (in scratch_reg1) to Chn_RxDataHoldReg
	asm("WRITE_RCVD_BIT_TO_RX_DATAHOLDREG:");
	// Shift the bit received by Chn_TxRxBitsDoneCtr
	asm(" LSL      r22, r22, R7.b1");

	// Read the Chn_RxDataHoldReg from Memory
	asm(" LBBO     &R13.w2, R9, 10,  2");

	// Write the bit received to Chn_RxDataHoldReg
	asm(" OR       R13.w2, R13.w2, r22");

	// Write updated Chn_RxDataHoldReg to memory
	asm(" SBBO     &R13.w2, R9, 10,  2");

	// Increment the Data bit Counter
	asm(" ADD      R7.b1, R7.b1, 1");
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// Reset the Repeat Done Counter
	asm(" XOR      R7.w2, R7.w2, R7.w2");
	asm(" SBBO     &R7.w2, R8, 14,  2");

	// initialize Chn_RxDataBitsHoldRegLow
	asm(" XOR      R14, R14, R14");
	asm(" SBBO     &R14, R9, 12,  4");

	// initialize Chn_RxDataBitsHoldRegHigh
	asm(" XOR      R15, R15, R15");
	asm(" SBBO     &R15, R9, 16,  4");

	// Read Bit Per Charater
	asm(" AND      r23, R5.w0, 0xF");

	// check is (N-1) bit is being received for current data frame, if yes jump to CHK_RECVD_DATA_FRAME
	// if all N bits has been Received Jump to RESET_BITS_CNTR, otherwise receive remaining bits.
	// (Chn_TxRxBitsDoneCntr >= Chn_Config2.BitsPerChar)
	asm(" QBGE     RESET_BITS_CNTR, r23, R7.b1");
	asm(" SUB      r23, r23, 1");
	asm(" QBEQ     CHK_RECVD_DATA_FRAME, r23, R7.b1");
	asm(" JMP      RxInterruptServiceRequestHndlr");

	// if all bits received, verify the Received data frame
	asm("CHK_RECVD_DATA_FRAME:");
	// Zero the (16 - Chn_TxRxBitsDoneCntr) Most significant bits in the Chn_RxDataHoldReg.
	asm(" RSB      r23, r23, 16");
	// load the count to for number of zero to inserted
	asm(" ADD      r23, r23, 0x10");
	// Used to Insert  Zero in MSB
	asm(" NOT      r22, r20");

	asm("REGHOLD_MSB_ZERO:");
	// Prepare the MASK with  ZERO's in bits that do not corresponds to data bits
	asm(" LSR      r22, r22, r23");

	// Read the Data hold Reg
	asm(" LBBO     &R13.w2, R9, 10,  2");
	// Insert the ZERO's in bits that do  not  corresponds to Data Bits
	asm(" AND      R13.w2, R13.w2, r22");
	asm(" SBBO     &R13.w2, R9, 10,  2");

	// removing start bit
	asm(" LSR      R13.w2, R13.w2, 1");

	// load the arm memory lacation address where data is to be written
	asm(" LBBO     &R6, R8, 8,  4");

	// Read Bits Per Character
	asm(" AND      r25, R5.w0, 0xF");
	asm(" SUB      r25, r25, 2");

	// Load the Bytes Done counter
	asm(" MOV      r22, R7.b0");

	// check, if two byte offset is required (bits per character greater than 8)
	asm(" QBGE     WRITE_RX_CHAR_TO_MEM, r25, 0x8");

	// calculate the offset in memory where received character is to be written
	asm(" LSL      r22, r22, 1");

	asm("WRITE_RX_CHAR_TO_MEM:");
	// Write the actual data to ARM Memory
	asm(" SBBO     &R13.w2, R6, r22,  2");

	asm(" JMP      RxInterruptServiceRequestHndlr");

	asm("RESET_BITS_CNTR:");
	// Check for Framing Error Framing Error
	asm(" SUB      r24, R7.b1, 1");

	// Reset bits done counter
	asm(" XOR      R7.b1, R7.b1, R7.b1");
	asm(" SBBO     &R7.b1, R8, 13,  1");

	// Get the Prescalar
	asm(" LDI      r23, 0x03FF");
	//scratch_reg2 hold prescalar
	asm(" AND      r23, r23, R4.w2");

	// Extract timing information by detecting start transition (first left most zero)
	asm(" LMBD     r25, r19, 0");
	// branch if zero start bit transition detected
	asm(" QBEQ     INVALID_SAMPLING_PNT, r25, 32");

	// determine the over-sampling rate
	asm(" LSR      r22, R4.w2, 10");
	// OVER_SAMPLE
	asm(" AND      r22, r22, 0x3");
	// 16 bit over sampling
	asm(" QBEQ     NXT_FRAME_SAMPLING_16BIT_OVS, r22, 0x2");

	// Calaulate sampling bit position for 8 bit over sampling
	asm("NXT_FRAME_SAMPLING_8BIT_OVS:");
	// to correct  bit timing error used 4
	asm(" QBLT     INVALID_SAMPLING_PNT, r25, 4");

	asm("CAL_SAMPL_PNT8:");
	// start bit position - center
	asm(" SUB      R16.b2, r25, 0x4");
	// sampling point
	asm(" AND      R16.b2, R16.b2, 7");
	asm(" QBGT     UPDATE_SAMPLING_PNT, r25, 4");
	asm(" JMP      NXT_FRAME_SAMPLING_PNT");

	// Calaulate sampling bit position for 16 bit over sampling
	asm("NXT_FRAME_SAMPLING_16BIT_OVS:");
	// to correct  bit timing error used 4
	asm(" QBLT     INVALID_SAMPLING_PNT, r25, 8");

	asm("CAL_SAMPL_PNT16:");
	// start bit position - center
	asm(" SUB      R16.b2, r25, 0x8");
	// sampling point
	asm(" AND      R16.b2, R16.b2, 15");
	asm(" QBGT     UPDATE_SAMPLING_PNT, r25, 8");

	asm("NXT_FRAME_SAMPLING_PNT:");
	// Increament Repeat Done Counter by One, write back to memory
	asm(" ADD      R7.w2, R7.w2, 1");
	asm(" SBBO     &R7.w2, R8, 14,  2");

	// Read Pre Scaler
	asm(" LDI      r23, 0x03FF");
	asm(" AND      r23, r23, R4.w2");

	// if Repeat Done count is greater than or equal to prescaler, start bit is received, jump to START_BIT
	asm(" QBLT     UPDATE_SAMPLING_PNT, r23, R7.w2");

	// Start bit is condition Detected properly
	asm("START_BIT:");
	// Increament Bit Count by One, and write it to memory
	asm(" ADD      R7.b1, R7.b1, 1");
	asm(" SBBO     &R7.b1, R8, 13,  1");

	asm(" XOR      R7.w2, R7.w2, R7.w2");
	asm(" SBBO     &R7.w2, R8, 14,  2");
	asm(" JMP      UPDATE_SAMPLING_PNT");

	asm("INVALID_SAMPLING_PNT:");
	// Reset the Sampling Point
	asm(" LDI      R16.b2, 0xff");

	asm("UPDATE_SAMPLING_PNT:");
	asm(" SBBO     &R16.b2, R9, 22,  1");

	// read interrupt mask regsiter
	asm(" LBBO     &R2.w0, r20, 0x080, 2");

	//read interrupt status regsiter
	asm(" LBBO     &R2.w2, r20, 0x082, 2");

	// check for error in received data frame
	// Check for Break Condiotion Error
	asm(" QBGE     RX_DATA_ZERO, R13.w2, 0");

	// Framing Error: Check if the Bit at Chn_TxRxBitsDoneCtr Bit Position in the Chn_RxDataHoldReg is set
	asm(" QBBC     BIT_CLEARD, R13.w2, r24");

	// increament Chn_TxRxBytesDoneCtr by one
	asm(" ADD      R7.b0, R7.b0, 1");
	asm(" SBBO     &R7.b0, R8, 12,  1");

	// Reset the Data Hold Reg
	asm(" XOR      R13.w2, R13.w2, R13.w2");
	asm(" SBBO     &R13.w2, R9, 10,  2");

	// Read the request count to be received
	asm(" LSR      r22, R5.w0, 0x8");
	asm(" AND      r22, r22, 0x0F");
	asm(" ADD      r22, r22, 0x01");

	// Read the bytes done counter
	asm(" MOV      r23, R7.b0");

	// check if bytes done counter is less than or equal to data len,
	// if yes go to CHK_RX_CMPL_INT and check for raise RX complete intr
	asm(" QBGE     CHK_RX_CMPL_INT, R7.b0, r22");

	// if bytes done counter is greater than data len subtract data len from it and
	// check if differnce is data len, if yes raise RX complete intr
	asm(" SUB      r23, R7.b0, r22");

	asm("CHK_RX_CMPL_INT:");
	// check if all data frame received or not, if RX request if complete, else receive next data frame
	asm(" QBLT     RxInterruptServiceRequestHndlr, r22, r23");

	// All requested frame received raise interrupt to ARM/DSP, set SUART_RX_FIFO_INDX_BIT, clear SUART_TXRX_READY_BIT
	asm("RX_COMPLETE:");
	// RX Data is in lower half of Fifo, if bytes done counter is equal to data len
	asm(" CLR      R5.b2, R5.b2, 1");

	// Raise the RX interrupt if Chn_TxRxBytesDoneCtr is equal to data len otherwise reset Chn_TxRxBytesDoneCtr and raise Rx interrupt
	asm(" QBEQ     CHK_RX_OVERRUN, R7.b0, r22");

	// reset Chn_TxRxBytesDoneCtr if Chn_TxRxBytesDoneCtr is equal to twice the data len
	asm(" XOR      R7.b0, R7.b0, R7.b0");
	asm(" SBBO     &R7.b0, R8, 12,  1");

	// RX data is in upper half of Fifo,if bytes done counter is equal to twice the data len
	asm(" SET      R5.b2, R5.b2, 1");

	asm("CHK_RX_OVERRUN:");
	asm(" LDI      r23.w0, 0x284 & 0xFFFF");
	asm(" LDI      r23.w2, 0x284 >> 16");
	asm(" LBCO     &r22, C0, r23, 4");
	asm(" ADD      r23, R10.b2, 2");
	asm(" ADD      r23, R10.b2, 2");

	asm("OVER_RUN_ERR:");
	asm(" QBBC     CHK_RX_READY_BIT, r22, r23");
	asm(" QBBC     CHK_RX_READY_BIT, R4.w2, 15");
	asm(" SET      R5.b2, R5.b2, 3");
	asm(" SET      R5.b2, R5.b2, 2");

	asm("CHK_RX_READY_BIT:");
	// If the receive is not activated from the host, then don't dump the data
	asm(" QBBC     RxInterruptServiceRequestHndlr, R5.b2, 0");
	asm(" JMP      RX_CHN_INTR");

	// Framing Error Detected, interrupt are masked go to DEACTIVATE_SERIALIZER, other wise update status reg
	asm("BIT_CLEARD:");
	asm(" SET      R5.b2, R5.b2, 4");
	asm(" JMP      SET_RX_ERR_STAT");

	// Break Condiotion Error detected, interrupt are masked go to DEACTIVATE_SERIALIZER, other wise update status reg
	asm("RX_DATA_ZERO:");
	asm(" SET      R5.b2, R5.b2, 5");

	// Update the Global and Channel Error Status Registers
	asm("SET_RX_ERR_STAT:");
	asm(" SET      R2.w2, R2.w2, 9");
	asm(" SET      R5.b2, R5.b2, 2");

	// if global Error interrupt is clear do not raise interrupt
	asm(" QBBC     RxInterruptServiceRequestHndlr, R2.w0, 9");

	// if Framing error status bit for channel is clear look for Break Condiotion Error
	asm(" QBBC     BREAK_COND_ERR, R5.b2, 4");

	// Framming Error Occurred, if Framing Error mask is not set jum to RxInterruptServiceRequestHndlr
	asm("FRAMING_ERR:");
	asm(" QBBS     RX_CHN_INTR, R4.w2, 12");
	asm(" JMP      RxInterruptServiceRequestHndlr");

	// if Break Error Mask not set jump to RxInterruptServiceRequestHndlr
	asm("BREAK_COND_ERR:");
	asm(" QBBC     RxInterruptServiceRequestHndlr, R4.w2, 13");

	// Set the Global interrupt status register
	asm("RX_CHN_INTR:");
	asm(" SET      R2.w2, R2.w2, R10.b2");

	// write interrupt status regsiter
	asm(" SBBO     &R2.w2, r20, 0x082, 2");

	// Update tx rx status regsiter status
	asm(" SBBO     &R5.b2, R8, 6,  1");

	// if interrupt are masked for channel then go to RxInterruptServiceRequestHndlr, otherwise raise the interrupt
	asm(" QBBC     RxInterruptServiceRequestHndlr, R2.w0, R10.b2");

	// Raise the interrupt to ARM/DSP
	asm(" JMP      PRU_TO_HOST_INTERRUPT");

//******************************************** RxInterruptServiceRequestHndlr: ENDs **************************

//========================================================================================================================================

//******************************************** LOAD_RX_CONTEXT_ADDRESS: Start*********************************

	asm("LOAD_RX_CONTEXT_ADDRESS:");
	asm(" QBEQ     RX_CONTEXT_CH1_ADDR, R10.b2, 1");
	asm(" QBEQ     RX_CONTEXT_CH3_ADDR, R10.b2, 3");
	asm(" QBEQ     RX_CONTEXT_CH5_ADDR, R10.b2, 5");
	asm(" QBEQ     RX_CONTEXT_CH7_ADDR, R10.b2, 7");
	asm(" jmp      r30.w0");

	asm("RX_CONTEXT_CH1_ADDR:");
	asm(" LDI      R9, 0x0C0");
	asm(" jmp      r30.w0");

	asm("RX_CONTEXT_CH3_ADDR:");
	asm(" LDI      R9, 0x110");
	asm(" jmp      r30.w0");

	asm("RX_CONTEXT_CH5_ADDR:");
	asm(" LDI      R9, 0x160");
	asm(" jmp      r30.w0");

	asm("RX_CONTEXT_CH7_ADDR:");
	asm(" LDI      R9, 0x1B0");
	asm(" jmp      r30.w0");

//******************************************** LOAD_RX_CONTEXT_ADDRESS Ends ***********************************

//=====================================================================================================================================

//			************************************************************************************************
//									SOFT-UART RECEIVE ROUTINE : ENDS
//			************************************************************************************************

//=====================================================================================================================================

}
