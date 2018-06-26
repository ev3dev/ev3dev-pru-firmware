/****************************************************************************/
/*  AM335x_PRU.cmd                                                          */
/*  Copyright (c) 2015  Texas Instruments Incorporated                      */
/*                                                                          */
/*    Description: This file is a linker command file that can be used for  */
/*                 linking PRU programs built with the C compiler and       */
/*                 the resulting .out file on an AM335x device.             */
/****************************************************************************/

-cr	/* Link using C conventions */

/* Specify the System Memory Map */
MEMORY
{
      PAGE 0:
	/* 4kB PRU0/1 Instruction RAM */
	PRU_IMEM	: org = 0x00000000 len = 0x00001000

      PAGE 1:
	/* 512B PRU Data RAM 0_1 */
	PRU_DMEM_0_1	: org = 0x00000000 len = 0x00000200	CREGISTER=3
	/* 512B PRU Data RAM 1_0 */
	PRU_DMEM_1_0	: org = 0x00002000 len = 0x00000200	CREGISTER=4
	/* interrupt controller */
	PRU_INTC	: org = 0x00004000 len = 0x00003000	CREGISTER=0

      PAGE 2:
	USB1		: org = 0x01E25000 len = 0x000001E8	CREGISTER=14
	UHPI		: org = 0x01E10000 len = 0x000001E8	CREGISTER=15
	TIMER64P0	: org = 0x01C20000 len = 0x00000080	CREGISTER=1
	ECAP0		: org = 0x01F06000 len = 0x00000100	CREGISTER=21
	ECAP1		: org = 0x01F07000 len = 0x00000140	CREGISTER=22
	ECAP2		: org = 0x01F08000 len = 0x00000880	CREGISTER=23
	EPWM0		: org = 0x01F00000 len = 0x000002C4	CREGISTER=18
	EPWM1		: org = 0x01F02000 len = 0x000002C4	CREGISTER=19
	I2C0		: org = 0x01C22000 len = 0x000000D8	CREGISTER=2
	I2C1		: org = 0x01E28000 len = 0x000000D8	CREGISTER=17
	MCASP0_DMA	: org = 0x01D02000 len = 0x00000100	CREGISTER=8
	SPI0		: org = 0x01C41000 len = 0x000001A4	CREGISTER=6
	MMCSD		: org = 0x01C40000 len = 0x00000300	CREGISTER=5
	UART0		: org = 0x01C42000 len = 0x00000038	CREGISTER=7
	UART1		: org = 0x01D0C000 len = 0x00000088	CREGISTER=11
	UART2		: org = 0x01D0D000 len = 0x00000088	CREGISTER=12
	USB0		: org = 0x01E00000 len = 0x00000100	CREGISTER=13

	RSVD9		: org = 0x01D06000 len = 0x00000100	CREGISTER=9
	RSVD10		: org = 0x01D0A000 len = 0x00000100	CREGISTER=10
	RSVD16		: org = 0x01E12000 len = 0x00000100	CREGISTER=16
	RSVD20		: org = 0x01F04000 len = 0x00000100	CREGISTER=20
	RSVD26		: org = 0x01D04000 len = 0x00000100	CREGISTER=26
	RSVD27		: org = 0x01D08000 len = 0x00000100	CREGISTER=27
}

/* Specify the sections allocation into memory */
SECTIONS {
	/* Forces _c_int00 to the start of PRU IRAM. Not necessary when loading
	 * an ELF file, but useful when loading a binary
	 */
	.text:_c_int00*	> 0x0, PAGE 0

	.text		> PRU_IMEM, PAGE 0
	.stack		> PRU_DMEM_0_1, PAGE 1
	.bss		> PRU_DMEM_0_1, PAGE 1
	.cio		> PRU_DMEM_0_1, PAGE 1
	.data		> PRU_DMEM_0_1, PAGE 1
	.switch		> PRU_DMEM_0_1, PAGE 1
	.sysmem		> PRU_DMEM_0_1, PAGE 1
	.cinit		> PRU_DMEM_0_1, PAGE 1
	.rodata		> PRU_DMEM_0_1, PAGE 1
	.rofardata	> PRU_DMEM_0_1, PAGE 1
	.farbss		> PRU_DMEM_0_1, PAGE 1
	.fardata	> PRU_DMEM_0_1, PAGE 1

	.resource_table > PRU_DMEM_0_1, PAGE 1
}
