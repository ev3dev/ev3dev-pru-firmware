/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PRU_INTC_H_
#define _PRU_INTC_H_

#include <stdint.h>

typedef struct {
	union {
		volatile uint32_t REVID;

		volatile struct {
			unsigned REV : 32;
		} REVID_bit;
	};	// 0x0

	union {
		volatile uint32_t CONTROL;

		volatile struct {
			unsigned RESERVED_0 : 2;	// 1:0
			unsigned NESTMODE : 2;		// 3:2
			unsigned RESERVED_4 : 28;	// 31:4
		} CONTROL_bit;
	};	// 0x4

	uint32_t RESERVED_8[2];		// 0x8 - 0xC

	union {
		volatile uint32_t GLBLEN;

		volatile struct {
			unsigned ENABLE : 1;		// 0
			unsigned RESERVED_1 : 31;	// 31:1
		} GLBLEN_bit;
	};	// 0x10

	uint32_t RESERVED_14[2];	// 0x14 - 0x18

	union {
		volatile uint32_t GLBLNSTLVL;

		volatile struct {
			unsigned NESTLEVEL : 9;		// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned OVERRIDE : 1;		// 31
		} GLBLNSTLVL_bit;
	};	// 0x1C

	union {
		volatile uint32_t STATIDXSET;

		volatile struct {
			unsigned INDEX : 10;		// 9:0
			unsigned RESERVED_10 : 22;	// 31:10
		} STATIDXSET_bit;
	};	// 0x20

	union {
		volatile uint32_t STATIDXCLR;

		volatile struct {
			unsigned INDEX : 10;		// 9:0
			unsigned RESERVED_10 : 22;	// 31:10
		} STATIDXCLR_bit;
	};	// 0x24

	union {
		volatile uint32_t ENIDXSET;

		volatile struct {
			unsigned INDEX : 10;		// 9:0
			unsigned RESERVED_10 : 22;	// 31:10
		} ENIDXSET_bit;
	};	// 0x28

	union {
		volatile uint32_t ENIDXCLR;

		volatile struct {
			unsigned INDEX : 10;		// 9:0
			unsigned RESERVED_10 : 22;	// 31:10
		} ENIDXCLR_bit;
	};	// 0x2C

	uint32_t RESERVED_30;	// 0x30

	union {
		volatile uint32_t HSTINTENIDXSET;

		volatile struct {
			unsigned INDEX : 10;		// 9:0
			unsigned RESERVED_10 : 22;	// 31:10
		} HSTINTENIDXSET_bit;
	};	// 0x34

	union {
		volatile uint32_t HSTINTENIDXCLR;

		volatile struct {
			unsigned INDEX : 10;		// 9:0
			unsigned RESERVED_10 : 22;	// 31:10
		} HSTINTENIDXCLR_bit;
	};	// 0x38

	uint32_t RESERVED_3C[17];	// 0x3C - 0x7C

	union {
		volatile uint32_t GLBLPRIIDX;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} GLBLPRIIDX_bit;
	};	// 0x80

	uint32_t RESERVED_84[95];	// 0x84 - 0x1FC

	union {
		volatile uint32_t STATESETINT0;

		volatile struct {
			unsigned RAW_STATUS : 32;	// 31:0
		} STATESETINT0_bit;
	};	// 0x200

	union {
		volatile uint32_t STATESETINT1;

		volatile struct {
			unsigned RAW_STATUS : 32;	// 31:0
		} STATESETINT1_bit;
	};	// 0x204

	uint32_t RESERVED_208[30];	// 0x208 - 0x27C

	union {
		volatile uint32_t STATCLRINT0;

		volatile struct {
			unsigned ENABLE_STATUS : 32;	// 31:0
		} STATCLRINT0_bit;
	};	// 0x280

	union {
		volatile uint32_t STATCLRINT1;

		volatile struct {
			unsigned ENABLE_STATUS : 32;	// 31:0
		} STATCLRINT1_bit;
	};	// 0x284

	uint32_t RESERVED_288[30];	// 0x288 - 0x2FC

	union {
		volatile uint32_t ENABLESET0;

		volatile struct {
			unsigned ENABLE : 32;		// 31:0
		} ENABLESET0_bit;
	};	// 0x300

	union {
		volatile uint32_t ENABLESET1;

		volatile struct {
			unsigned ENABLE : 32;		// 31:0
		} ENABLESET1_bit;
	};	// 0x304

	uint32_t RESERVED_308[30];	// 0x308 - 0x37C

	union {
		volatile uint32_t ENABLECLR0;

		volatile struct {
			unsigned ENABLE : 32;		// 31:0
		} ENABLECLR0_bit;
	};	// 0x380

	union {
		volatile uint32_t ENABLECLR1;

		volatile struct {
			unsigned ENABLE : 32;		// 31:0
		} ENABLECLR1_bit;
	};	// 0x384

	uint32_t RESERVED_388[30];	// 0x388 - 0x3FC

	union {
		volatile uint32_t CHANMAP0;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP0_bit;
	};	// 0x400

	union {
		volatile uint32_t CHANMAP1;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP1_bit;
	};	// 0x404

	union {
		volatile uint32_t CHANMAP2;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP2_bit;
	};	// 0x408

	union {
		volatile uint32_t CHANMAP3;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP3_bit;
	};	// 0x40C

	union {
		volatile uint32_t CHANMAP4;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP4_bit;
	};	// 0x410

	union {
		volatile uint32_t CHANMAP5;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP5_bit;
	};	// 0x414

	union {
		volatile uint32_t CHANMAP6;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP6_bit;
	};	// 0x418

	union {
		volatile uint32_t CHANMAP7;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP7_bit;
	};	// 0x41C

	union {
		volatile uint32_t CHANMAP8;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP8_bit;
	};	// 0x420

	union {
		volatile uint32_t CHANMAP9;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP9_bit;
	};	// 0x424

	union {
		volatile uint32_t CHANMAP10;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP10_bit;
	};	// 0x428

	union {
		volatile uint32_t CHANMAP11;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP11_bit;
	};	// 0x42C

	union {
		volatile uint32_t CHANMAP12;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP12_bit;
	};	// 0x430

	union {
		volatile uint32_t CHANMAP13;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP13_bit;
	};	// 0x434

	union {
		volatile uint32_t CHANMAP14;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP14_bit;
	};	// 0x438

	union {
		volatile uint32_t CHANMAP15;

		volatile struct {
			unsigned SYSN_MAP : 8;		// 7:0
			unsigned SYSN1_MAP : 8;		// 15:8
			unsigned SYSN2_MAP : 8;		// 23:16
			unsigned SYSN3_MAP : 8;		// 31:24
		} CHANMAP15_bit;
	};	// 0x43C

	uint32_t RESERVED_440[240];	// 0x440 - 0x7FC

	union {
		volatile uint32_t HOSTMAP0;

		volatile struct {
			unsigned CHANN_MAP : 8;		// 7:0
			unsigned CHANN1_MAP : 8;	// 15:8
			unsigned CHANN2_MAP : 8;	// 23:16
			unsigned CHANN3_MAP : 8;	// 31:24
		} HOSTMAP0_bit;
	};	// 0x800

	union {
		volatile uint32_t HOSTMAP1;

		volatile struct {
			unsigned CHANN_MAP : 8;		// 7:0
			unsigned CHANN1_MAP : 8;	// 15:8
			unsigned CHANN2_MAP : 8;	// 23:16
			unsigned CHANN3_MAP : 8;	// 31:24
		} HOSTMAP1_bit;
	};	// 0x804

	union {
		volatile uint32_t HOSTMAP2;

		volatile struct {
			unsigned CHANN_MAP : 8;		// 7:0
			unsigned CHANN1_MAP : 8;	// 15:8
			unsigned CHANN2_MAP : 8;	// 23:16
			unsigned CHANN3_MAP : 8;	// 31:24
		} HOSTMAP2_bit;
	};	// 0x808

	uint32_t RESERVED_80C[61];	// 0x80C - 0x8FC

	union {
		volatile uint32_t HOSTINTPRIIDX0;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX0_bit;
	};	// 0x900

	union {
		volatile uint32_t HOSTINTPRIIDX1;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX1_bit;
	};	// 0x904

	union {
		volatile uint32_t HOSTINTPRIIDX2;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX2_bit;
	};	// 0x908

	union {
		volatile uint32_t HOSTINTPRIIDX3;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX3_bit;
	};	// 0x90C

	union {
		volatile uint32_t HOSTINTPRIIDX4;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX4_bit;
	};	// 0x910

	union {
		volatile uint32_t HOSTINTPRIIDX5;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX5_bit;
	};	// 0x914

	union {
		volatile uint32_t HOSTINTPRIIDX6;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX6_bit;
	};	// 0x918

	union {
		volatile uint32_t HOSTINTPRIIDX7;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX7_bit;
	};	// 0x91C

	union {
		volatile uint32_t HOSTINTPRIIDX8;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX8_bit;
	};	// 0x920

	union {
		volatile uint32_t HOSTINTPRIIDX9;

		volatile struct {
			unsigned PRI_INDEX : 10;	// 9:0
			unsigned RESERVED_10 : 21;	// 30:10
			unsigned NONE : 1;		// 31
		} HOSTINTPRIIDX9_bit;
	};	// 0x924

	uint32_t RESERVED_928[246];	// 0x928 - 0xCFC

	union {
		volatile uint32_t POLARITY0;

		volatile struct {
			unsigned POLARITY : 32;		// 31:0
		} POLARITY0_bit;
	};	// 0xD00

	union {
		volatile uint32_t POLARITY1;

		volatile struct {
			unsigned POLARITY : 32;		// 31:0
		} POLARITY1_bit;
	};	// 0xD04

	uint32_t RESERVED_D08[30];	// 0xD08 - 0xD7C

	union {
		volatile uint32_t TYPE0;

		volatile struct {
			unsigned TYPE : 32;		// 31:0
		} TYPE0_bit;
	};	// 0xD80

	union {
		volatile uint32_t TYPE1;

		volatile struct {
			unsigned TYPE : 32;		// 31:0
		} TYPE1_bit;
	};	// 0xD84

	uint32_t RESERVED_D84[222];	// 0xD88 - 0x10FC

	union {
		volatile uint32_t HOSTINTNSTLVL0;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL0_bit;
	};	// 0x1100

	union {
		volatile uint32_t HOSTINTNSTLVL1;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL1_bit;
	};	// 0x1104

	union {
		volatile uint32_t HOSTINTNSTLVL2;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL2_bit;
	};	// 0x1108

	union {
		volatile uint32_t HOSTINTNSTLVL3;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL3_bit;
	};	// 0x110C

	union {
		volatile uint32_t HOSTINTNSTLVL4;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL4_bit;
	};	// 0x1110

	union {
		volatile uint32_t HOSTINTNSTLVL5;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL5_bit;
	};	// 0x1114

	union {
		volatile uint32_t HOSTINTNSTLVL6;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL6_bit;
	};	// 0x1118

	union {
		volatile uint32_t HOSTINTNSTLVL7;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL7_bit;
	};	// 0x111C

	union {
		volatile uint32_t HOSTINTNSTLVL8;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL8_bit;
	};	// 0x1120

	union {
		volatile uint32_t HOSTINTNSTLVL9;

		volatile struct {
			unsigned NESTING_LEVEL : 9;	// 8:0
			unsigned RESERVED_9 : 22;	// 30:9
			unsigned AUTO_OVERRIDE : 1;	// 31
		} HOSTINTNSTLVL9_bit;
	};	// 0x1124

	uint32_t RESERVED_1128[246];	// 0x1128 - 0x14FC

	union {
		volatile uint32_t HOSTINTEN;

		volatile struct {
			unsigned ENABLES : 10;	// 9:0
			unsigned RESERVED_9 : 22;		// 31:10
		} HOSTINTEN_bit;
	};	// 0x1500

} pru_intc;

volatile __far pru_intc PRU_INTC __attribute__((cregister("PRU_INTC", far), peripheral));

#endif /* _PRU_INTC_H_ */
