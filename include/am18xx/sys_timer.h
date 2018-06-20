/*
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

#ifndef _AM18XX_SYS_TIMER_H_
#define _AM18XX_SYS_TIMER_H_

#include <stdint.h>

typedef struct {
	union {
		volatile uint32_t REVID;
		volatile struct {
			unsigned REV : 32;
		} REVID_bit;
	}; // 0x0

	union {
		volatile uint32_t EMUMGT;
		volatile struct {
			unsigned FREE : 1;
			unsigned SOFT : 1;
			unsigned Reserved_2 : 30;
		} EMUMGT_bit;
	}; // 0x4

	union {
		volatile uint32_t GPINTGPEN;
		volatile struct {
			unsigned GPINT12ENI : 1;
			unsigned GPINT12ENO : 1;
			unsigned Reserved_2 : 2;
			unsigned GPINT12INVI : 1;
			unsigned GPINT12INVO : 1;
			unsigned Reserved_6 : 10;
			unsigned GPENI12 : 1;
			unsigned GPENO12 : 1;
			unsigned Reserved_18 : 14;
		} GPINTGPEN_bit;
	}; // 0x8

	union {
		volatile uint32_t GPDATGPDIR;
		volatile struct {
			unsigned GPDATI12 : 1;
			unsigned GPDATO12 : 1;
			unsigned Reserved_2 : 14;
			unsigned GPDIRI12 : 1;
			unsigned GPDIRO12 : 1;
			unsigned Reserved_18 : 14;
		} GPDATGPDIR_bit;
	}; // 0x0C

	union {
		volatile uint32_t TIM12;
		volatile struct {
			unsigned TIM12 : 32;
		} TIM12_bit;
	}; // 0x10

	union {
		volatile uint32_t TIM34;
		volatile struct {
			unsigned TIM34 : 32;
		} TIM34_bit;
	}; // 0x14

	union {
		volatile uint32_t PRD12;
		volatile struct {
			unsigned PRD12 : 32;
		} PRD12_bit;
	}; // 0x18

	union {
		volatile uint32_t PRD34;
		volatile struct {
			unsigned PRD34 : 32;
		} PRD34_bit;
	}; // 0x1C

	union {
		volatile uint32_t TCR;
		volatile struct {
			unsigned TSTAT12 : 1;
			unsigned INVOUTP12 : 1;
			unsigned INVINP12 : 1;
			unsigned CP12 : 1;
			unsigned PWID12 : 2;
			unsigned ENAMODE12 : 2;
			unsigned CLKSRC12 : 1;
			unsigned TIEN12 : 1;
			unsigned READRSTMODE12 : 1;
			unsigned CAPMODE12 : 1;
			unsigned CAPEVTMODE12 : 2;
			unsigned Reserved_14 : 8;
			unsigned ENAMODE34 : 2;
			unsigned Reserved_24 : 2;
			unsigned READRSTMODE34 : 1;
			unsigned Reserved_27 : 5;
		} TCR_bit;
	}; // 0x20

	union {
		volatile uint32_t TGCR;
		volatile struct {
			unsigned TIM12RS : 1;
			unsigned TIM34RS : 1;
			unsigned TIMMODE : 2;
			unsigned PLUSEN : 1;
			unsigned Reserved_5 : 3;
			unsigned PSC34 : 4;
			unsigned TDDR34 : 4;
			unsigned Reserved_16 : 16;
		} TGCR_bit;
	}; // 0x24

	union {
		volatile uint32_t WDTCR;
		volatile struct {
			unsigned Reserved_0 : 12;
			unsigned Reserved_12 : 2;
			unsigned WDEN : 1;
			unsigned WDFLAG : 1;
			unsigned WDKEY : 16;
		} WDTCR_bit;
	}; // 0x28

	uint32_t RESERVED_30[2]; // 0x2C - 0x30

	union {
		volatile uint32_t REL12;
		volatile struct {
			unsigned REL12 : 32;
		} REL12_bit;
	}; // 0x34

	union {
		volatile uint32_t REL34;
		volatile struct {
			unsigned REL34 : 32;
		} REL34_bit;
	}; // 0x38

	union {
		volatile uint32_t CAP12;
		volatile struct {
			unsigned CAP12 : 32;
		} CAP12_bit;
	}; // 0x3C

	union {
		volatile uint32_t CAP34;
		volatile struct {
			unsigned CAP34 : 32;
		} CAP34_bit;
	}; // 0x40

	union {
		volatile uint32_t INTCTLSTAT;
		volatile struct {
			unsigned PRDINTEN12 : 1;
			unsigned PRDINTSTAT12 : 1;
			unsigned EVTINTEN12 : 1;
			unsigned EVTINTSTAT12 : 1;
			unsigned Reserved_4 : 12;
			unsigned PRDINTEN34 : 1;
			unsigned PRDINTSTAT34 : 1;
			unsigned EVTINTEN34 : 1;
			unsigned EVTINTSTAT34 : 1;
			unsigned Reserved_20 : 12;
		} INTCTLSTAT_bit;
	}; // 0x44

	uint32_t RESERVED_48[6]; // 0x48 - 0x5C

	union {
		volatile uint32_t CMP0;
		volatile struct {
			unsigned CMP0 : 32;
		} CMP0_bit;
	}; // 0x60

	union {
		volatile uint32_t CMP1;
		volatile struct {
			unsigned CMP1 : 32;
		} CMP1_bit;
	}; // 0x64

	union {
		volatile uint32_t CMP2;
		volatile struct {
			unsigned CMP2 : 32;
		} CMP2_bit;
	}; // 0x68

	union {
		volatile uint32_t CMP3;
		volatile struct {
			unsigned CMP3 : 32;
		} CMP3_bit;
	}; // 0x6C

	union {
		volatile uint32_t CMP4;
		volatile struct {
			unsigned CMP4 : 32;
		} CMP4_bit;
	}; // 0x70

	union {
		volatile uint32_t CMP5;
		volatile struct {
			unsigned CMP5 : 32;
		} CMP5_bit;
	}; // 0x74

	union {
		volatile uint32_t CMP6;
		volatile struct {
			unsigned CMP6 : 32;
		} CMP6_bit;
	}; // 0x78

	union {
		volatile uint32_t CMP7;
		volatile struct {
			unsigned CMP7 : 32;
		} CMP7_bit;
	}; // 0x7C
} timer;

volatile __far timer TIMER64P0 __attribute__((cregister("TIMER64P0", near), peripheral));
#define TIMER64P1 (*(volatile timer *)(0x01c21000))
#define TIMER64P2 (*(volatile timer *)(0x01f0c000))
#define TIMER64P3 (*(volatile timer *)(0x01f0d000))

#endif /* _AM18XX_SYS_TIMER_H_ */
