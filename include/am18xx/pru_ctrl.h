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

#ifndef _PRU_CTRL_H_
#define _PRU_CTRL_H_

#include <stdint.h>

typedef struct {
	union {
		volatile uint32_t CONTROL;

		volatile struct {
			unsigned SOFTRESET : 1;
			unsigned ENABLE : 1;
			unsigned SLEEPING : 1;
			unsigned COUNTENABLE : 1;
			unsigned RESERVED_4 : 4;
			unsigned SINGLESTEP : 1;
			unsigned RESERVED_9 : 6;
			unsigned RUNSTATE : 1;
			unsigned PCTRRSTVAL : 16;
		} CONTROL_bit;
	};	// 0x0

	union {
		volatile uint32_t STATUS;

		volatile struct {
			unsigned PCOUNTER : 16;
			unsigned RESERVED_16 : 16;
		} STATUS_bit;
	};	// 0x4

	union {
		volatile uint32_t WAKEUP;

		volatile struct {
			unsigned BITWISEENABLES : 32;
		} WAKEUP_bit;
	};	// 0x8

	union {
		volatile uint32_t CYCLECNT;

		volatile struct {
			unsigned CYCLECOUNT : 32;
		} CYCLECNT_bit;
	};	// 0xC

	union {
		volatile uint32_t STALLCNT;

		volatile  struct {
			unsigned STALLCOUNT : 32;
		} STALLCNT_bit;
	};	// 0x10

	uint32_t RESERVED_14[3];	// 0x14 - 0x1C

	union {
		volatile uint32_t CONTABBLKIDX0;

		volatile struct {
			unsigned C24 : 4;
			unsigned RESERVED_4 : 12;
			unsigned C25 : 4;
			unsigned RESERVED_20 : 12;
		} CONTABBLKIDX0_bit;
	};	// 0x20

	uint32_t RESERVED_24;	// 0x24

	union {
		volatile uint32_t CONTABPROPTR0;

		volatile struct {
			unsigned C28 : 16;
			unsigned C29 : 16;
		} CONTABPROPTR0_bit;
	};	// 0x28

	union {
		volatile uint32_t CONTABPROPTR1;

		volatile struct {
			unsigned C30 : 16;
			unsigned C31 : 16;
		} CONTABPROPTR1_bit;
	};	// 0x2C

} pruCtrl;

#define PRU0_CTRL (*(volatile pruCtrl*)(0x00007000))
#define PRU1_CTRL (*(volatile pruCtrl*)(0x00007800))

#endif /* _PRU_CTRL_H_ */
