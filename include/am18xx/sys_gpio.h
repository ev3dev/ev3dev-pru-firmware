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

#ifndef _AM18XX_SYS_GPIO_H_
#define _AM18XX_SYS_GPIO_H_

#include <stdint.h>

typedef struct {
    unsigned GP0P0 : 1;
    unsigned GP0P1 : 1;
    unsigned GP0P2 : 1;
    unsigned GP0P3 : 1;
    unsigned GP0P4 : 1;
    unsigned GP0P5 : 1;
    unsigned GP0P6 : 1;
    unsigned GP0P7 : 1;
    unsigned GP0P8 : 1;
    unsigned GP0P9 : 1;
    unsigned GP0P10 : 1;
    unsigned GP0P11 : 1;
    unsigned GP0P12 : 1;
    unsigned GP0P13 : 1;
    unsigned GP0P14 : 1;
    unsigned GP0P15 : 1;
    unsigned GP1P0 : 1;
    unsigned GP1P1 : 1;
    unsigned GP1P2 : 1;
    unsigned GP1P3 : 1;
    unsigned GP1P4 : 1;
    unsigned GP1P5 : 1;
    unsigned GP1P6 : 1;
    unsigned GP1P7 : 1;
    unsigned GP1P8 : 1;
    unsigned GP1P9 : 1;
    unsigned GP1P10 : 1;
    unsigned GP1P11 : 1;
    unsigned GP1P12 : 1;
    unsigned GP1P13 : 1;
    unsigned GP1P14 : 1;
    unsigned GP1P15 : 1;
} gpio_bank_01;

typedef struct {
    unsigned GP2P0 : 1;
    unsigned GP2P1 : 1;
    unsigned GP2P2 : 1;
    unsigned GP2P3 : 1;
    unsigned GP2P4 : 1;
    unsigned GP2P5 : 1;
    unsigned GP2P6 : 1;
    unsigned GP2P7 : 1;
    unsigned GP2P8 : 1;
    unsigned GP2P9 : 1;
    unsigned GP2P10 : 1;
    unsigned GP2P11 : 1;
    unsigned GP2P12 : 1;
    unsigned GP2P13 : 1;
    unsigned GP2P14 : 1;
    unsigned GP2P15 : 1;
    unsigned GP3P0 : 1;
    unsigned GP3P1 : 1;
    unsigned GP3P2 : 1;
    unsigned GP3P3 : 1;
    unsigned GP3P4 : 1;
    unsigned GP3P5 : 1;
    unsigned GP3P6 : 1;
    unsigned GP3P7 : 1;
    unsigned GP3P8 : 1;
    unsigned GP3P9 : 1;
    unsigned GP3P10 : 1;
    unsigned GP3P11 : 1;
    unsigned GP3P12 : 1;
    unsigned GP3P13 : 1;
    unsigned GP3P14 : 1;
    unsigned GP3P15 : 1;
} gpio_bank_23;

typedef struct {
    unsigned GP4P0 : 1;
    unsigned GP4P1 : 1;
    unsigned GP4P2 : 1;
    unsigned GP4P3 : 1;
    unsigned GP4P4 : 1;
    unsigned GP4P5 : 1;
    unsigned GP4P6 : 1;
    unsigned GP4P7 : 1;
    unsigned GP4P8 : 1;
    unsigned GP4P9 : 1;
    unsigned GP4P10 : 1;
    unsigned GP4P11 : 1;
    unsigned GP4P12 : 1;
    unsigned GP4P13 : 1;
    unsigned GP4P14 : 1;
    unsigned GP4P15 : 1;
    unsigned GP5P0 : 1;
    unsigned GP5P1 : 1;
    unsigned GP5P2 : 1;
    unsigned GP5P3 : 1;
    unsigned GP5P4 : 1;
    unsigned GP5P5 : 1;
    unsigned GP5P6 : 1;
    unsigned GP5P7 : 1;
    unsigned GP5P8 : 1;
    unsigned GP5P9 : 1;
    unsigned GP5P10 : 1;
    unsigned GP5P11 : 1;
    unsigned GP5P12 : 1;
    unsigned GP5P13 : 1;
    unsigned GP5P14 : 1;
    unsigned GP5P15 : 1;
} gpio_bank_45;

typedef struct {
    unsigned GP6P0 : 1;
    unsigned GP6P1 : 1;
    unsigned GP6P2 : 1;
    unsigned GP6P3 : 1;
    unsigned GP6P4 : 1;
    unsigned GP6P5 : 1;
    unsigned GP6P6 : 1;
    unsigned GP6P7 : 1;
    unsigned GP6P8 : 1;
    unsigned GP6P9 : 1;
    unsigned GP6P10 : 1;
    unsigned GP6P11 : 1;
    unsigned GP6P12 : 1;
    unsigned GP6P13 : 1;
    unsigned GP6P14 : 1;
    unsigned GP6P15 : 1;
    unsigned GP7P0 : 1;
    unsigned GP7P1 : 1;
    unsigned GP7P2 : 1;
    unsigned GP7P3 : 1;
    unsigned GP7P4 : 1;
    unsigned GP7P5 : 1;
    unsigned GP7P6 : 1;
    unsigned GP7P7 : 1;
    unsigned GP7P8 : 1;
    unsigned GP7P9 : 1;
    unsigned GP7P10 : 1;
    unsigned GP7P11 : 1;
    unsigned GP7P12 : 1;
    unsigned GP7P13 : 1;
    unsigned GP7P14 : 1;
    unsigned GP7P15 : 1;
} gpio_bank_67;

typedef struct {
    unsigned GP8P0 : 1;
    unsigned GP8P1 : 1;
    unsigned GP8P2 : 1;
    unsigned GP8P3 : 1;
    unsigned GP8P4 : 1;
    unsigned GP8P5 : 1;
    unsigned GP8P6 : 1;
    unsigned GP8P7 : 1;
    unsigned GP8P8 : 1;
    unsigned GP8P9 : 1;
    unsigned GP8P10 : 1;
    unsigned GP8P11 : 1;
    unsigned GP8P12 : 1;
    unsigned GP8P13 : 1;
    unsigned GP8P14 : 1;
    unsigned GP8P15 : 1;
    unsigned RESERVED : 16;
} gpio_bank_8;

typedef struct {
    volatile uint32_t REV;
    volatile uint32_t RESERVED_0;
    union {
        volatile uint32_t BINTEN;
        struct {
            unsigned EN0 : 1;
            unsigned EN1 : 1;
            unsigned EN2 : 1;
            unsigned EN3 : 1;
            unsigned EN4 : 1;
            unsigned EN5 : 1;
            unsigned EN6 : 1;
            unsigned EN7 : 1;
            unsigned EN8 : 1;
            unsigned RESERVED : 23;
        } BINTEN_bit;
    };
    volatile uint32_t RESERVED_1;
    union {
        volatile uint32_t DIR01;
        volatile gpio_bank_01 DIR01_bit;
    };
    union {
        volatile uint32_t OUT_DATA01;
        volatile gpio_bank_01 OUT_DATA01_bit;
    };
    union {
        volatile uint32_t SET_DATA01;
        volatile gpio_bank_01 SET_DATA01_bit;
    };
    union {
        volatile uint32_t CLR_DATA01;
        volatile gpio_bank_01 CLR_DATA01_bit;
    };
    union {
        volatile uint32_t IN_DATA01;
        volatile gpio_bank_01 IN_DATA01_bit;
    };
    union {
        volatile uint32_t SET_RIS_TRIG01;
        volatile gpio_bank_01 SET_RIS_TRIG01_bit;
    };
    union {
        volatile uint32_t CLR_RIS_TRIG01;
        volatile gpio_bank_01 CLR_RIS_TRIG01_bit;
    };
    union {
        volatile uint32_t SET_FAL_TRIG01;
        volatile gpio_bank_01 SET_FAL_TRIG01_bit;
    };
    union {
        volatile uint32_t CLR_FAL_TRIG01;
        volatile gpio_bank_01 CLR_FAL_TRIG01_bit;
    };
    union {
        volatile uint32_t INTSTAT01;
        volatile gpio_bank_01 INTSTAT01_bit;
    };
    union {
        volatile uint32_t DIR23;
        volatile gpio_bank_23 DIR23_bit;
    };
    union {
        volatile uint32_t OUT_DATA23;
        volatile gpio_bank_23 OUT_DATA23_bit;
    };
    union {
        volatile uint32_t SET_DATA23;
        volatile gpio_bank_23 SET_DATA23_bit;
    };
    union {
        volatile uint32_t CLR_DATA23;
        volatile gpio_bank_23 CLR_DATA23_bit;
    };
    union {
        volatile uint32_t IN_DATA23;
        volatile gpio_bank_23 IN_DATA23_bit;
    };
    union {
        volatile uint32_t SET_RIS_TRIG23;
        volatile gpio_bank_23 SET_RIS_TRIG23_bit;
    };
    union {
        volatile uint32_t CLR_RIS_TRIG23;
        volatile gpio_bank_23 CLR_RIS_TRIG23_bit;
    };
    union {
        volatile uint32_t SET_FAL_TRIG23;
        volatile gpio_bank_23 SET_FAL_TRIG23_bit;
    };
    union {
        volatile uint32_t CLR_FAL_TRIG23;
        volatile gpio_bank_23 CLR_FAL_TRIG23_bit;
    };
    union {
        volatile uint32_t INTSTAT23;
        volatile gpio_bank_23 INTSTAT23_bit;
    };
    union {
        volatile uint32_t DIR45;
        volatile gpio_bank_45 DIR45_bit;
    };
    union {
        volatile uint32_t OUT_DATA45;
        volatile gpio_bank_45 OUT_DATA45_bit;
    };
    union {
        volatile uint32_t SET_DATA45;
        volatile gpio_bank_45 SET_DATA45_bit;
    };
    union {
        volatile uint32_t CLR_DATA45;
        volatile gpio_bank_45 CLR_DATA45_bit;
    };
    union {
        volatile uint32_t IN_DATA45;
        volatile gpio_bank_45 IN_DATA45_bit;
    };
    union {
        volatile uint32_t SET_RIS_TRIG45;
        volatile gpio_bank_45 SET_RIS_TRIG45_bit;
    };
    union {
        volatile uint32_t CLR_RIS_TRIG45;
        volatile gpio_bank_45 CLR_RIS_TRIG45_bit;
    };
    union {
        volatile uint32_t SET_FAL_TRIG45;
        volatile gpio_bank_45 SET_FAL_TRIG45_bit;
    };
    union {
        volatile uint32_t CLR_FAL_TRIG45;
        volatile gpio_bank_45 CLR_FAL_TRIG45_bit;
    };
    union {
        volatile uint32_t INTSTAT45;
        volatile gpio_bank_45 INTSTAT45_bit;
    };
    union {
        volatile uint32_t DIR67;
        volatile gpio_bank_67 DIR67_bit;
    };
    union {
        volatile uint32_t OUT_DATA67;
        volatile gpio_bank_67 OUT_DATA67_bit;
    };
    union {
        volatile uint32_t SET_DATA67;
        volatile gpio_bank_67 SET_DATA67_bit;
    };
    union {
        volatile uint32_t CLR_DATA67;
        volatile gpio_bank_67 CLR_DATA67_bit;
    };
    union {
        volatile uint32_t IN_DATA67;
        volatile gpio_bank_67 IN_DATA67_bit;
    };
    union {
        volatile uint32_t SET_RIS_TRIG67;
        volatile gpio_bank_67 SET_RIS_TRIG67_bit;
    };
    union {
        volatile uint32_t CLR_RIS_TRIG67;
        volatile gpio_bank_67 CLR_RIS_TRIG67_bit;
    };
    union {
        volatile uint32_t SET_FAL_TRIG67;
        volatile gpio_bank_67 SET_FAL_TRIG67_bit;
    };
    union {
        volatile uint32_t CLR_FAL_TRIG67;
        volatile gpio_bank_67 CLR_FAL_TRIG67_bit;
    };
    union {
        volatile uint32_t INTSTAT67;
        volatile gpio_bank_67 INTSTAT67_bit;
    };
    union {
        volatile uint32_t DIR8;
        volatile gpio_bank_8 DIR8_bit;
    };
    union {
        volatile uint32_t OUT_DATA8;
        volatile gpio_bank_8 OUT_DATA8_bit;
    };
    union {
        volatile uint32_t SET_DATA8;
        volatile gpio_bank_8 SET_DATA8_bit;
    };
    union {
        volatile uint32_t CLR_DATA8;
        volatile gpio_bank_8 CLR_DATA8_bit;
    };
    union {
        volatile uint32_t IN_DATA8;
        volatile gpio_bank_8 IN_DATA8_bit;
    };
    union {
        volatile uint32_t SET_RIS_TRIG8;
        volatile gpio_bank_8 SET_RIS_TRIG8_bit;
    };
    union {
        volatile uint32_t CLR_RIS_TRIG8;
        volatile gpio_bank_8 CLR_RIS_TRIG8_bit;
    };
    union {
        volatile uint32_t SET_FAL_TRIG8;
        volatile gpio_bank_8 SET_FAL_TRIG8_bit;
    };
    union {
        volatile uint32_t CLR_FAL_TRIG8;
        volatile gpio_bank_8 CLR_FAL_TRIG8_bit;
    };
    union {
        volatile uint32_t INTSTAT8;
        volatile gpio_bank_8 INTSTAT8_bit;
    };
} gpio;

#define GPIO (*(volatile gpio *)(0x01e26000))

#endif /* _AM18XX_SYS_GPIO_H_ */
