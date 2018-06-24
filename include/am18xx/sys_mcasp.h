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

#ifndef _AM18XX_SYS_MCASP_H_
#define _AM18XX_SYS_MCASP_H_

#include <stdint.h>

typedef struct {
    volatile uint32_t REV; // 0x0
    volatile uint32_t RESERVED_4[3]; // 0x4 - 0xc
    union {
        volatile uint32_t PFUNC;
        volatile struct {
            unsigned AXR0 : 1;
            unsigned AXR1 : 1;
            unsigned AXR2 : 1;
            unsigned AXR3 : 1;
            unsigned AXR4 : 1;
            unsigned AXR5 : 1;
            unsigned AXR6 : 1;
            unsigned AXR7 : 1;
            unsigned AXR8 : 1;
            unsigned AXR9 : 1;
            unsigned AXR10 : 1;
            unsigned AXR11 : 1;
            unsigned AXR12 : 1;
            unsigned AXR13 : 1;
            unsigned AXR14 : 1;
            unsigned AXR15 : 1;
            unsigned RESERVED_16 : 9;
            unsigned AMUTE : 1;
            unsigned ACLKX : 1;
            unsigned AHCLKX : 1;
            unsigned AFSX : 1;
            unsigned ACLKR : 1;
            unsigned AHCLKR : 1;
            unsigned AFSR : 1;
        } PFUNC_bit;
    }; // 0x10
    union {
        volatile uint32_t PDIR;
        struct {
            unsigned AXR0 : 1;
            unsigned AXR1 : 1;
            unsigned AXR2 : 1;
            unsigned AXR3 : 1;
            unsigned AXR4 : 1;
            unsigned AXR5 : 1;
            unsigned AXR6 : 1;
            unsigned AXR7 : 1;
            unsigned AXR8 : 1;
            unsigned AXR9 : 1;
            unsigned AXR10 : 1;
            unsigned AXR11 : 1;
            unsigned AXR12 : 1;
            unsigned AXR13 : 1;
            unsigned AXR14 : 1;
            unsigned AXR15 : 1;
            unsigned RESERVED_16 : 9;
            unsigned AMUTE : 1;
            unsigned ACLKX : 1;
            unsigned AHCLKX : 1;
            unsigned AFSX : 1;
            unsigned ACLKR : 1;
            unsigned AHCLKR : 1;
            unsigned AFSR : 1;
        } BPDIR_bit;
    }; // 0x14
    union {
        volatile uint32_t PDOUT;
        struct {
            unsigned AXR0 : 1;
            unsigned AXR1 : 1;
            unsigned AXR2 : 1;
            unsigned AXR3 : 1;
            unsigned AXR4 : 1;
            unsigned AXR5 : 1;
            unsigned AXR6 : 1;
            unsigned AXR7 : 1;
            unsigned AXR8 : 1;
            unsigned AXR9 : 1;
            unsigned AXR10 : 1;
            unsigned AXR11 : 1;
            unsigned AXR12 : 1;
            unsigned AXR13 : 1;
            unsigned AXR14 : 1;
            unsigned AXR15 : 1;
            unsigned RESERVED_16 : 9;
            unsigned AMUTE : 1;
            unsigned ACLKX : 1;
            unsigned AHCLKX : 1;
            unsigned AFSX : 1;
            unsigned ACLKR : 1;
            unsigned AHCLKR : 1;
            unsigned AFSR : 1;
        } PDOUT_bit;
    }; // 0x18
    union {
        volatile uint32_t PDIN;
        struct {
            unsigned AXR0 : 1;
            unsigned AXR1 : 1;
            unsigned AXR2 : 1;
            unsigned AXR3 : 1;
            unsigned AXR4 : 1;
            unsigned AXR5 : 1;
            unsigned AXR6 : 1;
            unsigned AXR7 : 1;
            unsigned AXR8 : 1;
            unsigned AXR9 : 1;
            unsigned AXR10 : 1;
            unsigned AXR11 : 1;
            unsigned AXR12 : 1;
            unsigned AXR13 : 1;
            unsigned AXR14 : 1;
            unsigned AXR15 : 1;
            unsigned RESERVED_16 : 9;
            unsigned AMUTE : 1;
            unsigned ACLKX : 1;
            unsigned AHCLKX : 1;
            unsigned AFSX : 1;
            unsigned ACLKR : 1;
            unsigned AHCLKR : 1;
            unsigned AFSR : 1;
        } PDIN_bit;
        volatile uint32_t PDSET;
        struct {
            unsigned AXR0 : 1;
            unsigned AXR1 : 1;
            unsigned AXR2 : 1;
            unsigned AXR3 : 1;
            unsigned AXR4 : 1;
            unsigned AXR5 : 1;
            unsigned AXR6 : 1;
            unsigned AXR7 : 1;
            unsigned AXR8 : 1;
            unsigned AXR9 : 1;
            unsigned AXR10 : 1;
            unsigned AXR11 : 1;
            unsigned AXR12 : 1;
            unsigned AXR13 : 1;
            unsigned AXR14 : 1;
            unsigned AXR15 : 1;
            unsigned RESERVED_16 : 9;
            unsigned AMUTE : 1;
            unsigned ACLKX : 1;
            unsigned AHCLKX : 1;
            unsigned AFSX : 1;
            unsigned ACLKR : 1;
            unsigned AHCLKR : 1;
            unsigned AFSR : 1;
        } PDSET_bit;
    }; // 0x1c
    union {
        volatile uint32_t PDCLR;
        struct {
            unsigned AXR0 : 1;
            unsigned AXR1 : 1;
            unsigned AXR2 : 1;
            unsigned AXR3 : 1;
            unsigned AXR4 : 1;
            unsigned AXR5 : 1;
            unsigned AXR6 : 1;
            unsigned AXR7 : 1;
            unsigned AXR8 : 1;
            unsigned AXR9 : 1;
            unsigned AXR10 : 1;
            unsigned AXR11 : 1;
            unsigned AXR12 : 1;
            unsigned AXR13 : 1;
            unsigned AXR14 : 1;
            unsigned AXR15 : 1;
            unsigned RESERVED_16 : 9;
            unsigned AMUTE : 1;
            unsigned ACLKX : 1;
            unsigned AHCLKX : 1;
            unsigned AFSX : 1;
            unsigned ACLKR : 1;
            unsigned AHCLKR : 1;
            unsigned AFSR : 1;
        } PDCLR_bit;
    }; // 0x20
    volatile uint32_t RESERVED_24[8]; // 0x24 - 0x40
    union {
        volatile uint32_t GBLCTL;
        struct {
            unsigned RCLKRST : 1;
            unsigned RHCLKRST : 1;
            unsigned RSRCLR : 1;
            unsigned RSMRST : 1;
            unsigned RFRST : 1;
            unsigned RESERVED_5 : 3;
            unsigned XCLKRST : 1;
            unsigned XHCLKRST : 1;
            unsigned XSRCLR : 1;
            unsigned XSMRST : 1;
            unsigned XFRST : 1;
            unsigned RESERVED_13 : 19;
        } GBLCTL_bit;
    }; // 0x44
    union {
        volatile uint32_t AMUTE;
        struct {
            unsigned MUTEN : 2;
            unsigned INPOL : 1;
            unsigned INEN : 1;
            unsigned INSTAT : 1;
            unsigned ROVRN : 1;
            unsigned XUNDRN : 1;
            unsigned RSYNCERR : 1;
            unsigned XSYNCERR : 1;
            unsigned RCKFAIL : 1;
            unsigned XCKFAIL : 1;
            unsigned RDMAERR : 1;
            unsigned XDMAERR : 1;
            unsigned RESERVED_13 : 19;
        } AMUTE_bit;
    }; // 0x48
    union {
        volatile uint32_t DLBCTL;
        struct {
            unsigned DLBEN : 1;
            unsigned ORD : 1;
            unsigned MODE : 1;
            unsigned RESERVED_4 : 29;
        } DLBCTL_bit;
    }; // 0x4c
    union {
        volatile uint32_t DITCTL;
        struct {
            unsigned DITEN : 1;
            unsigned RESERVED_1 : 1;
            unsigned VA : 1;
            unsigned VB : 1;
            unsigned RESERVED_4 : 29;
        } DITCTL_bit;
    }; // 0x50
    volatile uint32_t RESERVED_54[3]; // 0x54 - 0x5c
    union {
        volatile uint32_t RGBLCTL;
        struct {
            unsigned RCLKRST : 1;
            unsigned RHCLKRST : 1;
            unsigned RSRCLR : 1;
            unsigned RSMRST : 1;
            unsigned RFRST : 1;
            unsigned RESERVED_5 : 3;
            unsigned XCLKRST : 1;
            unsigned XHCLKRST : 1;
            unsigned XSRCLR : 1;
            unsigned XSMRST : 1;
            unsigned XFRST : 1;
            unsigned RESERVED_13 : 19;
        } RGBLCTL_bit;
    }; // 0x60
    union {
        volatile uint32_t RMASK;
        struct {
            unsigned RMASK0 : 1;
            unsigned RMASK1 : 1;
            unsigned RMASK2 : 1;
            unsigned RMASK3 : 1;
            unsigned RMASK4 : 1;
            unsigned RMASK5 : 1;
            unsigned RMASK6 : 1;
            unsigned RMASK7 : 1;
            unsigned RMASK8 : 1;
            unsigned RMASK9 : 1;
            unsigned RMASK10 : 1;
            unsigned RMASK11 : 1;
            unsigned RMASK12 : 1;
            unsigned RMASK13 : 1;
            unsigned RMASK14 : 1;
            unsigned RMASK15 : 1;
            unsigned RMASK16 : 1;
            unsigned RMASK17 : 1;
            unsigned RMASK18 : 1;
            unsigned RMASK19 : 1;
            unsigned RMASK20 : 1;
            unsigned RMASK21 : 1;
            unsigned RMASK22 : 1;
            unsigned RMASK23 : 1;
            unsigned RMASK24 : 1;
            unsigned RMASK25 : 1;
            unsigned RMASK26 : 1;
            unsigned RMASK27 : 1;
            unsigned RMASK28 : 1;
            unsigned RMASK29 : 1;
            unsigned RMASK30 : 1;
            unsigned RMASK31 : 1;
        } RMASK_bit;
    }; // 0x64
    union {
        volatile uint32_t RFMT;
        struct {
            unsigned RROT : 3;
            unsigned RBUSEL : 1;
            unsigned RSSZ : 4;
            unsigned RPBIT : 5;
            unsigned RPAD : 2;
            unsigned RRVRS : 1;
            unsigned RDATDLY : 2;
            unsigned RESERVED_18 : 14;
        } RFMT_bit;
    }; // 0x68
    union {
        volatile uint32_t AFSRCTL;
        struct {
            unsigned FSRP : 1;
            unsigned FSRM : 1;
            unsigned RESERVED_2 : 2;
            unsigned FRWID : 1;
            unsigned RESERVED_5 : 2;
            unsigned RMOD : 9;
            unsigned RESERVED_16 : 16;
        } AFSRCTL_bit;
    }; // 0x6c
    union {
        volatile uint32_t ACLKRCTL;
        struct {
            unsigned CLKRDIV : 4;
            unsigned CLKRM : 1;
            unsigned RESERVED_6 : 1;
            unsigned CLKRP : 1;
            unsigned RESERVED_8 : 24;
        } ACLKRCTL_bit;
    }; // 0x70
    union {
        volatile uint32_t AHCLKRCTL;
        struct {
            unsigned HCLKRDIV : 12;
            unsigned RESERVED_12 : 2;
            unsigned HCLKRP : 1;
            unsigned HCLKRM : 1;
            unsigned RESERVED_16 : 16;
        } AHCLKRCTL_bit;
    }; // 0x74
    union {
        volatile uint32_t RTDM;
        struct {
            unsigned RTDMS0 : 1;
            unsigned RTDMS1 : 1;
            unsigned RTDMS2 : 1;
            unsigned RTDMS3 : 1;
            unsigned RTDMS4 : 1;
            unsigned RTDMS5 : 1;
            unsigned RTDMS6 : 1;
            unsigned RTDMS7 : 1;
            unsigned RTDMS8 : 1;
            unsigned RTDMS9 : 1;
            unsigned RTDMS10 : 1;
            unsigned RTDMS11 : 1;
            unsigned RTDMS12 : 1;
            unsigned RTDMS13 : 1;
            unsigned RTDMS14 : 1;
            unsigned RTDMS15 : 1;
            unsigned RTDMS16 : 1;
            unsigned RTDMS17 : 1;
            unsigned RTDMS18 : 1;
            unsigned RTDMS19 : 1;
            unsigned RTDMS20 : 1;
            unsigned RTDMS21 : 1;
            unsigned RTDMS22 : 1;
            unsigned RTDMS23 : 1;
            unsigned RTDMS24 : 1;
            unsigned RTDMS25 : 1;
            unsigned RTDMS26 : 1;
            unsigned RTDMS27 : 1;
            unsigned RTDMS28 : 1;
            unsigned RTDMS29 : 1;
            unsigned RTDMS30 : 1;
            unsigned RTDMS31 : 1;
        } RTDM_bit;
    }; //0x78
    union {
        volatile uint32_t RINTCTL;
        struct {
            unsigned ROVRN : 1;
            unsigned RSYNCERR : 1;
            unsigned RCKFAIL : 1;
            unsigned RDMAERR : 1;
            unsigned RLAST : 1;
            unsigned RDATA : 1;
            unsigned RESERVED_6 : 1;
            unsigned RSTAFRM : 1;
            unsigned RESERVED_8 : 24;
        } RINTCTL_bit;
    }; // 0x7c
    union {
        volatile uint32_t RSTAT;
        struct {
            unsigned ROVRN : 1;
            unsigned RSYNCERR : 1;
            unsigned RCKFAIL : 1;
            unsigned RTDMSLOT : 1;
            unsigned RLAST : 1;
            unsigned RDATA : 1;
            unsigned RSTAFRM : 1;
            unsigned RDMAERR : 1;
            unsigned RERR : 1;
            unsigned RESERVED_9 : 23;
        } RSTAT_bit;
    }; // 0x80
    union {
        volatile uint32_t RSLOT;
        struct {
            unsigned RSLOTCNT : 9;
            unsigned RESERVED_9 : 23;
        } RSLOT_bit;
    }; // 0x84
    union {
        volatile uint32_t RCLKCHK;
        struct {
            unsigned RPS : 4;
            unsigned RESERVED_4 : 4;
            unsigned RMIN : 8;
            unsigned RMAX : 8;
            unsigned RCNT : 8;
        } RCLKCHK_bit;
    }; // 0x88
    union {
        volatile uint32_t REVTCTL;
        struct {
            unsigned RDATDMA : 1;
            unsigned RESERVED_1 : 31;
        } REVTCTL_bit;
    }; // 0x8c
    volatile uint32_t RESERVED_90[4]; // 0x90 - 0x9c
    union {
        volatile uint32_t XGBLCTL;
        struct {
            unsigned RCLKRST : 1;
            unsigned RHCLKRST : 1;
            unsigned RSRCLR : 1;
            unsigned RSMRST : 1;
            unsigned RFRST : 1;
            unsigned RESERVED_5 : 3;
            unsigned XCLKRST : 1;
            unsigned XHCLKRST : 1;
            unsigned XSRCLR : 1;
            unsigned XSMRST : 1;
            unsigned XFRST : 1;
            unsigned RESERVED_13 : 19;
        } XGBLCTL_bit;
    }; // 0xa0
    union {
        volatile uint32_t XMASK;
        struct {
            unsigned XMASK0 : 1;
            unsigned XMASK1 : 1;
            unsigned XMASK2 : 1;
            unsigned XMASK3 : 1;
            unsigned XMASK4 : 1;
            unsigned XMASK5 : 1;
            unsigned XMASK6 : 1;
            unsigned XMASK7 : 1;
            unsigned XMASK8 : 1;
            unsigned XMASK9 : 1;
            unsigned XMASK10 : 1;
            unsigned XMASK11 : 1;
            unsigned XMASK12 : 1;
            unsigned XMASK13 : 1;
            unsigned XMASK14 : 1;
            unsigned XMASK15 : 1;
            unsigned XMASK16 : 1;
            unsigned XMASK17 : 1;
            unsigned XMASK18 : 1;
            unsigned XMASK19 : 1;
            unsigned XMASK20 : 1;
            unsigned XMASK21 : 1;
            unsigned XMASK22 : 1;
            unsigned XMASK23 : 1;
            unsigned XMASK24 : 1;
            unsigned XMASK25 : 1;
            unsigned XMASK26 : 1;
            unsigned XMASK27 : 1;
            unsigned XMASK28 : 1;
            unsigned XMASK29 : 1;
            unsigned XMASK30 : 1;
            unsigned XMASK31 : 1;
        } XMASK_bit;
    }; // 0xa4
    union {
        volatile uint32_t XFMT;
        struct {
            unsigned XROT : 3;
            unsigned XBUSEL : 1;
            unsigned XSSZ : 4;
            unsigned XPBIT : 5;
            unsigned XPAD : 2;
            unsigned XRVRS : 1;
            unsigned XDATDLY : 2;
            unsigned RESERVED_18 : 14;
        } XFMT_bit;
    }; // 0xa8
    union {
        volatile uint32_t AFSXCTL;
        struct {
            unsigned FSXP : 1;
            unsigned FSXM : 1;
            unsigned RESERVED_2 : 2;
            unsigned RESERVED_5 : 2;
            unsigned XMOD : 9;
            unsigned RESERVED_16 : 16;
        } AFSXCTL_bit;
    }; // 0xac
    union {
        volatile uint32_t ACLKXCTL;
        struct {
            unsigned CLKXDIV : 5;
            unsigned CLKXM : 1;
            unsigned ASYNC : 1;
            unsigned CLKXP : 1;
            unsigned RESERVED_8 : 24;
        } ACLKXCTL_bit;
    }; // 0xb0
    union {
        volatile uint32_t AHCLKXCTL;
        struct {
            unsigned HCLKXDIV : 12;
            unsigned RESERVED_12 : 2;
            unsigned HCLKXP : 1;
            unsigned HCLKXM : 1;
            unsigned RESERVED_16 : 16;
        } AHCLKXCTL_bit;
    }; // 0xb4
    union {
        volatile uint32_t XTDM;
        struct {
            unsigned XTDMS0 : 1;
            unsigned XTDMS1 : 1;
            unsigned XTDMS2 : 1;
            unsigned XTDMS3 : 1;
            unsigned XTDMS4 : 1;
            unsigned XTDMS5 : 1;
            unsigned XTDMS6 : 1;
            unsigned XTDMS7 : 1;
            unsigned XTDMS8 : 1;
            unsigned XTDMS9 : 1;
            unsigned XTDMS10 : 1;
            unsigned XTDMS11 : 1;
            unsigned XTDMS12 : 1;
            unsigned XTDMS13 : 1;
            unsigned XTDMS14 : 1;
            unsigned XTDMS15 : 1;
            unsigned XTDMS16 : 1;
            unsigned XTDMS17 : 1;
            unsigned XTDMS18 : 1;
            unsigned XTDMS19 : 1;
            unsigned XTDMS20 : 1;
            unsigned XTDMS21 : 1;
            unsigned XTDMS22 : 1;
            unsigned XTDMS23 : 1;
            unsigned XTDMS24 : 1;
            unsigned XTDMS25 : 1;
            unsigned XTDMS26 : 1;
            unsigned XTDMS27 : 1;
            unsigned XTDMS28 : 1;
            unsigned XTDMS29 : 1;
            unsigned XTDMS30 : 1;
            unsigned XTDMS31 : 1;
        } XTDM_bit;
    }; // 0xb8
    union {
        volatile uint32_t XINTCTL;
        struct {
            unsigned XUNDRN : 1;
            unsigned XSYNCERR : 1;
            unsigned XCKFAIL : 1;
            unsigned XDMAERR : 1;
            unsigned XLAST : 1;
            unsigned XDATA : 1;
            unsigned RESERVED_6 : 1;
            unsigned XSTAFRM : 1;
            unsigned RESERVED_8 : 24;
        } XINTCTL_bit;
    }; // 0xbc
    union {
        volatile uint32_t XSTAT;
        struct {
            unsigned XUNDRN : 1;
            unsigned XSYNCERR : 1;
            unsigned XCKFAIL : 1;
            unsigned XTDMSLOT : 1;
            unsigned XLAST : 1;
            unsigned XDATA : 1;
            unsigned XSTAFRM : 1;
            unsigned XDMAERR : 1;
            unsigned XERR : 1;
            unsigned RESERVED_9 : 23;
        } XSTAT_bit;
    }; // 0xc0
    union {
        volatile uint32_t XSLOT;
        struct {
            unsigned XSLOTCNT : 9;
            unsigned RESERVED_9 : 23;
        } XSLOT_bit;
    }; // 0xc4
    union {
        volatile uint32_t XCLKCHK;
        struct {
            unsigned XPS : 4;
            unsigned RESERVED_4 : 4;
            unsigned XMIN : 8;
            unsigned XMAX : 8;
            unsigned XCNT : 8;
        } XCLKCHK_bit;
    }; // 0xc8
    union {
        volatile uint32_t XEVTCTL;
        struct {
            unsigned XDATDMA : 1;
            unsigned RESERVED_1 : 31;
        } XEVTCTL_bit;
    }; // 0xcc
    volatile uint32_t RESERVED_D0[12]; // 0xd0 - 0xfc
    union {
        volatile uint32_t DITCSRA0;
    }; // 0x100
    union {
        volatile uint32_t DITCSRA1;
    }; // 0x104
    union {
        volatile uint32_t DITCSRA2;
    }; // 0x108
    union {
        volatile uint32_t DITCSRA3;
    }; // 0x10c
    union {
        volatile uint32_t DITCSRA4;
    }; // 0x110
    union {
        volatile uint32_t DITCSRA5;
    }; // 0x114
    union {
        volatile uint32_t DITCSRB0;
    }; // 0x118
    union {
        volatile uint32_t DITCSRB1;
    }; // 0x11c
    union {
        volatile uint32_t DITCSRB2;
    }; // 0x120
    union {
        volatile uint32_t DITCSRB3;
    }; // 0x124
    union {
        volatile uint32_t DITCSRB4;
    }; // 0x128
    union {
        volatile uint32_t DITCSRB5;
    }; // 0x12c
    union {
        volatile uint32_t DITUDRA0;
    }; // 0x130
    union {
        volatile uint32_t DITUDRA1;
    }; // 0x134
    union {
        volatile uint32_t DITUDRA2;
    }; // 0x138
    union {
        volatile uint32_t DITUDRA3;
    }; // 0x13c
    union {
        volatile uint32_t DITUDRA4;
    }; // 0x140
    union {
        volatile uint32_t DITUDRA5;
    }; // 0x144
    union {
        volatile uint32_t DITUDRB0;
    }; // 0x148
    union {
        volatile uint32_t DITUDRB1;
    }; // 0x14c
    union {
        volatile uint32_t DITUDRB2;
    }; // 0x150
    union {
        volatile uint32_t DITUDRB3;
    }; // 0x154
    union {
        volatile uint32_t DITUDRB4;
    }; // 0x158
    union {
        volatile uint32_t DITUDRB5;
    }; // 0x15c
    volatile uint32_t RESERVED_160[8]; // 0x160 - 0x17c
    union {
        volatile uint32_t SRCTL0;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL0_bit;
    }; // 0x180
    union {
        volatile uint32_t SRCTL1;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL1_bit;
    }; // 0x184
    union {
        volatile uint32_t SRCTL2;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL2_bit;
    }; // 0x188
    union {
        volatile uint32_t SRCTL3;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL3_bit;
    }; // 0x18c
    union {
        volatile uint32_t SRCTL4;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL4_bit;
    }; // 0x190
    union {
        volatile uint32_t SRCTL5;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL5_bit;
    }; // 0x194
    union {
        volatile uint32_t SRCTL6;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL6_bit;
    }; // 0x198
    union {
        volatile uint32_t SRCTL7;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL7_bit;
    }; // 0x19c
    union {
        volatile uint32_t SRCTL8;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL8_bit;
    }; // 0x1a0
    union {
        volatile uint32_t SRCTL9;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL9_bit;
    }; // 0x1a4
    union {
        volatile uint32_t SRCTL10;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL10_bit;
    }; // 0x1a8
    union {
        volatile uint32_t SRCTL11;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL11_bit;
    }; // 0x1ac
    union {
        volatile uint32_t SRCTL12;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL12_bit;
    }; // 0x1b0
    union {
        volatile uint32_t SRCTL13;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL13_bit;
    }; // 0x1b4
    union {
        volatile uint32_t SRCTL14;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL14_bit;
    }; // 0x1b8
    union {
        volatile uint32_t SRCTL15;
        volatile struct {
            unsigned SRMOD : 2;
            unsigned DISMOD : 2;
            unsigned XRDY : 1;
            unsigned RRDY : 1;
            unsigned RESERVED_6 : 26;
        } SRCTL15_bit;
    }; // 0x1bc
    volatile uint32_t RESERVED_1CO[16]; // 0x1c0 - 0x1fc
    union {
        volatile uint32_t XBUF0;
    }; // 0x200
    union {
        volatile uint32_t XBUF1;
    }; // 0x204
    union {
        volatile uint32_t XBUF2;
    }; // 0x208
    union {
        volatile uint32_t XBUF3;
    }; // 0x20c
    union {
        volatile uint32_t XBUF4;
    }; // 0x210
    union {
        volatile uint32_t XBUF5;
    }; // 0x214
    union {
        volatile uint32_t XBUF6;
    }; // 0x218
    union {
        volatile uint32_t XBUF7;
    }; // 0x21c
    union {
        volatile uint32_t XBUF8;
    }; // 0x220
    union {
        volatile uint32_t XBUF9;
    }; // 0x224
    union {
        volatile uint32_t XBUF10;
    }; // 0x228
    union {
        volatile uint32_t XBUF11;
    }; // 0x22c
    union {
        volatile uint32_t XBUF12;
    }; // 0x230
    union {
        volatile uint32_t XBUF13;
    }; // 0x234
    union {
        volatile uint32_t XBUF14;
    }; // 0x238
    union {
        volatile uint32_t XBUF15;
    }; // 0x23c
    uint32_t volatile RESERVED_240[16]; // 0x240 - 0x27c
    union {
        volatile uint32_t RBUF0;
    }; // 0x280
    union {
        volatile uint32_t RBUF1;
    }; // 0x284
    union {
        volatile uint32_t RBUF2;
    }; // 0x288
    union {
        volatile uint32_t RBUF3;
    }; // 0x28c
    union {
        volatile uint32_t RBUF4;
    }; // 0x290
    union {
        volatile uint32_t RBUF5;
    }; // 0x294
    union {
        volatile uint32_t RBUF6;
    }; // 0x298
    union {
        volatile uint32_t RBUF7;
    }; // 0x29c
    union {
        volatile uint32_t RBUF8;
    }; // 0x2a0
    union {
        volatile uint32_t RBUF9;
    }; // 0x2a4
    union {
        volatile uint32_t RBUF10;
    }; // 0x2a8
    union {
        volatile uint32_t RBUF11;
    }; // 0x2ac
    union {
        volatile uint32_t RBUF12;
    }; // 0x2b0
    union {
        volatile uint32_t RBUF13;
    }; // 0x2b4
    union {
        volatile uint32_t RBUF14;
    }; // 0x2b8
    union {
        volatile uint32_t RBUF15;
    }; // 0x2bc
} mcasp;

volatile __far mcasp MCASP0 __attribute__((cregister("MCASP0_CTL", far), peripheral));

#endif /* _AM18XX_SYS_MCASP_H_ */
