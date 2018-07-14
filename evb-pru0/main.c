/*
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 *  
 *  
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 * 	* Redistributions of source code must retain the above copyright 
 * 	  notice, this list of conditions and the following disclaimer.
 * 
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	  notice, this list of conditions and the following disclaimer in the 
 * 	  documentation and/or other materials provided with the   
 * 	  distribution.
 * 
 * 	* Neither the name of Texas Instruments Incorporated nor the names of
 * 	  its contributors may be used to endorse or promote products derived
 * 	  from this software without specific prior written permission.
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <rsc_types.h>

#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>

#include <sys_gpio.h>
#include <sys_timer_1ms.h>

#include "resource_table.h"

/* from Linux */

#define EV3_PRU_TACHO_RING_BUF_SIZE 256 /* must be power of 2! */

enum ev3_pru_tacho_msg_type {
	/* Host >< PRU: request memory map address of ring buffer data */
	EV3_PRU_TACHO_MSG_DATA_ADDR,
};

struct ev3_pru_tacho_msg {
	uint32_t type;
	uint32_t value;
};

enum ev3_pru_tacho {
	EV3_PRU_TACHO_A,
	EV3_PRU_TACHO_B,
	EV3_PRU_TACHO_C,
	EV3_PRU_TACHO_D,
	NUM_EV3_PRU_TACHO
};

struct ev3_pru_tacho_remote_data {
	uint32_t position[NUM_EV3_PRU_TACHO][EV3_PRU_TACHO_RING_BUF_SIZE];
	uint32_t timestamp[NUM_EV3_PRU_TACHO][EV3_PRU_TACHO_RING_BUF_SIZE];
	uint32_t head[NUM_EV3_PRU_TACHO];
};

/* end Linux */

enum direction {
	REVERSE = -1,
	UNKNOWN = 0,
	FORWARD = 1
};

/* Host-0 Interrupt sets bit 30 in register R31 */
#define HOST_INT			((uint32_t) 1 << 30)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			16
#define FROM_ARM_HOST			17

#define PRU_GLOBAL_BASE_ADDR		0x4a300000
#define PRU_SRAM __far __attribute__((cregister("PRU_SHAREDMEM", far)))

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

volatile register uint32_t __R31;

#if defined(PLAT_EVB)

#define INTA GPIO2.GPIO_DATAIN_bit.DATAIN4
#define INTB GPIO0.GPIO_DATAIN_bit.DATAIN26
#define INTC GPIO0.GPIO_DATAIN_bit.DATAIN27
#define INTD GPIO2.GPIO_DATAIN_bit.DATAIN11

#define DIRA GPIO2.GPIO_DATAIN_bit.DATAIN3
#define DIRB GPIO1.GPIO_DATAIN_bit.DATAIN12
#define DIRC GPIO1.GPIO_DATAIN_bit.DATAIN31
#define DIRD GPIO2.GPIO_DATAIN_bit.DATAIN9

#elif defined(PLAT_QUEST)

#define INTA GPIO2.GPIO_DATAIN_bit.DATAIN4
#define INTB GPIO0.GPIO_DATAIN_bit.DATAIN26
#define INTC GPIO0.GPIO_DATAIN_bit.DATAIN27
#define INTD GPIO2.GPIO_DATAIN_bit.DATAIN12

#define DIRA GPIO2.GPIO_DATAIN_bit.DATAIN3
#define DIRB GPIO1.GPIO_DATAIN_bit.DATAIN12
#define DIRC GPIO1.GPIO_DATAIN_bit.DATAIN31
#define DIRD GPIO2.GPIO_DATAIN_bit.DATAIN13

#else
#error Must define PLAT_xyz
#endif

#define TACHO_STATE(x) ((INT##x << 1) | DIR##x)

static uint8_t tacho_state[NUM_EV3_PRU_TACHO];
static uint32_t tacho_prev_timestamp[NUM_EV3_PRU_TACHO];
static uint32_t tacho_counts[NUM_EV3_PRU_TACHO];

static uint8_t payload[RPMSG_BUF_SIZE];
static PRU_SRAM struct ev3_pru_tacho_remote_data remote_data;

static void update_tacho_state(enum ev3_pru_tacho idx, uint8_t new_state)
{
	uint8_t current_state = tacho_state[idx] & 0x3;
	enum direction new_dir = UNKNOWN;
	uint32_t now, elapsed;

	switch (current_state) {
		case 0x0:
			if (new_state == 0x1) {
				new_dir = FORWARD;
			} else if (new_state == 0x2) {
				new_dir = REVERSE;
			}
			break;

		case 0x1:
			if (new_state == 0x3) {
				new_dir = FORWARD;
			} else if (new_state == 0x0) {
				new_dir = REVERSE;
			}
			break;

		case 0x3:
			if (new_state == 0x2) {
				new_dir = FORWARD;
			} else if (new_state == 0x1) {
				new_dir = REVERSE;
			}
			break;

		case 0x2:
			if (new_state == 0x0) {
				new_dir = FORWARD;
			} else if (new_state == 0x3) {
				new_dir = REVERSE;
			}
			break;
	}

	tacho_state[idx] = new_state;
	tacho_counts[idx] += new_dir;

	now = DMTIMER1_1MS.TCRR;
	elapsed = now - tacho_prev_timestamp[idx];

	// if there was a change in count or if count hasn't changed for 50ms
	if (new_dir || elapsed > (50 * 1000000 * 3 / 125)) {
		uint32_t new_head = (remote_data.head[idx] + 1) & (EV3_PRU_TACHO_RING_BUF_SIZE - 1);

		tacho_prev_timestamp[idx] = now;

		remote_data.position[idx][new_head] = tacho_counts[idx];
		remote_data.timestamp[idx][new_head] = now;
		remote_data.head[idx] = new_head;
	}
}

int main(void) {
	volatile uint8_t *status;
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;

	// Clear the status of the PRU-system event that the ARM will use to 'kick' us
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	// Wait until Linux gives us the OK that the driver is loaded
	status = &resource_table.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	pru_virtqueue_init(&transport.virtqueue0, &resource_table.rpmsg_vring0, TO_ARM_HOST, FROM_ARM_HOST);
	pru_virtqueue_init(&transport.virtqueue1, &resource_table.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, "ev3-tacho-rpmsg", "", 0) != PRU_RPMSG_SUCCESS);

	while (true) {
		// wait for the ARM to kick us
		while (!(__R31 & HOST_INT)) {
			update_tacho_state(EV3_PRU_TACHO_A, TACHO_STATE(A));
			update_tacho_state(EV3_PRU_TACHO_B, TACHO_STATE(B));
			update_tacho_state(EV3_PRU_TACHO_C, TACHO_STATE(C));
			update_tacho_state(EV3_PRU_TACHO_D, TACHO_STATE(D));
		}

		// clear the interrupt
		CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

		// Receive all available messages, multiple messages can be sent per kick
		while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
			struct ev3_pru_tacho_msg *msg = (void *)payload;

			switch (msg->type) {
			case EV3_PRU_TACHO_MSG_DATA_ADDR:
				msg->value = PRU_GLOBAL_BASE_ADDR + (uint32_t)&remote_data;
				pru_rpmsg_send(&transport, dst, src, msg, sizeof(*msg));
				break;
			}
		}
	}
}
