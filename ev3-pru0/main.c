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
#include <am18xx/sys_timer.h>

#include "resource_table.h"

/* from Linux */

#define EV3_PRU_TACHO_RING_BUF_SIZE 1024 /* must be power of 2! */

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

#define REMOTE_DATA_ADDR 0x80002000
#define REMOTE_DATA (*(struct ev3_pru_tacho_remote_data *)(REMOTE_DATA_ADDR))

// system events to/from ARM

// PRU0
#define HOST_INT	(uint32_t)(1 << 30)	// host interrupt 0
#define EVENT_FROM_ARM	32
#define EVENT_TO_ARM	33

// PRU1
//#define HOST_INT	(uint32_t)(1 << 31)	// host interrupt 1
//#define EVENT_FROM_ARM	34
//#define EVENT_TO_ARM	35

volatile uint32_t register __R31;

// This is 512B (RPMSG_BUF_SIZE), so it won't fit in PRU RAM, so put it in shared RAM
#define payload ((void *)(REMOTE_DATA_ADDR + sizeof(struct ev3_pru_tacho_remote_data)))

#define INTA GPIO.IN_DATA45_bit.GP5P11	// GPIO 5[11]
#define INTB GPIO.IN_DATA45_bit.GP5P8	// GPIO 5[8]
#define INTC GPIO.IN_DATA45_bit.GP5P13	// GPIO 5[13]
#define INTD GPIO.IN_DATA67_bit.GP6P9	// GPIO 6[9]

#define DIRA GPIO.IN_DATA01_bit.GP0P4	// GPIO 0[4]
#define DIRB GPIO.IN_DATA23_bit.GP2P9	// GPIO 2[9]
#define DIRC GPIO.IN_DATA23_bit.GP3P14	// GPIO 3[14]
#define DIRD GPIO.IN_DATA23_bit.GP2P8	// GPIO 2[8]

#define TACHO_STATE(x) ((INT##x << 1) | DIR##x)

static uint8_t tacho_state[NUM_EV3_PRU_TACHO];
static uint32_t tacho_prev_timestamp[NUM_EV3_PRU_TACHO];
static uint32_t tacho_counts[NUM_EV3_PRU_TACHO];

static void update_tacho_state(enum ev3_pru_tacho idx, uint8_t new_state)
{
	uint8_t current_state = tacho_state[idx] & 0x3;
	enum direction new_dir = UNKNOWN;
	uint32_t now, elapsed;

	switch (current_state) {
		case 0x0:
			if (new_state == 0x1) {
				new_dir = REVERSE;
			} else if (new_state == 0x2) {
				new_dir = FORWARD;
			}
			break;

		case 0x1:
			if (new_state == 0x3) {
				new_dir = REVERSE;
			} else if (new_state == 0x0) {
				new_dir = FORWARD;
			}
			break;

		case 0x3:
			if (new_state == 0x2) {
				new_dir = REVERSE;
			} else if (new_state == 0x1) {
				new_dir = FORWARD;
			}
			break;

		case 0x2:
			if (new_state == 0x0) {
				new_dir = REVERSE;
			} else if (new_state == 0x3) {
				new_dir = FORWARD;
			}
			break;
	}

	tacho_state[idx] = new_state;
	tacho_counts[idx] += new_dir;

	now = TIMER64P0.TIM34;
	elapsed = now - tacho_prev_timestamp[idx];

	// if there was a change in count or if count hasn't changed for 50ms
	if (new_dir || elapsed > (50 * 1000000 * 3 / 125)) {
		uint32_t new_head = (REMOTE_DATA.head[idx] + 1) & (EV3_PRU_TACHO_RING_BUF_SIZE - 1);

		tacho_prev_timestamp[idx] = now;

		REMOTE_DATA.position[idx][new_head] = tacho_counts[idx];
		REMOTE_DATA.timestamp[idx][new_head] = now;
		REMOTE_DATA.head[idx] = new_head;
	}
}

int main(void) {
	volatile uint8_t *status;
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;

	// Clear the status of the PRU-system event that the ARM will use to 'kick' us
	PRU_INTC.STATIDXCLR = EVENT_FROM_ARM;

	// Wait until Linux gives us the OK that the driver is loaded
	status = &resource_table.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	// TODO: should probably check return value here
	pru_rpmsg_init(&transport, &resource_table.vring0, &resource_table.vring1, EVENT_TO_ARM, EVENT_FROM_ARM);

	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, "ev3-tacho-rpmsg", 0) != PRU_RPMSG_SUCCESS);

	while (true) {
		// wait for the ARM to kick us
		while (!(__R31 & HOST_INT)) {
			update_tacho_state(EV3_PRU_TACHO_A, TACHO_STATE(A));
			update_tacho_state(EV3_PRU_TACHO_B, TACHO_STATE(B));
			update_tacho_state(EV3_PRU_TACHO_C, TACHO_STATE(C));
			update_tacho_state(EV3_PRU_TACHO_D, TACHO_STATE(D));
		}

		// clear the interrupt
		PRU_INTC.STATIDXCLR = EVENT_FROM_ARM;

		// Receive all available messages, multiple messages can be sent per kick
		while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
			struct ev3_pru_tacho_msg *msg = payload;

			switch (msg->type) {
			case EV3_PRU_TACHO_MSG_DATA_ADDR:
				msg->value = REMOTE_DATA_ADDR;
				pru_rpmsg_send(&transport, dst, src, msg, sizeof(*msg));
				break;
			}
		}
	}
}
