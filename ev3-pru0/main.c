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

#define TRIGGER_PERIOD_MS 5

/* from Linux */

enum ev3_pru_tacho_msg_type {
	EV3_PRU_TACHO_MSG_START,
	EV3_PRU_TACHO_MSG_STOP,
	EV3_PRU_TACHO_MSG_REQ_ONE,
	EV3_PRU_TACHO_MSG_UPDATE,
};

enum ev3_pru_tacho_iio_channel {
	EV3_PRU_IIO_CH_TACHO_A,
	EV3_PRU_IIO_CH_TACHO_B,
	EV3_PRU_IIO_CH_TACHO_C,
	EV3_PRU_IIO_CH_TACHO_D,
	EV3_PRU_IIO_CH_TIMESTAMP_LOW,
	EV3_PRU_IIO_CH_TIMESTAMP_HIGH,
	NUM_EV3_PRU_IIO_CH
};

struct ev3_pru_tacho_msg {
	uint32_t type;
	uint32_t value[NUM_EV3_PRU_IIO_CH];
};

/* end Linux */

enum tacho {
	TACHO_A,
	TACHO_B,
	TACHO_C,
	TACHO_D,
	NUM_TACHO
};

enum direction {
	REVERSE = -1,
	UNKNOWN = 0,
	FORWARD = 1
};

// system events to/from ARM

// PRU0
#define HOST_INT	(uint32_t)(1 << 30)	// host interrupt 0
#define EVENT_FROM_ARM	32
#define EVENT_TO_ARM	33

// PRU1
//#define HOST_INT	(uint32_t)(1 << 31)	// host interrupt 1
//#define EVENT_FROM_ARM	34
//#define EVENT_TO_ARM	35

// clock is 24MHz, so 24000 ticks/msec
#define TRIGGER_PERIOD_TICKS (TRIGGER_PERIOD_MS * 24000)

volatile uint32_t register __R31;

// To be extra safe, this should be 512B (RPMSG_BUF_SIZE), but we dont' have
// that much RAM to spare. For now, only message type is struct ev3_pru_tacho_msg,
// so 64 bytes should be enough.
static uint8_t payload[64];

#define INTA GPIO.IN_DATA45_bit.GP5P11	// GPIO 5[11]
#define INTB GPIO.IN_DATA45_bit.GP5P8	// GPIO 5[8]
#define INTC GPIO.IN_DATA45_bit.GP5P13	// GPIO 5[13]
#define INTD GPIO.IN_DATA67_bit.GP6P9	// GPIO 6[9]

#define DIRA GPIO.IN_DATA01_bit.GP0P4	// GPIO 0[4]
#define DIRB GPIO.IN_DATA23_bit.GP2P9	// GPIO 2[9]
#define DIRC GPIO.IN_DATA23_bit.GP3P14	// GPIO 3[14]
#define DIRD GPIO.IN_DATA23_bit.GP2P8	// GPIO 2[8]

#define TACHO_STATE(x) ((INT##x << 1) | DIR##x)

static uint64_t timestamp;

static uint8_t tacho_state[NUM_TACHO];
static uint32_t tacho_counts[NUM_TACHO];

static void update_tacho_state(enum tacho idx, uint8_t new_state)
{
	uint8_t current_state = tacho_state[idx] & 0x3;
	enum direction new_dir = UNKNOWN;

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
}

static void fill_msg_value(struct ev3_pru_tacho_msg *msg)
{
	msg->value[EV3_PRU_IIO_CH_TACHO_A] = tacho_counts[TACHO_A];
	msg->value[EV3_PRU_IIO_CH_TACHO_B] = tacho_counts[TACHO_B];
	msg->value[EV3_PRU_IIO_CH_TACHO_C] = tacho_counts[TACHO_C];
	msg->value[EV3_PRU_IIO_CH_TACHO_D] = tacho_counts[TACHO_D];
	msg->value[EV3_PRU_IIO_CH_TIMESTAMP_LOW] = timestamp & 0xffffffff;
	msg->value[EV3_PRU_IIO_CH_TIMESTAMP_HIGH] = timestamp >> 32;
}

int main(void) {
	volatile uint8_t *status;
	struct pru_rpmsg_transport transport;
	uint32_t period_ticks, timestamp_ticks;
	uint16_t src, dst, len;
	uint16_t trigger_src = 0, trigger_dst = 0;
	bool started = false;

	// Clear the status of the PRU-system event that the ARM will use to 'kick' us
	PRU_INTC.STATIDXCLR_bit.INDEX = EVENT_FROM_ARM;

	// Wait until Linux gives us the OK that the driver is loaded
	status = &resource_table.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	// TODO: should probably check return value here
	pru_rpmsg_init(&transport, &resource_table.vring0, &resource_table.vring1, EVENT_TO_ARM, EVENT_FROM_ARM);

	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, "ev3-tacho-rpmsg", trigger_src) != PRU_RPMSG_SUCCESS);

	period_ticks = timestamp_ticks = TIMER64P0.TIM34;

	while (true) {
		// wait for the ARM to kick us
		while (!(__R31 & HOST_INT)) {
			uint32_t now = TIMER64P0.TIM34;

			// timestamp is basically converting timer from 32-bit to 64-bit
			// REVISIT: could use an unused timer for ticks that are divisible by a power of 2
			// then we could actually return nanoseconds so that users don't have to scale
			timestamp += now - timestamp_ticks;
			timestamp_ticks = now;

			update_tacho_state(TACHO_A, TACHO_STATE(A));
			update_tacho_state(TACHO_B, TACHO_STATE(B));
			update_tacho_state(TACHO_C, TACHO_STATE(C));
			update_tacho_state(TACHO_D, TACHO_STATE(D));

			// send periodic updates when trigger has been started
			if (now - period_ticks >= TRIGGER_PERIOD_TICKS) {
				period_ticks += TRIGGER_PERIOD_TICKS;

				if (started) {
					struct ev3_pru_tacho_msg msg;

					msg.type = EV3_PRU_TACHO_MSG_UPDATE;
					fill_msg_value(&msg);

					pru_rpmsg_send(&transport, trigger_src, trigger_dst, &msg, sizeof(msg));
				}
			}
		}

		// clear the interrupt
		PRU_INTC.STATIDXCLR_bit.INDEX = EVENT_FROM_ARM;

		// Receive all available messages, multiple messages can be sent per kick
		while (pru_rpmsg_receive(&transport, &src, &dst, (void *)payload, &len) == PRU_RPMSG_SUCCESS) {
			struct ev3_pru_tacho_msg *msg = (void *)payload;

			switch (msg->type) {
			case EV3_PRU_TACHO_MSG_REQ_ONE:
				fill_msg_value(msg);
				pru_rpmsg_send(&transport, dst, src, msg, sizeof(*msg));
				break;
			case EV3_PRU_TACHO_MSG_START:
				started = true;
				trigger_src = dst;
				trigger_dst = src;
				break;
			case EV3_PRU_TACHO_MSG_STOP:
				started = false;
				break;
			}
		}
	}
}
