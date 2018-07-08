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


// system events to/from ARM

// PRU0
#define HOST_INT	(uint32_t)(1 << 30)	// host interrupt 0
#define EVENT_FROM_ARM	60
#define EVENT_TO_ARM	61

// PRU1
//#define HOST_INT	(uint32_t)(1 << 31)	// host interrupt 1
//#define EVENT_FROM_ARM	62
//#define EVENT_TO_ARM	63

volatile uint32_t register __R31;

// only have 512B for stack/data, so this won't fit
//uint8_t payload[RPMSG_BUF_SIZE];
// using 128K shared RAM for now
#define payload ((uint8_t *)(0x8001F200))

#define LED (GPIO.OUT_DATA67_bit.GP6P7)

int main(void) {
	volatile uint8_t *status;
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;

	// Clear the status of the PRU-system event that the ARM will use to 'kick' us
	PRU_INTC.STATIDXCLR_bit.INDEX = EVENT_FROM_ARM;

	// Wait until Linux gives us the OK that the driver is loaded
	status = &resource_table.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	pru_rpmsg_init(&transport, &resource_table.vring0, &resource_table.vring1, EVENT_TO_ARM, EVENT_FROM_ARM);

	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, "rpmsg-client-sample", 0) != PRU_RPMSG_SUCCESS);

	while (true) {
		// Check bit 30 of register R31 to see if the ARM has kicked us
		if (__R31 & HOST_INT) {
			// clear the interrupt
			PRU_INTC.STATIDXCLR_bit.INDEX = EVENT_FROM_ARM;

			// Receive all available messages, multiple messages can be sent per kick
			while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
				// Echo the message back
				pru_rpmsg_send(&transport, dst, src, payload, len);
			}
		}
	}
}
