/*
 * PRU backend for I2C bus driver for FatcatLab EVB
 *
 * Copyright (C) 2013-2016 David Lechner <david@lechnology.com>
 *
 * Based on davinci_iic.c from lms2012
 * That file does not contain a copyright, but comes from the LEGO Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Partially based on main.c from PRU_RPMsg_Echo_Interrupt1 example:
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include "resource_table_1.h"

volatile register uint32_t __R31;

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT			((uint32_t) 1 << 31)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			18
#define FROM_ARM_HOST			19

/* This name matches rpmsg_device_id in the kernel driver */
#define CHAN_NAME			"evb-pru-i2c"

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

/* linux errno */
#define ENXIO 6

/* gpio registers */
#define GPIO_REVISION           (0x000 / 4)
#define GPIO_SYSCONFIG          (0x010 / 4)
#define GPIO_EOI                (0x020 / 4)
#define GPIO_IRQSTATUS_RAW_0    (0x024 / 4)
#define GPIO_IRQSTATUS_RAW_1    (0x028 / 4)
#define GPIO_IRQSTATUS_0        (0x02C / 4)
#define GPIO_IRQSTATUS_1        (0x030 / 4)
#define GPIO_IRQSTATUS_SET_0    (0x034 / 4)
#define GPIO_IRQSTATUS_SET_1    (0x038 / 4)
#define GPIO_IRQSTATUS_CLR_0    (0x03C / 4)
#define GPIO_IRQSTATUS_CLR_1    (0x040 / 4)
#define GPIO_IRQWAKEN_0         (0x044 / 4)
#define GPIO_IRQWAKEN_1         (0x048 / 4)
#define GPIO_SYSSTATUS          (0x114 / 4)
#define GPIO_CTRL               (0x130 / 4)
#define GPIO_OE                 (0x134 / 4)
#define GPIO_DATAIN             (0x138 / 4)
#define GPIO_DATAOUT            (0x13C / 4)
#define GPIO_LEVELDETECT0       (0x140 / 4)
#define GPIO_LEVELDETECT1       (0x144 / 4)
#define GPIO_RISINGDETECT       (0x148 / 4)
#define GPIO_FALLINGDETECT      (0x14C / 4)
#define GPIO_DEBOUNCENABLE      (0x150 / 4)
#define GPIO_DEBOUNCINGTIME     (0x154 / 4)
#define GPIO_CLEARDATAOUT       (0x190 / 4)
#define GPIO_SETDATAOUT         (0x194 / 4)

#define GPIO_PIN(n)             (1 << (n))

/*
 * Everything between here and "END LINUX DATA STRUCTS" must exactly match the
 * Linux driver. These structs are used to pass info to and from the PRU.
 */

#define MAX_BUF_SIZE	128
#define MESSAGE_LIMIT	2

/* from linux/i2c.h */
struct i2c_msg {
	uint16_t addr;
	uint16_t flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_STOP		0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_NOSTART */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
	uint16_t len;
	uint8_t buf[MAX_BUF_SIZE];
};

struct i2c_message_data {
	int32_t xfer_result;
	uint32_t num_msgs;
	struct i2c_msg msg[MESSAGE_LIMIT];
};

/* END LINUX DATA STRUCTS */

enum transfer_states {
	TRANSFER_IDLE,
	TRANSFER_START,
	TRANSFER_START2,
	TRANSFER_ADDR,
	TRANSFER_WRITE,
	TRANSFER_READ,
	TRANSFER_WBIT,
	TRANSFER_RBIT,
	TRANSFER_WACK,
	TRANSFER_RACK,
	TRANSFER_STOP,
	TRANSFER_STOP2,
	TRANSFER_STOP3,
	TRANSFER_RESTART,
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
};

enum i2c_pin {
	I2C_PIN_SDA,
	I2C_PIN_SCL,
	NUM_I2C_PIN
};

enum port {
	PORT_1,
	PORT_2,
	PORT_3,
	PORT_4,
	NUM_PORTS
};

struct gpio_info {
	uint16_t bank;
	uint16_t index;
};

struct i2c_data {
	char *name;
	uint16_t reply_to;
	struct gpio_info gpio[NUM_I2C_PIN];
	struct i2c_msg msgs[2];
	unsigned num_msgs;
	unsigned cur_msg;
	unsigned wait_cycles;
	int32_t xfer_result;
	uint16_t buf_offset;
	uint8_t bit_mask;
	uint8_t data_byte;
	enum transfer_states transfer_state;
	uint8_t clock_state;
	bool nacked;
};

/* global variables */

static volatile uint32_t * const gpio_base[4] = {
	(volatile uint32_t *)(0x44e07000),
	(volatile uint32_t *)(0x4804c000),
	(volatile uint32_t *)(0x481ac000),
	(volatile uint32_t *)(0x481ae000),
};

/* TODO: would be better to get gpio pin info from device tree */
#if defined PLAT_EVB

static struct i2c_data i2c_ports[NUM_PORTS] = {
	[PORT_1] = {
		.name = "Input port 1",
		.gpio = {
			[I2C_PIN_SDA] = {
				.bank	= 1,
				.index	= 18,
			},
			[I2C_PIN_SCL] = {
				.bank	= 0,
				.index	= 31,
			},
		},
	},
	[PORT_2] = {
		.name = "Input port 2",
		.gpio = {
			[I2C_PIN_SDA] = {
				.bank	= 0,
				.index	= 5,
			},
			[I2C_PIN_SCL] = {
				.bank	= 0,
				.index	= 3,
			},
		},
	},
	[PORT_3] = {
		.name = "Input port 3",
		.gpio = {
			[I2C_PIN_SDA] = {
				.bank	= 0,
				.index	= 13,
			},
			[I2C_PIN_SCL] = {
				.bank	= 0,
				.index	= 15,
			},
		},
	},
	[PORT_4] = {
		.name = "Input port 4",
		.gpio = {
			[I2C_PIN_SDA] = {
				.bank	= 1,
				.index	= 17,
			},
			[I2C_PIN_SCL] = {
				.bank	= 2,
				.index	= 14,
			},
		},
	},
};

#elif defined PLAT_QUEST

static struct i2c_data i2c_ports[NUM_PORTS] = {
	[PORT_1] = {
		.name = "Input port 1",
		.gpio = {
			[I2C_PIN_SDA] = {
				.bank	= 1,
				.index	= 18,
			},
			[I2C_PIN_SCL] = {
				.bank	= 0,
				.index	= 31,
			},
		},
	},
	[PORT_2] = {
		.name = "Input port 2",
		.gpio = {
			[I2C_PIN_SDA] = {
				.bank	= 0,
				.index	= 4,
			},
			[I2C_PIN_SCL] = {
				.bank	= 0,
				.index	= 3,
			},
		},
	},
	[PORT_3] = {
		.name = "Input port 3",
		.gpio = {
			[I2C_PIN_SDA] = {
				.bank	= 0,
				.index	= 12,
			},
			[I2C_PIN_SCL] = {
				.bank	= 0,
				.index	= 15,
			},
		},
	},
	[PORT_4] = {
		.name = "Input port 4",
		.gpio = {
			[I2C_PIN_SDA] = {
				.bank	= 1,
				.index	= 17,
			},
			[I2C_PIN_SCL] = {
				.bank	= 2,
				.index	= 14,
			},
		},
	},
};

#else
#error Must define PLAT_EVB or PLAT_QUEST
#endif

uint8_t payload[RPMSG_BUF_SIZE];

/*
 * Sets a gpio to input.
 */
static inline void gpio_dir_in(struct gpio_info *gpio)
{
	gpio_base[gpio->bank][GPIO_OE] |= GPIO_PIN(gpio->index);
}

/*
 * Sets gpio to output with specified state.
 */
static inline void gpio_dir_out(struct gpio_info *gpio, uint8_t state)
{
	gpio_base[gpio->bank][GPIO_OE] &= ~GPIO_PIN(gpio->index);

	if (state) {
		gpio_base[gpio->bank][GPIO_SETDATAOUT] = GPIO_PIN(gpio->index);
	}
	else {
		gpio_base[gpio->bank][GPIO_CLEARDATAOUT] = GPIO_PIN(gpio->index);
	}
}

/*
 * Reads the current value of a gpio.
 */
static inline uint8_t gpio_get_value(struct gpio_info *gpio)
{
	return !!(gpio_base[gpio->bank][GPIO_DATAIN] & GPIO_PIN(gpio->index));
}

/*
 * State machine for handling I2C messages.
 *
 * Returns true if the message is complete and we should notify the Linux driver
 * or false if busy or idle. It is important for the caller to set transfer_state
 * to TRANSER_IDLE after this function returns true, otherwise the state will
 * be stuck at TRANSER_COMPLETE.
 */
static bool update_i2c_port(struct i2c_data *data)
{
	struct i2c_msg *msg = &data->msgs[data->cur_msg];

	gpio_dir_out(&data->gpio[I2C_PIN_SCL], data->clock_state);

	switch (data->transfer_state) {
	case TRANSFER_START:
		/*
		 * Make sure to SYNC into Timer settings
		 * to ensure first bit time having full length
		 */
		data->cur_msg = 0;
		data->xfer_result = 0;
		data->transfer_state = TRANSFER_START2;
		break;

	case TRANSFER_START2:
		/* Generate start condition - sda low to high while clk high */
		gpio_dir_out(&data->gpio[I2C_PIN_SDA], 0);
		data->clock_state = 0;
		data->nacked = false;
		data->transfer_state = TRANSFER_ADDR;
		break;

	case TRANSFER_ADDR:
		data->data_byte = (msg->addr << 1);
		if (msg->flags & I2C_M_RD) {
			data->data_byte |= 1;
		}
		data->buf_offset = 0;
		/* no break */

	case TRANSFER_WRITE:
		if (data->transfer_state == TRANSFER_WRITE) {
			data->data_byte = msg->buf[data->buf_offset++];
		}
		data->transfer_state = TRANSFER_WBIT;
		data->bit_mask  = 0x80;
		/* no break */

	case TRANSFER_WBIT:
		if (!data->clock_state) {
			gpio_dir_out(&data->gpio[I2C_PIN_SDA],
					 data->data_byte & data->bit_mask);
			data->bit_mask >>= 1;
		}

		if (!data->bit_mask && data->clock_state) {
			data->transfer_state = TRANSFER_RACK;
		}
		data->clock_state ^= 1;
		break;

	case TRANSFER_READ:
		gpio_dir_in(&data->gpio[I2C_PIN_SDA]);
		data->transfer_state = TRANSFER_RBIT;
		data->bit_mask  = 0x80;
		data->data_byte = 0;
		/* no break */

	case TRANSFER_RBIT:
		if (data->clock_state) {
			data->data_byte |= gpio_get_value(&data->gpio[I2C_PIN_SDA])
					   ? data->bit_mask : 0;
			data->bit_mask >>= 1;

			if (!data->bit_mask) {
				msg->buf[data->buf_offset++] = data->data_byte;
				data->transfer_state = TRANSFER_WACK;
			}
		}
		data->clock_state ^= 1;
		break;

	case TRANSFER_RACK:
		if (!data->clock_state) {
			gpio_dir_in(&data->gpio[I2C_PIN_SDA]);
		}
		else {
			if (!gpio_get_value(&data->gpio[I2C_PIN_SDA])) {
				if (data->buf_offset < msg->len) {
					data->wait_cycles = 4;
					data->transfer_state = TRANSFER_WAIT;
				}
				else {
					data->transfer_state = TRANSFER_STOP;
				}
			}
			else {
				data->nacked = true;
				data->xfer_result = -ENXIO;
				data->transfer_state = TRANSFER_STOP;
			}
		}
		data->clock_state ^= 1;
		break;

	case TRANSFER_WACK:
		if (!data->clock_state) {
			/* ACK (or NACK the last byte read) */
			gpio_dir_out(&data->gpio[I2C_PIN_SDA],
					 data->buf_offset == msg->len);
		}
		else {
			if (data->buf_offset < msg->len) {
				data->wait_cycles = 2;
				data->transfer_state = TRANSFER_WAIT;
			}
			else {
				data->transfer_state = TRANSFER_STOP;
			}
		}
		data->clock_state ^= 1;
		break;

	case TRANSFER_WAIT:
		if (data->wait_cycles--) {
			break;
		}
		else if (msg->flags & I2C_M_RD) {
			data->transfer_state = TRANSFER_READ;
		}
		else {
			data->transfer_state = TRANSFER_WRITE;
		}
		break;

	case TRANSFER_STOP:
		/*
		 * Note: The official LEGO firmware does not generate stop
		 * condition except for in the middle of reads (see below).
		 * We are going by the book and doing a stop when we are
		 * supposed to. We can change it back if there are problems.
		 */
		gpio_dir_out(&data->gpio[I2C_PIN_SDA], 0);

		if (data->clock_state) {
			data->transfer_state = TRANSFER_STOP2;
		}
		data->clock_state = 1;
		break;

	case TRANSFER_STOP2:
		if ((data->cur_msg + 1) < data->num_msgs && !data->nacked) {
			/*
			 * This is some non-standard i2c weirdness for
			 * compatibility with the NXT ultrasonic sensor.
			 *
			 * Normal i2c would just send a restart (sda high
			 * to low while clk is high) between writing the
			 * address and reading the data. Instead, we send
			 * a stop (sda low to high while clk is high) and
			 * then do an extra clock cycle (low then high)
			 * before sending a start and reading the data.
			 */
			gpio_dir_out(&data->gpio[I2C_PIN_SDA], 1);
			data->clock_state ^= 1;
			if (data->clock_state) {
				data->cur_msg++;
				data->transfer_state = TRANSFER_RESTART;
			}
		}
		else {
			data->transfer_state = TRANSFER_STOP3;
		}
		break;

	case TRANSFER_RESTART:
		data->transfer_state = TRANSFER_START2;
		break;

	case TRANSFER_STOP3:
		/*
		 * Generate stop condition - sda low to high while clock
		 * is high. Leave sda in input position when not in use so
		 * that we can detect when a sensor is disconnected. (Device
		 * detection is implemented in the ev3-input-ports driver.)
		 */
		gpio_dir_in(&data->gpio[I2C_PIN_SDA]);
		data->transfer_state = TRANSFER_COMPLETE;
		/* no break */

	case TRANSFER_COMPLETE:
		/* send completion message */
		return true;

	default:
		break;
	}

	return false;
}

void main(void)
{
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	volatile uint8_t *status;
	uint8_t i;

	/* Allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize pru_virtqueue corresponding to vring0 (PRU to ARM Host direction) */
	pru_virtqueue_init(&transport.virtqueue0, &resourceTable.rpmsg_vring0, TO_ARM_HOST, FROM_ARM_HOST);

	/* Initialize pru_virtqueue corresponding to vring1 (ARM Host to PRU direction) */
	pru_virtqueue_init(&transport.virtqueue1, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	/* create a channel for each i2c port */
	for (i = 0; i < NUM_PORTS; i++) {
		while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, i2c_ports[i].name, i) != PRU_RPMSG_SUCCESS);
	}

	while (true) {
		/* reset cycle counter and start it */
		PRU1_CTRL.CTRL_bit.CTR_EN = 0;
		PRU1_CTRL.CYCLE = 0;
		PRU1_CTRL.CTRL_bit.CTR_EN = 1;

		/* handle each I2C port */
		for (i = 0; i < NUM_PORTS; i++) {
			if (i2c_ports[i].transfer_state != TRANSFER_IDLE && update_i2c_port(&i2c_ports[i])) {
				struct i2c_message_data *msg_data = (struct i2c_message_data *)payload;

				msg_data->num_msgs = i2c_ports[i].num_msgs;
				msg_data->xfer_result = i2c_ports[i].xfer_result;
				msg_data->msg[0] = i2c_ports[i].msgs[0];
				msg_data->msg[1] = i2c_ports[i].msgs[1];
				if (pru_rpmsg_send(&transport, i, i2c_ports[i].reply_to, msg_data, sizeof(struct i2c_message_data)) == PRU_RPMSG_SUCCESS) {
					i2c_ports[i].transfer_state = TRANSFER_IDLE;
				}
			}
		}

		/* Check bit 30 of register R31 to see if the ARM has kicked us */
		if (__R31 & HOST_INT) {
			/* Clear the event status */
			CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
			/* Receive all available messages, multiple messages can be sent per kick */
			while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
				struct i2c_message_data *msg_data = (struct i2c_message_data *)payload;

				if (dst >= NUM_PORTS)
					continue;
				if (len != sizeof(*msg_data))
					continue;

				i2c_ports[dst].reply_to = src;
				i2c_ports[dst].num_msgs = msg_data->num_msgs;
				i2c_ports[dst].msgs[0] = msg_data->msg[0];
				i2c_ports[dst].msgs[1] = msg_data->msg[1];
				i2c_ports[dst].transfer_state = msg_data->num_msgs ? TRANSFER_START : TRANSFER_IDLE;
				/* ensure clock signal is in correct state for IDLE/START */
				i2c_ports[dst].clock_state = 1;
				gpio_dir_out(&i2c_ports[dst].gpio[I2C_PIN_SCL], 1);
			}
		}

		/* We want the I2C clock to be 10kHz, so run the loop every 20kHz. (PRU runs at 200MHz) */
		while (PRU1_CTRL.CYCLE < 10000) { }
	}
}
