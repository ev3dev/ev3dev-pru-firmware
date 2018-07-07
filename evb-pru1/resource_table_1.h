/*
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

#ifndef _RSC_TABLE_PRU_H_
#define _RSC_TABLE_PRU_H_

#include <stddef.h>
#include <rsc_types.h>
#include "pru_virtio_ids.h"

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define PRU_RPMSG_VQ0_SIZE	16
#define PRU_RPMSG_VQ1_SIZE	16

/*
 * The feature bitmap for virtio rpmsg
 */
#define VIRTIO_RPMSG_F_NS	0		//name service notifications

/* This firmware supports name service notifications as one of its features */
#define RPMSG_PRU_C0_FEATURES	(1 << VIRTIO_RPMSG_F_NS)

/* Definition for unused interrupts */
#define HOST_UNUSED		255

/* Number of entries in the table */
#define NUM_RESORUCES	2

/* Mapping sysevts to a channel. Each pair contains a sysevt, channel. */
struct ch_map pru_intc_map[] = {
	{ .evt = 18, .ch = 3 },
	{ .evt = 19, .ch = 1 },
};

struct pru1_resource_table {
	struct resource_table base;

	uint32_t offset[NUM_RESORUCES];

	/* rpmsg vdev entry */
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring rpmsg_vring0;
	struct fw_rsc_vdev_vring rpmsg_vring1;

	/* intc definition */
	struct fw_rsc_custom pru_ints;
};

#pragma DATA_SECTION(resourceTable, ".resource_table")
#pragma RETAIN(resourceTable)
struct pru1_resource_table resourceTable = {
	.base = {
		.ver = 1,	/* only version 1 is supported by the current driver */
		.num = NUM_RESORUCES,
		.reserved = { 0, 0 },	/* must be zero */
	},
	/* offsets to entries */
	.offset = {
		offsetof(struct pru1_resource_table, rpmsg_vdev),
		offsetof(struct pru1_resource_table, pru_ints),
	},

	/* rpmsg vdev entry */
	.rpmsg_vdev = {
		.type          = TYPE_VDEV,
		.id            = VIRTIO_ID_RPMSG,
		.notifyid      = 0,
		.dfeatures     = RPMSG_PRU_C0_FEATURES,
		.gfeatures     = 0,
		.config_len    = 0,
		.status        = 0,
		.num_of_vrings = 2, // only two is supported
		.reserved      = { 0, 0 },
	},
	/* the two vrings */
	.rpmsg_vring0 = {
		.da       = 0,                      //will be populated by host, can't pass it in
		.align    = 16,                     //(bytes),
		.num      = PRU_RPMSG_VQ0_SIZE,     //num of descriptors
		.notifyid = 0,                      //will be populated, can't pass right now
		.reserved = 0,
	},
	.rpmsg_vring1 = {
		.da       = 0,                      //will be populated by host, can't pass it in
		.align    = 16,                     //(bytes),
		.num      = PRU_RPMSG_VQ1_SIZE,     //num of descriptors
		.notifyid = 0,                      //will be populated, can't pass right now
		.reserved = 0,
	},

	.pru_ints = {
		.type = TYPE_POSTLOAD_VENDOR,
		.sub_type = TYPE_PRU_INTS,
		.rsc_size = sizeof(struct fw_rsc_custom_ints),
		.rsc.pru_ints = {
			.version = 0x0000,
			.channel_host = {
				HOST_UNUSED,
				1,
				HOST_UNUSED,
				3,
				HOST_UNUSED,
				HOST_UNUSED,
				HOST_UNUSED,
				HOST_UNUSED,
				HOST_UNUSED,
				HOST_UNUSED,
			},
			.num_evts = (sizeof(pru_intc_map) / sizeof(struct ch_map)),
			.event_channel = pru_intc_map,
		},
	},
};

#endif /* _RSC_TABLE_PRU_H_ */
