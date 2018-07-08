/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
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

#ifndef _RSC_TABLE_PRU_H_
#define _RSC_TABLE_PRU_H_

#include <stddef.h>

#include <pru_virtio_ids.h>
#include <rsc_types.h>

/*
 * The feature bitmap for virtio rpmsg
 */
#define VIRTIO_RPMSG_F_NS	0	//name service notifications

/* This firmware supports name service notifications as one of its features */
#define RPMSG_FEATURES		(1 << VIRTIO_RPMSG_F_NS)

#define NUM_RESOURCES		1

struct custom_resource_table {
	struct resource_table base;

	uint32_t offset[NUM_RESOURCES];

	/* resource 0 */
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring vring0;
	struct fw_rsc_vdev_vring vring1;
};

#pragma DATA_SECTION(resource_table, ".resource_table")
#pragma RETAIN(resource_table)
struct custom_resource_table resource_table = {
	.base = {
		.ver = 1,
		.num = NUM_RESOURCES,
		.reserved = { 0, 0 },
	},
	.offset = {
		offsetof(struct custom_resource_table, rpmsg_vdev),
	},
	.rpmsg_vdev = {
		.type = TYPE_VDEV,
		.id = VIRTIO_ID_RPMSG,
		.notifyid = 0,
		.dfeatures = RPMSG_FEATURES,
		.gfeatures = 0,	//will be populated by host
		.config_len = 0,
		.status = 0,
		.num_of_vrings = 2,
		.reserved = { 0, 0 },
	},
	.vring0 = {
		.da       = 0,	//will be populated by host
		.align    = 16,
		.num      = 16,	// must be power of 2
		.notifyid = 0,	//will be populated by host
		.reserved = 0,
	},
	.vring1 = {
		.da       = 0,	//will be populated by host
		.align    = 16,
		.num      = 16,	// must be power of 2
		.notifyid = 0,	//will be populated by host
		.reserved = 0,
	},
};

#endif /* _RSC_TABLE_PRU_H_ */
