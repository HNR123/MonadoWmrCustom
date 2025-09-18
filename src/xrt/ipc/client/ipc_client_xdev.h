// Copyright 2020-2023, Collabora, Ltd.
// Copyright 2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared functions for IPC client @ref xrt_device.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup ipc_client
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


struct ipc_connection;

/*!
 * An IPC client proxy for an @ref xrt_device.
 *
 * @implements xrt_device
 * @ingroup ipc_client
 */
struct ipc_client_xdev
{
	struct xrt_device base;

	struct ipc_connection *ipc_c;

	uint32_t device_id;
};

/*!
 * Convenience helper to go from a xdev to @ref ipc_client_xdev.
 *
 * @ingroup ipc_client
 */
static inline struct ipc_client_xdev *
ipc_client_xdev(struct xrt_device *xdev)
{
	return (struct ipc_client_xdev *)xdev;
}

/*!
 * Initializes a ipc_client_xdev so that it's basically fully usable as a
 * @ref xrt_device object. Does not fill in the destroy function or the any
 * if the HMD components / functions.
 *
 * @ingroup ipc_client
 * @public @memberof ipc_client_xdev
 */
void
ipc_client_xdev_init(struct ipc_client_xdev *icx,
                     struct ipc_connection *ipc_c,
                     struct xrt_tracking_origin *xtrack,
                     uint32_t device_id);

/*!
 * Frees any memory that was allocated as part of init and resets some pointers.
 *
 * @ingroup ipc_client
 * @public @memberof ipc_client_xdev
 */
void
ipc_client_xdev_fini(struct ipc_client_xdev *icx);


#ifdef __cplusplus
}
#endif
