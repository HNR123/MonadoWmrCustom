// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS Move tracker code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#pragma once

#ifndef __cplusplus
#error "This header is C++-only."
#endif

#include "xrt/xrt_defines.h"
#include "xrt/xrt_tracking.h"

#include "util/u_time.h"

#include <memory>


namespace xrt::auxiliary::tracking {

class KalmanFusionInterface
{
public:
	static std::unique_ptr<KalmanFusionInterface>
	create();
	virtual ~KalmanFusionInterface() = default;

	virtual void
	add_ui(void *root, const char *device_name) = 0;

	/*!
	 * @brief If you've lost sight of the position tracking and won't even
	 * enter another function in this class.
	 */
	virtual void
	clear_position_tracked_flag() = 0;

	virtual void
	process_imu_data(const struct xrt_imu_sample *sample,
	                 const struct xrt_vec3 *accel_variance_optional,
	                 const struct xrt_vec3 *gyro_variance_optional) = 0;
	virtual void
	process_pose(const struct xrt_pose_sample *sample,
	             const struct xrt_vec3 *position_variance_optional,
	             const struct xrt_vec3 *orientation_variance_optional,
	             const float residual_limit) = 0;

	virtual void
	get_prediction(const timepoint_ns when_ns, struct xrt_space_relation *out_relation) = 0;
};
} // namespace xrt::auxiliary::tracking
