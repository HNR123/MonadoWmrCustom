// Copyright 2024, Joel Valenciano
// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  C interface to generalized kalman filter.
 *
 * @author Joel Valenciano <joelv1907@gmail.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_tracking
 */

#include "t_tracker_kalman_fusion.hpp"
#include "xrt/xrt_tracking.h"

#include <cstdlib>

using xrt::auxiliary::tracking::KalmanFusionInterface;

struct KalmanFusionInterfaceWrapper
{
	std::unique_ptr<KalmanFusionInterface> fusion;

	KalmanFusionInterfaceWrapper() : fusion(KalmanFusionInterface::create()) {}

	~KalmanFusionInterfaceWrapper() {}
};

extern "C" {

struct KalmanFusionInterfaceWrapper *
kalman_fusion_create(void)
{
	return new KalmanFusionInterfaceWrapper;
}

void
kalman_fusion_destroy(KalmanFusionInterfaceWrapper *wrapper)
{
	delete wrapper;
}

void
kalman_fusion_add_ui(struct KalmanFusionInterfaceWrapper *wrapper, void *root, const char *device_name)
{
	wrapper->fusion->add_ui(root, device_name);
}

void
kalman_fusion_process_imu_data(KalmanFusionInterfaceWrapper *wrapper,
                               const struct xrt_imu_sample *sample,
                               const struct xrt_vec3 *accel_variance_optional,
                               const struct xrt_vec3 *gyro_variance_optional)
{
	wrapper->fusion->process_imu_data(sample, accel_variance_optional, gyro_variance_optional);
}

void
kalman_fusion_process_pose(KalmanFusionInterfaceWrapper *wrapper,
                           const struct xrt_pose_sample *sample,
                           const struct xrt_vec3 *position_variance_optional,
                           const struct xrt_vec3 *orientation_variance_optional,
                           const float residual_limit)
{
	wrapper->fusion->process_pose(sample, position_variance_optional, orientation_variance_optional,
	                              residual_limit);
}

void
kalman_fusion_get_prediction(struct KalmanFusionInterfaceWrapper *wrapper,
                             const timepoint_ns timestamp_ns,
                             struct xrt_space_relation *out_relation)
{
	wrapper->fusion->get_prediction(timestamp_ns, out_relation);
}
}
