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

#include "xrt/xrt_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

struct KalmanFusionInterfaceWrapper;

struct KalmanFusionInterfaceWrapper *
kalman_fusion_create(void);

void
kalman_fusion_add_ui(struct KalmanFusionInterfaceWrapper *wrapper, void *root, const char *device_name);

void
kalman_fusion_destroy(struct KalmanFusionInterfaceWrapper *wrapper);

void
kalman_fusion_process_imu_data(struct KalmanFusionInterfaceWrapper *wrapper,
                               const struct xrt_imu_sample *sample,
                               const struct xrt_vec3 *accel_variance_optional,
                               const struct xrt_vec3 *gyro_variance_optional);

void
kalman_fusion_process_pose(struct KalmanFusionInterfaceWrapper *wrapper,
                           const struct xrt_pose_sample *sample,
                           const struct xrt_vec3 *position_variance_optional,
                           const struct xrt_vec3 *orientation_variance_optional,
                           float residual_limit);

void
kalman_fusion_get_prediction(struct KalmanFusionInterfaceWrapper *wrapper,
                             const timepoint_ns timestamp_ns,
                             struct xrt_space_relation *out_relation);

#ifdef __cplusplus
}
#endif // __cplusplus
