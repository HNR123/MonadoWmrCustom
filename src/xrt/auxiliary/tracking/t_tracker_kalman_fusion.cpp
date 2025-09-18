// Copyright 2024, Joel Valenciano
// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS VR2 tracker code that is expensive to compile.
 *
 * @author Joel Valenciano <joelv1907@gmail.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_tracking
 */
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <queue>
#include <iomanip>

#include <cassert>
#include <cstdio>
#include <queue>

#include "xrt/xrt_tracking.h"

#include "tracking/t_tracker_kalman_fusion.hpp"
#include "tracking/t_fusion.hpp"
#include "tracking/t_imu_fusion.hpp"

#include "math/m_eigen_interop.hpp"
#include "math/m_api.h"

#include "os/os_time.h"

#include "util/u_misc.h"
#include "util/u_var.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>

#include "flexkalman/FlexibleKalmanFilter.h"
#include "flexkalman/FlexibleUnscentedCorrect.h"
#include "flexkalman/AbsolutePositionMeasurement.h"
#include "flexkalman/AbsoluteOrientationMeasurement.h"
#include "flexkalman/AccelerometerMeasurement.h"
#include "flexkalman/PoseState.h"
#include "flexkalman/PoseSeparatelyDampedConstantVelocity.h"
#include "flexkalman/AugmentedState.h"
#include "flexkalman/AugmentedProcessModel.h"
#include "flexkalman/ConstantProcess.h"
#include "flexkalman/IMUBiasState.h"


DEBUG_GET_ONCE_OPTION(kalman_record_path, "KALMAN_RECORD_PATH", NULL);

#define CSV_EOL "\r\n"
#define CSV_PRECISION 10

namespace xrt::auxiliary::tracking {

using namespace xrt::auxiliary::math;

//! Anonymous namespace to hide implementation names
namespace {
	using Eigen::AngleAxisd;
	using Eigen::Quaterniond;
	using Eigen::Vector3d;
	using flexkalman::AccelerometerMeasurement;
	using flexkalman::pose_externalized_rotation::State;

	using BiasState = flexkalman::IMUBiasState;
	using CombinedState = flexkalman::AugmentedState<State, BiasState>;
	using ProcessModelA = flexkalman::PoseSeparatelyDampedConstantVelocityProcessModel<State>;
	using ProcessModelB = flexkalman::ConstantProcess<BiasState>;
	using CombinedProcessModel = flexkalman::AugmentedProcessModel<ProcessModelA, ProcessModelB>;
	using AbsolutePositionMeasurement = flexkalman::AbsolutePositionEKFMeasurement<State>;
	using AbsoluteOrientationMeasurement = flexkalman::AbsoluteOrientationEKFMeasurement<State>;

	class ImuPoseRecorder
	{
	public:
		ImuPoseRecorder(const char *record_path, const char *device_name)
		    : m_lock(), m_recording(false), m_record_path(record_path), m_device_name(device_name)
		{}

		void
		start()
		{
			std::lock_guard<std::mutex> lock(m_lock);
			if (m_recording) {
				return;
			}

			// Create dataset directories with current datetime suffix
			time_t seconds = os_realtime_get_ns() / U_1_000_000_000;
			constexpr size_t size = sizeof("YYYYMMDDHHmmss");
			char datetime[size] = {0};
			(void)strftime(datetime, size, "%Y%m%d%H%M%S", localtime(&seconds));
			std::string record_path = m_record_path + "/" + datetime + "_" + m_device_name;
			std::filesystem::create_directories(record_path);

			m_imu_csv = new std::ofstream{record_path + "/imu.csv"};
			*m_imu_csv << std::fixed << std::setprecision(CSV_PRECISION);
			*m_imu_csv << "#timestamp [ns],w_x [rad s^-1],w_y [rad s^-1],w_z [rad s^-1],"
			              "a_x [m s^-2],a_y [m s^-2],a_z [m s^-2]" CSV_EOL;

			m_pose_csv = new std::ofstream{record_path + "/pose.csv"};
			*m_pose_csv << std::fixed << std::setprecision(CSV_PRECISION);
			*m_pose_csv << "#timestamp [ns],p_x [m],p_y [m],p_z [m],"
			               "q_w [],q_x [],q_y [],q_z []" CSV_EOL;

			m_recording = true;
		}

		void
		stop()
		{
			std::lock_guard<std::mutex> lock(m_lock);
			if (!m_recording) {
				return;
			}

			if (m_imu_csv) {
				delete m_imu_csv;
				m_imu_csv = nullptr;
			}
			if (m_pose_csv) {
				delete m_pose_csv;
				m_pose_csv = nullptr;
			}
			m_recording = false;
		}

		void
		process_imu_data(const struct xrt_imu_sample *sample)
		{
			std::lock_guard<std::mutex> lock(m_lock);
			if (!m_recording) {
				return;
			}

			timepoint_ns ts = sample->timestamp_ns;
			xrt_vec3_f64 a = sample->accel_m_s2;
			xrt_vec3_f64 w = sample->gyro_rad_secs;

			*m_imu_csv << ts << ",";
			*m_imu_csv << w.x << "," << w.y << "," << w.z << ",";
			*m_imu_csv << a.x << "," << a.y << "," << a.z << CSV_EOL;
		}

		void
		process_pose(const struct xrt_pose_sample *sample)
		{
			std::lock_guard<std::mutex> lock(m_lock);
			if (!m_recording) {
				return;
			}

			timepoint_ns ts = sample->timestamp_ns;
			xrt_vec3 p = sample->pose.position;
			xrt_quat o = sample->pose.orientation;

			*m_pose_csv << ts << ",";
			*m_pose_csv << p.x << "," << p.y << "," << p.z << ",";
			*m_pose_csv << o.w << "," << o.x << "," << o.y << "," << o.z << CSV_EOL;
		}

	private:
		std::mutex m_lock;
		bool m_recording;
		std::string m_record_path;
		std::string m_device_name;

		std::ofstream *m_imu_csv = nullptr;
		std::ofstream *m_pose_csv = nullptr;
	};

	struct TrackingInfo
	{
		bool valid{false};
		bool tracked{false};
	};
	class KalmanFusion : public KalmanFusionInterface
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		void
		clear_position_tracked_flag() override;

		void
		process_imu_data(const struct xrt_imu_sample *sample,
		                 const struct xrt_vec3 *accel_variance_optional,
		                 const struct xrt_vec3 *gyro_variance_optional) override;
		void
		process_pose(const struct xrt_pose_sample *sample,
		             const struct xrt_vec3 *position_variance_optional,
		             const struct xrt_vec3 *orientation_variance_optional,
		             const float residual_limit) override;

		void
		get_prediction(const timepoint_ns when_ns, struct xrt_space_relation *out_relation) override;

		bool
		integrate_pose(const xrt_pose &pose,
		               const Vector3d &pos_variance,
		               const Vector3d &orient_variance,
		               double residual_limit);

		bool
		integrate_imu_sample(xrt_imu_sample &sample, Vector3d &accel_variance, Vector3d &gyro_variance);

		void
		integrate_samples_up_to(int64_t timestamp_ns);

		void
		add_ui(void *root, const char *device_name) override;

	private:
		void
		reset_filter();
		void
		reset_filter_and_imu();

		State filter_state;
		BiasState bias_state;
		CombinedState combined_state{filter_state, bias_state};

		ProcessModelA main_process_model;
		ProcessModelB bias_process_model;
		CombinedProcessModel combined_process_model{main_process_model, bias_process_model};

		SimpleIMUFusion imu;

		timepoint_ns filter_time_ns{0};
		bool tracked{false};
		TrackingInfo orientation_state;
		TrackingInfo position_state;

		// TODO: Figure out a good way to expose the pose offset
		// Distance from IMU origin to slam pose position
		Vector3d slam_pose_offset{0, 0, 0};

		std::queue<xrt_imu_sample> ff_samples;
		std::queue<std::pair<Vector3d, Vector3d>> ff_sample_variance;

		//! Recording info
		std::string m_device_name;
		bool m_recording;                    //!< Whether samples are being recorded
		struct u_var_button m_recording_btn; //!< UI button to start/stop `recording`
		ImuPoseRecorder *m_recorder;

		static void
		recorder_btn_cb(void *ptr)
		{
			KalmanFusion *self = static_cast<KalmanFusion *>(ptr);

			if (self->m_recording) {
				self->m_recorder->stop();
				(void)snprintf(self->m_recording_btn.label, sizeof(self->m_recording_btn.label),
				               "Record dataset");
				self->m_recording = false;
			} else {
				self->m_recorder->start();
				(void)snprintf(self->m_recording_btn.label, sizeof(self->m_recording_btn.label),
				               "Stop recording");
				self->m_recording = true;
			}
		}
	};

	void
	KalmanFusion::clear_position_tracked_flag()
	{
		position_state.tracked = false;
	}

	void
	KalmanFusion::reset_filter()
	{
		filter_state = State{};
		tracked = false;
		position_state = TrackingInfo{};
	}
	void
	KalmanFusion::reset_filter_and_imu()
	{
		reset_filter();
		orientation_state = TrackingInfo{};
		imu = SimpleIMUFusion{};
	}

	bool
	KalmanFusion::integrate_pose(const xrt_pose &pose,
	                             const Vector3d &pos_variance,
	                             const Vector3d &orient_variance,
	                             double residual_limit)
	{
		Vector3d pos = map_vec3(pose.position).cast<double>();
		Quaterniond orient = map_quat(pose.orientation).cast<double>();

		auto pos_meas = AbsolutePositionLeverArmMeasurement{pos, slam_pose_offset, pos_variance};
		auto orient_meas = AbsoluteOrientationMeasurement{orient, orient_variance};

		double pos_resid = pos_meas.getResidual(filter_state).norm();

		if (pos_resid > residual_limit) {
			// Residual arbitrarily "too large"
			U_LOG_W(
			    "position measurement residual is %f, resetting "
			    "filter state",
			    pos_resid);
			reset_filter();
			return false;
		}

		return flexkalman::correctUnscented(filter_state, orient_meas) &&
		       flexkalman::correctUnscented(filter_state, pos_meas);
	}

	bool
	KalmanFusion::integrate_imu_sample(xrt_imu_sample &sample, Vector3d &accel_variance, Vector3d &gyro_variance)
	{
		//! @todo use better measurements instead of the preceding "simple
		//! fusion"
		const Vector3d G = Vector3d::UnitY() * -MATH_GRAVITY_M_S2;
		Vector3d acc = map_vec3_f64(sample.accel_m_s2);
		Vector3d gyro = map_vec3_f64(sample.gyro_rad_secs);

		// TODO: Figure out the acceleration.
		acc = Vector3d::Zero();

		gyro = filter_state.getQuaternion() * gyro;

		auto acc_meas = AccelerometerMeasurement{acc, G, accel_variance};
		auto gyro_meas = BiasedGyroMeasurement{gyro, gyro_variance};

		if (!(flexkalman::correctUnscented(combined_state, acc_meas) &&
		      flexkalman::correctUnscented(combined_state, gyro_meas))) {
			U_LOG_E(
			    "Got non-finite something when filtering IMU - "
			    "resetting filter and IMU fusion!");
			reset_filter_and_imu();
		}

		// 7200 deg/sec
		constexpr double max_rad_per_sec = 20.0 * double(EIGEN_PI) * 2;
		if (filter_state.angularVelocity().squaredNorm() > max_rad_per_sec * max_rad_per_sec) {
			U_LOG_E(
			    "Got excessive angular velocity when filtering "
			    "IMU - resetting filter and IMU fusion!");
			reset_filter_and_imu();
		}

		return true;
	}

	void
	KalmanFusion::integrate_samples_up_to(int64_t target_ns)
	{
		while (!ff_samples.empty()) {
			auto sample = ff_samples.front();
			auto variance = ff_sample_variance.front();
			auto ts = sample.timestamp_ns;

			if (filter_time_ns > 0 && ts != filter_time_ns) {
				double dt = time_ns_to_s(ts - filter_time_ns);
				assert(dt > 0);
				flexkalman::predict(combined_state, combined_process_model, dt);
			}

			filter_time_ns = ts;

			if (ts > target_ns)
				break;

			integrate_imu_sample(sample, variance.first, variance.second);
			ff_sample_variance.pop();
			ff_samples.pop();
		}
	}

	void
	KalmanFusion::process_imu_data(const struct xrt_imu_sample *sample,
	                               const struct xrt_vec3 *accel_variance_optional,
	                               const struct xrt_vec3 *gyro_variance_optional)
	{
		Vector3d accel_variance = Vector3d::Constant(0.01);
		Vector3d gyro_variance = Vector3d::Constant(0.01);
		if (accel_variance_optional) {
			accel_variance = map_vec3(*accel_variance_optional).cast<double>();
		}
		if (gyro_variance_optional) {
			gyro_variance = map_vec3(*gyro_variance_optional).cast<double>();
		}

		imu.handleAccel(map_vec3_f64(sample->accel_m_s2), sample->timestamp_ns);
		imu.handleGyro(map_vec3_f64(sample->gyro_rad_secs), sample->timestamp_ns);
		imu.postCorrect();

		if (tracked) {
			// TODO: Prevent the queue from getting too big.
			// assert(ff_samples.size() <= 1024);
			if (ff_samples.size() >= 1024) {
				ff_samples.pop();
				ff_sample_variance.pop();
			}

			ff_samples.push(*sample);
			ff_sample_variance.push({accel_variance, gyro_variance});
		}

		if (m_recorder) {
			m_recorder->process_imu_data(sample);
		}
	}

	void
	KalmanFusion::process_pose(const struct xrt_pose_sample *sample,
	                           const struct xrt_vec3 *position_variance_optional,
	                           const struct xrt_vec3 *orientation_variance_optional,
	                           float residual_limit)
	{
		Vector3d position_variance{1.e-4, 1.e-4, 4.e-4};
		Vector3d orientation_variance{1.e-4, 1.e-4, 4.e-4};
		if (position_variance_optional) {
			position_variance = map_vec3(*position_variance_optional).cast<double>();
		}
		if (orientation_variance_optional) {
			orientation_variance = map_vec3(*orientation_variance_optional).cast<double>();
		}

		if (sample->timestamp_ns < filter_time_ns) {
			printf("Skipping old pose sample, filter_time=%zu, ts=%zu.\n", filter_time_ns,
			       sample->timestamp_ns);
			return;
		}

		printf("didnt skip old sample, filter_time=%zu, ts=%zu.\n", filter_time_ns, sample->timestamp_ns);

		integrate_samples_up_to(sample->timestamp_ns);

		if (integrate_pose(sample->pose, position_variance, orientation_variance, residual_limit)) {
			tracked = true;
			position_state.valid = true;
			position_state.tracked = true;
			orientation_state.valid = true;
			orientation_state.tracked = true;
		} else {
			U_LOG_W(
			    "Got non-finite something when filtering "
			    "tracker - resetting filter!");
			reset_filter();
		}

		if (m_recorder) {
			m_recorder->process_pose(sample);
		}
	}


	void
	KalmanFusion::get_prediction(timepoint_ns when_ns, struct xrt_space_relation *out_relation)
	{
		if (out_relation == NULL) {
			return;
		}
		// Clear to identity values
		U_ZERO(out_relation);
		out_relation->pose.orientation.w = 1;
		if (!tracked || filter_time_ns == 0) {
			return;
		}

		// integrate_samples_up_to(when_ns);

		float dt = time_ns_to_s(when_ns - filter_time_ns);
		auto predicted_state = flexkalman::getPrediction(filter_state, main_process_model, dt);

		map_vec3(out_relation->pose.position) = predicted_state.position().cast<float>();
		map_quat(out_relation->pose.orientation) = predicted_state.getQuaternion().cast<float>();
		map_vec3(out_relation->linear_velocity) = predicted_state.velocity().cast<float>();
		map_vec3(out_relation->angular_velocity) = predicted_state.angularVelocity().cast<float>();

		uint64_t flags = 0;
		if (position_state.valid) {
			flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;
			flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
			if (position_state.tracked) {
				flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
			}
		}
		if (orientation_state.valid) {
			flags |= XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
			flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
			if (orientation_state.tracked) {
				flags |= XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
			}
		}
		out_relation->relation_flags = (xrt_space_relation_flags)flags;
	}

	void
	KalmanFusion::add_ui(void *root, const char *device_name)
	{
		const char *record_path = debug_get_option_kalman_record_path();
		if (record_path != NULL) {
			m_recorder = new ImuPoseRecorder(record_path, device_name);

			m_recording_btn.cb = KalmanFusion::recorder_btn_cb;
			m_recording_btn.ptr = this;
			u_var_add_button(root, &m_recording_btn, "Record dataset");
		}
	}

} // namespace


std::unique_ptr<KalmanFusionInterface>
KalmanFusionInterface::create()
{
	auto ret = std::make_unique<KalmanFusion>();
	return ret;
}


} // namespace xrt::auxiliary::tracking
