// Copyright 2020-2021, N Madsen.
// Copyright 2020-2023, Collabora, Ltd.
// Copyright 2020-2023, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver for WMR Controller.
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_wmr
 */

#include "os/os_time.h"
#include "os/os_hid.h"

#include "math/m_mathinclude.h"
#include "math/m_api.h"
#include "math/m_vec2.h"
#include "math/m_vec3.h"
#include "math/m_predict.h"
#include "math/m_space.h"

#include "tracking/t_constellation_tracking.h"

#include "util/u_file.h"
#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_trace_marker.h"

#include "wmr_common.h"
#include "wmr_controller_base.h"
#include "wmr_config_key.h"
#include "wmr_hmd.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <inttypes.h>

#define WMR_TRACE(wcb, ...) U_LOG_XDEV_IFL_T(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_TRACE_HEX(wcb, ...) U_LOG_XDEV_IFL_T_HEX(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_DEBUG(wcb, ...) U_LOG_XDEV_IFL_D(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_DEBUG_HEX(wcb, ...) U_LOG_XDEV_IFL_D_HEX(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_INFO(wcb, ...) U_LOG_XDEV_IFL_I(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_WARN(wcb, ...) U_LOG_XDEV_IFL_W(&wcb->base, wcb->log_level, __VA_ARGS__)
#define WMR_ERROR(wcb, ...) U_LOG_XDEV_IFL_E(&wcb->base, wcb->log_level, __VA_ARGS__)

#define wmr_controller_hexdump_buffer(wcb, label, buf, length)                                                         \
	do {                                                                                                           \
		WMR_DEBUG(wcb, "%s", label);                                                                           \
		WMR_DEBUG_HEX(wcb, buf, length);                                                                       \
	} while (0);


static inline struct wmr_controller_base *
wmr_controller_base(struct xrt_device *p)
{
	return (struct wmr_controller_base *)p;
}

/* Called from subclasses' handle_input_packet method, with data_lock */
void
wmr_controller_base_imu_sample(struct wmr_controller_base *wcb,
                               struct wmr_controller_base_imu_sample *imu_sample,
                               timepoint_ns rx_mono_ns)
{
	/* Extend 32-bit tick count to 64-bit and convert to ns */
	uint32_t tick_delta = imu_sample->timestamp_ticks - (uint32_t)wcb->last_timestamp_ticks;
	wcb->last_timestamp_ticks += tick_delta;

	timepoint_ns now_hw_ns = wcb->last_timestamp_ticks * WMR_MOTION_CONTROLLER_NS_PER_TICK;

	/* Update windowed min-skew estimator and convert hardware timestamp into monotonic clock */
	m_clock_windowed_skew_tracker_push(wcb->hw2mono_clock, rx_mono_ns, now_hw_ns);

	timepoint_ns mono_time_ns;
	if (!m_clock_windowed_skew_tracker_to_local(wcb->hw2mono_clock, now_hw_ns, &mono_time_ns)) {
		WMR_DEBUG(wcb,
		          "Dropping IMU sample until clock estimator synchronises. Rcv ts %" PRIu64 " hw ts %" PRIu64,
		          rx_mono_ns, now_hw_ns);
		return;
	}

	/*
	 * Check if the timepoint does time travel, we get one or two
	 * old samples when the device has not been cleanly shut down,
	 * and if the controller is left idle and goes into low power
	 * mode it can come back with a different clock epoch
	 */
	if (wcb->last_imu_timestamp_ns > (uint64_t)mono_time_ns) {
		WMR_WARN(wcb,
		         "Received sample from the past, new: %" PRIu64 ", last: %" PRIu64 ", diff: %" PRIu64
		         ". resetting clock tracking",
		         mono_time_ns, now_hw_ns, mono_time_ns - now_hw_ns);
		// Reinit. The 3dof fusion will assert if time goes backward
		wcb->last_imu_timestamp_ns = 0;
		wcb->last_imu_device_timestamp_ns = 0;
		m_imu_3dof_init(&wcb->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
		m_clock_windowed_skew_tracker_reset(wcb->hw2mono_clock);
		m_clock_windowed_skew_tracker_push(wcb->hw2mono_clock, rx_mono_ns, now_hw_ns);
		return;
	}

	WMR_TRACE(wcb, "Accel [m/s^2] : %f", m_vec3_len(imu_sample->acc));

	m_imu_3dof_update(&wcb->fusion, mono_time_ns, &imu_sample->acc, &imu_sample->gyro);
	wcb->last_imu_timestamp_ns = mono_time_ns;
	wcb->last_imu_device_timestamp_ns = now_hw_ns;
	wcb->last_angular_velocity = imu_sample->gyro;
	wcb->last_imu = *imu_sample;

	struct xrt_imu_sample k_imu_sample = {
	    .timestamp_ns = mono_time_ns,
	    .gyro_rad_secs = {imu_sample->gyro.x, imu_sample->gyro.y, imu_sample->gyro.z},
	    .accel_m_s2 = {imu_sample->acc.x, imu_sample->acc.y, imu_sample->acc.z}};
	struct xrt_vec3 accel_variance = {0.01, 0.01, 0.01};
	struct xrt_vec3 gyro_variance = {0.01, 0.01, 0.01};
	kalman_fusion_process_imu_data(wcb->kalman_fusion, &k_imu_sample, &accel_variance, &gyro_variance);
}

static void
wmr_controller_base_send_timesync(struct wmr_controller_base *wcb);
static void
wmr_controller_base_send_keepalive(struct wmr_controller_base *wcb, uint64_t time_ns);

static void
receive_bytes(struct wmr_controller_base *wcb, uint64_t time_ns, uint8_t *buffer, uint32_t buf_size)
{
	if (buf_size < 1) {
		WMR_ERROR(wcb, "WMR Controller: Error receiving short packet");
		return;
	}

	switch (buffer[0]) {
	case WMR_MOTION_CONTROLLER_STATUS_MSG:
		os_mutex_lock(&wcb->data_lock);
		// Send a timesync packet if needed
		if (wcb->timesync_updated) {
			wmr_controller_base_send_timesync(wcb);
			wcb->timesync_updated = false;
		}

		wmr_controller_base_send_keepalive(wcb, time_ns);

		// Note: skipping msg type byte
		bool b = wcb->handle_input_packet(wcb, time_ns, &buffer[1], (size_t)buf_size - 1);
		os_mutex_unlock(&wcb->data_lock);

		if (!b) {
			WMR_ERROR(wcb, "WMR Controller: Failed handling message type: %02x, size: %i", buffer[0],
			          buf_size);
			wmr_controller_hexdump_buffer(wcb, "Controller Message", buffer, buf_size);
			return;
		}

		break;
	default: WMR_DEBUG(wcb, "WMR Controller: Unknown message type: %02x, size: %i", buffer[0], buf_size); break;
	}

	return;
}

static bool
wmr_controller_send_bytes(struct wmr_controller_base *wcb, const uint8_t *buffer, uint32_t buf_size)
{
	bool res = false;

	os_mutex_lock(&wcb->conn_lock);
	struct wmr_controller_connection *conn = wcb->wcc;
	if (conn != NULL) {
		res = wmr_controller_connection_send_bytes(conn, buffer, buf_size);
	}
	os_mutex_unlock(&wcb->conn_lock);

	return res;
}

static int
wmr_controller_read_sync(struct wmr_controller_base *wcb, uint8_t *buffer, uint32_t buf_size, int timeout_ms)
{
	int res = -1;
	os_mutex_lock(&wcb->conn_lock);
	struct wmr_controller_connection *conn = wcb->wcc;
	if (conn != NULL) {
		res = wmr_controller_connection_read_sync(conn, buffer, buf_size, timeout_ms);
	}
	os_mutex_unlock(&wcb->conn_lock);

	return res;
}

static int
wmr_controller_send_fw_cmd(struct wmr_controller_base *wcb,
                           const struct wmr_controller_fw_cmd *fw_cmd,
                           unsigned char response_code,
                           struct wmr_controller_fw_cmd_response *response)
{
	// comms timeout. Replies are usually in 10ms or so but the first can take longer
	const int timeout_ms = 250;
	const int timeout_ns = timeout_ms * U_TIME_1MS_IN_NS;
	int64_t timeout_start = os_monotonic_get_ns();
	int64_t timeout_end_ns = timeout_start + timeout_ns;

	if (!wmr_controller_send_bytes(wcb, fw_cmd->buf, sizeof(fw_cmd->buf))) {
		return -1;
	}

	do {
		int size = wmr_controller_read_sync(wcb, response->buf, sizeof(response->buf), timeout_ms);
		if (size == -1) {
			return -1;
		}

		if (size < 1) {
			// Ignore 0-byte reads (timeout) and try again
			continue;
		}

		if (response->buf[0] == response_code) {
			WMR_TRACE(wcb, "Controller fw read returned %d bytes", size);
			if (size != sizeof(response->buf) || (response->response.cmd_id_echo != fw_cmd->cmd.cmd_id)) {
				WMR_DEBUG(
				    wcb, "Unexpected fw response - size %d (expected %zu), cmd_id_echo %u != cmd_id %u",
				    size, sizeof(response->buf), response->response.cmd_id_echo, fw_cmd->cmd.cmd_id);
				return -1;
			}

			response->response.blk_remain = __le32_to_cpu(response->response.blk_remain);
			return size;
		}
	} while (os_monotonic_get_ns() < timeout_end_ns);

	WMR_WARN(wcb, "Controller fw read timed out after %u ms",
	         (unsigned int)((os_monotonic_get_ns() - timeout_start) / U_TIME_1MS_IN_NS));
	return -ETIMEDOUT;
}

XRT_MAYBE_UNUSED static int
wmr_read_fw_block(struct wmr_controller_base *d, uint8_t blk_id, uint8_t **out_data, size_t *out_size)
{
	struct wmr_controller_fw_cmd_response fw_cmd_response;

	uint8_t *data;
	uint8_t *data_pos;
	uint8_t *data_end;
	uint32_t data_size;
	uint32_t remain;

	struct wmr_controller_fw_cmd fw_cmd;
	memset(&fw_cmd, 0, sizeof(fw_cmd));

	fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x02, blk_id, 0xffffffff);
	if (wmr_controller_send_fw_cmd(d, &fw_cmd, 0x02, &fw_cmd_response) < 0) {
		WMR_WARN(d, "Failed to read fw - cmd 0x02 failed to read header for block %d", blk_id);
		return -1;
	}

	data_size = fw_cmd_response.response.blk_remain + fw_cmd_response.response.len;
	WMR_DEBUG(d, "FW header %d bytes, %u bytes in block", fw_cmd_response.response.len, data_size);
	if (data_size == 0)
		return -1;

	data = calloc(1, data_size + 1);
	if (!data) {
		return -1;
	}
	data[data_size] = '\0';

	remain = data_size;
	data_pos = data;
	data_end = data + data_size;

	uint8_t to_copy = fw_cmd_response.response.len;

	memcpy(data_pos, fw_cmd_response.response.data, to_copy);
	data_pos += to_copy;
	remain -= to_copy;

	while (remain > 0) {
		fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x02, blk_id, remain);

		os_nanosleep(U_TIME_1MS_IN_NS * 10); // Sleep 10ms
		if (wmr_controller_send_fw_cmd(d, &fw_cmd, 0x02, &fw_cmd_response) < 0) {
			WMR_WARN(d, "Failed to read fw - cmd 0x02 failed @ offset %zu", data_pos - data);
			return -1;
		}

		uint8_t to_copy = fw_cmd_response.response.len;
		if (data_pos + to_copy > data_end)
			to_copy = data_end - data_pos;

		WMR_DEBUG(d, "Read %d bytes @ offset %zu / %d", to_copy, data_pos - data, data_size);
		memcpy(data_pos, fw_cmd_response.response.data, to_copy);
		data_pos += to_copy;
		remain -= to_copy;
	}

	WMR_DEBUG(d, "Read %d-byte FW data block %d", data_size, blk_id);
	wmr_controller_hexdump_buffer(d, "Data block", data, data_size);

	*out_data = data;
	*out_size = data_size;

	return 0;
}

/*
 *
 * Config functions.
 *
 */
static bool
read_controller_fw_info(struct wmr_controller_base *wcb,
                        uint32_t *fw_revision,
                        uint16_t *calibration_size,
                        char serial_no[16])
{
	uint8_t *data = NULL;
	size_t data_size;
	int ret;

	/* FW block 0 contains the FW revision (offset 0x14, size 4) and
	 * calibration block size (offset 0x34 size 2) */
	ret = wmr_read_fw_block(wcb, 0x0, &data, &data_size);
	if (ret < 0 || data == NULL) {
		WMR_ERROR(wcb, "Failed to read FW info block 0");
		return false;
	}
	if (data_size < 0x36) {
		WMR_ERROR(wcb, "Failed to read FW info block 0 - too short");
		free(data);
		return false;
	}


	const unsigned char *tmp = data + 0x14;
	*fw_revision = read32(&tmp);
	tmp = data + 0x34;
	*calibration_size = read16(&tmp);

	free(data);

	/* FW block 3 contains the controller serial number at offset
	 * 0x84, size 16 bytes */
	ret = wmr_read_fw_block(wcb, 0x3, &data, &data_size);
	if (ret < 0 || data == NULL) {
		WMR_ERROR(wcb, "Failed to read FW info block 3");
		return false;
	}
	if (data_size < 0x94) {
		WMR_ERROR(wcb, "Failed to read FW info block 3 - too short");
		free(data);
		return false;
	}

	memcpy(serial_no, data + 0x84, 0x10);
	serial_no[16] = '\0';

	free(data);
	return true;
}

char *
build_cache_filename(char *serial_no)
{
	int outlen = strlen("controller-") + strlen(serial_no) + strlen(".json") + 1;
	char *out = malloc(outlen);
	int ret = snprintf(out, outlen, "controller-%s.json", serial_no);

	assert(ret <= outlen);
	(void)ret;

	// Make sure the filename is valid
	for (char *cur = out; *cur != '\0'; cur++) {
		if (!isalnum(*cur) && *cur != '.') {
			*cur = '_';
		}
	}

	return out;
}

static bool
read_calibration_cache(struct wmr_controller_base *wcb, char *cache_filename)
{
	FILE *f = u_file_open_file_in_config_dir_subpath("wmr", cache_filename, "r");
	uint8_t *buffer = NULL;

	if (f == NULL) {
		WMR_DEBUG(wcb, "Failed to open wmr/%s cache file or it doesn't exist.", cache_filename);
		return false;
	}

	// Read the file size to allocate a read buffer
	fseek(f, 0L, SEEK_END);
	size_t file_size = ftell(f);

	// Reset and read the data
	fseek(f, 0L, SEEK_SET);

	buffer = calloc(file_size + 1, sizeof(uint8_t));
	if (buffer == NULL) {
		goto fail;
	}
	buffer[file_size] = '\0';

	size_t ret = fread(buffer, sizeof(char), file_size, f);
	if (ret != file_size) {
		WMR_WARN(wcb, "Cache file wmr/%s failed to read %u bytes (got %u)", cache_filename, (int)file_size,
		         (int)ret);
		goto fail;
	}

	if (!wmr_controller_config_parse(&wcb->config, (char *)buffer, wcb->log_level)) {
		WMR_WARN(wcb, "Cache file wmr/%s contains invalid JSON. Ignoring", cache_filename);
		goto fail;
	}

	fclose(f);
	free(buffer);

	return true;

fail:
	if (buffer) {
		free(buffer);
	}
	fclose(f);
	return false;
}

static void
write_calibration_cache(struct wmr_controller_base *wcb, char *cache_filename, uint8_t *data, size_t data_size)
{
	FILE *f = u_file_open_file_in_config_dir_subpath("wmr", cache_filename, "w");
	if (f == NULL) {
		return;
	}

	size_t ret = fwrite(data, sizeof(char), data_size, f);
	if (ret != data_size) {
		fclose(f);
		return;
	}

	fclose(f);
}

static bool
read_controller_config(struct wmr_controller_base *wcb)
{
	unsigned char *config_json_block;
	int ret;
	uint32_t fw_revision;
	uint16_t calibration_size;
	char serial_no[16 + 1];

	if (!read_controller_fw_info(wcb, &fw_revision, &calibration_size, serial_no)) {
		return false;
	}

	WMR_INFO(wcb, "Reading configuration for controller serial %s. FW revision %x", serial_no, fw_revision);

#if 0
  /* WMR also reads block 0x14, which seems to have some FW revision info,
   * but we don't use it */
	// Read block 0x14
	ret = wmr_read_fw_block(wcb, 0x14, &data, &data_size);
	if (ret < 0 || data == NULL)
		return false;
	free(data);
	data = NULL;
#endif

	// Read config block
	WMR_INFO(wcb, "Reading %s controller config",
	         wcb->base.device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER ? "left" : "right");

	// Check if we have it cached already
	char *cache_filename = build_cache_filename(serial_no);

	if (!read_calibration_cache(wcb, cache_filename)) {
		unsigned char *data = NULL;
		size_t data_size;

		ret = wmr_read_fw_block(wcb, 0x02, &data, &data_size);
		if (ret < 0 || data == NULL || data_size < 2) {
			free(cache_filename);
			return false;
		}

		/* De-obfuscate the JSON config */
		config_json_block = data + sizeof(uint16_t);
		for (unsigned int i = 0; i < data_size - sizeof(uint16_t); i++) {
			config_json_block[i] ^= wmr_config_key[i % sizeof(wmr_config_key)];
		}

		if (!wmr_controller_config_parse(&wcb->config, (char *)config_json_block, wcb->log_level)) {
			free(cache_filename);
			free(data);
			return false;
		}

		/* Write to the cache file (if it fails, ignore it, it's just a cache) */
		write_calibration_cache(wcb, cache_filename, config_json_block, data_size - sizeof(uint16_t));
		free(data);
	} else {
		WMR_DEBUG(wcb, "Read %s controller config from cache %s",
		          wcb->base.device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER ? "left" : "right",
		          cache_filename);
	}
	free(cache_filename);

	WMR_DEBUG(wcb, "Parsed %d LED entries from controller calibration", wcb->config.led_count);
	return true;
}

static xrt_result_t
wmr_controller_base_get_tracked_pose(struct xrt_device *xdev,
                                     enum xrt_input_name name,
                                     int64_t at_timestamp_ns,
                                     struct xrt_space_relation *out_relation)
{
	DRV_TRACE_MARKER();

	struct wmr_controller_base *wcb = wmr_controller_base(xdev);

	struct xrt_relation_chain xrc = {0};
	struct xrt_space_relation relation = {0};

	if (name == XRT_INPUT_G2_CONTROLLER_AIM_POSE || name == XRT_INPUT_ODYSSEY_CONTROLLER_AIM_POSE ||
		name == XRT_INPUT_WMR_AIM_POSE)
		m_relation_chain_push_pose(&xrc, &wcb->P_aim);

	if (name == XRT_INPUT_G2_CONTROLLER_GRIP_POSE || name == XRT_INPUT_ODYSSEY_CONTROLLER_GRIP_POSE ||
		name == XRT_INPUT_WMR_GRIP_POSE) {
		m_relation_chain_push_pose(&xrc, &wcb->P_aim);
	m_relation_chain_push_pose(&xrc, &wcb->P_aim_grip);
		}


#if 1
	kalman_fusion_get_prediction(wcb->kalman_fusion, at_timestamp_ns, &relation);

	m_relation_chain_push_relation(&xrc, &relation);
	m_relation_chain_resolve(&xrc, out_relation);

#else

	/* Apply the controller rotation */
	struct xrt_pose pose = {{0, 0, 0, 1}, {0, 1.2, -0.5}};
	if (xdev->device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		pose.position.x = -0.2;
	} else {
		pose.position.x = 0.2;
	}

	// Variables needed for prediction.
	int64_t last_imu_timestamp_ns = 0;
	relation.relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);

	// Start with the static pose above, then apply IMU + tracking
	relation.pose = pose;

	// Copy data while holding the lock.
	os_mutex_lock(&wcb->data_lock);
	relation.pose.orientation = wcb->fusion.rot;
	relation.angular_velocity = wcb->last_angular_velocity;
	last_imu_timestamp_ns = wcb->last_imu_timestamp_ns;

	if (wcb->last_tracked_pose_ts != 0) {
		relation.pose.position = wcb->last_tracked_pose.position;
	}
	os_mutex_unlock(&wcb->data_lock);

	m_relation_chain_push_relation(&xrc, &relation);
	m_relation_chain_resolve(&xrc, &relation);

	// No prediction needed.
	if (at_timestamp_ns < last_imu_timestamp_ns) {
		*out_relation = relation;
		return XRT_SUCCESS;
	}

	int64_t prediction_ns = at_timestamp_ns - last_imu_timestamp_ns;
	double prediction_s = time_ns_to_s(prediction_ns);

	m_predict_relation(&relation, prediction_s, out_relation);
#endif

	wcb->pose = out_relation->pose;
	return XRT_SUCCESS;
}

void
wmr_controller_base_deinit(struct wmr_controller_base *wcb)
{
	DRV_TRACE_MARKER();

	// Remove the variable tracking.
	u_var_remove_root(wcb);

	// Disconnect from the connection so we don't
	// receive any more callbacks
	os_mutex_lock(&wcb->conn_lock);
	struct wmr_controller_connection *conn = wcb->wcc;
	wcb->wcc = NULL;
	os_mutex_unlock(&wcb->conn_lock);

	if (conn != NULL) {
		wmr_controller_connection_disconnect(conn);
	}

	if (wcb->tracking_connection) {
		t_constellation_tracked_device_connection_disconnect(wcb->tracking_connection);
		wcb->tracking_connection = NULL;
	}

	m_clock_windowed_skew_tracker_destroy(wcb->hw2mono_clock);

	os_mutex_destroy(&wcb->conn_lock);
	os_mutex_destroy(&wcb->data_lock);

	// Destroy the fusion.
	m_imu_3dof_close(&wcb->fusion);

	if (wcb->kalman_fusion)
		kalman_fusion_destroy(wcb->kalman_fusion);
}

/*
 *
 * 'Exported' functions.
 *
 */

bool
wmr_controller_base_init(struct wmr_controller_base *wcb,
                         struct wmr_controller_connection *conn,
                         enum xrt_device_type controller_type,
                         enum u_logging_level log_level)
{
	DRV_TRACE_MARKER();

	wcb->log_level = log_level;
	wcb->wcc = conn;
	wcb->receive_bytes = receive_bytes;
	wcb->pose = (struct xrt_pose)XRT_POSE_IDENTITY;

	// IMU samples arrive every 5ms on average
	// 1 second seems to be enough to smooth things
	const int IMU_ARRIVAL_FREQ = 200;
	wcb->hw2mono_clock = m_clock_windowed_skew_tracker_alloc(IMU_ARRIVAL_FREQ);

	if (controller_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		snprintf(wcb->base.str, ARRAY_SIZE(wcb->base.str), "WMR Left Controller");
		/* TODO: use proper serial from read_controller_config()? */
		snprintf(wcb->base.serial, XRT_DEVICE_NAME_LEN, "Left Controller");
	} else {
		snprintf(wcb->base.str, ARRAY_SIZE(wcb->base.str), "WMR Right Controller");
		/* TODO: use proper serial from read_controller_config()? */
		snprintf(wcb->base.serial, XRT_DEVICE_NAME_LEN, "Right Controller");
	}

	wcb->base.get_tracked_pose = wmr_controller_base_get_tracked_pose;

	wcb->base.name = XRT_DEVICE_WMR_CONTROLLER;
	wcb->base.device_type = controller_type;
	wcb->base.orientation_tracking_supported = true;
	wcb->base.position_tracking_supported = true;
	wcb->base.hand_tracking_supported = false;

	/* Default grip pose up by 35° degrees around the X axis and
	 * back about 10cm (back is +Z in OXR coords), but overridden
	 * by subclasses with real values from controller models */
	struct xrt_vec3 translation = {0.0, 0, 0.1};
	struct xrt_vec3 axis = {1.0, 0, 0};
	math_quat_from_angle_vector(DEG_TO_RAD(35), &axis, &wcb->P_aim_grip.orientation);
	wcb->P_aim_grip.position = translation;

	wcb->thumbstick_deadzone = 0.15;

	m_imu_3dof_init(&wcb->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	wcb->kalman_fusion = kalman_fusion_create();

	if (os_mutex_init(&wcb->conn_lock) != 0 || os_mutex_init(&wcb->data_lock) != 0) {
		WMR_ERROR(wcb, "WMR Controller: Failed to init mutex!");
		return false;
	}

	/* Send init commands */
	struct wmr_controller_fw_cmd fw_cmd = {
	    0,
	};
	struct wmr_controller_fw_cmd_response fw_cmd_response;

	/* Zero command. Reinits controller internal state */
	fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x0, 0, 0);
	if (wmr_controller_send_fw_cmd(wcb, &fw_cmd, 0x06, &fw_cmd_response) < 0) {
		return false;
	}

	/* Quiesce/restart controller tasks */
	fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x04, 0xc1, 0x02);
	if (wmr_controller_send_fw_cmd(wcb, &fw_cmd, 0x06, &fw_cmd_response) < 0) {
		return false;
	}

	// Read config file from controller
	if (!read_controller_config(wcb)) {
		return false;
	}

	wmr_config_precompute_transforms(&wcb->config.sensors, NULL);

	// Announce the availabilty of the config for the tracker to retrieve
	os_mutex_lock(&wcb->data_lock);
	wcb->have_config = true;
	os_mutex_unlock(&wcb->data_lock);

	/* Reset device time before controller outputs */
	fw_cmd = WMR_CONTROLLER_FW_CMD_INIT(0x06, 0x21, 0x00, 0x00);
	if (wmr_controller_send_fw_cmd(wcb, &fw_cmd, 0x06, &fw_cmd_response) < 0) {
		return false;
	}

	/* Enable the status reports, IMU and control status reports */
	const unsigned char wmr_controller_status_enable_cmd[64] = {0x06, 0x03, 0x01, 0x00, 0x02};
	wmr_controller_send_bytes(wcb, wmr_controller_status_enable_cmd, sizeof(wmr_controller_status_enable_cmd));
	os_nanosleep(U_TIME_1MS_IN_NS * 20); // Sleep 20ms
	const unsigned char wmr_controller_imu_on_cmd[64] = {0x06, 0x03, 0x02, 0xe1, 0x02};
	wmr_controller_send_bytes(wcb, wmr_controller_imu_on_cmd, sizeof(wmr_controller_imu_on_cmd));

	wcb->update_yaw_from_optical = true;

	wcb->timesync_counter = 2;
	wcb->timesync_led_intensity = 200;
	wcb->timesync_val2 = 0;
	wcb->timesync_time_offset = 0;

	wcb->timesync_led_intensity_uvar =
	    (struct u_var_draggable_u16){.val = &wcb->timesync_led_intensity, .min = 1, .max = 399, .step = 1};
	wcb->timesync_val2_uvar =
	    (struct u_var_draggable_u16){.val = &wcb->timesync_val2, .min = 0, .max = 1023, .step = 1};
	wcb->timesync_time_offset_uvar =
	    (struct u_var_draggable_u16){.val = &wcb->timesync_time_offset, .min = 0, .max = 44, .step = 1};

	u_var_add_root(wcb, wcb->base.str, true);
	u_var_add_log_level(wcb, &wcb->log_level, "Log Level");
	u_var_add_pose(wcb, &wcb->pose, "Reported pose");

	u_var_add_gui_header(wcb, NULL, "IMU");
	u_var_add_ro_vec3_f32(wcb, &wcb->last_imu.acc, "imu.accel");
	u_var_add_ro_vec3_f32(wcb, &wcb->last_imu.gyro, "imu.gyro");
	u_var_add_ro_quat_f32(wcb, &wcb->fusion.rot, "fusion.rot");
	u_var_add_i32(wcb, &wcb->last_imu.temperature, "imu.temperature");
	u_var_add_ro_u64(wcb, &wcb->last_imu_timestamp_ns, "Last CPU IMU TS");
	u_var_add_ro_u64(wcb, &wcb->last_imu_device_timestamp_ns, "Last device IMU TS");

	u_var_add_gui_header(wcb, NULL, "Optical Tracking");
	u_var_add_pose(wcb, &wcb->last_tracked_pose, "Last observed pose");
	u_var_add_ro_i64(wcb, &wcb->last_tracked_pose_ts, "Last observed pose TS");
	u_var_add_bool(wcb, &wcb->update_yaw_from_optical, "Update yaw using tracking");

	u_var_add_gui_header(wcb, NULL, "Kalman Fusion");
	kalman_fusion_add_ui(wcb->kalman_fusion, wcb,
	                     (wcb->base.device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) ? "wmr_left"
	                                                                                     : "wmr_right");

	u_var_add_gui_header(wcb, NULL, "LED Sync");
	u_var_add_draggable_u16(wcb, &wcb->timesync_led_intensity_uvar, "LED intensity");
	u_var_add_draggable_u16(wcb, &wcb->timesync_val2_uvar, "U2");
	u_var_add_draggable_u16(wcb, &wcb->timesync_time_offset_uvar, "time offset (0.5ms increment)");
	u_var_add_ro_u64(wcb, &wcb->last_timesync_timestamp_ns, "Last CPU timesync TS");
	u_var_add_ro_u64(wcb, &wcb->last_timesync_device_timestamp_ns, "Last device timesync TS");

	u_var_add_gui_header(wcb, NULL, "Misc");
	u_var_add_pose(wcb, &wcb->P_aim_grip, "Grip pose offset");
	u_var_add_ro_u64(wcb, &wcb->next_keepalive_timestamp_ns, "Next keepalive TS");

	return true;
}

/*
 * Timesync packet format:
 *  XX    YY    AA    BB    CC    CC    CC    CC    CC    CC    DD    EE
 *   0     1     2     3     4     5     6     7     8     9    10    11
 *
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |0              |    1          |        2      |            3  |
 * |0 1 2 3 4 5 6 7|8 9 0 1 2 3 4 5|6 7 8 9 0 1 2 3|4 5 6 7 8 9 0 1|
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |       XX      |      YY       | C |  U1[0:5]  :[6:8]| TS[0:4] |  XX YY AA BB
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                           TS[5:36]                            |  CC CC CC CC
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |         TS[37:54]                 |   U2[0:10]          | F2  |  CC CC DD EE
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * XX = 0x3
 * YY = 8 bit counter that increments sequentially across timesync and keepalives
 * C  = 2 bit counter. Always starts as 1, then counts 1,2,3 per packet,
 *      regardless of the time between. Must not be 0
 * U1 = Led intensity / pulse length
 *       - clamped at 1..399 in setLEDPulseLengthMaybe()
 * TS = 55-bit sync timestamp (in µS) taken from camera exposure timings
 * U2 = unknown 11-bits
 *      - Gets multiplied by 2 and has 1000 added before use?
 *      - Looks like it gets extended to 64-bit and used in an unclear way
 *      - Looks like it should affect something in the HID status report packet
 *      - First WMR packet is always 800
 *      - Minimum value seen from WMR is 0, max (after the initial 800) is 1023
 * F  = flags? 3 bits. Allowed values seem to be 1,2,3,4
 *      - WMR always sends 1. Seems like 2 might be 'off'
 *      - Referred to as 'LED train type'
 */
static void
fill_timesync_packet(
    uint8_t buf[12], uint8_t cmd_ctr, uint8_t ts_ctr, int led_intensity, uint64_t ts, int U2, uint8_t flags)
{
	ts_ctr = (ts_ctr & 0x3);
	led_intensity = CLAMP(led_intensity, 1, 399);
	U2 = CLAMP(U2, 0, 1023);

	buf[0] = 0x3;
	buf[1] = cmd_ctr;
	buf[2] = ts_ctr | ((led_intensity & 0x3f) << 2);
	buf[3] = ((led_intensity >> 6) & 0x7) | ((ts & 0x1f) << 3);
	buf[4] = ts >> 5;
	buf[5] = ts >> 13;
	buf[6] = ts >> 21;
	buf[7] = ts >> 29;
	buf[8] = ts >> 37;
	buf[9] = ts >> 45;
	buf[10] = ((ts >> 53) & 0x3) | (U2 << 2);
	buf[11] = ((U2 >> 6) & 0x1f) | ((flags & 0x3) << 5);
}

/* Called with data_lock held */
static void
wmr_controller_base_send_timesync(struct wmr_controller_base *wcb)
{
	/* @todo: Check if timesync values have changed and skip sending if not */
	uint8_t timesync_pkt[12];

	os_mutex_lock(&wcb->conn_lock);
	struct wmr_controller_connection *conn = wcb->wcc;
	if (conn != NULL) {
		/* Each timesync_time_offset step is 0.5ms */
		uint64_t time_offset_us = wcb->timesync_time_offset * 500;

		/* Timesync counter counts 1/2/3 in a loop per packet */
		uint8_t ts_ctr = wcb->timesync_counter++;
		if (wcb->timesync_counter == 4) {
			wcb->timesync_counter = 1;
		}

		int led_intensity = wcb->timesync_led_intensity;
		uint64_t slam_time_us = wcb->timesync_device_slam_time_us + time_offset_us;
		int val2 = wcb->timesync_val2;

		fill_timesync_packet(timesync_pkt, wcb->cmd_counter++, ts_ctr, led_intensity, slam_time_us, val2, 1);
		os_mutex_unlock(&wcb->data_lock);
		wmr_controller_connection_send_bytes(conn, timesync_pkt, sizeof(timesync_pkt));

		os_mutex_unlock(&wcb->conn_lock);
		os_mutex_lock(&wcb->data_lock);

		WMR_DEBUG(
		    wcb, "%s controller timesync counter %u led_intensity %u time %" PRIu64 " offset %" PRIu64 " U2 %u",
		    wcb->base.device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER ? "Left" : "Right", ts_ctr,
		    wcb->timesync_led_intensity, slam_time_us, time_offset_us, val2);
		WMR_TRACE_HEX(wcb, timesync_pkt, sizeof(timesync_pkt));
	} else {
		os_mutex_unlock(&wcb->conn_lock);
	}
}

static void
wmr_controller_base_notify_frame(struct xrt_device *xdev, uint64_t frame_mono_ns, uint64_t frame_sequence)
{
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)xdev;

	os_mutex_lock(&wcb->data_lock);

	/* The frame cadence is SLAM/controller/controller. Only controller frames are received
	 * here. On the 2nd controller frame, we need to pass the estimate of the time of the next
	 * *1st controller frame* minus 1/3 of a frame duration to the controller timesync methods.
	 * That is 2/90 + 1/270 = 5/270 or 5/3rds of the average frame duration.
	 * @todo: Also calculate how many full 3-frame cycles have passed since the last
	 * '2nd Controller frame' to add to our estimate in case of scheduling delays.
	 * @todo: Move the 'next frame' estimate into the controller timesync code so it's
	 * calculated when the led control packet is actually about to be sent.
	 * Since only controller frames get passed to here, the 2nd sequential controller
	 * frame is the one right before the next SLAM frame */
	bool is_second_frame = (wcb->last_frame_sequence + 1) == frame_sequence;

	if (is_second_frame) {
		// on the 2nd sequential (controller) frame, update the controller timesync estimate of the next SLAM
		// frame's time
		timepoint_ns next_slam_mono_ns = (timepoint_ns)(frame_mono_ns) + U_TIME_1MS_IN_NS * 18;

		timepoint_ns next_device_slam_time_ns;
		if (m_clock_windowed_skew_tracker_to_remote(wcb->hw2mono_clock, next_slam_mono_ns,
		                                            &next_device_slam_time_ns)) {
			uint64_t next_device_slam_time_us = next_device_slam_time_ns / 1000;

			if (next_device_slam_time_us != wcb->timesync_device_slam_time_us) {
				wcb->timesync_device_slam_time_us = next_device_slam_time_us;
				wcb->timesync_updated = true;
			}

			wcb->last_timesync_device_timestamp_ns = next_device_slam_time_ns;
			wcb->last_timesync_timestamp_ns = next_slam_mono_ns;
		}
	}

	wcb->last_frame_timestamp = frame_mono_ns;
	wcb->last_frame_sequence = frame_sequence;

	os_mutex_unlock(&wcb->data_lock);
}

/* Called with data_lock held */
static void
wmr_controller_base_send_keepalive(struct wmr_controller_base *wcb, uint64_t now_ns)
{
	const uint64_t KEEPALIVE_DURATION = 125 * U_TIME_1MS_IN_NS;

	if (wcb->next_keepalive_timestamp_ns > now_ns) {
		/* Too soon to send the Keepalive */
		return;
	}


	os_mutex_lock(&wcb->conn_lock);
	struct wmr_controller_connection *conn = wcb->wcc;
	if (conn != NULL) {
		uint8_t keepalive_pkt[2];

		keepalive_pkt[0] = WMR_MOTION_CONTROLLER_KEEPALIVE;
		keepalive_pkt[1] = wcb->cmd_counter++;

		os_mutex_unlock(&wcb->data_lock);
		wmr_controller_connection_send_bytes(conn, keepalive_pkt, sizeof(keepalive_pkt));

		os_mutex_unlock(&wcb->conn_lock);
		os_mutex_lock(&wcb->data_lock);
	}

	/* Calculate the next timeout */
	if (wcb->next_keepalive_timestamp_ns == 0) {
		wcb->next_keepalive_timestamp_ns = now_ns + KEEPALIVE_DURATION;
	} else {
		wcb->next_keepalive_timestamp_ns += KEEPALIVE_DURATION;
	}
}

static bool
wmr_controller_base_get_led_model(struct xrt_device *xdev, struct t_constellation_led_model *led_model)
{
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)(xdev);

	os_mutex_lock(&wcb->data_lock);
	if (!wcb->have_config) {
		os_mutex_unlock(&wcb->data_lock);
		return false;
	}
	os_mutex_unlock(&wcb->data_lock);

	t_constellation_led_model_init((int)wcb->base.device_type, NULL, led_model, wcb->config.led_count);

	// Note: This LED model is in OpenCV/WMR coordinates with
	// XYZ = Right/Down/Forward
	for (int i = 0; i < wcb->config.led_count; i++) {
		struct t_constellation_led *led = led_model->leds + i;

		led->id = i;

		led->pos = wcb->config.leds[i].pos;
		led->dir = wcb->config.leds[i].norm;

		led->radius_mm = 3.5;
	}

	return true;
}

static void
wmr_controller_base_push_observed_pose(struct xrt_device *xdev, timepoint_ns frame_mono_ns, const struct xrt_pose *pose)
{
	struct wmr_controller_base *wcb = (struct wmr_controller_base *)(xdev);
	os_mutex_lock(&wcb->data_lock);

	wcb->last_tracked_pose_ts = frame_mono_ns;
	wcb->last_tracked_pose = *pose;

	struct xrt_pose_sample sample = {.pose = *pose, .timestamp_ns = frame_mono_ns};
	struct xrt_vec3 position_variance = {1.e-6, 1.e-6, 1.e-6};
	struct xrt_vec3 orientation_variance = {1.e-3, 1.e-5, 1.e-3};
	kalman_fusion_process_pose(wcb->kalman_fusion, &sample, &position_variance, &orientation_variance, 15);

	if (wcb->update_yaw_from_optical) {
#if 1
		// Apply 5% of observed orientation yaw to 3dof fusion
		// FIXME: Do better
		struct xrt_quat delta;
		math_quat_unrotate(&wcb->fusion.rot, &pose->orientation, &delta);
		delta.x = delta.z = 0.0; // We only want Yaw

		if (fabs(delta.y) > sin(DEG_TO_RAD(5)) / 2) {
			delta.y = sin(0.10 * asinf(delta.y)); // 10% correction
			math_quat_normalize(&delta);

			struct xrt_quat prev = wcb->fusion.rot;
			math_quat_rotate(&wcb->fusion.rot, &delta, &wcb->fusion.rot);

			if (wcb->log_level <= U_LOGGING_DEBUG) {
				struct xrt_quat post_delta;
				math_quat_unrotate(&wcb->fusion.rot, &pose->orientation, &post_delta);
				post_delta.x = post_delta.z = 0.0;  // We only want Yaw
				post_delta.y = 0.10 * post_delta.y; // 5%
				math_quat_normalize(&post_delta);

				WMR_DEBUG(wcb,
				          "Applying delta yaw rotation of %f degrees delta quat %f,%f,%f,%f from "
				          "%f,%f,%f,%f to "
				          "%f,%f,%f,%f. delta after correction: %f,%f,%f,%f",
				          RAD_TO_DEG(2 * asinf(delta.y)), delta.x, delta.y, delta.z, delta.w, prev.x,
				          prev.y, prev.z, prev.w, wcb->fusion.rot.x, wcb->fusion.rot.y,
				          wcb->fusion.rot.z, wcb->fusion.rot.w, post_delta.x, post_delta.y,
				          post_delta.z, post_delta.w);
			}
		} else {
			math_quat_normalize(&delta);

			WMR_DEBUG(wcb, "Applying full yaw correction of %f degrees. delta quat %f,%f,%f,%f",
			          RAD_TO_DEG(2 * asinf(delta.y)), delta.x, delta.y, delta.z, delta.w);
			math_quat_rotate(&wcb->fusion.rot, &delta, &wcb->fusion.rot);
		}
#else
		wcb->fusion.rot = pose->orientation;
#endif
	}

	os_mutex_unlock(&wcb->data_lock);
}

static struct t_constellation_tracked_device_callbacks tracking_callbacks = {
    .get_led_model = wmr_controller_base_get_led_model,
    .notify_frame_received = wmr_controller_base_notify_frame,
    .push_observed_pose = wmr_controller_base_push_observed_pose,
};

void
wmr_controller_attach_to_hmd(struct wmr_controller_base *wcb, struct wmr_hmd *hmd)
{
	/* Register the controller with the HMD for LED constellation tracking and LED sync timing updates */
	wcb->tracking_connection = wmr_hmd_add_tracked_controller(hmd, &wcb->base, &tracking_callbacks);
}
