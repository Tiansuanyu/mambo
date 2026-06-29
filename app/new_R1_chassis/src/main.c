/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <ares/board/init.h>

#include "devices.h"
#include "homing.h"
#include "module_chassis.h"
#include "usb_control.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define CONTROL_POLL_TIMEOUT_MS 200
#define DM_INIT_STEP_DELAY_MS 100
#define FRICTION_TUNE_ENABLED 0
#define FRICTION_TUNE_TORQUE_START 0.02f
#define FRICTION_TUNE_COARSE_STEP 0.04f
#define FRICTION_TUNE_FINE_STEP 0.01f
#define FRICTION_TUNE_TORQUE_MAX 0.30f
#define FRICTION_TUNE_FIRST_MOVE_RPM 1.0f
#define FRICTION_TUNE_STABLE_AVG_RPM 0.7f
#define FRICTION_TUNE_COARSE_MS 350
#define FRICTION_TUNE_FINE_MS 700
#define FRICTION_TUNE_VERIFY_MS 1500
#define FRICTION_TUNE_SAMPLE_MS 50
#define FRICTION_TUNE_SETTLE_MS 250
#define FRICTION_TUNE_FINAL_STOP_MS 1000
#define FRICTION_TUNE_REPEAT_COUNT 3
#define FRICTION_TUNE_FF_LOW_SCALE 0.60f
#define FRICTION_TUNE_FF_HIGH_SCALE 0.75f

static const struct device *wheel_motors[] = {
	DEVICE_DT_GET(WHEELMOTOR1_NODE),
	DEVICE_DT_GET(WHEELMOTOR2_NODE),
	DEVICE_DT_GET(WHEELMOTOR3_NODE),
};

#if FRICTION_TUNE_ENABLED
static const struct device *steer_motors[] = {
	DEVICE_DT_GET(STEERMOTOR1_NODE),
	DEVICE_DT_GET(STEERMOTOR2_NODE),
	DEVICE_DT_GET(STEERMOTOR3_NODE),
};
#endif

static bool chassis_ready;

extern const k_tid_t chassis_thread;

#if FRICTION_TUNE_ENABLED
struct friction_tune_measure {
	float max_abs_rpm;
	float avg_abs_rpm;
};

struct friction_tune_result {
	float first_move;
	float stable_move;
	bool first_found;
	bool stable_found;
};
#endif

static int init_wheel_motors(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(wheel_motors); i++) {
		if (!device_is_ready(wheel_motors[i])) {
			LOG_ERR("wheel%u motor is not ready", (unsigned int)i + 1);
			return -ENODEV;
		}

		motor_control(wheel_motors[i], ENABLE_MOTOR);
		k_msleep(DM_INIT_STEP_DELAY_MS);
		motor_set_mode(wheel_motors[i], VO);
		k_msleep(DM_INIT_STEP_DELAY_MS);
		motor_set_vo(wheel_motors[i], 0.0f);
		k_msleep(DM_INIT_STEP_DELAY_MS);
		motor_control(wheel_motors[i], ENABLE_MOTOR);
		k_msleep(DM_INIT_STEP_DELAY_MS);
	}

	return 0;
}

#if FRICTION_TUNE_ENABLED
static struct friction_tune_measure friction_tune_apply_torque(const struct device *motor,
							       float torque, int hold_ms)
{
	motor_status_t status;
	struct friction_tune_measure measure = {0};
	float sum_abs_rpm = 0.0f;
	unsigned int samples = 0;
	int64_t end_time = k_uptime_get() + hold_ms;

	motor_set_torque(motor, torque);

	while (k_uptime_get() < end_time) {
		float abs_rpm;

		k_msleep(FRICTION_TUNE_SAMPLE_MS);
		if (motor_get(motor, &status) == 0) {
			abs_rpm = fabsf(status.rpm);
			measure.max_abs_rpm = MAX(measure.max_abs_rpm, abs_rpm);
			sum_abs_rpm += abs_rpm;
			samples++;
		}
	}

	motor_set_torque(motor, 0.0f);
	if (samples > 0U) {
		measure.avg_abs_rpm = sum_abs_rpm / samples;
	}

	return measure;
}

static float friction_tune_median(float values[], size_t count)
{
	for (size_t i = 0; i < count; i++) {
		for (size_t j = i + 1; j < count; j++) {
			if (values[j] < values[i]) {
				float tmp = values[i];

				values[i] = values[j];
				values[j] = tmp;
			}
		}
	}

	if (count == 0U) {
		return 0.0f;
	}

	if ((count % 2U) == 0U) {
		return (values[count / 2U - 1U] + values[count / 2U]) * 0.5f;
	}

	return values[count / 2U];
}

static struct friction_tune_result friction_tune_steer_direction_once(const struct device *motor,
								      size_t index,
								      float direction,
								      unsigned int run)
{
	const char *direction_name = direction > 0.0f ? "positive" : "negative";
	struct friction_tune_result result = {0};
	float coarse_hit = 0.0f;
	float fine_start;
	float fine_end;

	LOG_INF("friction tune steer%u %s run%u begin",
		(unsigned int)index + 1, direction_name, run);

	for (float torque = FRICTION_TUNE_TORQUE_START;
	     torque <= FRICTION_TUNE_TORQUE_MAX + 0.0001f;
	     torque += FRICTION_TUNE_COARSE_STEP) {
		float cmd_torque = direction * torque;
		struct friction_tune_measure measure;

		LOG_INF("friction tune steer%u %s coarse torque=%.3f Nm",
			(unsigned int)index + 1, direction_name, (double)cmd_torque);

		measure = friction_tune_apply_torque(motor, cmd_torque, FRICTION_TUNE_COARSE_MS);

		LOG_INF("friction tune steer%u %s coarse result torque=%.3f Nm max=%.2f avg=%.2f",
			(unsigned int)index + 1, direction_name, (double)cmd_torque,
			(double)measure.max_abs_rpm, (double)measure.avg_abs_rpm);

		k_msleep(FRICTION_TUNE_SETTLE_MS);

		if (measure.max_abs_rpm >= FRICTION_TUNE_FIRST_MOVE_RPM) {
			coarse_hit = torque;
			break;
		}
	}

	if (coarse_hit == 0.0f) {
		LOG_WRN("friction tune steer%u %s summary run=%u first_move=NOT_FOUND stable_move=NOT_FOUND limit=%.3f Nm",
			(unsigned int)index + 1, direction_name, run,
			(double)(direction * FRICTION_TUNE_TORQUE_MAX));
		goto stop;
	}

	fine_start = MAX(FRICTION_TUNE_TORQUE_START, coarse_hit - FRICTION_TUNE_COARSE_STEP);
	fine_end = coarse_hit;

	for (float torque = fine_start; torque <= fine_end + 0.0001f;
	     torque += FRICTION_TUNE_FINE_STEP) {
		float cmd_torque = direction * torque;
		struct friction_tune_measure measure;

		LOG_INF("friction tune steer%u %s fine torque=%.3f Nm",
			(unsigned int)index + 1, direction_name, (double)cmd_torque);

		measure = friction_tune_apply_torque(motor, cmd_torque, FRICTION_TUNE_FINE_MS);

		LOG_INF("friction tune steer%u %s fine result torque=%.3f Nm max=%.2f avg=%.2f",
			(unsigned int)index + 1, direction_name, (double)cmd_torque,
			(double)measure.max_abs_rpm, (double)measure.avg_abs_rpm);

		k_msleep(FRICTION_TUNE_SETTLE_MS);

		if (measure.max_abs_rpm >= FRICTION_TUNE_FIRST_MOVE_RPM) {
			result.first_move = torque;
			result.first_found = true;
			break;
		}
	}

	if (!result.first_found) {
		result.first_move = coarse_hit;
		result.first_found = true;
	}

	for (float torque = result.first_move;
	     torque <= MIN(FRICTION_TUNE_TORQUE_MAX,
			   result.first_move + FRICTION_TUNE_COARSE_STEP) +
			       0.0001f;
	     torque += FRICTION_TUNE_FINE_STEP) {
		float cmd_torque = direction * torque;
		struct friction_tune_measure measure;

		LOG_INF("friction tune steer%u %s verify torque=%.3f Nm",
			(unsigned int)index + 1, direction_name, (double)cmd_torque);

		measure = friction_tune_apply_torque(motor, cmd_torque, FRICTION_TUNE_VERIFY_MS);

		LOG_INF("friction tune steer%u %s verify result torque=%.3f Nm max=%.2f avg=%.2f",
			(unsigned int)index + 1, direction_name, (double)cmd_torque,
			(double)measure.max_abs_rpm, (double)measure.avg_abs_rpm);

		k_msleep(FRICTION_TUNE_SETTLE_MS);

		if (measure.avg_abs_rpm >= FRICTION_TUNE_STABLE_AVG_RPM) {
			result.stable_move = torque;
			result.stable_found = true;
			break;
		}
	}

	if (result.stable_found) {
		LOG_INF("friction tune steer%u %s summary run=%u first_move=%.3f Nm stable_move=%.3f Nm",
			(unsigned int)index + 1, direction_name, run,
			(double)(direction * result.first_move),
			(double)(direction * result.stable_move));
	} else {
		LOG_INF("friction tune steer%u %s summary run=%u first_move=%.3f Nm stable_move=NOT_FOUND",
			(unsigned int)index + 1, direction_name, run,
			(double)(direction * result.first_move));
	}

stop:
	motor_set_torque(motor, 0.0f);
	k_msleep(FRICTION_TUNE_FINAL_STOP_MS);
	return result;
}

static void friction_tune_steer_direction(const struct device *motor, size_t index,
					  float direction)
{
	const char *direction_name = direction > 0.0f ? "positive" : "negative";
	float stable_values[FRICTION_TUNE_REPEAT_COUNT];
	size_t valid_count = 0;
	float stable_min = 0.0f;
	float stable_max = 0.0f;

	for (unsigned int run = 1U; run <= FRICTION_TUNE_REPEAT_COUNT; run++) {
		struct friction_tune_result result =
			friction_tune_steer_direction_once(motor, index, direction, run);

		if (result.stable_found) {
			stable_values[valid_count] = result.stable_move;
			if (valid_count == 0U || result.stable_move < stable_min) {
				stable_min = result.stable_move;
			}
			if (valid_count == 0U || result.stable_move > stable_max) {
				stable_max = result.stable_move;
			}
			valid_count++;
		}
	}

	if (valid_count == 0U) {
		LOG_WRN("friction tune steer%u %s FINAL valid=0 stable_median=NOT_FOUND ff_init=NOT_FOUND",
			(unsigned int)index + 1, direction_name);
		return;
	}

	float median = friction_tune_median(stable_values, valid_count);
	float signed_median = direction * median;
	float signed_ff_low = direction * median * FRICTION_TUNE_FF_LOW_SCALE;
	float signed_ff_high = direction * median * FRICTION_TUNE_FF_HIGH_SCALE;

	LOG_INF("friction tune steer%u %s FINAL valid=%u stable_median=%.3f Nm stable_spread=%.3f Nm ff_init_60=%.3f Nm ff_init_75=%.3f Nm",
		(unsigned int)index + 1, direction_name, (unsigned int)valid_count,
		(double)signed_median, (double)(stable_max - stable_min),
		(double)signed_ff_low, (double)signed_ff_high);
}

static void friction_tune_steer_motors(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(steer_motors); i++) {
		if (!device_is_ready(steer_motors[i])) {
			LOG_ERR("steer%u motor is not ready", (unsigned int)i + 1);
			continue;
		}

		friction_tune_steer_direction(steer_motors[i], i, 1.0f);
		friction_tune_steer_direction(steer_motors[i], i, -1.0f);
	}
}
#endif

void console_feedback(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	struct team_usb_packet packet;

	memset(&packet, 0, sizeof(packet));

	while (1) {
		if (!chassis_ready) {
			k_msleep(5);
			continue;
		}

		bool updated = usb_control_poll(&packet, K_MSEC(CONTROL_POLL_TIMEOUT_MS));

		module_chassis_update(&packet, updated);
	}
}

K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);

int main(void)
{
	k_thread_suspend(chassis_thread);

	board_init();

	k_sleep(K_MSEC(2000));

	if (usb_control_init() != 0) {
		LOG_WRN("Steam Deck control unavailable, chassis waits for USB");
	}

	if (homing_steer() != 0) {
		LOG_ERR("steer homing failed, chassis remains disabled");
		while (1) {
			k_sleep(K_MSEC(1000));
		}
	}

#if FRICTION_TUNE_ENABLED
	LOG_INF("friction tune mode enabled; chassis control thread remains disabled");
	friction_tune_steer_motors();
	LOG_INF("friction tune done; reset or flash normal firmware before driving");
	while (1) {
		k_sleep(K_MSEC(1000));
	}
#endif

	if (init_wheel_motors() != 0) {
		LOG_ERR("wheel motor init failed, chassis remains disabled");
		while (1) {
			k_sleep(K_MSEC(1000));
		}
	}

	k_sleep(K_MSEC(100));
	chassis_set_speed(chassis, 0.0f, 0.0f);
	chassis_set_gyro(chassis, 0.0f);
	chassis_set_static(chassis, true);
	chassis_set_enabled(chassis, true);
	k_thread_resume(chassis_thread);
	chassis_ready = true;

	while (1) {
		k_sleep(K_MSEC(500));
	}

	return 0;
}
