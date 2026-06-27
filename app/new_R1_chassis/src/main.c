/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <ares/board/init.h>
#include "devices.h"
#include "homing.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* remote control / gears */
#define STICK_DEADZONE 0.06f
#define GEAR_SWITCH_DOWN_THRESHOLD -0.5f
#define GEAR_SWITCH_RELEASE_THRESHOLD -0.2f
#define HIGH_LINEAR_SPEED_SCALE 1.5f
#define HIGH_GYRO_SPEED_SCALE 15.0f
#define LOW_LINEAR_SPEED_SCALE 0.5f
#define LOW_GYRO_SPEED_SCALE 7.5f

static const struct device *wheel_motors[] = {
	DEVICE_DT_GET(WHEELMOTOR1_NODE),
	DEVICE_DT_GET(WHEELMOTOR2_NODE),
	DEVICE_DT_GET(WHEELMOTOR3_NODE),
};

static bool chassis_ready;

extern const k_tid_t chassis_thread;

static float clamp_unit(float value)
{
	if (!isfinite(value)) {
		return 0.0f;
	}
	if (value > 1.0f) {
		return 1.0f;
	}
	if (value < -1.0f) {
		return -1.0f;
	}
	return value;
}

static int init_wheel_motors(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(wheel_motors); i++) {
		if (!device_is_ready(wheel_motors[i])) {
			LOG_ERR("wheel%u motor is not ready", (unsigned int)i + 1);
			return -ENODEV;
		}

		motor_control(wheel_motors[i], ENABLE_MOTOR);
		motor_set_mode(wheel_motors[i], VO);
		motor_set_vo(wheel_motors[i], 0.0f);
		k_msleep(50);
	}

	return 0;
}

void console_feedback(void *arg1, void *arg2, void *arg3)
{
	bool low_gear = false;
	bool gear_switch_armed = true;

	while (1) {
		k_msleep(5);

		if (!chassis_ready) {
			continue;
		}

			float angvel = 0.0f;
			float X = 0.0f;
			float Y = 0.0f;
			float gear_switch = clamp_unit(sbus_get_percent(sbus, 2));
			bool gear_switch_active = gear_switch < GEAR_SWITCH_RELEASE_THRESHOLD;

			angvel = -clamp_unit(sbus_get_percent(sbus, 0));
			X = clamp_unit(sbus_get_percent(sbus, 3));
			Y = clamp_unit(sbus_get_percent(sbus, 1));

			if (gear_switch < GEAR_SWITCH_DOWN_THRESHOLD) {
				if (gear_switch_armed) {
					low_gear = !low_gear;
					gear_switch_armed = false;
					LOG_INF("chassis gear: %s", low_gear ? "low" : "high");
				}
			} else if (gear_switch > GEAR_SWITCH_RELEASE_THRESHOLD) {
				gear_switch_armed = true;
			}

		float linear_speed_scale = low_gear ? LOW_LINEAR_SPEED_SCALE : HIGH_LINEAR_SPEED_SCALE;
		float gyro_speed_scale = low_gear ? LOW_GYRO_SPEED_SCALE : HIGH_GYRO_SPEED_SCALE;

		float linear_magnitude = sqrtf(X * X + Y * Y);

		if (linear_magnitude < STICK_DEADZONE) {
			X = 0.0f;
			Y = 0.0f;
		}

			if (gear_switch_active || fabsf(angvel) < STICK_DEADZONE) {
				angvel = 0.0f;
			}

		bool in_deadzone = ((X == 0.0f) && (Y == 0.0f) && (angvel == 0.0f));

		if (in_deadzone) {
			chassis_set_speed(chassis, 0.0f, 0.0f);
			chassis_set_gyro(chassis, 0.0f);
		} else {
			chassis_set_static(chassis, false);
			chassis_set_speed(chassis, -X * linear_speed_scale, Y * linear_speed_scale);
			chassis_set_gyro(chassis, angvel * gyro_speed_scale);
		}
	}
}

K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);

int main(void)
{
	k_thread_suspend(chassis_thread);

	board_init();

	k_sleep(K_MSEC(2000));

	if (homing_steer() != 0) {
		LOG_ERR("steer homing failed, chassis remains disabled");
		while (1) {
			k_sleep(K_MSEC(1000));
		}
	}

	if (init_wheel_motors() != 0) {
		LOG_ERR("wheel motor init failed, chassis remains disabled");
		while (1) {
			k_sleep(K_MSEC(1000));
		}
	}

	k_sleep(K_MSEC(100));
	chassis_set_speed(chassis, 0.0f, 0.0f);
	chassis_set_gyro(chassis, 0.0f);
	chassis_set_enabled(chassis, true);
	k_thread_resume(chassis_thread);
	chassis_ready = true;

	while (1) {
		k_sleep(K_MSEC(500));
	}

	return 0;
}
