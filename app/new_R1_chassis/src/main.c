/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
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

static const struct device *wheel_motors[] = {
	DEVICE_DT_GET(WHEELMOTOR1_NODE),
	DEVICE_DT_GET(WHEELMOTOR2_NODE),
	DEVICE_DT_GET(WHEELMOTOR3_NODE),
};

static bool chassis_ready;

extern const k_tid_t chassis_thread;

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
