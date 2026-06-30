/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/drivers/chassis.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <ares/board/init.h>

#define CONTROL_POLL_TIMEOUT_MS 200
#define CHASSIS_DEBUG_MOTION_SEQUENCE 1
#define DEBUG_STEP_MS 3000
#define DEBUG_STOP_MS 1000
#define DEBUG_MOVE_SPEED 0.35f
#define DEBUG_GYRO_SPEED 2.0f

#include "devices.h"
#if !CHASSIS_DEBUG_MOTION_SEQUENCE
#include "module_chassis.h"
#include "usb_control.h"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static bool chassis_ready;

#if CHASSIS_DEBUG_MOTION_SEQUENCE
struct debug_motion_step {
	const char *name;
	float x;
	float y;
	float gyro;
};

static const struct debug_motion_step debug_motion_steps[] = {
	{ "forward +Y", 0.0f, DEBUG_MOVE_SPEED, 0.0f },
	{ "backward -Y", 0.0f, -DEBUG_MOVE_SPEED, 0.0f },
	{ "right +X", DEBUG_MOVE_SPEED, 0.0f, 0.0f },
	{ "left -X", -DEBUG_MOVE_SPEED, 0.0f, 0.0f },
	{ "front-right +X+Y", DEBUG_MOVE_SPEED * 0.7071f, DEBUG_MOVE_SPEED * 0.7071f, 0.0f },
	{ "front-left -X+Y", -DEBUG_MOVE_SPEED * 0.7071f, DEBUG_MOVE_SPEED * 0.7071f, 0.0f },
	{ "back-right +X-Y", DEBUG_MOVE_SPEED * 0.7071f, -DEBUG_MOVE_SPEED * 0.7071f, 0.0f },
	{ "back-left -X-Y", -DEBUG_MOVE_SPEED * 0.7071f, -DEBUG_MOVE_SPEED * 0.7071f, 0.0f },
	{ "spin ccw +gyro", 0.0f, 0.0f, DEBUG_GYRO_SPEED },
	{ "spin cw -gyro", 0.0f, 0.0f, -DEBUG_GYRO_SPEED },
};

static void debug_stop_zero(void)
{
	chassis_set_speed(chassis, 0.0f, 0.0f);
	chassis_set_gyro(chassis, 0.0f);
	chassis_set_static(chassis, false);
}

static void run_debug_motion_sequence(void)
{
	LOG_INF("chassis debug motion sequence enabled");
	LOG_INF("move speed=%.2f m/s, gyro=%.2f rad/s, step=%d ms",
		(double)DEBUG_MOVE_SPEED, (double)DEBUG_GYRO_SPEED, DEBUG_STEP_MS);

	k_msleep(2000);

	for (size_t i = 0; i < ARRAY_SIZE(debug_motion_steps); i++) {
		const struct debug_motion_step *step = &debug_motion_steps[i];

		LOG_INF("debug step %u/%u: %s x=%.2f y=%.2f gyro=%.2f",
			(unsigned int)i + 1, (unsigned int)ARRAY_SIZE(debug_motion_steps),
			step->name, (double)step->x, (double)step->y, (double)step->gyro);

		chassis_set_static(chassis, false);
		chassis_set_speed(chassis, step->x, step->y);
		chassis_set_gyro(chassis, step->gyro);
		k_msleep(DEBUG_STEP_MS);

		debug_stop_zero();
		k_msleep(DEBUG_STOP_MS);
	}

	LOG_INF("chassis debug motion sequence done");
}
#endif

void console_feedback(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

#if !CHASSIS_DEBUG_MOTION_SEQUENCE
	struct team_usb_packet packet;

	memset(&packet, 0, sizeof(packet));
#endif

	while (1) {
		if (!chassis_ready) {
			k_msleep(5);
			continue;
		}

#if CHASSIS_DEBUG_MOTION_SEQUENCE
		run_debug_motion_sequence();
		while (1) {
			debug_stop_zero();
			k_msleep(1000);
		}
#else
		bool updated = usb_control_poll(&packet, K_MSEC(CONTROL_POLL_TIMEOUT_MS));

		module_chassis_update(&packet, updated);
#endif
	}
}

K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);

int main(void)
{
	board_init();

	k_sleep(K_MSEC(2000));

#if !CHASSIS_DEBUG_MOTION_SEQUENCE
	if (usb_control_init() != 0) {
		LOG_WRN("Steam Deck control unavailable, chassis waits for USB");
	}
#else
	LOG_WRN("USB control disabled by chassis debug motion sequence");
#endif

	chassis_set_speed(chassis, 0.0f, 0.0f);
	chassis_set_gyro(chassis, 0.0f);
	chassis_set_static(chassis, false);
	chassis_set_enabled(chassis, true);
	chassis_ready = true;

	while (1) {
		k_sleep(K_MSEC(500));
	}

	return 0;
}
