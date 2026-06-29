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

#include <ares/board/init.h>

#include "devices.h"
#include "module_chassis.h"
#include "usb_control.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define CONTROL_POLL_TIMEOUT_MS 200

static bool chassis_ready;

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
	board_init();

	k_sleep(K_MSEC(2000));

	if (usb_control_init() != 0) {
		LOG_WRN("Steam Deck control unavailable, chassis waits for USB");
	}

	chassis_set_speed(chassis, 0.0f, 0.0f);
	chassis_set_gyro(chassis, 0.0f);
	chassis_set_static(chassis, true);
	chassis_set_enabled(chassis, true);
	chassis_ready = true;

	while (1) {
		k_sleep(K_MSEC(500));
	}

	return 0;
}
