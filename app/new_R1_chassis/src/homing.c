/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "homing.h"

LOG_MODULE_REGISTER(homing, LOG_LEVEL_INF);

/* Speeds / timings ------------------------------------------------------- */
#define HOMING_EXIT_SPEED_RPM -25.0f    /* reverse out of the photogate          */
#define HOMING_SEARCH_SPEED_RPM 60.0f   /* fast search for the photogate         */
#define HOMING_PRECISE_SPEED_RPM 5.0f   /* slow precise cut-in                   */
#define HOMING_TIMEOUT_MS 8000          /* per-phase timeout                     */
#define HOMING_DWELL_MS 150             /* keep moving after the edge clears     */
#define HOMING_DEBOUNCE_MS 2            /* photogate debounce on precise cut-in  */
#define HOMING_STOP_SETTLE_MS 50        /* let the motor stop before zeroing     */
#define HOMING_ZERO_SETTLE_MS 120       /* let SET_ZERO propagate before angle 0 */
#define HOMING_LOOP_MS 2                /* state-machine tick period             */

/* Devicetree references (kept local so this file does NOT pull in devices.h,
 * whose file-scope `const struct device *` definitions would otherwise be
 * duplicated and break the link). */
static const struct gpio_dt_spec limit0 = GPIO_DT_SPEC_GET(DT_NODELABEL(limit_0), gpios);
static const struct gpio_dt_spec limit1 = GPIO_DT_SPEC_GET(DT_NODELABEL(limit_1), gpios);
static const struct gpio_dt_spec limit2 = GPIO_DT_SPEC_GET(DT_NODELABEL(limit_2), gpios);

static const struct gpio_dt_spec *limit_gpios[] = {&limit0, &limit1, &limit2};

static int configure_limit_gpios(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(limit_gpios); i++) {
		const struct gpio_dt_spec *limit = limit_gpios[i];

		if (!device_is_ready(limit->port)) {
			LOG_ERR("limit%u gpio port is not ready", (unsigned int)i + 1);
			return -ENODEV;
		}

		int ret = gpio_pin_configure_dt(limit, GPIO_INPUT);
		if (ret != 0) {
			LOG_ERR("limit%u gpio configure failed: %d", (unsigned int)i + 1, ret);
			return ret;
		}
	}

	return 0;
}

enum home_phase {
	H_PREPARE,  /* enable, decide initial direction                 */
	H_BACKOUT,  /* started inside the limit: reverse until it clears */
	H_SEARCH,   /* drive forward until the limit triggers            */
	H_EXIT,     /* reverse until the limit clears again              */
	H_PRECISE,  /* slow forward until the limit triggers (debounced) */
	H_SETTLE,   /* stop, SET_ZERO, set_angle(0)                      */
	H_DONE,
	H_FAIL,
};

struct steer_unit {
	const struct device *steer_motor;
	const struct gpio_dt_spec *limit;
	const char *name;
	/* runtime homing state */
	enum home_phase phase;
	int64_t phase_start;
	int64_t dwell_until;
	bool zero_sent;
};

static struct steer_unit steer_units[] = {
	{DEVICE_DT_GET(DT_NODELABEL(motor_steer0)), &limit0, "steer1"},
	{DEVICE_DT_GET(DT_NODELABEL(motor_steer1)), &limit1, "steer2"},
	{DEVICE_DT_GET(DT_NODELABEL(motor_steer2)), &limit2, "steer3"},
};

static void stop_steer_units(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(steer_units); i++) {
		if (device_is_ready(steer_units[i].steer_motor)) {
			motor_set_speed(steer_units[i].steer_motor, 0.0f);
		}
	}
}

static void home_enter(struct steer_unit *u, enum home_phase phase, int64_t now)
{
	u->phase = phase;
	u->phase_start = now;
	u->dwell_until = 0;
}

static void home_fail(struct steer_unit *u)
{
	motor_set_speed(u->steer_motor, 0.0f);
	u->phase = H_FAIL;
	LOG_ERR("%s homing timeout", u->name);
}

static bool phase_is_moving(enum home_phase p)
{
	return p == H_BACKOUT || p == H_SEARCH || p == H_EXIT || p == H_PRECISE;
}

static void home_tick(struct steer_unit *u, int64_t now)
{
	if (u->phase == H_DONE || u->phase == H_FAIL) {
		return;
	}

	/* one timeout rule for every "drive until the switch changes" phase */
	if (phase_is_moving(u->phase) && (now - u->phase_start > HOMING_TIMEOUT_MS)) {
		home_fail(u);
		return;
	}

	int at_limit = gpio_pin_get_dt(u->limit); /* 1 == on the photogate */

	switch (u->phase) {
	case H_PREPARE:
		motor_control(u->steer_motor, ENABLE_MOTOR);
		if (at_limit == 1) {
			LOG_WRN("%s starts inside limit, backing out first", u->name);
			motor_set_speed(u->steer_motor, HOMING_EXIT_SPEED_RPM);
			home_enter(u, H_BACKOUT, now);
		} else {
			motor_set_speed(u->steer_motor, HOMING_SEARCH_SPEED_RPM);
			home_enter(u, H_SEARCH, now);
		}
		break;

	case H_BACKOUT:
		if (at_limit == 0) {
			if (u->dwell_until == 0) {
				u->dwell_until = now + HOMING_DWELL_MS;
			} else if (now >= u->dwell_until) {
				motor_set_speed(u->steer_motor, HOMING_SEARCH_SPEED_RPM);
				home_enter(u, H_SEARCH, now);
			}
		} else {
			u->dwell_until = 0;
		}
		break;

	case H_SEARCH:
		if (at_limit == 1) {
			motor_set_speed(u->steer_motor, HOMING_EXIT_SPEED_RPM);
			home_enter(u, H_EXIT, now);
		}
		break;

	case H_EXIT:
		if (at_limit == 0) {
			if (u->dwell_until == 0) {
				u->dwell_until = now + HOMING_DWELL_MS;
			} else if (now >= u->dwell_until) {
				motor_set_speed(u->steer_motor, HOMING_PRECISE_SPEED_RPM);
				home_enter(u, H_PRECISE, now);
			}
		} else {
			u->dwell_until = 0;
		}
		break;

	case H_PRECISE:
		if (at_limit == 1) {
			if (u->dwell_until == 0) {
				u->dwell_until = now + HOMING_DEBOUNCE_MS;
			} else if (now >= u->dwell_until) {
				motor_set_speed(u->steer_motor, 0.0f);
				home_enter(u, H_SETTLE, now);
				u->dwell_until = now + HOMING_STOP_SETTLE_MS;
				u->zero_sent = false;
			}
		} else {
			u->dwell_until = 0; /* bounced off: keep cutting in */
		}
		break;

	case H_SETTLE:
		if (now >= u->dwell_until) {
			if (!u->zero_sent) {
				motor_control(u->steer_motor, SET_ZERO);
				u->zero_sent = true;
				u->dwell_until = now + HOMING_ZERO_SETTLE_MS;
			} else {
				motor_set_angle(u->steer_motor, 0.0f);
				u->phase = H_DONE;
				LOG_INF("%s homing done", u->name);
			}
		}
		break;

	default:
		break;
	}
}

int homing_steer(void)
{
	int64_t now = k_uptime_get();

	if (configure_limit_gpios() != 0) {
		return -ENODEV;
	}

	for (size_t i = 0; i < ARRAY_SIZE(steer_units); i++) {
		struct steer_unit *u = &steer_units[i];

		if (!device_is_ready(u->steer_motor)) {
			LOG_ERR("%s motor is not ready", u->name);
			return -ENODEV;
		}

		u->phase = H_PREPARE;
		u->phase_start = now;
		u->dwell_until = 0;
		u->zero_sent = false;
	}

	LOG_INF("homing 3 steer units in parallel");

	while (1) {
		now = k_uptime_get();
		bool all_done = true;

		for (size_t i = 0; i < ARRAY_SIZE(steer_units); i++) {
			home_tick(&steer_units[i], now);

			enum home_phase p = steer_units[i].phase;

			if (p == H_FAIL) {
				stop_steer_units();
				return -EIO;
			}
			if (p != H_DONE) {
				all_done = false;
			}
		}

		if (all_done) {
			break;
		}

		k_msleep(HOMING_LOOP_MS);
	}

	LOG_INF("all steer units homed");
	return 0;
}