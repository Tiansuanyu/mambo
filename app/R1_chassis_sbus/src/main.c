/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/drivers/chassis.h>
#include <ares/board/init.h>
#include "devices.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define STICK_DEADZONE 0.06f
#define GEAR_SWITCH_DOWN_THRESHOLD -0.5f
#define GEAR_SWITCH_RELEASE_THRESHOLD -0.2f
#define HIGH_LINEAR_SPEED_SCALE 1.5f
#define HIGH_GYRO_SPEED_SCALE 15.0f
#define LOW_LINEAR_SPEED_SCALE 0.5f
#define LOW_GYRO_SPEED_SCALE 7.5f

void console_feedback(void *arg1, void *arg2, void *arg3)
{
    bool low_gear = true;
    bool gear_switch_armed = true;

    while (1) {
        k_msleep(5);

        float angvel = -sbus_get_percent(sbus, 0);
        float X = sbus_get_percent(sbus, 3);
        float Y = sbus_get_percent(sbus, 1);
        float gear_switch = sbus_get_percent(sbus, 2);
        bool gear_switch_active = gear_switch < GEAR_SWITCH_RELEASE_THRESHOLD;

        if (gear_switch < GEAR_SWITCH_DOWN_THRESHOLD) {
            if (gear_switch_armed) {
                low_gear = !low_gear;
                gear_switch_armed = false;
            }
        } else if (gear_switch > GEAR_SWITCH_RELEASE_THRESHOLD) {
            gear_switch_armed = true;
        }

        float linear_speed_scale = low_gear ? LOW_LINEAR_SPEED_SCALE : HIGH_LINEAR_SPEED_SCALE;
        float gyro_speed_scale = low_gear ? LOW_GYRO_SPEED_SCALE : HIGH_GYRO_SPEED_SCALE;

        float linear_magnitude = sqrtf(X * X + Y * Y);

        if (linear_magnitude < STICK_DEADZONE) {
            X = 0;
            Y = 0;
        }

        if (gear_switch_active || fabsf(angvel) < STICK_DEADZONE) {
            angvel = 0;
        }

        bool in_deadzone = ((X == 0) && (Y == 0) && (angvel == 0));

        if (in_deadzone) {
            chassis_set_static(chassis, true);
            chassis_set_speed(chassis, 0.0f, 0.0f); 
            chassis_set_gyro(chassis, 0.0f); 
        } else {
            chassis_set_static(chassis, false);
            chassis_set_speed(chassis, X * linear_speed_scale, Y * linear_speed_scale); 
            chassis_set_gyro(chassis, -angvel * gyro_speed_scale);     
        }
    }
}

K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);

int main(void)
{
    k_sleep(K_MSEC(2000));
    chassis_set_enabled(chassis, false);
    k_sleep(K_MSEC(100)); 
    chassis_set_enabled(chassis, true);
    chassis_set_gyro(chassis, 0);

    while (1) {
        k_sleep(K_MSEC(500));
    }

    return 0;
}
