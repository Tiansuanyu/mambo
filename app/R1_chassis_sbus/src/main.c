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
#define BRAKE_LINEAR_DECEL_MPS2 5.0f
#define BRAKE_GYRO_DECEL_RADPS2 50.0f
#define DEFAULT_CONTROL_DT_SEC 0.005f
#define MAX_CONTROL_DT_SEC 0.05f
#define STOP_LINEAR_EPS 0.02f
#define STOP_GYRO_EPS 0.05f

static float control_dt_seconds(int64_t now_ms, int64_t *last_ms)
{
    if (*last_ms == 0) {
        *last_ms = now_ms;
        return DEFAULT_CONTROL_DT_SEC;
    }

    int64_t elapsed_ms = now_ms - *last_ms;
    *last_ms = now_ms;

    if (elapsed_ms <= 0) {
        return DEFAULT_CONTROL_DT_SEC;
    }

    float dt = (float)elapsed_ms / 1000.0f;

    if (dt > MAX_CONTROL_DT_SEC) {
        return MAX_CONTROL_DT_SEC;
    }

    return dt;
}

static float slew_toward(float current, float target, float max_delta)
{
    float delta = target - current;

    if (delta > max_delta) {
        return current + max_delta;
    }

    if (delta < -max_delta) {
        return current - max_delta;
    }

    return target;
}

static bool chassis_command_stopped(float speed_x, float speed_y, float gyro)
{
    return fabsf(speed_x) < STOP_LINEAR_EPS && fabsf(speed_y) < STOP_LINEAR_EPS &&
           fabsf(gyro) < STOP_GYRO_EPS;
}

static void enable_chassis_motors(void)
{
    motor_control(steer_motor1, ENABLE_MOTOR);
    motor_control(steer_motor2, ENABLE_MOTOR);
    motor_control(steer_motor3, ENABLE_MOTOR);
    motor_control(wheel_motor1, ENABLE_MOTOR);
    motor_control(wheel_motor2, ENABLE_MOTOR);
    motor_control(wheel_motor3, ENABLE_MOTOR);
}

void console_feedback(void *arg1, void *arg2, void *arg3)
{
    bool low_gear = true;
    bool gear_switch_armed = true;
    int64_t last_control_ms = 0;
    float cmd_speed_x = 0.0f;
    float cmd_speed_y = 0.0f;
    float cmd_gyro = 0.0f;

    while (1) {
        k_msleep(5);

        int64_t now_ms = k_uptime_get();
        float dt_sec = control_dt_seconds(now_ms, &last_control_ms);
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
            cmd_speed_x = slew_toward(cmd_speed_x, 0.0f, BRAKE_LINEAR_DECEL_MPS2 * dt_sec);
            cmd_speed_y = slew_toward(cmd_speed_y, 0.0f, BRAKE_LINEAR_DECEL_MPS2 * dt_sec);
            cmd_gyro = slew_toward(cmd_gyro, 0.0f, BRAKE_GYRO_DECEL_RADPS2 * dt_sec);

            if (chassis_command_stopped(cmd_speed_x, cmd_speed_y, cmd_gyro)) {
                cmd_speed_x = 0.0f;
                cmd_speed_y = 0.0f;
                cmd_gyro = 0.0f;
                chassis_set_static(chassis, true);
                chassis_set_speed(chassis, 0.0f, 0.0f);
                chassis_set_gyro(chassis, 0.0f);
            } else {
                chassis_set_static(chassis, false);
                chassis_set_speed(chassis, cmd_speed_x, cmd_speed_y);
                chassis_set_gyro(chassis, cmd_gyro);
            }
        } else {
            cmd_speed_x = -X * linear_speed_scale;
            cmd_speed_y = -Y * linear_speed_scale;
            cmd_gyro = angvel * gyro_speed_scale;

            chassis_set_static(chassis, false);
            chassis_set_speed(chassis, cmd_speed_x, cmd_speed_y);
            chassis_set_gyro(chassis, cmd_gyro);
        }
    }
}

K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);

int main(void)
{
    k_sleep(K_MSEC(2000));
    chassis_set_enabled(chassis, false);
    k_sleep(K_MSEC(100));
    enable_chassis_motors();
    chassis_set_enabled(chassis, true);
    chassis_set_gyro(chassis, 0);

    while (1) {
        k_sleep(K_MSEC(500));
    }

    return 0;
}
