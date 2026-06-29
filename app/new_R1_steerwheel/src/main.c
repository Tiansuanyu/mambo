/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>
#include <ares/board/init.h>
#include "devices.h"

LOG_MODULE_REGISTER(homing_test, LOG_LEVEL_INF);

static const struct gpio_dt_spec limit_0 = GPIO_DT_SPEC_GET(DT_NODELABEL(limit_0), gpios);

int main(void)
{
    board_init();

    k_sleep(K_MSEC(2000));
    LOG_INF("单舵轮 (steer_motor1) 光电门寻零测试");

    if (!device_is_ready(limit_0.port)) {
        LOG_ERR("光电门未就绪！");
        return -1;
    }
    gpio_pin_configure_dt(&limit_0, GPIO_INPUT);

    if (!device_is_ready(steer_motor1)) {
        LOG_ERR("舵向电机未就绪！");
        return -1;
    }

    motor_control(steer_motor1, ENABLE_MOTOR);
    LOG_INF("舵向电机已使能 (ENABLE_MOTOR)");

    LOG_INF("[准备阶段] 初始状态确认...");
    if (gpio_pin_get_dt(&limit_0) == 1) {
        LOG_WRN("开机时铁片在光电门内！后退退出...");
        motor_set_speed(steer_motor1, -25.0f);
        while (gpio_pin_get_dt(&limit_0) == 1) {
            k_msleep(5);
        }
        k_msleep(150); 
    }

    LOG_INF("[阶段 1] 快速寻找光电门 (60 RPM)...");
    motor_set_speed(steer_motor1, 60.0f);
    while (true) {
        if (gpio_pin_get_dt(&limit_0) == 1) {
            LOG_INF("-> 光电门已触发！");
            break;
        }
        k_msleep(5);
    }

    LOG_INF("[阶段 2] 反转退出光电门 (-25 RPM)...");
    motor_set_speed(steer_motor1, -25.0f);
    while (true) {
        if (gpio_pin_get_dt(&limit_0) == 0) {
            k_msleep(150); 
            break;
        }
        k_msleep(5);
    }

    LOG_INF("[阶段 3] 极慢速精确切入 (5 RPM)...");
    motor_set_speed(steer_motor1, 5.0f);
    while (true) {
        if (gpio_pin_get_dt(&limit_0) == 1) {
            k_msleep(2);
            if (gpio_pin_get_dt(&limit_0) == 1) {
                motor_set_speed(steer_motor1, 0.0f);
                k_msleep(50); 
                
                motor_control(steer_motor1, SET_ZERO);
                LOG_INF("寻零完美锁定！");
                break;
            }
        }
        k_msleep(2);
    }

    k_msleep(100);
    LOG_INF("切换至位置环，死守目标角度: 0.0度");
    motor_set_angle(steer_motor1, 0.0f);

    motor_status_t status;
    while (1) {
        if (motor_get(steer_motor1, &status) == 0) {
            LOG_INF("angle: %7.2f° | speed: %6.1f RPM | torque: %5.2f | limit_0: %s", 
                    status.angle, status.rpm, status.torque,
                    gpio_pin_get_dt(&limit_0) ? "1" : "0");
        }
        k_sleep(K_MSEC(500));
    }
    return 0;
}