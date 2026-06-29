#ifndef DEVICES_H
#define DEVICES_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define STEERMOTOR1_NODE    DT_NODELABEL(motor_steer0)
#define LIMIT0_NODE         DT_NODELABEL(limit_0)

/* 导出设备指针 */
static const struct device *steer_motor1 = DEVICE_DT_GET(STEERMOTOR1_NODE);

#endif /* DEVICES_H */