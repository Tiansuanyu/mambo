/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BRT25_H_
#define ZEPHYR_DRIVERS_SENSOR_BRT25_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

void brt25_set_return_time(const struct device *dev, uint16_t time_ms);

/**
 * @brief Get the current absolute angle from the BRT encoder.
 *
 * @param dev Pointer to the BRT encoder device structure.
 * @return float The angle in degrees (0.0f to 360.0f). Returns 0.0f on error.
 */
float brt25_get_angle(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_BRT25_H_ */