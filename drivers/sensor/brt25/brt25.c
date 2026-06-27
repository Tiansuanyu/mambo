/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brt_25

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "brt25.h"

LOG_MODULE_REGISTER(brt25, LOG_LEVEL_INF);

struct brt25_config {
	const struct device *can_dev;
	uint32_t can_id;
	float resolution;
};

struct brt25_data {
	uint32_t raw_value;
};

float brt25_get_angle(const struct device *dev)
{
	if (dev == NULL) return 0.0f;
	struct brt25_data *data = dev->data;
	const struct brt25_config *config = dev->config;
	return ((float)data->raw_value * 360.0f) / config->resolution;
}

static void brt25_can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct brt25_data *data = dev->data;
	const struct brt25_config *config = dev->config;

	if (frame->dlc >= 7 && frame->data[0] == 0x07 && frame->data[2] == 0x01) {
		if (frame->data[1] == config->can_id) {
			data->raw_value = frame->data[3] |
					     (frame->data[4] << 8) |
					     (frame->data[5] << 16) |
					     (frame->data[6] << 24);
		}
	}
}

//设置回传时间
void brt25_set_return_time(const struct device *dev, uint16_t time_ms)
{
    if (dev == NULL) return;
    const struct brt25_config *config = dev->config;

    struct can_frame config_frame = {
        .id = config->can_id,
        .dlc = 5,
        .flags = 0,
    };

    config_frame.data[0] = 0x05;
    config_frame.data[1] = config->can_id;
    config_frame.data[2] = 0x05;

    uint16_t time_us = time_ms * 1000;

    config_frame.data[3] = (uint8_t)(time_us & 0xFF);
    config_frame.data[4] = (uint8_t)((time_us >> 8) & 0xFF);

    can_send(config->can_dev, &config_frame, K_MSEC(10), NULL, NULL);
    LOG_INF("BRT25 (ID:%d) return time set to %d ms", config->can_id, time_ms);
}

static int brt25_init(const struct device *dev)
{
	const struct brt25_config *config = dev->config;

	if (!device_is_ready(config->can_dev)) {
		LOG_ERR("CAN device %s is not ready!", config->can_dev->name);
		return -ENODEV;
	}

	int start_err = can_start(config->can_dev);
	/* 如果返回 0 说明启动成功；如果返回 -EALREADY 说明大疆电机已经帮我们启动过了 */
	if (start_err != 0 && start_err != -EALREADY) {
		LOG_ERR("Failed to start CAN bus (err %d)", start_err);
		return start_err;
	}

	struct can_filter filter = {
		.id = config->can_id,
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};

	int err = can_add_rx_filter(config->can_dev, brt25_can_rx_callback, (void *)dev, &filter);
	if (err < 0) {
		LOG_ERR("Failed to add CAN filter for BRT encoder ID: %d (err %d)", config->can_id, err);
		return err;
	}

	LOG_INF("BRT Encoder (ID: %d, Res: %.0f) Initialized.", config->can_id, (double)config->resolution);
	return 0;
}

#define BRT25_INIT(inst)                                                       \
	static struct brt25_data brt25_data_##inst = {                    \
		.raw_value = 0,                                              \
	};                                                                          \
	static const struct brt25_config brt25_config_##inst = {          \
		.can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),           \
		.can_id = DT_INST_PROP(inst, can_id),                               \
		.resolution = (float)DT_INST_PROP(inst, resolution),                \
	};                                                                          \
	DEVICE_DT_INST_DEFINE(inst, brt25_init, NULL,                          \
			      &brt25_data_##inst, &brt25_config_##inst,   \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BRT25_INIT)
