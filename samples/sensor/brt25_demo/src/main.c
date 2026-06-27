#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "brt25.h"

LOG_MODULE_REGISTER(brt25_test, LOG_LEVEL_INF);

/* 从设备树中获取你在 overlay 里定义的 brt_encoder1 */
const struct device *brt_dev = DEVICE_DT_GET(DT_NODELABEL(brt_encoder1));

int main(void)
{
    LOG_INF("BRT25 High-Performance Demo Starting...");

    /* 检查底层 CAN 过滤器是否初始化成功 */
    if (!device_is_ready(brt_dev)) {
        LOG_ERR("BRT25 device is not ready. Please check overlay and CAN bus!");
        return -1;
    }
    LOG_INF("BRT25 device found!");

    /* 强制降频到 50ms */
    brt25_set_return_time(brt_dev, 50);

    /* 给编码器内部 RAM 留出十几毫秒的生效和状态切换时间 */
	    k_msleep(20);

    LOG_INF("Reading absolute angles...");

    /* 3. 进入主循环 */
    while (1) {
        float current_angle = brt25_get_angle(brt_dev);

        LOG_INF("BRT25 (ID: 1) Angle: %.2f deg", (double)current_angle);

        k_msleep(50);
    }

    return 0;
}
