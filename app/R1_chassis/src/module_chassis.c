#include <math.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "module_chassis.h"

LOG_MODULE_REGISTER(module_chassis, LOG_LEVEL_INF);

#define STICK_DEADZONE 0.06f
#define GEAR_SWITCH_DOWN_THRESHOLD -0.8f
#define GEAR_SWITCH_RELEASE_THRESHOLD -0.2f
#define HIGH_LINEAR_SPEED_SCALE 1.8f
#define HIGH_GYRO_SPEED_SCALE 15.0f
#define LOW_LINEAR_SPEED_SCALE 0.6f
#define LOW_GYRO_SPEED_SCALE 7.5f
#define USB_RX_LOG_INTERVAL_MS 500
#define USB_RX_TIMEOUT_LOG_INTERVAL_MS 2000

static const struct device *chassis_dev = DEVICE_DT_GET(DT_NODELABEL(chassis));

static float finite_or_zero(float value)
{
	if (!isfinite(value)) {
		return 0.0f;
	}
	return value;
}

static void chassis_stop_static(void)
{
	chassis_set_speed(chassis_dev, 0.0f, 0.0f);
	chassis_set_gyro(chassis_dev, 0.0f);
	chassis_set_static(chassis_dev, true);
}

void module_chassis_update(const struct team_usb_packet *packet, bool updated)
{
	static bool low_gear = true;
	static bool gear_switch_armed = true;
	static int64_t last_rx_log_ms;
	static int64_t last_timeout_log_ms;

	if (!updated || packet == NULL) {
		int64_t now_ms = k_uptime_get();

		if (now_ms - last_timeout_log_ms >= USB_RX_TIMEOUT_LOG_INTERVAL_MS) {
			LOG_WRN("USB no packet, chassis static");
			last_timeout_log_ms = now_ms;
		}
		chassis_stop_static();
		return;
	}

	float x = finite_or_zero(packet->raw[TEAM_USB_IDX_LX]);
	float y = finite_or_zero(packet->raw[TEAM_USB_IDX_LY]);
	float angvel = -finite_or_zero(packet->raw[TEAM_USB_IDX_RX]);
	float gear_switch = finite_or_zero(packet->raw[TEAM_USB_IDX_RY]);
	int64_t now_ms = k_uptime_get();

	if (now_ms - last_rx_log_ms >= USB_RX_LOG_INTERVAL_MS) {
		LOG_INF("USB axes cnt=%u raw3-6=[%.2f %.2f %.2f %.2f] -> x=%.2f y=%.2f w=%.2f gear=%.2f",
			packet->rx_count, (double)packet->raw[3], (double)packet->raw[4],
			(double)packet->raw[5], (double)packet->raw[6],
			(double)x, (double)y, (double)angvel, (double)gear_switch);
		last_rx_log_ms = now_ms;
	}

	bool gear_switch_active = gear_switch < GEAR_SWITCH_RELEASE_THRESHOLD;
	if (gear_switch < GEAR_SWITCH_DOWN_THRESHOLD) {
		if (gear_switch_armed) {
			low_gear = !low_gear;
			gear_switch_armed = false;
			LOG_INF("chassis gear: %s", low_gear ? "low" : "high");
		}
	} else if (gear_switch > GEAR_SWITCH_RELEASE_THRESHOLD) {
		gear_switch_armed = true;
	}

	float linear_speed_scale = low_gear ? LOW_LINEAR_SPEED_SCALE : HIGH_LINEAR_SPEED_SCALE;
	float gyro_speed_scale = low_gear ? LOW_GYRO_SPEED_SCALE : HIGH_GYRO_SPEED_SCALE;
	float linear_magnitude = sqrtf(x * x + y * y);

	if (linear_magnitude < STICK_DEADZONE) {
		x = 0.0f;
		y = 0.0f;
	}

	if (gear_switch_active || fabsf(angvel) < STICK_DEADZONE) {
		angvel = 0.0f;
	}

	if (x == 0.0f && y == 0.0f && angvel == 0.0f) {
		chassis_stop_static();
		return;
	}

	chassis_set_static(chassis_dev, false);
	chassis_set_speed(chassis_dev, -x * linear_speed_scale, y * linear_speed_scale);
	chassis_set_gyro(chassis_dev, angvel * gyro_speed_scale);
}
