#include <math.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/logging/log.h>

#include "module_chassis.h"

LOG_MODULE_REGISTER(module_chassis, LOG_LEVEL_INF);

#define STICK_DEADZONE 0.06f
#define GEAR_SWITCH_DOWN_THRESHOLD -0.5f
#define GEAR_SWITCH_RELEASE_THRESHOLD -0.2f
#define HIGH_LINEAR_SPEED_SCALE 1.8f
#define HIGH_GYRO_SPEED_SCALE 15.0f
#define LOW_LINEAR_SPEED_SCALE 0.6f
#define LOW_GYRO_SPEED_SCALE 7.5f

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

	if (!updated || packet == NULL) {
		chassis_stop_static();
		return;
	}

	float x = finite_or_zero(packet->raw[TEAM_USB_IDX_LX]);
	float y = finite_or_zero(packet->raw[TEAM_USB_IDX_LY]);
	float angvel = -finite_or_zero(packet->raw[TEAM_USB_IDX_RX]);
	float gear_switch = finite_or_zero(packet->raw[TEAM_USB_IDX_RY]);

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
	chassis_set_speed(chassis_dev, x * linear_speed_scale, y * linear_speed_scale);
	chassis_set_gyro(chassis_dev, angvel * gyro_speed_scale);
}
