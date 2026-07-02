#include <math.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "module_chassis.h"

LOG_MODULE_REGISTER(module_chassis, LOG_LEVEL_INF);

#define STICK_DEADZONE 0.06f
#define GEAR_TOGGLE_BUTTON_ID 10U /* RT_FULL / R2 in usb_float51_rx */
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

void module_chassis_update(const struct team_usb_packet *packet, bool updated)
{
	static bool low_gear = true;
	static bool gear_button_was_pressed;
	static int64_t last_rx_log_ms;
	static int64_t last_timeout_log_ms;
	static int64_t last_control_ms;
	static float cmd_speed_x;
	static float cmd_speed_y;
	static float cmd_gyro;

	if (!updated || packet == NULL) {
		int64_t now_ms = k_uptime_get();

		if (now_ms - last_timeout_log_ms >= USB_RX_TIMEOUT_LOG_INTERVAL_MS) {
			LOG_WRN("USB no packet, chassis static");
			last_timeout_log_ms = now_ms;
		}
		cmd_speed_x = 0.0f;
		cmd_speed_y = 0.0f;
		cmd_gyro = 0.0f;
		last_control_ms = now_ms;
		chassis_stop_static();
		return;
	}

	float x = finite_or_zero(packet->raw[TEAM_USB_IDX_LX]);
	float y = finite_or_zero(packet->raw[TEAM_USB_IDX_LY]);
	float angvel = -finite_or_zero(packet->raw[TEAM_USB_IDX_RX]);
	bool gear_button_pressed = team_usb_button_pressed(packet, GEAR_TOGGLE_BUTTON_ID);
	int64_t now_ms = k_uptime_get();
	float dt_sec = control_dt_seconds(now_ms, &last_control_ms);

	if (now_ms - last_rx_log_ms >= USB_RX_LOG_INTERVAL_MS) {
		LOG_INF("USB axes cnt=%u raw3-6=[%.2f %.2f %.2f %.2f] r2=%u -> x=%.2f y=%.2f w=%.2f",
			packet->rx_count, (double)packet->raw[3], (double)packet->raw[4],
			(double)packet->raw[5], (double)packet->raw[6], gear_button_pressed ? 1U : 0U,
			(double)x, (double)y, (double)angvel);
		last_rx_log_ms = now_ms;
	}

	if (gear_button_pressed && !gear_button_was_pressed) {
		low_gear = !low_gear;
		LOG_INF("chassis gear: %s", low_gear ? "low" : "high");
	}
	gear_button_was_pressed = gear_button_pressed;

	float linear_speed_scale = low_gear ? LOW_LINEAR_SPEED_SCALE : HIGH_LINEAR_SPEED_SCALE;
	float gyro_speed_scale = low_gear ? LOW_GYRO_SPEED_SCALE : HIGH_GYRO_SPEED_SCALE;
	float linear_magnitude = sqrtf(x * x + y * y);

	if (linear_magnitude < STICK_DEADZONE) {
		x = 0.0f;
		y = 0.0f;
	}

	if (fabsf(angvel) < STICK_DEADZONE) {
		angvel = 0.0f;
	}

	if (x == 0.0f && y == 0.0f && angvel == 0.0f) {
		cmd_speed_x = slew_toward(cmd_speed_x, 0.0f, BRAKE_LINEAR_DECEL_MPS2 * dt_sec);
		cmd_speed_y = slew_toward(cmd_speed_y, 0.0f, BRAKE_LINEAR_DECEL_MPS2 * dt_sec);
		cmd_gyro = slew_toward(cmd_gyro, 0.0f, BRAKE_GYRO_DECEL_RADPS2 * dt_sec);

		if (chassis_command_stopped(cmd_speed_x, cmd_speed_y, cmd_gyro)) {
			cmd_speed_x = 0.0f;
			cmd_speed_y = 0.0f;
			cmd_gyro = 0.0f;
			chassis_stop_static();
			return;
		}

		chassis_set_static(chassis_dev, false);
		chassis_set_speed(chassis_dev, cmd_speed_x, cmd_speed_y);
		chassis_set_gyro(chassis_dev, cmd_gyro);
		return;
	}

	cmd_speed_x = -x * linear_speed_scale;
	cmd_speed_y = y * linear_speed_scale;
	cmd_gyro = angvel * gyro_speed_scale;

	chassis_set_static(chassis_dev, false);
	chassis_set_speed(chassis_dev, cmd_speed_x, cmd_speed_y);
	chassis_set_gyro(chassis_dev, cmd_gyro);
}
