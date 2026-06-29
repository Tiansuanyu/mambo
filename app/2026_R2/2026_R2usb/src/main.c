#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/drivers/chassis.h>
#include <ares/board/init.h>
#include <ares/ekf/imu_task.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>
#include <sys/_stdint.h>
// #include "ares/ekf/QuaternionEKF.h"
#include "devices.h"
#include <arm_math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#ifndef M_PI
#define M_PI 3.14159265f
#endif

int pub_cnt = 0;
// #define CHASSIS_SRC_SBUS
#define CHASSIS_SRC_USB

float action[9] = {0};
uint8_t actionbuf[36]= {0};

int rank[4] = {1, 1, -1, -1};
bool ups[4] = {true, true, true, true};
const struct device *dmmotor[4];
void run()
{
	for(int id = 0; id < 4; id++){
		motor_set_mit(dmmotor[id], 10.0f, -50.0f * rank[id], 0.0);
		ups[id] = true;
		k_sleep(K_USEC(130));
}
}
void lift(int id, bool up)
{
if(up&&(!ups[id])) {
	motor_set_mit(dmmotor[id], 10.0f, 0.0f * rank[id], 0.0);
	ups[id] = true;
} else if((!up)&&(ups[id])) {
	motor_set_mit(dmmotor[id], 10.0f, -800.0f * rank[id], 0.0);
	ups[id] = false;
}
k_sleep(K_USEC(130));
}
void lifthigh(int id, float high)
{
	if(high>0.4f) {
		high = 1.0f;
	} else if (high<0.0f) {
		high = 0.0f;
	}
	if (dmmotor[id] != NULL) {
        motor_set_mit(dmmotor[id], 10.0f, -4000.0f * rank[id] * high, 0.0f);
    }
	
k_sleep(K_USEC(130));
}
int losscnt = 0;
void console_feedback(void *arg1, void *arg2, void *arg3)
{
	float angvel = 0;
	
	bool zeroed = false;

	while (1) {
		k_msleep(5);
#ifdef CHASSIS_SRC_USB
		losscnt++;
		// LOG_ERR("DATA： %f, %f, %f, %f, %f, %f, %f, %f, %f", 
		// 		(double)action[0], (double)action[1], (double)action[2], (double)action[3], (double)action[4], (double)action[5], (double)action[6], (double)action[7], (double)action[8]);
float X = action[0];
float Y = action[1];
angvel = action[2];

float linear_magnitude = sqrtf(X * X + Y * Y);
bool in_deadzone = ((linear_magnitude < 0.06f) && (fabsf(angvel) < 0.06f));
if (linear_magnitude < 0.06f) {
	X = 0;
	Y = 0;

}
 if (in_deadzone||losscnt>20) {
	chassis_set_static(chassis, true);
	zeroed = true;
} else {
	chassis_set_static(chassis, false);
	chassis_set_speed(chassis, X , Y );
	chassis_set_gyro(chassis, angvel );
	zeroed = false;
}
lifthigh(0, action[3]);
lifthigh(1, action[4]);
lifthigh(2, action[5]);
lifthigh(3, action[6]);
#endif		
#ifdef CHASSIS_SRC_SBUS
		if (sbus_get_percent(sbus, 4) > 0.5f) {
			lift(0, false);
			lift(1, false);
			lift(2, false);
			lift(3, false);

		} else if (sbus_get_percent(sbus, 4) < -0.5f) {
			run();
		}
		if (sbus_get_percent(sbus, 5) > 0.5f) {
			lift(2, false);
			lift(1, false);
		}else if (sbus_get_percent(sbus, 5) < -0.5f) {
			lift(2, true);
			lift(1, true);
		}
		if(sbus_get_percent(sbus,6) > 0.5f) {
			lift(0, false);
			lift(3, false);
		}else if (sbus_get_percent(sbus, 6) < -0.5f) {
			lift(0, true);
			lift(3, true);
		}
		angvel = -sbus_get_percent(sbus, 0);
		float X = -sbus_get_percent(sbus, 3);
		float Y = -sbus_get_percent(sbus, 1);
		float linear_magnitude = sqrtf(X * X + Y * Y);
		bool in_deadzone = ((linear_magnitude < 0.06f) && (fabsf(angvel) < 0.06f));
		if (linear_magnitude < 0.06f) {
			X = 0;
			Y = 0;
		
		}
		 if (in_deadzone) {
			chassis_set_static(chassis, true);
			zeroed = true;
		} else {
			chassis_set_static(chassis, false);
			chassis_set_speed(chassis, -X * 2.5f, Y * 2.5f);
			chassis_set_gyro(chassis, angvel * 12.5f);
			zeroed = false;
			
		}

		// if (cnt++ % 500 == 0) {
		// 		// LOG_INF("Manual: X=%.1f Y=%.1f Gyro=%.1f", (double)X, (double)Y,
		// 		// 	(double)angvel);
		// 		LOG_INF("Chassis: X=%.1f Y=%.1f Gyro=%.1f", (double)sbus_get_percent(sbus, 4), (double)sbus_get_percent(sbus, 5), (double)sbus_get_percent(sbus, 6));
			
		// }
#endif
	}
}


K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);
uint32_t log_cnt;

int action_rx_cb(int status)
{
	losscnt=0;
	if(status == SYNC_PACK_STATUS_DONE) {
		memcpy(&action, &actionbuf, 36);
	}else{
		return;
	}
	return 0;
}
DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);
int main(void)
{
	k_msleep(100);

	dmmotor[0] = dm_motor1;
	dmmotor[1] = dm_motor2;
	dmmotor[2] = dm_motor3;
	dmmotor[3] = dm_motor4;
	motor_control(dm_motor1, ENABLE_MOTOR);
	motor_control(dm_motor2, ENABLE_MOTOR);
	motor_control(dm_motor3, ENABLE_MOTOR);
	motor_control(dm_motor4, ENABLE_MOTOR);
	
	motor_set_mode(dm_motor1, MIT);
	motor_set_mode(dm_motor2, MIT);
	motor_set_mode(dm_motor3, MIT);
	motor_set_mode(dm_motor4, MIT);
	
	
	k_msleep(2000);
	motor_control(wheel_motor1, ENABLE_MOTOR);
	motor_control(wheel_motor2, ENABLE_MOTOR);
	motor_control(wheel_motor3, ENABLE_MOTOR);
	motor_control(wheel_motor4, ENABLE_MOTOR);
	k_msleep(100); // 等待电机使能
	motor_set_mode(wheel_motor1, ML_SPEED);
	motor_set_mode(wheel_motor2, ML_SPEED);
	motor_set_mode(wheel_motor3, ML_SPEED);
	motor_set_mode(wheel_motor4, ML_SPEED);
	motor_control(steer_motor1, ENABLE_MOTOR);
	motor_control(steer_motor2, ENABLE_MOTOR);
	motor_control(steer_motor3, ENABLE_MOTOR);
	motor_control(steer_motor4, ENABLE_MOTOR);
	ares_bind_interface(&usb_bulk_interface, &dual_protocol);
	
	chassis_set_enabled(chassis, true);
	chassis_set_gyro(chassis, 0);
	dual_sync_add(&dual_protocol, 0x0101, &actionbuf, 36, (dual_trans_cb_t)action_rx_cb);

	while (1) {
			k_msleep(2000);
	}

	return 0;
}
