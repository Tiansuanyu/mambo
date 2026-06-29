#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>

LOG_MODULE_REGISTER(usb_float51_rx, LOG_LEVEL_INF);

#define CTRL_SYNC_ID 0x0101
#define CTRL_FLOAT_NUM 51

DUAL_PROPOSE_PROTOCOL_DEFINE(control_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

static float ctrl_rx[CTRL_FLOAT_NUM];
static float ctrl_data[CTRL_FLOAT_NUM];
static uint32_t ctrl_rx_count;

K_SEM_DEFINE(ctrl_rx_sem, 0, 1);

static void control_rx_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	memcpy(ctrl_data, ctrl_rx, sizeof(ctrl_data));
	ctrl_rx_count++;
	k_sem_give(&ctrl_rx_sem);
}

static void ctrl_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_sem_take(&ctrl_rx_sem, K_FOREVER);

		LOG_INF("rx=%u A=%.0f B=%.0f X=%.0f Y=%.0f lx=%.3f ly=%.3f rx=%.3f ry=%.3f",
			ctrl_rx_count,
			(double)ctrl_data[3], (double)ctrl_data[4],
			(double)ctrl_data[5], (double)ctrl_data[6],
			(double)ctrl_data[47], (double)ctrl_data[48],
			(double)ctrl_data[49], (double)ctrl_data[50]);
	}
}

K_THREAD_DEFINE(ctrl_log, 1024, ctrl_thread, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	int ret;
	sync_table_t *control_rx;

	LOG_INF("usb_float51_rx start");

	ret = ares_bind_interface(&usb_bulk_interface, &control_protocol);
	if (ret != 0) {
		LOG_ERR("failed to initialize ARES USB interface: %d", ret);
		return ret;
	}

	control_rx = dual_sync_add(&control_protocol, CTRL_SYNC_ID, (uint8_t *)ctrl_rx,
				   sizeof(ctrl_rx), control_rx_cb);
	if (control_rx == NULL) {
		LOG_ERR("failed to register control sync pack");
		return -ENOMEM;
	}

	LOG_INF("rx sync id=0x%04x, floats=%d", CTRL_SYNC_ID, CTRL_FLOAT_NUM);

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
