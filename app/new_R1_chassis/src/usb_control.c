#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>

#include "usb_control.h"

LOG_MODULE_REGISTER(usb_control, LOG_LEVEL_INF);

DUAL_PROPOSE_PROTOCOL_DEFINE(control_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

static float ctrl_rx[TEAM_USB_FLOAT_NUM];
static struct team_usb_packet latest_packet;
static uint32_t ctrl_rx_count;
static sync_table_t *control_rx;

K_SEM_DEFINE(ctrl_rx_sem, 0, 1);
K_MUTEX_DEFINE(ctrl_lock);

static void control_rx_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	struct team_usb_packet packet = {0};

	memcpy(packet.raw, ctrl_rx, sizeof(packet.raw));
	team_usb_decode_buttons(&packet);

	k_mutex_lock(&ctrl_lock, K_FOREVER);
	packet.rx_count = ++ctrl_rx_count;
	latest_packet = packet;
	k_mutex_unlock(&ctrl_lock);

	k_sem_give(&ctrl_rx_sem);
}

int usb_control_init(void)
{
	int ret = ares_bind_interface(&usb_bulk_interface, &control_protocol);
	if (ret != 0) {
		LOG_ERR("failed to initialize ARES USB interface: %d", ret);
		return ret;
	}

	control_rx = dual_sync_add(&control_protocol, TEAM_USB_SYNC_ID, (uint8_t *)ctrl_rx,
				   sizeof(ctrl_rx), control_rx_cb);
	if (control_rx == NULL) {
		LOG_ERR("failed to register control sync id=0x%04x", TEAM_USB_SYNC_ID);
		return -ENOMEM;
	}

	LOG_INF("USB control ready: sync id=0x%04x payload=%d floats",
		TEAM_USB_SYNC_ID, TEAM_USB_FLOAT_NUM);
	return 0;
}

bool usb_control_poll(struct team_usb_packet *packet, k_timeout_t timeout)
{
	if (packet == NULL) {
		return false;
	}

	if (k_sem_take(&ctrl_rx_sem, timeout) != 0) {
		return false;
	}

	k_mutex_lock(&ctrl_lock, K_FOREVER);
	*packet = latest_packet;
	k_mutex_unlock(&ctrl_lock);

	return true;
}
