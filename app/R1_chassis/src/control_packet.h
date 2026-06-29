#ifndef TEAM_USB_CONTROL_PACKET_H_
#define TEAM_USB_CONTROL_PACKET_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#define TEAM_USB_SYNC_ID 0x0303
#define TEAM_USB_FLOAT_NUM 7
#define TEAM_USB_BUTTON_MASK_NUM 3

enum team_usb_data_index {
	TEAM_USB_IDX_BUTTONS_0_15 = 0,
	TEAM_USB_IDX_BUTTONS_16_31,
	TEAM_USB_IDX_BUTTONS_32_47,
	TEAM_USB_IDX_LX,
	TEAM_USB_IDX_LY,
	TEAM_USB_IDX_RX,
	TEAM_USB_IDX_RY,
};

struct team_usb_packet {
	float raw[TEAM_USB_FLOAT_NUM];
	uint16_t button_masks[TEAM_USB_BUTTON_MASK_NUM];
	uint32_t rx_count;
};

static inline uint16_t team_usb_float_to_mask(float value)
{
	if (!isfinite(value) || value <= 0.0f) {
		return 0U;
	}

	if (value >= 65535.0f) {
		return UINT16_MAX;
	}

	return (uint16_t)(value + 0.5f);
}

static inline void team_usb_decode_buttons(struct team_usb_packet *packet)
{
	packet->button_masks[0] = team_usb_float_to_mask(packet->raw[TEAM_USB_IDX_BUTTONS_0_15]);
	packet->button_masks[1] = team_usb_float_to_mask(packet->raw[TEAM_USB_IDX_BUTTONS_16_31]);
	packet->button_masks[2] = team_usb_float_to_mask(packet->raw[TEAM_USB_IDX_BUTTONS_32_47]);
}

static inline bool team_usb_button_pressed(const struct team_usb_packet *packet, uint8_t button_id)
{
	if (button_id >= TEAM_USB_BUTTON_MASK_NUM * 16U) {
		return false;
	}

	return (packet->button_masks[button_id / 16U] & BIT(button_id % 16U)) != 0U;
}

#endif
