#ifndef TEAM_USB_CONTROL_H_
#define TEAM_USB_CONTROL_H_

#include <stdbool.h>

#include <zephyr/kernel.h>

#include "control_packet.h"

int usb_control_init(void);
bool usb_control_poll(struct team_usb_packet *packet, k_timeout_t timeout);

#endif
