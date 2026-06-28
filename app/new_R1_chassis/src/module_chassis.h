#ifndef MODULE_CHASSIS_H_
#define MODULE_CHASSIS_H_

#include <stdbool.h>

#include "control_packet.h"

void module_chassis_update(const struct team_usb_packet *packet, bool updated);

#endif
