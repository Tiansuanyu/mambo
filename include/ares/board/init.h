// init.h
#ifndef INIT_H
#define INIT_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>

struct led_rgb;

int board_init(void);

#endif /* INIT_H */
