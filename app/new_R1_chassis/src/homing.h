/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HOMING_H
#define HOMING_H

/**
 * @brief Home all three steer units to their photogate zero, in parallel.
 *
 * All three units are advanced together by a single cooperative loop, so the
 * wheels move simultaneously and each one zeroes on its own schedule. The call
 * blocks until every unit is homed, or returns an error as soon as any unit
 * fails / times out (the others are stopped on failure).
 *
 * Must be called before the chassis control loop is enabled, so that nothing
 * else writes the steer-motor setpoints while homing runs.
 *
 * @return 0 on success, negative errno on failure.
 */
int homing_steer(void);

#endif /* HOMING_H */