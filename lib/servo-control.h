/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Fyrby Additive Manufacturing & Engineering
 *
 * Servo Control Interface for UVC Pan/Tilt
 *
 * Communicates with servo_controller.py via HTTP
 */

#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#include <stdint.h>

/* Pan/Tilt absolute control structure (UVC spec) */
struct uvc_pantilt_absolute {
	int32_t pan;   /* Pan in degrees * 3600 (arc-seconds) */
	int32_t tilt;  /* Tilt in degrees * 3600 (arc-seconds) */
};

/* Pan/Tilt relative control structure (UVC spec) */
struct uvc_pantilt_relative {
	int8_t pan;    /* Pan speed (-100 to 100) */
	uint8_t pan_speed;
	int8_t tilt;   /* Tilt speed (-100 to 100) */
	uint8_t tilt_speed;
};

/* Initialize servo control (connect to controller) */
int servo_control_init(const char *controller_url);

/* Cleanup servo control */
void servo_control_cleanup(void);

/* Get current pan/tilt position */
int servo_control_get_pantilt(struct uvc_pantilt_absolute *pantilt);

/* Set pan/tilt absolute position */
int servo_control_set_pantilt(const struct uvc_pantilt_absolute *pantilt);

/* Set pan/tilt relative movement */
int servo_control_move_pantilt(const struct uvc_pantilt_relative *pantilt);

#endif /* __SERVO_CONTROL_H__ */
