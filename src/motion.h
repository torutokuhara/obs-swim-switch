/*
 * Frame-difference motion measurement on small grayscale frames.
 * Pure C, no OBS dependencies.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Rectangle in normalised frame coordinates (0..1). */
struct zone_rect {
	float x, y, w, h;
};

/* Which on-screen direction of movement a zone counts. */
enum motion_dir {
	MOTION_ANY = 0,
	MOTION_RIGHT = 1,
	MOTION_LEFT = 2,
	MOTION_DOWN = 3,
	MOTION_UP = 4,
};

enum motion_dir motion_dir_opposite(enum motion_dir d);

/* RGBA/BGRA rows -> 8-bit luma. */
void motion_to_gray(const uint8_t *rgba, uint32_t linesize, int w, int h,
		    uint8_t *gray_out);

/* Fraction (0..1) of pixels in the zone whose luma changed by more than
 * pixel_threshold between prev and cur, and (unless dir is MOTION_ANY) whose
 * local motion points in direction dir.
 *
 * Direction comes from the brightness-constancy constraint: for an edge
 * moving with velocity u along an axis, dI/dt ~= -u * dI/dx, so the sign of
 * -(dI/dt)(dI/dx) is the sign of u. Each moving pixel votes on its own, so
 * swimmers going opposite ways in different lanes do not cancel out. */
float motion_zone_activity(const uint8_t *cur, const uint8_t *prev, int w,
			   int h, const struct zone_rect *z,
			   int pixel_threshold, enum motion_dir dir);

#ifdef __cplusplus
}
#endif
