#include "motion.h"

#include <stdlib.h>

enum motion_dir motion_dir_opposite(enum motion_dir d)
{
	switch (d) {
	case MOTION_RIGHT:
		return MOTION_LEFT;
	case MOTION_LEFT:
		return MOTION_RIGHT;
	case MOTION_DOWN:
		return MOTION_UP;
	case MOTION_UP:
		return MOTION_DOWN;
	default:
		return MOTION_ANY;
	}
}

void motion_to_gray(const uint8_t *rgba, uint32_t linesize, int w, int h,
		    uint8_t *gray_out)
{
	for (int y = 0; y < h; y++) {
		const uint8_t *row = rgba + (size_t)y * linesize;
		uint8_t *out = gray_out + (size_t)y * w;
		for (int x = 0; x < w; x++) {
			const uint8_t *p = row + x * 4;
			/* channel order does not matter much for motion */
			out[x] = (uint8_t)((p[0] + 2 * p[1] + p[2]) >> 2);
		}
	}
}

static int clampi(int v, int lo, int hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

#define MIN_GRADIENT 3

float motion_zone_activity(const uint8_t *cur, const uint8_t *prev, int w,
			   int h, const struct zone_rect *z,
			   int pixel_threshold, enum motion_dir dir)
{
	int x0 = clampi((int)(z->x * w), 0, w);
	int y0 = clampi((int)(z->y * h), 0, h);
	int x1 = clampi((int)((z->x + z->w) * w + 0.5f), 0, w);
	int y1 = clampi((int)((z->y + z->h) * h + 0.5f), 0, h);

	int total = (x1 - x0) * (y1 - y0);
	if (total <= 0)
		return 0.0f;

	int horizontal = (dir == MOTION_RIGHT || dir == MOTION_LEFT);
	int sign = (dir == MOTION_RIGHT || dir == MOTION_DOWN) ? 1 : -1;

	int moving = 0;
	for (int y = y0; y < y1; y++) {
		const uint8_t *c = cur + (size_t)y * w;
		const uint8_t *p = prev + (size_t)y * w;
		for (int x = x0; x < x1; x++) {
			int it = (int)c[x] - (int)p[x];
			if (abs(it) <= pixel_threshold)
				continue;

			if (dir == MOTION_ANY) {
				moving++;
				continue;
			}

			/* spatial gradient of the average of both frames */
			int g;
			if (horizontal) {
				int xa = x > 0 ? x - 1 : x;
				int xb = x < w - 1 ? x + 1 : x;
				g = ((int)c[xb] + p[xb]) - ((int)c[xa] + p[xa]);
			} else {
				int ya = y > 0 ? y - 1 : y;
				int yb = y < h - 1 ? y + 1 : y;
				g = ((int)cur[(size_t)yb * w + x] +
				     prev[(size_t)yb * w + x]) -
				    ((int)cur[(size_t)ya * w + x] +
				     prev[(size_t)ya * w + x]);
			}
			if (abs(g) < MIN_GRADIENT)
				continue;

			/* velocity sign = sign(-It * Ix) */
			long v = -(long)it * (long)g;
			if (v * sign > 0)
				moving++;
		}
	}
	return (float)moving / (float)total;
}
