/*
 * Race state machine for the swim meet auto-switcher.
 * Pure C, no OBS dependencies, so it can be unit tested on its own.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum race_camera { CAM_NEAR = 0, CAM_FAR = 1 };
enum zone_kind { ZONE_WALL = 0, ZONE_EXIT = 1 };

enum race_phase {
	PHASE_IDLE,     /* behind the blocks, waiting for start  -> near */
	PHASE_NEAR_OUT, /* leaving the near end, watch near exit -> near */
	PHASE_FAR_IN,   /* approaching far wall, watch far wall  -> far  */
	PHASE_FAR_OUT,  /* leaving the far wall, watch far exit  -> far  */
	PHASE_NEAR_IN,  /* approaching near wall, watch near wall-> near */
	PHASE_FINISHED, /* race over                             -> near */
};

/* Event bits returned by race_* functions (for logging). */
#define RACE_EV_START (1u << 0)
#define RACE_EV_EXIT_DETECTED (1u << 1)
#define RACE_EV_SWITCH_FAR (1u << 2)
#define RACE_EV_SWITCH_NEAR (1u << 3)
#define RACE_EV_LENGTH (1u << 4)
#define RACE_EV_FINISH (1u << 5)
#define RACE_EV_TIMEOUT (1u << 6)
#define RACE_EV_MANUAL (1u << 7)
#define RACE_EV_RESET (1u << 8)

struct race_config {
	int total_lengths;      /* e.g. 4 for a 100 in a 25 pool */
	float wall_threshold;   /* fraction of zone pixels in motion, 0..1 */
	float exit_threshold;   /* fraction of zone pixels in motion, 0..1 */
	int hold_frames;        /* consecutive frames above threshold */
	double quiet_s;         /* zone must be calm this long before it can fire */
	double start_guard_s;   /* ignore near exit zone right after the start */
	double turn_guard_s;    /* ignore exit zone right after a wall touch */
	double arrival_guard_s; /* ignore wall zone right after a camera switch */
	double switch_delay_s;  /* wait after exit detected before cutting */
	double phase_timeout_s; /* force the next step if nothing fires; 0 = off */
	/* Ignore the target zone for this fraction of the lead swimmer's
	 * measured time for the same segment on previous lengths. Stops slower
	 * or lapped swimmers moving the same way from triggering. 0 = off. */
	double pace_guard;
};

enum segment_kind {
	SEG_WALL_TO_EXIT = 0, /* start or wall touch -> lead leaves frame */
	SEG_EXIT_TO_WALL = 1, /* camera switch -> lead touches wall */
};

struct zone_trigger {
	double quiet_time;
	int hot_frames;
	bool primed;
};

struct race_state {
	enum race_phase phase;
	int lengths_done;
	double phase_time;
	bool from_start;
	bool switch_pending;
	double switch_countdown;
	struct zone_trigger trig;
	float activity[2]; /* last measured activity [zone_kind] of active cam */
	double expected[2]; /* lead's segment durations [segment_kind], 0 = unknown */
	bool pace_from_dive; /* expected[SEG_WALL_TO_EXIT] came from the start */
};

void race_config_defaults(struct race_config *c);

void race_reset(struct race_state *st);
uint32_t race_start(struct race_state *st);
/* Operator "next" button: does whatever the current phase is waiting for. */
uint32_t race_advance(struct race_state *st, const struct race_config *c);
/* Call once per video frame. */
uint32_t race_tick(struct race_state *st, const struct race_config *c,
		   double dt);
/* Call once per analysed frame of the camera that is currently on air. */
uint32_t race_feed(struct race_state *st, const struct race_config *c,
		   float wall_activity, float exit_activity, double dt);

enum race_camera race_camera_for(const struct race_state *st);
enum zone_kind race_target_zone(const struct race_state *st);
bool race_is_running(const struct race_state *st);
/* True once the guard time has passed and the target zone is armed. */
bool race_target_armed(const struct race_state *st,
		       const struct race_config *c);
const char *race_phase_name(enum race_phase p);

#ifdef __cplusplus
}
#endif
