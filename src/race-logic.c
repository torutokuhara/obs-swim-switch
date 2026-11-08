#include "race-logic.h"

#include <string.h>

void race_config_defaults(struct race_config *c)
{
	c->total_lengths = 4;
	c->wall_threshold = 0.04f;
	c->exit_threshold = 0.04f;
	c->hold_frames = 3;
	c->quiet_s = 0.0;
	c->start_guard_s = 3.0;
	c->turn_guard_s = 2.0;
	c->arrival_guard_s = 1.0;
	c->switch_delay_s = 0.0;
	c->phase_timeout_s = 0.0;
	c->pace_guard = 0.8;
}

static enum segment_kind segment_of(enum race_phase p)
{
	return (p == PHASE_FAR_IN || p == PHASE_NEAR_IN) ? SEG_EXIT_TO_WALL
							 : SEG_WALL_TO_EXIT;
}

/* Learn the lead swimmer's pace from a phase that ended by detection or by
 * the operator. The start segment includes the dive, so it is only used as
 * a first (short, therefore permissive) estimate. */
static void learn_pace(struct race_state *st)
{
	enum segment_kind k = segment_of(st->phase);
	double d = st->phase_time;
	if (d <= 0.0)
		return;
	double e = st->expected[k];
	bool dive = st->phase == PHASE_NEAR_OUT && st->from_start;
	if (e <= 0.0 || (k == SEG_WALL_TO_EXIT && st->pace_from_dive)) {
		st->expected[k] = d;
		if (k == SEG_WALL_TO_EXIT)
			st->pace_from_dive = dive;
		return;
	}
	/* Nobody gets 25% faster from one length to the next: a much shorter
	 * segment was a false trigger (lapped swimmer), so do not learn it. */
	if (d < 0.75 * e)
		return;
	st->expected[k] = 0.5 * e + 0.5 * d;
}

static void enter_phase(struct race_state *st, enum race_phase p)
{
	st->phase = p;
	st->phase_time = 0.0;
	st->switch_pending = false;
	st->switch_countdown = 0.0;
	memset(&st->trig, 0, sizeof(st->trig));
}

void race_reset(struct race_state *st)
{
	memset(st, 0, sizeof(*st));
	st->phase = PHASE_IDLE;
}

uint32_t race_start(struct race_state *st)
{
	race_reset(st);
	enter_phase(st, PHASE_NEAR_OUT);
	st->from_start = true;
	return RACE_EV_START;
}

bool race_is_running(const struct race_state *st)
{
	return st->phase != PHASE_IDLE && st->phase != PHASE_FINISHED;
}

enum race_camera race_camera_for(const struct race_state *st)
{
	switch (st->phase) {
	case PHASE_FAR_IN:
	case PHASE_FAR_OUT:
		return CAM_FAR;
	default:
		return CAM_NEAR; /* idle, starts, near lengths and finishes */
	}
}

enum zone_kind race_target_zone(const struct race_state *st)
{
	switch (st->phase) {
	case PHASE_NEAR_OUT:
	case PHASE_FAR_OUT:
		return ZONE_EXIT;
	default:
		return ZONE_WALL;
	}
}

static double fixed_guard(const struct race_state *st,
			  const struct race_config *c)
{
	switch (st->phase) {
	case PHASE_NEAR_OUT:
		return st->from_start ? c->start_guard_s : c->turn_guard_s;
	case PHASE_FAR_OUT:
		return c->turn_guard_s;
	case PHASE_FAR_IN:
	case PHASE_NEAR_IN:
		return c->arrival_guard_s;
	default:
		return 0.0;
	}
}

static double phase_guard(const struct race_state *st,
			  const struct race_config *c)
{
	double g = fixed_guard(st, c);
	double e = st->expected[segment_of(st->phase)];
	if (c->pace_guard > 0.0 && e > 0.0 && c->pace_guard * e > g)
		g = c->pace_guard * e;
	return g;
}

bool race_target_armed(const struct race_state *st,
		       const struct race_config *c)
{
	return race_is_running(st) && !st->switch_pending &&
	       st->phase_time >= phase_guard(st, c) && st->trig.primed;
}

static uint32_t do_switch(struct race_state *st)
{
	if (st->phase == PHASE_NEAR_OUT) {
		enter_phase(st, PHASE_FAR_IN);
		return RACE_EV_SWITCH_FAR;
	}
	if (st->phase == PHASE_FAR_OUT) {
		enter_phase(st, PHASE_NEAR_IN);
		return RACE_EV_SWITCH_NEAR;
	}
	return 0;
}

/* The thing the current phase is waiting for has happened. */
static uint32_t fire(struct race_state *st, const struct race_config *c,
		     bool learn)
{
	if (learn)
		learn_pace(st);

	switch (st->phase) {
	case PHASE_NEAR_OUT:
	case PHASE_FAR_OUT:
		if (c->switch_delay_s > 0.0) {
			st->switch_pending = true;
			st->switch_countdown = c->switch_delay_s;
			return RACE_EV_EXIT_DETECTED;
		}
		return RACE_EV_EXIT_DETECTED | do_switch(st);

	case PHASE_FAR_IN:
		st->lengths_done++;
		if (st->lengths_done >= c->total_lengths) {
			/* odd length count; still end on the near camera */
			enter_phase(st, PHASE_FINISHED);
			return RACE_EV_LENGTH | RACE_EV_FINISH;
		}
		enter_phase(st, PHASE_FAR_OUT);
		return RACE_EV_LENGTH;

	case PHASE_NEAR_IN:
		st->lengths_done++;
		if (st->lengths_done >= c->total_lengths) {
			enter_phase(st, PHASE_FINISHED);
			return RACE_EV_LENGTH | RACE_EV_FINISH;
		}
		enter_phase(st, PHASE_NEAR_OUT);
		st->from_start = false;
		return RACE_EV_LENGTH;

	default:
		return 0;
	}
}

uint32_t race_advance(struct race_state *st, const struct race_config *c)
{
	if (st->phase == PHASE_IDLE)
		return RACE_EV_MANUAL | race_start(st);
	if (!race_is_running(st))
		return 0;
	if (st->switch_pending)
		return RACE_EV_MANUAL | do_switch(st);
	return RACE_EV_MANUAL | fire(st, c, true);
}

uint32_t race_tick(struct race_state *st, const struct race_config *c,
		   double dt)
{
	if (!race_is_running(st))
		return 0;

	st->phase_time += dt;

	if (st->switch_pending) {
		st->switch_countdown -= dt;
		if (st->switch_countdown <= 0.0)
			return do_switch(st);
		return 0;
	}

	if (c->phase_timeout_s > 0.0 && st->phase_time >= c->phase_timeout_s)
		return RACE_EV_TIMEOUT | fire(st, c, false);

	return 0;
}

uint32_t race_feed(struct race_state *st, const struct race_config *c,
		   float wall_activity, float exit_activity, double dt)
{
	st->activity[ZONE_WALL] = wall_activity;
	st->activity[ZONE_EXIT] = exit_activity;

	if (!race_is_running(st) || st->switch_pending)
		return 0;

	if (st->phase_time < phase_guard(st, c)) {
		memset(&st->trig, 0, sizeof(st->trig));
		return 0;
	}

	enum zone_kind z = race_target_zone(st);
	float a = (z == ZONE_WALL) ? wall_activity : exit_activity;
	float thr = (z == ZONE_WALL) ? c->wall_threshold : c->exit_threshold;
	struct zone_trigger *t = &st->trig;

	if (c->quiet_s <= 0.0)
		t->primed = true;

	if (a < thr) {
		t->hot_frames = 0;
		t->quiet_time += dt;
		if (t->quiet_time >= c->quiet_s)
			t->primed = true;
		return 0;
	}

	if (!t->primed) {
		/* still busy from something else (splash, wake): not calm yet */
		t->quiet_time = 0.0;
		return 0;
	}

	int need = c->hold_frames < 1 ? 1 : c->hold_frames;
	if (++t->hot_frames >= need)
		return fire(st, c, true);
	return 0;
}

const char *race_phase_name(enum race_phase p)
{
	switch (p) {
	case PHASE_IDLE:
		return "idle";
	case PHASE_NEAR_OUT:
		return "near: leaving start end";
	case PHASE_FAR_IN:
		return "far: approaching far wall";
	case PHASE_FAR_OUT:
		return "far: leaving far wall";
	case PHASE_NEAR_IN:
		return "near: approaching near wall";
	case PHASE_FINISHED:
		return "finished";
	}
	return "?";
}
