/*
 * "Swim Meet Auto-Switcher" source.
 *
 * Renders either the near-end or far-end camera source. While a race is
 * running it reads back a small copy of the camera that is on air, measures
 * motion inside a "wall" zone and an "exit" zone that the user positions on
 * each camera, and drives the race state machine in race-logic.c.
 */
#include <obs-module.h>
#include <graphics/graphics.h>
#include <graphics/vec4.h>
#include <util/bmem.h>
#include <util/platform.h>
#include <util/threading.h>

#include <stdio.h>
#include <string.h>

#include "motion.h"
#include "race-logic.h"

#define AW 160 /* analysis resolution */
#define AH 90

#define S_NEAR "near_source"
#define S_FAR "far_source"
#define S_WIDTH "width"
#define S_HEIGHT "height"
#define S_LENGTHS "lengths"
#define S_PIXEL_THR "pixel_threshold"
#define S_WALL_THR "wall_threshold"
#define S_EXIT_THR "exit_threshold"
#define S_HOLD "hold_frames"
#define S_QUIET "quiet_ms"
#define S_START_GUARD "start_guard"
#define S_TURN_GUARD "turn_guard"
#define S_ARRIVAL_GUARD "arrival_guard"
#define S_SWITCH_DELAY "switch_delay_ms"
#define S_TIMEOUT "phase_timeout"
#define S_DEBUG "debug_overlay"
#define S_PACE "pace_guard"
static const char *dir_key[2] = {"near_direction", "far_direction"};

static const char *cam_key[2] = {"near", "far"};
static const char *zone_key[2] = {"wall", "exit"};
static const char *coord_key[4] = {"x", "y", "w", "h"};

struct swim_switcher {
	obs_source_t *self;
	pthread_mutex_t mutex;

	char *cam_name[2];
	obs_source_t *cam[2];
	uint64_t last_lookup_ns;

	uint32_t out_w, out_h;
	struct zone_rect zones[2][2]; /* [camera][zone_kind] */
	int pixel_threshold;
	bool debug_overlay;
	enum motion_dir toward_far[2]; /* on-screen direction, per camera */

	struct race_config cfg;
	struct race_state st;

	/* graphics-thread only */
	gs_texrender_t *texrender;
	gs_stagesurf_t *stagesurf;
	bool staged;
	int staged_cam;
	uint8_t cur[AW * AH];
	uint8_t prev[AW * AH];
	bool prev_valid;
	int prev_cam;
	uint64_t last_analysis_ns;
	bool analyze_this_frame;

	obs_hotkey_id hk_start, hk_reset, hk_advance;
};

/* ------------------------------------------------------------------------ */
/* logging                                                                   */

static void log_events(struct swim_switcher *s, uint32_t ev)
{
	if (!ev)
		return;
	const char *name = obs_source_get_name(s->self);
	int done = s->st.lengths_done, total = s->cfg.total_lengths;

	if (ev & RACE_EV_RESET)
		blog(LOG_INFO, "[swim-switcher] '%s': reset", name);
	if (ev & RACE_EV_START)
		blog(LOG_INFO, "[swim-switcher] '%s': race started (%d lengths)",
		     name, total);
	if (ev & RACE_EV_MANUAL)
		blog(LOG_INFO, "[swim-switcher] '%s': manual advance", name);
	if (ev & RACE_EV_TIMEOUT)
		blog(LOG_WARNING,
		     "[swim-switcher] '%s': phase timeout, advancing", name);
	if (ev & RACE_EV_EXIT_DETECTED)
		blog(LOG_INFO, "[swim-switcher] '%s': lead swimmer leaving frame",
		     name);
	if (ev & RACE_EV_SWITCH_FAR)
		blog(LOG_INFO, "[swim-switcher] '%s': -> FAR camera", name);
	if (ev & RACE_EV_SWITCH_NEAR)
		blog(LOG_INFO, "[swim-switcher] '%s': -> NEAR camera", name);
	if (ev & RACE_EV_LENGTH)
		blog(LOG_INFO, "[swim-switcher] '%s': wall touch, length %d/%d",
		     name, done, total);
	if (ev & RACE_EV_FINISH)
		blog(LOG_INFO, "[swim-switcher] '%s': race finished", name);
}

/* ------------------------------------------------------------------------ */
/* child sources                                                             */

/* Takes ownership of new_src (a strong reference, or NULL). */
static void set_child(struct swim_switcher *s, int idx, obs_source_t *new_src)
{
	if (new_src) {
		if (new_src == s->self ||
		    !obs_source_add_active_child(s->self, new_src)) {
			blog(LOG_WARNING,
			     "[swim-switcher] '%s' cannot be used as a camera "
			     "here (it would contain this switcher)",
			     obs_source_get_name(new_src));
			obs_source_release(new_src);
			new_src = NULL;
		}
	}

	pthread_mutex_lock(&s->mutex);
	obs_source_t *old = s->cam[idx];
	s->cam[idx] = new_src;
	pthread_mutex_unlock(&s->mutex);

	if (old) {
		obs_source_remove_active_child(s->self, old);
		obs_source_release(old);
	}
}

static void resolve_children(struct swim_switcher *s, bool force_lookup)
{
	uint64_t now = os_gettime_ns();
	bool lookup = force_lookup ||
		      now - s->last_lookup_ns > 1000000000ULL; /* 1 s */

	for (int i = 0; i < 2; i++) {
		pthread_mutex_lock(&s->mutex);
		obs_source_t *cur = s->cam[i];
		bool removed = cur && obs_source_removed(cur);
		char *want = (!cur && lookup && s->cam_name[i] &&
			      *s->cam_name[i])
				     ? bstrdup(s->cam_name[i])
				     : NULL;
		pthread_mutex_unlock(&s->mutex);

		if (removed)
			set_child(s, i, NULL);

		if (want) {
			obs_source_t *src = obs_get_source_by_name(want);
			if (src)
				set_child(s, i, src);
			bfree(want);
		}
	}
	if (lookup)
		s->last_lookup_ns = now;
}

/* ------------------------------------------------------------------------ */
/* hotkeys and buttons                                                       */

static void do_start(struct swim_switcher *s)
{
	pthread_mutex_lock(&s->mutex);
	uint32_t ev = race_start(&s->st);
	log_events(s, ev);
	pthread_mutex_unlock(&s->mutex);
}

static void do_reset(struct swim_switcher *s)
{
	pthread_mutex_lock(&s->mutex);
	race_reset(&s->st);
	log_events(s, RACE_EV_RESET);
	pthread_mutex_unlock(&s->mutex);
}

static void do_advance(struct swim_switcher *s)
{
	pthread_mutex_lock(&s->mutex);
	uint32_t ev = race_advance(&s->st, &s->cfg);
	log_events(s, ev);
	pthread_mutex_unlock(&s->mutex);
}

static void hk_start_cb(void *data, obs_hotkey_id id, obs_hotkey_t *hk,
			bool pressed)
{
	UNUSED_PARAMETER(id);
	UNUSED_PARAMETER(hk);
	if (pressed)
		do_start(data);
}

static void hk_reset_cb(void *data, obs_hotkey_id id, obs_hotkey_t *hk,
			bool pressed)
{
	UNUSED_PARAMETER(id);
	UNUSED_PARAMETER(hk);
	if (pressed)
		do_reset(data);
}

static void hk_advance_cb(void *data, obs_hotkey_id id, obs_hotkey_t *hk,
			  bool pressed)
{
	UNUSED_PARAMETER(id);
	UNUSED_PARAMETER(hk);
	if (pressed)
		do_advance(data);
}

static bool btn_start(obs_properties_t *props, obs_property_t *p, void *data)
{
	UNUSED_PARAMETER(props);
	UNUSED_PARAMETER(p);
	do_start(data);
	return true;
}

static bool btn_reset(obs_properties_t *props, obs_property_t *p, void *data)
{
	UNUSED_PARAMETER(props);
	UNUSED_PARAMETER(p);
	do_reset(data);
	return true;
}

static bool btn_advance(obs_properties_t *props, obs_property_t *p,
			void *data)
{
	UNUSED_PARAMETER(props);
	UNUSED_PARAMETER(p);
	do_advance(data);
	return true;
}

/* ------------------------------------------------------------------------ */
/* settings                                                                  */

static void zone_setting_name(char *buf, size_t n, int cam, int zone, int c)
{
	snprintf(buf, n, "%s_%s_%s", cam_key[cam], zone_key[zone],
		 coord_key[c]);
}

static void ss_update(void *data, obs_data_t *settings)
{
	struct swim_switcher *s = data;
	bool changed[2] = {false, false};
	const char *names[2] = {obs_data_get_string(settings, S_NEAR),
				obs_data_get_string(settings, S_FAR)};

	pthread_mutex_lock(&s->mutex);

	for (int i = 0; i < 2; i++) {
		if (!s->cam_name[i] || strcmp(s->cam_name[i], names[i]) != 0) {
			bfree(s->cam_name[i]);
			s->cam_name[i] = bstrdup(names[i]);
			changed[i] = true;
		}
	}

	s->out_w = (uint32_t)obs_data_get_int(settings, S_WIDTH);
	s->out_h = (uint32_t)obs_data_get_int(settings, S_HEIGHT);
	s->pixel_threshold = (int)obs_data_get_int(settings, S_PIXEL_THR);
	s->debug_overlay = obs_data_get_bool(settings, S_DEBUG);

	struct race_config *c = &s->cfg;
	c->total_lengths = (int)obs_data_get_int(settings, S_LENGTHS);
	c->wall_threshold =
		(float)obs_data_get_double(settings, S_WALL_THR) / 100.0f;
	c->exit_threshold =
		(float)obs_data_get_double(settings, S_EXIT_THR) / 100.0f;
	c->hold_frames = (int)obs_data_get_int(settings, S_HOLD);
	c->quiet_s = (double)obs_data_get_int(settings, S_QUIET) / 1000.0;
	c->start_guard_s = obs_data_get_double(settings, S_START_GUARD);
	c->turn_guard_s = obs_data_get_double(settings, S_TURN_GUARD);
	c->arrival_guard_s = obs_data_get_double(settings, S_ARRIVAL_GUARD);
	c->switch_delay_s =
		(double)obs_data_get_int(settings, S_SWITCH_DELAY) / 1000.0;
	c->phase_timeout_s = obs_data_get_double(settings, S_TIMEOUT);
	c->pace_guard = obs_data_get_double(settings, S_PACE) / 100.0;
	for (int i = 0; i < 2; i++)
		s->toward_far[i] =
			(enum motion_dir)obs_data_get_int(settings, dir_key[i]);

	char key[64];
	for (int cam = 0; cam < 2; cam++) {
		for (int z = 0; z < 2; z++) {
			float v[4];
			for (int k = 0; k < 4; k++) {
				zone_setting_name(key, sizeof(key), cam, z, k);
				v[k] = (float)obs_data_get_double(settings,
								  key) /
				       100.0f;
			}
			s->zones[cam][z] =
				(struct zone_rect){v[0], v[1], v[2], v[3]};
		}
	}

	pthread_mutex_unlock(&s->mutex);

	for (int i = 0; i < 2; i++) {
		if (changed[i])
			set_child(s, i, NULL);
	}
	if (changed[0] || changed[1])
		resolve_children(s, true);
}

static void set_zone_default(obs_data_t *settings, int cam, int zone, double x,
			     double y, double w, double h)
{
	char key[64];
	double v[4] = {x, y, w, h};
	for (int k = 0; k < 4; k++) {
		zone_setting_name(key, sizeof(key), cam, zone, k);
		obs_data_set_default_double(settings, key, v[k]);
	}
}

static void ss_defaults(obs_data_t *settings)
{
	struct race_config c;
	race_config_defaults(&c);

	obs_data_set_default_int(settings, S_WIDTH, 1920);
	obs_data_set_default_int(settings, S_HEIGHT, 1080);
	obs_data_set_default_int(settings, S_LENGTHS, c.total_lengths);
	obs_data_set_default_int(settings, S_PIXEL_THR, 30);
	obs_data_set_default_double(settings, S_WALL_THR,
				    c.wall_threshold * 100.0);
	obs_data_set_default_double(settings, S_EXIT_THR,
				    c.exit_threshold * 100.0);
	obs_data_set_default_int(settings, S_HOLD, c.hold_frames);
	obs_data_set_default_int(settings, S_QUIET, (int)(c.quiet_s * 1000));
	obs_data_set_default_double(settings, S_START_GUARD, c.start_guard_s);
	obs_data_set_default_double(settings, S_TURN_GUARD, c.turn_guard_s);
	obs_data_set_default_double(settings, S_ARRIVAL_GUARD,
				    c.arrival_guard_s);
	obs_data_set_default_int(settings, S_SWITCH_DELAY,
				 (int)(c.switch_delay_s * 1000));
	obs_data_set_default_double(settings, S_TIMEOUT, c.phase_timeout_s);
	obs_data_set_default_bool(settings, S_DEBUG, true);
	obs_data_set_default_double(settings, S_PACE, c.pace_guard * 100.0);
	obs_data_set_default_int(settings, dir_key[CAM_NEAR], MOTION_RIGHT);
	obs_data_set_default_int(settings, dir_key[CAM_FAR], MOTION_RIGHT);

	/* Side-on cameras, start end on the left of the near camera and the
	 * far wall on the right of the far camera. Adjust with the overlay. */
	set_zone_default(settings, CAM_NEAR, ZONE_WALL, 0, 20, 12, 70);
	set_zone_default(settings, CAM_NEAR, ZONE_EXIT, 88, 20, 12, 70);
	set_zone_default(settings, CAM_FAR, ZONE_WALL, 88, 20, 12, 70);
	set_zone_default(settings, CAM_FAR, ZONE_EXIT, 0, 20, 12, 70);
}

/* ------------------------------------------------------------------------ */
/* properties                                                                */

struct list_ctx {
	obs_property_t *list;
	obs_source_t *self;
};

static bool add_to_list(void *param, obs_source_t *src)
{
	struct list_ctx *ctx = param;
	if (src == ctx->self)
		return true;
	if (!(obs_source_get_output_flags(src) & OBS_SOURCE_VIDEO))
		return true;
	const char *name = obs_source_get_name(src);
	if (name && *name)
		obs_property_list_add_string(ctx->list, name, name);
	return true;
}

static obs_property_t *add_source_list(obs_properties_t *props,
				       const char *key, const char *label,
				       obs_source_t *self)
{
	obs_property_t *p = obs_properties_add_list(props, key, label,
						    OBS_COMBO_TYPE_LIST,
						    OBS_COMBO_FORMAT_STRING);
	obs_property_list_add_string(p, obs_module_text("None"), "");
	struct list_ctx ctx = {p, self};
	obs_enum_sources(add_to_list, &ctx);
	obs_enum_scenes(add_to_list, &ctx);
	return p;
}

static void add_direction_list(obs_properties_t *props, int cam)
{
	obs_property_t *p = obs_properties_add_list(
		props, dir_key[cam],
		obs_module_text(cam == CAM_NEAR ? "Direction.near"
						: "Direction.far"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(p, obs_module_text("Direction.Right"),
				  MOTION_RIGHT);
	obs_property_list_add_int(p, obs_module_text("Direction.Left"),
				  MOTION_LEFT);
	obs_property_list_add_int(p, obs_module_text("Direction.Up"),
				  MOTION_UP);
	obs_property_list_add_int(p, obs_module_text("Direction.Down"),
				  MOTION_DOWN);
	obs_property_list_add_int(p, obs_module_text("Direction.Any"),
				  MOTION_ANY);
	obs_property_set_long_description(
		p, obs_module_text("Direction.Tooltip"));
}

static void add_zone_group(obs_properties_t *parent, int cam, int zone)
{
	char group_key[64], key[64], label_key[64];
	snprintf(group_key, sizeof(group_key), "grp_%s_%s", cam_key[cam],
		 zone_key[zone]);
	snprintf(label_key, sizeof(label_key), "Zone.%s.%s", cam_key[cam],
		 zone_key[zone]);

	obs_properties_t *g = obs_properties_create();
	static const char *coord_label[4] = {"Zone.X", "Zone.Y", "Zone.W",
					     "Zone.H"};
	for (int k = 0; k < 4; k++) {
		zone_setting_name(key, sizeof(key), cam, zone, k);
		obs_properties_add_float_slider(g, key,
						obs_module_text(coord_label[k]),
						0.0, 100.0, 0.5);
	}
	obs_properties_add_group(parent, group_key, obs_module_text(label_key),
				 OBS_GROUP_NORMAL, g);
}

static obs_properties_t *ss_properties(void *data)
{
	struct swim_switcher *s = data;
	obs_properties_t *props = obs_properties_create();

	/* status + race controls */
	char status[160];
	pthread_mutex_lock(&s->mutex);
	snprintf(status, sizeof(status), "%s: %s (%d/%d)",
		 obs_module_text("Status"), race_phase_name(s->st.phase),
		 s->st.lengths_done, s->cfg.total_lengths);
	pthread_mutex_unlock(&s->mutex);
	obs_properties_add_text(props, "status", status, OBS_TEXT_INFO);

	obs_properties_add_button(props, "btn_start",
				  obs_module_text("StartRace"), btn_start);
	obs_properties_add_button(props, "btn_advance",
				  obs_module_text("Advance"), btn_advance);
	obs_properties_add_button(props, "btn_reset", obs_module_text("Reset"),
				  btn_reset);

	obs_property_t *p = obs_properties_add_int(
		props, S_LENGTHS, obs_module_text("Lengths"), 2, 200, 2);
	obs_property_set_long_description(p,
					  obs_module_text("Lengths.Tooltip"));

	/* cameras */
	add_source_list(props, S_NEAR, obs_module_text("NearSource"), s->self);
	add_direction_list(props, CAM_NEAR);
	add_source_list(props, S_FAR, obs_module_text("FarSource"), s->self);
	add_direction_list(props, CAM_FAR);
	obs_properties_add_int(props, S_WIDTH, obs_module_text("Width"), 16,
			       8192, 1);
	obs_properties_add_int(props, S_HEIGHT, obs_module_text("Height"), 16,
			       8192, 1);

	obs_properties_add_bool(props, S_DEBUG,
				obs_module_text("DebugOverlay"));

	/* zones */
	add_zone_group(props, CAM_NEAR, ZONE_WALL);
	add_zone_group(props, CAM_NEAR, ZONE_EXIT);
	add_zone_group(props, CAM_FAR, ZONE_WALL);
	add_zone_group(props, CAM_FAR, ZONE_EXIT);

	/* detection tuning */
	obs_properties_t *d = obs_properties_create();
	obs_properties_add_int_slider(d, S_PIXEL_THR,
				      obs_module_text("PixelThreshold"), 1,
				      255, 1);
	obs_properties_add_float_slider(d, S_WALL_THR,
					obs_module_text("WallThreshold"), 0.1,
					100.0, 0.1);
	obs_properties_add_float_slider(d, S_EXIT_THR,
					obs_module_text("ExitThreshold"), 0.1,
					100.0, 0.1);
	obs_properties_add_int(d, S_HOLD, obs_module_text("HoldFrames"), 1, 60,
			       1);
	obs_properties_add_int(d, S_QUIET, obs_module_text("QuietMs"), 0, 5000,
			       50);
	obs_properties_add_float(d, S_START_GUARD,
				 obs_module_text("StartGuard"), 0, 120, 0.1);
	obs_properties_add_float(d, S_TURN_GUARD, obs_module_text("TurnGuard"),
				 0, 120, 0.1);
	obs_properties_add_float(d, S_ARRIVAL_GUARD,
				 obs_module_text("ArrivalGuard"), 0, 120, 0.1);
	obs_properties_add_int(d, S_SWITCH_DELAY,
			       obs_module_text("SwitchDelay"), 0, 10000, 50);
	p = obs_properties_add_float_slider(d, S_PACE,
					    obs_module_text("PaceGuard"), 0.0,
					    95.0, 5.0);
	obs_property_set_long_description(p,
					  obs_module_text("PaceGuard.Tooltip"));
	p = obs_properties_add_float(d, S_TIMEOUT,
				     obs_module_text("PhaseTimeout"), 0, 600,
				     0.5);
	obs_property_set_long_description(
		p, obs_module_text("PhaseTimeout.Tooltip"));
	obs_properties_add_group(props, "grp_detect",
				 obs_module_text("Detection"),
				 OBS_GROUP_NORMAL, d);

	return props;
}

/* ------------------------------------------------------------------------ */
/* lifecycle                                                                 */

static const char *ss_get_name(void *unused)
{
	UNUSED_PARAMETER(unused);
	return obs_module_text("SwimSwitcher");
}

static void *ss_create(obs_data_t *settings, obs_source_t *source)
{
	struct swim_switcher *s = bzalloc(sizeof(*s));
	s->self = source;
	pthread_mutex_init(&s->mutex, NULL);
	race_config_defaults(&s->cfg);
	race_reset(&s->st);

	s->hk_start = obs_hotkey_register_source(
		source, "swim_switcher.start", obs_module_text("StartRace"),
		hk_start_cb, s);
	s->hk_advance = obs_hotkey_register_source(
		source, "swim_switcher.advance", obs_module_text("Advance"),
		hk_advance_cb, s);
	s->hk_reset = obs_hotkey_register_source(source, "swim_switcher.reset",
						 obs_module_text("Reset"),
						 hk_reset_cb, s);

	ss_update(s, settings);
	return s;
}

static void ss_destroy(void *data)
{
	struct swim_switcher *s = data;

	for (int i = 0; i < 2; i++)
		set_child(s, i, NULL);

	obs_enter_graphics();
	gs_texrender_destroy(s->texrender);
	gs_stagesurface_destroy(s->stagesurf);
	obs_leave_graphics();

	bfree(s->cam_name[0]);
	bfree(s->cam_name[1]);
	pthread_mutex_destroy(&s->mutex);
	bfree(s);
}

static uint32_t ss_width(void *data)
{
	struct swim_switcher *s = data;
	return s->out_w;
}

static uint32_t ss_height(void *data)
{
	struct swim_switcher *s = data;
	return s->out_h;
}

static void ss_enum_sources(void *data, obs_source_enum_proc_t cb,
			    void *param)
{
	struct swim_switcher *s = data;
	obs_source_t *refs[2] = {NULL, NULL};

	pthread_mutex_lock(&s->mutex);
	for (int i = 0; i < 2; i++)
		refs[i] = s->cam[i] ? obs_source_get_ref(s->cam[i]) : NULL;
	pthread_mutex_unlock(&s->mutex);

	for (int i = 0; i < 2; i++) {
		if (refs[i]) {
			cb(s->self, refs[i], param);
			obs_source_release(refs[i]);
		}
	}
}

static void ss_tick(void *data, float seconds)
{
	struct swim_switcher *s = data;
	resolve_children(s, false);

	pthread_mutex_lock(&s->mutex);
	uint32_t ev = race_tick(&s->st, &s->cfg, seconds);
	log_events(s, ev);
	s->analyze_this_frame = true;
	pthread_mutex_unlock(&s->mutex);
}

/* ------------------------------------------------------------------------ */
/* analysis (graphics thread)                                                */

static void process_frame(struct swim_switcher *s, int cam)
{
	uint64_t now = os_gettime_ns();
	double dt = s->last_analysis_ns
			    ? (double)(now - s->last_analysis_ns) / 1e9
			    : 0.0;
	if (dt > 0.5)
		dt = 0.5;
	s->last_analysis_ns = now;

	if (!s->prev_valid || s->prev_cam != cam) {
		memcpy(s->prev, s->cur, sizeof(s->prev));
		s->prev_valid = true;
		s->prev_cam = cam;
		return;
	}

	pthread_mutex_lock(&s->mutex);
	struct zone_rect wall = s->zones[cam][ZONE_WALL];
	struct zone_rect exit_z = s->zones[cam][ZONE_EXIT];
	int thr = s->pixel_threshold;
	enum motion_dir far_dir = s->toward_far[cam];
	pthread_mutex_unlock(&s->mutex);

	/* Near camera: swimmers leave through the exit heading to the far wall
	 * and arrive at the wall heading back. Far camera: the reverse. */
	enum motion_dir back_dir = motion_dir_opposite(far_dir);
	enum motion_dir wall_dir = cam == CAM_NEAR ? back_dir : far_dir;
	enum motion_dir exit_dir = cam == CAM_NEAR ? far_dir : back_dir;

	float a_wall = motion_zone_activity(s->cur, s->prev, AW, AH, &wall,
					    thr, wall_dir);
	float a_exit = motion_zone_activity(s->cur, s->prev, AW, AH, &exit_z,
					    thr, exit_dir);
	memcpy(s->prev, s->cur, sizeof(s->prev));

	pthread_mutex_lock(&s->mutex);
	/* camera may have changed between staging and now */
	if ((int)race_camera_for(&s->st) == cam) {
		uint32_t ev = race_feed(&s->st, &s->cfg, a_wall, a_exit, dt);
		log_events(s, ev);
	}
	pthread_mutex_unlock(&s->mutex);
}

static void analyze(struct swim_switcher *s, int cam, obs_source_t *src)
{
	if (!s->texrender)
		s->texrender = gs_texrender_create(GS_RGBA, GS_ZS_NONE);
	if (!s->stagesurf)
		s->stagesurf = gs_stagesurface_create(AW, AH, GS_RGBA);
	if (!s->texrender || !s->stagesurf)
		return;

	/* 1. read back what was staged last frame (avoids a GPU stall) */
	if (s->staged) {
		uint8_t *pixels;
		uint32_t linesize;
		if (gs_stagesurface_map(s->stagesurf, &pixels, &linesize)) {
			if (s->staged_cam == cam) {
				motion_to_gray(pixels, linesize, AW, AH,
					       s->cur);
				gs_stagesurface_unmap(s->stagesurf);
				process_frame(s, cam);
			} else {
				gs_stagesurface_unmap(s->stagesurf);
				s->prev_valid = false;
			}
		}
		s->staged = false;
	}

	/* 2. render a small copy of the on-air camera and stage it */
	if (!src)
		return;
	uint32_t cw = obs_source_get_width(src);
	uint32_t ch = obs_source_get_height(src);
	if (!cw || !ch)
		return;

	gs_texrender_reset(s->texrender);
	if (gs_texrender_begin(s->texrender, AW, AH)) {
		struct vec4 clear;
		vec4_zero(&clear);
		gs_clear(GS_CLEAR_COLOR, &clear, 0.0f, 0);
		gs_ortho(0.0f, (float)cw, 0.0f, (float)ch, -100.0f, 100.0f);

		gs_blend_state_push();
		gs_blend_function(GS_BLEND_ONE, GS_BLEND_ZERO);
		obs_source_video_render(src);
		gs_blend_state_pop();

		gs_texrender_end(s->texrender);

		gs_stage_texture(s->stagesurf,
				 gs_texrender_get_texture(s->texrender));
		s->staged = true;
		s->staged_cam = cam;
	}
}

/* ------------------------------------------------------------------------ */
/* debug overlay                                                             */

static void fill_rect(gs_eparam_t *color, const struct vec4 *c, float x,
		      float y, float w, float h)
{
	if (w < 1.0f || h < 1.0f)
		return;
	gs_effect_set_vec4(color, c);
	gs_matrix_push();
	gs_matrix_translate3f(x, y, 0.0f);
	gs_draw_sprite(NULL, 0, (uint32_t)w, (uint32_t)h);
	gs_matrix_pop();
}

static void outline_rect(gs_eparam_t *color, const struct vec4 *c, float x,
			 float y, float w, float h, float t)
{
	fill_rect(color, c, x, y, w, t);
	fill_rect(color, c, x, y + h - t, w, t);
	fill_rect(color, c, x, y, t, h);
	fill_rect(color, c, x + w - t, y, t, h);
}

static void draw_overlay(struct swim_switcher *s, int cam, float ow, float oh)
{
	pthread_mutex_lock(&s->mutex);
	struct zone_rect zones[2] = {s->zones[cam][0], s->zones[cam][1]};
	float act[2] = {s->st.activity[0], s->st.activity[1]};
	float thr[2] = {s->cfg.wall_threshold, s->cfg.exit_threshold};
	bool running = race_is_running(&s->st);
	bool armed = race_target_armed(&s->st, &s->cfg);
	bool pending = s->st.switch_pending;
	enum zone_kind target = race_target_zone(&s->st);
	enum race_phase phase = s->st.phase;
	int done = s->st.lengths_done;
	int total = s->cfg.total_lengths;
	pthread_mutex_unlock(&s->mutex);

	gs_effect_t *solid = obs_get_base_effect(OBS_EFFECT_SOLID);
	gs_eparam_t *color = gs_effect_get_param_by_name(solid, "color");
	gs_technique_t *tech = gs_effect_get_technique(solid, "Solid");

	gs_blend_state_push();
	gs_enable_blending(true);
	gs_blend_function(GS_BLEND_SRCALPHA, GS_BLEND_INVSRCALPHA);

	gs_technique_begin(tech);
	gs_technique_begin_pass(tech, 0);

	float t = oh / 270.0f;
	if (t < 2.0f)
		t = 2.0f;

	for (int z = 0; z < 2; z++) {
		float x = zones[z].x * ow, y = zones[z].y * oh;
		float w = zones[z].w * ow, h = zones[z].h * oh;
		bool is_target = running && (int)target == z;

		struct vec4 border;
		if (!is_target)
			vec4_set(&border, 0.6f, 0.6f, 0.6f, 0.6f); /* ignored */
		else if (pending)
			vec4_set(&border, 1.0f, 0.0f, 1.0f, 0.9f); /* cutting */
		else if (armed)
			vec4_set(&border, 0.1f, 1.0f, 0.2f, 0.9f); /* armed */
		else
			vec4_set(&border, 1.0f, 0.85f, 0.0f, 0.9f); /* guard */
		outline_rect(color, &border, x, y, w, h, t);

		/* activity bar, full height = trigger threshold */
		float frac = thr[z] > 0.0f ? act[z] / thr[z] : 0.0f;
		bool over = frac >= 1.0f;
		if (frac > 1.0f)
			frac = 1.0f;
		struct vec4 bar;
		if (over)
			vec4_set(&bar, 1.0f, 0.15f, 0.1f, 0.45f);
		else
			vec4_set(&bar, 0.2f, 0.6f, 1.0f, 0.35f);
		float bh = (h - 2 * t) * frac;
		fill_rect(color, &bar, x + t, y + h - t - bh, w - 2 * t, bh);
	}

	/* phase marker + one square per length, top-left */
	float sq = oh / 40.0f, gap = sq / 3.0f, x0 = sq, y0 = sq;
	struct vec4 pc;
	switch (phase) {
	case PHASE_IDLE:
		vec4_set(&pc, 0.5f, 0.5f, 0.5f, 0.9f);
		break;
	case PHASE_FINISHED:
		vec4_set(&pc, 0.1f, 1.0f, 0.2f, 0.9f);
		break;
	case PHASE_FAR_IN:
	case PHASE_FAR_OUT:
		vec4_set(&pc, 1.0f, 0.55f, 0.0f, 0.9f);
		break;
	default:
		vec4_set(&pc, 0.2f, 0.5f, 1.0f, 0.9f);
		break;
	}
	fill_rect(color, &pc, x0, y0, sq * 1.5f, sq * 1.5f);

	struct vec4 on, off;
	vec4_set(&on, 1.0f, 1.0f, 1.0f, 0.9f);
	vec4_set(&off, 1.0f, 1.0f, 1.0f, 0.25f);
	for (int i = 0; i < total; i++) {
		int row = i / 25, col = i % 25;
		float x = x0 + sq * 1.5f + gap * 2 + col * (sq + gap);
		float y = y0 + row * (sq + gap);
		fill_rect(color, i < done ? &on : &off, x, y, sq, sq);
	}

	gs_technique_end_pass(tech);
	gs_technique_end(tech);
	gs_blend_state_pop();
}

/* ------------------------------------------------------------------------ */
/* render                                                                    */

static void ss_render(void *data, gs_effect_t *effect)
{
	UNUSED_PARAMETER(effect);
	struct swim_switcher *s = data;

	pthread_mutex_lock(&s->mutex);
	int cam = (int)race_camera_for(&s->st);
	obs_source_t *src = s->cam[cam] ? obs_source_get_ref(s->cam[cam])
					: NULL;
	bool do_analysis = s->analyze_this_frame;
	s->analyze_this_frame = false; /* once per frame, not per view */
	float ow = (float)s->out_w, oh = (float)s->out_h;
	bool overlay = s->debug_overlay;
	pthread_mutex_unlock(&s->mutex);

	if (do_analysis)
		analyze(s, cam, src);

	if (src) {
		uint32_t cw = obs_source_get_width(src);
		uint32_t ch = obs_source_get_height(src);
		if (cw && ch) {
			gs_matrix_push();
			gs_matrix_scale3f(ow / (float)cw, oh / (float)ch,
					  1.0f);
			obs_source_video_render(src);
			gs_matrix_pop();
		}
		obs_source_release(src);
	}

	if (overlay)
		draw_overlay(s, cam, ow, oh);
}

struct obs_source_info swim_switcher_source_info = {
	.id = "swim_meet_auto_switcher",
	.type = OBS_SOURCE_TYPE_INPUT,
	.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_CUSTOM_DRAW,
	.get_name = ss_get_name,
	.create = ss_create,
	.destroy = ss_destroy,
	.update = ss_update,
	.get_defaults = ss_defaults,
	.get_properties = ss_properties,
	.get_width = ss_width,
	.get_height = ss_height,
	.video_tick = ss_tick,
	.video_render = ss_render,
	.enum_active_sources = ss_enum_sources,
	.enum_all_sources = ss_enum_sources,
	.icon_type = OBS_ICON_TYPE_CAMERA,
};
