#include <obs-module.h>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("obs-swim-switcher", "en-US")

MODULE_EXPORT const char *obs_module_description(void)
{
	return "Automatically switches between near-end and far-end cameras "
	       "during a swim race.";
}

extern struct obs_source_info swim_switcher_source_info;

bool obs_module_load(void)
{
	obs_register_source(&swim_switcher_source_info);
	blog(LOG_INFO, "[swim-switcher] loaded");
	return true;
}
