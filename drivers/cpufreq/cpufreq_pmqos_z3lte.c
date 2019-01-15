/* cpufreq_pmqos_z3lte.c */

#include <linux/cpuidle.h>

#if defined(ROTARY_BOOSTER)
/* ROTARY BOOSTER + */
#define ROTARY_BOOSTER_DELAY	200
static int rotary_min_cpu_freq = 600000;
/* ROTARY BOOSTER - */
#endif

#if defined(HARDKEY_BOOSTER)
/* HARD KEY BOOSTER + */
static int hard_key_min_cpu_freq = 1000000;
#define KEY_BOOSTER_DELAY	200
/* HARD KEY BOOSTER - */
#endif

#if defined(TOUCH_WAKEUP_BOOSTER)
/* TOUCH WAKEUP BOOSTER + */
static int touch_wakeup_min_cpu_freq = 1000000;
#define TOUCH_WAKEUP_BOOSTER_DELAY	200
/* TOUCH WAKEUP BOOSTER - */
#endif

#define TOUCH_BOOSTER_OFF_TIME		100
#define TOUCH_BOOSTER_CHG_TIME		200

static struct pm_qos_request pm_qos_cpu_req;
static struct pm_qos_request pm_qos_cpu_online_req;

unsigned int press_cpu_freq, release_cpu_freq;
int touch_cpu_online;

void touch_booster_press_sub(void)
{
	press_cpu_freq = cpufreq_get_touch_boost_press();
	touch_cpu_online = touch_cpu_get_online_min();

	if (!pm_qos_request_active(&pm_qos_cpu_req))
		pm_qos_add_request(&pm_qos_cpu_req
			, PM_QOS_CPU_FREQ_MIN, press_cpu_freq);

	if (touch_cpu_online > 1) {
		if (!pm_qos_request_active(&pm_qos_cpu_online_req))
			pm_qos_add_request(&pm_qos_cpu_online_req
				, PM_QOS_CPU_ONLINE_MIN, touch_cpu_online);
	}
}


void touch_booster_move_sub(void)
{
	unsigned int move_cpu_freq = cpufreq_get_touch_boost_move();

	pm_qos_update_request(&pm_qos_cpu_req, move_cpu_freq);
}


void touch_booster_release_sub(void)
{
	release_cpu_freq = cpufreq_get_touch_boost_release();

	pm_qos_update_request(&pm_qos_cpu_req, release_cpu_freq);
}



void touch_booster_off_sub(void)
{
 	if (pm_qos_request_active(&pm_qos_cpu_req))
		pm_qos_remove_request(&pm_qos_cpu_req);

	if (pm_qos_request_active(&pm_qos_cpu_online_req))
		pm_qos_remove_request(&pm_qos_cpu_online_req);

}


