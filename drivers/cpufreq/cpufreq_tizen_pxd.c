/*
 *  drivers/cpufreq/cpufreq_tizen_pxd.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * tpis program is free software; you can redistribute it and/or modify
 * it under tpe terms of tpe GNU General Public License version 2 as
 * published by tpe Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/tick.h>
#include <linux/types.h>
#include <linux/cpu.h>
#include <linux/pm_qos.h>

#include "cpufreq_governor.h"
#if defined(CONFIG_SYSTEM_LOAD_ANALYZER)
#include <linux/load_analyzer.h>
#endif

/* On-demand governor macros */
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(50000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(90)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(5000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)

static DEFINE_PER_CPU(struct tp_cpu_dbs_info_s, tp_cpu_dbs_info);

static struct tp_ops tp_ops;

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_TIZEN_PXD
static struct cpufreq_governor cpufreq_gov_tizen_pxd;
#endif

static unsigned int default_powersave_bias;

static void tizen_pxd_powersave_bias_init_cpu(int cpu)
{
	struct tp_cpu_dbs_info_s *dbs_info = &per_cpu(tp_cpu_dbs_info, cpu);

	dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
	dbs_info->freq_lo = 0;
}

/*
 * Not all CPUs want IO time to be accounted as busy; this depends on how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (android.com) claims this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
	/*
	 * For Intel, Core 2 (model 15) and later have an efficient idle.
	 */
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
			boot_cpu_data.x86 == 6 &&
			boot_cpu_data.x86_model >= 15)
		return 1;
#endif
	return 0;
}


static void tizen_pxd_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		tizen_pxd_powersave_bias_init_cpu(i);
	}
}

static void dbs_freq_increase(struct cpufreq_policy *p, unsigned int freq)
{
	struct dbs_data *dbs_data = p->governor_data;
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;

	if (p->cur == p->max)
		return;

	__cpufreq_driver_target(p, freq, tp_tuners->powersave_bias ?
			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);
}

static int get_index(int cnt, int ring_size, int diff)
{
	int ret = 0, modified_diff;

	if ((diff > ring_size) || (diff * (-1) > ring_size))
		modified_diff = diff % ring_size;
	else
		modified_diff = diff;

	ret = (ring_size + cnt + modified_diff) % ring_size;

	return ret;
}


#define CPU_RUNNING_TASK_NUM 25
static unsigned int  cpu_run_task[CPU_RUNNING_TASK_NUM];
static unsigned int  cpu_run_task_cnt;
static void store_run_task(unsigned int run_task_num)
{
	unsigned int cnt;

	if (++cpu_run_task_cnt >= CPU_RUNNING_TASK_NUM)
		cpu_run_task_cnt = 0;
	cnt = cpu_run_task_cnt;

	cpu_run_task[cnt] = run_task_num;
}
static int get_run_task(unsigned int sample_num)
{
	unsigned int loop = 0, cnt = 0, running_task = 0;
	unsigned int  sum_of_running_task = 0;

	cnt = cpu_run_task_cnt;

	for (loop = 0; loop < sample_num ; loop++) {
		sum_of_running_task += cpu_run_task[cnt];
		cnt = get_index(cnt, CPU_RUNNING_TASK_NUM, -1);
	}
	if (sample_num > 0)
		running_task = sum_of_running_task / sample_num;

	return running_task;
}



#define CPU_AVG_FREQ_NUM 25

struct cpu_load_freq_time_tag {
	unsigned int cpu_load_freq;
	unsigned int time;
};
struct cpu_load_freq_time_tag cpu_load_freq_time[CPU_AVG_FREQ_NUM];

static unsigned int  cpu_avg_freq_cnt;
static void store_avg_load_freq(unsigned int cpu_load_freq, unsigned int ms_time)
{
	unsigned int cnt;

	if (++cpu_avg_freq_cnt >= CPU_AVG_FREQ_NUM)
		cpu_avg_freq_cnt = 0;
	cnt = cpu_avg_freq_cnt;

	cpu_load_freq_time[cnt].cpu_load_freq = cpu_load_freq;

	/* max approval time 500ms to prevent overflow in u32
	   1000Mhz X cpu 2 X 100 % X 500ms X number 25 < u32 max
	*/
	if (ms_time > 500)
		ms_time = 500;

	cpu_load_freq_time[cnt].time = ms_time;
}

static int get_sample_num_from_time(unsigned int time)
{
	unsigned int loop = 0, cnt = 0;
	unsigned int sum_of_time = 0;

	cnt = cpu_avg_freq_cnt;

	for (loop = 0; (sum_of_time < time) && (loop < CPU_AVG_FREQ_NUM) ; loop++) {
		sum_of_time += cpu_load_freq_time[cnt].time;
		cnt = get_index(cnt, CPU_AVG_FREQ_NUM, -1);
	}

	return loop;
}

static int get_avg_load_freq(unsigned int ms_time)
{
	unsigned int sample_num;
	unsigned int loop = 0, cnt = 0, avg_load_freq = 0;
	unsigned int sum_of_load_freq = 0, sum_of_time = 0;

	cnt = cpu_avg_freq_cnt;

	sample_num = get_sample_num_from_time(ms_time);

	for (loop = 0; loop < sample_num ; loop++) {
		sum_of_load_freq += (cpu_load_freq_time[cnt].cpu_load_freq * cpu_load_freq_time[cnt].time);
		sum_of_time += cpu_load_freq_time[cnt].time;

		cnt = get_index(cnt, CPU_AVG_FREQ_NUM, -1);
	}

	if (sum_of_time > 0)
		avg_load_freq = sum_of_load_freq / sum_of_time;

	return avg_load_freq;
}

static struct workqueue_struct *cpu_wakeup_wq;
struct delayed_work cpu_wakeup_work;
static void cpu_wakeup(struct work_struct *work)
{
	return;
}

extern int cpu_hotplug_go_to_sleep_flag;
static unsigned int need_cpu_online_num;
int needed_cpu_online_pxd(unsigned int cpu_load[], unsigned int cpu_freq_mhz)
{
	static unsigned int num_online_cpu;
	static unsigned int num_running_task;
	unsigned int max_cpu_freq = 1190; // 1190Mhz
	unsigned int delta_ms_time;

	unsigned int cpu_load_freq;
	unsigned long long current_t;
	static unsigned long long pre_t;

	current_t = cpu_clock(UINT_MAX);
	if (pre_t == 0)
		pre_t = current_t;

	delta_ms_time = (unsigned int) (div_u64((current_t -pre_t) , 1000000));

	pre_t = current_t;

	cpu_load_freq = ((cpu_load[0] + cpu_load[1] + cpu_load[2] + cpu_load[3])* (cpu_freq_mhz));
	store_avg_load_freq(cpu_load_freq, delta_ms_time);

	num_running_task = nr_running();
	store_run_task(num_running_task * 100);
#if defined(CONFIG_SYSTEM_LOAD_ANALYZER)
	store_external_load_factor(NR_RUNNING_TASK, num_running_task * 100);
#endif

	num_online_cpu = num_online_cpus();

	need_cpu_online_num = num_online_cpu;

	if (cpu_hotplug_go_to_sleep_flag == 1) {
		need_cpu_online_num = 1;
	} else {
		switch (num_online_cpu) {

		case 1 :
			if ((cpu_freq_mhz >= 998)
				&& (cpu_load[0] >= 80)
				&& (get_avg_load_freq(200) > max_cpu_freq * 80)
				&& (get_run_task(3) >= 250)) {
				need_cpu_online_num++;
			}

			break;

		case 2 :
			if ((get_run_task(3) <= 200)
				&& (get_avg_load_freq(150) < max_cpu_freq * 80)
				&& (cpu_load[0] + cpu_load[1] < 80)) {
				need_cpu_online_num--;
			} else if ((cpu_freq_mhz >= 1094)
				&& (get_avg_load_freq(200) > max_cpu_freq * 160)
				&& (get_run_task(2) >= 350)) {
				need_cpu_online_num++;
			}

			break;

		case 3 :
			if ((get_run_task(3) <= 300)
				&& (get_avg_load_freq(150) <= max_cpu_freq * 150)) {
				need_cpu_online_num--;
			} else if ((cpu_freq_mhz >= 1094)
				&& (get_avg_load_freq(200) > max_cpu_freq * 240)
				&& (get_run_task(3) >= 450)) {
				need_cpu_online_num++;
			}

			break;

		case 4 :
			if ((get_run_task(3) <= 400)
				&& (get_avg_load_freq(150) <= max_cpu_freq * 200)) {
				need_cpu_online_num--;
			}

			break;
		}
	}

	if ((num_online_cpu >= 2) && (need_cpu_online_num >=2)) {
		if (delayed_work_pending(&cpu_wakeup_work))
			cancel_delayed_work_sync(&cpu_wakeup_work);

		queue_delayed_work(cpu_wakeup_wq
				, &cpu_wakeup_work, msecs_to_jiffies(100));
	}

	return need_cpu_online_num;
}

void __ref cpu_sim_hotplug_work_pxd(struct work_struct *work)
{
	int cpu;
	int num_online_cpu;

	num_online_cpu = num_online_cpus();

#if 1
	if (need_cpu_online_num != num_online_cpu) {
		if (need_cpu_online_num < num_online_cpu)
			for (cpu = num_online_cpu; cpu > need_cpu_online_num; cpu--) {
				if (cpu_online(cpu - 1) == 1) {
					cpu_down(cpu - 1);
				}
			}
		else
			for (cpu = num_online_cpu; cpu < need_cpu_online_num; cpu++) {
				if (cpu_online(cpu) == 0) {
					cpu_up(cpu);
				}
			}
	}
#endif
}


/*
 * Every sampling_rate, we check, if current idle time is less than 20%
 * (default), then we try to increase frequency. Every sampling_rate, we look
 * for the lowest frequency which can sustain the load while keeping idle time
 * over 30%. If such a frequency exist, we try to decrease to this frequency.
 *
 * Any frequency increase takes it to the maximum frequency. Frequency reduction
 * happens at minimum steps of 5% (default) of current frequency
 */
static void tp_check_cpu(int cpu, unsigned int load)
{
	struct tp_cpu_dbs_info_s *dbs_info = &per_cpu(tp_cpu_dbs_info, cpu);
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	struct dbs_data *dbs_data = policy->governor_data;
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;

	dbs_info->freq_lo = 0;

	/* Check for frequency increase */
	if (load > tp_tuners->up_threshold) {
		/* If switching to max speed, apply sampling_down_factor */
		if (policy->cur < policy->max)
			dbs_info->rate_mult =
				tp_tuners->sampling_down_factor;
		dbs_freq_increase(policy, policy->max);
		return;
	} else {
		unsigned int freq_next;
		freq_next = load * policy->cpuinfo.max_freq / 100;

		/* No longer fully busy, reset rate_mult */
		dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (!tp_tuners->powersave_bias) {
				__cpufreq_driver_target(policy, freq_next,
						CPUFREQ_RELATION_L);
		} else {
			int freq = tp_ops.powersave_bias_target(policy, freq_next,
					CPUFREQ_RELATION_L);
			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
	}
}

static void tp_dbs_timer(struct work_struct *work)
{
	struct tp_cpu_dbs_info_s *dbs_info =
		container_of(work, struct tp_cpu_dbs_info_s, cdbs.work.work);
	unsigned int cpu = dbs_info->cdbs.cur_policy->cpu;
	struct tp_cpu_dbs_info_s *core_dbs_info = &per_cpu(tp_cpu_dbs_info,
			cpu);
	struct dbs_data *dbs_data = dbs_info->cdbs.cur_policy->governor_data;
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;
	int delay = 0, sample_type = core_dbs_info->sample_type;
	bool modify_all = true;

	mutex_lock(&core_dbs_info->cdbs.timer_mutex);
	if (!need_load_eval(&core_dbs_info->cdbs, tp_tuners->sampling_rate)) {
		modify_all = false;
		goto max_delay;
	}

	/* Common NORMAL_SAMPLE setup */
	core_dbs_info->sample_type = OD_NORMAL_SAMPLE;
	if (sample_type == OD_SUB_SAMPLE) {
		delay = core_dbs_info->freq_lo_jiffies;
		__cpufreq_driver_target(core_dbs_info->cdbs.cur_policy,
				core_dbs_info->freq_lo, CPUFREQ_RELATION_H);
	} else {
		dbs_check_cpu(dbs_data, cpu);
		if (core_dbs_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			core_dbs_info->sample_type = OD_SUB_SAMPLE;
			delay = core_dbs_info->freq_hi_jiffies;
		}
	}

max_delay:
	if (!delay)
		delay = delay_for_sampling_rate(tp_tuners->sampling_rate
				* core_dbs_info->rate_mult);
#if 0
	gov_queue_work(dbs_data, dbs_info->cdbs.cur_policy, delay, modify_all);
#else
	delay = usecs_to_jiffies(50000); /* 50ms fixed */
	gov_queue_work(dbs_data, dbs_info->cdbs.cur_policy, delay, false);
#endif
	mutex_unlock(&core_dbs_info->cdbs.timer_mutex);
}

/************************** sysfs interface ************************/
static struct common_dbs_data tp_dbs_cdata;

/**
 * update_sampling_rate - update sampling rate effective immediately if needed.
 * @new_rate: new sampling rate
 *
 * If new rate is smaller than the old, simply updating
 * dbs_tuners_int.sampling_rate might not be appropriate. For example, if the
 * original sampling_rate was 1 second and the requested new sampling rate is 10
 * ms because the user needs immediate reaction from ondemand governor, but not
 * sure if higher frequency will be required or not, then, the governor may
 * change the sampling rate too late; up to 1 second later. thus, if we are
 * reducing the sampling rate, we need to make the new value effective
 * immediately.
 */
static void update_sampling_rate(struct dbs_data *dbs_data,
		unsigned int new_rate)
{
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;
	int cpu;

	tp_tuners->sampling_rate = new_rate = max(new_rate,
			dbs_data->min_sampling_rate);

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct tp_cpu_dbs_info_s *dbs_info;
		unsigned long next_sampling, appointed_at;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		if (policy->governor != &cpufreq_gov_tizen_pxd) {
			cpufreq_cpu_put(policy);
			continue;
		}
		dbs_info = &per_cpu(tp_cpu_dbs_info, cpu);
		cpufreq_cpu_put(policy);

		mutex_lock(&dbs_info->cdbs.timer_mutex);

		if (!delayed_work_pending(&dbs_info->cdbs.work)) {
			mutex_unlock(&dbs_info->cdbs.timer_mutex);
			continue;
		}

		next_sampling = jiffies + usecs_to_jiffies(new_rate);
		appointed_at = dbs_info->cdbs.work.timer.expires;

		if (time_before(next_sampling, appointed_at)) {

			mutex_unlock(&dbs_info->cdbs.timer_mutex);
			cancel_delayed_work_sync(&dbs_info->cdbs.work);
			mutex_lock(&dbs_info->cdbs.timer_mutex);

			gov_queue_work(dbs_data, dbs_info->cdbs.cur_policy,
					usecs_to_jiffies(new_rate), true);

		}
		mutex_unlock(&dbs_info->cdbs.timer_mutex);
	}
	put_online_cpus();
}

static ssize_t store_sampling_rate(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	update_sampling_rate(dbs_data, input);
	return count;
}

static ssize_t store_io_is_busy(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	tp_tuners->io_is_busy = !!input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct tp_cpu_dbs_info_s *dbs_info = &per_cpu(tp_cpu_dbs_info,
									j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
			&dbs_info->cdbs.prev_cpu_wall, tp_tuners->io_is_busy);
	}
	return count;
}

static ssize_t store_up_threshold(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}

	tp_tuners->up_threshold = input;
	return count;
}

static ssize_t store_sampling_down_factor(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	tp_tuners->sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct tp_cpu_dbs_info_s *dbs_info = &per_cpu(tp_cpu_dbs_info,
				j);
		dbs_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_ignore_nice_load(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == tp_tuners->ignore_nice_load) { /* notping to do */
		return count;
	}
	tp_tuners->ignore_nice_load = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct tp_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(tp_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
			&dbs_info->cdbs.prev_cpu_wall, tp_tuners->io_is_busy);
		if (tp_tuners->ignore_nice_load)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}

static ssize_t store_powersave_bias(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct tp_dbs_tuners *tp_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1000)
		input = 1000;

	tp_tuners->powersave_bias = input;
	tizen_pxd_powersave_bias_init();
	return count;
}

show_store_one(tp, sampling_rate);
show_store_one(tp, io_is_busy);
show_store_one(tp, up_threshold);
show_store_one(tp, sampling_down_factor);
show_store_one(tp, ignore_nice_load);
show_store_one(tp, powersave_bias);
declare_show_sampling_rate_min(tp);

gov_sys_pol_attr_rw(sampling_rate);
gov_sys_pol_attr_rw(io_is_busy);
gov_sys_pol_attr_rw(up_threshold);
gov_sys_pol_attr_rw(sampling_down_factor);
gov_sys_pol_attr_rw(ignore_nice_load);
gov_sys_pol_attr_rw(powersave_bias);
gov_sys_pol_attr_ro(sampling_rate_min);

static struct attribute *dbs_attributes_gov_sys[] = {
	&sampling_rate_min_gov_sys.attr,
	&sampling_rate_gov_sys.attr,
	&up_threshold_gov_sys.attr,
	&sampling_down_factor_gov_sys.attr,
	&ignore_nice_load_gov_sys.attr,
	&powersave_bias_gov_sys.attr,
	&io_is_busy_gov_sys.attr,
	NULL
};

static struct attribute_group tp_attr_group_gov_sys = {
	.attrs = dbs_attributes_gov_sys,
	.name = "tizen_pxd",
};

static struct attribute *dbs_attributes_gov_pol[] = {
	&sampling_rate_min_gov_pol.attr,
	&sampling_rate_gov_pol.attr,
	&up_threshold_gov_pol.attr,
	&sampling_down_factor_gov_pol.attr,
	&ignore_nice_load_gov_pol.attr,
	&powersave_bias_gov_pol.attr,
	&io_is_busy_gov_pol.attr,
	NULL
};

static struct attribute_group tp_attr_group_gov_pol = {
	.attrs = dbs_attributes_gov_pol,
	.name = "tizen_pxd",
};

/************************** sysfs end ************************/
struct work_struct sim_hotplug_work_pxd;

static int tp_init(struct dbs_data *dbs_data)
{
	struct tp_dbs_tuners *tuners;
	u64 idle_time;
	int cpu;

	tuners = kzalloc(sizeof(struct tp_dbs_tuners), GFP_KERNEL);
	if (!tuners) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	cpu = get_cpu();
	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		tuners->up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		/*
		 * In nohz/micro accounting case we set the minimum frequency
		 * not depending on HZ, but fixed (very low). the deferred
		 * timer might skip some samples if idle/sleeping as needed.
		*/
		dbs_data->min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		tuners->up_threshold = DEF_FREQUENCY_UP_THRESHOLD;

		/* For correct statistics, we need 10 ticks for each measure */
		dbs_data->min_sampling_rate = MIN_SAMPLING_RATE_RATIO *
			jiffies_to_usecs(10);
	}

	tuners->sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;
	tuners->ignore_nice_load = 0;
	tuners->powersave_bias = default_powersave_bias;
	tuners->io_is_busy = should_io_be_busy();

	dbs_data->tuners = tuners;

	INIT_WORK(&sim_hotplug_work_pxd, cpu_sim_hotplug_work_pxd);

	cpu_wakeup_wq = alloc_workqueue("cpu_wakeup_wq", WQ_HIGHPRI, 0);
	if (!cpu_wakeup_wq) {
		printk(KERN_ERR "Failed to create cpufreq_msm8226_wq workqueue\n");
	}
	INIT_DELAYED_WORK(&cpu_wakeup_work, cpu_wakeup);


	mutex_init(&dbs_data->mutex);
	return 0;
}

static void tp_exit(struct dbs_data *dbs_data)
{
	kfree(dbs_data->tuners);
}

define_get_cpu_dbs_routines(tp_cpu_dbs_info);

static void cpufreq_governor_empty_func(void) {
	return;
}


static struct tp_ops tp_ops = {
	.powersave_bias_init_cpu = (void (*)(int))cpufreq_governor_empty_func,
	.powersave_bias_target = (unsigned int (*)(struct cpufreq_policy *,unsigned int, unsigned int))\
							cpufreq_governor_empty_func,
	.freq_increase = dbs_freq_increase,
};

static struct common_dbs_data tp_dbs_cdata = {
	.governor = GOV_TIZEN_PXD,
	.attr_group_gov_sys = &tp_attr_group_gov_sys,
	.attr_group_gov_pol = &tp_attr_group_gov_pol,
	.get_cpu_cdbs = get_cpu_cdbs,
	.get_cpu_dbs_info_s = get_cpu_dbs_info_s,
	.gov_dbs_timer = tp_dbs_timer,
	.gov_check_cpu = tp_check_cpu,
	.gov_ops = &tp_ops,
	.init = tp_init,
	.exit = tp_exit,
};

static int tp_cpufreq_governor_dbs(struct cpufreq_policy *policy,
		unsigned int event)
{
	return cpufreq_governor_dbs(policy, &tp_dbs_cdata, event);
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_TIZEN_PXD
static
#endif
struct cpufreq_governor cpufreq_gov_tizen_pxd = {
	.name			= "tizen_pxd",
	.governor		= tp_cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

#define CPU_BOOTING_TIME	19
static struct workqueue_struct *cpu_msm8916_wq;
struct delayed_work booting_cpu_boost_work;
static struct pm_qos_request pm_qos_cpu_min_freq_req;
bool booting_hotplug_condition = 0;

void msm8916_cpu_boot_completed(struct work_struct *work)
{
	pr_info("%s+", __FUNCTION__);

	booting_hotplug_condition = 1;

	if (pm_qos_request_active(&pm_qos_cpu_min_freq_req))
		pm_qos_remove_request(&pm_qos_cpu_min_freq_req);

	pr_info("%s-", __FUNCTION__);
}

static int __init cpufreq_gov_dbs_init(void)
{
	cpu_msm8916_wq = alloc_workqueue("cpu_msm8916_wq", WQ_HIGHPRI, 0);
	if (!cpu_msm8916_wq) {
		printk(KERN_ERR "Failed to create cpufreq_msm8916_wq workqueue\n");
	}

	INIT_DELAYED_WORK(&booting_cpu_boost_work, msm8916_cpu_boot_completed);

	queue_delayed_work(cpu_msm8916_wq
				, &booting_cpu_boost_work, (CPU_BOOTING_TIME)*HZ);

	if (!pm_qos_request_active(&pm_qos_cpu_min_freq_req)) {
		pm_qos_add_request(&pm_qos_cpu_min_freq_req
					, PM_QOS_CPU_FREQ_MIN, 1190400);
	}

	return cpufreq_register_governor(&cpufreq_gov_tizen_pxd);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_tizen_pxd);
}

MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand' - A dynamic cpufreq governor for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_TIZEN_PXD
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
