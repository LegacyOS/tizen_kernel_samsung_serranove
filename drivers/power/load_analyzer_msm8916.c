/* drivers/power/load_analyzer_swa100.c */


unsigned int get_bimc_clk(void);
unsigned int get_snoc_clk(void);

void store_cpu_load(unsigned int cpufreq[], unsigned int cpu_load[])
{
	unsigned int j = 0, cnt = 0;
	unsigned long long t, t_interval;
	unsigned int  t_interval_us;
	static unsigned long long before_t;
	unsigned long  nanosec_rem;
	int cpu_max = 0, cpu_min = 0;
	struct cpufreq_policy *policy;

	if (cpu_task_history_onoff == 0)
		return;

	if (++cpu_load_freq_history_cnt >= cpu_load_history_num)
		cpu_load_freq_history_cnt = 0;

	cnt = cpu_load_freq_history_cnt;

	policy = cpufreq_cpu_get(0);

	if (policy !=NULL) {
		cpu_min = pm_qos_request(PM_QOS_CPU_FREQ_MIN);
		if (cpu_min < policy->min)
			cpu_min = policy->min;

		cpu_max = pm_qos_request(PM_QOS_CPU_FREQ_MAX);
		if (cpu_max > policy->max)
			cpu_max = policy->max;

		cpufreq_cpu_put(policy);
	}

	cpu_load_freq_history[cnt].cpu_min_locked_freq = cpu_min;
	cpu_load_freq_history[cnt].cpu_max_locked_freq = cpu_max;

	t = cpu_clock(UINT_MAX);

	if (before_t == 0)
		before_t = t;

	for (j = 0; j < CPU_NUM; j++) {
		cpu_load_freq_history[cnt].task_history_cnt[j]
			= cpu_task_history_cnt[j];
	}

#if defined (CONFIG_CHECK_WORK_HISTORY)
	for (j = 0; j < CPU_NUM; j++) {
		cpu_load_freq_history[cnt].work_history_cnt[j]
			= cpu_work_history_cnt[j];
	}
#endif

#if defined (CONFIG_SLP_INPUT_REC)
	cpu_load_freq_history[cnt].input_rec_history_cnt
		= input_rec_history_cnt;
#endif

	t_interval = t -before_t;
	do_div(t_interval, 1000);
	t_interval_us = t_interval;
	before_t = t;

	if (t_interval != 0) {
		cpu_load_freq_history[cnt].cpu_idle_time[0]
			= (cpuidle_get_idle_residency_time(0) * 1000 + 5) / t_interval_us;
		cpu_load_freq_history[cnt].cpu_idle_time[1]
			= (cpuidle_get_idle_residency_time(1) * 1000 +5) / t_interval_us;
		cpu_load_freq_history[cnt].cpu_idle_time[2]
			= (cpuidle_get_idle_residency_time(2) * 1000 +5) / t_interval_us;
	}
#if 0
	if (suspending_flag == 1)
		cpu_load_freq_history[cnt].status = 'S';
	else if (suspending_flag == 0)
		cpu_load_freq_history[cnt].status = 'N';
	else if (suspending_flag == -1)
		cpu_load_freq_history[cnt].status = 'F';
#endif
	cpu_load_freq_history[cnt].time_stamp = t;
	nanosec_rem = do_div(t, 1000000000);
	snprintf(cpu_load_freq_history[cnt].time, sizeof(cpu_load_freq_history[cnt].time),
		"%2lu.%02lu", (unsigned long) t,(unsigned long)nanosec_rem / 10000000);

	for (j = 0; j < CPU_NUM; j++) {
		cpu_load_freq_history[cnt].cpufreq[j] = cpufreq[j];
		cpu_load_freq_history[cnt].cpu_load[j] = cpu_load[j];
	}
	cpu_load_freq_history[cnt].touch_event = 0;
	cpu_load_freq_history[cnt].nr_onlinecpu = num_online_cpus();
	cpu_load_freq_history[cnt].nr_run_avg
		= saved_load_factor.nr_running_task;

#if defined(CONFIG_SLP_CHECK_BUS_LOAD)
	if ((value_for_debug == 1) || (value_for_debug == 3)) {
		cpu_load_freq_history[cnt].bimc_clk = ( get_bimc_clk() + 50000) / 100000;
		cpu_load_freq_history[cnt].snoc_clk = (get_snoc_clk()+ 50000) / 100000;
	} else {
		cpu_load_freq_history[cnt].bimc_clk = 0;
		cpu_load_freq_history[cnt].snoc_clk = 0;
	}

	cpu_load_freq_history[cnt].gpu_freq = saved_load_factor.gpu_freq;
	cpu_load_freq_history[cnt].gpu_utilization = saved_load_factor.gpu_utilization;
#endif


	cpu_load_freq_history[cnt].pid = saved_load_factor.active_app_pid;
	cpu_load_freq_history[cnt].battery_soc = saved_load_factor.battery_soc;

#if defined(CONFIG_SLP_BUS_CLK_CHECK_LOAD)
{
	unsigned int pm_domains[PWR_DOMAINS_NUM];
	unsigned int clk_gates[CLK_GATES_NUM];

	if ((value_for_debug == 2) || (value_for_debug == 3)) {
		clk_mon_power_domain(pm_domains);
		clk_mon_clock_gate(clk_gates);

		memcpy(cpu_load_freq_history[cnt].power_domains
					, pm_domains, PWR_DOMAINS_NUM*sizeof(unsigned int));

		memcpy(cpu_load_freq_history[cnt].clk_gates
					, clk_gates, CLK_GATES_NUM*sizeof(unsigned int));
	} else {
		memset(cpu_load_freq_history[cnt].power_domains
			, 0, PWR_DOMAINS_NUM*sizeof(unsigned int));
		memset(cpu_load_freq_history[cnt].clk_gates
			, 0, CLK_GATES_NUM*sizeof(unsigned int));
	}
}
#endif


#if defined(CONFIG_SLP_CURRENT_MONITOR)
	if (current_monitor_en == 1)
		current_monitor_manager(cnt);
#endif

}

unsigned int show_cpu_load_freq_sub(int cnt, int show_cnt, char *buf, unsigned int buf_size, int ret)
{
	int j, delta = 0;

	if ((cnt - show_cnt) < 0) {
		delta = cnt - show_cnt;
		cnt = cpu_load_history_num + delta;
	} else
		cnt -= show_cnt;

	if ((cnt+1 >= cpu_load_history_num)
			|| (cpu_load_freq_history_view[cnt+1].time == 0))
		cnt = 0;
	else
		cnt++;

	for (j = 0; j < show_cnt; j++) {
		if (cnt > cpu_load_history_num-1)
			cnt = 0;

		ret +=  snprintf(buf + ret, buf_size - ret
		, "%8s\t%d.%02d\t%d.%02d\t%d.%02d\t%d.%02d\t%d.%02d/%d.%02d" \
		"    [%5d]\t%3d\t%3d\t%3d\t%3d\t%3d \t%2d.%1d\n"
		, cpu_load_freq_history_view[cnt].time
		, cpu_load_freq_history_view[cnt].cpufreq[0]/1000000
		, (cpu_load_freq_history_view[cnt].cpufreq[0]/10000) % 100
		, cpu_load_freq_history_view[cnt].cpufreq[1]/1000000
		, (cpu_load_freq_history_view[cnt].cpufreq[1]/10000) % 100
		, cpu_load_freq_history_view[cnt].cpufreq[2]/1000000
		, (cpu_load_freq_history_view[cnt].cpufreq[2]/10000) % 100
		, cpu_load_freq_history_view[cnt].cpufreq[3]/1000000
		, (cpu_load_freq_history_view[cnt].cpufreq[3]/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_min_locked_freq/1000000
		, (cpu_load_freq_history_view[cnt].cpu_min_locked_freq/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_max_locked_freq/1000000
		, (cpu_load_freq_history_view[cnt].cpu_max_locked_freq/10000) % 100
		, cnt
		, cpu_load_freq_history_view[cnt].cpu_load[0]
		, cpu_load_freq_history_view[cnt].cpu_load[1]
		, cpu_load_freq_history_view[cnt].cpu_load[2]
		, cpu_load_freq_history_view[cnt].cpu_load[3]
		, cpu_load_freq_history_view[cnt].nr_onlinecpu
		, cpu_load_freq_history_view[cnt].nr_run_avg/100
		, cpu_load_freq_history_view[cnt].nr_run_avg%100);

		++cnt;
	}
	return ret;

}

#if defined(CONFIG_SLP_CHECK_BUS_LOAD)

char cpu_bus_load_freq_menu[] ="=======================================" \
			"===========================================" \
			"===========================================" \
			"===========================================\n" \
			"    TIME   STA  CPUF    CPUF_LOCK" \
			"    [INDEX]   CPU0 ( C1/ C2/ C3)   CPU1  CPU2  CPU3" \
			"\tONLINE\tNR_RUN" \
			"      BMIC     CNOC     PNOC     SNOC     GPUFREQ  GPU_UTIL  INPUT\n";


unsigned int show_cpu_bus_load_freq_sub(int cnt, int show_cnt
					, char *buf, unsigned int buf_size, int ret)
{
	int j, delta = 0;

	if ((cnt - show_cnt) < 0) {
		delta = cnt - show_cnt;
		cnt = cpu_load_history_num + delta;
	} else
		cnt -= show_cnt;

	if ((cnt+1 >= cpu_load_history_num)
			|| (cpu_load_freq_history_view[cnt+1].time == 0))
		cnt = 0;
	else
		cnt++;

	for (j = 0; j < show_cnt; j++) {
		if (cnt > cpu_load_history_num-1)
			cnt = 0;

		ret +=  snprintf(buf + ret, buf_size - ret
		, "%8s    %c    %d.%02d\t%d.%02d/%d.%02d" \
		"    [%5d]    %3d (%3d/%3d/%3d)     %3d  %3d  %3d     %3d" \
		"\t%3d      %4d.%1d    %3d.%1d    %3d.%1d     %3d.%1d       %3d     %3d.%1d" \
		"      %d\n"
		, cpu_load_freq_history_view[cnt].time
		, cpu_load_freq_history_view[cnt].status
		, cpu_load_freq_history_view[cnt].cpufreq[0]/1000000
		, (cpu_load_freq_history_view[cnt].cpufreq[0]/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_min_locked_freq/1000000
		, (cpu_load_freq_history_view[cnt].cpu_min_locked_freq/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_max_locked_freq/1000000
		, (cpu_load_freq_history_view[cnt].cpu_max_locked_freq/10000) % 100
		, cnt
		, cpu_load_freq_history_view[cnt].cpu_load[0]
		, cpu_load_freq_history[cnt].cpu_idle_time[0]/10
		, cpu_load_freq_history[cnt].cpu_idle_time[1]/10
		, cpu_load_freq_history[cnt].cpu_idle_time[2]/10
		, cpu_load_freq_history_view[cnt].cpu_load[1]
		, cpu_load_freq_history_view[cnt].cpu_load[2]
		, cpu_load_freq_history_view[cnt].cpu_load[3]
		, cpu_load_freq_history_view[cnt].nr_onlinecpu
		, cpu_load_freq_history_view[cnt].nr_run_avg
		, cpu_load_freq_history_view[cnt].bimc_clk/10
		, cpu_load_freq_history_view[cnt].bimc_clk%10
		, cpu_load_freq_history_view[cnt].cnoc_clk/10
		, cpu_load_freq_history_view[cnt].cnoc_clk%10
		, cpu_load_freq_history_view[cnt].pnoc_clk/10
		, cpu_load_freq_history_view[cnt].pnoc_clk%10
		, cpu_load_freq_history_view[cnt].snoc_clk/10
		, cpu_load_freq_history_view[cnt].snoc_clk%10
		, cpu_load_freq_history_view[cnt].gpu_freq/1000000
		, cpu_load_freq_history_view[cnt].gpu_utilization/10
		, cpu_load_freq_history_view[cnt].gpu_utilization%10
		, cpu_load_freq_history_view[cnt].input_rec_history_cnt);

		++cnt;
	}
	return ret;

}
#endif

#ifdef CONFIG_SLP_BUS_CLK_CHECK_LOAD
unsigned int show_cpu_bus_clk_load_freq_sub(int cnt
					, int show_cnt, char *buf, int ret)
{
	int j, delta = 0;

	if ((cnt - show_cnt) < 0) {
		delta = cnt - show_cnt;
		cnt = cpu_load_history_num + delta;
	} else
		cnt -= show_cnt;

	if ((cnt+1 >= cpu_load_history_num)
			|| (cpu_load_freq_history_view[cnt+1].time == 0))
		cnt = 0;
	else
		cnt++;

	for (j = 0; j < show_cnt; j++) {

		if (cnt > cpu_load_history_num-1)
			cnt = 0;

		if (ret < PAGE_SIZE - 1) {
		ret +=  snprintf(buf + ret, PAGE_SIZE - ret
			,"%8s   %d.%02d  %d.%02d %d.%02d/%d.%02d" \
			"  [%5d]  %3d %3d %3d " \
			" %2d.%1d      %3d        %3d     %3d    %3d.%1d" \
			"  %6x  %6x  %6x  %6x  %6x  %6x   %6x\n"
			, cpu_load_freq_history_view[cnt].time
			, cpu_load_freq_history_view[cnt].cpufreq[0]/1000
			, (cpu_load_freq_history_view[cnt].cpufreq[0]/10) % 100
			, cpu_load_freq_history_view[cnt].cpufreq[1]/1000000
			, (cpu_load_freq_history_view[cnt].cpufreq[1]/10000) % 100
			, cpu_load_freq_history_view[cnt].cpu_min_locked_freq/1000000
			, (cpu_load_freq_history_view[cnt].cpu_min_locked_freq/10000) % 100
			, cpu_load_freq_history_view[cnt].cpu_max_locked_freq/1000000
			, (cpu_load_freq_history_view[cnt].cpu_max_locked_freq/10000) % 100
			, cnt
			, cpu_load_freq_history_view[cnt].cpu_load[0]
			, cpu_load_freq_history_view[cnt].cpu_load[1]
			, cpu_load_freq_history_view[cnt].nr_onlinecpu
			, cpu_load_freq_history_view[cnt].nr_run_avg/10
			, cpu_load_freq_history_view[cnt].nr_run_avg%10
			, cpu_load_freq_history_view[cnt].mif_bus_freq/1000
			, cpu_load_freq_history_view[cnt].mif_bus_load
			, cpu_load_freq_history_view[cnt].gpu_freq/1000
			, cpu_load_freq_history_view[cnt].gpu_utilization/10
			, cpu_load_freq_history_view[cnt].gpu_utilization%10
			, cpu_load_freq_history_view[cnt].power_domains[0]
			, cpu_load_freq_history_view[cnt].power_domains[1]
			, cpu_load_freq_history_view[cnt].clk_gates[0]
			, cpu_load_freq_history_view[cnt].clk_gates[1]
			, cpu_load_freq_history_view[cnt].clk_gates[2]
			, cpu_load_freq_history_view[cnt].clk_gates[3]
			, cpu_load_freq_history_view[cnt].clk_gates[4]
			);
		} else
			break;
		++cnt;
	}

	return ret;

}
#endif

#if defined(CONFIG_CHECK_NOT_CPUIDLE_CAUSE)
static int not_lpa_cause_check_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret += snprintf(buf + ret, buf_size - ret, "%s\n",  get_not_lpa_cause());

	return ret;
}
#endif

#if defined(CONFIG_SLP_CURRENT_MONITOR)
unsigned int show_current_monitor_read_sub(int cnt, int show_cnt
					, char *buf, unsigned int buf_size, int ret)
{
	int j, delta = 0;

	if ((cnt - show_cnt) < 0) {
		delta = cnt - show_cnt;
		cnt = cpu_load_history_num + delta;
	} else
		cnt -= show_cnt;

	if ((cnt+1 >= cpu_load_history_num)
			|| (cpu_load_freq_history[cnt+1].time == 0))
		cnt = 0;
	else
		cnt++;

	for (j = 0; j < show_cnt; j++) {
		char task_name[TASK_COMM_LEN]={0,};

		if (cnt > cpu_load_history_num-1)
			cnt = 0;

		get_name_from_pid(task_name, cpu_load_freq_history[cnt].pid);

		ret +=  snprintf(buf + ret, buf_size - ret
		, "%8s    %c    %d.%02d\t%d.%02d/%d.%02d  %16s" \
		"    [%5d]    %3d (%3d/%3d/%3d)     %3d  %3d  %3d     %3d" \
		"\t%2d.%1d      %4d.%1d    %3d.%1d    %3d.%1d     %3d.%1d       %3d     %3d.%1d\
			\n"
		, cpu_load_freq_history[cnt].time
		, cpu_load_freq_history[cnt].status
		, cpu_load_freq_history[cnt].cpufreq[0]/1000000
		, (cpu_load_freq_history[cnt].cpufreq[0]/10000) % 100
		, cpu_load_freq_history[cnt].cpu_min_locked_freq/1000000
		, (cpu_load_freq_history[cnt].cpu_min_locked_freq/10000) % 100
		, cpu_load_freq_history[cnt].cpu_max_locked_freq/1000000
		, (cpu_load_freq_history[cnt].cpu_max_locked_freq/10000) % 100
		, task_name
		, cnt
		, cpu_load_freq_history[cnt].cpu_load[0]
		, cpu_load_freq_history[cnt].cpu_idle_time[0]/10
		, cpu_load_freq_history[cnt].cpu_idle_time[1]/10
		, cpu_load_freq_history[cnt].cpu_idle_time[2]/10
		, cpu_load_freq_history[cnt].cpu_load[1]
		, cpu_load_freq_history[cnt].cpu_load[2]
		, cpu_load_freq_history[cnt].cpu_load[3]
		, cpu_load_freq_history[cnt].nr_onlinecpu
		, cpu_load_freq_history[cnt].nr_run_avg/10
		, cpu_load_freq_history[cnt].nr_run_avg%10
		, cpu_load_freq_history[cnt].bimc_clk/10
		, cpu_load_freq_history[cnt].bimc_clk%10
		, cpu_load_freq_history[cnt].cnoc_clk/10
		, cpu_load_freq_history[cnt].cnoc_clk%10
		, cpu_load_freq_history[cnt].pnoc_clk/10
		, cpu_load_freq_history[cnt].pnoc_clk%10
		, cpu_load_freq_history[cnt].snoc_clk/10
		, cpu_load_freq_history[cnt].snoc_clk%10
		, cpu_load_freq_history[cnt].gpu_freq/1000000
		, cpu_load_freq_history[cnt].gpu_utilization/10
		, cpu_load_freq_history[cnt].gpu_utilization%10);

		++cnt;
	}
	return ret;

}
#endif


#if defined (CONFIG_SLP_CPU_TESTER)

enum {
	CPUIDLE_C1,
	CPUIDLE_C2,
	CPUIDLE_C3,
	CPUIDLE_RANDOM,
};

enum {
	CPUFREQ_1200M,
	CPUFREQ_1100M,
	CPUFREQ_1000M,
	CPUFREQ_800M,
	CPUFREQ_533M,
	CPUFREQ_400M,
	CPUFREQ_200M,
	CPUFREQ_RANDOM,
};



struct cpu_test_list_tag cpu_idletest_list[] = {
	{CPU_IDLE_TEST, CPUIDLE_C1, 1000},
	{CPU_IDLE_TEST, CPUIDLE_C1, 1000},
	{CPU_IDLE_TEST, CPUIDLE_C1, 1000},

	{END_OF_LIST, 0, 0},
};

struct cpu_test_list_tag cpu_freqtest_list[] = {
	{CPU_FREQ_TEST, CPUFREQ_1200M, 1000},
	{CPU_FREQ_TEST, CPUFREQ_1100M, 1000},
	{CPU_FREQ_TEST, CPUFREQ_1000M, 1000},
	{CPU_FREQ_TEST, CPUFREQ_800M, 1000},
	{CPU_FREQ_TEST, CPUFREQ_533M, 1000},
	{CPU_FREQ_TEST, CPUFREQ_400M, 1000},
	{CPU_FREQ_TEST, CPUFREQ_200M, 1000},

	{END_OF_LIST, 0, 0},
};

struct cpu_test_freq_table_tag cpu_test_freq_table[] = {
	{CPUFREQ_1200M},
	{CPUFREQ_1100M},
	{CPUFREQ_1000M},
	{CPUFREQ_800M},
	{CPUFREQ_533M},
	{CPUFREQ_400M},
	{CPUFREQ_200M},
};

struct cpu_test_idle_table_tag cpu_test_idle_table[] ={
	{CPUIDLE_C1},
	{CPUIDLE_C2},
	{CPUIDLE_C3},
};

void set_cpufreq_force_state(int cpufreq_enum)
{
	switch(cpufreq_enum) {
	case CPUFREQ_1200M :
		cpufreq_force_state = 1190400;
		break;
	case CPUFREQ_1100M :
		cpufreq_force_state = 1094400;
		break;
	case CPUFREQ_1000M :
		cpufreq_force_state = 998400;
		break;
	case CPUFREQ_800M :
		cpufreq_force_state = 800000;
		break;
	case CPUFREQ_533M :
		cpufreq_force_state = 533333;
		break;
	case CPUFREQ_400M :
		cpufreq_force_state = 400000;
		break;
	case CPUFREQ_200M :
		cpufreq_force_state = 200000;
		break;
	}
}




int cpu_freq_to_enum(int cpufreq)
{
	int cpufreq_enum = -1;

	switch (cpufreq) {
	case 1190400 :
		cpufreq_enum = CPUFREQ_1200M;
		break;
	case 1094400 :
		cpufreq_enum = CPUFREQ_1100M;
		break;
	case 998400 :
		cpufreq_enum = CPUFREQ_1000M;
		break;
	case 800000 :
		cpufreq_enum = CPUFREQ_800M;
		break;
	case 533333 :
		cpufreq_enum = CPUFREQ_533M;
		break;
	case 400000 :
		cpufreq_enum = CPUFREQ_400M;
		break;
	case 200000 :
		cpufreq_enum = CPUFREQ_200M;
		break;
	}

	return cpufreq_enum;

}


void cpu_tester_enum_to_str(char *str, int type, int enum_value)
{
	if (type == CPU_FREQ_TEST) {
		switch (enum_value) {
		 	case CPUFREQ_1200M :
				strcpy(str, "1.2Ghz");
				break;
		 	case CPUFREQ_1100M :
				strcpy(str, "1.1Ghz");
				break;
		 	case CPUFREQ_1000M :
				strcpy(str, "1.0Ghz");
				break;
		 	case CPUFREQ_800M :
				strcpy(str, "800Mhz");
				break;
		 	case CPUFREQ_533M :
				strcpy(str, "533Mhz");
				break;
		 	case CPUFREQ_400M :
				strcpy(str, "400Mhz");
				break;
		 	case CPUFREQ_200M :
				strcpy(str, "200Mhz");
				break;
			case CPUFREQ_RANDOM:
				strcpy(str, "RANDOM");
				break;
		}
	}else if (type == CPU_IDLE_TEST) {
		switch (enum_value) {
		 	case CPUIDLE_C1 :
				strcpy(str, "C1");
				break;
		 	case CPUIDLE_C2 :
				strcpy(str, "C2");
				break;
		 	case CPUIDLE_C3 :
				strcpy(str, "C3");
				break;
			case CPUIDLE_RANDOM:
				strcpy(str, "RANDOM");
				break;
		}
	}

}

#endif

