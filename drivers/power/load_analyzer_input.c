/* drivers/power/load_analyzer_input.c */

#include <linux/input.h>

static unsigned int  input_rec_time_list_view(unsigned int start_cnt, unsigned int end_cnt
			, char *buf, unsigned int buf_size, unsigned int ret)
{
	unsigned  int i = 0, start_array_num, data_line, cnt=0;
	unsigned int end_array_num, start_array_num_for_time;

	start_array_num_for_time
	= cpu_load_freq_history_view[start_cnt].input_rec_history_cnt;
	start_array_num
	= (cpu_load_freq_history_view[start_cnt].input_rec_history_cnt+1)
			% input_rec_history_num;
	end_array_num
		= cpu_load_freq_history_view[end_cnt].input_rec_history_cnt;

	total_time = section_end_time - section_start_time;

	if (end_cnt == start_cnt+1) {
		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d] TOTAL SECTION TIME = %lld[ns]\n[%5d]~[%5d]/(%lld ~ %lld)\n\n"
			, end_cnt, total_time
			, (cpu_load_freq_history_view[start_cnt].input_rec_history_cnt\
							+ 1)	% cpu_task_history_num
			, (cpu_load_freq_history_view[end_cnt].input_rec_history_cnt\
							+ 1)	% cpu_task_history_num
			, cpu_load_freq_history_view[start_cnt].time_stamp
			, cpu_load_freq_history_view[end_cnt].time_stamp);
	} else {
		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d~%d] TOTAL SECTION TIME = %lld[ns]\n[%5d]~[%5d]/(%lld ~ %lld)\n\n"
			, get_index(start_cnt, input_rec_history_num, 1)
			, end_cnt, total_time
			, (cpu_load_freq_history_view[start_cnt].input_rec_history_cnt\
							+ 1)	% cpu_task_history_num
			, (cpu_load_freq_history_view[end_cnt].input_rec_history_cnt\
							+ 1)	% cpu_task_history_num
			, cpu_load_freq_history_view[start_cnt].time_stamp
			, cpu_load_freq_history_view[end_cnt].time_stamp);
	}

	end_array_num = get_index(end_array_num, input_rec_history_num, 2);

	if (end_array_num >= start_array_num_for_time)
		data_line = end_array_num -start_array_num_for_time + 1;
	else {
		data_line = (end_array_num + input_rec_history_num) \
				-start_array_num_for_time + 1;
	}
	cnt = start_array_num_for_time;

	for (i = 0; i < data_line; i++) {
		if (cnt > input_rec_history_num-1)
			cnt = 0;

		if (ret >= buf_size)
			break;

		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d] %lld %s [TYPE]%d [CODE]%d [value]%X\n", cnt
			, input_rec_history_view[cnt].time
			, input_rec_history_view[cnt].dev->name
			, input_rec_history_view[cnt].type
			, input_rec_history_view[cnt].code
			, input_rec_history_view[cnt].value);
		cnt++;
	}

	return ret;

}

void __slp_store_input_history(void *dev,
			       unsigned int type, unsigned int code, int value)
{
	unsigned int cnt ;
	struct input_dev *idev = dev;

	if (input_rec_history_onoff == 0)
		return ;

	if (strstr(idev->name, "accelerometer") != NULL)
		return ;

	if (++input_rec_history_cnt >= input_rec_history_num)
		input_rec_history_cnt = 0;
	cnt = input_rec_history_cnt;

	input_rec_history[cnt].time = cpu_clock(UINT_MAX);
	input_rec_history[cnt].dev = dev;
	input_rec_history[cnt].type = type;
	input_rec_history[cnt].code = code;
	input_rec_history[cnt].value = value;

}


int check_input_rec_valid_range(unsigned int start_cnt, unsigned int end_cnt)
{
	int ret = 0;
	unsigned long long t1, t2;
	unsigned int end_sched_cnt = 0, end_sched_cnt_margin;
	unsigned int load_cnt, last_load_cnt, overflow = 0;
	unsigned int cnt, search_cnt;
	unsigned int upset = 0;
	int cpu = 0;
	unsigned int i;

	t1 = cpu_load_freq_history_view[start_cnt].time_stamp;
	t2 = cpu_load_freq_history_view[end_cnt].time_stamp;

	if ((t2 <= t1) || (t1 == 0) || (t2 == 0)) {
		pr_info("[time error] t1=%lld t2=%lld\n", t1, t2);
		return WRONG_TIME_STAMP;
	}

	last_load_cnt = cpu_load_freq_history_view_cnt;

	cnt = cpu_load_freq_history_view[last_load_cnt].input_rec_history_cnt;
	t1 = cpu_task_history_view[cnt][cpu].time;
	search_cnt = cnt;
	for (i = 0;  i < cpu_task_history_num; i++) {
		search_cnt = get_index(search_cnt, cpu_task_history_num, 1);
		t2 = cpu_task_history_view[search_cnt][cpu].time;

		if (t2 < t1) {
			end_sched_cnt = search_cnt;
			break;
		}

		if (i >= cpu_task_history_num - 1)
			end_sched_cnt = cnt;
	}

	load_cnt = last_load_cnt;
	for (i = 0;  i < cpu_load_history_num; i++) {
		unsigned int sched_cnt, sched_before_cnt;
		unsigned int sched_before_cnt_margin;

		sched_cnt
			= cpu_load_freq_history_view[load_cnt]\
						.input_rec_history_cnt;
		load_cnt = get_index(load_cnt, cpu_load_history_num, -1);

		sched_before_cnt
			= cpu_load_freq_history_view[load_cnt]\
						.input_rec_history_cnt;

		if (sched_before_cnt > sched_cnt)
			upset++;

		end_sched_cnt_margin
			= get_index(end_sched_cnt, input_rec_history_num, 1);
		sched_before_cnt_margin
			= get_index(sched_before_cnt, input_rec_history_num, -1);

		/* "end_sched_cnt -1" is needed
		  *  because of calulating schedule time */
		if ((upset >= 2) || ((upset == 1)
			&& (sched_before_cnt_margin < end_sched_cnt_margin))) {
			overflow = 1;
			pr_err("[LA] overflow cpu=%d upset=%d sched_before_cnt_margin=%d" \
				"end_sched_cnt_margin=%d end_sched_cnt=%d" \
				"sched_before_cnt=%d sched_cnt=%d load_cnt=%d" \
				, cpu , upset, sched_before_cnt_margin
				, end_sched_cnt_margin, end_sched_cnt
				, sched_before_cnt, sched_cnt, load_cnt);
			break;
		}

		if (load_cnt == start_cnt)
			break;
	}

	if (overflow == 0)
		ret = 0;
	else {
		ret = OVERFLOW_ERROR;
		pr_info("[overflow error]\n");
	}
	return ret;
}

static ssize_t check_input_rec_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{


	return 0;
}

static ssize_t check_input_rec_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	set_cpu_load_freq_history_array_range(user_buf);

	input_rec_history_show_select_cpu =cpu_task_history_show_select_cpu;
	input_rec_history_show_start_cnt=cpu_task_history_show_start_cnt;
	input_rec_history_show_end_cnt = cpu_task_history_show_end_cnt;

	return count;
}


static int check_input_rec_detail_sub(char *buf, int buf_size)
{
	int ret = 0, i = 0, ret_check_valid = 0;
	unsigned int start_cnt = input_rec_history_show_start_cnt;
	unsigned int end_cnt = input_rec_history_show_end_cnt;

	ret_check_valid = check_input_rec_valid_range(start_cnt, end_cnt);
	if (ret_check_valid < 0)	{
		ret +=  snprintf(buf + ret, buf_size - ret
			, "[ERROR] Invalid range !!! err=%d\n"
			, ret_check_valid);
		pr_info("[ERROR] Invalid range !!! err=%d\n"
			, ret_check_valid);
	}

	ret += snprintf(buf + ret, buf_size - ret,
		"###########################################"
		"################ CPU %d ######################"
		"##########################################\n", i);

	ret = input_rec_time_list_view(start_cnt, end_cnt, buf, buf_size, ret);

	ret += snprintf(buf + ret, buf_size - ret ,"\n\n");

	return ret;
}

static ssize_t check_input_rec_detail(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,check_input_rec_detail_sub);

	return size_for_copy;
}


void input_rec_index_to_cnt(int *start_cnt, int *end_cnt, int start_index, int end_index)
{
	*start_cnt = get_index(cpu_load_freq_history_view[start_index].input_rec_history_cnt
						, input_rec_history_num, 1);

	*end_cnt = cpu_load_freq_history_view[end_index].input_rec_history_cnt;

	pr_info("*start_cnt=%d *end_cnt=%d\n", *start_cnt, *end_cnt);
}


void input_rec_str_to_index(int *start_index, int *end_index, char *str)
{
	int show_array_num = 0;
	char *p1;
	char cpy_buf[80] = {0,};

	p1 = strstr(str, "-");

	if (p1 != NULL) {
		strncpy(cpy_buf, str, sizeof(cpy_buf) - 1);
		*p1 = '\0';
		*start_index = get_index(atoi(cpy_buf) ,cpu_load_history_num ,-1);
		*end_index = atoi(p1+1);
	} else {
		show_array_num = atoi(str);
		*start_index = get_index(show_array_num, cpu_load_history_num, -1);
		*end_index = show_array_num;
	}

	pr_info("*start_index=%d *end_index=%d\n", *start_index, *end_index);
}

void input_rec_reproduce_exec(int usec, struct input_dev *dev, unsigned int type
							, unsigned int code, int value)
{
	usleep(usec);

	#if defined(CONFIG_SLP_MINI_TRACER)
	{
		char str[128]={0,};
		sprintf(str, "usec=%d name=%s type=%d code=%d value=%d\n"
					,usec, dev->name, type, code, value);
		kernel_mini_tracer_smp(str);
	}
	#endif

	input_event(dev, type, code, value);

}

static ssize_t input_rec_reproduce(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{

	int start_cnt, end_cnt, cnt, before_cnt;
	int start_index, end_index;
	int data_num, i;
	u64 time;

	input_rec_str_to_index(&start_index, &end_index, (char *)user_buf);

	input_rec_index_to_cnt(&start_cnt, &end_cnt, start_index, end_index);


	if (end_cnt >= start_cnt)
		data_num = end_cnt - start_cnt;
	else
		data_num = end_cnt + input_rec_history_num - start_cnt;

	pr_info("data_num=%d", data_num);

	cnt = start_cnt;
	for(i=0; i< data_num; i++) {
		before_cnt = get_index(cnt, input_rec_history_num, -1);

	#if defined(CONFIG_SLP_MINI_TRACER)
	{
		char str[64]={0,};
		sprintf(str, "cnt=%d before_cnt=%d\n", cnt, before_cnt);
		kernel_mini_tracer_smp(str);
	}
	#endif
		time = input_rec_history_view[cnt].time -input_rec_history_view[before_cnt].time;
		do_div(time , 1000);

		input_rec_reproduce_exec((int)time, input_rec_history_view[cnt].dev
			, input_rec_history_view[cnt].type
			, input_rec_history_view[cnt].code, input_rec_history_view[cnt].value);
		cnt++;
	}

	return count;
}


static const struct file_operations check_input_rec_fops = {
	.owner = THIS_MODULE,
	.read = check_input_rec_read,
	.write =check_input_rec_write,
};

static const struct file_operations check_input_rec_detail_fops = {
	.owner = THIS_MODULE,
	.read = check_input_rec_detail,
};

static const struct file_operations input_rec_reproduce_fops = {
	.owner = THIS_MODULE,
	.write =input_rec_reproduce,
};


void debugfs_input_rec(struct dentry *d)
{
	if (!debugfs_create_file("check_input_rec", 0600
		, d, NULL,&check_input_rec_fops))   \
			pr_err("%s : debugfs_create_file, error\n", "check_input_rec");
	if (!debugfs_create_file("check_input_rec_detail", 0600
		, d, NULL,&check_input_rec_detail_fops))   \
			pr_err("%s : debugfs_create_file, error\n", "check_input_rec_detail");
	if (!debugfs_create_file("input_rec_reproduce", 0600
		, d, NULL,&input_rec_reproduce_fops))   \
			pr_err("%s : debugfs_create_file, error\n", "input_rec_reproduce");
}




