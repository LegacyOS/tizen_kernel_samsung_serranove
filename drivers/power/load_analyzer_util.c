/* drivers/power/load_analyzer_util.c */

static int atoi(const char *str)
{
	int result = 0;
	int count = 0;
	if (str == NULL)
		return -1;
	while (str[count] && str[count] >= '0' && str[count] <= '9') {
		result = result * 10 + str[count] - '0';
		++count;
	}
	return result;
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

static int wrapper_for_debug_fs(char __user *buffer
				,size_t count, loff_t *ppos, int (*fn)(char *, int))
{

	static char *buf = NULL;
	int buf_size = (PAGE_SIZE * 256);
	unsigned int ret = 0, size_for_copy = count;
	static unsigned int rest_size = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		buf = vmalloc(buf_size);

		if (!buf)
			return -ENOMEM;

		ret = fn(buf, buf_size - PAGE_SIZE); /* PAGE_SIZE mean margin */

		if (ret <= count) {
			size_for_copy = ret;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size = ret -size_for_copy;
		}
	} else {
		if (rest_size <= count) {
			size_for_copy = rest_size;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size -= size_for_copy;
		}
	}

	if (size_for_copy >  0) {
		int offset = (int) *ppos;
		if (copy_to_user(buffer, buf + offset , size_for_copy)) {
			vfree(buf);
			return -EFAULT;
		}
		*ppos += size_for_copy;
	} else
		vfree(buf);

	return size_for_copy;
}

struct task_info_tag {
	struct task_struct *p_task;
	char comm[TASK_COMM_LEN];
	int pid;
};

int get_name_from_pid(char *task_name, int pid)
{
	#define MAX_LOOP_CNT	5000

	struct list_head *p;
	int ret = 0;
	int loop_cnt=0, found = 0;
	struct task_struct *p_curr_task, *p_start_task, *p_next_task;
	static struct task_info_tag last_task_info = {NULL, };

	if(pid == last_task_info.pid) {
		strcpy(task_name, last_task_info.comm);
		ret = 0;
		goto end;
	}

	p_start_task = get_current();

	list_for_each(p, &(p_start_task->tasks)) {
		p_curr_task = list_entry(p, struct task_struct, tasks);
		p_next_task = list_entry(p_curr_task->tasks.next, struct task_struct, tasks);

		if (p_curr_task->pid == pid) {
			strcpy(task_name, p_curr_task->comm);
			pr_info("pid %d = %s cnt=%d\n", pid, task_name, loop_cnt);
			found = 1;
			last_task_info.p_task = p_curr_task;
			last_task_info.pid =p_curr_task->pid;
			strcpy(last_task_info.comm, p_curr_task->comm);
			break;
		}

		if (++loop_cnt > MAX_LOOP_CNT) {
			pr_info("pid %d is not found cnt=%d\n", pid, loop_cnt);
			break;
		}
	}

	if (found == 0) {
		if(search_killed_task(pid, task_name) >= 0) {
			found = 1;
			last_task_info.pid = pid;
			strcpy(last_task_info.comm, task_name);
		} else {
			sprintf(task_name, "NOT found %d", pid);
		}
	}

	if (found == 1)
		ret = 0;
	else
		ret = -1;

end:
	return ret;
}


struct saved_load_factor_tag saved_load_factor;

void store_external_load_factor(int type, unsigned int data)
{
	switch (type) {

	case ACTIVE_APP_PID:
		saved_load_factor.active_app_pid = data;
		break;
	case NR_RUNNING_TASK:
		saved_load_factor.nr_running_task = data;
		break;
	case MIF_BUS_FREQ:
		saved_load_factor.mif_bus_freq = data;
		break;
	case MIF_BUS_LOAD:
		saved_load_factor.mif_bus_load = data;
		break;
	case INT_BUS_FREQ:
		saved_load_factor.int_bus_freq = data;
		break;
	case INT_BUS_LOAD:
		saved_load_factor.int_bus_load = data;
		break;
	case GPU_FREQ:
		saved_load_factor.gpu_freq = data;
		break;
	case GPU_UTILIZATION:
		saved_load_factor.gpu_utilization = data;
		break;
	case BATTERY_SOC:
		saved_load_factor.battery_soc = data;
		break;

	default:
		break;
	}
}


