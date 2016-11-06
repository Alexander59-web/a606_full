
#include "m4u_sec_systrace.h"

#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/div64.h>


#define SIZE_SYSTRACE_LOG   (sizeof(struct systrace_log))
#define NUM_SYSTRACE_LOG    (systrace_buffer_size/SIZE_SYSTRACE_LOG)

struct systrace_log {
    uint64_t timestamp;
    unsigned int thread_id;
    int type; /* 0: begin, 1: end */
    char name[16];
};


static unsigned int module_id = 0;
static char log_name[32];
static void *systrace_buffer = NULL;
static size_t systrace_buffer_size = 0;
static int start_item = -1;
static struct secure_callback_funcs *sec_cb_funcs = NULL;


static struct systrace_log* get_log_item(loff_t *pos)
{
    int n;
    struct systrace_log *log;

    n = (int)(*pos);
    n += start_item;

    if (n >= NUM_SYSTRACE_LOG)
        n -= NUM_SYSTRACE_LOG;
    log = (struct systrace_log *)(systrace_buffer + (n * SIZE_SYSTRACE_LOG));

    return log;
}

static void *systrace_seq_start(struct seq_file *m, loff_t *pos)
{
    struct systrace_log *log;

    if (start_item == -1)
        return NULL;

    if (*pos >= NUM_SYSTRACE_LOG)
        return NULL;

    log = get_log_item(pos);
    
    if (log->timestamp == 0)
        return NULL;

	return pos;
}

static void *systrace_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
    struct systrace_log *log;

    log = get_log_item(pos);

	(*pos)++;

    if (*pos >= NUM_SYSTRACE_LOG)
        return NULL;

    log = get_log_item(pos);

    if (log->timestamp == 0)
        return NULL;

	return pos;
}

static void systrace_seq_stop(struct seq_file *m, void *v)
{
    return;
}

static int systrace_seq_show(struct seq_file *m, void *v)
{
    int n = *(loff_t *)v;
    struct systrace_log *log;
    uint64_t second;
    uint32_t microsecond;

    n += start_item;
    if (n >= NUM_SYSTRACE_LOG)
        n -= NUM_SYSTRACE_LOG;
    log = (struct systrace_log *)(systrace_buffer + (n * SIZE_SYSTRACE_LOG));

    if (n == 0) {
        uint64_t timestamp = log->timestamp;
        timestamp -= 10;
        second = timestamp;
        microsecond = do_div(second, 1000000);
        seq_printf(m, "%16s-%-6u[%03u]%7llu.%06u: sched_switch: prev_comm=dummyProc"
                      "prev_pid=0 prev_prio=120 prev_state=S ==> next_comm=%s next_pid=%u next_prio=120\n",
                    "dummyProc", 0, module_id, second, microsecond, log_name, log->thread_id);
    }

    second = log->timestamp;
    microsecond = do_div(second, 1000000);

    if (log->type == 0) {
        seq_printf(m, "%16s-%-6u[%03u]%7llu.%06u: tracing_mark_write: B|%u|%s\n",
                    log_name, log->thread_id, module_id, second, microsecond, log->thread_id, log->name);
    } else {
        seq_printf(m, "%16s-%-6u[%03u]%7llu.%06u: tracing_mark_write: E\n",
                    log_name, log->thread_id, module_id, second, microsecond);
    }

	return 0;
}

static const struct seq_operations proc_systrace_seq_ops = {
	.start	= systrace_seq_start,
	.next	= systrace_seq_next,
	.stop	= systrace_seq_stop,
	.show	= systrace_seq_show,
};

static int proc_systrace_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &proc_systrace_seq_ops);
}

static ssize_t proc_systrace_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	struct seq_file *m = file->private_data;

    mutex_lock(&m->lock);

    if (size > 0) {
        char str;
        if (copy_from_user(&str, buf, sizeof(str))) {
            mutex_unlock(&m->lock);
            return -EFAULT;
        }

        switch (str) {
        case '1': /* enable */
            start_item = -1;
            if (systrace_buffer != NULL) {
                memset(systrace_buffer, 0, systrace_buffer_size);
            }
            sec_cb_funcs->enable_systrace(systrace_buffer, systrace_buffer_size);
            break;

        case '0': /* disable */
            start_item = -1;
            sec_cb_funcs->disable_systrace();
            break;

        case '2': /* pause */
        {
            unsigned int head;
            struct systrace_log *log;
            start_item = -1;
            
            sec_cb_funcs->pause_systrace(&head);
            if (head >= NUM_SYSTRACE_LOG) {
                /* invalid */
                break;
            }

            log = (struct systrace_log *)(systrace_buffer + (SIZE_SYSTRACE_LOG * head));
            if (log->timestamp != 0) {
                /* log buffer is full */
                start_item = head;
            } else {
                /* log buffer is not full */
                start_item = 0;
            }
        }
            break;

        case '3': /* resume */
            start_item = -1;
            sec_cb_funcs->resume_systrace();
            break;

        default: /* do nothing */
            break;
        }
    }

    mutex_unlock(&m->lock);

    return size;
}


static const struct file_operations proc_systrace_operations = {
	.open		= proc_systrace_open,
	.read		= seq_read,
	.write      = proc_systrace_write,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

int m4u_sec_systrace_init(const unsigned int id, const char *module,
        const char *name, const size_t buf_size, 
        struct secure_callback_funcs *callback)
{
    char proc_name[64];

    if ((module == NULL) || (name == NULL) || (callback == NULL) ||
            (callback->enable_systrace == NULL) || (callback->disable_systrace == NULL) ||
            (callback->pause_systrace == NULL) || (callback->resume_systrace == NULL)) {
        return -EINVAL;
    }

    systrace_buffer_size = ALIGN(buf_size, SIZE_SYSTRACE_LOG);
    //systrace_buffer_size = (buf_size+SIZE_SYSTRACE_LOG-1)/SIZE_SYSTRACE_LOG*SIZE_SYSTRACE_LOG;
    if (systrace_buffer_size == 0) {
        return -EINVAL;
    }

    systrace_buffer = kzalloc(systrace_buffer_size, GFP_KERNEL);
    if (systrace_buffer == NULL) {
        return -ENOMEM;
    }

    module_id = 8 + id;
    snprintf(proc_name, sizeof(proc_name), "systrace_%s", module);
    snprintf(log_name, sizeof(log_name), name);
    sec_cb_funcs = callback;

	proc_create(proc_name, S_IRUGO | S_IWUGO, NULL, &proc_systrace_operations);

	return 0;
}

void m4u_sec_systrace_deinit(const char *module)
{
    char proc_name[64];

    snprintf(proc_name, sizeof(proc_name), "systrace_%s", module);
    remove_proc_entry(proc_name, NULL);

    if (systrace_buffer != NULL) {
        vfree(systrace_buffer);
    }

    systrace_buffer = NULL;
    systrace_buffer_size = 0;
    start_item = -1;
    sec_cb_funcs = NULL;
    module_id = 0;
}


