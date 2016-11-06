#ifndef __SECURE_SYSTRACE_H_
#define __SECURE_SYSTRACE_H_

#include <linux/types.h>

enum {
    SEC_SYSTRACE_MODULE_CMDQ = 0,
    SEC_SYSTRACE_MODULE_VENC = 1,
    SEC_SYSTRACE_MODULE_VDEC = 2,
    SEC_SYSTRACE_MODULE_DDP = 3,
    SEC_SYSTRACE_MODULE_M4U = 4,
};

struct secure_callback_funcs {
    int (*enable_systrace)(void *buf, size_t size);
    int (*disable_systrace)(void);
    int (*pause_systrace)(unsigned int *head);
    int (*resume_systrace)(void);
};

int m4u_sec_systrace_init(const unsigned int id, const char *module,
        const char *name, const size_t buf_size, 
        struct secure_callback_funcs *callback);

#endif // __SECURE_SYSTRACE_H_
