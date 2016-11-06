#ifndef __DRSVPCOMMAND_H__
#define __DRSVPCOMMAND_H__

/**
 * Command id definitions for FID_DR_OPERATE
 */
#define CMD_DRV_CONFIG_OVL_LAYER 1
#define CMD_DRV_SWITCH_OVL_LAYER 2
#define CMD_DRV_DUMP_OVL_REGISTER 3
#define CMD_DRV_DUMP_DISP_REGISTER 4
#define CMD_DRV_REGISTER_IRQ 5
#define CMD_DRV_UNREGISTER_IRQ 6

#define CMD_DRV_DUMMY 11

#define CMD_DRV_DUMMY_SET_SECURE_DEBUG_BUFFER 12
#define CMD_DRV_DUMMY_INIT_SECURE_DEBUG_BUFFER 13
#define CMD_DRV_DUMMY_DISABLE_SECURE_DEBUG_LAYER 14
#define CMD_DRV_DUMMY_ENABLE_SECURE_DEBUG_LAYER 15

#define CMD_DRV_CONFIG_WDMA_ADDR 16
#define CMD_DRV_CONFIG_RDMA 17

#define CMD_DRV_NOTIFY_RDMA_CONFIG 18

#define CMD_DRV_NOTIFY_WDMA_MVA 19
#define CMD_DRV_NOTIFY_RDMA_MVA 20

#define CMD_DRV_CONFIG_RDMA_ADDR 21

#define CMD_DRV_INFORM_WDMA_CONFIG_SHARED 22

#define CMD_DRV_CONFIG_WDMA 23

#define CMD_DRV_START_ISRH 24
#define CMD_DRV_STOP_ISRH 25

#define CMD_DRV_MAP_INTR_INFO 26

#define CMD_DRV_CLEAR_RDMA_INTR 27
#define CMD_DRV_CLEAR_WDMA_INTR 28

#define CMD_DRV_CONFIG_OVL_ROI 29

#define CMD_DRV_ENABLE_SECURE_SYSTRACE 30
#define CMD_DRV_DISABLE_SECURE_SYSTRACE 31
#define CMD_DRV_MAP_SYSTRACE_BUF 32
#define CMD_DRV_PAUSE_SECURE_SYSTRACE 33
#define CMD_DRV_RESUME_SECURE_SYSTRACE 34

#define CMD_DRV_RESET_OVL 35

typedef struct {
    unsigned int intr;
    unsigned int reg_val;

    unsigned int intr_rdma;
    unsigned int reg_val_rdma;
} DISP_INTR_INFO;


#endif // __DRSVPCOMMAND_H__
