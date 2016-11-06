# drivers/dispsys
include $(MTK_ROOT_BUILD)/Makefile

ccflags-y += -I$(MTK_PATH_SOURCE)/drivers/video/ 							\
             -I$(MTK_PATH_PLATFORM)/../../common/kernel/drivers/video/ 		\
             -I$(MTK_PATH_PLATFORM)/drivers/video/

ifeq ($(strip $(MTK_SEC_VIDEO_PATH_SUPPORT)),yes)
ifeq ($(strip $(TRUSTONIC_TEE_SUPPORT)),yes)
ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/gud/MobiCoreKernelApi/include \
             -I$(MTK_PATH_PLATFORM)/drivers/gud/MobiCoreKernelApi/public
endif
endif

# for secure cmdp
ifeq ($(strip $(MTK_SEC_VIDEO_PATH_SUPPORT)),yes)
ifeq ($(strip $(TRUSTONIC_TEE_SUPPORT)),yes)
ccflags-y += -I$(MTK_PATH_SOURCE)/../platform/${ARCH_MTK_PLATFORM}/kernel/core/include/trustzone/cmdq/cmdq_sec_dr \
             -I$(MTK_PATH_SOURCE)/../platform/${ARCH_MTK_PLATFORM}/kernel/core/include/trustzone/cmdq/cmdq_sec_tl
endif
endif

# for secure ddp
ifeq ($(strip $(MTK_SEC_VIDEO_PATH_SUPPORT)),yes)
ifeq ($(strip $(TRUSTONIC_TEE_SUPPORT)),yes)
ccflags-y += -I$(MTK_PATH_SOURCE)/../platform/${ARCH_MTK_PLATFORM}/kernel/core/include/trustzone/svp/TlSvp/Locals/Code/public \
             -I$(MTK_PATH_SOURCE)/../platform/${ARCH_MTK_PLATFORM}/kernel/core/include/trustzone/svp/DrSvp/Locals/Code/public
endif
endif

# for setting t-play handle
ifeq ($(strip $(MTK_SEC_VIDEO_PATH_SUPPORT)),yes)
ifeq ($(strip $(TRUSTONIC_TEE_SUPPORT)),yes)
ccflags-y += -I$(MTK_PATH_SOURCE)/../platform/${ARCH_MTK_PLATFORM}/kernel/core/include/trustzone/sec/Tlsec/inc
endif
endif

# for secure memory
ifeq ($(strip $(MTK_SEC_VIDEO_PATH_SUPPORT)),yes)
ifeq ($(strip $(TRUSTONIC_TEE_SUPPORT)),yes)
ccflags-y += -I$(MTK_PATH_SOURCE)/../platform/${ARCH_MTK_PLATFORM}/kernel/core/include/trustzone/utils/tlutils \
             -I$(MTK_PATH_SOURCE)/../platform/${ARCH_MTK_PLATFORM}/kernel/core/include/trustzone/utils/drutils
endif
endif

obj-y += ddp_drv.o
obj-y += ddp_ovl.o
obj-y += ddp_bls.o
obj-y += ddp_color.o
# obj-y += ddp_tdshp.o
obj-y += ddp_rdma.o
obj-y += ddp_wdma.o
# obj-y += ddp_scl.o
# obj-y += ddp_rot.o
obj-y += ddp_debug.o
obj-y += ddp_cmdq.o
obj-y += ddp_cmdq_debug.o
obj-y += ddp_cmdq_sec.o
#obj-y += ddp_cmdq_record.o
#obj-y += ddp_cmdq_test.o
# obj-y += ddp_dpfd.o
obj-y += ddp_aal.o
obj-y += data_rgb888_64x64.o
obj-y += data_rgb888_64x64_golden.o
ifeq ($(strip $(MTK_SEC_VIDEO_PATH_SUPPORT)),yes)
ifeq ($(strip $(TRUSTONIC_TEE_SUPPORT)),yes)
obj-y += ddp_svp.o
obj-y += ddp_path_svp.o
obj-y += secmem_secure_systrace.o
obj-y += dapc_secure_systrace.o
else
obj-y += ddp_path.o
endif
else
obj-y += ddp_path.o
endif
# EOF
