#include <linux/string.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <asm/uaccess.h>

#include "ddp_debug.h"
#include "ddp_hal.h"
#include "ddp_ovl.h"
#include "ddp_reg.h"

#include <debug.h>
#include <disp_svp.h>

#include <tlSvp_Api.h> // trustlet inter face for TLC
#include <drSvp_Api.h>
#include <tlSvpCommand.h>

#include "mobicore_driver_api.h"

#define SYSTRACE_PROFILING 1
#include "secure_systrace.h"

#define DRV_DBG_UUID { { 2, 0xf, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }

//------------------------------------------------------------------------------
// definitions
//------------------------------------------------------------------------------
#define TPLAY_DEV_NAME        "t-play-drv-ddp"

//------------------------------------------------------------------------------
// debugging definitions: DON'T CHANGE!!
//------------------------------------------------------------------------------
#define RDMA_SINGLE_SESSION
//#define DISABLE_OVL_SECURE_INTR
//#define DISABLE_DAPC_PROTECTION
//#define TPLAY_DUMP_PA_DEBUG
#define SEC_BUFFER // don't change this: we need to use secure buffer(s).

//------------------------------------------------------------------------------
// Global
//------------------------------------------------------------------------------
static const struct mc_uuid_t uuid = SVP_TL_DBG_UUID;
static const struct mc_uuid_t uuid_dr = SVP_DRV_DBG_UUID;
static const struct mc_uuid_t uuid_mem = DRV_DBG_UUID;

static struct mc_session_handle tlSessionHandle;
static struct mc_session_handle drHandle;
static struct mc_session_handle rdmaSessionHandle;
static struct mc_session_handle rdmaHandle;

static const uint32_t mc_deviceId = MC_DEVICE_ID_DEFAULT;

static tciMessage_t *pTci = NULL;
static dciMessage_t *pDci = NULL;
static tciMessage_t *pRdmaTci = NULL;
static dciMessage_t *pRdmaDci = NULL;

volatile unsigned int is_secure_port = 0;

volatile unsigned int is_decouple_secure_port_wdma = 0;
volatile unsigned int is_decouple_secure_port_rdma = 0;
volatile unsigned int need_release_rdma_secure_port = 0;
volatile unsigned int need_release_wdma_secure_port = 0;

//#define SECURE_PORT_FLAG_LOCK
static spinlock_t secure_port_lock;
static unsigned long _lock_flag = 0;
unsigned int secure_port_flag(void)
{
    unsigned int result;
#ifdef SECURE_PORT_FLAG_LOCK
    spin_lock_irqsave(&secure_port_lock, _lock_flag);
#endif
    result = is_secure_port;
#ifdef SECURE_PORT_FLAG_LOCK
    spin_unlock_irqrestore(&secure_port_lock, _lock_flag);
#endif
    return result;
}

static void set_secure_port_flag(unsigned int secure)
{
#ifdef SECURE_PORT_FLAG_LOCK
    spin_lock_irqsave(&secure_port_lock, _lock_flag);
#endif
    is_secure_port = secure;    
#ifdef SECURE_PORT_FLAG_LOCK
    spin_unlock_irqrestore(&secure_port_lock, _lock_flag);
#endif
}

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------
static int late_init_session(void);
static void close_session(void);

static enum mc_result late_init_session_tl(void);
static enum mc_result late_init_session_drv(void);
static enum mc_result late_open_mobicore_device(void);
static enum mc_result late_init_session_rdma_tl(void);

static void close_session_tl(void);
static void close_session_drv(void);
static void close_mobicore_device(void);
static void close_session_tl_rdma(void);
static void close_session_drv_rdma(void);

//------------------------------------------------------------------------------
// Type
//------------------------------------------------------------------------------
typedef struct
{
    unsigned int layer_en[OVL_LAYER_NUM];
    unsigned int addr[OVL_LAYER_NUM];
    unsigned int size[OVL_LAYER_NUM];
} OVL_LAYER_INFO;

//------------------------------------------------------------------------------
// handle address for t-play
//------------------------------------------------------------------------------
static unsigned int tplay_handle_virt_addr;
static dma_addr_t handle_pa;

// allocate a fixed physical memory address for storing tplay handle
unsigned int init_tplay_handle(void)
{
    void *va;
    va = dma_alloc_coherent(NULL, sizeof(handle_pa), &handle_pa, GFP_KERNEL);
    if (0 == va)
    {
        DISP_MSG("[SVP] failed to allocate handle_pa \n");
    }
    tplay_handle_virt_addr = (unsigned int)va;
    return tplay_handle_virt_addr;
}

static int write_tplay_handle(unsigned int handle_value)
{
    unsigned int *x;
    if (0 != tplay_handle_virt_addr)
    {
        DISP_DBG("[SVP] write_tplay_handle 0x%x \n", handle_value);
        x = (unsigned int *)tplay_handle_virt_addr;
        *x = handle_value;
    }
    return 0;
}

unsigned int get_tplay_handle(void)
{
    return tplay_handle_virt_addr;
}

//------------------------------------------------------------------------------
// set the handle address to t-play driver
//------------------------------------------------------------------------------
#include "tlcApisec.h"
// TODO: where is the header for MC_UUID_SDRV_DEVINE?
#define MC_UUID_SDRV_DEFINE { 0x05, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
static const struct mc_uuid_t MC_UUID_HACC = {MC_UUID_SDRV_DEFINE};

static struct mc_session_handle tplaySessionHandle;
static dapc_tciMessage_t *pTplayTci  = NULL;

static int open_tplay_driver_connection(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    if (tplaySessionHandle.session_id != 0)
    {
        DISP_MSG("tplay TDriver session already created\n");
        return 0;
    }

    DISP_DBG("=============== late init tplay TDriver session ===============\n");
    do
    {
        late_open_mobicore_device();

        /* Allocating WSM for DCI */
        mcRet = mc_malloc_wsm(mc_deviceId, 0, sizeof(dapc_tciMessage_t), (uint8_t **) &pTplayTci, 0);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_malloc_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            return -1;
        }

        /* Open session the TDriver */
        memset(&tplaySessionHandle, 0, sizeof(tplaySessionHandle));
        tplaySessionHandle.device_id = mc_deviceId;
        mcRet = mc_open_session(&tplaySessionHandle,
                                &MC_UUID_HACC,
                                (uint8_t *) pTplayTci,
                                (uint32_t) sizeof(dapc_tciMessage_t));
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_open_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            // if failed clear session handle
            memset(&tplaySessionHandle, 0, sizeof(tplaySessionHandle));
            return -1;
        }
    } while (false);

    return (MC_DRV_OK == mcRet) ? 0 : -1;
}

static int close_tplay_driver_connection(void)
{
    enum mc_result mcRet = MC_DRV_OK;
    DISP_DBG("=============== close tplay TDriver session ===============\n");
        /* Close session*/
    if (tplaySessionHandle.session_id != 0) // we have an valid session
    {
        mcRet = mc_close_session(&tplaySessionHandle);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_close_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            memset(&tplaySessionHandle, 0, sizeof(tplaySessionHandle)); // clear handle value anyway
            return -1;
        }
    }
    memset(&tplaySessionHandle, 0, sizeof(tplaySessionHandle));

    mcRet = mc_free_wsm(mc_deviceId, (uint8_t *)pTplayTci);
    if (MC_DRV_OK != mcRet)
    {
        DISP_ERR("mc_free_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
        return -1;
    }

    return 0;
}

// return 0 for success and -1 for error
static int set_tplay_handle_addr_request(void)
{
    int ret = 0;
    enum mc_result mcRet = MC_DRV_OK;
    DISP_DBG("[SVP] set_tplay_handle_addr_request \n");

    open_tplay_driver_connection();
    if (tplaySessionHandle.session_id == 0)
    {
        DISP_ERR("[SVP] invalid tplay session \n");
        return -1;
    }

    DISP_DBG("[SVP] handle_pa=0x%x \n", handle_pa);
    /* set other TCI parameter */
    pTplayTci->tplay_handle_low_addr = handle_pa;
    pTplayTci->tplay_handle_high_addr = 0;
    /* set TCI command */
    pTplayTci->cmd.header.commandId = CMD_TPLAY_REQUEST;

    /* notify the trustlet */
    DISP_DBG("[SVP] notify Tlsec trustlet CMD_TPLAY_REQUEST \n");
    mcRet = mc_notify(&tplaySessionHandle);
    if (MC_DRV_OK != mcRet)
    {
        DISP_ERR("[SVP] mc_notify failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
        ret = -1;
        goto _notify_to_trustlet_fail;
    }

    /* wait for response from the trustlet */
    if (MC_DRV_OK != mc_wait_notification(&tplaySessionHandle, 20000))
    {
        DISP_ERR("[SVP] mc_wait_notification 20s timeout: %d @%s line %d\n", mcRet, __func__, __LINE__);
        ret = -1;
        goto _notify_from_trustlet_fail;
    }

    DISP_DBG("[SVP] CMD_TPLAY_REQUEST result=%d, return code=%d\n", pTplayTci->result, pTplayTci->rsp.header.returnCode);

_notify_from_trustlet_fail:
_notify_to_trustlet_fail:
    close_tplay_driver_connection();

    return ret;
}

#ifdef TPLAY_DUMP_PA_DEBUG
static int dump_tplay_physcial_addr(void)
{
    DISP_DBG("[SVP] dump_tplay_physcial_addr \n");
    int ret = 0;   
    enum mc_result mcRet = MC_DRV_OK;
    open_tplay_driver_connection();
    if (tplaySessionHandle.session_id == 0)
    {
        DISP_ERR("[SVP] invalid tplay session \n");
        return -1;
    }

    /* set TCI command */
    pTplayTci->cmd.header.commandId = CMD_TPLAY_DUMP_PHY; 
   
	/* notify the trustlet */        
	DISP_MSG("[SVP] notify Tlsec trustlet CMD_TPLAY_DUMP_PHY \n");
	mcRet = mc_notify(&tplaySessionHandle);        
	if (MC_DRV_OK != mcRet)        
	{            
		DISP_ERR("[SVP] mc_notify failed: %d @%s line %d\n", mcRet, __func__, __LINE__); 
        ret = -1;
		goto _notify_to_trustlet_fail;
	}
    
	/* wait for response from the trustlet */        
	if (MC_DRV_OK != mc_wait_notification(&tplaySessionHandle, 20000))        
	{            
		DISP_ERR("[SVP] mc_wait_notification 20s timeout: %d @%s line %d\n", mcRet, __func__, __LINE__);
		ret = -1;
		goto _notify_from_trustlet_fail;
	}    

    DISP_DBG("[SVP] CMD_TPLAY_DUMP_PHY result=%d, return code=%d\n", pTplayTci->result, pTplayTci->rsp.header.returnCode);
    
_notify_from_trustlet_fail:
_notify_to_trustlet_fail:
    close_tplay_driver_connection();
    return ret;    
}
#endif

static struct task_struct *threadId;
static struct cdev tplay_cdev;

//------------------------------------------------------------------------------
// layer flag: the status of current ovl layer security setting.
//   stores the security flag each time the OVL layer is configurated.
//------------------------------------------------------------------------------
static unsigned int ovl_layer_flag[OVL_LAYER_NUM] = {0};
DEFINE_MUTEX(ovlLayerFlagMutex);

void update_layer_flag(unsigned int layer, unsigned int secure)
{
    unsigned int secure_flag = secure & 0xffff; // the bit 0~15 is used for secured layer flag; bit 16~31 for alignment shift

    mutex_lock(&ovlLayerFlagMutex);
    if (layer >= OVL_LAYER_NUM || layer < 0)
    {
        mutex_unlock(&ovlLayerFlagMutex);
        return; // nothing was done
    }
    if (ovl_layer_flag[layer] != secure_flag)
    {
        DISP_DBG("[SVP] update secure flag %d >> %d for layer #%d",
                ovl_layer_flag[layer], secure_flag, layer);
        ovl_layer_flag[layer] = secure_flag;
    }
    mutex_unlock(&ovlLayerFlagMutex);
}

unsigned int is_ovl_secured(void)
{
    unsigned int secure = 0;
    unsigned int i;

    mutex_lock(&ovlLayerFlagMutex);
    for (i = 0; i < OVL_LAYER_NUM; i++)
    {
        secure |= ovl_layer_flag[i];
    }
    mutex_unlock(&ovlLayerFlagMutex);
    return secure;
}

void disp_path_update_ovl_secure(unsigned int layer, unsigned int secure)
{
    update_layer_flag(layer, secure);
}

//------------------------------------------------------------------------------
// last ovl layer info: when switching OVL to secure port, the current layers, 
//   which are accessing normal buffers, should have their address mapped to 
//   secure page table, avoiding m4u translation fault.
//------------------------------------------------------------------------------
static OVL_LAYER_INFO last_ovl_layer_info = {{0},{0},{0}};
static DEFINE_MUTEX(LastLayerInfoMutex);

void acquireLastLayerInfoMutex(void)
{
    //mutex_lock(&LastLayerInfoMutex);
}

void releaseLastLayerInfoMutex(void)
{
    //mutex_unlock(&LastLayerInfoMutex);
}

static int get_ovl_layer_info(OVL_LAYER_INFO *pInfo)
{
    unsigned int i;
    for (i = 0; i < OVL_LAYER_NUM; i++)
    {
        pInfo->layer_en[i] = last_ovl_layer_info.layer_en[i];
        pInfo->addr[i] = last_ovl_layer_info.addr[i];
        pInfo->size[i] = last_ovl_layer_info.size[i];
        DISP_DBG("[SVP] get_ovl_layer_info layer=%d, layer_en=%d, addr=0x%x, size=%d \n",
                i, last_ovl_layer_info.layer_en[i], last_ovl_layer_info.addr[i], last_ovl_layer_info.size[i]);
    }
    return 0;
}

// each time the dirty config is set for OVL, driver call this to update OVL layer information
int update_ovl_layer_info(OVL_CONFIG_STRUCT *pConfig)
{
    unsigned int layer;
    layer = pConfig->layer;
    last_ovl_layer_info.layer_en[layer] = pConfig->layer_en;
    last_ovl_layer_info.addr[layer] = pConfig->addr;
    last_ovl_layer_info.size[layer] = (pConfig->src_h + pConfig->src_y) * pConfig->src_pitch;
    return 0;
}

//------------------------------------------------------------------------------
// local functions
//------------------------------------------------------------------------------

// return 0 for success and -1 for failure
static int _notifyTrustletCommandValue(uint32_t command, uint32_t *value1, uint32_t *value2)
{
    enum mc_result ret = MC_DRV_OK;
    late_init_session_tl();
    if (tlSessionHandle.session_id == 0)
    {
        DISP_ERR("invalid session handle of Trustlet @%s line %d\n", __func__, __LINE__);
        return -1;
    }

    /* prepare data */
    memset(pTci, 0, sizeof(tciMessage_t));
    pTci->cmdSvp.header.commandId = command;
    pTci->Value1 = *value1;
    pTci->Value2 = *value2;

    DISP_DBG("notify Trustlet CMD: %d \n", command);
    /* Notify the trustlet */
    ret = mc_notify(&tlSessionHandle);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("Trustlet CMD: %d wait notification \n", command);
    /* Wait for response from the trustlet */
    ret = mc_wait_notification(&tlSessionHandle, MC_INFINITE_TIMEOUT);
    //ret = mc_wait_notification(&tlSessionHandle, 20);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    *value1 = pTci->Value1;
    *value2 = pTci->Value2;

    DISP_DBG("Trustlet CMD: %d done \n", command);

    return 0;
}

static int notifyTrustletCommandValue(uint32_t command, uint32_t value1, uint32_t value2)
{
    return _notifyTrustletCommandValue(command, &value1, &value2);
}

static DEFINE_MUTEX(dciMutex);

static int _notifyTDriverCommandValueWithTimeout(uint32_t command, uint32_t *value1, uint32_t *value2, int timeout)
{
    enum mc_result ret = MC_DRV_OK;
    //late_init_session_drv();
    if (drHandle.session_id == 0)
    {
        DISP_ERR("invalid session handle of TDriver @%s line %d\n", __func__, __LINE__);
        return -1;
    }

    mutex_lock(&dciMutex);
    /* prepare data */
    memset(pDci, 0, sizeof(dciMessage_t));
    pDci->command.header.commandId = command;
    pDci->Value1 = *value1;
    pDci->Value2 = *value2;

    DISP_DBG("notify TDriver CMD: %d \n", command);
    /* Notify the trustlet */
    ret = mc_notify(&drHandle);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        mutex_unlock(&dciMutex);
        return -1;
    }

    DISP_DBG("TDriver CMD: %d wait notification \n", command);
    /* Wait for response from the trustlet */
    //ret = mc_wait_notification(&drHandle, MC_INFINITE_TIMEOUT);
    ret = mc_wait_notification(&drHandle, timeout);
    if (MC_DRV_OK != ret)
    {
        //DISP_ERR("mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        MMProfileLog(MTKFB_MMP_Events.DciSessionError, MMProfileFlagPulse);
        mutex_unlock(&dciMutex);
        return -1;
    }

    *value1 = pDci->Value1;
    *value2 = pDci->Value2;

    DISP_DBG("TDriver CMD: %d done \n", command);
    mutex_unlock(&dciMutex);
    return 0;
}

static int notifyTDriverCommandValueWithTimeout(uint32_t command, uint32_t value1, uint32_t value2, int timeout)
{
    return _notifyTDriverCommandValueWithTimeout(command, &value1, &value2, timeout);
}

static int notifyTDriverCommandValue(uint32_t command, uint32_t value1, uint32_t value2)
{
#if 0
    enum mc_result ret = MC_DRV_OK;
    late_init_session_drv();
    if (drHandle.session_id == 0)
    {
        DISP_ERR("invalid session handle of TDriver @%s line %d\n", __func__, __LINE__);
        return -1;
    }

    /* prepare data */
    memset(pDci, 0, sizeof(dciMessage_t));
    pDci->command.header.commandId = command;
    pDci->Value1 = value1;
    pDci->Value2 = value2;

    DISP_DBG("notify TDriver CMD: %d \n", command);
    /* Notify the trustlet */
    ret = mc_notify(&drHandle);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("TDriver CMD: %d wait notification \n", command);
    /* Wait for response from the trustlet */
    //ret = mc_wait_notification(&drHandle, MC_INFINITE_TIMEOUT);
    //ret = mc_wait_notification(&drHandle, 40);
    ret = mc_wait_notification(&drHandle, MC_NO_TIMEOUT);
    if (MC_DRV_OK != ret)
    {
        //DISP_ERR("mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("TDriver CMD: %d done \n", command);

    return 0;
#endif
    return notifyTDriverCommandValueWithTimeout(command, value1, value2, MC_NO_TIMEOUT);
}

static int notifyTDriverCommandValueInfinite(uint32_t command, uint32_t value1, uint32_t value2)
{
    return notifyTDriverCommandValueWithTimeout(command, value1, value2, MC_INFINITE_TIMEOUT);
}

// return 0 for success and -1 for failure
// we should have an valid session first
// the corresponding session and pTci
static int notifyTrustletCommandValueToSession(struct mc_session_handle *session,
        tciMessage_t *pTci, uint32_t command, uint32_t value1, uint32_t value2)
{
    enum mc_result ret = MC_DRV_OK;

    if (session->session_id == 0)
    {
        DISP_ERR("invalid session handle @%s line %d\n", __func__, __LINE__);
        return -1;
    }

    /* prepare data */
    memset(pTci, 0, sizeof(tciMessage_t));
    pTci->cmdSvp.header.commandId = command;
    pTci->Value1 = value1;
    pTci->Value2 = value2;

    DISP_DBG("notify Trustlet CMD: %d \n", command);
    /* Notify the trustlet */
    ret = mc_notify(session);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("Trustlet CMD: %d wait notification \n", command);
    /* Wait for response from the trustlet */
    ret = mc_wait_notification(session, MC_INFINITE_TIMEOUT);
    //ret = mc_wait_notification(session, 20);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("Trustlet CMD: %d done \n", command);

    return 0;
}

static int notifyTDriverCommandValueToSession(struct mc_session_handle *session,
        dciMessage_t *pDci, uint32_t command, uint32_t value1, uint32_t value2)
{
    enum mc_result ret = MC_DRV_OK;

    if (session->session_id == 0)
    {
        DISP_ERR("invalid session handle @%s line %d\n", __func__, __LINE__);
        return -1;
    }

    /* prepare data */
    memset(pDci, 0, sizeof(dciMessage_t));
    pDci->command.header.commandId = command;
    pDci->Value1 = value1;
    pDci->Value2 = value2;

    DISP_DBG("notify TDriver CMD: %d \n", command);
    /* Notify the trustlet */
    ret = mc_notify(session);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("TDriver CMD: %d wait notification \n", command);
    /* Wait for response from the trustlet */
    //ret = mc_wait_notification(session, MC_INFINITE_TIMEOUT);
    ret = mc_wait_notification(session, 40);
    if (MC_DRV_OK != ret)
    {
        //DISP_ERR("mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("TDriver CMD: %d done \n", command);

    return 0;
}

/* This is our main function which will be called by the init function once the
 * module is loaded and our tplay thread is started.
 * For the example here we do everything, open, close, and call a function in the
 * driver. However you can implement this to wait for event from the driver, or
 * to do whatever is required.
 */
int mainThread(void * uarg) {
    int ret = 0;
    return ret;
}

/*This is where we handle the IOCTL commands coming from user space*/

static long tplay_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    DISP_MSG("Entering ioctl switch \n");

    switch (cmd) {

    default:
        return -ENOTTY;
    }

    return ret;
}

static struct file_operations tplay_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = tplay_ioctl,
};

static struct secure_callback_funcs secure_systrace_funcs;

extern void secmem_systrace_init(void);
extern void secmem_systrace_deinit(void);
extern void dapc_systrace_init(void);
extern void dapc_systrace_deinit(void);

// This is the initial point of entry and first code to execute after the insmod command is sent
static int __init tlckTPlay_init(void)
{
    dev_t devno;
    int err;
    static struct class *tplay_class;

    DISP_MSG("=============== Running TPlay Kernel TLC : ddp_svp driver ===============\n");


    err = alloc_chrdev_region(&devno, 0, 1, TPLAY_DEV_NAME);
    if (err) {
        DISP_ERR(KERN_ERR "Unable to allocate TPLAY device number\n");
        return err;
    }

    cdev_init(&tplay_cdev, &tplay_fops);
    tplay_cdev.owner = THIS_MODULE;

    err = cdev_add(&tplay_cdev, devno, 1);
    if (err) {
        DISP_ERR(KERN_ERR "Unable to add Tplay char device\n");
        unregister_chrdev_region(devno, 1);
        return err;
    }

    tplay_class = class_create(THIS_MODULE, "tplay_ddp_svp");
    device_create(tplay_class, NULL, devno, NULL, TPLAY_DEV_NAME);

    // make sure the session handle cleared
    memset(&tlSessionHandle, 0, sizeof(struct mc_session_handle));
    memset(&drHandle, 0, sizeof(struct mc_session_handle));
    memset(&rdmaSessionHandle, 0, sizeof(struct mc_session_handle));
    memset(&rdmaHandle, 0, sizeof(struct mc_session_handle));
    /* Create the TlcTplay Main thread */
    threadId = kthread_run(mainThread, NULL, "dci_thread");
    if (!threadId) {
        DISP_ERR(KERN_ERR "Unable to start tplay main thread\n");
        return -1;
    }

    systrace_init(SEC_SYSTRACE_MODULE_DDP, "ddp", "SWd_tlSvp", 0x10000, &secure_systrace_funcs);
    secmem_systrace_init();
    dapc_systrace_init();

    return 0;
}

static void __exit tlckTPlay_exit(void)
{
    DISP_MSG("=============== Unloading TPlay Kernel TLC : ddp_svp driver ===============\n");

    systrace_deinit("ddp");
    secmem_systrace_deinit();
    dapc_systrace_deinit();

    close_session();
}

module_init(tlckTPlay_init);
module_exit(tlckTPlay_exit);


MODULE_AUTHOR("Trustonic-MediaTek");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("TPlay Kernel TLC ddp_svp");

//------------------------------------------------------------------------------
// local functions
//------------------------------------------------------------------------------
static int late_init_session(void)
{
    late_init_session_tl();
    late_init_session_drv();
    //late_init_session_rdma_tl();
    return 0;
}

static enum mc_result late_init_session_tl(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    late_open_mobicore_device();
    if (tlSessionHandle.session_id != 0)
    {
        DISP_DBG("trustlet session already created\n");
        return MC_DRV_OK;
    }

    DISP_DBG("=============== late init trustlet session ===============\n");
    do
    {
        /* Allocating WSM for TCI */
        mcRet = mc_malloc_wsm(mc_deviceId, 0, sizeof(tciMessage_t), (uint8_t **) &pTci, 0);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_malloc_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            break;
        }

        /* Open session the trustlet */
        memset(&tlSessionHandle, 0, sizeof(tlSessionHandle));
        tlSessionHandle.device_id = mc_deviceId;
        mcRet = mc_open_session(&tlSessionHandle,
                                &uuid,
                                (uint8_t *) pTci,
                                (uint32_t) sizeof(tciMessage_t));
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_open_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            // if failed clear session handle
            memset(&tlSessionHandle, 0, sizeof(tlSessionHandle));
            break;
        }
    } while (false);

    return mcRet;
}

static enum mc_result late_init_session_drv(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    late_open_mobicore_device();
    if (drHandle.session_id != 0)
    {
        DISP_DBG("TDriver session already created\n");
        return MC_DRV_OK;
    }

    DISP_DBG("=============== late init TDriver session ===============\n");
    do
    {
        /* Allocating WSM for DCI */
        mcRet = mc_malloc_wsm(mc_deviceId, 0, sizeof(dciMessage_t), (uint8_t **) &pDci, 0);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_malloc_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            break;
        }

        /* Open session the TDriver */
        memset(&drHandle, 0, sizeof(drHandle));
        drHandle.device_id = mc_deviceId;
        mcRet = mc_open_session(&drHandle,
                                &uuid_dr,
                                (uint8_t *) pDci,
                                (uint32_t) sizeof(dciMessage_t));
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_open_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            // if failed clear session handle
            memset(&drHandle, 0, sizeof(drHandle));
            break;
        }
    } while (false);

    return mcRet;
}

static enum mc_result late_init_session_rdma_tl(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    late_open_mobicore_device();
    if (rdmaSessionHandle.session_id != 0)
    {
        DISP_DBG("rdma trustlet session already created\n");
        return MC_DRV_OK;
    }

    DISP_DBG("=============== late init rdma trustlet session ===============\n");
    do
    {
        /* Allocating WSM for TCI */
        mcRet = mc_malloc_wsm(mc_deviceId, 0, sizeof(tciMessage_t), (uint8_t **) &pRdmaTci, 0);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_malloc_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            break;
        }

        /* Open session the trustlet */
        memset(&rdmaSessionHandle, 0, sizeof(rdmaSessionHandle));
        rdmaSessionHandle.device_id = mc_deviceId;
        mcRet = mc_open_session(&rdmaSessionHandle,
                                &uuid,
                                (uint8_t *) pRdmaTci,
                                (uint32_t) sizeof(tciMessage_t));
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_open_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            // if failed clear session handle
            memset(&rdmaSessionHandle, 0, sizeof(rdmaSessionHandle));
            break;
        }
    } while (false);

    return mcRet;
}

volatile static unsigned int opened_device = 0;
static enum mc_result late_open_mobicore_device(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    if (0 == opened_device)
    {
        DISP_DBG("=============== open mobicore device ===============\n");
        /* Open MobiCore device */
        mcRet = mc_open_device(mc_deviceId);
        if (MC_DRV_ERR_INVALID_OPERATION == mcRet)
        {
            // skip false alarm when the mc_open_device(mc_deviceId) is called more than once
            DISP_DBG("mc_open_device already done \n");
        }
        else if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_open_device failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            return mcRet;
        }
        opened_device = 1;
    }
    return MC_DRV_OK;
}

static void close_session(void)
{
    close_session_tl();
    close_session_drv();
    close_session_tl_rdma();
    close_mobicore_device();
}

static void close_session_tl(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    DISP_DBG("=============== close trustlet session ===============\n");
        /* Close session*/
    if (tlSessionHandle.session_id != 0) // we have an valid session
    {
        mcRet = mc_close_session(&tlSessionHandle);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_close_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            return;
        }
    }

    mcRet = mc_free_wsm(mc_deviceId, (uint8_t *)pTci);
    if (MC_DRV_OK != mcRet)
    {
        DISP_ERR("mc_free_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
        return;
    }
}

static void close_session_drv(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    DISP_DBG("=============== close TDriver session ===============\n");
        /* Close session*/
    if (drHandle.session_id != 0) // we have an valid session
    {
        mcRet = mc_close_session(&drHandle);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_close_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            return;
        }
    }

    mcRet = mc_free_wsm(mc_deviceId, (uint8_t *)pDci);
    if (MC_DRV_OK != mcRet)
    {
        DISP_ERR("mc_free_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
        return;
    }
}

static void close_session_tl_rdma(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    DISP_DBG("=============== close rdma trustlet session ===============\n");
        /* Close session*/
    if (rdmaSessionHandle.session_id != 0) // we have an valid session
    {
        mcRet = mc_close_session(&rdmaSessionHandle);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_close_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            return;
        }
    }

    mcRet = mc_free_wsm(mc_deviceId, (uint8_t *)pRdmaTci);
    if (MC_DRV_OK != mcRet)
    {
        DISP_ERR("mc_free_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
        return;
    }
}

static void close_session_drv_rdma(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    DISP_DBG("=============== close rdma TDriver session ===============\n");
        /* Close session*/
    if (rdmaHandle.session_id != 0) // we have an valid session
    {
        mcRet = mc_close_session(&rdmaHandle);
        if (MC_DRV_OK != mcRet)
        {
            DISP_ERR("mc_close_session failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
            return;
        }
    }

    mcRet = mc_free_wsm(mc_deviceId, (uint8_t *)pRdmaDci);
    if (MC_DRV_OK != mcRet)
    {
        DISP_ERR("mc_free_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
        return;
    }
}

static void close_mobicore_device(void)
{
    enum mc_result mcRet = MC_DRV_OK;

    DISP_DBG("=============== close mobicore device ===============\n");
        /* Close MobiCore device */
    mcRet = mc_close_device(mc_deviceId);
    if (MC_DRV_OK != mcRet)
    {
        DISP_ERR("mc_close_device failed: %d @%s, line %d\n", mcRet, __func__, __LINE__);
        return;
    }
}

//------------------------------------------------------------------------------
// cache the ovl configurations, and they are sent to trustlet while calling
//   {config_wdma_secure}
//------------------------------------------------------------------------------
static OVL_CONFIG_STRUCT ovl_config_cache[OVL_LAYER_NUM] = {{0},{0},{0},{0}};

// return 0 for success and -1 for failure
int config_ovl_layer_secure(void* config, uint32_t len, uint32_t secure)
{
    OVL_CONFIG_STRUCT *p_ovl_config = (OVL_CONFIG_STRUCT *)config;
    // all from OVL_CONFIG_STRUCT
    unsigned int layer = p_ovl_config->layer;

    ovl_config_cache[layer].layer = layer;
    ovl_config_cache[layer].layer_en = p_ovl_config->layer_en;
    ovl_config_cache[layer].source = (uint32_t)p_ovl_config->source;
    ovl_config_cache[layer].fmt = p_ovl_config->fmt;
    ovl_config_cache[layer].addr = p_ovl_config->addr;
    ovl_config_cache[layer].vaddr = p_ovl_config->vaddr;
    ovl_config_cache[layer].src_x = p_ovl_config->src_x;
    ovl_config_cache[layer].src_y = p_ovl_config->src_y;
    ovl_config_cache[layer].src_w = p_ovl_config->src_w;
    ovl_config_cache[layer].src_h = p_ovl_config->src_h;
    ovl_config_cache[layer].src_pitch = p_ovl_config->src_pitch;
    ovl_config_cache[layer].dst_x = p_ovl_config->dst_x;
    ovl_config_cache[layer].dst_y = p_ovl_config->dst_y;
    ovl_config_cache[layer].dst_w = p_ovl_config->dst_w;
    ovl_config_cache[layer].dst_h = p_ovl_config->dst_h;
    ovl_config_cache[layer].keyEn = p_ovl_config->keyEn;
    ovl_config_cache[layer].key = p_ovl_config->key;
    ovl_config_cache[layer].aen = p_ovl_config->aen;
    ovl_config_cache[layer].alpha = (uint32_t)p_ovl_config->alpha;
    ovl_config_cache[layer].isTdshp = p_ovl_config->isTdshp;
    ovl_config_cache[layer].isDirty = p_ovl_config->isDirty;
    ovl_config_cache[layer].buff_idx = (uint32_t)p_ovl_config->buff_idx;
    ovl_config_cache[layer].identity = (uint32_t)p_ovl_config->identity;
    ovl_config_cache[layer].connected_type = (uint32_t)p_ovl_config->connected_type;
    ovl_config_cache[layer].security = p_ovl_config->security;

    return 0;
}

int config_ovl_roi_secure(unsigned int w, unsigned int h)
{
    //return notifyTDriverCommandValueInfinite(
        //CMD_SVP_DRV_CONFIG_OVL_ROI, w, h);

    return notifyTrustletCommandValue(
        CMD_SVP_CONFIG_OVL_ROI, w, h);
}

static struct mc_bulk_map mapped_info;
// return 0 for success and -1 for failure.
// map the current ovl buffers to secure world page table so that it can be accessed after
static int map_ovl_config(void)
{
    enum mc_result ret = MC_DRV_OK;

    // the current ovl setting is stored in the [last_ovl_layer_info] variable
    // note: we assume the vmalloc allocation is successful
    OVL_LAYER_INFO *p_last_ovl_info = vmalloc(sizeof(OVL_LAYER_INFO));
    memset(p_last_ovl_info, 0, sizeof(OVL_LAYER_INFO));
    get_ovl_layer_info(p_last_ovl_info);

    // buffer mapped to secure world
    ret = mc_map(&tlSessionHandle, (void*)p_last_ovl_info, sizeof(OVL_LAYER_INFO), &mapped_info);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_map failed: %d @%s line %d\n", ret, __func__, __LINE__);
        vfree(p_last_ovl_info);
        return -1;
    }

    /* prepare data */
    memset(pTci, 0, sizeof(tciMessage_t));
    pTci->cmdSvp.header.commandId = CMD_SVP_MAP_OVL_CONFIG;
    pTci->Value1 = (uint32_t)(mapped_info.secure_virt_addr);
    pTci->Value2 = mapped_info.secure_virt_len;

    DISP_DBG("notify Trustlet CMD_SVP_MAP_OVL_CONFIG \n");
    /* Notify the trustlet */
    ret = mc_notify(&tlSessionHandle);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        ret = mc_unmap(&tlSessionHandle, (void*)p_last_ovl_info, &mapped_info);
        if (MC_DRV_OK != ret)
        {
            DISP_ERR("mc_unmap failed: %d @%s line %d\n", ret, __func__, __LINE__);
            /* continue anyway */
        }
        vfree(p_last_ovl_info);
        return -1;
    }

    DISP_DBG("Trustlet CMD_SVP_MAP_OVL_CONFIG wait notification \n");
    /* Wait for response from the trustlet */
    ret = mc_wait_notification(&tlSessionHandle, MC_INFINITE_TIMEOUT);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        ret = mc_unmap(&tlSessionHandle, (void*)p_last_ovl_info, &mapped_info);
        if (MC_DRV_OK != ret)
        {
            DISP_ERR("mc_unmap failed: %d @%s line %d\n", ret, __func__, __LINE__);
            /* continue anyway */
        }
        vfree(p_last_ovl_info);
        return -1;
    }

    DISP_DBG("Trustlet CMD_SVP_MAP_OVL_CONFIG done \n");
    // now unmap the buffer and free it
    ret = mc_unmap(&tlSessionHandle, (void*)p_last_ovl_info, &mapped_info);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_unmap failed: %d @%s line %d\n", ret, __func__, __LINE__);
        /* continue anyway */;
    }
    vfree(p_last_ovl_info);
    return 0;
}

static int map_ovl_current_mva(void)
{
    enum mc_result ret = MC_DRV_OK;
    unsigned int i;

    // the current ovl setting is stored in the [last_ovl_layer_info] variable
    // note: we assume the vmalloc allocation is successful
    /*OVL_LAYER_INFO *p_last_ovl_info = vmalloc(sizeof(OVL_LAYER_INFO));
    memset(p_last_ovl_info, 0, sizeof(OVL_LAYER_INFO));
    get_ovl_layer_info(p_last_ovl_info);

    // buffer mapped to secure world
    ret = mc_map(&tlSessionHandle, (void*)p_last_ovl_info, sizeof(OVL_LAYER_INFO), &mapped_info);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_map failed: %d @%s line %d\n", ret, __func__, __LINE__);
        vfree(p_last_ovl_info);
        return -1;
    }*/

    /* prepare data */
    memset(pTci, 0, sizeof(tciMessage_t));
    pTci->cmdSvp.header.commandId = CMD_SVP_MAP_OVL_CURRENT_MVA;
    for (i = 0; i < OVL_LAYER_NUM; i++)
    {
        pTci->ovl_layer_en[i] = last_ovl_layer_info.layer_en[i];
        pTci->ovl_addr[i] = last_ovl_layer_info.addr[i];
        pTci->ovl_size[i] = last_ovl_layer_info.size[i];
        DISP_DBG("[SVP] map_ovl_current_mva layer=%d, layer_en=%d, addr=0x%x, size=%d \n",
                i, last_ovl_layer_info.layer_en[i], last_ovl_layer_info.addr[i], last_ovl_layer_info.size[i]);
    }

    DISP_DBG("notify Trustlet CMD_SVP_MAP_OVL_CURRENT_MVA \n");
    /* Notify the trustlet */
    ret = mc_notify(&tlSessionHandle);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
#if 0
        ret = mc_unmap(&tlSessionHandle, (void*)p_last_ovl_info, &mapped_info);
        if (MC_DRV_OK != ret)
        {
            DISP_ERR("mc_unmap failed: %d @%s line %d\n", ret, __func__, __LINE__);
            /* continue anyway */
        }
        vfree(p_last_ovl_info);
#endif
        return -1;
    }

    DISP_DBG("Trustlet CMD_SVP_MAP_OVL_CURRENT_MVA wait notification \n");
    /* Wait for response from the trustlet */
    ret = mc_wait_notification(&tlSessionHandle, MC_INFINITE_TIMEOUT);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
#if 0
        ret = mc_unmap(&tlSessionHandle, (void*)p_last_ovl_info, &mapped_info);
        if (MC_DRV_OK != ret)
        {
            DISP_ERR("mc_unmap failed: %d @%s line %d\n", ret, __func__, __LINE__);
            /* continue anyway */
        }
        vfree(p_last_ovl_info);
#endif
        return -1;
    }

    DISP_DBG("Trustlet CMD_SVP_MAP_OVL_CURRENT_MVA done \n");

#if 0
    // now unmap the buffer and free it
    ret = mc_unmap(&tlSessionHandle, (void*)p_last_ovl_info, &mapped_info);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_unmap failed: %d @%s line %d\n", ret, __func__, __LINE__);
        /* continue anyway */;
    }
    vfree(p_last_ovl_info);
#endif
    return 0;
}

static int start_isrh_secure(void)
{
#ifndef DISABLE_OVL_SECURE_INTR
    DISP_MSG("[SVP] start_isrh_secure \n");
    //notifyTrustletCommandValue(CMD_SVP_START_ISRH, 0, 0);
    notifyTDriverCommandValueWithTimeout(CMD_SVP_DRV_START_ISRH, 0, 0, 40);
#endif
    return 0;
}

static int stop_isrh_secure(void)
{
#ifndef DISABLE_OVL_SECURE_INTR
    DISP_MSG("[SVP] stop_isrh_secure \n");
    //notifyTrustletCommandValue(CMD_SVP_STOP_ISRH, 0, 0);
    notifyTDriverCommandValueWithTimeout(CMD_SVP_DRV_STOP_ISRH, 0, 0, 40);
#endif
    return 0;
}

int map_decouple_mva(void);
// this is called once when the display driver enters secure mode
int prepare_display_secure(void)
{
    DISP_MSG("[SVP] prepare_display_secure \n");
	MMProfileLog(MTKFB_MMP_Events.StartSecure, MMProfileFlagStart);
    late_init_session();

    if (tlSessionHandle.session_id == 0)
    {
        DISP_ERR("invalid session handle of Trustlet @%s line %d\n", __func__, __LINE__);
        return -1;
    }
    if (drHandle.session_id == 0)
    {
        DISP_ERR("invalid session handle of TDriver @%s line %d\n", __func__, __LINE__);
        return -1;
    }

    //if (-1 == map_ovl_config())
    if (-1 == map_ovl_current_mva())
    {
        return -1;
    }

    if (-1 == map_decouple_mva())
    {
        return -1;
    }

    start_isrh_secure();
	MMProfileLog(MTKFB_MMP_Events.StartSecure, MMProfileFlagEnd);

    return 0;
}

int release_display_secure(void)
{
    DISP_MSG("[SVP] release_display_secure \n");
    stop_isrh_secure();

    return 0;
}

//------------------------------------------------------------------------------
// HAL interface implementation
//------------------------------------------------------------------------------
static int set_rdma_secure_port(unsigned int secure);
static int set_wdma_secure_port(unsigned int secure);
static int set_ovl_secure_port(unsigned int secure);
volatile static unsigned int is_device_apc_ovl = 0;
volatile unsigned int is_device_apc_wdma = 0;
volatile unsigned int is_device_apc_rdma = 0;
volatile unsigned int need_release_rdma_dapc = 0;
int switch_rdma_apc(unsigned int secure);
int disp_path_update_secure_port(void)
{
    // RDMA
    if (is_decouple_secure_port_rdma == 0 && need_release_rdma_secure_port == 1)
    {
        DISP_MSG("[SVP] switch rdma to non-secure port \n");
        set_rdma_secure_port(0);
        need_release_rdma_secure_port = 0;
    }

    if (is_device_apc_rdma == 1 && need_release_rdma_dapc == 1)
    {
        switch_rdma_apc(0);
        need_release_rdma_dapc = 0;
    }

    return 0;
}

int switch_ovl_apc(unsigned int secure);
int switch_wdma_apc(unsigned int secure);
int switch_rdma_apc(unsigned int secure);
int switch_ovl_secure_port(unsigned int secure);
#if 0
int disp_path_update_decouple_secure_port(void)
{
    // WDMA
    if (is_decouple_secure_port_wdma == 0 && need_release_wdma_secure_port == 1)
    {
        DISP_MSG("[SVP] switch wdma to non-secure port \n");
        set_wdma_secure_port(0);
        need_release_wdma_secure_port = 0;
    }

    // OVL
    acquireLastLayerInfoMutex();
    if (secure_port_flag() == 1 && is_ovl_secured() == DISP_NORMAL_BUFFER)
    {
	MMProfileLog(MTKFB_MMP_Events.StopSecure, MMProfileFlagStart);
        /*switch_ovl_apc(0);
        switch_wdma_apc(0);
        switch_rdma_apc(0);*/
        notifyTrustletCommandValue(CMD_SVP_RELEASE_ALL_APC, 0, 0);
        is_device_apc_ovl = 0;
        is_device_apc_wdma = 0;
        is_device_apc_rdma = 0;

        switch_ovl_secure_port(0);
        release_display_secure();
	MMProfileLog(MTKFB_MMP_Events.StopSecure, MMProfileFlagEnd);
    }

    /*if (is_ovl_secured() == DISP_SECURE_BUFFER)
    {
        switch_ovl_apc(1);
    }*/
    releaseLastLayerInfoMutex();

    return 0;
}
#endif
int disp_path_notify_tplay_handle(unsigned int handle_value)
{
    static int executed = 0; // this function can execute only once
    if (0 == executed)
    {
        set_tplay_handle_addr_request();
        executed = 1;
    }
    write_tplay_handle(handle_value);
#ifdef TPLAY_DUMP_PA_DEBUG
    dump_tplay_physcial_addr();
#endif // TPLAY_DUMP_PA_DEBUG
    return 0;
}

//------------------------------------------------------------------------------
// handle decouple settings: WDMA and RDMA
//------------------------------------------------------------------------------
#define MAX_BUFFER_COUNT 3
static unsigned int existed_decouple_mva[MAX_BUFFER_COUNT] = {0};
static unsigned int existed_mva_index = 0;

// check if the given addr is already in the existed_decouple_mva[MAX_BUFFER_COUNT] array.
// return 1: exist in the array. otherwise 0.
static unsigned int check_existed_mva(unsigned int addr)
{
    unsigned int i;
    for (i = 0; i < MAX_BUFFER_COUNT; i++)
    {
        if (existed_decouple_mva[i] == addr)
        {
            // found a match
            return 1;
        }
    }

    // otherwise, no matching found
    return 0;
}

// the total size of the pre-reserved decouple buffer (include MAX_BUFFER_COUNT blocks)
static unsigned int ovl_ram_size = 0;
static unsigned int cal_ovl_ram_size(void)
{
    if (0 == ovl_ram_size && existed_mva_index >= MAX_BUFFER_COUNT)
    {
        unsigned int min = existed_decouple_mva[0];
        unsigned int max = existed_decouple_mva[0];
        unsigned int i;
        for (i = 1; i < MAX_BUFFER_COUNT; i++)
        {
            if (min > existed_decouple_mva[i])
            {
                min = existed_decouple_mva[i];
            }
            if (max < existed_decouple_mva[i])
            {
                max = existed_decouple_mva[i];
            }
        }
        ovl_ram_size = (max - min) / (MAX_BUFFER_COUNT - 1) * MAX_BUFFER_COUNT;
    }
    return ovl_ram_size;
}

// get the index of the addr, which in the pre-reserved decouple buffer existed_decouple_mva[MAX_BUFFER_COUNT] array.
// return -1: the given address is not a pre-reserved decouple buffer mva, or the array is invalid (not full).
//   otherwise the index (0 ~ (MAX_BUFFER_COUNT - 1))
static int get_mva_index(unsigned int addr)
{
    int i;

    if (existed_mva_index < MAX_BUFFER_COUNT)
    {
        DISP_ERR("[SVP] decouple mva storage invalid, existed_mva_index=%d MAX_BUFFER_COUNT=%d \n",
            existed_mva_index, MAX_BUFFER_COUNT);
        return -1;
    }

    for (i = 0; i < MAX_BUFFER_COUNT; i++)
    {
        if (existed_decouple_mva[i] == addr)
        {
            // found a match
            return i;
        }
    }

    // otherwise, no matching found
    return -1;
}

// the display drive call this to tell SVP module about the pre-reserved decouple buffer it used.
int notify_decouple_mva(unsigned int mva)
{
    if (existed_mva_index < MAX_BUFFER_COUNT && 0 == check_existed_mva(mva))
    {
        // store the mva
        existed_decouple_mva[existed_mva_index] = mva;
        existed_mva_index += 1;
        DISP_MSG("[SVP] existed_decouple_mva [0x%x] [0x%x] [0x%x] \n",
                existed_decouple_mva[0], existed_decouple_mva[1], existed_decouple_mva[2]);
    }

    return 0;
}

unsigned int decouple_buffer_handles[MAX_BUFFER_COUNT] = {0};
// return 0 for success and -1 for failure
int prepare_decouple_buffer_handle(unsigned int w, unsigned int h, unsigned int bpp,
                                   unsigned int *pHandle, unsigned int cnt)
{
    enum mc_result ret = MC_DRV_OK;
    late_init_session();

    /* prepare data */
    memset(pTci, 0, sizeof(tciMessage_t));
    pTci->cmdSvp.header.commandId = CMD_SVP_PREPARE_DECOUPLE_BUFFER_HANDLE;
    pTci->Value1 = w * h * bpp; // the size of each buffer
    pTci->Value2 = 0;

    DISP_DBG("[SVP] notify Trustlet CMD_SVP_PREPARE_DECOUPLE_BUFFER_HANDLE \n");
    /* Notify the trustlet */
    ret = mc_notify(&tlSessionHandle);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("[SVP] Trustlet CMD_SVP_PREPARE_DECOUPLE_BUFFER_HANDLE wait notification \n");
    /* Wait for response from the trustlet */
    ret = mc_wait_notification(&tlSessionHandle, MC_INFINITE_TIMEOUT);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("[SVP] mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("[SVP] Trustlet CMD_SVP_PREPARE_DECOUPLE_BUFFER_HANDLE done \n");

    // the result int the tci message was the allocated handle.
    *(unsigned int *)(pHandle + 0) = pTci->ResultData;
    *(unsigned int *)(pHandle + 1) = pTci->Value1;
    *(unsigned int *)(pHandle + 2) = pTci->Value2;
    // also keep a copy of handle values in SVP module.
    decouple_buffer_handles[0] = pTci->ResultData;
    decouple_buffer_handles[1] = pTci->Value1;
    decouple_buffer_handles[2] = pTci->Value2;
    DISP_MSG("[SVP] prepare_decouple_buffer_handle #0=0x%x #1=0x%x #2=0x%x \n",
            decouple_buffer_handles[0], decouple_buffer_handles[1], decouple_buffer_handles[2]);

    return 0;
}

static unsigned int convert_mva_secure(unsigned int mva)
{
    int index;
    index = get_mva_index(mva);
    if (-1 == index)
    {
        // the given mva is not a decouple buffer mva, we don't know what's it now
        // maybe: 1) a normal mva which is not a decouple buffer 2) a secure handle for wfd
        return mva;
    }

    if (0 == decouple_buffer_handles[index])
    {
        // for some reason, the decouple_buffer_handles (secure buffer) was not
        //   successfully prepared.
        return mva;
    }

    return decouple_buffer_handles[index];
}

unsigned int handles[3];
// prepare secure decouple buffers. the allocated buffers are returned as handles.
int disp_path_query_primary_handle(unsigned int w, 
                                   unsigned int h, 
                                   unsigned int bpp, 
                                   unsigned int *pHandle,
                                   unsigned int buf_num)
{
    int ret = 0;
    int i = 0;

    if ((pHandle == NULL) || (buf_num == 0) || (buf_num > 10) || (bpp > 4))
    {
        DISP_ERR("[SVP] disp_path_query_primary_handle parameter invalid, w=%d, h=%d, bpp=%d, pHandle=0x%x, buf_num=%d \n",
                 w, h, bpp, (unsigned int)pHandle, buf_num);
        return -1;
    }

    ret = prepare_decouple_buffer_handle(w, h, bpp, pHandle, buf_num);
    if (ret != 0)
    {
        DISP_ERR("[SVP] disp_path_query_primary_handle fail! ret=%d \n", ret);
        return -1;
    }
    else  // print handle for debug
    {
        for (i = 0; i < buf_num; i++)
        {
            DISP_MSG("[SVP] disp_path_query_primary_handle return #%d 0x%x, \n",
                    i, *(pHandle + i));
        }
    }

    return 0;
}

int config_wdma_secure(void* config, uint32_t len)
{
    enum mc_result ret = MC_DRV_OK;
    unsigned int i;
    WDMA_CONFIG_STRUCT *p_wdma_config = NULL;

    late_init_session_tl();
    if (tlSessionHandle.session_id == 0)
    {
        DISP_ERR("[SVP] invalid trsutlet session handle @%s line %d\n", __func__, __LINE__);
        return -1;
    }

    /* prepare data */
    memset(pTci, 0, sizeof(tciMessage_t));
    pTci->cmdSvp.header.commandId = CMD_SVP_CONFIG_WDMA;

    p_wdma_config = (WDMA_CONFIG_STRUCT *)config;

    pTci->wdma_idx = p_wdma_config->idx;
    pTci->wdma_inputFormat = p_wdma_config->inputFormat;
    pTci->wdma_srcWidth = p_wdma_config->srcWidth;
    pTci->wdma_srcHeight = p_wdma_config->srcHeight;
    pTci->wdma_clipX = p_wdma_config->clipX;
    pTci->wdma_clipY = p_wdma_config->clipY;
    pTci->wdma_clipWidth = p_wdma_config->clipWidth;
    pTci->wdma_clipHeight = p_wdma_config->clipHeight;
    pTci->wdma_outputFormat = p_wdma_config->outputFormat;
#ifndef SEC_BUFFER
    pTci->wdma_dstAddress = p_wdma_config->dstAddress;
#else
    pTci->wdma_dstAddress = convert_mva_secure(p_wdma_config->dstAddress);
#endif
    pTci->wdma_dstWidth = p_wdma_config->dstWidth;
    pTci->wdma_dstPitch = p_wdma_config->dstPitch;
    pTci->wdma_dstPitchUV = p_wdma_config->dstPitchUV;
    pTci->wdma_useSpecifiedAlpha = p_wdma_config->useSpecifiedAlpha;
    pTci->wdma_alpha = p_wdma_config->alpha;

    // use the cached ovl configurations
    for (i = 0; i < OVL_LAYER_NUM; i++)
    {
        pTci->layer[i] = ovl_config_cache[i].layer;
        pTci->layer_en[i] = ovl_config_cache[i].layer_en;
        pTci->source[i] = (uint32_t)ovl_config_cache[i].source;
        pTci->fmt[i] = ovl_config_cache[i].fmt;
        pTci->addr[i] = ovl_config_cache[i].addr;
        pTci->vaddr[i] = ovl_config_cache[i].vaddr;
        pTci->src_x[i] = ovl_config_cache[i].src_x;
        pTci->src_y[i] = ovl_config_cache[i].src_y;
        pTci->src_w[i] = ovl_config_cache[i].src_w;
        pTci->src_h[i] = ovl_config_cache[i].src_h;
        pTci->src_pitch[i] = ovl_config_cache[i].src_pitch;
        pTci->dst_x[i] = ovl_config_cache[i].dst_x;
        pTci->dst_y[i] = ovl_config_cache[i].dst_y;
        pTci->dst_w[i] = ovl_config_cache[i].dst_w;
        pTci->dst_h[i] = ovl_config_cache[i].dst_h;
        pTci->keyEn[i] = ovl_config_cache[i].keyEn;
        pTci->key[i] = ovl_config_cache[i].key;
        pTci->aen[i] = ovl_config_cache[i].aen;
        pTci->alpha[i] = (uint32_t)ovl_config_cache[i].alpha;
        pTci->isTdshp[i] = ovl_config_cache[i].isTdshp;
        pTci->isDirty[i] = ovl_config_cache[i].isDirty;
        pTci->buff_idx[i] = (uint32_t)ovl_config_cache[i].buff_idx;
        pTci->identity[i] = (uint32_t)ovl_config_cache[i].identity;
        pTci->connected_type[i] = (uint32_t)ovl_config_cache[i].connected_type;
        pTci->security[i] = ovl_config_cache[i].security;
    }
    // clear ovl configuration cache!
    memset(ovl_config_cache, 0, sizeof(ovl_config_cache));

    DISP_DBG("[SVP] notify Trustlet CMD_SVP_CONFIG_WDMA \n");
    /* Notify the trustlet */
    ret = mc_notify(&tlSessionHandle);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("[SVP] mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("[SVP] Trustlet CMD_SVP_CONFIG_WDMA wait notification \n");
    /* Wait for response from the trustlet */
    ret = mc_wait_notification(&tlSessionHandle, MC_INFINITE_TIMEOUT);
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("[SVP] mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }
    DISP_DBG("[SVP] Trustlet CMD_SVP_CONFIG_WDMA done \n");

    return 0;
}

extern unsigned int gUltraLevel;
extern unsigned int gEnableUltra;

// return 0 for success and -1 for failure
int config_rdma_secure(void* config, uint32_t len)
{
    enum mc_result ret = MC_DRV_OK;
    RDMA_CONFIG_STRUCT *p_rdma_config = NULL;
#ifdef RDMA_SINGLE_SESSION
    late_init_session_rdma_tl();
#else
    late_init_session();
#endif

#ifdef RDMA_SINGLE_SESSION
    if (rdmaSessionHandle.session_id == 0)
#else
    if (tlSessionHandle.session_id == 0)
#endif
    {
#ifdef RDMA_SINGLE_SESSION
        DISP_ERR("[SVP] invalid rdma session handle @%s line %d\n", __func__, __LINE__);
#else
        DISP_ERR("[SVP] invalid trustlet session handle @%s line %d\n", __func__, __LINE__);
#endif
        return -1;
    }

    /* prepare data */
#ifdef RDMA_SINGLE_SESSION
    memset(pRdmaTci, 0, sizeof(tciMessage_t));
    pRdmaTci->cmdSvp.header.commandId = CMD_SVP_CONFIG_RDMA;

    p_rdma_config = (RDMA_CONFIG_STRUCT *)config;

    pRdmaTci->idx = p_rdma_config->idx;
    pRdmaTci->mode = (uint32_t)(p_rdma_config->mode);
    pRdmaTci->inputFormat = (uint32_t)(p_rdma_config->inputFormat);
#ifndef SEC_BUFFER
    pRdmaTci->address = p_rdma_config->address;
#else
    pRdmaTci->address = convert_mva_secure(p_rdma_config->address);
#endif
    pRdmaTci->pitch = p_rdma_config->pitch;
    pRdmaTci->isByteSwap = (uint32_t)(p_rdma_config->isByteSwap);
    pRdmaTci->outputFormat = (uint32_t)(p_rdma_config->outputFormat);
    pRdmaTci->width = p_rdma_config->width;
    pRdmaTci->height = p_rdma_config->height;
    pRdmaTci->isRGBSwap = (uint32_t)(p_rdma_config->isRGBSwap);

    pRdmaTci->ultraLevel = gUltraLevel;
    pRdmaTci->enableUltra = gEnableUltra;
#else
    //memset(pRdmaTci, 0, sizeof(tciMessage_t));
    pTci->cmdSvp.header.commandId = CMD_SVP_CONFIG_RDMA;

    p_rdma_config = (RDMA_CONFIG_STRUCT *)config;

    pTci->idx = p_rdma_config->idx;
    pTci->mode = (uint32_t)(p_rdma_config->mode);
    pTci->inputFormat = (uint32_t)(p_rdma_config->inputFormat);
#ifndef SEC_BUFFER
    pTci->address = p_rdma_config->address;
#else
    pTci->address = convert_mva_secure(p_rdma_config->address);
#endif
    pTci->pitch = p_rdma_config->pitch;
    pTci->isByteSwap = (uint32_t)(p_rdma_config->isByteSwap);
    pTci->outputFormat = (uint32_t)(p_rdma_config->outputFormat);
    pTci->width = p_rdma_config->width;
    pTci->height = p_rdma_config->height;
    pTci->isRGBSwap = (uint32_t)(p_rdma_config->isRGBSwap);

    pTci->ultraLevel = gUltraLevel;
    pTci->enableUltra = gEnableUltra;
#endif

    DISP_DBG("[SVP] notify Trustlet CMD_SVP_CONFIG_RDMA \n");
    /* Notify the trustlet */
#ifdef RDMA_SINGLE_SESSION
    ret = mc_notify(&rdmaSessionHandle);
#else
    ret = mc_notify(&tlSessionHandle);
#endif
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("[SVP] mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }

    DISP_DBG("[SVP] Trustlet CMD_SVP_CONFIG_RDMA wait notification \n");
    /* Wait for response from the trustlet */
#ifdef RDMA_SINGLE_SESSION
    ret = mc_wait_notification(&rdmaSessionHandle, MC_INFINITE_TIMEOUT);
#else
    ret = mc_wait_notification(&tlSessionHandle, MC_INFINITE_TIMEOUT);
#endif
    if (MC_DRV_OK != ret)
    {
        DISP_ERR("[SVP] mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
        return -1;
    }
    DISP_DBG("[SVP] Trustlet CMD_SVP_CONFIG_RDMA done \n");

    return 0;
}

// map the normal decouple buffer mva into secure page table.
int map_decouple_mva(void)
{
    enum mc_result ret = MC_DRV_OK;
    static int executed = 0;
    int result = 0;
    unsigned int i;
    if (0 == executed)
    {
        /* prepare data */
        memset(pTci, 0, sizeof(tciMessage_t));
        pTci->cmdSvp.header.commandId = CMD_SVP_MAP_DECOUPLE_MVA;
        pTci->Value2 = cal_ovl_ram_size() / MAX_BUFFER_COUNT;
        DISP_MSG("[SVP] map_decouple_mva size=%x \n", cal_ovl_ram_size() / MAX_BUFFER_COUNT);

        for (i = 0; i < MAX_BUFFER_COUNT; i++)
        {
            DISP_MSG("[SVP] map_decouple_mva #%d mva=0x%x \n",
                i, existed_decouple_mva[i]);
            pTci->decouple_mva[i] = existed_decouple_mva[i];
            //result = notifyTrustletCommandValue(CMD_SVP_MAP_DECOUPLE_MVA,
              //  existed_decouple_mva[i], cal_ovl_ram_size() / MAX_BUFFER_COUNT);
        }

        DISP_DBG("notify Trustlet CMD_SVP_MAP_DECOUPLE_MVA \n");
        /* Notify the trustlet */
        ret = mc_notify(&tlSessionHandle);
        if (MC_DRV_OK != ret)
        {
            DISP_ERR("mc_notify failed: %d @%s line %d\n", ret, __func__, __LINE__);
            return -1;
        }

        DISP_DBG("Trustlet CMD_SVP_MAP_DECOUPLE_MVA wait notification \n");
        /* Wait for response from the trustlet */
        ret = mc_wait_notification(&tlSessionHandle, MC_INFINITE_TIMEOUT);
        if (MC_DRV_OK != ret)
        {
            DISP_ERR("mc_wait_notification failed: %d @%s line %d\n", ret, __func__, __LINE__);
            return -1;
        }

        DISP_DBG("Trustlet CMD_SVP_MAP_DECOUPLE_MVA done \n");
        executed = 1;
    }
    return result;
}

static int set_wdma_secure_port(unsigned int secure)
{
#ifndef SEC_BUFFER
    return 0;
#else
    return notifyTrustletCommandValue(
        CMD_SVP_SWITCH_SECURE_PORT, SECURE_PORT_WDMA, (uint32_t)secure);
#endif
}

static int set_rdma_secure_port(unsigned int secure)
{
#ifndef SEC_BUFFER
    return 0;
#else

#ifdef RDMA_SINGLE_SESSION
    late_init_session_rdma_tl();
    return notifyTrustletCommandValueToSession(&rdmaSessionHandle, pRdmaTci,
#else
    return notifyTrustletCommandValue(
#endif
        CMD_SVP_SWITCH_SECURE_PORT, SECURE_PORT_RDMA, (uint32_t)secure);

#endif
}

static int set_ovl_secure_port(unsigned int secure)
{
    // value1: 0 for DISP_OVL_0
    return notifyTrustletCommandValue(CMD_SVP_SWITCH_SECURE_PORT,
        SECURE_PORT_OVL_0, (uint32_t)secure);
}

int switch_wdma_secure_port(unsigned int secure)
{
    int result = 0;
    if (0 != secure) {
        if (0 == is_decouple_secure_port_wdma) {
            DISP_MSG("[SVP] switch wdma to secure port \n");
            result = set_wdma_secure_port(secure);
            is_decouple_secure_port_wdma = 1;
        }
    }
    else
    {
        if (1 == is_decouple_secure_port_wdma) {
            DISP_MSG("[SVP] will switch wdma to non-secure port \n");
            result = set_wdma_secure_port(secure);
            is_decouple_secure_port_wdma = 0;
        }
    }
    return result;
}

int switch_rdma_secure_port(unsigned int secure)
{
    int result = 0;
    if (0 != secure) {
        if (0 == is_decouple_secure_port_rdma) {
            DISP_MSG("[SVP] switch rdma to secure port \n");
            result = set_rdma_secure_port(secure);
            is_decouple_secure_port_rdma = 1;
        }
    }
    else
    {
        if (1 == is_decouple_secure_port_rdma) {
            DISP_MSG("[SVP] will switch rdma to non-secure port \n");
            is_decouple_secure_port_rdma = 0;
            need_release_rdma_secure_port = 1;
        }
    }
    return result;
}

int switch_ovl_secure_port(unsigned int secure)
{
    int result = 0;
    if (0 != secure)
    {
        if (secure_port_flag() == 0)
        {
            DISP_MSG("[SVP] switch ovl to access secure port \n");
            result = set_ovl_secure_port(secure);
            set_secure_port_flag(1);
        }
    }
    else
    {
        if (secure_port_flag() == 1)
        {
            DISP_MSG("[SVP] switch ovl to non-secure port \n");
            result = set_ovl_secure_port(secure);
            set_secure_port_flag(0);
        }
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////
int alloc_decouple_buffer(void)
{
    DISP_MSG("[SVP] alloc_decouple_buffer \n");
    return notifyTrustletCommandValue(CMD_SVP_ALLOC_DECOUPLE_BUFFER, cal_ovl_ram_size(), 0);
}

int debug_start_isrh(void)
{
    DISP_MSG("[SVP] debug_start_isrh \n");
    notifyTrustletCommandValue(CMD_SVP_START_ISRH, 0, 0);
    return 0;
}

int debug_stop_isrh(void)
{
    DISP_MSG("[SVP] debug_stop_isrh \n");
    notifyTrustletCommandValue(CMD_SVP_STOP_ISRH, 0, 0);
    return 0;
}

extern void disp_invoke_irq_callbacks(DISP_MODULE_ENUM module, unsigned int param);

int clear_rdma_intr_secure(unsigned int reg_val)
{
    int result = 0;
    unsigned int value1 = reg_val;
    unsigned int value2;
 	//MMProfileLog(MTKFB_MMP_Events.ClearRDMAIntr, MMProfileFlagStart);
 	DISP_DBG("[SVP] clear_rdma_intr_secure %x \n", reg_val);
    result = _notifyTDriverCommandValueWithTimeout(
        CMD_SVP_DRV_CLEAR_RDMA_INTR, &value1, &value2, 40);
 	//MMProfileLog(MTKFB_MMP_Events.ClearRDMAIntr, MMProfileFlagEnd);

    value2 = value2 ^ reg_val;
    if (0 != value2)
    {
        disp_invoke_irq_callbacks(DISP_MODULE_RDMA, value2);
        //MMProfileLog(MTKFB_MMP_Events.ClearRDMAIntr, MMProfileFlagPulse);
    }
#if 0
    value1 = DISP_REG_GET(DISP_REG_RDMA_INT_STATUS);
    if (0 != value1)
    {
        DISP_MSG("[SVP] skip secure intr rdma %x \n", value1);
        result = _notifyTDriverCommandValueWithTimeout(
            CMD_SVP_DRV_CLEAR_RDMA_INTR, &value1, &value2, 40);
        disp_invoke_irq_callbacks(DISP_MODULE_RDMA, value1);
    }
#endif
 	return result;
}

int clear_wdma_intr_secure(unsigned int reg_val)
{
    int result = 0;
    unsigned int value1 = reg_val;
    unsigned int value2;
 	//MMProfileLog(MTKFB_MMP_Events.ClearWDMAIntr, MMProfileFlagStart);
 	DISP_DBG("[SVP] clear_wdma_intr_secure %x \n", reg_val);
    result = _notifyTDriverCommandValueWithTimeout(
        CMD_SVP_DRV_CLEAR_WDMA_INTR, &value1, &value2, 40);
 	//MMProfileLog(MTKFB_MMP_Events.ClearWDMAIntr, MMProfileFlagEnd);

    value2 = value2 ^ reg_val;
    if (0 != value2)
    {
        disp_invoke_irq_callbacks(DISP_MODULE_WDMA0, value2);
        //MMProfileLog(MTKFB_MMP_Events.ClearWDMAIntr, MMProfileFlagPulse);
    }
#if 0
    value1 = DISP_REG_GET(DISP_REG_WDMA_INTSTA);
    if (0 != value1)
    {
        DISP_MSG("[SVP] skip secure intr wdma %x \n", value1);
        result = _notifyTDriverCommandValueWithTimeout(
            CMD_SVP_DRV_CLEAR_WDMA_INTR, &value1, &value2, 40);
        disp_invoke_irq_callbacks(DISP_MODULE_WDMA0, value1);
    }
#endif
    return result;
}

int clear_ovl_intr_secure(unsigned int reg_val)
{
    return notifyTDriverCommandValueWithTimeout(
        CMD_SVP_DRV_CLEAR_OVL_INTR, reg_val, 0, 40);
}

//------------------------------------------------------------------------------
// Device APC protection operation
//------------------------------------------------------------------------------

static int set_rdma_apc(unsigned int secure)
{
#ifdef RDMA_SINGLE_SESSION
//#if 0
    late_init_session_rdma_tl();
    return notifyTrustletCommandValueToSession(&rdmaSessionHandle, pRdmaTci,
#else
    return notifyTrustletCommandValue(
#endif
        CMD_SVP_SET_RDMA_APC, secure, 0);
}

static int set_wdma_apc(unsigned int secure)
{
    return notifyTrustletCommandValue(
        CMD_SVP_SET_WDMA_APC, secure, 0);
}

static int set_ovl_apc(unsigned int secure)
{
    return notifyTrustletCommandValue(
        CMD_SVP_SET_OVL_APC, secure, 0);
}

int switch_rdma_apc(unsigned int secure)
{
    int result = 0;
#ifndef DISABLE_DAPC_PROTECTION
//#if 0
    if (0 != secure)
    {
        if (0 == is_device_apc_rdma)
        {
            DISP_MSG("[SVP] switch RDMA to DAPC protected \n");
            is_device_apc_rdma = 1;
            result = set_rdma_apc(secure);
        }
    }
    else
    {
        if (0 != is_device_apc_rdma)
        {
            result = set_rdma_apc(secure);
            is_device_apc_rdma = 0;
            DISP_MSG("[SVP] release RDMA DAPC protection \n");
        }
    }
#endif
    return result;
}

int switch_wdma_apc(unsigned int secure)
{
    int result = 0;
#ifndef DISABLE_DAPC_PROTECTION
//#if 0
    if (0 != secure)
    {
        if (0 == is_device_apc_wdma)
        {
            DISP_MSG("[SVP] switch WDMA to DAPC protected \n");
            is_device_apc_wdma = 1;
            result = set_wdma_apc(secure);
        }
    }
    else
    {
        if (0 != is_device_apc_wdma)
        {
            result = set_wdma_apc(secure);
            is_device_apc_wdma = 0;
            DISP_MSG("[SVP] release WDMA DAPC protection \n");
        }
    }
#endif
    return result;
}

int switch_ovl_apc(unsigned int secure)
{
    int result = 0;
#ifndef DISABLE_DAPC_PROTECTION
//#if 0
    if (0 != secure)
    {
        if (0 == is_device_apc_ovl)
        {
            DISP_MSG("[SVP] switch OVL to DAPC protected \n");
            is_device_apc_ovl = 1;
            result = set_ovl_apc(secure);
        }
    }
    else
    {
        if (0 != is_device_apc_ovl)
        {
            result = set_ovl_apc(secure);
            is_device_apc_ovl = 0;
            DISP_MSG("[SVP] release OVL DAPC protection \n");
        }
    }
#endif
    return result;
}

static struct mc_bulk_map systrace_mapped_info;
volatile static unsigned long p_systrace_buf = 0;
static int enable_secure_systrace(void *buf, size_t size)
{
    enum mc_result ret = MC_DRV_OK;
    late_init_session_tl();
    if (0 == p_systrace_buf)
    {
        p_systrace_buf = (unsigned long)buf;

        // buffer mapped to secure world
        ret = mc_map(&tlSessionHandle, buf, size, &systrace_mapped_info);
        if (MC_DRV_OK != ret)
        {
            DISP_ERR("mc_map failed: %d @%s line %d\n", ret, __func__, __LINE__);
            return -1;
        }

        notifyTrustletCommandValue(CMD_SVP_MAP_SYSTRACE_BUF,
            (uint32_t)(systrace_mapped_info.secure_virt_addr),
            systrace_mapped_info.secure_virt_len);
    }

    notifyTrustletCommandValue(CMD_SVP_ENABLE_SECURE_SYSTRACE,
        0, 0);

    return 0;
}

static int pause_secure_systrace(unsigned int *head)
{
    uint32_t val = 0;
    return _notifyTrustletCommandValue(CMD_SVP_PAUSE_SECURE_SYSTRACE,
        head, &val);
}

static int resume_secure_systrace(void)
{
    return notifyTrustletCommandValue(CMD_SVP_RESUME_SECURE_SYSTRACE,
        0, 0);
}

static int disable_secure_systrace(void)
{
    enum mc_result ret = MC_DRV_OK;
    notifyTrustletCommandValue(CMD_SVP_DISABLE_SECURE_SYSTRACE,
        0, 0);

    if (0 != p_systrace_buf)
    {
        ret = mc_unmap(&tlSessionHandle, (void*)p_systrace_buf, &systrace_mapped_info);
        if (MC_DRV_OK != ret)
        {
            DISP_ERR("mc_unmap failed: %d @%s line %d\n", ret, __func__, __LINE__);
            /* continue anyway */
        }
        p_systrace_buf = 0;
    }

    return 0;
}

static struct secure_callback_funcs secure_systrace_funcs = {
    .enable_systrace = enable_secure_systrace,
    .disable_systrace = disable_secure_systrace,
    .pause_systrace = pause_secure_systrace,
    .resume_systrace = resume_secure_systrace,
};

