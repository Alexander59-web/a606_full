#include <linux/kernel.h>

#include "ddp_debug.h"

#include <tlSvp_Api.h> // trustlet inter face for TLC

#include "mobicore_driver_api.h"

#define SYSTRACE_PROFILING 1
#include "secure_systrace.h"


static const uint32_t mc_deviceId = MC_DEVICE_ID_DEFAULT;
static const struct mc_uuid_t uuid = SVP_TL_DBG_UUID;

static struct mc_session_handle tlSessionHandle;
static tciMessage_t *pTci = NULL;


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

static void close_mobicore_device(void)
{
    if (opened_device != 0)
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
        opened_device = 0;
    }
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
            mc_free_wsm(mc_deviceId, (uint8_t *)pTci);
            break;
        }
    } while (0);

    return mcRet;
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
    memset(&tlSessionHandle, 0, sizeof(tlSessionHandle));

    mcRet = mc_free_wsm(mc_deviceId, (uint8_t *)pTci);
    if (MC_DRV_OK != mcRet)
    {
        DISP_ERR("mc_free_wsm failed: %d @%s line %d\n", mcRet, __func__, __LINE__);
        return;
    }
    pTci = NULL;
}

// return 0 for success and -1 for failure
static int notifyTrustletCommandValue(uint32_t command, uint32_t *value1, uint32_t *value2)
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
    pTci->Value1 = (value1 != NULL) ? *value1 : 0;
    pTci->Value2 = (value2 != NULL) ? *value2 : 0;

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

    if (value1 != NULL)
        *value1 = pTci->Value1;
    if (value2 != NULL)
        *value2 = pTci->Value2;

    DISP_DBG("Trustlet CMD: %d done \n", command);

    return 0;
}

static int enable_secure_systrace(void *buf, size_t size)
{
    uint32_t buf_pa = (uint32_t)__pa(buf);
    uint32_t buf_size = (uint32_t)size;
    return notifyTrustletCommandValue(CMD_DAPC_ENABLE_SECURE_SYSTRACE, &buf_pa, &buf_size);
}

static int pause_secure_systrace(unsigned int *head)
{
    uint32_t buf_head = 0;
    int ret = notifyTrustletCommandValue(CMD_DAPC_PAUSE_SECURE_SYSTRACE, &buf_head, NULL);
    *head = buf_head;
printk("DAPCTEST: ret=%d, head=%u\n", ret, *head);
    return ret;
}

static int resume_secure_systrace(void)
{
    return notifyTrustletCommandValue(CMD_DAPC_RESUME_SECURE_SYSTRACE, NULL, NULL);
}

static int disable_secure_systrace(void)
{
    return notifyTrustletCommandValue(CMD_DAPC_DISABLE_SECURE_SYSTRACE, NULL, NULL);
}

static struct secure_callback_funcs secure_systrace_funcs = {
    .enable_systrace = enable_secure_systrace,
    .disable_systrace = disable_secure_systrace,
    .pause_systrace = pause_secure_systrace,
    .resume_systrace = resume_secure_systrace,
};

void dapc_systrace_init(void)
{
    systrace_init1(SEC_SYSTRACE_MODULE_DAPC, "dapc", "SWd_DAPC", 0x10000, &secure_systrace_funcs);
}

void dapc_systrace_deinit(void)
{
    systrace_deinit1("dapc");

    close_session_tl();
    close_mobicore_device();
}

