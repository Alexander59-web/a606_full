#include "ddp_cmdq_sec.h"

#if defined(__CMDQ_SECURE_PATH_SUPPORT__)
// secure wrold profile
#define SYSTRACE_PROFILING (1)
#include "secure_systrace.h" 
#else
#define SYSTRACE_PROFILING (0)
#endif

static atomic_t gDebugSecSwCopy = ATOMIC_INIT(0);
static atomic_t gDebugSecCmdId = ATOMIC_INIT(0);

static DEFINE_MUTEX(gCmdqSecExecLock);       // lock to protect atomic secure task execution
static DEFINE_MUTEX(gCmdqSecContextLock);    // lock to protext atomic access gCmdqSecContextList
static struct list_head gCmdqSecContextList; // secure context list. note each porcess has its own sec context

// function declaretion
cmdqSecContextHandle cmdq_sec_context_handle_create(uint32_t tgid);

#if defined(__CMDQ_SECURE_PATH_SUPPORT__)

// mobicore driver interface
#include "mobicore_driver_api.h"

// secure path header
#include "cmdq_sec_iwc_common.h"
#include "cmdqSecTl_Api.h"
// secure dci interface (debug usage)
#include "cmdq_sec_dciapi.h"

#define CMDQ_DR_UUID { { 2, 0xb, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }
#define CMDQ_TL_UUID { { 9, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }

// internal control
#define CMDQ_OPEN_SESSION_ONCE (1)  // set 1 to open once for each process context, because mc_open_session is too slow

//--------------------------------------------------------------------------------
#define CMDQ_SEC_SYSTRACE_MAX_TRACE_COUNT (20 * 100)
#define CMDQ_SEC_SYSTRACE_BUFFER_SIZE (CMDQ_SEC_SYSTRACE_MAX_TRACE_COUNT * sizeof(struct systrace_log))
#define CMDQ_SEC_SYSTRACE_MODULE_ID (0)

static DEFINE_MUTEX(gCmdqSecSystraceLock); // ensure atomic access secure systrace buffer index
static uint32_t gCmdqSecSystraceIndex = 0; 

int cmdq_sec_pause_secure_systrace(unsigned int *head)
{
    int flag = 0; 
    
    CMDQ_VERBOSE("SYSTRACE: pause\n");

    // set profile flag
    flag = (cmdq_core_profile_enabled() & ~(0x1 << CMDQ_IWC_PROF_SYSTRACE));
    cmdq_core_set_profile_enabled(flag);
    
    mutex_lock(&gCmdqSecSystraceLock);
    do
    {    
        *head = gCmdqSecSystraceIndex;
        CMDQ_VERBOSE("SYSTRACE: *head:%d, start_item:%d\n", *head, start_item);
    }while(0);
    mutex_unlock(&gCmdqSecSystraceLock);
    
    return 0;
}

int cmdq_sec_resume_secure_systrace(void)
{
    int flag = 0; 

    CMDQ_VERBOSE("SYSTRACE: resume\n");

    mutex_lock(&gCmdqSecSystraceLock);
    gCmdqSecSystraceIndex = 0; 
    mutex_unlock(&gCmdqSecSystraceLock);

    // set profile flag
    flag = (cmdq_core_profile_enabled() | (0x1 << CMDQ_IWC_PROF_SYSTRACE));
    cmdq_core_set_profile_enabled(flag);
    return 0; 
}

int cmdq_sec_enable_secure_systrace(void *buf, size_t size)
{
    CMDQ_VERBOSE("SYSTRACE: enable\n");

    return 0;   
}

int cmdq_sec_disable_secure_systrace(void)
{
    CMDQ_VERBOSE("SYSTRACE: disable\n");

    return 0;   
}

static struct secure_callback_funcs cmdq_secure_systrace_funcs = {
    .enable_systrace = cmdq_sec_enable_secure_systrace,
    .disable_systrace = cmdq_sec_disable_secure_systrace,
    .pause_systrace = cmdq_sec_pause_secure_systrace,
    .resume_systrace = cmdq_sec_resume_secure_systrace,
};

const char* cmdq_sec_get_secure_systrace_tag(CMDQ_IWC_SYSTRACE_TAG_ENUM event)
{
    switch(event)
    {
    // cmdqSecDr
    case CMDQ_IST_CMDQ_IPC       : return "IPC";
    case CMDQ_IST_CMDQ_TASK_ACQ  : return "TASK+";
    case CMDQ_IST_CMDQ_TASK_REL  : return "TASK-";
    case CMDQ_IST_CMDQ_HW        : return "HW"; 
    case CMDQ_IST_CMDQ_SIG_WAIT  : return "SigIpcTHR";
    // cmdqSecTL
    case CMDQ_IST_CMD_ALLOC      : return "CMD+";
    case CMDQ_IST_H2PAS          : return "H2PAS";
    case CMDQ_IST_PORT_ON        : return "PORT+";
    case CMDQ_IST_PORT_OFF       : return "PORT-";
    case CMDQ_IST_DAPC_ON        : return "DAPC+";
    case CMDQ_IST_DAPC_OFF       : return "DAPC-";
    case CMDQ_IST_SUBMIT         : return "SUBMIT";
    default: return "UNKOWN";
    }

    return "UNKOWN";
}

void cmdq_sec_insert_item_to_systrace_buffer(uint64_t startTime, uint64_t endTime, CMDQ_IWC_SYSTRACE_TAG_ENUM tagId)
{
#if SYSTRACE_PROFILING
    uint32_t tagLength;
    uint32_t copyLength;

    char* tag = NULL;
    struct systrace_log *pSystraceLog = NULL;
    const uint32_t TAG_MAX_LENGTH = sizeof(pSystraceLog->name) / sizeof(pSystraceLog->name[0]) - 1; // reserve last char for '\0'
        
    mutex_lock(&gCmdqSecSystraceLock);
    do
    {
        CMDQ_VERBOSE("SYSTRACE_INSERT: +time(%llx, %llx), tagId:%d, TAG_MAX_LENGTH:%d\n", startTime, endTime, tagId, TAG_MAX_LENGTH);
    
        if(NULL == systrace_buffer)
        {
            CMDQ_ERR("SYSTRACE_INSERT: null buffer\n");
            break; 
        }

        // get systrace buffer
        pSystraceLog = (struct systrace_log*)(systrace_buffer);
        
        // write trace begin log
        gCmdqSecSystraceIndex = (CMDQ_SEC_SYSTRACE_MAX_TRACE_COUNT <= gCmdqSecSystraceIndex) ? (0) : (gCmdqSecSystraceIndex);
        tag = cmdq_sec_get_secure_systrace_tag(tagId); 
        if(tag)
        {
            tagLength = strlen(tag); // strlen not include terminating null char
            copyLength = (TAG_MAX_LENGTH < tagLength) ? (TAG_MAX_LENGTH) : (tagLength);

            memcpy(pSystraceLog[gCmdqSecSystraceIndex].name, tag, copyLength);
            pSystraceLog[gCmdqSecSystraceIndex].name[copyLength] = '\0';
        }
        pSystraceLog[gCmdqSecSystraceIndex].timestamp = startTime;
        pSystraceLog[gCmdqSecSystraceIndex].type = 0;
        gCmdqSecSystraceIndex += 1;
        
        // write trace end log
        gCmdqSecSystraceIndex = (CMDQ_SEC_SYSTRACE_MAX_TRACE_COUNT <= gCmdqSecSystraceIndex) ? (0) : (gCmdqSecSystraceIndex);
        pSystraceLog[gCmdqSecSystraceIndex].name[0] = '\0';
        pSystraceLog[gCmdqSecSystraceIndex].timestamp = endTime;
        pSystraceLog[gCmdqSecSystraceIndex].type = 1;
        gCmdqSecSystraceIndex += 1;

        #if 0
        CMDQ_LOG("SYSTRACE_INSERT: -item[%d, %llx, %s], item[%d, %llx, %s]\n", 
            gCmdqSecSystraceIndex-2, pSystraceLog[gCmdqSecSystraceIndex-2].timestamp, pSystraceLog[gCmdqSecSystraceIndex-2].name,
            gCmdqSecSystraceIndex-1, pSystraceLog[gCmdqSecSystraceIndex-1].timestamp, pSystraceLog[gCmdqSecSystraceIndex-1].name);
        #endif
    }while(0); 
    
    mutex_unlock(&gCmdqSecSystraceLock);
#endif
}

//--------------------------------------------------------------------------------

int32_t cmdq_sec_open_mobicore_impl(uint32_t deviceId)
{
    int32_t status = 0;
    enum mc_result mcRet = mc_open_device(deviceId);

    // Currently, a process context limits to open mobicore device once,
    // and mc_open_device dose not support reference cout
    // so skip the false alarm error....
    if(MC_DRV_ERR_INVALID_OPERATION == mcRet)
    {
        CMDQ_MSG("[SEC]_MOBICORE_OPEN: already opened, continue to execution\n");
        status = -EEXIST;
    }
    else if(MC_DRV_OK != mcRet)
    {
        CMDQ_ERR("[SEC]_MOBICORE_OPEN: err[0x%x]\n", mcRet);
        status = -1;
    }

    CMDQ_MSG("[SEC]_MOBICORE_OPEN: status[%d], ret[0x%x]\n", status, mcRet);
    return status;
}


int32_t cmdq_sec_close_mobicore_impl(const uint32_t deviceId, const uint32_t openMobicoreByOther)
{
    int32_t status = 0;
    enum mc_result mcRet = 0;

    if(1 == openMobicoreByOther)
    {
        // do nothing
        // let last user to close mobicore....
        CMDQ_MSG("[SEC]_MOBICORE_CLOSE: opened by other, bypass device close\n");
    }
    else
    {
        mcRet = mc_close_device(deviceId);
        CMDQ_MSG("[SEC]_MOBICORE_CLOSE: status[%d], ret[%0x], openMobicoreByOther[%d]\n", status, mcRet, openMobicoreByOther);
        if(MC_DRV_OK != mcRet)
        {
            CMDQ_ERR("[SEC]_MOBICORE_CLOSE: err[0x%x]\n", mcRet);
            status = -1;
        }
    }

    return status;
}


int32_t cmdq_sec_allocate_wsm_impl(uint32_t deviceId, uint8_t **ppWsm, uint32_t wsmSize)
{
    int32_t status = 0;
    enum mc_result mcRet = MC_DRV_OK;

    do
    {
        if((*ppWsm) != NULL)
        {
            status = -1;
            CMDQ_ERR("[SEC]_WSM_ALLOC: err[pWsm is not NULL]");
            break;
        }

        // because world shared mem(WSM) will ba managed by mobicore device, not linux kernel
        // instead of vmalloc/kmalloc, call mc_malloc_wasm to alloc WSM to prvent error such as
        // "can not resolve tci physicall address" etc
        mcRet = mc_malloc_wsm(deviceId, 0, wsmSize, ppWsm, 0);
        if (MC_DRV_OK != mcRet)
        {
            CMDQ_ERR("[SEC]_WSM_ALLOC: err[0x%x]\n", mcRet);
            status = -1;
            break;
        }
        CMDQ_MSG("[SEC]_WSM_ALLOC: status[%d], *ppWsm: 0x%p\n", status, (*ppWsm));
    }while(0);

    return status;
}


int32_t cmdq_sec_free_wsm_impl(uint32_t deviceId, uint8_t **ppWsm)
{
    int32_t status = 0;
    enum mc_result mcRet = mc_free_wsm(deviceId, (*ppWsm));

    (*ppWsm) = (MC_DRV_OK == mcRet) ? (NULL) : (*ppWsm);
    CMDQ_VERBOSE("_WSM_FREE: ret[0x%x], *ppWsm[0x%p]\n", mcRet, (*ppWsm));

    if(MC_DRV_OK != mcRet)
    {
        CMDQ_ERR("_WSM_FREE: err[0x%x]", mcRet);
        status = -1;
    }

    return status;
}


int32_t cmdq_sec_open_session_impl(
            uint32_t deviceId,
            const struct mc_uuid_t *uuid,
            uint8_t *pWsm,
            uint32_t wsmSize,
            struct mc_session_handle* pSessionHandle)
{
    int32_t status = 0;
    enum mc_result mcRet = MC_DRV_OK;
    do
    {
        if(NULL == pWsm || NULL == pSessionHandle)
        {
            status = -1;
            CMDQ_ERR("[SEC]_SESSION_OPEN: invalid param, pWsm[0x%p], pSessionHandle[0x%p]\n", pWsm, pSessionHandle);
            break;
        }

        memset(pSessionHandle, 0, sizeof(*pSessionHandle));
        pSessionHandle->device_id = deviceId;
        mcRet = mc_open_session(pSessionHandle,
                              uuid,
                              pWsm,
                              wsmSize);
        if (MC_DRV_OK != mcRet)
        {
            CMDQ_ERR("[SEC]_SESSION_OPEN: err[0x%x]\n", mcRet);
            status = -1;
            break;
        }
        CMDQ_MSG("[SEC]_SESSION_OPEN: status[%d], mcRet[0x%x]\n", status, mcRet);
    }while(0);

    return status;
}


int32_t cmdq_sec_close_session_impl(struct mc_session_handle* pSessionHandle)
{
    int32_t status = 0;
    enum mc_result mcRet = mc_close_session(pSessionHandle);

    if(MC_DRV_OK != mcRet)
    {
        CMDQ_ERR("_SESSION_CLOSE: err[0x%x]", mcRet);
        status = -1;
    }
    return status;
}


int32_t cmdq_sec_init_session_unlocked(
            const struct mc_uuid_t *uuid,
            uint8_t** ppWsm,
            uint32_t wsmSize,
            struct mc_session_handle* pSessionHandle,
            CMDQ_IWC_STATE_ENUM *pIwcState,
            uint32_t* openMobicoreByOther)
{
    int32_t openRet = 0;
    int32_t status = 0;
    uint32_t deviceId = MC_DEVICE_ID_DEFAULT;

    CMDQ_MSG("[SEC]-->SESSION_INIT: iwcState[%d]\n", (*pIwcState));
    do
    {
        #if CMDQ_OPEN_SESSION_ONCE
        if(IWC_SES_OPENED <= (*pIwcState))
        {
            CMDQ_MSG("SESSION_INIT: already opened\n");
            break;
        }
        else
        {
            CMDQ_MSG("[SEC]SESSION_INIT: open new session[%d]\n", (*pIwcState));
        }
        #endif
    	CMDQ_VERBOSE("[SEC]SESSION_INIT: wsmSize[%d], pSessionHandle: 0x%p, sizeof(*pSessionHandle): %d\n",
    	    wsmSize, pSessionHandle, sizeof(*pSessionHandle));

        CMDQ_PROF_START("CMDQ_SEC_INIT");

        // open mobicore device
        openRet = cmdq_sec_open_mobicore_impl(deviceId);
        if(-EEXIST == openRet)
        {
            // mobicore has been opened in this process context
            // it is a ok case, so continue to execute
            status = 0;
            (*openMobicoreByOther) = 1;
        }
        else if(0 > openRet)
        {
            status = -1;
            break;
        }
        (*pIwcState) = IWC_MOBICORE_OPENED;


        // allocate world shared memory
        if(0 > cmdq_sec_allocate_wsm_impl(deviceId, ppWsm, wsmSize))
        {
             status = -1;
            break;
        }
        (*pIwcState) = IWC_WSM_ALLOCATED;

        // open a secure session
        if(0 > cmdq_sec_open_session_impl(deviceId, uuid, (*ppWsm), wsmSize, pSessionHandle))
        {
             status = -1;
            break;
        }
        (*pIwcState) = IWC_SES_OPENED;

        CMDQ_PROF_END("CMDQ_SEC_INIT");
    } while (0);

    CMDQ_MSG("[SEC]<--SESSION_INIT[%d]\n",  status);
    return status;
}


int32_t cmdq_sec_fill_iwc_buffer_unlocked(
            iwcCmdqMessage_t* pIwc,
            uint32_t iwcCommand,
            TaskStruct *pTask,
            int32_t thread,
            CmdqSecFillIwcCB iwcFillCB )
{
    int32_t status = 0;

    CMDQ_MSG("[SEC]-->SESSION_MSG: cmdId[%d]\n", iwcCommand);

    // fill message buffer for inter world communication
    memset(pIwc, 0x0, sizeof(iwcCmdqMessage_t));
    if(NULL != pTask && CMDQ_INVALID_THREAD != thread)
    {
        // basic data
        pIwc->cmd             = iwcCommand;
        pIwc->scenario        = pTask->scenario;
        pIwc->thread          = thread;
        pIwc->priority        = pTask->priority;
        pIwc->engineFlag      = pTask->engineFlag;
        pIwc->blockSize    = pTask->blockSize;
        memcpy((pIwc->pVABase), (pTask->pVABase), (pTask->blockSize));

        // metadata
        CMDQ_VERBOSE("[SEC]SESSION_MSG: addrList[%d][0x%p], portList[%d][0x%p]\n",
            pTask->secData.addrListLength, pTask->secData.addrList, pTask->secData.portListLength, pTask->secData.portList);
        if(0 < pTask->secData.addrListLength)
        {
            pIwc->metadata.addrListLength = pTask->secData.addrListLength;
            memcpy((pIwc->metadata.addrList), (pTask->secData.addrList), (pTask->secData.addrListLength)*sizeof(iwcCmdqAddrMetadata_t));
        }

        if(0 < pTask->secData.portListLength)
        {
            pIwc->metadata.portListLength = pTask->secData.portListLength;
            memcpy((pIwc->metadata.portList), (pTask->secData.portList), (pTask->secData.portListLength)*sizeof(iwcCmdqPortMetadata_t));
        }

        // medatada: debug config
        pIwc->metadata.debug.logLevel = (cmdq_core_should_print_msg()) ? (1) : (0);
        pIwc->metadata.debug.enableProfile = cmdq_core_profile_enabled();
    }
    else if(NULL != iwcFillCB)
    {
        status = (*iwcFillCB)(iwcCommand, (void*)pIwc);
    }
    else
    {
        // relase resource, or debug function will go here
        CMDQ_VERBOSE("[SEC]-->SESSION_MSG: no task, cmdId[%d]\n", iwcCommand);
        pIwc->cmd = iwcCommand;
        pIwc->blockSize = 0;
        pIwc->metadata.addrListLength = 0;
        pIwc->metadata.portListLength = 0;
    }

    CMDQ_MSG("[SEC]<--SESSION_MSG[%d]\n", status);
    return status;
}

int32_t cmdq_sec_execute_session_unlocked(
            struct mc_session_handle* pSessionHandle,
            CMDQ_IWC_STATE_ENUM *pIwcState, 
            int32_t timeout_ms)
{
    enum mc_result mcRet;
    int32_t status = 0;

    CMDQ_PROF_START("CMDQ_SEC_EXE");

    do
    {
        // notify to secure world
        mcRet = mc_notify(pSessionHandle);
        if (MC_DRV_OK != mcRet)
        {
            CMDQ_ERR("[SEC]EXEC: mc_notify err[0x%x]\n", mcRet);
            status = -1;
            break;
        }
        else
        {
            CMDQ_MSG("[SEC]EXEC: mc_notify ret[0x%x]\n", mcRet);
        }
        (*pIwcState) = IWC_SES_TRANSACTED;


        // wait respond
    #if 0
        mcRet = mc_wait_notification(pSessionHandle, MC_INFINITE_TIMEOUT);
    #else
        const int32_t secureWoldTimeout_ms = (0 < timeout_ms)?(timeout_ms):(MC_INFINITE_TIMEOUT); 
    
        mcRet = mc_wait_notification(pSessionHandle, secureWoldTimeout_ms);
        if(MC_DRV_ERR_TIMEOUT == mcRet)
        {
            CMDQ_ERR("[SEC]EXEC: mc_wait_notification timeout, err[0x%x], secureWoldTimeout_ms[%d]\n", mcRet, secureWoldTimeout_ms);
            status = -ETIMEDOUT;
            break;
        }
    #endif
        if (MC_DRV_OK != mcRet)
        {
            CMDQ_ERR("[SEC]EXEC: mc_wait_notification err[0x%x]\n", mcRet);
            status = -1;
            break;
        }
        else
        {
            CMDQ_MSG("[SEC]EXEC: mc_wait_notification err[%d]\n", mcRet);
        }
        (*pIwcState) = IWC_SES_ON_TRANSACTED;
    }while(0);

    CMDQ_PROF_END("CMDQ_SEC_EXE");
    
    return status;
}


void cmdq_sec_deinit_session_unlocked(
        uint8_t **ppWsm,
        struct mc_session_handle* pSessionHandle,
        const CMDQ_IWC_STATE_ENUM iwcState,
        const uint32_t openMobicoreByOther)
{
    uint32_t deviceId = MC_DEVICE_ID_DEFAULT;

    CMDQ_MSG("[SEC]-->SESSION_DEINIT\n");
    do
    {
        switch(iwcState)
        {
        case IWC_SES_ON_TRANSACTED:
        case IWC_SES_TRANSACTED:
        case IWC_SES_MSG_PACKAGED:
             // continue next clean-up
        case IWC_SES_OPENED:
            cmdq_sec_close_session_impl(pSessionHandle);
            // continue next clean-up
        case IWC_WSM_ALLOCATED:
            cmdq_sec_free_wsm_impl(deviceId, ppWsm);
            // continue next clean-up
        case IWC_MOBICORE_OPENED:
            cmdq_sec_close_mobicore_impl(deviceId, openMobicoreByOther);
            // continue next clean-up
            break;
        case IWC_INIT:
            //CMDQ_ERR("open secure driver failed\n");
            break;
        default:
            break;
        }

    }while(0);

    CMDQ_MSG("[SEC]<--SESSION_DEINIT\n");
}


int32_t cmdq_sec_setup_context_session(cmdqSecContextHandle handle)
{
    int32_t status = 0;
    const struct mc_uuid_t uuid = CMDQ_TL_UUID;

    // init iwc parameter
    if(IWC_INIT == handle->state)
    {
        handle->uuid = uuid;
    }

    // init secure session
    status = cmdq_sec_init_session_unlocked(
                &(handle->uuid),
                (uint8_t**)( &(handle->iwcMessage) ),
                sizeof(iwcCmdqMessage_t),
                &(handle->sessionHandle),
                &(handle->state),
                &(handle->openMobicoreByOther));
    CMDQ_MSG("SEC_SETUP: status[%d], tgid[%d], mobicoreOpenByOther[%d]\n", status, handle->tgid, handle->openMobicoreByOther);
    return status;
}

void cmdq_sec_insert_secure_systrace(cmdqSecContextHandle handle)
{
    // profile
    uint32_t i =0;
    uint64_t startTime = 0;
    uint64_t endTime = 0; 

    // insert systrace
    for(i = 0; i < CMDQ_IST_MAX_COUNT; i++)
    {
        if(0 == ((iwcCmdqMessage_t*)handle->iwcMessage)->metadata.systraceRec[i].startTime ||
           0 == ((iwcCmdqMessage_t*)handle->iwcMessage)->metadata.systraceRec[i].endTime)
        {
            continue; 
        }
        
        startTime = ((iwcCmdqMessage_t*)handle->iwcMessage)->metadata.systraceRec[i].startTime; 
        endTime = ((iwcCmdqMessage_t*)handle->iwcMessage)->metadata.systraceRec[i].endTime; 

        cmdq_sec_insert_item_to_systrace_buffer(startTime, endTime, i);
    }
}

int32_t cmdq_sec_send_context_session_message(
            cmdqSecContextHandle handle,
            uint32_t iwcCommand,
            TaskStruct *pTask,
            int32_t thread,
            CmdqSecFillIwcCB iwcFillCB)
{
    int32_t status = 0;
    int32_t iwcRsp = 0;
    int32_t timeout_ms = (3 * 1000);

    do
    {
        // fill message bufer
        status = cmdq_sec_fill_iwc_buffer_unlocked(handle->iwcMessage, iwcCommand, pTask, thread, iwcFillCB);
        if(0 > status)
        {
            CMDQ_ERR("[SEC]cmdq_sec_fill_iwc_buffer_unlocked failed[%d], pid[%d:%d]\n", status, current->tgid, current->pid); 
            break;
        }

        // send message
        status = cmdq_sec_execute_session_unlocked(&(handle->sessionHandle), &(handle->state), timeout_ms);
    #if 1
        if(-ETIMEDOUT == status)
        {
            CMDQ_ERR("[SEC]cmdq_sec_execute_session_unlocked failed[%d], pid[%d:%d]\n", status, current->tgid, current->pid); 
            break; 
        }
    #endif
        if(0 > status)
        {
            CMDQ_ERR("[SEC]cmdq_sec_execute_session_unlocked failed[%d], pid[%d:%d]\n", status, current->tgid, current->pid); 
            break;
        }

        // get secure task execution result
        iwcRsp = ((iwcCmdqMessage_t*)(handle->iwcMessage))->rsp;
        status = iwcRsp; //(0 == iwcRsp)? (0):(-EFAULT);

        // and then, update task state
        if(pTask)
        {
            pTask->taskState = (0 == iwcRsp)? (TASK_STATE_DONE): (TASK_STATE_ERROR);
        }

        // log print
        if(0 < status)
        {
            CMDQ_ERR("SEC_SEND: status[%d], cmdId[%d], iwcRsp[%d]\n", status, iwcCommand, iwcRsp);
        }
        else
        {
            CMDQ_MSG("SEC_SEND: status[%d], cmdId[%d], iwcRsp[%d]\n", status, iwcCommand, iwcRsp);
        }
    }while(0);

    cmdq_sec_insert_secure_systrace(handle);

    return status;
}

int32_t cmdq_sec_teardown_context_session(cmdqSecContextHandle handle)
{
    int32_t status = 0;
    if(handle)
    {
        CMDQ_MSG("[SEC]SEC_TEARDOWN: state: %d, iwcMessage:0x%p\n", handle->state, handle->iwcMessage);
        cmdq_sec_deinit_session_unlocked(
                    (uint8_t**)(&(handle->iwcMessage)),
                    &(handle->sessionHandle),
                    handle->state,
                    handle->openMobicoreByOther);

         //clrean up handle's attritubes
         handle->state = IWC_INIT;
    }
    else
    {
        CMDQ_ERR("[SEC]SEC_TEARDOWN: null secCtxHandle\n");
        status = -1;
    }
    return status;
}


int32_t cmdq_sec_submit_to_secure_world(
            uint32_t iwcCommand,
            TaskStruct *pTask,
            int32_t thread,
            CmdqSecFillIwcCB iwcFillCB)
{
    const bool skipSecCtxDump = (CMD_CMDQ_TL_RES_RELEASE == iwcCommand) ? (true) : (false); // prevent nested lock gCmdqSecContextLock
    const int32_t tgid = current->tgid;
    const int32_t pid = current->pid;
    cmdqSecContextHandle handle = NULL;
    int32_t status = 0;
    int32_t duration = 0;

    struct timeval tEntrySec;
    struct timeval tExitSec;

    mutex_lock(&gCmdqSecExecLock);
    smp_mb();

    CMDQ_MSG("[SEC]-->SEC_SUBMIT: tgid[%d:%d]\n", tgid, pid);
    do
    {
        // find handle first
        handle = cmdq_sec_find_context_handle_unlocked(tgid);
        if(NULL == handle)
        {
            CMDQ_ERR("SEC_SUBMIT: tgid %d err[NULL secCtxHandle]\n", tgid);
            status = -(CMDQ_ERR_NULL_SEC_CTX_HANDLE);
            break;
        }

        if(0 > cmdq_sec_setup_context_session(handle))
        {
            status = -(CMDQ_ERR_SEC_CTX_SETUP);
            break;
        }

        //
        // record profile data
        // tbase timer/time support is not enough currently,
        // so we treats entry/exit timing to secure world as the trigger/gotIRQ_&_wakeup timing
        //
        CMDQ_GET_CURRENT_TIME(tEntrySec);

        status = cmdq_sec_send_context_session_message(handle, iwcCommand, pTask, thread, iwcFillCB);
#if 1
        if(-ETIMEDOUT == status)
        {
            CMDQ_ERR("[SEC]cmdq_sec_execute_session_unlocked failed[%d], pid[%d:%d]\n", status, current->tgid, current->pid); 
        }
#endif

        CMDQ_GET_CURRENT_TIME(tExitSec);
        CMDQ_GET_TIME_DURATION(tEntrySec, tExitSec, duration);
        if(pTask)
        {
            pTask->trigger = tEntrySec;
            pTask->gotIRQ  = tExitSec;
            pTask->wakedUp = tExitSec;
        }

        // release resource
        #if !(CMDQ_OPEN_SESSION_ONCE)
        cmdq_sec_teardown_context_session(handle)
        #endif

        // 
        // because secure world has no clkmgr support, delay reset HW when back to normal world
        // note do reset flow only if secure driver exec failed (i.e. result = -CMDQ_ERR_DR_EXEC_FAILED)
        //
        if((-CMDQ_ERR_DR_EXEC_FAILED) == status)
        {
            cmdqResetHWEngine(pTask, thread);
        }
    }while(0);

    mutex_unlock(&gCmdqSecExecLock);

    if(-ETIMEDOUT == status)
    {
        // t-base strange issue, mc_wait_notification false timeout when secure world has done 
        // becuase retry may failed, give up retry method
        CMDQ_ERR("CMDQ", "[SEC]<--SEC_SUBMIT: err[%d][mc_wait_notification timeout], pTask[0x%p], THR[%d], tgid[%d:%d], duration_ms[%d], cmdId[%d]\n", status, pTask, thread, tgid, pid, duration, iwcCommand);
    }
    else if(0 > status)
    {
        if(!skipSecCtxDump)
        {
            mutex_lock(&gCmdqSecContextLock);
            cmdq_sec_dump_context_list();
            mutex_unlock(&gCmdqSecContextLock);
        }
        
        // throw AEE
        CMDQ_AEE("CMDQ", "[SEC]<--SEC_SUBMIT: err[%d], pTask[0x%p], THR[%d], tgid[%d:%d], duration_ms[%d], cmdId[%d]\n", status, pTask, thread, tgid, pid, duration, iwcCommand);
    }
    else
    {
        CMDQ_LOG("[SEC]<--SEC_SUBMIT: err[%d], pTask[0x%p], THR[%d], tgid[%d:%d], duration_ms[%d], cmdId[%d]\n", status, pTask, thread, tgid, pid, duration, iwcCommand);        
    }
    return status;
}


//
// sec ping
//
int32_t cmdq_sec_ping_secure_world(cmdqSecContextHandle handle)
{
    int32_t status = 0; 

    do
    {
        CMDQ_LOG("[SEC]try to ping cmdqSecTL+...\n"); 
        status = cmdq_sec_send_context_session_message(handle, CMD_CMDQ_TL_TEST_HELLO_TL, NULL, CMDQ_INVALID_THREAD, NULL);
        CMDQ_LOG("[SEC]try to ping cmdqSecTL-..., status[%d]\n", status); 

        CMDQ_LOG("[SEC]try to ping cmdqSecTL, cmdqSecDr+...\n"); 
        status = cmdq_sec_send_context_session_message(handle, CMD_CMDQ_TL_TEST_DUMMY, NULL, CMDQ_INVALID_THREAD, NULL);
        CMDQ_LOG("[SEC]try to ping cmdqSecTL, cmdqSecDr-..., status[%d]\n", status); 
    }while(0);
}


//
// sec dumper
//
static cmdqSecContextHandle gSecDciHandle = NULL;

int32_t cmdq_sec_dumper_init(void)
{
    int32_t status = 0;
    const struct mc_uuid_t uuid = CMDQ_DR_UUID;

    // init secure session
    gSecDciHandle = cmdq_sec_context_handle_create(0); // set tgid 0 for don't care tgid here
    gSecDciHandle->uuid = uuid;
    status = cmdq_sec_init_session_unlocked(
                &(gSecDciHandle->uuid),
                (uint8_t**)( &(gSecDciHandle->iwcMessage) ),
                sizeof(dciCmdqMessage_t),
                &(gSecDciHandle->sessionHandle),
                &(gSecDciHandle->state),
                &(gSecDciHandle->openMobicoreByOther));
    return status;
}

int32_t cmdq_sec_dumper_dump(void)
{
    dciCmdqMessage_t* pDciMessage;
    int32_t status = 0;

    do
    {
        if(NULL == gSecDciHandle)
        {
            break;
        }

        // send msg to seucre world to dump
        pDciMessage = (dciCmdqMessage_t*)(gSecDciHandle->iwcMessage);
        pDciMessage->cmd.header.commandId = DCI_CMDQ_DUMP;
        pDciMessage->cmd.len = sizeof(dciCmdqDataParam_t);
        status = cmdq_sec_execute_session_unlocked(&(gSecDciHandle->sessionHandle), &(gSecDciHandle->state), -1);

        //
        // do not case result code here,
        //      int32_t executeResult = pDciMessage->rsp.header.returnCode;
    }while(0);

    return status;
}

int32_t cmdq_sec_dumper_deinit(void)
{
    int32_t status = 0;

    cmdq_sec_deinit_session_unlocked(
                (uint8_t**)(&(gSecDciHandle->iwcMessage)),
                &(gSecDciHandle->sessionHandle),
                gSecDciHandle->state,
                gSecDciHandle->openMobicoreByOther);
    return status;
}

#endif //__CMDQ_SECURE_PATH_SUPPORT__


int32_t cmdq_exec_task_secure_with_retry(
    TaskStruct *pTask,
    int32_t thread,
    const uint32_t maxRetry)
{
#if defined(__CMDQ_SECURE_PATH_SUPPORT__)
    int32_t status = 0;
    int32_t i = 0;
    do
    {
        uint32_t commandId = cmdq_sec_get_commandId();
        commandId = (0 < commandId) ? (commandId) : (CMD_CMDQ_TL_SUBMIT_TASK);

        status = cmdq_sec_submit_to_secure_world(commandId, pTask, thread, NULL);
        if(0 > status)
        {
            CMDQ_ERR("%s[%d]\n", __FUNCTION__, status);
            break;
        }
        i ++;
    }while(i < maxRetry);

    return status;

#else
    CMDQ_ERR("secure path not support\n");
    return -EFAULT;
#endif //__CMDQ_SECURE_PATH_SUPPORT__
}


//--------------------------------------------------------------------------------------------

cmdqSecContextHandle cmdq_sec_find_context_handle_unlocked(uint32_t tgid)
{
    cmdqSecContextHandle handle = NULL;

    do
    {
        struct cmdqSecContextStruct *secContextEntry = NULL;
        struct list_head *pos = NULL;
        list_for_each(pos, &gCmdqSecContextList)
        {
            secContextEntry = list_entry(pos, struct cmdqSecContextStruct, listEntry);
            if(secContextEntry && tgid == secContextEntry->tgid)
            {
                handle = secContextEntry;
                break;
            }
        }
    }while(0);

    CMDQ_MSG("SecCtxHandle_SEARCH: H[0x%p], tgid[%d]\n", handle, tgid);
    return handle;
}


int32_t cmdq_sec_release_context_handle_unlocked(cmdqSecContextHandle handle)
{
    int32_t status = 0;

    do
    {
        handle->referCount --;
        if(0 < handle->referCount)
        {
            break;
        }

        // when reference count <= 0, this secContext is not used. so, we should:
        // 1. clean up secure path in secure world
        #if defined(__CMDQ_SECURE_PATH_SUPPORT__)
        switch (handle->state)
        {
            case IWC_SES_ON_TRANSACTED:
            case IWC_SES_TRANSACTED:
                CMDQ_VERBOSE("SecCtxHandle_RELEASE: ask cmdqSecTl resource free\n");
                cmdq_sec_submit_to_secure_world (CMD_CMDQ_TL_RES_RELEASE, NULL, CMDQ_INVALID_THREAD, NULL);
                break;
            case IWC_SES_MSG_PACKAGED:
            case IWC_SES_OPENED:
            case IWC_WSM_ALLOCATED:
            case IWC_MOBICORE_OPENED:
            case IWC_INIT:
            default:
                CMDQ_VERBOSE("SecCtxHandle_RELEASE: no need cmdqSecTl resource free\n");
                break;
        }
        #endif

        // 2. delete secContext from list
        list_del(&(handle->listEntry));

        // 3. release secure path resource in normal world
        #if defined(__CMDQ_SECURE_PATH_SUPPORT__)
        {
            // because mc_open_session is too slow, we open once for each process context
            // and delay session clean up till process closes CMDQ device node
            #if (CMDQ_OPEN_SESSION_ONCE)
            cmdq_sec_teardown_context_session(handle);
            #endif
        }
        #endif // defined(__CMDQ_SECURE_PATH_SUPPORT__)
    }while(0);
    return status;
}

int32_t cmdq_sec_release_context_handle(uint32_t tgid)
{
    int32_t status = 0;
    cmdqSecContextHandle handle = NULL;

    mutex_lock(&gCmdqSecContextLock);
    smp_mb();

    handle = cmdq_sec_find_context_handle_unlocked(tgid);
    if(handle)
    {
        CMDQ_MSG("SecCtxHandle_RELEASE: +tgid[%d], handle[0x%p], iwcState[%d]\n", tgid, handle, handle->state);
        status = cmdq_sec_release_context_handle_unlocked(handle);
        CMDQ_MSG("SecCtxHandle_RELEASE: -tgid[%d], status[%d]\n", tgid, status);
    }
    else
    {
        status = -1;
        CMDQ_ERR("SecCtxHandle_RELEASE: err[secCtxHandle not exist], tgid[%d]\n", tgid);
    }

    mutex_unlock(&gCmdqSecContextLock);
    return status;
}

cmdqSecContextHandle cmdq_sec_context_handle_create(uint32_t tgid)
{
    cmdqSecContextHandle handle = NULL;
    handle = kmalloc(sizeof(uint8_t*) * sizeof(cmdqSecContextStruct), GFP_ATOMIC);
    if (handle)
    {
        handle->state = IWC_INIT;
        handle->iwcMessage = NULL;

        handle->tgid = tgid;
        handle->referCount = 0;
        handle->openMobicoreByOther = 0;
    }
    else
    {
        CMDQ_ERR("SecCtxHandle_CREATE: err[LOW_MEM], tgid[%d]\n", tgid);
    }

    CMDQ_MSG("SecCtxHandle_CREATE: create new, H[0x%p], tgid[%d]\n", handle, tgid);
    return handle;
}

cmdqSecContextHandle cmdq_sec_acquire_context_handle(uint32_t tgid)
{
    cmdqSecContextHandle handle = NULL;

    mutex_lock(&gCmdqSecContextLock);
    smp_mb();
    do
    {
        // find sec context of a process
        handle = cmdq_sec_find_context_handle_unlocked(tgid);
        // if it dose not exist, create new one
        if(NULL == handle)
        {
            handle = cmdq_sec_context_handle_create(tgid);
            list_add_tail(&(handle->listEntry), &gCmdqSecContextList);
        }
    }while(0);

    // increase caller referCount
    if(handle)
    {
        handle->referCount ++;
    }

    CMDQ_MSG("[CMDQ]SecCtxHandle_ACQUIRE, H[0x%p], tgid[%d], refCount[%d]\n", handle, tgid, handle->referCount);
    mutex_unlock(&gCmdqSecContextLock);

    return handle;
}

void cmdq_sec_dump_context_list(void)
{
    struct cmdqSecContextStruct *secContextEntry = NULL;
    struct list_head *pos = NULL;

    CMDQ_ERR("=============== [CMDQ] sec context ===============\n");

    list_for_each(pos, &gCmdqSecContextList)
    {
        secContextEntry = list_entry(pos, struct cmdqSecContextStruct, listEntry);
        CMDQ_ERR("secCtxHandle[0x%p], tgid_%d[referCount: %d], state[%d], iwc[0x%p]\n",
            secContextEntry,
            secContextEntry->tgid,
            secContextEntry->referCount,
            secContextEntry->state,
            secContextEntry->iwcMessage);
    }
}

void cmdqSecDeInitialize(void)
{
    // do nothing for secCtxHandles
    // secCtxHandles release cleana up are aligned with cmdq device node's life cycle

    // deinit systrace
#if SYSTRACE_PROFILING
    systrace_deinit(CMDQ_SEC_SYSTRACE_MODULE_ID); 
#endif    
    return;
}


void cmdqSecInitialize(void)
{
    INIT_LIST_HEAD(&gCmdqSecContextList);
    
    // init systrace
#if SYSTRACE_PROFILING
    CMDQ_LOG("sec systrace buffer:%d, sizeof(struct systrace_log):%d\n", CMDQ_SEC_SYSTRACE_BUFFER_SIZE, sizeof(struct systrace_log));
    systrace_init(CMDQ_SEC_SYSTRACE_MODULE_ID, "cmdq", "cmdqSec", CMDQ_SEC_SYSTRACE_BUFFER_SIZE, &cmdq_secure_systrace_funcs);
#endif
}

void cmdq_sec_set_commandId(uint32_t cmdId)
{
    atomic_set(&gDebugSecCmdId, cmdId);
}

const uint32_t cmdq_sec_get_commandId(void)
{
    return (uint32_t)(atomic_read(&gDebugSecCmdId));
}

void cmdq_debug_set_sw_copy(int32_t value)
{
    atomic_set(&gDebugSecSwCopy, value);
}

int32_t cmdq_debug_get_sw_copy(void)
{
    return atomic_read(&gDebugSecSwCopy);
}

