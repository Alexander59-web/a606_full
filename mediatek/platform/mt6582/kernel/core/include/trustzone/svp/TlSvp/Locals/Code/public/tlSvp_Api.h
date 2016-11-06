#ifndef TLSVP_API_H_
#define TLSVP_API_H_

#include "tci.h"

//#define _TEE_DEBUG_VERBOSE_

/**
 * Command ID's for communication Trustlet Connector -> Trustlet.
 */
// TODO: define proper command for normal world caller
#define CMD_SVP_CONFIG_OVL_LAYER 1
#define CMD_SVP_SWITCH_OVL_LAYER 2
#define CMD_SVP_DUMP_OVL_REGISTER 3
#define CMD_SVP_DUMP_DISP_REGISTER 4
#define CMD_SVP_REGISTER_IRQ 5
#define CMD_SVP_UNREGISTER_IRQ 6

#define CMD_SVP_DUMMY 11

#define CMD_SVP_DUMMY_IPC 12
#define CMD_SVP_DUMMY_MEM_ALLOC 13

#define CMD_SVP_CONFIG_OVL_SECURE 14
#define CMD_SVP_CONFIG_OVL_NONSECURE 15

#define CMD_SVP_SWITCH_SECURE_PORT 16
#define CMD_SVP_SWITCH_DEBUG_LAYER 19

#define CMD_SVP_NOTIFY_OVL_LAYER_INFO 17
#define CMD_SVP_NOTIFY_OVL_CONFIG 18

#define CMD_SVP_MAP_OVL_CONFIG 20

#define CMD_SVP_MAP_NONSECURE_WDMA_MVA 21
#define CMD_SVP_MAP_NONSECURE_RDMA_MVA 22

#define CMD_SVP_CONFIG_WDMA_ADDR 23
#define CMD_SVP_CONFIG_RDMA_ADDR 24

#define CMD_SVP_CONFIG_RDMA 25
#define CMD_SVP_NOTIFY_RDMA_CONFIG 26

#define CMD_SVP_ALLOC_DECOUPLE_BUFFER 27

#define CMD_SVP_MAP_DECOUPLE_MVA 28

#define CMD_SVP_ALLOC_DECOUPLE_BUF 29

#define CMD_SVP_NOTIFY_WDMA_CONFIG_SHARED 30

#define CMD_SVP_CONFIG_WDMA 31

#define CMD_SVP_PREPARE_DECOUPLE_BUFFER_HANDLE 32

#define CMD_SVP_START_ISRH 33
#define CMD_SVP_STOP_ISRH 34

#define CMD_SVP_MAP_INTR_INFO 35

#define CMD_SVP_CLEAR_RDMA_INTR 36
#define CMD_SVP_CLEAR_WDMA_INTR 37

#define CMD_SVP_SET_RDMA_APC 38
#define CMD_SVP_SET_WDMA_APC 39
#define CMD_SVP_SET_OVL_APC 40

#define CMD_SVP_CONFIG_OVL_ROI 41

#define CMD_SVP_ENABLE_SECURE_SYSTRACE 42
#define CMD_SVP_DISABLE_SECURE_SYSTRACE 43
#define CMD_SVP_MAP_SYSTRACE_BUF 44
#define CMD_SVP_PAUSE_SECURE_SYSTRACE 45
#define CMD_SVP_RESUME_SECURE_SYSTRACE 46

#define CMD_SVP_MAP_OVL_CURRENT_MVA 47

#define CMD_SVP_RELEASE_ALL_APC 48

#define CMD_DAPC_ENABLE_SECURE_SYSTRACE 49
#define CMD_DAPC_DISABLE_SECURE_SYSTRACE 50
#define CMD_DAPC_PAUSE_SECURE_SYSTRACE 51
#define CMD_DAPC_RESUME_SECURE_SYSTRACE 52

#define CMD_SVP_RESET_OVL 53

/* secure port id for HW engines. */
#define SECURE_PORT_OVL_0 0
#define SECURE_PORT_RDMA  1
#define SECURE_PORT_WDMA  2

#define MAX_BUFFER_COUNT 3
/**
 * Return codes
 */


/**
 * Termination codes
 */
#define EXIT_ERROR                  ((uint32_t)(-1))

/**
 * command message.
 *
 * @param len Lenght of the data to process.
 * @param data Data to processed (cleartext or ciphertext).
 */
typedef struct {
    tciCommandHeader_t  header;     /**< Command header */
    uint32_t            len;        /**< Length of data to process or buffer */
    uint32_t            respLen;    /**< Length of response buffer */
} tci_cmd_t;

/**
 * Response structure Trustlet -> Trustlet Connector.
 */
typedef struct {
    tciResponseHeader_t header;     /**< Response header */
    uint32_t            len;
} tci_rsp_t;

#define OVL_LAYER_NUM  4
/**
 * TCI message data.
 */
typedef struct {
    union {
      tci_cmd_t     cmdSvp;
      tci_rsp_t     rspSvp;
    };
    uint32_t    Value1;
    uint32_t    Value2;
    uint32_t    ResultData;
    // all from OVL_CONFIG_STRUCT
    uint32_t layer[OVL_LAYER_NUM];
    uint32_t layer_en[OVL_LAYER_NUM]; // origin: enum OVL_LAYER_SOURCE
    uint32_t source[OVL_LAYER_NUM];
    uint32_t fmt[OVL_LAYER_NUM];
    uint32_t addr[OVL_LAYER_NUM];
    uint32_t vaddr[OVL_LAYER_NUM];
    uint32_t src_x[OVL_LAYER_NUM];
    uint32_t src_y[OVL_LAYER_NUM];
    uint32_t src_w[OVL_LAYER_NUM];
    uint32_t src_h[OVL_LAYER_NUM];
    uint32_t src_pitch[OVL_LAYER_NUM];
    uint32_t dst_x[OVL_LAYER_NUM];
    uint32_t dst_y[OVL_LAYER_NUM];
    uint32_t dst_w[OVL_LAYER_NUM];
    uint32_t dst_h[OVL_LAYER_NUM];
    uint32_t keyEn[OVL_LAYER_NUM];
    uint32_t key[OVL_LAYER_NUM];
    uint32_t aen[OVL_LAYER_NUM];
    uint32_t alpha[OVL_LAYER_NUM]; // origin: unsigned char

    uint32_t isTdshp[OVL_LAYER_NUM];
    uint32_t isDirty[OVL_LAYER_NUM];

    uint32_t buff_idx[OVL_LAYER_NUM]; // origin: int
    uint32_t identity[OVL_LAYER_NUM]; // origin: int
    uint32_t connected_type[OVL_LAYER_NUM]; // origin: int
    uint32_t security[OVL_LAYER_NUM];

    // all from RDMA_CONFIG_STRUCT
    uint32_t idx;
    uint32_t mode; // orgin: enum RDMA_MODE
    uint32_t inputFormat; // orgin: DpColorFormat
    uint32_t address;
    uint32_t pitch;
    uint32_t isByteSwap; // origin: bool
    uint32_t outputFormat; // origin: enum RDMA_OUTPUT_FORMAT
    uint32_t width;
    uint32_t height;
    uint32_t isRGBSwap; // origin: bool
    // add for ultra setting
    uint32_t ultraLevel;
    uint32_t enableUltra;

    // all from WDMA_CONFIG_STURCT
    uint32_t wdma_idx;
    uint32_t wdma_inputFormat;
    uint32_t wdma_srcWidth;
    uint32_t wdma_srcHeight;
    uint32_t wdma_clipX;
    uint32_t wdma_clipY;
    uint32_t wdma_clipWidth;
    uint32_t wdma_clipHeight;
    uint32_t wdma_outputFormat;
    uint32_t wdma_dstAddress;
    uint32_t wdma_dstWidth;
    uint32_t wdma_dstPitch;
    uint32_t wdma_dstPitchUV;
    uint32_t wdma_useSpecifiedAlpha; // origin: bool
    uint32_t wdma_alpha; // origin: unsigned char

    // current cached ovl buffer setting
    uint32_t ovl_layer_en[OVL_LAYER_NUM];
    uint32_t ovl_addr[OVL_LAYER_NUM];
    uint32_t ovl_size[OVL_LAYER_NUM];

    // decouple mva for main display
    uint32_t decouple_mva[MAX_BUFFER_COUNT];

} tciMessage_t;

struct ALLOC_DECOUPLE_BUF_STRUCT
{
    unsigned int w;
    unsigned int h;
    unsigned int bpp;
    unsigned int *phandle;
    unsigned int buf_num;
};

/**
 * Trustlet UUID.
 */
#define SVP_TL_DBG_UUID { { 8, 0xb, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }


#endif // TLSVP_API_H_
