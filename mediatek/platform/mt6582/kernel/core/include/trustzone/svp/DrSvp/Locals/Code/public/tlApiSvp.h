#ifndef __TLAPISVP_H__
#define __TLAPISVP_H__

#include "TlApi/TlApiError.h"

#define OVL_LAYER_NUM  4
/* Marshaled function parameters.
 * structs and union of marshaling parameters via tlApi_Svp.
 *
 * @note The structs can NEVER be packed !
 * @note The structs can NOT used via sizeof(..) !
 */
typedef struct {
    uint32_t commandId;
    uint32_t x; /* use it for a buffer length */
    uint32_t y; /* use it for buffer address (pointer) */
    uint32_t result;

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
} tlApiSvp_t, *tlApiSvp_ptr;

/** execute specified function.
 *
 * @param sid  session id
 * @param data Pointer to the tlApiSvp_t structure
 * @return TLAPI_OK if data has successfully been processed.
 *
 * note that [tlApiResult_t] is actually the same with [tciReturnCode_t] (both unsigned int)
 */
_TLAPI_EXTERN_C tlApiResult_t tlApiOperate(uint32_t sid, tlApiSvp_ptr data);

/** dump OVL hw registers (to log output)
 *
 * @param sid session id
 */
_TLAPI_EXTERN_C void tlApiDumpOvlRegister(uint32_t sid);

/** dump display hw registers (to log output)
 *
 * @param sid session id
 */
_TLAPI_EXTERN_C void tlApiDumpDispRegister(uint32_t sid);

/** map secure systrace buffer
 *
 * @param sid session id
 * @param buf_ptr buffer TA address
 * @param buf_size the size of buffer
 * @param buf_head_ptr the TA address of head variable
 * @return TLAPI_OK if data has successfully been processed.
 */
_TLAPI_EXTERN_C tlApiResult_t tlApiMapSystraceBuf(uint32_t sid, uint32_t buf_ptr, uint32_t buf_size, uint32_t *buf_head_ptr);

#endif // __TLAPISVP_H__

