/*
 * Copyright (c) 2013 TRUSTONIC LIMITED
 * All rights reserved
 *
 * The present software is the confidential and proprietary information of
 * TRUSTONIC LIMITED. You shall not disclose the present software and shall
 * use it only in accordance with the terms of the license agreement you
 * entered into with TRUSTONIC LIMITED. This software may be subject to
 * export or import laws in certain countries.
 */

/**
 * @file   drSvp_Api.h
 * @brief  Contains DCI command definitions and data structures
 *
 */

#ifndef __DRSVPAPI_H__
#define __DRSVPAPI_H__

#include "dci.h"

//#define _TEE_DEBUG_VERBOSE_

/**
 * Command ID's
  */
#define CMD_ID_01       1
#define CMD_ID_02       2
/*... add more command ids when needed */
// debugging purpose
#define CMD_SVP_DRV_DUMP_OVL_REGISTER 3
#define CMD_SVP_DRV_DUMP_DISP_REGISGER 4
#define CMD_SVP_DRV_DUMMY 11

// OVL
#define CMD_SVP_DRV_SWITCH_OVL_LAYER 5
#define CMD_SVP_DRV_CONFIG_OVL_LAYER 6
#define CMD_SVP_DRV_NOTIFY_OVL_LAYER_CONFIG 9

// RDMA
#define CMD_SVP_DRV_CONFIG_RDMA 7
#define CMD_SVP_DRV_NOTIFY_RDMA_CONFIG 8

// WDMA
#define CMD_SVP_DRV_CONFIG_WDMA_ADDRESS 10
#define CMD_SVP_DRV_CONFIG_WDMA 12
#define CMD_SVP_DRV_NOTIFY_WDMA_CONFIG 13
#define CMD_SVP_DRV_CONFIG_RDMA_ADDRESS 14

#define CMD_SVP_DRV_CLEAR_RDMA_INTR 15
#define CMD_SVP_DRV_CLEAR_WDMA_INTR 16

#define CMD_SVP_DRV_START_ISRH 17
#define CMD_SVP_DRV_STOP_ISRH 18

#define CMD_SVP_DRV_CLEAR_OVL_INTR 19

#define CMD_SVP_DRV_CONFIG_OVL_ROI 20

#define CMD_SVP_DRV_RESET_OVL 21

/**
 * command message.
 *
 * @param len Lenght of the data to process.
 * @param data Data to be processed
 */
typedef struct {
    dciCommandHeader_t  header;     /**< Command header */
    uint32_t            len;        /**< Length of data to process */
} dci_cmd_t;


/**
 * Response structure
 */
typedef struct {
    dciResponseHeader_t header;     /**< Response header */
    uint32_t            len;
} dci_rsp_t;



/**
 * Sample 01 data structure
 */
typedef struct {
    uint32_t len;
    uint32_t addr;
} sample01_t;


/**
 * Sample 02 data structure
 */
typedef struct {
    uint32_t data;
} sample02_t;

/**
 * we define dummy struct definition for RDMA and WDMA config
 */
typedef struct _RDMA_CONFIG_STRUCT_DUMMY
{
    unsigned int idx;            // instance index
    unsigned int mode;          // data mode
    unsigned int inputFormat;
    unsigned int address;
    unsigned int pitch;
    unsigned int isByteSwap;
    unsigned int outputFormat;
    unsigned int width;
    unsigned int height;
    unsigned int isRGBSwap;
} RDMA_CONFIG_STRUCT_DUMMY;

typedef struct _WDMA_CONFIG_STRUCT_DUMMY
{
    unsigned int idx;           // module idx
    unsigned int inputFormat;
    unsigned int srcWidth;
    unsigned int srcHeight;     // input
    unsigned int clipX;
    unsigned int clipY;
    unsigned int clipWidth;
    unsigned int clipHeight;    // clip
    unsigned int outputFormat;
    unsigned int dstAddress;
    unsigned int dstWidth;     // output
    unsigned int useSpecifiedAlpha;
    unsigned char alpha;
} WDMA_CONFIG_STRUCT_DUMMY;

/**
 * DCI message data.
 */
typedef struct {
    union {
        dci_cmd_t     command;
        dci_rsp_t     response;
    };

    uint32_t    Value1;
    uint32_t    Value2;
    uint32_t    ResultData;
} dciMessage_t;

/**
 * Driver UUID. Update accordingly after reserving UUID
 */
#define SVP_DRV_DBG_UUID { { 8, 0xc, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }


#endif // __DRSVPAPI_H__
