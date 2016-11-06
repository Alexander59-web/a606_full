#ifndef __CMDQSECTL_API_H__
#define __CMDQSECTL_API_H__

/**
 * Command IDs for normal world(TLC or linux kernel) to Trustlet
 */
#define CMD_CMDQ_TL_SUBMIT_TASK      1  
#define CMD_CMDQ_TL_RES_RELEASE      2  // release resource in secure path 

#define CMD_CMDQ_TL_TEST_HELLO_TL    (4000) // entry cmdqSecTl, and do nothing
#define CMD_CMDQ_TL_TEST_DUMMY       (4001) // entry cmdqSecTl and cmdqSecDr, and do nothing
#define CMD_CMDQ_TL_TEST_SMI_DUMP    (4002)
#define CMD_CMDQ_TL_DEBUG_SW_COPY    (4003)
#define CMD_CMDQ_TL_TRAP_DR_INFINITELY (4004)
#define CMD_CMDQ_TL_DUMP (4005)


/**
 * Termination codes
 */
#define EXIT_ERROR                  ((uint32_t)(-1))

/**
 * TCI message data: see cmdq_sec_iwc_common.h
 */
 
/**
 * Trustlet UUID: 
 * filename of output bin is {TL_UUID}.tlbin
 */
#define TL_CMDQ_UUID { { 9, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }

#endif // __CMDQSECTEST_API_H__
