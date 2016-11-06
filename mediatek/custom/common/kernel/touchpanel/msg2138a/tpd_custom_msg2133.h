#ifndef __TPD_CUSTOM_MSG2133_H__
#define __TPD_CUSTOM_MSG2133_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_I2C_NUMBER           0

//#define TOUCH_ADDR_MSG20XX 0Xc0

#define TPD_DELAY                (2*HZ/100)

//#define SWAP_X_Y
//#define REVERSE_Y
//#define REVERSE_X

#define TPD_POWER_SOURCE_CUSTOM         MT6323_POWER_LDO_VGP1

#define MAX_TOUCH_FINGER      2
#define MS_TS_MSG21XX_X_MAX   480//480
#define MS_TS_MSG21XX_Y_MAX   854//800
#define REPORT_PACKET_LENGTH  8//2--8--128--80

#define TPD_HAVE_BUTTON
#define HAVE_TOUCH_KEY
#define TPD_BUTTON_HEIGH        (60)
#define TPD_KEY_COUNT           3
//#define TPD_KEYS                {KEY_BACK, KEY_HOMEPAGE, KEY_MENU}
#define TPD_KEYS                {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM        {{80,900,80,60},{240,900,80,60},{400,900,80,60}}

#endif /* TOUCHPANEL_H__ */

#define __MSG_DMA_MODE__

#define ITO_TEST
//#define WT_CTP_GESTURE_SUPPORT
#define PROC_FIRMWARE_UPDATE


#define TP_DEBUG	printk//(x)		//x

#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)

#ifdef WT_CTP_GESTURE_SUPPORT
#define GTP_GESTURE_TPYE_STR  "KUDLR"
#define GTP_GLOVE_SUPPORT_ONOFF  'N'	// 'N' is off
#define GTP_GESTURE_SUPPORT_ONOFF   'Y'	// 'N' is off
#define GTP_DRIVER_VERSION          "GTP_V1.0_20140327"
#endif


