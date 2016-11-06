#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

#include <cust_eint.h>
#include <linux/jiffies.h>
/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
//#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE      MT65XX_POWER_LDO_VGP2     
#define TPD_I2C_NUMBER           0
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_VELOCITY_CUSTOM_X 10
#define TPD_VELOCITY_CUSTOM_Y 12
//#define REVERSE_X

#define TPD_DELAY                (2*HZ/100)
#define TPD_RES_X                480  //jin
#define TPD_RES_Y                854  //jin
//#define SWITCH_XY  //jin 20120808 交换XY的值
//#define REVERSE_X
//#define REVERSE_Y
//#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
#define TPD_HAVE_BUTTON
//#define TPD_HAVE_TREMBLE_ELIMINATION

#define USE_RTP_RST  //用于CTP RST管脚1.8V和2.8V兼容

#define TPD_BUTTON_HEIGH        (60)
#define TPD_KEY_COUNT           3
//#define TPD_KEYS                { KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS                { KEY_BACK,KEY_HOMEPAGE,KEY_MENU}
#define TPD_KEYS_DIM        {{40,900,80,TPD_BUTTON_HEIGH},{160,900,80,TPD_BUTTON_HEIGH},{280,900,80,TPD_BUTTON_HEIGH}}

#define FT6206_UPDATE				1
#ifdef FT6206_UPDATE
#define FT6206_SOFTWARE_UPDATE		1
//#define FT6206_AUTO_UPDATE		0

#define FT6206_UPDATE_DMA_WAY


#define IC_FT5X06	0	/*x=2,3,4*/
#define IC_FT5606	1	/*ft5506*/
#define IC_FT5316	2	/*5x16*/
#define IC_FT6206	3


#define FT_UPGRADE_AA	0xAA
#define FT_UPGRADE_55 	0x55


/*upgrade config of FT5606*/
#define FT5606_UPGRADE_AA_DELAY 		50
#define FT5606_UPGRADE_55_DELAY 		10
#define FT5606_UPGRADE_ID_1			0x79
#define FT5606_UPGRADE_ID_2			0x06
#define FT5606_UPGRADE_READID_DELAY 	100
#define FT5606_UPGRADE_EARSE_DELAY	2000


/*upgrade config of FT5316*/
#define FT5316_UPGRADE_AA_DELAY 		50
#define FT5316_UPGRADE_55_DELAY 		30
#define FT5316_UPGRADE_ID_1			0x79
#define FT5316_UPGRADE_ID_2			0x07
#define FT5316_UPGRADE_READID_DELAY 	1
#define FT5316_UPGRADE_EARSE_DELAY	1500


/*upgrade config of FT5x06(x=2,3,4)*/
#define FT5X06_UPGRADE_AA_DELAY 		50
#define FT5X06_UPGRADE_55_DELAY 		30
#define FT5X06_UPGRADE_ID_1			0x79
#define FT5X06_UPGRADE_ID_2			0x03
#define FT5X06_UPGRADE_READID_DELAY 	1
#define FT5X06_UPGRADE_EARSE_DELAY	2000

/*upgrade config of FT6206*/
#define FT6206_UPGRADE_AA_DELAY 		50//100//60
#define FT6206_UPGRADE_55_DELAY 		30//10
#define FT6206_UPGRADE_ID_1			0x79
#define FT6206_UPGRADE_ID_2			0x08
#define FT6206_UPGRADE_READID_DELAY 	10
#define FT6206_UPGRADE_EARSE_DELAY	2000

#define DEVICE_IC_TYPE	IC_FT6206

#define FTS_PACKET_LENGTH        		128
#define FTS_SETTING_BUF_LEN      		128
#define FTS_DMA_BUF_SIZE 				1024

#define FTS_UPGRADE_LOOP	30

struct fw_version {
	unsigned short major;
	unsigned short minor;
};
#endif



#endif /* TOUCHPANEL_H__ */
