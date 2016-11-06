/*
 * This software is licensed under the terms of the GNU General Public 
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms. 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * * VERSION      	DATE			AUTHOR          Note
 *    1.0		  2013-7-16			Focaltech        initial  based on MTK platform
 * 
 */

#include "tpd.h"

#include "tpd_custom_fts.h"
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef TPD_SYSFS_DEBUG
#include "focaltech_ex_fun.h"
#endif
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include "cust_gpio_usage.h"


  struct Upgrade_Info fts_updateinfo[] =
{
        {0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
        {0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
	{0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
};
				
struct Upgrade_Info fts_updateinfo_curr;

#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag 			= 0;
static u8 tpd_proximity_flag_one 		= 0; //add for tpd_proximity by wangdongfang
static u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away

u8	fts_tpd_proximity_flag	= 0;
u8	fts_tpd_proximity_detect = 1;
#endif
/*--------------------------------------yanghaichao add for esd_protect 2014-02-12--------------------------------------*/
//#define GTP_ESD_PROTECT  //for tp esd check
#ifdef GTP_ESD_PROTECT
#define TPD_ESD_CHECK_CIRCLE        200
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
#if 0
static u8 run_odd_run_flag = 0;
#endif
static unsigned long esd_check_circle = TPD_ESD_CHECK_CIRCLE;
static u8 run_check_91_register = 0;
#endif

extern int send_far_flag;// add by zhaofei - 2014-01-02-20-19
extern int ps_close_flag;



/*-----------------------------------------yanghaichao add end-----------------------------------------------*/
#define LCT_ADD_TP_VERSION		0 //yanghaichao add tpd_info 2014-01-21
static int count_irq = 0; //add for esd
/*-----------------------------yanghaichao add for device info 2014-01-21--------------------------------*/
#ifdef SLT_DEVINFO_CTP

#define SLT_DEVINFO_CTP_DEBUG



#include  <linux/dev_info.h>
static int devinfo_first=0;
struct devinfo_struct *h_DEVINFO_ctp;   //suppose 10 max lcm device 
static char  temp_ver[64];
static int temp_pid;
 unsigned char ft5x06_vendor_id = 0xFF;
#define FT6x06_REG_VENDOR_ID			0xA8
//The followd code is for GTP style
static void devinfo_ctp_regchar(char *module,char * vendor,char *version,char *used)
{
 	
	h_DEVINFO_ctp =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);	
	h_DEVINFO_ctp->device_type="CTP";
	h_DEVINFO_ctp->device_module=module;
	h_DEVINFO_ctp->device_vendor=vendor;
	h_DEVINFO_ctp->device_ic="FT6306";
	h_DEVINFO_ctp->device_info=DEVINFO_NULL;
	h_DEVINFO_ctp->device_version=version;
	h_DEVINFO_ctp->device_used=used;
#ifdef SLT_DEVINFO_CTP_DEBUG
		printk("[DEVINFO CTP]registe CTP device! type:<%s> module:<%s> vendor<%s> ic<%s> version<%s> info<%s> used<%s>\n",
				h_DEVINFO_ctp->device_type,h_DEVINFO_ctp->device_module,h_DEVINFO_ctp->device_vendor,
				h_DEVINFO_ctp->device_ic,h_DEVINFO_ctp->device_version,h_DEVINFO_ctp->device_info,h_DEVINFO_ctp->device_used);
#endif
DEVINFO_CHECK_DECLARE(h_DEVINFO_ctp->device_type,h_DEVINFO_ctp->device_module,h_DEVINFO_ctp->device_vendor,
				h_DEVINFO_ctp->device_ic,h_DEVINFO_ctp->device_version,h_DEVINFO_ctp->device_info,h_DEVINFO_ctp->device_used);

}


#endif
/*-------------------------------------------------yanghaichao add end ---------------------------------*/
extern struct tpd_device *tpd;
 
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
 
 
static void tpd_eint_interrupt_handler(void);
// start:Here maybe need port to different platform,like MT6575/MT6577
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
// End:Here maybe need port to different platform,like MT6575/MT6577
 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag 					= 0;
static int tpd_halt						= 0;
static int point_num 					= 0;
static int p_point_num 					= 0;



//#define TPD_CLOSE_POWER_IN_SLEEP
#define TPD_OK 							0
//register define
#define DEVICE_MODE 					0x00
#define GEST_ID 						0x01
#define TD_STATUS 						0x02
//point1 info from 0x03~0x08
//point2 info from 0x09~0x0E
//point3 info from 0x0F~0x14
//point4 info from 0x15~0x1A
//point5 info from 0x1B~0x20
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 			3
//extern int tpd_mstar_status ;  // compatible mstar and ft6306 chenzhecong

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5206
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 			10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 			10
#endif

#define TOUCH_IOC_MAGIC 				'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)


static int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
static int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;

 s32 fts_i2c_smbus_read_i2c_block_data(const struct i2c_client *client,\
                                          u8 command, u8 length, u8 *values)
{
	int ret = 0;
	mutex_lock(&i2c_access);
	ret = i2c_smbus_read_i2c_block_data(client, command, length, values);
	mutex_unlock(&i2c_access);
	return ret;
}                                      
 s32 fts_i2c_smbus_write_i2c_block_data(const struct i2c_client *client,\
                                           u8 command, u8 length,\
                                           const u8 *values)
{
	int ret = 0;
	mutex_lock(&i2c_access);
	ret = i2c_smbus_write_i2c_block_data(client, command, length, values);
	mutex_unlock(&i2c_access);
	return ret;
}                                           


static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;

	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	//char strbuf[256];
	void __user *data;
	
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TPD_NAME,
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[10];
    int x[10];
    int p[10];
    int id[10];
};
 
static const struct i2c_device_id ft5206_tpd_id[] = {{TPD_NAME,0},{}};
//unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO(TPD_NAME, (0x70>>1))};
 
static struct i2c_driver tpd_i2c_driver = {
  	.driver = {
	 	.name 	= TPD_NAME,
	//	.owner 	= THIS_MODULE,
  	},
  	.probe 		= tpd_probe,
  	.remove 	= __devexit_p(tpd_remove),
  	.id_table 	= ft5206_tpd_id,
  	.detect 	= tpd_detect,
// 	.shutdown	= tpd_shutdown,
//  .address_data = &addr_data,
};
/*------------------------yanghaichao add for tpd_info 2014-01-21-------------------------------*/
#if LCT_ADD_TP_VERSION// add by yanghaichao for tpd_info 2014-01-18
#define FT5436_CTP_PROC_FILE "tp_info"
static struct proc_dir_entry *ft5436_g_ctp_proc = NULL;

static int ft5436_ctp_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int cnt= 0;
	printk("Enter ctp_proc_read.\n");
	if(off != 0)
		return 0;

	cnt = sprintf(page, "pid:%s\n", temp_ver);
	
	*eof = 1;
	printk("vid:%4x",temp_ver);
	printk("Leave ctp_proc_read. cnt = %d\n", cnt);
	return cnt;
}
#endif
#ifdef TPD_PROXIMITY

static int tpd_enable_ps(int enable)
{
	u8 state = 0x00;
	int ret = -1;

//	i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
     //fts_i2c_Read(i2c_client, 0xB0, 1, &state,1);//  Zhuqy ADD for Psensor verify	//luowj@20140307
	
	//printk("[proxi_5206]read: 999 0xb0's value is 0x%02X\n", state);
	if (enable){
		//state |= 0x01;
		state = 0x01;	//luowj@20140307
		fts_tpd_proximity_flag	= 1;
		tpd_proximity_flag = 1;
ps_close_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is on\n");	
	}else{
		fts_tpd_proximity_flag	= 0;
		//state &= 0x00;	
		state = 0x00;//luowj@20140307
		tpd_proximity_flag = 0;
ps_close_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is off\n");
	}

	//ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);	    
	ret = fts_i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);	    
	TPD_PROXIMITY_DEBUG("[proxi_5206]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}

int tpd_ps_operate1(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	int ret=0;
	int state=1;
	hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_5206]command = 0x%02X\n", command);		
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{		
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				if( send_far_flag == 1 && tpd_proximity_detect==0)
				{
					//reset , flag
					hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
					msleep(200);
					hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
					msleep(5);

					mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
					mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
					mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
					msleep(20);
					TPD_DMESG(" fts ic reset 1\n");
					mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
					mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
					mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

					mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
					mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
					mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
					mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

					msleep(300);
	
					tpd_enable_ps(1);
					TPD_PROXIMITY_DEBUG("power key press to reset\n");
					//sensor_data->values[0] = 1;
					tpd_proximity_detect = 1;
					send_far_flag = 0;
				} else {
                	sensor_data->values[0] = tpd_proximity_detect;
					TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}
               
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;	
}
#endif
/*-----------------------------------yanghaichao add end ------------------------------------------*/
static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	printk("FTS yanghaichao [%4d %4d %4d] ", x, y, p);
	//printk("FTS yanghaichao %d , %d, %d, %d\n",point_num, tpd_key, cinfo->x[0], cinfo->y[0]);
	/* track id Start 0 */
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	input_mt_sync(tpd->dev);
#ifndef MT6572
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
#endif	
    {   
      tpd_button(x, y, 1);  
    }
	TPD_EM_PRINT(x, y, x, y, p-1, 1);
}
 
static  void tpd_up(int x, int y) {
	//input_report_abs(tpd->dev, ABS_PRESSURE, 0);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//printk("U[%4d %4d %4d] ", x, y, 0);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, 0, 0);
#ifndef MT6572
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
#endif
    {   
       tpd_button(x, y, 0); 
    }   		 
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
//#if (TPD_MAX_POINTS==2)
//	char data[35] = {0};
//#else
//	char data[16] = {0};
//#endif
#ifdef TPD_PROXIMITY
	int err;
	//hwm_sensor_data sensor_data;
	u8 proximity_status;
	u8 state;
#endif	
char data[128] = {0};
    u16 high_byte,low_byte,reg;

	u8 report_rate =0;

	p_point_num = point_num;
	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
	//mutex_lock(&i2c_access);


    reg = 0x00;
	//fts_i2c_Read(i2c_client, &reg, 1, data, 33);  //64  ===33
	//mutex_unlock(&i2c_access);
	//TPD_DEBUG("received raw data from touch panel as following:\n");
	TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
	TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);
	fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &data[0]);
	fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &data[8]);
	fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &data[16]);
	fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &data[24]);
/***************************************luowj@20140307*************/   
#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			//i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
			fts_i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
			//fts_i2c_Read(i2c_client, 0xB0, 1, &state,1);//  Zhuqy ADD for Psensor verify
			  
			TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);

			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}

			//i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
			//fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
			 //fts_i2c_Read(i2c_client, 0x01, 1,&proximity_status,1);//  Zhuqy ADD for Psensor verify
			   proximity_status = data[1];  
			     
			TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);
			
			if (proximity_status == 0xC0)
			{
				fts_tpd_proximity_detect	= 0;
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
				fts_tpd_proximity_detect	= 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

		}  
#endif
/************************************************************************/
	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;
	
	//TPD_DEBUG("point_num =%d\n",point_num);
		
	for(i = 0; i < point_num; i++)  
	{
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
     	cinfo->id[i] = data[3+6*i+2]>>4; //touch id
	   	/*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;

		//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
	
		/*get the Y coordinate, 2 bytes*/
		
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;

		 //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
	}
	//TPD_DEBUG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
	//TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
	//TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
		  
	return true;
};



 static int touch_event_handler(void *unused)
 { 
   	struct touch_info cinfo, pinfo;
	int i=0;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
	u8 state;
#endif
 
	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);
						 
		tpd_flag = 0;
			 
		set_current_state(TASK_RUNNING);
#if 0
#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
			//  fts_i2c_Read(i2c_client, 0xB0, 1, &state,1);//  Zhuqy ADD for Psensor verify
			  
			TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);

			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}

			i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
		//	 fts_i2c_Read(i2c_client, 0x01, 1,&proximity_status,1);//  Zhuqy ADD for Psensor verify
			     
			TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);
			
			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

		}  
#endif
#endif
		 
		if (tpd_touchinfo(&cinfo, &pinfo)) 
		{
		    //TPD_DEBUG("point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
			if(point_num >0) 
			{
			    for(i =0; i<point_num; i++)//only support 3 point
			    {
			         tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
			    }
			    input_sync(tpd->dev);
			}
			else  
    		{
			    tpd_up(cinfo.x[0], cinfo.y[0]);
        	    //TPD_DEBUG("release --->\n"); 
        	    //input_mt_sync(tpd->dev);
        	    input_sync(tpd->dev);
        		}
        	}

        	if(tpd_mode==12)
        	{
           //power down for desence debug
           //power off, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
			hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#else
			hwPowerDown(MT65XX_POWER_LDO_VGP4, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
			hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif 
    		msleep(20);
    	}
 	}while(!kthread_should_stop());
 
	return 0;
}
 
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);	
	return 0;
}
 
static void tpd_eint_interrupt_handler(void)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	//printk("#############hx8526_eint############\n");
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	count_irq ++;
	wake_up_interruptible(&waiter);
}

void focaltech_get_upgrade_array(void)
{

	u8 chip_id;
	u32 i;

	//i2c_smbus_read_i2c_block_data(i2c_client,FT_REG_CHIP_ID,1,&chip_id);
	fts_i2c_smbus_read_i2c_block_data(i2c_client,FT_REG_CHIP_ID,1,&chip_id);
//	fts_i2c_Read(i2c_client, FT_REG_CHIP_ID, 1,&chip_id,1);//  Zhuqy ADD for Psensor verify
	printk("%s chip_id = %x\n", __func__, chip_id);

		if(chip_id==0x0a)//shaohui add ,should be del
			chip_id=0x06;
	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{

		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	printk("yanghaichao tpd_probe start %d\n",__LINE__);
	int retval = TPD_OK;
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;
	u8 chip_id,i;
	u8 regaddr = 0x00;
	u8 regvalue = 0x00;
reset_proc:   
	i2c_client = client;
   
	//power on, need confirm with SA
       mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
       mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(5);
	TPD_DMESG(" fts ic reset\n");
	
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
#endif

#if 0 //def TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
       mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    
#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);

#else
       mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
       mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(5);
	TPD_DMESG(" fts ic reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
       mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
       mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
       mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
       mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	//mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, tpd_eint_interrupt_handler, 1); 
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1); 
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 
	msleep(400);
 
	//if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	if((fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
		   return -1; 
	}

	tpd_load_status = 1;
 //   tpd_mstar_status =0 ;  // compatible mstar and ft6306 chenzhecong
 
        focaltech_get_upgrade_array();

	#ifdef FTS_APK_DEBUG
	ft5x0x_create_apk_debug_channel(client);
        #endif
	#ifdef TPD_SYSFS_DEBUG
	fts_create_sysfs(i2c_client);
	#endif

	#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(i2c_client) < 0)
		TPD_DMESG(TPD_DEVICE, "%s:[FTS] create fts control iic driver failed\n",__func__);
	#endif
	
	#ifdef VELOCITY_CUSTOM_FT5206
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif

	#ifdef TPD_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(i2c_client);
	#endif
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{ 
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}

	TPD_DMESG("FTS Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef TPD_PROXIMITY
	struct hwmsen_object obj_ps;
	
	obj_ps.polling = 1;//interrupt mode
	obj_ps.sensor_operate = tpd_ps_operate1;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("proxi_fts attach fail = %d\n", err);
	}
	else
	{
		APS_ERR("proxi_fts attach ok = %d\n", err);
	}		
#endif
/*-------------------------------yanghaichao add for esd_protect 2014-02-12---------------------------------*/
#ifdef GTP_ESD_PROTECT
   	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif
/*--------------------------------yanghaichao add end ----------------------------------------------------*/
#if LCT_ADD_TP_VERSION// add by yanghaichao for tpd_info- 2014-01-21


		ft5436_g_ctp_proc = create_proc_entry(FT5436_CTP_PROC_FILE, 0444, NULL);
		if (ft5436_g_ctp_proc == NULL) {
			printk("create_proc_entry failed\n");
		} else {
			ft5436_g_ctp_proc->read_proc = ft5436_ctp_proc_read;
			ft5436_g_ctp_proc->write_proc = NULL;
			//g_ctp_proc->owner = THIS_MODULE;
			printk("create_proc_entry success\n");
		}
#endif
/*---------------------------------------yanghaihcao for deivce info 2014-01-21--------------------------------------*/
#ifdef SLT_DEVINFO_CTP
		//sprintf(temp_ver, "%s\n",CFG_VER_MIN_FLASH_buff);
		//i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &temp_ver);
		fts_read_reg(i2c_client,FT6x06_REG_VENDOR_ID,&ft5x06_vendor_id);
		regaddr = 0xA6;
		fts_read_reg(client, regaddr, &regvalue);
		sprintf(temp_ver, "current fw version:0x%02x\n", regvalue);
		//devinfo_ctp_regchar("unkown","baoming",temp_ver,DEVINFO_USED);
		switch(ft5x06_vendor_id)
		{
			case 0x5d:
			devinfo_ctp_regchar("unkown","baoming",temp_ver,DEVINFO_USED);
			break;
			case 0x51:
            devinfo_ctp_regchar("unkown","ofilm",temp_ver,DEVINFO_USED);
			break;
			case 0xa0:
			devinfo_ctp_regchar("unkown","Toptouch",temp_ver,DEVINFO_USED);
			break;
			default:
			break;
		}
#endif
/*---------------------------------------yanghaichao add end -----------------------------------------------------*/
	printk("yanghaichao tpd_probe end %d\n",__LINE__);
   return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
{

        #ifdef FTS_APK_DEBUG
	ft5x0x_release_apk_debug_channel();
	#endif
   	#ifdef TPD_SYSFS_DEBUG
	fts_release_sysfs(client);
	#endif
/*--------------------------yanghaichao add for esd_protect 2014-02-12------------------------------*/
	#ifdef GTP_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
	#endif
/*------------------------------yanghaichao add end-------------------------------------------------*/
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	
	TPD_DEBUG("TPD removed\n");
 
   	return 0;
}
 /*-------------------------yanghaichao add for esd_protect 2014-02-12------------------------------------*/
#ifdef GTP_ESD_PROTECT


static void force_reset_guitar(void)
{
    s32 i;
    s32 ret;

    TPD_DMESG("force_reset_guitar\n");

	hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
	msleep(200);
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
	msleep(5);

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(20);
	TPD_DMESG(" fts ic reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	msleep(300);
	
	#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			tpd_enable_ps(1);
			  
			//TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);
		}
	#endif
}

extern int apk_debug_flag; //0 for no apk upgrade, 1 for apk upgrade// add by zhaofei - 2014-02-22-15-12
#define A3_REG_VALUE	0x12
static u8 g_old_91_Reg_Value = 0x00;
static u8 g_first_read_91 = 0x01;
static u8 g_91value_same_count = 0;
#define RESET_91_REGVALUE_SAMECOUNT 5
static void gtp_esd_check_func(struct work_struct *work)
{
	int i;
	int ret = -1;
	//u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	u8 data, data_old;
	u8 flag_error = 0;
	int reset_flag = 0;// add by zhaofei - 2014-02-24-17-07
	u8 check_91_reg_flag = 0;

	if (tpd_halt || apk_debug_flag )// add by zhaofei - 2014-02-22-15-15
	{
		return;
	}
#if 0
	if(0x01 == run_odd_run_flag) {
		for (i = 0; i < 3; i++)
		{
			//ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xA3, 1, &data);
			ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0xA3, 1, &data);
			//   ret= fts_i2c_Read(i2c_client, 0xA3, 1,&data,1)//  Zhuqy ADD for Psensor verify
			
			
			if (ret==1 && A3_REG_VALUE==data)
			{
			    break;
			}
		}

	  if (i >= 3)
	  {
	      force_reset_guitar();
	      //msleep(300);
	      printk("focal--tpd reset. i >= 3  ret = %d	A3_Reg_Value = 0x%02x\n ", ret, data);
	      count_irq = 0;
	        data=0;
	      //i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
	      fts_i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
		    reset_flag = 1;
	  }

	//esd check for count// add by zhaofei - 2014-02-18-17-04
	  	ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x8F, 1, &data);
		printk("driver version:20140311_4	***0x8F:%d, count_irq is %d\n", data, count_irq);
				
		flag_error = 0;
		if(  (count_irq - data) > 10)
		{
			if((data+200) > (count_irq+10) )
			{
				flag_error = 1;
			}
		}
		
		if((data - count_irq ) > 10)


		{
			//if()
			{
				flag_error = 1;
			}  		
		}
			
		if(1 == flag_error)
		{		
			printk("focal--tpd reset.1 == flag_error...data=%d	count_irq\n ", data, count_irq);
		    force_reset_guitar();
	    
		//	data = 0;
	//		count_irq = 0;
			//i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
			//fts_i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
			reset_flag = 1;
		}
		
		count_irq=0;
		data=0;
		fts_i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
		run_odd_run_flag = 0x00;
	} else {
		run_odd_run_flag = 0x01;
	}
	if(1 == run_check_91_register)
		check_91_reg_flag = 1;
	else if(run_odd_run_flag == 0)
		check_91_reg_flag = 1;
	else
		check_91_reg_flag = 0;

	if(1 == check_91_reg_flag) {
		ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x91, 1, &data);
		if(0x01 == g_first_read_91) {
			g_old_91_Reg_Value = data;
			g_first_read_91 = 0x00;
		} else {
			if(g_old_91_Reg_Value == data){
				g_91value_same_count++;
				if(RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count) {
					force_reset_guitar();
					printk("focal--tpd reset. g_91value_same_count = 5\n");
				}
				run_check_91_register = 1;
				g_old_91_Reg_Value = data;
			} else {
				g_old_91_Reg_Value = data;
				g_91value_same_count = 0;
				run_check_91_register = 0;
			}
		}
	}
	#else
	run_check_91_register = 0;
	for (i = 0; i < 3; i++)
	{
		//ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xA3, 1, &data);
		ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0xA3, 1, &data);
		//   ret= fts_i2c_Read(i2c_client, 0xA3, 1,&data,1)//  Zhuqy ADD for Psensor verify
		
		
		if (ret==1 && A3_REG_VALUE==data)
		{
		    break;
		}
	}

  if (i >= 3)
  {
      force_reset_guitar();
      //msleep(300);
      printk("focal--tpd reset. i >= 3  ret = %d	A3_Reg_Value = 0x%02x\n ", ret, data);
	    reset_flag = 1;
	goto FOCAL_RESET_A3_REGISTER;
  }

//esd check for count// add by zhaofei - 2014-02-18-17-04
  	ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x8F, 1, &data);
	printk("driver version:20140312_2	***0x8F:%d, count_irq is %d\n", data, count_irq);
			
	flag_error = 0;
	if(  (count_irq - data) > 10)
	{
		if((data+200) > (count_irq+10) )
		{
			flag_error = 1;
		}
	}
	
	if((data - count_irq ) > 10)
	{
		//if()
		{
			flag_error = 1;
		}  		
	}
		
	if(1 == flag_error)
	{		
		printk("focal--tpd reset.1 == flag_error...data=%d	count_irq\n ", data, count_irq);
	    force_reset_guitar();
    
		
		reset_flag = 1;
		goto FOCAL_RESET_INT;
	}

	run_check_91_register = 1;
	ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x91, 1, &data);
	printk("focal---------91 register value = 0x%02x	old value = 0x%02x\n",
		data, g_old_91_Reg_Value);
	if(0x01 == g_first_read_91) {
		g_old_91_Reg_Value = data;
		g_first_read_91 = 0x00;
	} else {
		if(g_old_91_Reg_Value == data){
			g_91value_same_count++;
			if(RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count) {
				force_reset_guitar();
				printk("focal--tpd reset. g_91value_same_count = 5\n");
				reset_flag = 1;
			}
			printk("focal 91 value ==============\n");
			//run_check_91_register = 1;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE / 2;
			g_old_91_Reg_Value = data;
		} else {
			g_old_91_Reg_Value = data;
			g_91value_same_count = 0;
			//run_check_91_register = 0;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE;
		}
	}
FOCAL_RESET_INT:
FOCAL_RESET_A3_REGISTER:
	count_irq=0;
	data=0;
	fts_i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
	if(0 == run_check_91_register)
	g_91value_same_count = 0;
	#endif


	
	if( (1 == reset_flag) && ( 1 == tpd_proximity_flag) )
	{
		if((tpd_enable_ps(1) != 0))
		{
			APS_ERR("enable ps fail\n"); 
				return -1;
		}
	}
	//end esd check for count// add by zhaofei - 2014-02-18-17-04

    if (!tpd_halt)
    {
        //queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
    }

    return;
}
#endif
/*--------------------------------------------yanghaichao add end ----------------------------------------------------*/
static int tpd_local_init(void)
{
  	TPD_DMESG("FTS I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 	printk("yanghaichao tpd_local_init start %d\n",__LINE__);
   	if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("FTS unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("FTS add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
	//if(TPD_RES_Y > 854)
	{
	    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
	}
	//else
	{
	    //tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local_fwvga);// initialize tpd button data
	}
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
    TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
    tpd_type_cap = 1;
    return 0; 
 }

 static void tpd_resume( struct early_suspend *h )
 {
  //int retval = TPD_OK;
  //char data;
#ifdef TPD_PROXIMITY	
	if (tpd_proximity_flag == 1)
	{
		if(tpd_proximity_flag_one == 1)
		{
			tpd_proximity_flag_one = 0;	
			TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
			return;
		}
	}
#endif	
 
   	TPD_DMESG("TPD wake up\n");
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
	msleep(5);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");

#else

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(10);  
   // mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   // mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
	
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	msleep(30);
	tpd_halt = 0;
	/* for resume debug
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
	}
	*/
/*-----------------------------yanghaichao add for esd_protect 2014-02-12------------------------------------*/
#ifdef GTP_ESD_PROTECT
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
	#endif
/*---------------------------------yanghaichao add end-------------------------------------------------------*/
	tpd_up(0,0);
	input_sync(tpd->dev);
	TPD_DMESG("TPD wake up done\n");
	 //return retval;
 }

 static void tpd_suspend( struct early_suspend *h )
 {
	// unsigned char buf[32] = {};
	// int retval = TPD_OK;
	 static char data = 0x3;

#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_proximity_flag_one = 1;	
		return;
	}
#endif
/*---------------------------------yanghaichao add for esd_protect 2014-02-12---------------------------------*/
#ifdef GTP_ESD_PROTECT
    cancel_delayed_work_sync(&gtp_esd_check_work);
#endif
/*-------------------------------------yanghaichao add end----------------------------------------------------*/
 	 tpd_halt = 1;

	 TPD_DMESG("TPD enter sleep\n");
	 mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
	//mutex_lock(&i2c_access);
	//i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode

	//buf[0] = 0xA5;
	//buf[1] = 0x03;	
	// fts_i2c_Write(i2c_client, buf, 2);
	//mutex_unlock(&i2c_access);
	fts_i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
#endif
	hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
	TPD_DMESG("TPD enter sleep done\n");
	//return retval;
	printk("yanghaichao tpd_local_init end %d\n",__LINE__);
 } 


 static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TPD_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("MediaTek FTS touch panel driver init\n");
	printk("yanghaichao add driver_init start\n");
	i2c_register_board_info(IIC_PORT, &ft5206_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FTS driver failed\n");
	printk("yanghaichao add driver_init end\n");
	 return 0;
 }
 
 /* should never be called */
static void __exit tpd_driver_exit(void) {
	TPD_DMESG("MediaTek FTS touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}
 
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


