#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 

/*ergate-013*/
#include <linux/dma-mapping.h>

#include "ft6206tomsg2133.h"
#include "tpd.h"

/*ergate-001*/
#include "cust_gpio_usage.h"
#include "cust_eint.h"

/*ergate-037*/
#include <linux/types.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

//zym 2011-12-26 for lcm_resume_in_lcd
#if defined(HQ_PROJECT_A16)
#include "lcm_drv.h"
#endif

#undef USE_RTP_RST

#ifdef FT6206_UPDATE_DMA_WAY //for MT6572
static unsigned char *I2CDMABuf_va = NULL;
static volatile unsigned int I2CDMABuf_pa = NULL;
#endif


#if 0
/*ergate-016 start*/
#if defined(HQ_PROJECT_A20)
#undef GPIO_CAPTOUCH_EINT_PIN
#undef GPIO_CAPTOUCH_EINT_PIN_M_EINT
#undef CUST_EINT_TOUCH_PANEL_NUM
#undef GPIO_CTP_EN_PIN

static int GPIO_CAPTOUCH_EINT_PIN,GPIO_CAPTOUCH_EINT_PIN_M_EINT,CUST_EINT_TOUCH_PANEL_NUM;
static int GPIO_CTP_EN_PIN;
#else // add for a16

#undef GPIO_CAPTOUCH_EINT_PIN
#undef GPIO_CAPTOUCH_EINT_PIN_M_EINT
#undef CUST_EINT_TOUCH_PANEL_NUM
#undef GPIO_CTP_EN_PIN
static int GPIO_CAPTOUCH_EINT_PIN,GPIO_CAPTOUCH_EINT_PIN_M_EINT,CUST_EINT_TOUCH_PANEL_NUM;
static int GPIO_CTP_EN_PIN;
#endif
/*ergate-016 end*/
#endif
//wangfuqiang add
 #define TPD_POWER_SOURCE_CUSTOM	MT6323_POWER_LDO_VGP1
extern struct tpd_device *tpd;
 
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
static struct early_suspend early_suspend;
 
#ifdef CONFIG_HAS_EARLYSUSPEND
static void tpd_early_suspend(struct early_suspend *handler);
static void tpd_late_resume(struct early_suspend *handler);
#endif 
 

extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
//extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
//extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
/*extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
*/
 extern void mt65xx_eint_set_hw_debounce(unsigned int eintno, unsigned int ms);
 extern kal_uint32 mt65xx_eint_set_sens(unsigned int eintno, unsigned int sens);
 extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en,
			  unsigned int pol, void (EINT_FUNC_PTR) (void),
			  unsigned int is_auto_umask);
static void tpd_eint_interrupt_handler(void);
static int tpd_get_bl_info(int show);
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static int tpd_initialize(struct i2c_client * client);

extern int tpd_firmware_version[2];

#define HQ_PROJECT_A16

#if defined(HQ_PROJECT_A16)
static int pre_count =0;
static int pre_x1 =0;
static int pre_y1 =0;
static int pre_x2 =0;
static int pre_y2 =0;
#endif

#define TPD_OK 0

#define TPD_PACKET_HEAD_CMD    0x54
#define TPD_PACKET_HEAD_RSP    0x52

#define TPD_PACKET_CMD_VER    0x10

#define TPD_PACKET_CMD_MODE_READ    0x20
#define TPD_PACKET_CMD_MODE_WRITE   0x21
#define TPD_MODE_ACTIVE             0x10
#define TPD_MODE_FAST               0x20
#define TPD_MODE_FREEZE             0x90

#define TPD_PACKET_CMD_FORMAT_READ    0x60
#define TPD_PACKET_CMD_FORMAT_WRITE   0x61
#define TPD_PACKET_FORMAT_1           0x00
#define TPD_PACKET_FORMAT_2           0x01
#define TPD_PACKET_FORMAT_3           0x03

/*ergate-012 start*/
#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
/*ergate-012 end*/

struct TpdPacketT
{
    U8 head;
    U8 command;
    U8 data_1;
    U8 data_2;
};

struct TpdTouchDataT
{
	U8 packet_id;
	U8 x_h_y_h;
	U8 x_l;
	U8 y_l;
	U8 disx_h_disy_h;
	U8 disx_l;
	U8 disy_l;
    U8 checksum;
};

struct TouchInfoT
{
	int x1, y1;
	int x2, y2;
	int pressure;
	int count;
#ifdef TPD_HAVE_BUTTON 
    int key_id;
    int key_value;
#endif
};


static int tpd_flag = 0;

static struct TpdTouchDataT TpdTouchData;

#define FT6206_DBG    printk
//#define FT6206_DBG 
        
static const struct i2c_device_id tpd_id[] = {{"ft6206msg2133",0},{}};
/*ergate-001*/
static unsigned short force[] = {0,0x4c,I2C_CLIENT_END,I2C_CLIENT_END};  //0x4C  add by tianhaiyang
static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
//static struct i2c_board_info __initdata i2c_tpd
static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("ft6206msg2133", 0x38)};  //0x26 add by tianhaiyang
static struct i2c_driver tpd_driver = 
{
	.driver = 
	{
		.name = "ft6206msg2133",
		//.owner = THIS_MODULE,
	},
	.probe = tpd_probe,
	.remove = __devexit_p(tpd_remove),
	.id_table = tpd_id,
	.detect = tpd_detect,
	.driver.name = "ft6206msg2133",
	//.address_data = &addr_data,
	//.address_list = (const unsigned short *) forces,
};
static void ft6206_reset();

static void tpd_hw_reset(void);
#ifdef FT6206_UPDATE
struct class *firmware_class6206;
struct  device *firmware_cmd_dev6206;
static int update_switch = 0;
static int FwDataCnt;
static struct fw_version fw_v;
static unsigned char temp[24 * 1024];

struct Upgrade_Info {
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	unsigned char upgrade_id_1;	/*upgrade id 1 */
	unsigned char upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_earse_flash; /*delay of earse flash*/
};

static int ft6206_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
    //FT6206_DBG("The msg_i2c_client->addr=0x%x\n",i2c_client->addr);
    ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);

    if(ret <= 0)
        FT6206_DBG("msg_i2c_read_interface error\n");
    return ret;
}

static int ft6206_i2c_write(char *pbt_buf, int dw_lenth)
{
    int ret;
    //FT6206_DBG("The msg_i2c_client->addr=0x%x\n",i2c_client->addr);
    ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);

    if(ret <= 0)
        FT6206_DBG("msg_i2c_read_interface error\n");

    return ret;
}




int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret, i;
#ifdef FT6206_UPDATE_DMA_WAY
	if(writelen!=0)
	{
		//DMA Write
		if(writelen < 8  )
		{
			client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);	
			//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
			ret= i2c_master_send(client, writebuf, writelen);
		}
		else
		{
			for(i = 0 ; i < writelen; i++)
			{
				I2CDMABuf_va[i] = writebuf[i];
			}
			client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
			if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
				dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
			//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
			//return ret;
		}
	}
	//DMA Read 
	if(readlen!=0)
	{
		if (readlen <8) {
			client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);	
			ret = i2c_master_recv(client, (unsigned char *)readbuf, readlen);
		}
		else
		{
			client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
			
			ret = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, readlen);

			for(i = 0; i < readlen; i++)
	        {
	            readbuf[i] = I2CDMABuf_va[i];
	        }
		}
	}
	
#else

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
#endif
	return ret;
}
/*write data by i2c*/
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret, i;
#ifdef FT6206_UPDATE_DMA_WAY
	if(writelen < 8){
		client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);	
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		ret = i2c_master_send(client, writebuf, writelen);
	}else{
		for(i = 0 ; i < writelen; i++){
			I2CDMABuf_va[i] = writebuf[i];
		}
		client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;	
		if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
			dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
	} 
#else
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
#endif
	return ret;
}


int ft5x0x_write_reg(struct i2c_client *client, unsigned char regaddr, unsigned char regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft5x0x_i2c_Write(client, buf, sizeof(buf));
}


int ft5x0x_read_reg(struct i2c_client *client, unsigned char regaddr, unsigned char *regvalue)
{
	return ft5x0x_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

int fts_ctpm_fw_read_app(struct i2c_client *client, unsigned char *pbt_buf,
			  u32 dw_lenth)
{
	u32 packet_number;
	u32 j = 0;
	u32 temp;
	u32 lenght = 0;
	unsigned char *pReadBuf = NULL;
	unsigned char auc_i2c_write_buf[10];
	int i_ret;

	dw_lenth = dw_lenth - 2;

	pReadBuf = kmalloc(dw_lenth + 1, GFP_ATOMIC);
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;

	/*Read flash*/
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		auc_i2c_write_buf[2] = (unsigned char) (temp >> 8);
		auc_i2c_write_buf[3] = (unsigned char) temp;
		
		i_ret = ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, 
			pReadBuf+lenght, FTS_PACKET_LENGTH);
		if (i_ret < 0)
			return -EIO;
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		lenght += FTS_PACKET_LENGTH;
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		auc_i2c_write_buf[2] = (unsigned char) (temp >> 8);
		auc_i2c_write_buf[3] = (unsigned char) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;

		i_ret = ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, 
			pReadBuf+lenght, temp);
		if (i_ret < 0)
			return -EIO;
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		lenght += temp;
	}

	/*read the last six byte */
	temp = 0x6ffa + j;
	auc_i2c_write_buf[2] = (unsigned char) (temp >> 8);
	auc_i2c_write_buf[3] = (unsigned char) temp;
	temp = 6;
	i_ret = ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, 
		pReadBuf+lenght, temp);
	if (i_ret < 0)
		return -EIO;
	msleep(FTS_PACKET_LENGTH / 6 + 1);
	lenght += temp;


	/*read app from flash and compart*/
	for (j=0; j<dw_lenth-2; j++) {
		if(pReadBuf[j] != pbt_buf[j]) {
			kfree(pReadBuf);
			return -EIO;
		}
	}

	kfree(pReadBuf);
	return 0;
}
/*
*get upgrade information depend on the ic type
*/
static void fts_get_upgrade_info(struct Upgrade_Info *upgrade_info)
{
	switch (DEVICE_IC_TYPE) {
	case IC_FT5X06:
		upgrade_info->delay_55 = FT5X06_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X06_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X06_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X06_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X06_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT5X06_UPGRADE_EARSE_DELAY;
		break;
	case IC_FT5606:
		upgrade_info->delay_55 = FT5606_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5606_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5606_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5606_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5606_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT5606_UPGRADE_EARSE_DELAY;
		break;
	case IC_FT5316:
		upgrade_info->delay_55 = FT5316_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5316_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5316_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5316_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5316_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT5316_UPGRADE_EARSE_DELAY;
		break;
	case IC_FT6206:
		upgrade_info->delay_55 = FT6206_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT6206_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT6206_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT6206_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT6206_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT6206_UPGRADE_EARSE_DELAY;
		break;
	default:
		break;
	}
}


int fts_ctpm_fw_upgrade_for_funa(struct i2c_client *client, unsigned char *pbt_buf,
			  unsigned int dw_lenth)
{
		unsigned int old_timing;
	unsigned char reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j = 0;
	u32 temp;
	u32 lenght;
	unsigned char packet_buf[FTS_PACKET_LENGTH + 6];
	unsigned char auc_i2c_write_buf[10];
	unsigned char bt_ecc;
	int i_ret;
	struct Upgrade_Info upgradeinfo;
	int upgrade_count;
	
	old_timing = client->timing;
	client->timing = 100;
	
	printk("old_timing = %d\n",old_timing);

	fts_get_upgrade_info(&upgradeinfo);
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
	#if 0	
		if (DEVICE_IC_TYPE == IC_FT6206)
			ft5x0x_write_reg(client, 0xbc, FT_UPGRADE_AA);
		else
			ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(upgradeinfo.delay_aa);

		/*write 0x55 to register 0xfc */
		if (DEVICE_IC_TYPE == IC_FT6206)
			ft5x0x_write_reg(client, 0xbc, FT_UPGRADE_55);
		else
			ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);
	#endif
                client->addr = 0x38 | I2C_ENEXT_FLAG;
    msleep(5);
		mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
		msleep(10);
		mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
                //mdelay(5);

    if(i < 15)
		{
			msleep(30+i*2);
		}
		else
		{
			msleep(30-(i-15)*2);
		}


	        //mdelay(30);
		//msleep(upgradeinfo.delay_55);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		j = 0;
		do { 
			j++;
			i_ret = ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && j < 5);



/* --   I2C slave address 0x38 begin   --  */
		/*********Step 3:check READ-ID***********************/

		msleep(upgradeinfo.delay_readid);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		i_ret =ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
                 printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);


		if (reg_val[0] == upgradeinfo.upgrade_id_1
			&& reg_val[1] == upgradeinfo.upgrade_id_2) {
			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				//reg_val[0], reg_val[1]);
			FT6206_DBG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
                        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
		
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0xcd;

	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);


	/*Step 4:erase app and panel paramenter area*/
	FT6206_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(upgradeinfo.delay_earse_flash);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = 0x63;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(100);
	
	///////////////////////////////
	//erase app 2th time
	auc_i2c_write_buf[0] = 0x61;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(upgradeinfo.delay_earse_flash);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = 0x63;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(100);
	/////////////////////////////
	//erase app 3th time
	auc_i2c_write_buf[0] = 0x61;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(upgradeinfo.delay_earse_flash);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = 0x63;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(100);
	/////////////////////////////////////////////

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (unsigned char) (temp >> 8);
		packet_buf[3] = (unsigned char) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (unsigned char) (lenght >> 8);
		packet_buf[5] = (unsigned char) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft5x0x_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		//DBG("write bytes:0x%04x\n", (j+1) * FTS_PACKET_LENGTH);
		//delay_qt_ms(FTS_PACKET_LENGTH / 6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (unsigned char) (temp >> 8);
		packet_buf[3] = (unsigned char) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (unsigned char) (temp >> 8);
		packet_buf[5] = (unsigned char) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft5x0x_i2c_Write(client, packet_buf, temp + 6);
		msleep(20);
	}


	/*send the last six byte */
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (unsigned char) (temp >> 8);
		packet_buf[3] = (unsigned char) temp;
		temp = 1;
		packet_buf[4] = (unsigned char) (temp >> 8);
		packet_buf[5] = (unsigned char)  temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		ft5x0x_i2c_Write(client, packet_buf, 7);
		msleep(20);
	}


	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	printk("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

#if 0
	/*read app from flash and compare*/
	FT6206_DBG("Read flash and compare\n");
	if(fts_ctpm_fw_read_app(client, pbt_buf, dw_lenth+6) < 0) {
		dev_err(&client->dev, "[FTS]--app from flash is not equal to app.bin\n");
		return -EIO;
	}
#endif
		
	/*********Step 7: reset the new FW***********************/
	printk("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	
	 client->addr = 0x38;
        mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
        msleep(10);
        mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
        msleep(20);

     client->timing =  old_timing;

/* --   I2C slave address 0x38 end   --  */	

	return 1;
}


static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_version_show\n");
	FT6206_DBG("*** firmware_version_show fw_version = %03d.%03d***\n", fw_v.major, fw_v.minor);
	return sprintf(buf, "%03d%03d\n", fw_v.major, fw_v.minor);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4] ;
	int time = 0;
	
	printk("%s\n", __func__);
	while(time < 10){
		dbbus_tx_data[0] = 0x53;
		dbbus_tx_data[1] = 0x00;
		dbbus_tx_data[2] = 0x74;
		ft6206_i2c_write(&dbbus_tx_data[0], 3);
		//mdelay(50);
		ft6206_i2c_read(&dbbus_rx_data[0], 4);
		fw_v.major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
		fw_v.minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
		
		time++;
		if((fw_v.major & 0xff00) == 0)
			break;
		msleep(50);
	}
	
	return size;
}
static int ver_ft6206_get_version(struct fw_version *fw)
{
	int ret = 0;
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4];
	
	FT6206_DBG("%s\n", __func__);

	FT6206_DBG("\n");

	dbbus_tx_data[0] = 0x53;
	dbbus_tx_data[1] = 0x00;
	dbbus_tx_data[2] = 0x74;
	ret = ft6206_i2c_write(&dbbus_tx_data[0], 3);
	if(ret < 0)
		return ret;
	//mdelay(50);
	ret = ft6206_i2c_read(&dbbus_rx_data[0], 4);
	if(ret < 0)
		return ret;
	fw->major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
	fw->minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
	FT6206_DBG("%s, major = 0x%x, minor = 0x%x\n", __func__, fw->major, fw->minor);
	return 0;
}

static ssize_t firmware_update_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_update_show\n");
	return sprintf(buf, "%03d%03d\n", fw_v.major, fw_v.minor);
}

static ssize_t firmware_update_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char i;
	unsigned char dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	update_switch = 1;
	//drvISP_EntryIspMode();
	//drvISP_BlockErase(0x00000);
	//M by cheehwa _HalTscrHWReset();

	//disable_irq_nosync(i2c_client->irq);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	/*mt65xx_eint_mask cann't stop EINT, so doing following*/
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, 			NULL, 1);


#ifdef FT6206_UPDATE_DMA_WAY//for MT6572
	I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, FTS_DMA_BUF_SIZE, &I2CDMABuf_pa, GFP_KERNEL);

	if(!I2CDMABuf_va){
	    printk("FTP: %s Allocate DMA I2C Buffer failed!\n", __func__);
	    return -EIO;
	}
	printk("FTP: %s I2CDMABuf_pa=%x,val=%x val2=%x\n", __func__, &I2CDMABuf_pa,I2CDMABuf_pa,(unsigned char *)I2CDMABuf_pa);
#endif

	fts_ctpm_fw_upgrade_for_funa(i2c_client, temp, FwDataCnt);
	
#ifdef FT6206_UPDATE_DMA_WAY//for MT6572
    if(I2CDMABuf_va){
        dma_free_coherent(NULL, FTS_DMA_BUF_SIZE, I2CDMABuf_va, I2CDMABuf_pa);
        I2CDMABuf_va = NULL;
        I2CDMABuf_pa = 0;
    }
#endif

    	FwDataCnt = 0;
    	tpd_hw_reset();
    	FT6206_DBG("update OK\n");
    	update_switch = 0;
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler,1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
    	//enable_irq(i2c_client->irq);
    	return size;
}

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_data_show\n");
    	return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    	int i;
	printk("tyd-tp: firmware_data_store\n");
    	for(i = 0; i < size; i++){
        	temp[FwDataCnt] = buf[i];
			FwDataCnt++;
    	}
    	FT6206_DBG("***FwDataCnt = %d ***\n", FwDataCnt);

    	
    	return size;
}


static DEVICE_ATTR(version, 0777, firmware_version_show, firmware_version_store);
static DEVICE_ATTR(update, 0777, firmware_update_show, firmware_update_store);
static DEVICE_ATTR(data, 0777, firmware_data_show, firmware_data_store);

void ft6206_init_fw_class()
{
	FwDataCnt = 0;

	firmware_class6206 = class_create(THIS_MODULE,"mtk-tpd" );//client->name

	if(IS_ERR(firmware_class6206))
		pr_err("Failed to create class(firmware)!\n");

	firmware_cmd_dev6206 = device_create(firmware_class6206,
	                                     NULL, 0, NULL, "device");//device

	if(IS_ERR(firmware_cmd_dev6206))
		pr_err("Failed to create device(firmware_cmd_dev)!\n");
		
	// version /sys/class/mtk-tpd/device/version
	if(device_create_file(firmware_cmd_dev6206, &dev_attr_version) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);

	// update /sys/class/mtk-tpd/device/update
	if(device_create_file(firmware_cmd_dev6206, &dev_attr_update) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);

	// data /sys/class/mtk-tpd/device/data
	if(device_create_file(firmware_cmd_dev6206, &dev_attr_data) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

}

#define CTP_AUTHORITY_PROC 0777 
static  char fw_version_proc[16];
static struct proc_dir_entry *msg_version_proc = NULL;
static struct proc_dir_entry *msg_update_proc = NULL;
static struct proc_dir_entry *msg_data_proc = NULL;
#define PROC_VERSION "version"
#define PROC_UPDATE  "update"
#define PROC_DATA    "data"
//#define PROC_CL  "class"

static int proc_version_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt= 0;
    
    printk("Enter proc_version_read.\n");
    
    cnt = sprintf(page, "%s\n", fw_version_proc);
    printk("%s\n", page);  
    
    *eof = 1;
    return cnt;
}

static int proc_version_write(struct file *file, const char *buffer, unsigned long count, void *data)
{

	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4] ;
	unsigned short major = 0, minor = 0;
	int time = 0;
	memset(fw_version_proc, 0,  sizeof(fw_version_proc));
	while(time < 10){
		//Get_Chip_Version();
		dbbus_tx_data[0] = 0x53;
		dbbus_tx_data[1] = 0x00;
		dbbus_tx_data[2] = 0x74;
		i2c_master_send(i2c_client, &dbbus_tx_data[0], 3);
		msleep(20);
		i2c_master_recv(i2c_client, &dbbus_rx_data[0], 4);

		//ft5x0x_i2c_Read(i2c_client, dbbus_tx_data, 3, dbbus_rx_data, 4);

		printk("%s, dbbus_rx_data = [0x%x] [0x%x] [0x%x] [0x%x]", __func__, dbbus_rx_data[0], dbbus_rx_data[1],
			dbbus_rx_data[2], dbbus_rx_data[3]);
		major = dbbus_rx_data[0];
		minor = dbbus_rx_data[2];

		if(major == 1)
			break;

		time++;
		msleep(50);
	}
	
	sprintf(fw_version_proc, "%03d%03d", major, minor);

	return count;
}

static int proc_update_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    printk("proc_update_write count %ld\n", count);

    /// start update procedure in firmware_update_store(), just call 
    count = (unsigned long)firmware_update_store(NULL, NULL, NULL, (size_t)count);	

    return count;
}

static int proc_data_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt= 0;
    
    printk("Enter proc_data_read.\n");
    
    cnt = sprintf(page, "%s\n", FwDataCnt);   
    printk("%s\n", page); 
    
    *eof = 1;    
    return cnt;
}

static int proc_data_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    printk("proc_data_write count %ld\n", count);

    if (copy_from_user(&temp[FwDataCnt], buffer, 1024/*sizeof(buffer)*/))
    {
        printk("copy from user failed\n");
        return -EFAULT;
    }
    FwDataCnt+=1024;
    printk("***FwDataCnt = %d ***\n", FwDataCnt); 

    return count;
}

static void proc_update_firmeare_init()
{

struct proc_dir_entry *parent_class;
struct proc_dir_entry *parent_msg;
struct proc_dir_entry *parent_dev;

parent_class = proc_mkdir("class",NULL);
parent_msg =  proc_mkdir("ms-touchscreen-msg20xx",parent_class);
parent_dev =  proc_mkdir("device",parent_msg);
       
	msg_version_proc = create_proc_entry(PROC_VERSION, CTP_AUTHORITY_PROC, parent_dev);
	if (msg_version_proc == NULL) 
	{
		printk("create_proc_entry msg_version_proc failed\n");
	} 
	else 
	{
		msg_version_proc->read_proc = proc_version_read;
		msg_version_proc->write_proc = proc_version_write;
		///msg_version_proc->owner = THIS_MODULE;
	printk("create_proc_entry msg_version_proc success\n");
	}
	
	msg_data_proc = create_proc_entry(PROC_DATA, CTP_AUTHORITY_PROC, parent_dev);
	if (msg_data_proc == NULL) 
	{
		printk("create_proc_entry msg_data_proc failed\n");
	} 
	else 
	{
		msg_data_proc->read_proc = proc_data_read;
		msg_data_proc->write_proc = proc_data_write;
		///msg_data_proc->owner = THIS_MODULE;
		printk("create_proc_entry msg_data_proc success\n");
	}
	
	msg_update_proc = create_proc_entry(PROC_UPDATE, CTP_AUTHORITY_PROC, parent_dev);
	if (msg_update_proc == NULL) 
	{
		printk("create_proc_entry msg_update_proc failed\n");
	} 
	else 
	{
		msg_update_proc->read_proc = NULL;
		msg_update_proc->write_proc = proc_update_write;
		///msg_update_proc->owner = THIS_MODULE;
		printk("create_proc_entry msg_update_proc success\n");
	}    

}
#endif //endif FT6206_UPDATE

//#define FT6206_AUTO_UPDATE
#ifdef FT6206_AUTO_UPDATE
enum wk_wdt_type {
	WK_WDT_LOC_TYPE,
	WK_WDT_EXT_TYPE,
	WK_WDT_LOC_TYPE_NOLOCK,
	WK_WDT_EXT_TYPE_NOLOCK,
};
extern mtk_wdt_restart(enum wk_wdt_type type);
////static ssize_t FT6206_firmware_AUTO_update_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static void FT6206_firmware_AUTO_update_store(unsigned long data);
static unsigned char auto_temp[24*1024] = {
#include "Y325-funa-WT12_C118.i"
};
#endif

#ifdef FT6206_AUTO_UPDATE
#define AUTO_UPDATER_DELAY	10
#define FT6206_FUNA_MAJOR			1
#define FT6206_FUNA_MINOR			18

static void ft6206_auto_update_fw()
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4] ;
	unsigned short major, minor;
	int time = 0;
#if 0
#ifdef FT6206_UPDATE_DMA_WAY//for MT6572
	FT6206_DBG("wingtech enter 2\n ");

	I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, FTS_DMA_BUF_SIZE, &I2CDMABuf_pa, GFP_KERNEL);

	if(!I2CDMABuf_va){
	FT6206_DBG("wingtech enter DMA faild\n ");
	FT6206_DBG("FTP: %s Allocate DMA I2C Buffer failed!\n", __func__);
	return -EIO;
	}
	FT6206_DBG("wingtech enter DMA successed\n ");
	FT6206_DBG("FTP: %s I2CDMABuf_pa=%x,val=%x val2=%x\n", __func__, &I2CDMABuf_pa,I2CDMABuf_pa,(unsigned char *)I2CDMABuf_pa);
#endif
#endif

	//ft5x0x_i2c_Read(i2c_client, &dbbus_tx_start, 1, (u8 *)&TpdTouchData, sizeof(TpdTouchData));
    //i2c_master_send(i2c_client, &dbbus_tx_start, 1);
    //retval = i2c_master_recv_ext(i2c_client, (u8 *)&TpdTouchData, sizeof(TpdTouchData));

	printk("%s\n", __func__);
	while(time < 5){
		dbbus_tx_data[0] = 0x53;
		dbbus_tx_data[1] = 0x00;
		dbbus_tx_data[2] = 0x74;
		ft6206_i2c_write(&dbbus_tx_data[0], 3);
		mdelay(20);
		ft6206_i2c_read(&dbbus_rx_data[0], 4);
		//dma
		//ft5x0x_i2c_Read(i2c_client,&dbbus_tx_data[0], 3,&dbbus_rx_data[0], 4);
		major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
		minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
		
		time++;
		if((major & 0xff00) == 0)
			break;
		msleep(10);
	}
	#if 0
	#ifdef FT6206_UPDATE_DMA_WAY//for MT6572
	if(I2CDMABuf_va){
		dma_free_coherent(NULL, FTS_DMA_BUF_SIZE, I2CDMABuf_va, I2CDMABuf_pa);
		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;
	}
#endif
#endif

	printk("%s, major = %d, minor = %d\n", __func__, major, minor);

	if(major == FT6206_FUNA_MAJOR){
		if(minor != FT6206_FUNA_MINOR){
			printk("%s, auto upgrade firmware start.....\n", __func__);
			//fts_ctpm_fw_upgrade_for_funa(i2c_client, auto_temp, sizeof(auto_temp));
			FT6206_firmware_AUTO_update_store(0);
			ft6206_reset();
			printk("%s, auto upgrade firmware end.\n", __func__);
		}
	}

}
static DEFINE_TIMER(ft6206_auto_update_delay, FT6206_firmware_AUTO_update_store, 0, 0);

static void FT6206_firmware_AUTO_update_store(unsigned long data)
{
		FT6206_DBG("FT6206ToMsg2133 firmware_AUTO_update begin\n ");
		volatile unsigned char i;
	////	int j;
		unsigned char dbbus_tx_data[4];
		unsigned char dbbus_rx_data[2] = {0};
		update_switch= 1;
		//drvISP_EntryIspMode();
		//drvISP_BlockErase(0x00000);
		//M by cheehwa _HalTscrHWReset();
	
		//disable_irq_nosync(this_client->irq);
#if 1
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		/*mt65xx_eint_mask cann't stop EINT, so doing following*/
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY,			NULL, 1);
#endif
#if 1
#ifdef FT6206_UPDATE_DMA_WAY//for MT6572
		FT6206_DBG("wingtech enter 2\n ");

		I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, FTS_DMA_BUF_SIZE, &I2CDMABuf_pa, GFP_KERNEL);
	
		if(!I2CDMABuf_va){
			FT6206_DBG("wingtech enter DMA faild\n ");
			FT6206_DBG("FTP: %s Allocate DMA I2C Buffer failed!\n", __func__);
			return -EIO;
		}
		FT6206_DBG("wingtech enter DMA successed\n ");
		FT6206_DBG("FTP: %s I2CDMABuf_pa=%x,val=%x val2=%x\n", __func__, &I2CDMABuf_pa,I2CDMABuf_pa,(unsigned char *)I2CDMABuf_pa);
#endif
#endif	
		//for(i=0; i < 23*1024; i++)
		//		temp[i] = auto_temp[i];

		FwDataCnt = 23*1024;
		//firmware_update_store(NULL, NULL, NULL, 0);
		FT6206_DBG("wingtech enter 3\n ");

		fts_ctpm_fw_upgrade_for_funa(i2c_client, auto_temp, FwDataCnt);
		FT6206_DBG("wingtech enter 8\n ");

		///FwDataCnt = 0;
#if 1
#ifdef FT6206_UPDATE_DMA_WAY//for MT6572
		if(I2CDMABuf_va){
			dma_free_coherent(NULL, FTS_DMA_BUF_SIZE, I2CDMABuf_va, I2CDMABuf_pa);
			I2CDMABuf_va = NULL;
			I2CDMABuf_pa = 0;
		}
#endif
#endif
			FwDataCnt = 0;
			tpd_hw_reset();
			FT6206_DBG("update OK\n");
			update_switch= 0;
#if 1
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler,1);
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);	
#endif 
			//enable_irq(i2c_client->irq);
	}

//#define FT6206_FORCE_UPDATE

#ifdef FT6206_FORCE_UPDATE

#define MSG2133_FW_ADDR			0x62
#define MSG2133_TS_ADDR			0x26


static void i2c_read_msg2133(unsigned char *pbt_buf, int dw_lenth)
{
	i2c_client->addr = MSG2133_FW_ADDR;
	i2c_master_recv(i2c_client, pbt_buf, dw_lenth);	//0xC5_8bit
	i2c_client->addr = MSG2133_TS_ADDR;
}

static void i2c_write_msg2133(unsigned char *pbt_buf, int dw_lenth)
{

	i2c_client->addr = MSG2133_FW_ADDR;
	i2c_master_send(i2c_client, pbt_buf, dw_lenth);		//0xC4_8bit
	i2c_client->addr = MSG2133_TS_ADDR;
}

void dbbusDWIICEnterSerialDebugMode(void)
{
    unsigned char data[5];
    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    i2c_write_msg2133(data, 5);
}

void dbbusDWIICStopMCU(void)
{
    unsigned char data[1];
    // Stop the MCU
    data[0] = 0x37;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICUseBus(void)
{
    unsigned char data[1];
    // IIC Use Bus
    data[0] = 0x35;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICReshape(void)
{
    unsigned char data[1];
    // IIC Re-shape
    data[0] = 0x71;
    i2c_write_msg2133(data, 1);
}

static int ft6206_firmware_force_update()
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4] ;
	unsigned short major, minor;
	int time = 0, i;
	int force_upgrage = 0;

	i2c_client->addr = 0x38;
	
	printk("%s\n", __func__);

	ft6206_reset();
	msleep(10);

	dbbusDWIICEnterSerialDebugMode();
	dbbusDWIICStopMCU();
	dbbusDWIICIICUseBus();
	dbbusDWIICIICReshape();
	msleep(10);

	// Disable the Watchdog
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	i2c_write_msg2133 ( dbbus_tx_data, 4 );
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	i2c_write_msg2133 ( dbbus_tx_data, 4 );
	// Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	i2c_write_msg2133 ( dbbus_tx_data, 4 );
	/////////////////////////
	// Difference between C2 and C3
	/////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
	//check id
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0xCC;
	i2c_write_msg2133 ( dbbus_tx_data, 3 );
	i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );

	printk("%s, dbbus_rx_data = [%d][%d]", __func__, dbbus_rx_data[0], dbbus_rx_data[1]);

	//i2c_client->addr = 0x38;

	if ( dbbus_rx_data[0] != 2  && dbbus_rx_data[0] != 1)
	{
		printk("%s, auto upgrade firmware start.....\n", __func__);
		//fts_ctpm_fw_upgrade_for_funa(i2c_client, auto_temp, sizeof(auto_temp));
		ft6206_auto_update_fw();
		ft6206_reset();
		printk("%s, auto upgrade firmware end.\n", __func__);
		return 1;
	}

	return 0;

	

}
#endif //FT6206_FORCE_UPDATE

	
#endif


/*ergate-026 start*/
static int log_level = 2; //0-2(verbose)
#define TPD_LOGI(fmt, arg...)    if(log_level >= 1) \
        printk(KERN_INFO fmt, ##arg)

#define TPD_LOGV(fmt, arg...)    if(log_level >= 2) \
        printk(KERN_INFO fmt, ##arg)


static ssize_t show_loglevel(struct device_driver *ddri, char *buf);
static ssize_t store_loglevel(struct device_driver *ddri, char *buf, size_t count);
static DRIVER_ATTR(loglevel,   S_IWUSR | S_IRUGO, show_loglevel,      store_loglevel);
//extern int tpd_create_attr(struct driver_attribute *attr); 

static ssize_t show_loglevel(struct device_driver *ddri, char *buf)
{
	snprintf(buf,PAGE_SIZE,"level:%d\nrange:0-2(verbose)\n",log_level);
}


static ssize_t store_loglevel(struct device_driver *ddri, char *buf, size_t count)
{
	sscanf(buf,"%d",&log_level);
	return count;
}
/*ergate-026 end*/

#if defined(HQ_PROJECT_A18)
/*ergate-037 start*/
int state_down = 0;
int x_down,y_down,x_up,y_up;
struct timeval time_down,time_up;

int tpd_check_allow_key(void)
{
	int ret = 1;
	int64_t  delay;
	struct timeval now;

	do_gettimeofday(&now); 
	delay = (now.tv_sec*1000 + now.tv_usec/1000) - (time_up.tv_sec*1000 + time_up.tv_usec/1000);

	if((y_up-y_down > 100 && y_up > 300 && y_up < 400 && delay < (int64_t)80) ||
		(y_up-y_down > 100 && y_up >= 400 && y_up < 450 && delay < (int64_t)200) ||
		(y_up-y_down > 100 && y_up >= 450 && delay < (int64_t)300))
		ret = 0;

	TPD_LOGI("y_dis=%d,y_up=%d,delay=%lldms, ret:%d \n",y_up-y_down,y_up,delay,ret);

	return ret;
}
/*ergate-037 end*/
#endif

/*FW UPDATE*/
static int i2c_master_send_ext(struct i2c_client *client,const char *buf ,int count);
static int i2c_master_recv_ext(struct i2c_client *client, char *buf ,int count);







/*ergate-001 start*/
static void tpd_hw_power(unsigned int on)
{
#if  1 //
	if(on)
	{
	    //power on, need confirm with SA
	    //hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	    //hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP"); 
	    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
	}
	else
	{
	    //hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
	    //hwPowerDown(MT65XX_POWER_LDO_VGP, "TP");
	    // hwPowerOn(TPD_POWER_SOURCE_CUSTOM, "TP");
	}
	msleep(30);
#else
	mt_set_gpio_mode(GPIO_CTP_EN_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN,GPIO_DIR_OUT);
	if(on)
	{
		mt_set_gpio_out(GPIO_CTP_EN_PIN,GPIO_OUT_ONE);
	}
	else
	{
		mt_set_gpio_out(GPIO_CTP_EN_PIN,GPIO_OUT_ZERO);
	}
#endif
}


static void tpd_hw_reset(void)
{
	mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
	mdelay(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
        mdelay(20);
}
static void ft6206_reset()
{
	//tmp_prox = 0;

	msleep(10);
	mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
	msleep(10);
	mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
	msleep(300);
	//tmp_prox = 1;
}


static void tpd_hw_enable(unsigned int on)
{
	mt_set_gpio_mode(GPIO_CTP_RST_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN,GPIO_DIR_OUT);
#ifdef USE_RTP_RST
	mt_set_gpio_mode(GPIO_RTP_RST_PIN,GPIO_RTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_RTP_RST_PIN,GPIO_DIR_OUT);
#endif
    mdelay(10);
	if(on)
	{
		mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
		mdelay(10);		
		mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
		mdelay(20);
		mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
#ifdef USE_RTP_RST
		mt_set_gpio_out(GPIO_RTP_RST_PIN,GPIO_OUT_ONE);
#endif
	}
	else
	{
		mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
#ifdef USE_RTP_RST
		mt_set_gpio_out(GPIO_RTP_RST_PIN,GPIO_OUT_ZERO);
#endif
	}
    mdelay(20);//50 jin 提高从暗屏待机唤醒到屏点亮的速度
}


/*
mode:
0: output 0
1: output 1
2: eint
*/
static void tpd_hw_config_eint_pin(int mode)
{
	switch(mode)
	{
	case 0:
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN,GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EINT_PIN,GPIO_OUT_ZERO);
		break;
	case 1:
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN,GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EINT_PIN,GPIO_OUT_ONE);
		break;
	case 2:
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
		break;
	default:
		break;			
	}
}
/*ergate-001 end*/


#if 1
/*ergate-009 start*/
static ssize_t show_calibrate(struct device_driver *ddri, char *buf);
static ssize_t store_calibrate(struct device_driver *ddri, char *buf, size_t count);
static DRIVER_ATTR(calibrate,   S_IWUSR | S_IRUGO, show_calibrate,      store_calibrate);
//extern int tpd_create_attr(struct driver_attribute *attr); 

static int tpd_hw_calibrate(void)
{
	int retval = TPD_OK;

	return 0;
}

static ssize_t show_calibrate(struct device_driver *ddri, char *buf)
{
    TPD_LOGI("tpd show_calibrate\n");

    snprintf(buf,PAGE_SIZE,"%s\n","tpd show_calibrate");
}


static ssize_t store_calibrate(struct device_driver *ddri, char *buf, size_t count)
{
    int retval = TPD_OK;
    printk("tpd store_calibrate:%s\n",buf);
    if(!strncmp(buf,"cali",4))
    {
        printk("tpd begin calibration...\n");
        retval = tpd_hw_calibrate();
        printk("tpd end calibration=%d\n",retval);
    }
    else
    {
        printk("Invalid format\n");
    }
}
/*ergate-009 end*/
#endif


/*ergate-013 start*/
#define MAX_CMD_LEN 255
static int i2c_master_send_ext(struct i2c_client *client,const char *buf ,int count)
{
#if 0
    u32 phyAddr = 0; 
    u8 *buf_dma = NULL;
    u32 old_addr = 0; 
    int ret = 0; 
    int retry = 3; 
    
    if (count > MAX_CMD_LEN) {
        TPD_LOGI("[i2c_master_send_ext] exceed the max write length \n"); 
        return -1; 
    }

    phyAddr = 0; 
    buf_dma = dma_alloc_coherent(0, count, &phyAddr, GFP_KERNEL);
    if (NULL == buf_dma) {
        TPD_LOGI("[i2c_master_send_ext] Not enough memory \n"); 
        return -1; 
    }
    memcpy(buf_dma, buf, count); 
    old_addr = client->addr; 
    //client->addr = ( (client->addr &  I2C_MASK_FLAG) | I2C_DMA_FLAG ); 
    client->addr = ( client->addr | I2C_DMA_FLAG ); //save flag I2C_ENEXT_FLAG

#if 0
    int i =0; 
    for (i = 0; i < count; i++) {
        TPD_LOGV("0x%02x ", buf_dma[i]); 
    }
#endif     

    do {
        ret = i2c_master_send(client, (u8*)phyAddr, count);     
        retry --; 
        if (ret != count) {
            TPD_LOGI("[i2c_master_send_ext] Error sent I2C ret = %d\n", ret); 
        }
    }while ((ret != count) && (retry > 0)); 

    dma_free_coherent(0, count, buf_dma, phyAddr);

    client->addr = old_addr; 

    return ret; 
#endif
	u8 *buf_dma = NULL;
	u32 old_addr = 0; 
	int ret = 0; 
	int retry = 3; 

	if (count > MAX_CMD_LEN) {
		printk("[i2c_master_send_ext] exceed the max write length \n"); 
		return -1; 
	}
	
	buf_dma= kmalloc(count,GFP_KERNEL);
	
	old_addr = client->addr; 
	client->addr |= I2C_ENEXT_FLAG ; 
	
	memcpy(buf_dma, buf, count); 
	
	do {
		ret = i2c_master_send(client, (u8*)buf_dma, count); 	
		retry --; 
		if (ret != count) {
			printk("[i2c_master_send_ext] Error sent I2C ret = %d\n", ret); 
		}
	}while ((ret != count) && (retry > 0)); 

	client->addr = old_addr; 
	kfree(buf_dma);
	
	return ret; 

}


static int i2c_master_recv_ext(struct i2c_client *client, char *buf ,int count)
{
    u32 phyAddr = 0; 
    u8  buf_dma[8] = {0};
    u32 old_addr = 0; 
    int ret = 0; 
    int retry = 3; 
    int i = 0; 
    u8  *buf_test ;
	buf_test = &buf_dma[0];

    old_addr = client->addr; 
    client->addr |= I2C_ENEXT_FLAG ;
	
	//printk("[i2c_master_recv_ext] client->addr = %x\n", client->addr); 

    do {
        ret = i2c_master_recv(client, buf_dma, count);   
        retry --; 
        if (ret != count) {
            printk("[i2c_master_recv_ext] Error sent I2C ret = %d\n", ret); 
        }
    }while ((ret != count) && (retry > 0)); 
    
    memcpy(buf, buf_dma, count); 
	
    client->addr = old_addr; 

    return ret; 
}
/*ergate-013 end*/


static int tpd_switch_mode(U8 mode)
{
	int retval = TPD_OK;
    struct TpdPacketT packet = 
    {
        TPD_PACKET_HEAD_CMD,
        TPD_PACKET_CMD_MODE_WRITE,
        0x00,
        mode
    };
 
    retval = i2c_master_send(i2c_client, &packet, sizeof(packet));
    msleep(5);
    if(mode != TPD_MODE_FREEZE)
        retval = i2c_master_recv(i2c_client, &packet, sizeof(packet));

	if(retval < TPD_OK)
		return -1;

	return 0;
}


static void tpd_down(int x, int y, int p)
{
#if defined(REVERSE_X)	
    if(x<=TPD_RES_X && y<=TPD_RES_Y) x=TPD_RES_X-x;
#endif
#if defined(REVERSE_Y)	
    if(x<=TPD_RES_X && y<=TPD_RES_Y) y=TPD_RES_Y-y;
#endif
	input_report_abs(tpd->dev, ABS_PRESSURE, 128);
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 128);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	TPD_LOGV("######tpd_down[%4d %4d %4d] ", x, y, p);
	input_mt_sync(tpd->dev);
	TPD_DOWN_DEBUG_TRACK(x,y);
 	//printk("jin msg2133 tpd_down  x=%d  y=%d\n",x,y);

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{	
	  tpd_button(x, y, 1);	
	}
	
}

#if defined(HQ_PROJECT_A16)
static int tpd_up(int x, int y,int p)
{
//	input_report_abs(tpd->dev, ABS_PRESSURE, 0);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
//	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
//	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
//	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//printk("######UP[%4d %4d %4d] ", x, y, p);
	TPD_LOGV("######tpd_up[%4d %4d %4d] ", x, y, p);
	input_mt_sync(tpd->dev);
	TPD_UP_DEBUG_TRACK(x,y);

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{	
	  tpd_button(x, y, 0);	
	}
	
}
#else
static int tpd_up(int x, int y,int *count)
{
	if(*count>0) 
	{
		input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		//TPD_LOGV("U[%4d %4d %4d] ", x, y, 0);
		TPD_LOGV("######tpd_up[%4d %4d %4d] ", x, y, p);
		input_mt_sync(tpd->dev);
		TPD_UP_DEBUG_TRACK(x,y);
		
		if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
		{	
		  tpd_button(x, y, 0);	
		}
		
		(*count)--;
		return 1;
	} 
	return 0;
}
#endif

static int tpd_touchinfo(struct TouchInfoT *cinfo, struct TouchInfoT *pinfo)
{
	u32 retval;
       u8 key;
	static U8 pre_is_key = 0;
	char dbbus_tx_start = 0x0;
	//pinfo->count = cinfo->count;
    memcpy(pinfo, cinfo, sizeof(struct TouchInfoT));
    //printk("hsh_0808\n");
    //add for sure addr correct
    //i2c_client->addr = 0x4c;
	//billy   HalTscrCDevWriteI2CSeq(0x4C, &dbbus_tx_start, 1);
#if 0
	// retval=HalTscrCDevWriteI2CSeq(0x4c, &dbbus_tx_start, 1);   //0x4C add by tianhaiyang
//	if(retval<0)
	//{
		//printk("Send to TP 0x00 error\n");
		//return 0;
	//}

	//i2c_client->addr = 0x4c;
    retval = i2c_master_recv_ext(i2c_client, (u8 *)&TpdTouchData, sizeof(TpdTouchData));
    FT6206_DBG("Receive data from TP error retval=%d\n",retval);
	if(retval<0)
	{
		FT6206_DBG("Receive data from TP error\n");
		return 0;
	}
	//printk("jiangtao 0: %d, %d, %d ,%d, %d ,%d, %d, %d \n",*(p+5), *(p+5),*(p+5),*(p+5),*(p+5),*(p+5),*(p+5),*(p+5));
#endif
	i2c_master_send(i2c_client, &dbbus_tx_start, 1);
    retval = i2c_master_recv_ext(i2c_client, (u8 *)&TpdTouchData, sizeof(TpdTouchData));

	if(TpdTouchData.packet_id != 0x52 )
    {
		return 0;
    }
	/*touch*/
	FT6206_DBG("FT6206ToMsg2133 data= 0x%x, 0x%x, 0x%x ,0x%x, 0x%x ,0x%x, 0x%x \n",TpdTouchData.packet_id,TpdTouchData.x_h_y_h,TpdTouchData.x_l,TpdTouchData.y_l,TpdTouchData.disx_h_disy_h,TpdTouchData.disx_l,TpdTouchData.disy_l);
	if(TpdTouchData.packet_id == 0x52)
	{
	
		if(TpdTouchData.x_h_y_h == 0xFF 
			&& TpdTouchData.x_l == 0xFF 
			&& TpdTouchData.y_l == 0xFF 
			&& TpdTouchData.disx_h_disy_h == 0xFF 
          )
        {
#ifdef TPD_HAVE_BUTTON
		 {
			U8 *p = &TpdTouchData;
			cinfo->key_value = 0;
			cinfo->key_value = *(p+5);			
			//printk("p1: (%3d)\n",cinfo->key_value);
			{
				//tpd_button(cinfo->key_value);
				 if((cinfo->key_value == 0)||(cinfo->key_value == 0xff))
					{
					//printk("jiangtao 0: %d, %d, %d ,%d, %d ,%d, %d, %d \n",*(p+5), *(p+5),*(p+5),*(p+5),*(p+5),*(p+5),*(p+5),*(p+5));
						if(pre_is_key == 1)
							{
							cinfo->x1 = tpd_keys_dim_local[0][0];
							cinfo->y1 = tpd_keys_dim_local[0][1];
							pre_is_key = 0;
							}
						else if(pre_is_key == 2)
							{
							cinfo->x1 = tpd_keys_dim_local[1][0];
							cinfo->y1 = tpd_keys_dim_local[1][1];
							pre_is_key = 0;
							}
						else if(pre_is_key == 4)
							{
							cinfo->x1 = tpd_keys_dim_local[2][0];
							cinfo->y1 = tpd_keys_dim_local[2][1];
							pre_is_key = 0;
							}

						cinfo->count = 0;
					}
				else
					{
						if(cinfo->key_value == 1)
							{
							pre_is_key = 1;
							cinfo->x1 = tpd_keys_dim_local[0][0];
							cinfo->y1 = tpd_keys_dim_local[0][1];
							}
						else if(cinfo->key_value == 2)
							{
							pre_is_key = 2;
							cinfo->x1 = tpd_keys_dim_local[1][0];
							cinfo->y1 = tpd_keys_dim_local[1][1];
							}
						else if(cinfo->key_value == 4)
							{
							pre_is_key = 4;
							cinfo->x1 = tpd_keys_dim_local[2][0];
							cinfo->y1 = tpd_keys_dim_local[2][1];
							}

					
					//tpd_button(cinfo->key_value);
					//FT6206_DBG("jiangtao 1: %d, %d, %d ,%d, %d ,%d, %d, %d \n",*(p+5), *(p+5),*(p+5),*(p+5),*(p+5),*(p+5),*(p+5),*(p+5));
						cinfo->count = 1;

					}
					//tpd_button(cinfo->key_value);
				
			}
		 }
				return 1;
#endif
			cinfo->count = 0;
        }
		else if(TpdTouchData.disx_h_disy_h == 0
			&& TpdTouchData.disx_l == 0 
			&& TpdTouchData.disy_l == 0)
			cinfo->count = 1;
		else
			cinfo->count = 2;
		
		//printk("cinfo: count=%d\n",cinfo->count);
		if(cinfo->count > 0)
		{
			int tmp_x,tmp_y;
			/*point1*/
			#ifdef SWITCH_XY //jin 20120808 交换XY的值
			cinfo->y1 = (((TpdTouchData.x_h_y_h&0xF0)<<4) | (TpdTouchData.x_l));
			cinfo->x1 = (((TpdTouchData.x_h_y_h&0x0F)<<8) | (TpdTouchData.y_l));
			#else
			cinfo->x1 = (((TpdTouchData.x_h_y_h&0xF0)<<4) | (TpdTouchData.x_l));
			cinfo->y1 = (((TpdTouchData.x_h_y_h&0x0F)<<8) | (TpdTouchData.y_l));
			#endif
			if(cinfo->count >1)
			{	
				/*point2*/
				short disx,disy;
	            			
				#ifdef SWITCH_XY //jin 20120808 交换XY的值
				disy = (((TpdTouchData.disx_h_disy_h&0xF0)<<4) | (TpdTouchData.disx_l));
				disx = (((TpdTouchData.disx_h_disy_h&0x0F)<<8) | (TpdTouchData.disy_l));
				#else
				disx = (((TpdTouchData.disx_h_disy_h&0xF0)<<4) | (TpdTouchData.disx_l));
				disy = (((TpdTouchData.disx_h_disy_h&0x0F)<<8) | (TpdTouchData.disy_l));
				#endif
				disy = (disy<<4);
				disy = disy/16;
				if(disx >= 2048)
					disx -= 4096;
				if(disy >= 2048)
					disy -= 4096;
				cinfo->x2 = cinfo->x1 + disx;
				cinfo->y2 = cinfo->y1 + disy;				

				tmp_x = cinfo->x2;
				tmp_y = cinfo->y2;
				cinfo->y2 = tmp_y * (TPD_RES_Y - 1)/ 2047;
				cinfo->x2 = tmp_x * (TPD_RES_X - 1) / 2047;		
			}
			tmp_x = cinfo->x1;
			tmp_y = cinfo->y1;
			cinfo->y1 = tmp_y * (TPD_RES_Y - 1) / 2047;
			cinfo->x1 = tmp_x * (TPD_RES_X - 1) / 2047;

			//printk("p1: (%3d,%3d)(%3d,%3d)\n",cinfo->x1,cinfo->y1,TPD_RES_X,TPD_RES_Y);
	        cinfo->pressure = 1;
	        //printk("pressure: %d\n",cinfo->pressure);
		}
	}
	else
	{
		cinfo->count = 0;
	}

    /*ergate-012 start*/
    /*ergate-012 end*/

	return 1;
}

static int touch_event_handler(void *unused)
{
	int pending = 0;
	struct TouchInfoT cinfo, pinfo;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
	
	FT6206_DBG("FT6206ToMsg2133:touch_event_handler\n");
	
    memset(&cinfo, 0, sizeof(struct TouchInfoT));
    memset(&pinfo, 0, sizeof(struct TouchInfoT));
	do
	{
		set_current_state(TASK_INTERRUPTIBLE); 
		if(!kthread_should_stop())
		{
			TPD_DEBUG_CHECK_NO_RESPONSE;

			do
			{
				if (pending) 
					wait_event_interruptible_timeout(waiter, tpd_flag != 0, HZ/10);
				else 
					wait_event_interruptible_timeout(waiter,tpd_flag != 0, HZ*2);
				 
			}while(0);

			

			 
			if (tpd_flag == 0 && !pending) 
				continue; // if timeout for no touch, then re-wait.
			 
			if (tpd_flag != 0 && pending > 0)	
				pending = 0;
			 
			tpd_flag = 0;
			TPD_DEBUG_SET_TIME; 
		}
		set_current_state(TASK_RUNNING);

		if (tpd_touchinfo(&cinfo, &pinfo))
		{
			if(cinfo.count >0)
			{
			    #if defined(HQ_PROJECT_A16)
			    pre_count =cinfo.count;
				pre_x1 =cinfo.x1;
			    pre_y1 =cinfo.y1;
				#endif
				tpd_down(cinfo.x1, cinfo.y1, cinfo.pressure);
				if(cinfo.count>1)
             	{
             	
                 #if defined(HQ_PROJECT_A16)
					pre_x2 =cinfo.x2;
					pre_y2 =cinfo.y2;
		   #endif	
					tpd_down(cinfo.x2, cinfo.y2, cinfo.pressure);
				}
				input_sync(tpd->dev);
	#if defined(HQ_PROJECT_A18)
				/*ergate-037 start*/
				if(state_down == 0)
				{
					x_down = cinfo.x1;
					y_down = cinfo.y1;
					do_gettimeofday(&time_down);
				}
				if(state_down == 1)
				{
					x_up = cinfo.x1;
					y_up = cinfo.y1;
					do_gettimeofday(&time_up);
				}
				state_down = 1;
				/*ergate-037 end*/
	#endif				
				FT6206_DBG("press  --->\n");
			}
			else if(cinfo.count==0 && pinfo.count!=0)
			{
			
             #if defined(HQ_PROJECT_A16)
				tpd_up(pre_x1, pre_y1, cinfo.pressure);
				if(pre_count > 1)
				{
					tpd_up(pre_x2, pre_y2, cinfo.pressure);
				}				
				input_sync(tpd->dev);
			 #else
			 input_mt_sync(tpd->dev);
			 input_sync(tpd->dev);
			 #endif
	#if defined(HQ_PROJECT_A18)
				/*ergate-037*/
				state_down = 0;
	#endif
				FT6206_DBG("release --->\n"); 
			}
		}		   
	}while(!kthread_should_stop());

	return 0;
}

 
static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
{
	strcpy(info->type, "ft6206msg2133");
	return 0;
}

 
static void tpd_eint_interrupt_handler(void)
{
	// mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); //1009 mask eint
	//TPD_LOGV("TPD interrupt has been triggered\n");
	//printk("jiangtao msg2133:tpd_eint_interrupt_handler\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}
//zhang added 
static int ver_ft6206_get_vendorid(struct i2c_client *client)
{
#if 0
	u8 buf[FTS_SETTING_BUF_LEN];
	u8 reg_val[2] = {0};
	u8 auc_i2c_write_buf[10] = {0};
	u32 i = 0;
	int i_ret;

	/*Step 1:Reset  CTPM
	*write 0xaa to register 0xfc
	*/
	fts_write_reg(client, 0xbc, 0xaa);
	msleep(50);

	/*write 0x55 to register 0xfc */
	fts_write_reg(client, 0xbc, 0x55);
	msleep(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i++;
		i_ret = ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);
		msleep(5);
	} while (i_ret <= 0 && i < 5);


	/*********Step 3:check READ-ID***********************/
	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;

	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
printk("reg_val[0] =0x%x reg_val[1]=0x%x\n",reg_val[0],reg_val[1]);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x8)
		dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			 reg_val[0], reg_val[1]);
	else
		return -EIO;
 
	auc_i2c_write_buf[0] = 0xcd;
	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	dev_dbg(&client->dev, "bootloader version = 0x%x\n", reg_val[0]);

	/*--------- read current project setting  ---------- */
	/*set read start address */
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;

	ft5x0x_i2c_Read(client, buf, 4, buf, FTS_SETTING_BUF_LEN);
printk("buf[4] =0x%x buf[5]=0x%x\n",buf[4],buf[5]);
	if (buf[4]==0x53 && buf[5]==0xac)
	{
		return 0;
	}
	else
	{
		return -1;
	}
	
	msleep(100);
	return 0;
#endif
	u8 reg_val[2] = {0};
	u32 i = 0;
	u8 is_5336_new_bootloader = 0;
	u32 packet_number;
	u32 j=0;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	u8 buf[FTS_SETTING_BUF_LEN];
      // struct Upgrade_Info upgradeinfo;
#if 1
	  #ifdef FT6206_UPDATE_DMA_WAY//for MT6572
		FT6206_DBG("wingtech enter 2\n ");

		I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, FTS_DMA_BUF_SIZE, &I2CDMABuf_pa, GFP_KERNEL);
	
		if(!I2CDMABuf_va){
			FT6206_DBG("wingtech enter DMA faild\n ");
			FT6206_DBG("FTP: %s Allocate DMA I2C Buffer failed!\n", __func__);
			return -EIO;
		}
		FT6206_DBG("wingtech enter DMA successed\n ");
		FT6206_DBG("FTP: %s I2CDMABuf_pa=%x,val=%x val2=%x\n", __func__, &I2CDMABuf_pa,I2CDMABuf_pa,(unsigned char *)I2CDMABuf_pa);
	#endif
#endif
	//fts_get_upgrade_info(&upgradeinfo);
	for (i = 0; i < 30; i++) 
	{
	        msleep(100);
		printk("[FTS] zhang Step 1:Reset  CTPM\n");
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		//if (DEVICE_IC_TYPE == IC_FT6208 || DEVICE_IC_TYPE == IC_FT6x06)
		//if(fts_updateinfo_curr.CHIP_ID==0x05 || fts_updateinfo_curr.CHIP_ID==0x06 ) 
		//	fts_write_reg(client, 0xbc, 0xaa);
		//else
		//	fts_write_reg(client, 0xfc, FT_UPGRADE_AA);
		//msleep(100);


		/*write 0x55 to register 0xfc */
		//if(DEVICE_IC_TYPE == IC_FT6208 || DEVICE_IC_TYPE == IC_FT6x06)
		//if(fts_updateinfo_curr.CHIP_ID==0x05 || fts_updateinfo_curr.CHIP_ID==0x06 )
		//	fts_write_reg(client, 0xbc, 0x55);
		//else
		//	fts_write_reg(client, 0xfc, FT_UPGRADE_55);
		client->addr = 0x38 | I2C_ENEXT_FLAG;
    		msleep(5);
		mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
		msleep(10);
		mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
              mdelay(300);

		if(i<=15)
		{
		msleep(30+i*3);
		}
		else
		{
		msleep(30-(i-15)*2);
		}

   
		/*********Step 2:Enter upgrade mode *****/
		printk("[FTS] zhang Step 2:Enter upgrade mode \n");
		#if 1
			auc_i2c_write_buf[0] = FT_UPGRADE_55;
			auc_i2c_write_buf[1] = FT_UPGRADE_AA;
			do {
				j++;
				i_ret = ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);
				msleep(5);
			} while (i_ret <= 0 && j < 5);
		#else
			auc_i2c_write_buf[0] = 0x55;
			ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
			msleep(5);
			auc_i2c_write_buf[0] = 0xaa;
			ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
		#endif

#if 1
		/*********Step 3:check READ-ID***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
		ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
             msleep(5);
		/*--------- read current project setting  ---------- */
		/*set read start address */
		buf[0] = 0x3;
		buf[1] = 0x0;
		buf[2] = 0x78;
		buf[3] = 0x0;

		ft5x0x_i2c_Read(client, buf, 4, buf, 128);
		printk("zhang buf[4] =0x%x buf[5]=0x%x\n",buf[4],buf[5]);
		
		printk("[FTS] zhang Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
		if ((reg_val[0] == 0x79
			&& reg_val[1] == 0x08)&&(buf[4]==0x53 && buf[5]==0xac)) {//mudong
			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				//reg_val[0], reg_val[1]);
			printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
		
#endif	
	}
#if 1
	#ifdef FT6206_UPDATE_DMA_WAY//for MT6572
		if(I2CDMABuf_va){
			dma_free_coherent(NULL, FTS_DMA_BUF_SIZE, I2CDMABuf_va, I2CDMABuf_pa);
			I2CDMABuf_va = NULL;
			I2CDMABuf_pa = 0;
		}
	#endif
#endif
	client->addr = 0x38;
	if (i >= 30)
		return -EIO;
	
	//auc_i2c_write_buf[0] = 0xcd;
	//ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	//if (reg_val[0] > 4)
	//	is_5336_new_bootloader = 1;
	return 0;
}
#if 0 //def FT6206_UPDATE_DMA_WAY//for MT6572
static int funa_dma_create()
{
	FT6206_DBG("wingtech enter 2\n ");

	I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, FTS_DMA_BUF_SIZE, &I2CDMABuf_pa, GFP_KERNEL);

	if(!I2CDMABuf_va){
	FT6206_DBG("wingtech enter DMA faild\n ");
	FT6206_DBG("FTP: %s Allocate DMA I2C Buffer failed!\n", __func__);
	return -EIO;
	}
	FT6206_DBG("wingtech enter DMA successed\n ");
	FT6206_DBG("FTP: %s I2CDMABuf_pa=%x,val=%x val2=%x\n", __func__, &I2CDMABuf_pa,I2CDMABuf_pa,(unsigned char *)I2CDMABuf_pa);
}
static void funa_dma_release()
{
	if(I2CDMABuf_va){
		dma_free_coherent(NULL, FTS_DMA_BUF_SIZE, I2CDMABuf_va, I2CDMABuf_pa);
		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;
	}
}
#endif
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	int error;
	int retval = TPD_OK;
	i2c_client = client;
	#ifdef FT6206_AUTO_UPDATE
	struct fw_version fw;
	int err = 0, time = 0;
	#endif
	FwDataCnt = 0;
    FT6206_DBG("FT6206ToMsg2133 tpd_probe begin\n");
    /*ergate-dbg*/
    i2c_client->addr |= I2C_ENEXT_FLAG; //I2C_HS_FLAG;
	i2c_client->timing = 100;

	extern int tpd_load_status;

	/*ergate-016 start*/

#ifdef MT6575
	//power on, need confirm with SA
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif  

//wangfuqiang add begin
#ifdef TPD_POWER_SOURCE_CUSTOM
	   hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	   hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	   hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 
//wangfuqiang add end

#if 0
   // set deep sleep off
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
    msleep(300); 
    FT6206_DBG("FT6206ToMsg2133 reset \n");
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(300); 
	printk("FT6206ToMsg2133 reset 2 \n");
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE); 

#ifdef USE_RTP_RST
    mt_set_gpio_mode(GPIO_RTP_RST_PIN, GPIO_RTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_RTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_RTP_RST_PIN, GPIO_OUT_ONE);  
    msleep(10);  
#endif
#endif
    mdelay(20);//50 jin 提高从暗屏待机唤醒到屏点亮的速度

	tpd_hw_power(1);
	msleep(50);
	tpd_hw_enable(1);


	msleep(400);    //add delay by tianhaiyang
	FT6206_DBG("FT6206ToMsg2133 add delay time =400\n");
	//wangfuqiang add begin

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
   	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_DOWN);
		
		//wangfuqiang add end

#if FT6206_AUTO_UPDATE//
	//while(time < 5){
	
		err = ver_ft6206_get_version(&fw);
		if(err < 0){
			FT6206_DBG("%s, I2C not ask\n", __func__);
			//zhang added	
			if(0==ver_ft6206_get_vendorid( i2c_client))//mudong	
				return -1;
			//return -ENODEV;
			//return -1;
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
			FT6206_firmware_AUTO_update_store(0);
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
			//break;
		}
		//time++;
		//if((fw.major & 0xff00) == 0)
		//	break;
		//msleep(10);
	//}
		//FT6206_DBG("%s, time = %d, w_version = [0x%x].[0x%x]\n", __func__, time, fw.major, fw.minor);
	#if 0
			if(fw.major >= 1 && fw.major <= 2 ){
				//tp_type = FT6206_TP_HUANGZE; //major = 1
				//factory_set_prop(FACTORY_TP, "FT6206_Huangze");
				FT6206_DBG("%s, This is HUANGZE TP\n", __func__);
			}
			//else{
	#if 0//def FT6206_FORCE_UPDATE
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
				if(ft6206_firmware_force_update() == 0)
	#endif
				//{
					FT6206_DBG("%s, This is NO TP\n", __func__);
					//return -ENODEV;
					//return -1;
				//}
			//mtk_wdt_restart(WK_WDT_EXT_TYPE);
			//mtk_wdt_restart(WK_WDT_LOC_TYPE);
			//}
	#endif
#if 0//def FT6206_AUTO_UPDATE
			// mod_timer(&ft6206_auto_update_delay,jiffies + msecs_to_jiffies(AUTO_UPDATER_DELAY * 1000));
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
			//FT6206_firmware_AUTO_update_store(0);
			ft6206_auto_update_fw();
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
#endif
#endif
	/*ergate-016 end*/ 
#if 1
	error = tpd_initialize(client);
	FT6206_DBG("FT6206ToMsg2133 tpd_initialize error=%d\n",error);
	if(error)
	{
		TPD_LOGI("tpd_initialize error\n");
		tpd_load_status = 0;
		FT6206_DBG("FT6206ToMsg2133 tpd_initialize error\n");
		return -1;
	}
	
#endif
#if 0
    /*ergate-009*/
    tpd_create_attr(&driver_attr_calibrate);
    /*ergate-026*/
    tpd_create_attr(&driver_attr_loglevel);
#endif

//#ifdef CONFIG_HAS_EARLYSUSPEND
	//if(!(retval < TPD_OK)) 
	{
		//early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 30;
		early_suspend.suspend = tpd_early_suspend;
		early_suspend.resume = tpd_late_resume;
		register_early_suspend(&early_suspend);
	}
//#endif

	/*ergate-001*/
	/*skip wrong EINT during init*/
	//msleep(500);  

	thread = kthread_run(touch_event_handler, 0, "ft6206msg2133");
	FT6206_DBG("FT6206ToMsg2133 kthread_run finish\n");
	if(IS_ERR(thread))
	{ 
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread\n");
		FT6206_DBG("FT6206ToMsg2133 failed to create kernel thread\n");
	}
/*
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, 1);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1); 
	*/
	 mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 
	printk("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

	tpd_load_status = 1;
#ifdef FT6206_SOFTWARE_UPDATE
		//ft6206_init_fw_class();
		proc_update_firmeare_init();
#endif

#if FT6206_AUTO_UPDATE
			// mod_timer(&ft6206_auto_update_delay,jiffies + msecs_to_jiffies(AUTO_UPDATER_DELAY * 1000));
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
			//FT6206_firmware_AUTO_update_store(0);
			ft6206_auto_update_fw();
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
#endif
	return 0;	  
}
 

static int tpd_power_on ()
{
	int retval = TPD_OK;
	int tries = 0;
	u8 host_reg;
	FT6206_DBG(" 2133 Power on\n");

	 //power on, need confirm with SA
    //hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    //hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");   
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");

	/*ergate-001 start*/
	tpd_hw_power(0);
	tpd_hw_power(1);
	msleep(20);
	tpd_hw_enable(1);
    tpd_hw_config_eint_pin(2);
	/*ergate-001 end*/
 
	return retval;
}
static int tpd_initialize(struct i2c_client * client)
{
	int timer_Count=0;
	int retval = TPD_OK;
    struct TpdPacketT packet = 
    {
        TPD_PACKET_HEAD_CMD,
        TPD_PACKET_CMD_FORMAT_WRITE,
        0x00,
        0x00
    };
    struct TpdPacketT pr;
    unsigned char buf[4];
    while(timer_Count<30)
    {
    		if((timer_Count % 5) == 0 && timer_Count != 0){
			tpd_hw_reset();
			mdelay(300);
		}
	    buf[0] = 0x53;
	    buf[1] = 0x00;        
	    buf[2] = 0x74;
	
    memset(&tpd_firmware_version,0, sizeof(tpd_firmware_version));
//	retval = tpd_power_on();
	
	msleep(200);
	retval = i2c_master_send(i2c_client, buf, 3);
			FT6206_DBG("FT6206ToMsg2133 the retval write = %d,\n",retval);
			msleep(20);
	retval = i2c_master_recv(i2c_client, buf, 4);
			FT6206_DBG("FT6206ToMsg2133 the retval read = %d,\n",retval);
	tpd_firmware_version[0]=buf[0];
	tpd_firmware_version[1]=buf[1];
			timer_Count++;
			FT6206_DBG("FT6206ToMsg2133 the retval read fw.major= 0x%x,fw.minor= 0x%x\n",tpd_firmware_version[0],tpd_firmware_version[1]);
			if(tpd_firmware_version[0]==0x01)
			{
					break;
			}
			msleep(50);
	}
	if(retval > TPD_OK)
		FT6206_DBG("FT6206ToMsg2133 Firmware Version:0x%02x,0x%02x\n",tpd_firmware_version[0],tpd_firmware_version[1]);
	else
		FT6206_DBG("FT6206ToMsg2133 Firmware Version:error\n");

	if(retval > TPD_OK)
		retval = TPD_OK;
	else
		retval = 1;
	return retval;
}


static int __devexit tpd_remove(struct i2c_client *client) 
{
	int error;
 
  #ifdef CONFIG_HAS_EARLYSUSPEND
	 unregister_early_suspend(&early_suspend);
  #endif /* CONFIG_HAS_EARLYSUSPEND */
   
	printk("TPD removed\n");
 
	return 0;
}
 
 
static int tpd_local_init(void)
{
	int retval;
 
	printk("ft6206 msg2133 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

//	retval = i2c_add_driver(&tpd_driver);
   if(i2c_add_driver(&tpd_driver)!=0)
   	{
  		printk("ft6206 msg2133 unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	printk("ft6206 msg2133 add error touch panel driver.\n");
    	i2c_del_driver(&tpd_driver);
    	return -1;
    }

    /*ergate-012*/
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif 

    /*ergate-001*/
	tpd_type_cap = 1;
	return 0; 
}

static int tpd_resume(struct i2c_client *client)
{
	int retval = TPD_OK;
#if 1//!defined(HQ_PROJECT_A16)
	FT6206_DBG("FT6206ToMsg2133 wake up\n");
	/*ergate-001*/
	tpd_hw_power(1);
	//tpd_initialize(i2c_client);
	msleep(30);
	tpd_hw_enable(1);
    //tpd_hw_config_eint_pin(2);
	msleep(300);//jin 提高从暗屏待机唤醒到屏点亮的速度
	
	/*wait for CTP stablity like init*/
	//msleep(500);
	/*match with tpd_suspend()*/
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
#endif

#if 1
	tpd_up(0, 0, 0);
#endif

	return retval;
}

 
static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
	int retval = TPD_OK;
    char buf[4] ={0xff,0x44,0xff,0x01};

	retval = i2c_master_send(i2c_client, buf, 4);

	FT6206_DBG("FT6206ToMsg2133 enter sleep mode\n");
	msleep(30);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	/*mt65xx_eint_mask cann't stop EINT, so doing following*/
//	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, 			NULL, 1);
	/*ergate-001*/
   // tpd_hw_enable(0);
    //msleep(10);
   // tpd_hw_power(0);
 

    return retval;
}

void tpd_uniform_ctp(char *buf,int strlen)
{
    int retval = TPD_OK;
    char result = buf[strlen-1] + 1;
    int i = 0;


    for(i = 0;i < 5 && (result != buf[strlen-1]);i++)
    {
        i2c_client->addr = 0xc4 | I2C_ENEXT_FLAG;
        retval = i2c_master_send(i2c_client, buf, strlen);
        udelay(100);
        retval = i2c_master_send(i2c_client, buf, strlen-1);
        i2c_client->addr = 0xc5 | I2C_ENEXT_FLAG;
        retval = i2c_master_recv(i2c_client, &result, 1);
    }
}


#if 0//defined(HQ_PROJECT_A18) || defined(HQ_PROJECT_A16)
extern LCM_DRIVER  *lcm_drv;

int tpd_resume_in_lcm(void)
{
	int retval = TPD_OK;
    unsigned short addr_bake = 0;
	TPD_LOGI("TPD wake up\n");	
	msleep(30);
	tpd_hw_power(1);	
	tpd_hw_enable(1);	
	msleep(200);  
	
    //zym 2011-12-26
    //hupeng add to read out the ctp version
	char buf_1[5] = {0x53,0x45,0x52,0x44,0x42};
	char buf_2[1] = {0x37};
	char buf_3[1] = {0x35};
	char buf_4[1] = {0x71};
	char buf_5[1] = {0x45};
    
//lcd dijing 0x62
    char buf_dj_reg1[4] = {0x10,0x11,0x0e,0x19};
    char buf_dj_reg2[4] = {0x10,0x11,0x12,0x19};
	char buf_dj_reg3[4] = {0x10,0x11,0x20,0x80};
    char buf_dj_reg4[4] = {0x10,0x11,0x23,0x4B};
//lcd tcl 0x62
    char buf_tcl_reg1[4] = {0x10,0x11,0x0e,0x00};
    char buf_tcl_reg2[4] = {0x10,0x11,0x12,0x00};
	char buf_tcl_reg3[4] = {0x10,0x11,0x20,0x96};
    char buf_tcl_reg4[4] = {0x10,0x11,0x23,0x83};
	
#if !defined(HQ_PROJECT_A16)
	msleep(100);
#endif
	addr_bake = i2c_client->addr;
	i2c_client->addr = 0xc4 | I2C_ENEXT_FLAG;
	retval = i2c_master_send(i2c_client, buf_1, 5);
	msleep(1);
	retval = i2c_master_send(i2c_client, buf_2, 1);
	msleep(1);
	retval = i2c_master_send(i2c_client, buf_3, 1);
	msleep(1);
	retval = i2c_master_send(i2c_client, buf_4, 1);
//zym 2011-12-26
    TPD_LOGI("<<>> current lcd is %s \n",lcm_drv->name);
    if(!strncmp(lcm_drv->name,"ili9341_dijing",14))
    {
        TPD_LOGI("<<>> current lcd is %s \n",lcm_drv->name);
        tpd_uniform_ctp(buf_dj_reg1,4);
        tpd_uniform_ctp(buf_dj_reg2,4);
		tpd_uniform_ctp(buf_dj_reg3,4);
	    tpd_uniform_ctp(buf_dj_reg4,4);
    }
    else if(!strncmp(lcm_drv->name,"ili9341_tcl",11))
    {
        TPD_LOGI("<<>> current lcd is %s > \n",lcm_drv->name);
        tpd_uniform_ctp(buf_tcl_reg1,4);
        tpd_uniform_ctp(buf_tcl_reg2,4);  
		tpd_uniform_ctp(buf_tcl_reg3,4);
	    tpd_uniform_ctp(buf_tcl_reg4,4);
    }
    else
    {
        TPD_LOGI("current lcd is default \n");
        //donothing
    }
	i2c_client->addr = 0xc4 | I2C_ENEXT_FLAG;
	retval = i2c_master_send(i2c_client, buf_5, 1);
	msleep(1);
	i2c_client->addr = addr_bake;
//end by zym

	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, 			tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	return retval;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tpd_early_suspend(struct early_suspend *handler)
{
	tpd_suspend(i2c_client, PMSG_SUSPEND);	
}


static void tpd_late_resume(struct early_suspend *handler)
{
	tpd_resume(i2c_client);
}
#endif
 

static struct tpd_driver_t tpd_device_driver = 
{
		.tpd_device_name = "ft6206msg2133",
		.tpd_local_init = tpd_local_init,
	//	.suspend = tpd_suspend,
#if 0//defined(HQ_PROJECT_A18) || defined(HQ_PROJECT_A16)
		.resume = NULL,
#else
	//	.resume = tpd_resume,
#endif
#ifdef TPD_HAVE_BUTTON
		.tpd_have_button = 1,
#else
		.tpd_have_button = 0,
#endif		
};


/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	printk(" ft6206 msg2133 touch panel driver init\n");
	i2c_register_board_info(0, &i2c_tpd, 1);//wangfuqiang add
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("ft6206 add tpd driver failed\n");
	return 0;
}


/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("ft6206 MediaTek msg2133 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}


module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
