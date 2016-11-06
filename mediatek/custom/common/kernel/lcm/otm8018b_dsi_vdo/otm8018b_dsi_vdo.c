#ifdef BUILD_LK
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_OTM8018B	0x8009

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{0x00,	1,	{0x00}},
       {0xff,	3,	{0x80, 0x09, 0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x80, 0x09}},
	{0x00,	1,	{0x03}},
	{0xff,	1,	{0x01}},
	{0x00,	1,	{0xc6}},
	{0xb0,	1,	{0x03}},
	{0x00,	1,	{0x8b}},
	{0xb0,	1,	{0x40}},
	{0x00,	1,	{0x90}},
	{0xb3,	1,	{0x02}},
	{0x00,	1,	{0x92}},
	{0xb3,	1,	{0x45}},
	{0x00,	1,	{0xa2}},
	{0xC0,	3,	{0x04,0x00, 0x02}},
	{0x00,	1,	{0x80}},
	{0xC0,	5,	{0x00,0x58,0x00,0x14, 0x16}},
	{0x00,	1,	{0x90}},
	{0xC0,	6,	{0x00,0x4F,0x00,0x00,0x00, 0x03}},
	{0x00,	1,	{0xB4}},
	{0xC0,	1,	{0x10}},

	{0x00,	1,	{0x81}},
	{0xC1,	1,	{0x77}},//0X08
	{0x00,	1,	{0xA0}},
	{0xC1,	1,	{0xEA}},
	{0x00,	1,	{0xA1}},
	{0xC1,	1,	{0x0E}},
	{0x00,	1,	{0xA6}},
	{0xC1,	3,	{0x01,0x00,0x00}},
	{0x00,	1,	{0x80}},
	{0xC4,	2,	{0x30,0x83}},
	{0x00,	1,	{0x89}},
	{0xC4,	1,	{0x08}},
	{0x00,	1,	{0x82}},
	{0xC5,	1,	{0xA3}},
	{0x00,	1,	{0x90}},
	{0xC5,	2,	{0x96,0x76}},
	{0x00,	1,	{0xB1}},
	{0xC5,	1,	{0xA9}},
	{0x00,	1,	{0xC0}},
	{0xC5,	1,	{0x00}},
	{0x00,	1,	{0xB2}},
	{0xF5,	4,	{0x15,  0x00, 0x15, 0x00}},
	{0x00,	1,	{0x80}},
	{0xCE,	12,	{0x87, 0x03, 0x00, 0x85,
				0x03, 0x00, 0x86, 0x03,
				0x00, 0x84, 0x03, 0x00,}},
	{0x00,	1,	{0xA0}},
	{0xCE,	14,	{0x38, 0x03, 0x03, 0x58,
				0x00, 0x00, 0x00, 0x38,
				0x02, 0x03, 0x59, 0x00,
				0x00, 0x00}},
	{0x00,	1,	{0xB0}},
	{0xCE,	14,	{0x38, 0x01, 0x03, 0x5A,
				0x00, 0x00, 0x00, 0x38,
				0x00, 0x03, 0x5B, 0x00,
				0x00, 0x00}},
	{0x00,	1,	{0xC0}},
	{0xCE,	14,	{0x30, 0x00, 0x03, 0x5C,
				0x00, 0x00, 0x00, 0x30,
				0x01, 0x03, 0x5D, 0x00,
				0x00, 0x00}},
	{0x00,	1,	{0xD0}},
	{0xCE,	14,	{0x30, 0x02, 0x03, 0x5E,
				0x00, 0x00, 0x00, 0x30,
				0x03, 0x03, 0x5F, 0x00,
				0x00, 0x00}},
	{0x00,	1,	{0xC7}},
	{0xCF,	1,	{0x00}},
	{0x00,	1,	{0xC9}},
	{0xCF,	1,	{0x00}},
	{0x00,	1,	{0xC4}},
	{0xCB,	6,	{0x04, 0x04, 0x04, 0x04,
				0x04, 0x04}},
	{0x00,	1,	{0xD9}},
	{0xCB,	6,	{0x04, 0x04, 0x04, 0x04,
				0x04, 0x04}},
	{0x00,	1,	{0x84}},
	{0xCC,	6,	{0x0C, 0x0A, 0x10, 0x0E,
				0x03, 0x04}},
	{0x00,	1,	{0x9E}},
	{0xCC,	1,	{0x0B}},
	{0x00,	1,	{0xA0}},
	{0xCC,	5,	{0x09, 0x0F, 0x0D, 0x01,
				0x02}},
	{0x00,	1,	{0xB4}},
	{0xCC,	6,	{0x0D, 0x0F, 0x09, 0x0B,
				0x02, 0x01}},
	{0x00,	1,	{0xCE}},
	{0xCC,	1,	{0x0E}},
	{0x00,	1,	{0xD0}},
	{0xCC,	5,	{0x10, 0x0A, 0x0C, 0x04,
				0x03}},
	{0x00,	1,	{0x00}},
	{0xD8,	2,	{0x75,0x75}},
	{0x00,	1,	{0x00}},
	{0xD9,	1,	{0x52}},
	{0x00,	1,	{0x00}},
	{0xE1,	16,	{0x00,0x02,0x08,0x0E,0x08,0x1D,0x0E,0x0E,0x00,0x04,0x02,0x07,0x0E,0x23,0x20,0x14}},
	{0x00,	1,	{0x00}},
	{0xE2,	16,	{0x00,0x02,0x08,0x0E,0x08,0x1D,0x0E,0x0E,0x00,0x04,0x02,0x07,0x0E,0x24,0x20,0x14}},

	{0x00,	1,	{0x00}},
	{0xEC,	33,	{0x40,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x00}},

	{0x00,	1,	{0x00}},
	{0xED,	33,	{0x40,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x00}},

	{0x00,	1,	{0x00}},
	{0xEE,	33,	{0x30,0x34,0x34,0x43,0x43,0x33,0x34,0x43,0x43,0x33,0x34,0x34,0x43,0x43,0x33,0x34,0x34,0x43,0x33,0x34,0x34,0x43,0x43,0x33,0x34,0x34,0x43,0x33,0x34,0x34,0x43,0x43,0x03}},
	
	{0x3A,	1,	{0x77}},
	{0x00,	1,	{0x00}},
	{0xff,	3,	{0xff,0xff,0xff}},
	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 200, {}},
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 50, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		/*shaohui add for LCM device regist*/

#ifdef SLT_DEVINFO_LCM
	params->module="BT045TN06V";
	params->vendor="BTL";
	params->ic="otm8018b";
	params->info="480*854";
#endif	
        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 0x05;// 3    2
		params->dsi.vertical_backporch					= 40;// 20   1
		params->dsi.vertical_frontporch					= 40; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 0x0B;// 50  2
		params->dsi.horizontal_backporch				= 80;
		params->dsi.horizontal_frontporch				= 80;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_select=1;
		//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
		params->dsi.PLL_CLOCK=250;
		//params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
#if (LCM_DSI_CMD_MODE)
		//params->dsi.fbk_div =9;
#else
		//params->dsi.fbk_div =9;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
//	SET_RESET_PIN(1);
//	SET_RESET_PIN(0);
//	MDELAY(10);
//	SET_RESET_PIN(1);
//	MDELAY(20);
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
//	lcm_init();
	
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

/*
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

*/


static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;

	#if defined(BUILD_LK)
		//printf("OTM8018B uboot %s \n", __func__);
		//printf("%s id = 0x%08x \n", __func__, id);
	#else
		//printk("OTM8018B kernel %s \n", __func__);
		//printk("%s id = 0x%08x \n", __func__, id);
	#endif

	return (LCM_ID_OTM8018B == id)?1:0;
}



static unsigned int lcm_esd_check(void)
{
	unsigned int ret=FALSE;


  #ifndef BUILD_LK
	char  buffer[6];
	int   array[4];

	array[0] = 0x00083700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 2);
//	printk(" otm 8018b esd buffer0 =%x,buffer1 =%x  \n",buffer[0],buffer[1]);
	//read_reg_v2(0x09,buffer,5);
	//printk(" esd buffer0=%x, buffer1 =%x buffer2=%x,buffer3=%x,buffer4=%x \n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
#if 1
	if(buffer[0]==0x9C)
	{
		ret=FALSE;
	}
	else
	{			 
		ret=TRUE;
	}
#endif
 #endif
 return ret;

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	
	#ifndef BUILD_LK
	printk("lcm_esd_recover  otm8018B_video_dsi \n");
	#endif
	return TRUE;
}


LCM_DRIVER otm8018b_dsi_vdo_lcm_drv = 
{
    .name			= "otm8018b_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

