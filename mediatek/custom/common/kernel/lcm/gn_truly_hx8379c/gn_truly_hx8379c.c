#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#ifdef BUILD_LK
#define print printf
#else
#define print printk
#endif

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

#define LCM_ID_HX8379C (0x8379)


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    	(lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, v)) //(lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
static unsigned int lcm_compare_id(void);

#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0


static LCM_setting_table_V3 lcm_initialization_setting[] = {
	
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

{0x39,0xB9,3,	{0xFF,0x83,0x79}},

{0x39,0xB1,20,{0x44,0x18,0x18,0x31,0x51,0x90,0xD0,0xEE,0xD4,0x80,
			   0x38,0x38,0xF8,0x44,0x44,0x42,0x00,0x80,0x30,0x00}},

{0x39,0xB2,9,{0x80,0xFE,0x09,0x0C,0x30,0x50,0x11,0x42,0x1D}},

{0x39,0xB4,13,{0x01,0x28,0x00,0x34,0x00,0x34,0x17,0x3A,0x17,0x3A,0xB0,0x00,0xFF}}, 

{0x15,0xCC,1,	{0x02}}, 

{0x39,0xD3,29,{0x00,0x00,0x00,0x00,0x00,0x06,0x06,0x32,0x10,0x03,
			   0x00,0x03,0x03,0x5F,0x03,0x5F,0x00,0x08,0x00,0x08,
			   0x35,0x33,0x07,0x07,0x37,0x07,0x07,0x37,0x07}}, 


{0x39,0xD5,32,{0x18,0x18,0x19,0x19,0x18,0x18,0x20,0x21,0x24,0x25,
			   0x18,0x18,0x18,0x18,0x00,0x01,0x04,0x05,0x02,0x03,
			   0x06,0x07,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}}, 

{0x39,0xD6,32,{0x18,0x18,0x18,0x18,0x19,0x19,0x25,0x24,0x21,0x20,
			   0x18,0x18,0x18,0x18,0x05,0x04,0x01,0x00,0x03,0x02,
			   0x07,0x06,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}}, 

{0x15,0xD2,1,{0x33}},

{0x39,0xE0,42,{0x00,0x03,0x07,0x12,0x14,0x20,0x2E,0x39,0x06,0x0A,
			   0x12,0x19,0x10,0x14,0x17,0x15,0x16,0x08,0x14,0x14,
			   0x17,0x00,0x03,0x07,0x12,0x14,0x20,0x2E,0x39,0x06,
			   0x0A,0x12,0x19,0x10,0x14,0x17,0x15,0x16,0x08,0x14,
			   0x14,0x17}}, 

{0x39,0xB6,2,{0x38,0x38}}, 

{0x05,0x11,0,{}},		
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 130, {}},
{0x05,0x29,0,{}},
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 50, {}},

};

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

		params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;

	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;


		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;


		params->dsi.vertical_sync_active				= 5;
		params->dsi.vertical_backporch					= 5;
		params->dsi.vertical_frontporch					= 5;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 60;
		params->dsi.horizontal_backporch				= 120;
		params->dsi.horizontal_frontporch				= 100;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		params->dsi.PLL_CLOCK=245;

/*
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4
		params->dsi.pll_div2=2;		// div2=0,1,2,3;div1_real=1,2,4,4
		params->dsi.fbk_div =0x21;	// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		25 wangrw
*/		
 	params->dsi.noncont_clock = TRUE;		
 	params->dsi.noncont_clock_period = 2;

}

static void lcm_init(void)
{
	print("lihl truly_hx8379c lcm_init \n");

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(130);

	dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(LCM_setting_table_V3),1);
		   
}


static void lcm_suspend(void)
{
	unsigned int data_array[2];
	
	print("lihl truly_hx8379c lcm_suspend \n");
	

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(130);  
}
static void lcm_resume(void)
{
	unsigned int data_array[16];
	print("lihl truly_hx8379c lcm_resume \n");
	
	data_array[0]=0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(130);	
	data_array[0]=0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}


extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static int GN_IMM_GetOneChannelValue(int dwChannel, int deCount)
{
	int data[4], i;
	unsigned int ret = 0,ret_value1=0,ret_value2=0;

	i = deCount;
	while(i--){
		IMM_GetOneChannelValue(dwChannel, data, 0);
		ret_value1 += data[0];
		ret_value2 += data[1];
		//print("TCL [GN_IMM_GetOneChannelValue(channel%d)]: ret_temp=%d\n",dwChannel,data[0]*1000+data[1]*10);
	}

	ret = ret_value1*1000/deCount + ret_value2*10/deCount;
	return ret;
}



static unsigned int lcm_compare_id(void)
{
#define COMPARE_ID_V_LEVEL	  500	   //0.5V
#define COMPARE_ID_V_LEVEL_CAB	  250	  // 0.25V
#define COMPARE_ADC_CHANNEL 	1
	
	unsigned int id_vol = 0;

	unsigned int id1 = 0, id2 = 0,id =0;
	unsigned char buffer[2];
	unsigned int data_array[16];


	SET_RESET_PIN(1);
	MDELAY(1);	
	SET_RESET_PIN(0);
	MDELAY(10);	
	SET_RESET_PIN(1);
	MDELAY(120); 

	data_array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	
	read_reg_v2(0xDA, buffer, 2);
	id1 = buffer[1]; 

	buffer[0] = 0;
	buffer[1] = 0;	
	read_reg_v2(0xDB, buffer, 2);
	id2 = buffer[1];

	id = (id1<<8) | id2;

	
       id_vol = GN_IMM_GetOneChannelValue(COMPARE_ADC_CHANNEL, 10);

	print("lihl truly_hx8379c id = 0x%x ;id_vol = %d\n", id,id_vol);

	return TRUE;
	//return (((COMPARE_ID_V_LEVEL-COMPARE_ID_V_LEVEL_CAB)<id_vol )&&(id_vol < (COMPARE_ID_V_LEVEL+COMPARE_ID_V_LEVEL_CAB))&&(id == LCM_ID_HX8379C)) ? 1 : 0;

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
	//printk(" lihl truly esd buffer0 =%x,buffer1 =%x  \n",buffer[0],buffer[1]);
	//read_reg_v2(0x09,buffer,5);
	//printk(" esd buffer0=%x, buffer1 =%x buffer2=%x,buffer3=%x,buffer4=%x \n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);

	if(buffer[0]==0x1C)
	{
		ret=FALSE;
	}
	else
	{			 
		ret=TRUE;
	}
 #endif
 	return ret;
 
}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	
	//#ifndef BUILD_LK
	//print("lihl truly_lcm_esd_recover \n");
	//#endif
	return TRUE;
}




LCM_DRIVER hx8379c_fwvga_dsi_vdo_truly_lcm_drv = 
{
	.name	= "truly_hx8379c_fwvga",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
    };
