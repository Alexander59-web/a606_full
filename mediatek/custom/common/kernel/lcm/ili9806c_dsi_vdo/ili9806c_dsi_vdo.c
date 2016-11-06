#ifdef BUILD_LK
#include "platform/mt_gpio.h"
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

#define LCM_DSI_CMD_MODE									0
#define LCM_ID_ILI9806    0x9816

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

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

   {0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},// EXTC Command Set enable register
    {0x08,1,{0x10}},// SPI Interface Setting
    {0x21,1,{0x01}},// Interface Mode Control
    {0x30,1,{0x01}},
    {0x31,1,{0x02}},
    {0x40,1,{0x13}},   
    {0x41,1,{0x33}},  
    {0x42,1,{0x02}},   
    {0x43,1,{0x09}},  
    {0x44,1,{0x09}},
    
    {0x50,1,{0x78}},
    {0x51,1,{0x78}},
    {0x52,1,{0x00}},
    {0x53,1,{0x66}},
    {0x57,1,{0x50}},
    
    {0x60,1,{0x07}},
    {0x61,1,{0x00}},
    {0x62,1,{0x08}},
    {0x63,1,{0x00}},

    {0xa0,1,{0x00}},
    {0xa1,1,{0x05}},	
    {0xa2,1,{0x0f}},
    {0xa3,1,{0x11}},
    {0xa4,1,{0x0e}},
    {0xa5,1,{0x1a}},	
    {0xa6,1,{0x07}},
    {0xa7,1,{0x0d}},
    {0xa8,1,{0x04}},
    {0xa9,1,{0x0c}},	
    {0xaa,1,{0x04}},
    {0xab,1,{0x06}},
    {0xac,1,{0x10}},
    {0xad,1,{0x32}},	
    {0xae,1,{0x28}},
    {0xaf,1,{0x00}},

   {0xc0,1,{0x00}},
   {0xc1,1,{0x06}},
   {0xc2,1,{0x10}},
   {0xc3,1,{0x10}},
   {0xc4,1,{0x07}},
   {0xc5,1,{0x13}},
   {0xc6,1,{0x09}},
   {0xc7,1,{0x06}},
   {0xc8,1,{0x05}},
   {0xc9,1,{0x08}},
   {0xca,1,{0x09}},
   {0xcb,1,{0x06}},
   {0xcc,1,{0x0b}},
   {0xcd,1,{0x22}},
   {0xce,1,{0x24}},
   {0xcf,1,{0x00}},

   {0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},

   {0x00,1,{0x21}},
   {0x01,1,{0x06}},
   {0x02,1,{0x00}},
   {0x03,1,{0x00}},
   {0x04,1,{0x01}},
   {0x05,1,{0x01}},
   {0x06,1,{0x80}},
   {0x07,1,{0x02}},
   {0x08,1,{0x05}},
   {0x09,1,{0x00}},
   {0x0a,1,{0x00}},
   {0x0b,1,{0x00}},
   {0x0c,1,{0x01}},
   {0x0d,1,{0x01}},
   {0x0e,1,{0x00}},
   {0x0f,1,{0x00}},
   {0x10,1,{0xf0}},
   {0x11,1,{0xf4}},
   {0x12,1,{0x00}},
   {0x13,1,{0x00}},
   {0x14,1,{0x00}},
   {0x15,1,{0xc0}},
   {0x16,1,{0x08}},
   {0x17,1,{0x00}},
   {0x18,1,{0x00}},
   {0x19,1,{0x00}},
   {0x1a,1,{0x00}},
   {0x1b,1,{0x00}},
   {0x1c,1,{0x00}},
   {0x1d,1,{0x00}},

   {0x20,1,{0x02}},
   {0x21,1,{0x13}},
   {0x22,1,{0x45}},
   {0x23,1,{0x67}},
   {0x24,1,{0x01}},
   {0x25,1,{0x23}},
   {0x26,1,{0x45}},
   {0x27,1,{0x67}},
   
{0x30,1,{0x13}},
   {0x31,1,{0x22}},
   {0x32,1,{0x22}},
   {0x33,1,{0x22}},
   {0x34,1,{0x22}},
   {0x35,1,{0xbb}},
   {0x36,1,{0xaa}},
   {0x37,1,{0xdd}},
   {0x38,1,{0xcc}},
   {0x39,1,{0x66}},
   {0x3a,1,{0x77}},
   {0x3b,1,{0x22}},
   {0x3c,1,{0x22}},
   {0x3d,1,{0x22}},
   {0x3e,1,{0x22}},
   {0x3f,1,{0x22}},
   {0x40,1,{0x22}},

   {0x52,1,{0x10}},
   {0x53,1,{0x10}},

   {0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},
  {0x17,1,{0x22}},
  {0x02,1,{0x77}},
  {0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},


    {0x11,1,{0x00}},
    {REGFLAG_DELAY, 120, {}}, 
    {0x29,1,{0x00}},
        {REGFLAG_DELAY, 10, {}}, 
    //{REGFLAG_DELAY, 120, {}}, 
    //{0x2C,1,{0x00}},//GRAM start writing
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};





static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/
	static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
		// Display off sequence
		{0x28, 1, {0x00}},
		{REGFLAG_DELAY, 50, {}},
	
		// Sleep Mode On
		{0x10, 1, {0x00}},
		{REGFLAG_DELAY, 200, {}},
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
   #if 1
   memset(params, 0, sizeof(LCM_PARAMS));
    
    params->type   = LCM_TYPE_DSI;
    
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    
    // enable tearing-free
    params->dbi.te_mode				= LCM_DBI_TE_MODE_DISABLED;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   =SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE ;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting		
    params->dsi.packet_size=256;
    params->dsi.intermediat_buffer_num = 0;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=854;

    params->dsi.vertical_sync_active				= 6;
    params->dsi.vertical_backporch					= 14;
    params->dsi.vertical_frontporch					= 20;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active				= 60;
    params->dsi.horizontal_backporch				= 50;
    params->dsi.horizontal_frontporch				= 60;
    params->dsi.horizontal_blanking_pixel			= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    
    // Bit rate calculation
#if 0//def CONFIG_MT6589_FPGA
		params->dsi.pll_div1=2; 	// div1=0,1,2,3;div1_real=1,2,4,4
		params->dsi.pll_div2=2; 	// div2=0,1,2,3;div1_real=1,2,4,4
		params->dsi.fbk_sel=0;		// fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
		params->dsi.fbk_div =8; 	// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else
		params->dsi.pll_div1=0; 	// div1=0,1,2,3;div1_real=1,2,4,4
		params->dsi.pll_div2=2; 	// div2=0,1,2,3;div2_real=1,2,4,4
		//params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
		params->dsi.fbk_div =28;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
#endif
	
#endif
#if 0	
			memset(params, 0, sizeof(LCM_PARAMS));
	
			params->type   = LCM_TYPE_DSI;
	
			params->width  = FRAME_WIDTH;
			params->height = FRAME_HEIGHT;
	
			// enable tearing-free
		//	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
	
#if (LCM_DSI_CMD_MODE)
			params->dsi.mode   = CMD_MODE;
#else
			params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
		
			// DSI
			/* Command mode setting */
			params->dsi.LANE_NUM				= LCM_TWO_LANE;
			//The following defined the fomat for data coming from LCD engine.
			params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
			params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
			params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
			params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;
	
			// Highly depends on LCD driver capability.
			// Not support in MT6573
			params->dsi.packet_size=256;
	
			// Video mode setting		
			params->dsi.intermediat_buffer_num = 2;
	
			params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
			//params->dsi.word_count=480*3; 
	
			params->dsi.vertical_sync_active				= 4;
			params->dsi.vertical_backporch					= 8;
			params->dsi.vertical_frontporch 				= 8;
			params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	
			params->dsi.horizontal_sync_active				= 6;//6
			params->dsi.horizontal_backporch				= 35;//37
			params->dsi.horizontal_frontporch				= 35;//37
			params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;
	
			// Bit rate calculation
			//params->dsi.pll_div1=29;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
			//params->dsi.pll_div2=1;		// div2=0~15: fout=fvo/(2*div2)
			params->dsi.pll_div1=1; 	// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
			params->dsi.pll_div2=1; 	// div2=0,1,2,3;div1_real=1,2,4,4	
			params->dsi.fbk_div =20;	// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif
	}




static void lcm_init(void)
{
    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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
    read_reg_v2(0xD3, buffer, 5);

    id_high = buffer[1];
    id_low = buffer[2];
    id = (id_high<<8) | id_low;

    return (LCM_ID_ILI9806 == id)?1:0;

}

static unsigned int lcm_esd_check(void)
{
 // #ifndef BUILD_LK
	char  buffer[4];
	int   array[4];
	int rega =0,regb=0,regc=0,rege=0;
	int reg9[4] = {0};
	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0a, buffer, 1);
	rega =buffer[0];
	
	
	
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0b, buffer, 1);
	regb =buffer[0];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0e, buffer, 1);
	rege =buffer[0];

		array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0c, buffer, 1);
	regc =buffer[0];

	array[0] = 0x00043700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x09, buffer, 4);
	reg9[0] =buffer[0];
	reg9[1] =buffer[1];
	reg9[2] =buffer[2];
	reg9[3] =buffer[3];
	if((rega==0x9c)&&(regb==0x00)&&(regc==0x70)&&(rege==0x00)&&(buffer[0]==0x80)&&(buffer[1]==0x73)&&(buffer[2]==0x04)&&(buffer[3]==0x00))
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
//#endif
}
static unsigned int lcm_esd_recover(void)
{
	lcm_init();


	return TRUE;
}



LCM_DRIVER ili9806_dsi_vdo_lcm_drv = 
{
    .name			= "ili9806_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
	//.esd_check      = lcm_esd_check,
    //.esd_recover    = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
    //.update         = lcm_update,
#endif
};

