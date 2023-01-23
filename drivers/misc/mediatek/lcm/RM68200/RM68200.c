/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include "lcm_drv.h"
#include "ddp_irq.h"

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif

static struct regulator *lcm_vgp = NULL;
static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_RST_EN;


#define LCM_ID 0x93


#if 1
/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo = NULL;

	pr_debug("LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = regulator_get(dev, "reg-lcm-vibr");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm-vibr LDO, %d\n", ret);
		return ret;
	}
	lcm_vgp = lcm_vgp_ldo;
	pr_debug("LCM: lcm get supply ok.\n");
	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_err("lcm LDO voltage = %d in LK stage\n", ret);

	return ret;
}

static int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_debug("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp)
		return 0;

	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 2.8V\n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 3300000, 3300000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 3300000)
		pr_err("LCM: check regulator voltage=3300000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=3300000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

static int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_debug("LCM: lcm query regulator enable status[0x%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}

	return ret;
}
#endif
void lcm_get_gpio_infor_a(void)
{
	static struct device_node *node;

        printk("RM68200 lcm_get_gpio_infor\n");
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6580-lcm");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "gpio_lcm_pwr", 0);
	GPIO_LCD_RST_EN = of_get_named_gpio(node, "gpio_lcm_rst", 0);
        //printk("LQ --->  GPIO_LCD_PWR_EN = %d\n",GPIO_LCD_PWR_EN);
        //printk("LQ ---> GPIO_LCD_RST_EN = %d\n",GPIO_LCD_RST_EN);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
        printk("lcm_set_gpio_output\n");
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

static int lcm_probe(struct device *dev)
{
        printk("RM68200 lcm_probe \n");
	lcm_get_vgp_supply(dev);
	lcm_get_gpio_infor_a();
	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,mt6580-lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "mtk_lcm",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	printk("LCM:RM68200 Register lcm driver\n");
	if(strcmp(emdoor_lcm_name, "RM68200")==0){
		if (platform_driver_register(&lcm_driver)) {
			pr_err("LCM: failed to register disp driver\n");
			return -ENODEV;
		}
		printk("LCM: RM68200 Register lcm driver success\n");
	}
	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_debug("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");


/**
 * Local Constants
 */
#define FRAME_WIDTH		(800)
#define FRAME_HEIGHT		(1280)

#define REGFLAG_DELAY		0XFFE//0xFE
#define REGFLAG_END_OF_TABLE	0xFFF//0xFF   /* END OF REGISTERS MARKER */

#define LCM_DSI_CMD_MODE	0

/**
 * Local Variables
 */
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


/**
 * Local Functions
 */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)				lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg				lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#ifndef ASSERT
#define ASSERT(expr)					\
	do {						\
		if (expr)				\
			break;				\
		pr_debug("DDP ASSERT FAILED %s, %d\n",	\
		       __FILE__, __LINE__);		\
		BUG();					\
	} while (0)
#endif

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

/**
 * Note :
 *
 * Data ID will depends on the following rule.
 *
 * count of parameters > 1	=> Data ID = 0x39
 * count of parameters = 1	=> Data ID = 0x15
 * count of parameters = 0	=> Data ID = 0x05
 *
 * Structure Format :
 *
 * {DCS command, count of parameters, {parameter list}}
 * {REGFLAG_DELAY, milliseconds of time, {} },
 * ...
 *
 * Setting ending by predefined flag
 *
 * {REGFLAG_END_OF_TABLE, 0x00, {}}
 */
#if 1
static struct LCM_setting_table lcm_initialization_setting[] = {
#if 0
	{0xFF,4,{0xAA,0x55,0x25,0x01}},
	{0xFC,1,{0x08}},
	{0xFC,1,{0x00}}, 		
	{0x6F,1,{0x21}},
	{0xF7,1,{0x01}},
	{0x6F,1,{0x21}},
	{0xF7,1,{0x00}},		
	{0x6F,1,{0x1A}},
	{0xF7,1,{0x05}},
	{0xFF,4,{0xAA,0x55,0x25,0x00}},								
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},							
	{0xB1,2,{0x68,0x01}},										
	{0xB6,1,{0x08}},									
	{0x6F,1,{0x02}},					
	{0xB8,1,{0x08}},									
	{0xBB,2,{0x54,0x44}},						
	{0xBC,2,{0x05,0x05}},										
	{0xC7,1,{0x01}},										
	{0xBD,5,{0x02,0xB0,0x1E,0x1E,0x00}},									
	{0xC5,2,{0x01,0x07}},								
	{0xC8,1,{0x83}},					
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},								
	{0xB0,2,{0x05,0x05}},				
	{0xB1,2,{0x05,0x05}},										
	{0xBC,2,{0x90,0x01}},					
	{0xBD,2,{0x90,0x01}},									
	{0xCA,1,{0x00}},									
	{0xC0,1,{0x04}},									
	{0xBE,1,{0x29}},					 				
	{0xB3,2,{0x28,0x28}},					
	{0xB4,2,{0x12,0x12}},									
	{0xB9,2,{0x34,0x34}},					
	{0xBA,2,{0x24,0x24}},									
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},				
	{0xEE,1,{0x01}},					 
	{0xEF,4,{0x09,0x06,0x15,0x18}},					
	{0xB0,6,{0x00,0x00,0x00,0x19,0x00,0x35}},					
	{0x6F,1,{0x06}},					
	{0xB0,6,{0x00,0x48,0x00,0x59,0x00,0x77}},					
	{0x6F,1,{0x0C}},					
	{0xB0,4,{0x00,0x9A,0x00,0xCF}},                                                                                                                                      					// postive Red {cont'd)
	{0xB1,6,{0x00,0xFC,0x01,0x43,0x01,0x7C}},					
	{0x6F,1,{0x06}},					
	{0xB1,6,{0x01,0xD2,0x02,0x1A,0x02,0x1C}},					
	{0x6F,1,{0x0C}},					
	{0xB1,4,{0x02,0x5F,0x02,0xA9}},					
	{0xB2,6,{0x02,0xD9,0x03,0x1C,0x03,0x4B}},					
	{0x6F,1,{0x06}},				
	{0xB2,6,{0x03,0x86,0x03,0x9E,0x03,0xBF}},					
	{0x6F,1,{0x0C}},					
	{0xB2,4,{0x03,0xD3,0x03,0xEA}},					
	{0xB3,4,{0x03,0xFD,0x03,0xF3}},  									
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},					
	{0xB0,2,{0x0B,0x2E}},					
	{0xB1,2,{0x2E,0x2E}},					
	{0xB2,2,{0x2E,0x09}},					
	{0xB3,2,{0x2A,0x29}},					
	{0xB4,2,{0x1B,0x19}},					
	{0xB5,2,{0x17,0x15}},					
	{0xB6,2,{0x13,0x11}},					
	{0xB7,2,{0x01,0x2E}},					
	{0xB8,2,{0x2E,0x2E}},					
	{0xB9,2,{0x2E,0x2E}},					
	{0xBA,2,{0x2E,0x2E}},					
	{0xBB,2,{0x2E,0x2E}},					
	{0xBC,2,{0x2E,0x00}},					
	{0xBD,2,{0x10,0x12}},					
	{0xBE,2,{0x14,0x16}},					
	{0xBF,2,{0x18,0x1A}},					
	{0xC0,2,{0x29,0x2A}},					
	{0xC1,2,{0x08,0x2E}},					
	{0xC2,2,{0x2E,0x2E}},					
	{0xC3,2,{0x2E,0x0A}},					
	{0xE5,2,{0x2E,0x2E}},					
	{0xC4,2,{0x0A,0x2E}},					
	{0xC5,2,{0x2E,0x2E}},					
	{0xC6,2,{0x2E,0x00}},					
	{0xC7,2,{0x2A,0x29}},					
	{0xC8,2,{0x10,0x12}},					
	{0xC9,2,{0x14,0x16}},					
	{0xCA,2,{0x18,0x1A}},					
	{0xCB,2,{0x08,0x2E}},					
	{0xCC,2,{0x2E,0x2E}},					
	{0xCD,2,{0x2E,0x2E}},					
	{0xCE,2,{0x2E,0x2E}},					
	{0xCF,2,{0x2E,0x2E}},					
	{0xD0,2,{0x2E,0x09}},					
	{0xD1,2,{0x1B,0x19}},					
	{0xD2,2,{0x17,0x15}},					
	{0xD3,2,{0x13,0x11}},					
	{0xD4,2,{0x29,0x2A}},					
	{0xD5,2,{0x01,0x2E}},					
	{0xD6,2,{0x2E,0x2E}},					
	{0xD7,2,{0x2E,0x0B}},					
	{0xE6,2,{0x2E,0x2E}},					
	{0xD8,5,{0x00,0x00,0x00,0x00,0x00}},					
	{0xD9,5,{0x00,0x00,0x00,0x00,0x00}},					
	{0xE7,1,{0x00}},									
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},					
	{0xB0,2,{0x20,0x00}},					
	{0xB1,2,{0x20,0x00}},					
	{0xB2,5,{0x05,0x00,0x00,0x00,0x00}},									
	{0xB6,5,{0x05,0x00,0x00,0x00,0x00}},					
	{0xB7,5,{0x05,0x00,0x00,0x00,0x00}},									
	{0xBA,5,{0x57,0x00,0x00,0x00,0x00}},					
	{0xBB,5,{0x57,0x00,0x00,0x00,0x00}},									
	{0xC0,4,{0x00,0x00,0x00,0x00}},					
	{0xC1,4,{0x00,0x00,0x00,0x00}},										
	{0xC4,1,{0x60}},					
	{0xC5,1,{0x40}},								
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},					
	{0xBD,5,{0x03,0x01,0x03,0x03,0x03}},					
	{0xB0,2,{0x17,0x06}},					
	{0xB1,2,{0x17,0x06}},					
	{0xB2,2,{0x17,0x06}},					
	{0xB3,2,{0x17,0x06}},					
	{0xB4,2,{0x17,0x06}},					
	{0xB5,2,{0x17,0x06}},									
	{0xB8,1,{0x00}},					
	{0xB9,1,{0x00}},					
	{0xBA,1,{0x00}},					
	{0xBB,1,{0x02}},					
	{0xBC,1,{0x00}},									
	{0xC0,1,{0x07}},									
	{0xC4,1,{0x80}},					
	{0xC5,1,{0xA4}},									
	{0xC8,2,{0x05,0x30}},					
	{0xC9,2,{0x01,0x31}},									
	{0xCC,3,{0x00,0x00,0x3C}},					
	{0xCD,3,{0x00,0x00,0x3C}},									
	{0xD1,5,{0x00,0x05,0x09,0x07,0x10}},					
	{0xD2,5,{0x00,0x05,0x0E,0x07,0x10}},									
	{0xE5,1,{0x06}},					
	{0xE6,1,{0x06}},					
	{0xE7,1,{0x06}},					
	{0xE8,1,{0x06}},					
	{0xE9,1,{0x06}},					
	{0xEA,1,{0x06}},									
	{0xED,1,{0x30}},								
	{0x35,1,{0x00}},

	{0xFF,4,{0xAA,0x55,0x25,0x01}},//3LANEģʽ
	{0x6F,1,{0x16}},
	{0xF7,1,{0x10}},
	{0xFF,4,{0xAA,0x55,0x25,0x00}},
#endif
	{0xFE, 1, {0x04}},
	{0x62, 1, {0x13}},
	{0x6D, 1, {0x19}},
	{0x65, 1, {0x13}},
	{0x6A, 1, {0x10}},
	{0x61, 1, {0x0D}},
	{0x63, 1, {0x0F}},
	{0x64, 1, {0x07}},
	{0x66, 1, {0x0F}},
	{0x67, 1, {0x0B}},
	{0x68, 1, {0x16}},
	{0x69, 1, {0x0C}},
	{0x6B, 1, {0x0A}},
	{0x6C, 1, {0x11}},
	{0x6E, 1, {0x13}},
	{0x60, 1, {0x01}},
	{0x6F, 1, {0x0B}},
	{0x72, 1, {0x13}},
	{0x7D, 1, {0x19}},
	{0x75, 1, {0x13}},
	{0x7A, 1, {0x10}},
	{0x71, 1, {0x0D}},
	{0x73, 1, {0x0F}},
	{0x74, 1, {0x07}},
	{0x76, 1, {0x0F}},
	{0x77, 1, {0x0B}},
	{0x78, 1, {0x16}},
	{0x79, 1, {0x0C}},
	{0x7B, 1, {0x0A}},
	{0x7C, 1, {0x11}},
	{0x7E, 1, {0x13}},
	{0x70, 1, {0x01}},
	{0x7F, 1, {0x0B}},
	{0xFE, 1, {0x01}},
	{0x5F, 1, {0x12}},
	
	{0xFE, 1, {0x0E}},
	{0x1B, 1, {0x00}},
	{0x1C, 1, {0x00}},
	
	{0xFE, 1, {0x00}},
	{0x11, 1, {0x00}},			// Normal Display	//sleep out
	{REGFLAG_DELAY, 150, {}},
	{0x29, 1, {0x00}},			// Display ON
	{REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
}; 

#endif
#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A, 4, {0x00, 0x00, (FRAME_WIDTH >> 8), (FRAME_WIDTH&0xFF) } },
	{0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT&0xFF)} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	/* Sleep Out */
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },

	/* Display ON */
	{0x29, 0, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Sleep Mode On */
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif
#if 0
static void init_lcm_registers(void)
{
	unsigned int data_array[16];


//add by ricky
data_array[0] = 0x00023902;
data_array[1] = 0x000066D8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00023902;
data_array[1] = 0x00000092;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00133902;
data_array[1] = 0xFC000019;
data_array[2] = 0xFFBFFA35;
data_array[3] = 0x0091BFFB;
data_array[4] = 0xF81C0000;
data_array[5] = 0x00FB011F;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0xFA001019;
data_array[2] = 0x20009061;
data_array[3] = 0x8953554D;
data_array[4] = 0xE57E0757;
data_array[5] = 0x003DFB6A;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0x4B002019;
data_array[2] = 0xDAB936DA;
data_array[3] = 0x5F96CF7E;
data_array[4] = 0xD81D5E25;
data_array[5] = 0x0050A154;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0x20003019;
data_array[2] = 0xBA114840;
data_array[3] = 0x0648E054;
data_array[4] = 0x00000000;
data_array[5] = 0x003F7F0E;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0x84004019;
data_array[2] = 0x14304008;
data_array[3] = 0xAC37FFC2;
data_array[4] = 0x78000004;
data_array[5] = 0x00C00389;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0x00005019;
data_array[2] = 0xE2008062;
data_array[3] = 0x1486C229;
data_array[4] = 0x3C01FB21;
data_array[5] = 0x00641400;
dsi_set_cmdq(data_array, 6, 1);

data_array[0]= 0x00133902;
data_array[1] = 0x64006019;
data_array[2] = 0x1E0A10A0;
data_array[3] = 0x089C361C;
data_array[4] = 0x801E3A01;
data_array[5] = 0x003C0291;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0x86007019;
data_array[2] = 0x4220633F;
data_array[3] = 0x438C86BA;
data_array[4] = 0xFEFEFE18;
data_array[5] = 0x00FEFEFE;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0xFE008019;
data_array[2] = 0x00FEFEFE;
data_array[3] = 0x01010101;
data_array[4] = 0x01010101;
data_array[5] = 0x00E80101;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0x0F009019;
data_array[2] = 0x34000109;
data_array[3] = 0xD16C8020;
data_array[4] = 0x23624220;
data_array[5] = 0x00404040;
dsi_set_cmdq(data_array, 6, 1);

data_array[0]= 0x00133902;
data_array[1] = 0x4000A019;
data_array[2] = 0x40404040;
data_array[3] = 0x01008040;
data_array[4] = 0x01010101;
data_array[5] = 0x00800509;
dsi_set_cmdq(data_array, 6, 1);

data_array[0] = 0x00133902;
data_array[1] = 0x0100B019;
data_array[2] = 0x08040201;
data_array[3] = 0x10101010;
data_array[4] = 0x00000000;
data_array[5] = 0x00000000;
dsi_set_cmdq(data_array, 6, 1);

data_array[0]= 0x00133902;
data_array[1] = 0x0000C019;
data_array[2] = 0x7ABD5EAF;
data_array[3] = 0xFAFAFAF5;
data_array[4] = 0x0000000A;
data_array[5] = 0x00000000;
dsi_set_cmdq(data_array, 6, 1);

data_array[0]= 0x000E3902;
data_array[1] = 0x0000D019;
data_array[2] = 0x00000000;
data_array[3] = 0x00006E64;
data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);


#if 0	
	// SET password
	data_array[0]=0x00043902;
	data_array[1]=0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);
	
	data_array[0]=0x00033902;
	data_array[1]=0x008333BA;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);
	
	data_array[0]=0x00053902;
	data_array[1]=0x7d0000b0;
	data_array[2]=0x0000000c;
	dsi_set_cmdq(data_array, 3, 1);
//	MDELAY(10);	
	
	//Set Power
	data_array[0]=0x00103902;
	data_array[1]=0x15156cB1;
	data_array[2]=0xf1110424;
	data_array[3]=0x2397E480;
	data_array[4]=0x58D2C080;
	dsi_set_cmdq(data_array, 5, 1);
//	MDELAY(10);
	
	// SET CYC 
	data_array[0]=0x000C3902;
	data_array[1]=0x106400B2;
	data_array[2]=0x081C2207;
    data_array[3]=0x004D1C08;	
	dsi_set_cmdq(data_array, 4, 1);
//	MDELAY(10);

    // SET CYC 
	data_array[0]=0x000D3902;
	data_array[1]=0x03FF00B4;
	data_array[2]=0x035A035A;
	data_array[3]=0x306a015A;
	data_array[4]=0x0000006a;
	dsi_set_cmdq(data_array, 5, 1);
//	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x000007BC;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);
	
	data_array[0]=0x00043902;
	data_array[1]=0x010E41BF;
	dsi_set_cmdq(data_array, 2, 1);	
//	MDELAY(10);	
	
	//Set VCOM
	data_array[0]=0x00033902;
    data_array[1]=0x005c5cB6;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);	

	// Set panel 
	data_array[0]=0x00023902;
	data_array[1]=0x000009CC;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);

	// SET GIP 
	data_array[0]=0x001F3902;
	data_array[1]=0x000600D3;
	data_array[2]=0x00080740;
	data_array[3]=0x00071032;
	data_array[4]=0x0F155407;
	data_array[5]=0x12020405;
	data_array[6]=0x33070510;
	data_array[7]=0x370B0B33;   
	data_array[8]=0x00070710;
	dsi_set_cmdq(data_array, 9, 1);
//	MDELAY(10);	
	
	// SET GIP 
	data_array[0]=0x002d3902;
	data_array[1]=0x060504D5;
	data_array[2]=0x02010007;
	data_array[3]=0x22212003;
	data_array[4]=0x18181823;
	data_array[5]=0x18181818;
	data_array[6]=0x18191918;
	data_array[7]=0x1B181818;
	data_array[8]=0x181A1A1B;   
	data_array[9]=0x18181818;
	data_array[10]=0x18181818;
	data_array[11]=0x18181818;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
//	MDELAY(10);	
	
	// SET GIP 
	data_array[0]=0x002D3902;
	data_array[1]=0x010203D6;
	data_array[2]=0x05060700;
	data_array[3]=0x21222304;
	data_array[4]=0x18181820;
	data_array[5]=0x58181818;
	data_array[6]=0x19181858;
	data_array[7]=0x1B181819;   
	data_array[8]=0x181A1A1B;
	data_array[9]=0x18181818;
	data_array[10]=0x18181818;
	data_array[11]=0x18181818;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
//	MDELAY(10);	

	// R Gamma
	data_array[0]=0x002B3902;
	data_array[1]=0x161000E0;
	data_array[2]=0x233F332D;
	data_array[3]=0x0D0B073E;
	data_array[4]=0x14120E17;
	data_array[5]=0x11061312;
	data_array[6]=0x10001813;
	data_array[7]=0x3F332D16;
	data_array[8]=0x0B073E23;   
	data_array[9]=0x120E170D;
	data_array[10]=0x06131214;
	data_array[11]=0x00181311;
	dsi_set_cmdq(data_array, 12, 1);
//	MDELAY(10);
	
	data_array[0]=0x00033902;
    data_array[1]=0x001430C0;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);	
	
	data_array[0]=0x00053902;
	data_array[1]=0x40C000C7;
	data_array[2]=0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);
//	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x00008edf;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x000066d2;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);	
	
	data_array[0]= 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	
	data_array[0]= 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
//	MDELAY(20);
#endif
}

#endif
#if 1
static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;
		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}
	}
}  
#endif
/**
 * LCM Driver Implementations
 */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
        printk("RM68200 lcm_set_util_funcs\n");
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
        printk("RM68200 lcm_get_params\n");
		memset(params, 0, sizeof(LCM_PARAMS));

		params->type = LCM_TYPE_DSI;
		params->width = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

/*#if (LCM_DSI_CMD_MODE)
		params->dsi.mode = CMD_MODE;
		params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
		params->dsi.mode = SYNC_PULSE_VDO_MODE;
#endif */
        params->dsi.mode   = BURST_VDO_MODE;

		/* DSI */
		/* Command mode setting */
		params->dsi.LANE_NUM = LCM_THREE_LANE;//LCM_FOUR_LANE;
		/* The following defined the fomat for data coming from LCD engine. */
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

		/* Highly depends on LCD driver capability. */
		/* Not support in MT6/573 */
		params->dsi.packet_size = 256;
		params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
    	params->dsi.vertical_sync_active				= 4;//4
    	params->dsi.vertical_backporch					= 8;//8
    	params->dsi.vertical_frontporch					= 8;//8
    	params->dsi.vertical_active_line				= FRAME_HEIGHT;

    	params->dsi.horizontal_sync_active				= 4;//4
    	params->dsi.horizontal_backporch				= 32;//132
    	params->dsi.horizontal_frontporch				= 24;//24
    	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		//params->dsi.HS_ZERO=11;
		//params->dsi.HS_PRPR=5;

		params->dsi.ssc_disable = 1;
		params->dsi.PLL_CLOCK = 285;//325-61fps;
	
//	    params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
//		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
//		params->dsi.fbk_div =9;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#if 1
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd= 0x0A;
		params->dsi.lcm_esd_check_table[0].count= 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C; 
#endif
}

static void lcm_init_lcm(void)
{
        
/*	lcm_vgp_supply_enable();
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	SET_RESET_PIN(1);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 0);
	SET_RESET_PIN(0);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	SET_RESET_PIN(1); */
	//unsigned int data_array[16];
	//printk("LQ RM68200 lcm_init_lcm\n");
    //lcm_set_gpio_output(70, 0);
	//MDELAY(20);
	lcm_set_gpio_output(12, 1);
	lcm_vgp_supply_enable();
	MDELAY(35);
	lcm_set_gpio_output(11, 1); //vsp
	lcm_set_gpio_output(10, 1); //vsn

	MDELAY(8);
	#if 1
	lcm_set_gpio_output(70, 1);
	MDELAY(30);
	lcm_set_gpio_output(70, 0);
	MDELAY(30);
	lcm_set_gpio_output(70, 1);	
	MDELAY(50);
	#endif
	push_table(lcm_initialization_setting,sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  	
   //init_lcm_registers();	
}
#if 0
static unsigned int lcm_compare_id(void)
{
#if 0
	unsigned int id;
	unsigned char buffer[2];

	printk("LQ ---> KD34 %s \n",__func__);
	
	lcm_set_gpio_output(70, 0);
	MDELAY(20);
	lcm_set_gpio_output(68, 1);
	//lcm_vgp_supply_enable();
	MDELAY(8);
	lcm_set_gpio_output(70, 1);
	MDELAY(30);
	lcm_set_gpio_output(70, 0);
	MDELAY(30);
	lcm_set_gpio_output(70, 1);	
	MDELAY(50);

	read_reg_v2(0x04, buffer, 2);
	id = buffer[0]; 
	printk("LQ ---> KD34 %s id=0x%x,buffer[1]=0x%x\n",__func__,id,buffer[1]);
	return (LCM_ID == id) ? 1 : 0;
#endif

	int lcd_id1=-1;
	int lcd_id2=-1;
	int gpio_id1=6;
	int gpio_id2=1;
	lcm_set_gpio_output(12, 1);
	lcm_vgp_supply_enable();
	MDELAY(35);	
	lcm_set_gpio_output(11, 1); //vsp
	lcm_set_gpio_output(10, 1); //vsn
	
	
	
	MDELAY(8);
	
	lcm_set_gpio_output(70, 1);
	MDELAY(30);
	lcm_set_gpio_output(70, 0);
	MDELAY(30);
	lcm_set_gpio_output(70, 1);	
	MDELAY(50);

	gpio_direction_input(gpio_id1);
	gpio_direction_input(gpio_id2);
	lcd_id1=gpio_get_value(gpio_id1);
	lcd_id2=gpio_get_value(gpio_id2);
	printk("LQ --> RM68200 %s lcd_id1=%d,lcd_id2=%d\n",__func__,lcd_id1,lcd_id2);

	if((1==lcd_id1)&&(0==lcd_id2)){
		return 1;
	}else{
		return 0;
	}
	
}

#endif
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
{	// Display off sequence	
    {0x28, 0, {0x00}},
	{REGFLAG_DELAY, 100, {}},    // Sleep Mode On	
	{0x10, 0, {0x00}},	 
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_suspend(void)
{
#if 0
    unsigned int data_array[16];

    data_array[0]=0x00023902;
    data_array[1]=0x000066d8;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00133902;
    data_array[1]=0x21003019;
    data_array[2]=0xba114840;
    data_array[3]=0x06488054;
    data_array[4]=0x00000000;
    data_array[5]=0x003f7f0e;
    dsi_set_cmdq(data_array, 6, 1);
    MDELAY(50);
    
    data_array[0]=0x00280502;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]=0x00100502;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

#endif
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    lcm_set_gpio_output(70,0);
	MDELAY(10);
	
	lcm_set_gpio_output(11, 0);
	lcm_set_gpio_output(10, 0);
	MDELAY(35);
	lcm_vgp_supply_disable();
    lcm_set_gpio_output(12, 0);

}

static void lcm_resume(void)
{
	lcm_init_lcm();
}

LCM_DRIVER RM68200_lcm_drv = {
	.name		= "RM68200",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	/*.set_backlight	= lcm_setbacklight,*/
	/* .set_pwm        = lcm_setpwm, */
	/* .get_pwm        = lcm_getpwm, */
	/*.update         = lcm_update, */
#endif
};
