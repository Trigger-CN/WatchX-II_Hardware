/*
 *
 * FocalTech ftxxxx TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_FTXXXX_H__
#define __LINUX_FTXXXX_H__
 /*******************************************************************************
*
* File Name: Ftxxxx_ts.h
*
*    Author: Xu YongFeng
*
*   Created: 2015-01-29
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/




#include "tpd.h"
//#include "tpd_custom_fts.h"
#include "cust_gpio_usage.h"
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <asm/unistd.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/irqs.h>
#include <cust_eint.h>
#include <linux/jiffies.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
#endif
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/mount.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <../fs/proc/internal.h>
/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/

/**********************Custom define begin**********************************************/


#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0))
	#if defined(MODULE) || defined(CONFIG_HOTPLUG)
		#define __devexit_p(x) x
	#else
		#define __devexit_p(x) NULL
	#endif
	// Used for HOTPLUG
	#define __devinit        __section(.devinit.text) __cold notrace
	#define __devinitdata    __section(.devinit.data)
	#define __devinitconst   __section(.devinit.rodata)
	#define __devexit        __section(.devexit.text) __exitused __cold notrace
	#define __devexitdata    __section(.devexit.data)
	#define __devexitconst   __section(.devexit.rodata)
#endif


#define TPD_POWER_SOURCE_CUSTOM         	MT6323_POWER_LDO_VGP1
#define IIC_PORT                   					0				//MT6572: 1  MT6589:0 , Based on the I2C index you choose for TPM
#define TPD_HAVE_BUTTON									// if have virtual key,need define the MACRO
#define TPD_BUTTON_HEIGH        				(40)  			//100
#define TPD_KEY_COUNT           				3    				//  4
#define TPD_KEYS                					{ KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM            					{{80,900,20,TPD_BUTTON_HEIGH}, {240,900,20,TPD_BUTTON_HEIGH}, {400,900,20,TPD_BUTTON_HEIGH}}

/*********************Custom Define end*************************************************/
#define MT_PROTOCOL_B
#define TPD_NAME    							"FTS"
/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE         
#define TPD_I2C_NUMBER           				0
#define TPD_WAKEUP_TRIAL         				60
#define TPD_WAKEUP_DELAY         				100
#define TPD_VELOCITY_CUSTOM_X 				15
#define TPD_VELOCITY_CUSTOM_Y 				20

#define CFG_MAX_TOUCH_POINTS				5
#define MT_MAX_TOUCH_POINTS				10
#define FTS_MAX_ID							0x0F
#define FTS_TOUCH_STEP						6
#define FTS_FACE_DETECT_POS				1
#define FTS_TOUCH_X_H_POS					3
#define FTS_TOUCH_X_L_POS					4
#define FTS_TOUCH_Y_H_POS					5
#define FTS_TOUCH_Y_L_POS					6
#define FTS_TOUCH_EVENT_POS				3
#define FTS_TOUCH_ID_POS					5
#define FT_TOUCH_POINT_NUM				2
#define FTS_TOUCH_XY_POS					7
#define FTS_TOUCH_MISC						8
#define POINT_READ_BUF						(3 + FTS_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

#define	FT_FW_NAME_MAX_LEN	50
#define TPD_DELAY                					(2*HZ/100)
#define TPD_RES_X                					480
#define TPD_RES_Y                					800
#define TPD_CALIBRATION_MATRIX  			{962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
//#define TPD_HAVE_TREMBLE_ELIMINATION
//#define TPD_CLOSE_POWER_IN_SLEEP
/******************************************************************************/
/*Chip Device Type*/
#define IC_FT5X06							0				/*x=2,3,4*/
#define IC_FT5606							1				/*ft5506/FT5606/FT5816*/
#define IC_FT5316							2				/*ft5x16*/
#define IC_FT6208							3	  			/*ft6208*/
#define IC_FT6x06     							4				/*ft6206/FT6306*/
#define IC_FT5x06i     						5				/*ft5306i*/
#define IC_FT5x36     							6				/*ft5336/ft5436/FT5436i*/



/*register address*/
#define FTS_REG_CHIP_ID						0xA3    			//chip ID 
#define FTS_REG_FW_VER						0xA6   			//FW  version 
#define FTS_REG_VENDOR_ID					0xA8   			// TP vendor ID 
#define FTS_REG_POINT_RATE					0x88   			//report rate	


#define TPD_MAX_POINTS_2                        		2
#define TPD_MAX_POINTS_5                        		5
#define TPD_MAXPOINTS_10                        		10
#define AUTO_CLB_NEED                           		1
#define AUTO_CLB_NONEED                         		0


#define FTS_PACKET_LENGTH        				128
#define FTS_DRV_VERSION	              		"drv: MTK_2.3_20150122  \n"


#define FTS_GESTRUE_POINTS 				255
#define FTS_GESTRUE_POINTS_ONETIME  		62
#define FTS_GESTRUE_POINTS_HEADER 		8
#define FTS_GESTURE_OUTPUT_ADRESS 		0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 	4

#define KEY_GESTURE_U 						KEY_U
#define KEY_GESTURE_UP 						KEY_UP
#define KEY_GESTURE_DOWN 					KEY_DOWN
#define KEY_GESTURE_LEFT 					KEY_LEFT 
#define KEY_GESTURE_RIGHT 					KEY_RIGHT
#define KEY_GESTURE_O 						KEY_O
#define KEY_GESTURE_E 						KEY_E
#define KEY_GESTURE_M 						KEY_M 
#define KEY_GESTURE_L 						KEY_L
#define KEY_GESTURE_W 						KEY_W
#define KEY_GESTURE_S 						KEY_S 
#define KEY_GESTURE_V 						KEY_V
#define KEY_GESTURE_Z 						KEY_Z

#define GESTURE_LEFT						0x20
#define GESTURE_RIGHT						0x21
#define GESTURE_UP		    					0x22
#define GESTURE_DOWN						0x23
#define GESTURE_DOUBLECLICK				0x24
#define GESTURE_O		    					0x30
#define GESTURE_W		    					0x31
#define GESTURE_M		    					0x32
#define GESTURE_E		    					0x33
#define GESTURE_L		    					0x44
#define GESTURE_S		    					0x46
#define GESTURE_V		    					0x54
#define GESTURE_Z		    					0x41
/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
/* IC info */
struct fts_Upgrade_Info 
{
        u8 CHIP_ID;
        u8 TPD_MAX_POINTS;
        u8 AUTO_CLB;
	 u16 delay_aa;						/*delay of write FT_UPGRADE_AA */
	 u16 delay_55;						/*delay of write FT_UPGRADE_55 */
	 u8 upgrade_id_1;					/*upgrade id 1 */
	 u8 upgrade_id_2;					/*upgrade id 2 */
	 u16 delay_readid;					/*delay of read id */
	 u16 delay_earse_flash; 				/*delay of earse flash*/
};

/*touch event info*/
struct ts_event 
{
	u16 au16_x[CFG_MAX_TOUCH_POINTS];				/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];				/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];		/*touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];			/*touch ID */
	u16 pressure[CFG_MAX_TOUCH_POINTS];
	u16 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
	int touchs;
	u8 touch_point_num;
};
struct fts_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	const struct ftxxxx_ts_platform_data *pdata;
	struct work_struct 	touch_event_work;
	struct workqueue_struct *ts_workqueue;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
};
/*******************************************************************************
* Static variables
*******************************************************************************/




/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
//Function Switchs: define to open,  comment to close
#define FTS_GESTRUE_EN 0
#define FTS_APK_DEBUG
#define MTK_EN 1
//#define CONFIG_TOUCHPANEL_PROXIMITY_SENSOR


extern struct i2c_client *fts_i2c_client;
extern struct input_dev *fts_input_dev;
extern struct tpd_device *tpd;
//Getstre functions

extern struct fts_Upgrade_Info fts_updateinfo_curr;
int fts_rw_iic_drv_init(struct i2c_client *client);
void  fts_rw_iic_drv_exit(void);
void fts_get_upgrade_array(void);
#if FTS_GESTRUE_EN
		extern int fts_Gesture_init(struct input_dev *input_dev);
		extern int fts_read_Gestruedata(void);
#endif
extern int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue);
extern int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
extern int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern int HidI2c_To_StdI2c(struct i2c_client * client);
extern int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name);
extern int fts_ctpm_auto_clb(struct i2c_client *client);
extern int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
extern int fts_ctpm_get_i_file_ver(void);
extern int fts_remove_sysfs(struct i2c_client *client);
extern void fts_release_apk_debug_channel(void);
extern int fts_ctpm_auto_upgrade(struct i2c_client *client);

extern void fts_reset_tp(int HighOrLow);
extern int fts_create_sysfs(struct i2c_client *client);
//Apk and ADB functions
extern int fts_create_apk_debug_channel(struct i2c_client * client);
/*
static DEFINE_MUTEX(i2c_rw_access);



//upgrade functions
extern void ftxxxx_update_fw_vendor_id(struct ftxxxx_ts_data *data);
extern void ftxxxx_update_fw_ver(struct ftxxxx_ts_data *data);
extern void focaltech_get_upgrade_array(void);
extern int ftxxxx_fw_upgrade(struct device *dev, bool force);



//Base functions
extern int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern int ftxxxx_read_reg(struct i2c_client *client, u8 addr, u8 *val);
extern int ftxxxx_write_reg(struct i2c_client *client, u8 addr, const u8 val);

//Power functions
extern int ftxxxx_ts_suspend(struct device *dev);
extern int ftxxxx_ts_resume(struct device *dev);

*/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
/*
#define FTS_DBG
#ifdef FTS_DBG
#define FTS_DBG(fmt, args...) 				printk("[FTS]" fmt, ## args)
#else
#define FTS_DBG(fmt, args...) 				do{}while(0)
#endif
*/
#define FTS_DBG
#ifdef FTS_DBG
	#define FTS_DBG(fmt, args...) 				printk("[FTS]" fmt, ## args)
#else
	#define FTS_DBG(fmt, args...) 				do{}while(0)
#endif
#endif
