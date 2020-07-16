docs: document

src: code

	Note:
		the code just provide for MTK xx82 platform

		android 5.0:
				add #include <../fs/proc/internal.h> 
		gesture:
			when gesture function need be complied, please put 32 or 64 bit ft_gesture_lib.a_shipped in the src folder 
		esd:
			set "A3_REG_VALUE" value for diff ic


log:
version: 1.0

version: 2.1
		add driver version
version: 2.2
		add 5826 ic and auto upgrade
version: 2.3
		modify auto upgrade 0xa6
version: 2.4
		modify i2c_smbus_read/write_i2c_block_data,delete i2c_master_send() in fts_i2c_Write()
version: 2.4
		add ic type in doc
version: 2.5
		modify input_set_abs_params in tpd_local_init
		delete input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS);
		modify i2c_register_board_info(IIC_PORT, &fts_i2c_tpd, 1);
version: 2.6
		modify reset low -> power -> reset high	
		
		interrupt,resume add:
		#ifdef GTP_ESD_PROTECT
			count_irq ++;
	 	#endif
	 	modify A3_REG_VALUE, fts_write_reg(i2c_client, 0x8F,data);
version: 2.7
		#include <../fs/proc/internal.h> for android 5.0
		add i2c_transfer style for read/write i2c

		protect report press and area are 0

		modify para in fts_write_reg 

		add i2c ecc address:0xcc for 5x46 upgrade
version 3.0
		new structure

		add uppoint++ in reprot functiion and solve the final finger report missing issue

		//release all touches in final
version 3.1
		modify DMA=128