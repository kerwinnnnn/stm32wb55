#include"lsm6dsv16b_reg.h"
#include"IMU.h"
#include"string.h"
#include"stdio.h"
void lsm6dsv16b_pedometer(void)
{
		uint32_t  step_count=0;
		stmdev_ctx_t dev_ctx;
		IMU_init();
		dev_ctx=IMU_get_ctx();
	    lsm6dsv16b_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	    lsm6dsv16b_xl_full_scale_set(&dev_ctx, LSM6DSV16B_2g);
	    lsm6dsv16b_xl_data_rate_set(&dev_ctx, LSM6DSV16B_XL_ODR_AT_30Hz);

	    lsm6dsv16b_stpcnt_mode_t pedometer_mode = {
	        .false_step_rej = 1,
	        .step_counter_enable = 1
	    };
	    lsm6dsv16b_stpcnt_mode_set(&dev_ctx, pedometer_mode);

	    lsm6dsv16b_stpcnt_rst_step_set(&dev_ctx, 1);


	    while (1) {

	       if(lsm6dsv16b_stpcnt_steps_get(&dev_ctx, &step_count)!=0)
	       {
	    	   printf("get steps error");
	       }
	        uint8_t tx_buffer[100];
	        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Steps: %u\r\n", step_count);
	        tx_com(tx_buffer, strlen((char const *)tx_buffer));


	        dev_ctx.mdelay(1000);
	    }

}


