
#define SENSOR_BUS hspi2


#define CS_up_Pin     GPIO_PIN_12
#define CS_up_GPIO_Port    GPIOB


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsv16b_reg.h"


#include "stm32wbxx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include"dbg_trace.h"

#include"IMU.h"


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static lsm6dsv16b_filt_settling_mask_t filt_settling_mask;

/* Extern variables ----------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;
/* Private functions ---------------------------------------------------------*/
static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;

//enum GESTURE{
//	  GESTURE_DETECTED_LEFT,
//	  GESTURE_DETECTED_RIGHT,
//	  GESTURE_DETECTED_FORWARD,
//	  GESTURE_DETECTED_BACK,
//	  GESTURE_IDLE
//};

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);

stmdev_ctx_t IMU_get_ctx(void)
{
	stmdev_ctx_t dev_ctx;
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.mdelay = platform_delay;
	dev_ctx.handle = &SENSOR_BUS;
	return dev_ctx;
}

void IMU_init(void)
{
	stmdev_ctx_t dev_ctx;
	lsm6dsv16b_reset_t rst;
	uint8_t whoamI;
	dev_ctx=IMU_get_ctx();
	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	  /* Check device ID */
	lsm6dsv16b_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != LSM6DSV16B_ID)
	while (1)
	  {
	    printf("who am i is error!");
	  }

	  /* Restore default configuration */
	lsm6dsv16b_reset_set(&dev_ctx, LSM6DSV16B_RESTORE_CTRL_REGS);
	do {
	    lsm6dsv16b_reset_get(&dev_ctx, &rst);
	    } while (rst != LSM6DSV16B_READY);
}
/* Main Example --------------------------------------------------------------*/
void lsm6dsv16b_fifo(void)
{
  lsm6dsv16b_fifo_status_t fifo_status;
  stmdev_ctx_t dev_ctx;
  dev_ctx=IMU_get_ctx();

  /* Uncomment to configure INT 1 */
  //lsm6dsv16b_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  //lsm6dsv16b_pin_int2_route_t int2_route;

  /* Initialize mems driver interface */

  IMU_init();
  /* Enable Block Data Update */
  lsm6dsv16b_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  lsm6dsv16b_xl_full_scale_set(&dev_ctx, LSM6DSV16B_2g);
  lsm6dsv16b_gy_full_scale_set(&dev_ctx, LSM6DSV16B_2000dps);

  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv16b_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  lsm6dsv16b_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv16b_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV16B_GY_ULTRA_LIGHT);
  lsm6dsv16b_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv16b_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV16B_XL_STRONG);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 10 samples
   */
  lsm6dsv16b_fifo_watermark_set(&dev_ctx, 50);
  lsm6dsv16b_fifo_stop_on_wtm_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv16b_fifo_xl_batch_set(&dev_ctx, LSM6DSV16B_XL_BATCHED_AT_7Hz5);
  lsm6dsv16b_fifo_gy_batch_set(&dev_ctx, LSM6DSV16B_GY_BATCHED_AT_15Hz);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv16b_fifo_mode_set(&dev_ctx, LSM6DSV16B_STREAM_MODE);

  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv16b_xl_data_rate_set(&dev_ctx, LSM6DSV16B_XL_ODR_AT_30Hz);
  lsm6dsv16b_gy_data_rate_set(&dev_ctx, LSM6DSV16B_GY_ODR_AT_60Hz);

  /* Wait samples. */
  while (1) {
    uint16_t num = 0;

    /* Read watermark flag */
    lsm6dsv16b_fifo_status_get(&dev_ctx, &fifo_status);

    if (fifo_status.fifo_th == 1) {
      num = fifo_status.fifo_level;
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "-- FIFO num %d \r\n", num);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));

      while (num--) {
        lsm6dsv16b_fifo_out_raw_t f_data;

        /* Read FIFO tag */
        lsm6dsv16b_fifo_out_raw_get(&dev_ctx, &f_data);
        dataz = (int16_t *)&f_data.data[0]; /* axis are inverted */
        datay = (int16_t *)&f_data.data[2];
        datax = (int16_t *)&f_data.data[4];

        switch (f_data.tag) {
          case LSM6DSV16B_XL_NC_TAG:
            snprintf((char *)tx_buffer, sizeof(tx_buffer), "ACC [mg]:\t%4.2f\t%4.2f\t%4.2f\r\n",
                  lsm6dsv16b_from_fs2_to_mg(*datax),
                  lsm6dsv16b_from_fs2_to_mg(*datay),
                  lsm6dsv16b_from_fs2_to_mg(*dataz));
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          case LSM6DSV16B_GY_NC_TAG:
            snprintf((char *)tx_buffer, sizeof(tx_buffer), "GYR [mdps]:\t%4.2f\t%4.2f\t%4.2f\r\n",
                  lsm6dsv16b_from_fs2000_to_mdps(*datax),
                  lsm6dsv16b_from_fs2000_to_mdps(*datay),
                  lsm6dsv16b_from_fs2000_to_mdps(*dataz));
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          default:
            /* Flush unused samples */
            break;
        }
      }

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "------ \r\n\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}


//      while(1){
//      lsm6dsv16b_fifo_status_get(&dev_ctx, &fifo_status);
//      enum GESTURE gesture_state = GESTURE_IDLE;
//      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//
//      if (fifo_status.fifo_th == 1) {
//    	  	  	  lsm6dsv16b_fifo_out_raw_t f_data;
//    	  	  	  lsm6dsv16b_fifo_out_raw_get(&dev_ctx, (int16_t *)&f_data);
//
//                  dataz = (int16_t *)&f_data.data[0];
//                  datay = (int16_t *)&f_data.data[2];
//                  datax = (int16_t *)&f_data.data[4];
//
//                  float accel_mg[3];
//                  if(f_data.tag==LSM6DSV16B_XL_NC_TAG){
//                  accel_mg[0] = lsm6dsv16b_from_fs2_to_mg(*datax);
//                  accel_mg[1] = lsm6dsv16b_from_fs2_to_mg(*datay);
//                  accel_mg[2] = lsm6dsv16b_from_fs2_to_mg(*dataz);
//                  }
//                  snprintf((char *)tx_buffer, sizeof(tx_buffer), "ACC [mg]: X=%4.2f, Y=%4.2f, Z=%4.2f\r\n",
//      	  	  	  accel_mg[0], accel_mg[1], accel_mg[2]);
//      	  	  	  tx_com(tx_buffer, strlen((char const *)tx_buffer));
//
//                  switch (gesture_state) {
//                      case GESTURE_IDLE:
//                          if (accel_mg[0] <-100) {
//                              gesture_state = GESTURE_DETECTED_LEFT;
//                              snprintf((char *)tx_buffer, sizeof(tx_buffer), "Gesture LEFT detected! \r\n");
//                              tx_com(tx_buffer, strlen((char const *)tx_buffer));
//                              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//                              HAL_Delay(1000);
//                          } else if (accel_mg[0] > 100) {
//                              gesture_state = GESTURE_DETECTED_RIGHT;
//                              snprintf((char *)tx_buffer, sizeof(tx_buffer), "Gesture RIGHT detected! \r\n");
//                              tx_com(tx_buffer, strlen((char const *)tx_buffer));
//                              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//                              HAL_Delay(1000);
//                          }
//                          break;
//
//                      case GESTURE_DETECTED_LEFT:
//                          if (accel_mg[0] >=0) {
//                              gesture_state = GESTURE_IDLE;
//      	  	  	  	  	  	  	snprintf((char*)tx_buffer,sizeof(tx_buffer),"Gesture LEFT complete!");
//      	  	  	  	  	  	  	  	tx_com(tx_buffer,strlen((char const*)tx_buffer));
//      	  	  	  	  	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//      	  	  	  	  	  HAL_Delay(1000);
//                          }
//                          break;
//
//                      case GESTURE_DETECTED_RIGHT:
//                          if (accel_mg[0]<=0) {
//                              gesture_state = GESTURE_IDLE;
//      	  	  	  	  	  	  	snprintf((char*)tx_buffer,sizeof(tx_buffer),"Gesture RIGHT complete!");
//      	  	  	  	  	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//      	  	  	  	  	HAL_Delay(1000);
//                          }
//                          break;
//
//                      default:
//                          gesture_state = GESTURE_IDLE;
//                          break;
//                  }
//
//
//                  platform_delay(50);
//              }
//          }
//      }

//      snprintf((char *)tx_buffer, sizeof(tx_buffer), "------ \r\n\r\n");
//      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//    }
//  }




/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);

  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

  return 0;
}

/*
 * @brief  platform specific outputs on terminal (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
void tx_com(uint8_t *tx_buffer, uint16_t len)
{

  HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

