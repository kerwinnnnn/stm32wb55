读取X轴加速度
当X轴加速度从0到大于100mg，再从大于100mg到0左右，可以看成一次手势向右的动作并到最后停止，灯亮到熄灭。
当X轴加速度从0到小于-100mg，再从小于-100mg到0左右，可以看成一次手势向左的动作并到最后停止，灯亮到熄灭。
由于该imu对加速度测量并不是那么稳定，可能存在突变情况，所以检测会有误识别。

由于git的仓库还没建，暂时贴出部分代码

 switch (gesture_state) {
                      case GESTURE_IDLE:
                          if (accel_mg[0] <-100) {
                              gesture_state = GESTURE_DETECTED_LEFT;
                              snprintf((char *)tx_buffer, sizeof(tx_buffer), "Gesture LEFT detected! \r\n");
                              tx_com(tx_buffer, strlen((char const *)tx_buffer));
                              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                              HAL_Delay(1000);
                          } else if (accel_mg[0] > 100) {
                              gesture_state = GESTURE_DETECTED_RIGHT;
                              snprintf((char *)tx_buffer, sizeof(tx_buffer), "Gesture RIGHT detected! \r\n");
                              tx_com(tx_buffer, strlen((char const *)tx_buffer));
                              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                              HAL_Delay(1000);
                          }
                          break;

                      case GESTURE_DETECTED_LEFT:
                          if (accel_mg[0] >=0) {
                              gesture_state = GESTURE_IDLE;
      	              snprintf((char*)tx_buffer,sizeof(tx_buffer),"Gesture LEFT complete!");
      	              tx_com(tx_buffer,strlen((char const*)tx_buffer));
      	              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      	             HAL_Delay(1000);
                          }
                          break;

                      case GESTURE_DETECTED_RIGHT:
                          if (accel_mg[0]<=0) {
                              gesture_state = GESTURE_IDLE;
      	              snprintf((char*)tx_buffer,sizeof(tx_buffer),"Gesture RIGHT complete!");
      	             HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      	             HAL_Delay(1000);
                          }
                          break;

                      default:
                          gesture_state = GESTURE_IDLE;
                          break;
                  }