/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define PWM_DETAL_VALUE 1750

#define SERVO_ADD_PWM_OPEN_KEY  KEY_PRESSED_OFFSET_C
#define SERVO_ADD_PWM_CLOSE_KEY  KEY_PRESSED_OFFSET_V
const RC_ctrl_t *servo_rc;
uint16_t servo_pwm[4] = {PWM_DETAL_VALUE, PWM_DETAL_VALUE, PWM_DETAL_VALUE, PWM_DETAL_VALUE};
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
		short int t=1;
    while(1)
    {
			t++;
        for(uint8_t i = 0; i < 4; i++)
      {
			if(servo_rc->rc.ch[4]>200)//开弹仓
            {
				servo_pwm[i] = 2400;
				servo_pwm_set(servo_pwm[i], i);
//				HAL_Delay(10000);	
//        servo_pwm[i] = PWM_DETAL_VALUE;
//				t=0;
            }
			else if(servo_rc->rc.ch[4]<-200)//关闭弹仓
			{
//				t=0;
				servo_pwm[i] = PWM_DETAL_VALUE;
				servo_pwm_set(servo_pwm[i], i);
			}
			

//			else
//			{
//				servo_pwm[i] = PWM_DETAL_VALUE;
//				servo_pwm_set(servo_pwm[i], i);
//			}
            //limit the pwm
           //限制pwm
//            if(servo_pwm[i] < SERVO_MIN_PWM)
//            {
//                servo_pwm[i] = SERVO_MIN_PWM;
//            }
//            else if(servo_pwm[i] > SERVO_MAX_PWM)
//            {
//                servo_pwm[i] = SERVO_MAX_PWM;
//            }
				servo_pwm_set(servo_pwm[i], i);
        }
        osDelay(10);
    }
}


