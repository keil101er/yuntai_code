/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "CANdata_analysis.h"

//#include "AutoGimbal.h"
#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮
#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) //I7

shoot_control_t shoot_control;          //射击数据

extern power_heat_data_t power_heat_data_t1;
extern robot_status_t robot_state;
extern c_fbpara_t  receive_chassis_data;


// 初始化射击模块
void shoot_init(void)
{
	//初始化摩擦轮pid
 //   static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 fric_speed_pid[2][3] = {{FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD},
                                              {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD}};

    shoot_control.shoot_mode = SHOOT_STOP;//模式先为停止射击
//											  
//	//获取遥控器值**************************************************************************************									  
//    shoot_control.shoot_rc = get_remote_control_point(); 
	//获取板间通信flag										  
	shoot_control.shoot_flag =  0;
											  
//    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fricL_motor_measure = get_fricL_motor_measure_point();
    shoot_control.fricR_motor_measure = get_fricR_motor_measure_point();

											  
 //   PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_L_pid, PID_POSITION, fric_speed_pid[0], FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_R_pid, PID_POSITION, fric_speed_pid[1], FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);

											  
											  
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
    shoot_control.press_l_time = 0;  //左键时间
    shoot_control.rc_s_time = 0;
    shoot_control.block_time = 0;
    shoot_control.reverse_time = 0;
    shoot_control.key = 0;
    shoot_control.heat_limit = 0;  //热量限制
    shoot_control.heat = 0;

    shoot_control.fric_enabled = 0;  // 默认关闭摩擦轮
    SHOOT_ON_KEYBOARD;
}

// 射击数据更新
static void shoot_feedback_update(void)
{
//    static fp32 speed_fliter_1 = 0.0f;
//    static fp32 speed_fliter_2 = 0.0f;
//    static fp32 speed_fliter_3 = 0.0f;
//    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

//    speed_fliter_1 = speed_fliter_2;
//    speed_fliter_2 = speed_fliter_3;
//    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
//    shoot_control.speed = speed_fliter_3;
    shoot_control.fricL_speed = shoot_control.fricL_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    shoot_control.fricR_speed = shoot_control.fricR_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

    // 编码器跨零点检测（向下溢出）：当前值 - 上次值 > 半量程（4096），说明编码器从小值（接近0）跳到大值（接近8191）
    // Encoder zero-crossing detection (downward overflow): current - last > half range (4096), encoder jumped from small (near 0) to large (near 8191)
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;  // 圈数减1 / Decrement revolution counter
    }
    // 编码器跨零点检测（向上溢出）：当前值 - 上次值 < -半量程（-4096），说明编码器从大值（接近8191）跳到小值（接近0）
    // Encoder zero-crossing detection (upward overflow): current - last < -half range (-4096), encoder jumped from large (near 8191) to small (near 0)
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;  // 圈数加1 / Increment revolution counter
    }

    // 圈数计数器上溢出处理：达到最大圈数（18圈）时环绕到负最大值
    // Revolution counter overflow handling: wrap to negative max when reaching positive max (18 revolutions)
    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);  // 环绕到 -17 / Wrap to -17
    }
    // 圈数计数器下溢出处理：达到负最大圈数（-18圈）时环绕到正最大值
    // Revolution counter underflow handling: wrap to positive max when reaching negative max (-18 revolutions)
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;  // 环绕到 17 / Wrap to 17
    }

    // 计算电机绝对角度（弧度）：(圈数 * 单圈编码器量程8191 + 当前编码器值) * 编码器转角度系数
    // Calculate motor absolute angle (radians): (revolutions * encoder range 8191 + current encoder) * ecd-to-angle coefficient
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;

    // 从底盘通信数据的保留字段1获取射击标志（0=停止射击，1=开始射击）
    // Get shoot flag from chassis communication data reserve field 1 (0=stop shooting, 1=start shooting)
	shoot_control.shoot_flag =  receive_chassis_data.reserve1;
	
	
	//    shoot_control.key = BUTTEN_TRIG_PIN;

//    shoot_control.last_press_l = shoot_control.press_l;
//    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;

  //********************************************************************************************************************
//    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(receive_chassis_data.mode_flag))
//    {
//        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
//        {
//            shoot_control.rc_s_time++;
//        }
//    }
//    else
//    {
//        shoot_control.rc_s_time = 0;
//    }
}

//// 堵转倒转处理
//static void trigger_motor_turn_back(void)
//{
//    if (shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.speed_set = shoot_control.trigger_speed_set;
//    }
//    else
//    {
//        shoot_control.speed_set = -shoot_control.trigger_speed_set;
//    }

//    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.block_time++;
//        shoot_control.reverse_time = 0;
//    }
//    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
//    {
//        shoot_control.reverse_time++;
//    }
//    else
//    {
//        shoot_control.block_time = 0;
//    }
//}

//// 射击控制，控制拨弹电机角度，完成一次发射，目前并没有用上，需要配合位动开关
//static void shoot_bullet_control(void)
//{
//    if (shoot_control.move_flag == 0)
//    {
//        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
//        shoot_control.move_flag = 1;
//    }
//    if (shoot_control.key == SWITCH_TRIGGER_OFF)
//    {
//        shoot_control.shoot_mode = SHOOT_DONE;
//    }
//    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
//    {
//        shoot_control.trigger_speed_set = TRIGGER_SPEED;
//    }
//    else
//    {
//        shoot_control.move_flag = 0;
//    }
//}

// 射击状态机设置
static void shoot_set_mode(void)
{
//    static int8_t last_s = RC_SW_UP;
	
	if (shoot_control.shoot_flag ==0)
{
        shoot_control.fric_l_current = 0;
		shoot_control.fric_r_current = 0;
        shoot_control.fric_set = 0;
}
    
    if (shoot_control.shoot_flag ==1)
    {
        shoot_control.fric_set = BULLET_SPEED;
//		CAN_cmd_fric(1000, 1000, 0, 0);
		
      }
    
    if (receive_chassis_data.mode_flag == 0)
    {
       shoot_control.fric_l_current = 0;
		shoot_control.fric_r_current = 0;
		CAN_cmd_fric(0, 0, 0, 0);

    }

}

extern int board_receive_data[8];
// 射击循环，简洁一点
void shoot_control_loop(void)
{
    shoot_set_mode();
	
    shoot_feedback_update();
   
    PID_calc(&shoot_control.fric_motor_L_pid, shoot_control.fricL_speed, shoot_control.fric_set);
    PID_calc(&shoot_control.fric_motor_R_pid, shoot_control.fricR_speed, -shoot_control.fric_set);
    shoot_control.fric_l_current = (int16_t)(shoot_control.fric_motor_L_pid.out);
    shoot_control.fric_r_current = (int16_t)(shoot_control.fric_motor_R_pid.out);

//   CAN_cmd_fric(shoot_control.fric_l_current, shoot_control.fric_r_current, 0, 0);
//      
  
}    
