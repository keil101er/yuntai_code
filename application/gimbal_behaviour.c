/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculate by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    add a gimbal behaviour mode
    1. in gimbal_behaviour.h , add a new behaviour name in gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // new add
    }gimbal_behaviour_e,
    2. implement new function. gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" param is gimbal movement contorl input. 
        first param: 'yaw' usually means  yaw axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.
        second param: 'pitch' usually means pitch axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.

        in this new function, you can assign set-point to "yaw" and "pitch",as your wish
    3.  in "gimbal_behavour_set" function, add new logical judgement to assign GIMBAL_XXX_XXX to  "gimbal_behaviour" variable,
        and in the last of the "gimbal_behaviour_mode_set" function, add "else if(gimbal_behaviour == GIMBAL_XXX_XXX)" 
        choose a gimbal control mode.
        four mode:
        GIMBAL_MOTOR_RAW : will use 'yaw' and 'pitch' as motor current set,  derectly sent to can bus.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' are angle increment,  control enconde relative angle.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' are angle increment,  control gyro absolute angle.
    4. in the last of "gimbal_behaviour_control_set" function, add
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }

        
    如果要添加一个新的行为模式
    1.首先，在gimbal_behaviour.h文件中， 添加一个新行为名字在 gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // 新添加的
    }gimbal_behaviour_e,

    2. 实现一个新的函数 gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" 参数是云台运动控制输入量
        第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        第二个参数: 'pitch' 通常控制pitch轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        在这个新的函数, 你能给 "yaw"和"pitch"赋值想要的参数
    3.  在"gimbal_behavour_set"这个函数中，添加新的逻辑判断，给gimbal_behaviour赋值成GIMBAL_XXX_XXX
        在gimbal_behaviour_mode_set函数最后，添加"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,然后选择一种云台控制模式
        3种:
        GIMBAL_MOTOR_RAW : 使用'yaw' and 'pitch' 作为电机电流设定值,直接发送到CAN总线上.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 是角度增量,  控制编码相对角度.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 是角度增量,  控制陀螺仪绝对角度.
    4.  在"gimbal_behaviour_control_set" 函数的最后，添加
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "AutoGimbal.h"
#include "user_lib.h"

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
//when gimbal is in calibrating, set buzzer frequency and strenght
//当云台在校准, 设置蜂鸣器频率和强度
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()
uint8_t auto_flag=0;
uint8_t last_auto_flag=0; 
uint8_t auto_cnt=0;
#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value 
  * @param          output: the processed channel value
  * @param          deadline
  */
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
extern TX_AUTO_AIM auto_to_nuchassis_transmit;
/**
  * @brief          judge if gimbal reaches the limit by gyro
  * @param          gyro: rotation speed unit rad/s
  * @param          timing time, input "GIMBAL_CALI_STEP_TIME"
  * @param          record angle, unit rad
  * @param          feedback angle, unit rad
  * @param          record ecd, unit raw
  * @param          feedback ecd, unit raw
  * @param          cali step, +1 by one step
  */
/**
  * @brief          通过判断角速度来判断云台是否到达极限位置
  * @param          对应轴的角速度，单位rad/s
  * @param          计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param          记录的角度 rad
  * @param          反馈的角度 rad
  * @param          记录的编码值 raw
  * @param          反馈的编码值 raw
  * @param          校准的步骤 完成一次 加一
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }
float yaw_value,pitch_value;
/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set,c_fbpara_t *motor);
static void gimbal_auto_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set,c_fbpara_t  *chassis_transmit);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis, 
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
  *                 and gimbal control mode is gyro mode. 
  * @param[out]     yaw: yaw axia absolute angle increment, unit rad
  * @param[out]     pitch: pitch axia absolute angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set,c_fbpara_t  *chassis_transmit);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     pitch: pitch axia relative angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set,c_fbpara_t  *chassis_transmit);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     pitch: pitch axia relative angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

//云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
/**
  * @brief 被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set,c_fbpara_t *chassis_send_to_gimbal)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //云台行为状态机设置  行为gimbal_behaviour通过判断来进行切换   通过底盘发送上来的模式状态机mode_flag 来控制 云台模式切换
    gimbal_behavour_set(gimbal_mode_set,chassis_send_to_gimbal);

    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)   //行为模式为无力模式
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
//    else if (gimbal_behaviour == GIMBAL_INIT)  //行为模式为初始化模式
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
//        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
//    }
//    else if (gimbal_behaviour == GIMBAL_CALI)//行为模式为校准模式
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
//        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
//    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)  //行为模式为绝对角度模式
    {
       gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
		//*********************************************************************************
//		gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)  //行为模式为相对角度模式(暂时没用)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)  //行为模式为静止不动模式(暂时没用)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }    
		else if (gimbal_behaviour == GIMBAL_AUTO)  //行为模式为自瞄模式(暂时没开)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_AUTO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_AUTO;
    }
}

extern chassis_behaviour_e chassis_behaviour_mode;
//自瞄模式下对yaw和pitch的控制
static void gimbal_AUTO_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set,c_fbpara_t  *chassis_transmit)
{
//	yaw_value = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//	pitch_value = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//	send_gimbal_angles(1,0.0f,pitch_value,yaw_value);
//	send_gimbal_angles(1,0.0f,1.0f,2.0f);
    if (yaw == NULL ||  gimbal_control_set == NULL)
    {
        return;
    }
		static int16_t yaw_channel = 0, pitch_channel = 0;
    last_auto_flag=auto_flag;
		
//    rc_deadband_limit(chassis_transmit->yaw_ch, yaw_channel, RC_DEADBAND);
//    rc_deadband_limit(chassis_transmit->pitch_ch, pitch_channel, RC_DEADBAND);
//比较上次自瞄数据与这次自瞄数据是否有差别
		if(((gimbal_control_set->gimbal_yaw_motor.last_auto_data) == (gimbal_control_set->gimbal_AUTO_ctrl->x)) && ((gimbal_control_set->gimbal_pitch_motor.last_auto_data) == (gimbal_control_set->gimbal_AUTO_ctrl->y)))
    {
      auto_cnt++;
    }
    else
    {
      auto_cnt=0;
    }
//			// 阶梯性处理yaw数据
//			gimbal_control_set->gimbal_AUTO_ctrl->y = fabs(gimbal_control_set->gimbal_AUTO_ctrl->y) >3.0f ?gimbal_control_set->gimbal_AUTO_ctrl->y:gimbal_control_set->gimbal_AUTO_ctrl->y/1.2f;
//			if(fabs(gimbal_control_set->gimbal_AUTO_ctrl->y)<1.0f) gimbal_control_set->gimbal_AUTO_ctrl->y =0; 
	//如果接收相同自瞄数据超过一定次数，就切回手动控制
	if(auto_cnt>10)
  {
    auto_cnt=11;
    auto_flag=0;
    *yaw = -chassis_transmit->receive_yaw_ch*4;
    *pitch = - chassis_transmit->receive_pitch_ch*3;
  }
  else
  {
	// 如果没有识别到目标，关闭自瞄控制
  // if (gimbal_control_set->gimbal_AUTO_ctrl->x == 0 && gimbal_control_set->gimbal_AUTO_ctrl->y == 0)
    if(gimbal_control_set->gimbal_AUTO_ctrl->distance==-1)
    {
      auto_flag=0;
      *yaw = -chassis_transmit->receive_yaw_ch*4;
      *pitch = - chassis_transmit->receive_pitch_ch*3;
    }
	  else
	  {
      auto_flag=1;
		//小陀螺模式下的赋值
		  if(chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
		  {
		    // *yaw = (double)gimbal_control_set->gimbal_AUTO_ctrl->x* YAW_AUTO_SEN_WZ; //
        // *pitch  =-(double)gimbal_control_set->gimbal_AUTO_ctrl->y* YAW_AUTO_SEN_WZ;//
        *yaw = (double)gimbal_control_set->gimbal_AUTO_ctrl->x; //
        *pitch  =-(double)gimbal_control_set->gimbal_AUTO_ctrl->y;//
		  }
		  //底盘跟随云台下的赋值
		  else
		  {
		    // *yaw = (double)gimbal_control_set->gimbal_AUTO_ctrl->x* YAW_AUTO_SEN; //
        // *pitch=-(double)gimbal_control_set->gimbal_AUTO_ctrl->y* PITCH_AUTO_SEN;//
		    *yaw = (double)gimbal_control_set->gimbal_AUTO_ctrl->x; //
        *pitch=-(double)gimbal_control_set->gimbal_AUTO_ctrl->y;//        
		  }	
	  }
  }
		gimbal_control_set->gimbal_yaw_motor.last_auto_data = gimbal_control_set->gimbal_AUTO_ctrl->x;
		gimbal_control_set->gimbal_pitch_motor.last_auto_data = gimbal_control_set->gimbal_AUTO_ctrl->y;
		// gimbal_control_set->gimbal_AUTO_ctrl->x = 0.0f;
		// gimbal_control_set->gimbal_AUTO_ctrl->y = 0.0f;		
		yaw_value = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
	    pitch_value = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//	    send_gimbal_angles(1,0.0f,pitch_value,yaw_value);
//		send_gimbal_angles(1,0.0f,1.0f,2.0f);
}

/**
  * @brief          the function is called by gimbal_set_contorl function in gimbal_task.c
  *                 accoring to the gimbal_behaviour variable, call the corresponding function
  * @param[out]     add_yaw:yaw axis increment angle, unit rad
  * @param[out]     add_pitch:pitch axis increment angle,unit rad
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set,c_fbpara_t *chassis_transmit)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
//	yaw_value = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//	pitch_value = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//	send_gimbal_angles(1,0.0f,pitch_value,yaw_value);

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
//    else if (gimbal_behaviour == GIMBAL_INIT)
//    {
//        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
//    }
//    else if (gimbal_behaviour == GIMBAL_CALI)
//    {
//        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
//    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch,gimbal_control_set,chassis_transmit);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch,gimbal_control_set,chassis_transmit);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
		 else if (gimbal_behaviour == GIMBAL_AUTO)
    {
//		yaw_value = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//	    pitch_value = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//	    send_gimbal_angles(1,0.0f,pitch_value,yaw_value);
        gimbal_AUTO_angle_control(add_yaw, add_pitch, gimbal_control_set,chassis_transmit);
    }

}

/**
  * @brief          in some gimbal mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          云台在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some gimbal mode, need shoot keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          云台在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}



/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
extern float angle_error;

static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set,c_fbpara_t *chassis_transmit)
{

    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //in cali mode, return
    //校准行为，return 不会设置其他的模式
//    if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
//    {
//        return;
//    }
    //if other operate make step change to start, means enter cali mode
    //如果外部使得校准步骤从0 变成 start，则进入校准模式
//    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP && ! chassis_transmit->DUBS_on)
//    {
//        gimbal_behaviour = GIMBAL_CALI;
//        return;
//    }

   //**********************************************/
//    //初始化模式判断是否到达中值位置
//    if (gimbal_behaviour == GIMBAL_INIT)
//    {
//        static uint16_t init_time = 0;
//        static uint16_t init_stop_time = 0;
//        init_time++;
//        
//        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
//             fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
//        {
//            
//            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
//            {
//                init_stop_time++;
//            }
//        }
//        else
//        {
//            
//            if (init_time < GIMBAL_INIT_TIME)
//            {
//                init_time++;
//            }
//        }

//        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
//        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
//            !switch_is_down(chassis_transmit->mode_flag) && ! motor->DUBS_on)
//        {
//            return;
//        }
//        else
//        {
//            init_stop_time = 0;
//            init_time = 0;
//        }
//    }
//     // 如果当前处于自瞄模式且没有识别到目标，保持自瞄模式
//    if (gimbal_behaviour == GIMBAL_AUTO && 
//        gimbal_mode_set->gimbal_AUTO_ctrl->x == 0 && 
//        gimbal_mode_set->gimbal_AUTO_ctrl->y == 0)
//    {
//        return;
//    }
    //开关控制 云台状态  **********************************chassis_transmit->mode_flag 下板对上板进行模式切换*******************************************
	/*****************************************************************************************************************/




    if(chassis_transmit->mode_flag == 0) //如果拨杆处在最下面
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;    //云台无力模式
    }
    else if (chassis_transmit->mode_flag == 1)  //如果拨杆处在中间
    { 
//        gimbal_behaviour = GIMBAL_AUTO;
//        if( gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r) //如果在中间并且按下了遥控器的右键，为自瞄模式 否则为绝对角度控制
//            gimbal_behaviour = GIMBAL_AUTO;
//        else 
           gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE; //鼠标右键不按下，拨杆处于中档则为绝对角度
//  gimbal_behaviour =  GIMBAL_RELATIVE_ANGLE;

	}
  else if(chassis_transmit->mode_flag == 2)
	{
		gimbal_behaviour = GIMBAL_AUTO;
	}
	// gimbal_behaviour = GIMBAL_AUTO;






//    else if (switch_is_up(chassis_transmit->mode_flag))  //拨杆最上面云台则为自瞄模式
//    {
////				if(	gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r)
//						gimbal_behaviour = GIMBAL_AUTO;
////				else
////						gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
//    }
	
	
	
	

    //enter init mode
    //判断进入init状态机
 //   {
//        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
//        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
//        {
//            gimbal_behaviour = GIMBAL_INIT;   //进入云台初始化模式
//        }
//        last_gimbal_behaviour = gimbal_behaviour;
//    }
}
/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * 云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴

  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *yaw = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *pitch = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}   

/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

//        *pitch = GIMBAL_CALI_MOTOR_SET_4310;
        *yaw = 0;

        //判断陀螺仪数据， 并记录最大最小角度数据
        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
//        *pitch = -GIMBAL_CALI_MOTOR_SET_4310;
        *yaw = 0;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}


/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
//手动控制下对yaw和pitch的控制
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set,c_fbpara_t  *chassis_transmit)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

//    rc_deadband_limit(chassis_transmit->yaw_ch, yaw_channel, RC_DEADBAND);
//    rc_deadband_limit(chassis_transmit->pitch_ch, pitch_channel, RC_DEADBAND);

    *yaw = -chassis_transmit->receive_yaw_ch*4;
    *pitch = - chassis_transmit->receive_pitch_ch*3;

//    {
//        static uint16_t last_turn_keyboard = 0;
//        static uint8_t gimbal_turn_flag = 0;
//        static fp32 gimbal_end_angle = 0.0f;

//        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
//        {
//            if (gimbal_turn_flag == 0)
//            {
//                gimbal_turn_flag = 1;
//                //保存掉头的目标值
//                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
//            }
//        }
//        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

//        if (gimbal_turn_flag)
//        {
//            //不断控制到掉头的目标值，正转，反装是随机
//            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
//            {
//                *yaw += TURN_SPEED;
//            }
//            else
//            {
//                *yaw -= TURN_SPEED;
//            }
//        }
//        //到达pi （180°）后停止
//        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
//        {
//            gimbal_turn_flag = 0;
//        }
//    }
}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     pitch: pitch axia relative angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set,c_fbpara_t *chassis_transmit)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;
		
//		rc_deadband_limit(chassis_transmit->yaw_ch, yaw_channel, RC_DEADBAND);
//    rc_deadband_limit(chassis_transmit->pitch_ch, pitch_channel, RC_DEADBAND);

    *yaw = chassis_transmit->receive_yaw_ch;
    *pitch = chassis_transmit->receive_pitch_ch;


}



/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     pitch: pitch axia relative angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}
