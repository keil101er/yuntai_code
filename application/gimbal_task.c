/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
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

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"

#include "main.h"
#include <stdlib.h>

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "AutoGimbal.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "CANdata_analysis.h"

#define aim_color 0 //      //识别红色是0，蓝色是1

#define aim_color_id (robot_state.robot_id)
float angle_error;
float aim_speed;
fp32 add_yaw_angle = 0.0f;
fp32 add_pitch_angle = 0.0f;
extern uint8_t auto_flag;
// motor enconde value format, range[0-8191]
// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init);
static void gimbal_auto_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode, c_fbpara_t *motor);
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control, c_fbpara_t *C_data);
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop);
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_motor_absolute_angle_control_pitch(gimbal_motor_t *gimbal_motor);
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
/**
 * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
 * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
 * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
 * @param[out]     pid:云台PID指针
 * @param[in]      maxout: pid最大输出
 * @param[in]      intergral_limit: pid最大积分输出
 * @param[in]      kp: pid kp
 * @param[in]      ki: pid ki
 * @param[in]      kd: pid kd
 * @retval         none
 */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
 * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
 * @param[out]     pid:云台PID指针
 * @param[in]      get: 角度反馈
 * @param[in]      set: 角度设定
 * @param[in]      error_delta: 角速度
 * @retval         pid 输出
 */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

static fp32 gimbal2_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

/**
 * @brief          云台校准计算
 * @param[in]      gimbal_cali: 校准数据
 * @param[out]     yaw_offset:yaw电机云台中值
 * @param[out]     pitch_offset:pitch 电机云台中值
 * @param[out]     max_yaw:yaw 电机最大机械角度
 * @param[out]     min_yaw: yaw 电机最小机械角度
 * @param[out]     max_pitch: pitch 电机最大机械角度
 * @param[out]     min_pitch: pitch 电机最小机械角度
 * @retval         none
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

#if GIMBAL_TEST_MODE
// j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif

// extern c_fbpara_t  C_data;
// gimbal control data
// 云台控制所有相关数据
gimbal_control_t gimbal_control;
// motor current
// 发送的电机电流
static int16_t pitch_can_set_current = 0, shoot_can_set_current = 0;
fp32 yaw_can_set_current = 0;
// 自瞄时 发给nuc的数据
extern TX_AUTO_AIM auto_to_nuc_data;
extern int board_receive_data[8];
extern robot_status_t robot_state;

extern c_fbpara_t receive_chassis_data;

extern motor_measure_t motor_chassis[9];

extern shoot_control_t shoot_control; // 射击数据

void gimbal_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    // wait a time - increased to ensure INS is fully initialized
    vTaskDelay(GIMBAL_TASK_INIT_TIME * 3); // Triple the delay
    // gimbal init
    // 云台初始化
    gimbal_init(&gimbal_control);

    // Validate initialization
    if (gimbal_control.gimbal_INT_angle_point == NULL ||
        gimbal_control.gimbal_INT_gyro_point == NULL ||
        gimbal_control.gimbal_yaw_motor.gimbal_motor_measure == NULL ||
        gimbal_control.gimbal_pitch_motor.gimbal_motor_measure == NULL)
    {
        // Initialization failed - enter safe mode
        while (1)
        {
            vTaskDelay(1000); // Wait indefinitely
        }
    }

    // shoot init
    // 射击初始化
    shoot_init();

    // 自瞄初始化
    AUTO_control_init();
    // wait for all motor online
    // 判断电机是否都上线
    while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        vTaskDelay(GIMBAL_CONTROL_TIME);
    }

    while (1)
    {
        // 使能pitch4310
        for (int j = 0; j < 10; j++)
        {
            init_gimbalpitch();
            osDelay(1);
        }

        // 使能yaw4310
        for (int j = 0; j < 10; j++)
        {

            enable_motor_mode_yaw();
            osDelay(1);
        }
        //********************************************************************************
        // 云台行为状态机以及电机状态机设置
        gimbal_set_mode(&gimbal_control, &receive_chassis_data); // 设置云台控制模式
        // 电机控制模式切换 电机控制数据过渡
        gimbal_mode_change_control_transit(&gimbal_control); // 电机控制模式切换 控制数据过渡
        // 云台反馈更新
        gimbal_feedback_update(&gimbal_control); // 云台数据反馈

        gimbal_control.yaw_motor_angle = -motor_ecd_to_angle_change(motor_chassis[4].ecd, 0);

        gimbal_set_control(&gimbal_control, &receive_chassis_data); // 设置云台控制量

        //****************************************************************************************************************************
        
        gimbal_control_loop(&gimbal_control); // 云台控制PID计算

        //		shoot_control_loop();

        //        shoot_can_set_current =  shoot_control_loop();       //射击任务控制循环

#if YAW_TURN
        yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current_yaw;
#endif

#if PITCH_TURN
#else
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

        if (receive_chassis_data.mode_flag == 0)
        {
            CAN_cmd_gimbal(0, 0);
            osDelay(2);
            mit_ctrl(&hcan1, 0x07, 0, 0, 0, 0, 0);
            osDelay(2);
        }
        else if (receive_chassis_data.mode_flag == 1||receive_chassis_data.mode_flag == 2)
        {
            //              CAN_cmd_gimbal(shoot_can_set_current,0);
            //				osDelay(2);
            mit_ctrl(&hcan1, 0x07, 0, 0, 0, 0, yaw_can_set_current);
            //				mit_ctrl(&hcan1,0x07, 0, 0, 0, 0,0);
            osDelay(2);
            // 射击标志为 1：启动摩擦轮进行射击
            // Shoot flag is 1: Start friction wheels for shooting
            if (receive_chassis_data.reserve1 == 1)
            {
                // 设置摩擦轮目标速度为预定义的弹速对应转速
                // Set friction wheel target speed to predefined bullet speed
                shoot_control.fric_set = BULLET_SPEED;

                // 读取左右摩擦轮电机速度反馈，转换 RPM 到实际速度单位
                // Read left and right friction wheel motor speed feedback, convert RPM to actual speed units
                shoot_control.fricL_speed = shoot_control.fricL_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
                shoot_control.fricR_speed = shoot_control.fricR_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

                // PID 速度闭环控制：左摩擦轮反向旋转，右摩擦轮正向旋转（从后方看，两轮相向旋转夹紧弹丸）
                // PID speed closed-loop control: left wheel rotates forward, right wheel rotates backward (viewed from rear, wheels rotate towards each other to grip projectile)
                PID_calc(&shoot_control.fric_motor_L_pid, shoot_control.fricL_speed, -shoot_control.fric_set);
                PID_calc(&shoot_control.fric_motor_R_pid, shoot_control.fricR_speed, shoot_control.fric_set);

                // 将 PID 输出转换为电机驱动电流（int16_t 范围）
                // Convert PID output to motor drive current (int16_t range)
                shoot_control.fric_l_current = (int16_t)(shoot_control.fric_motor_L_pid.out);
                shoot_control.fric_r_current = (int16_t)(shoot_control.fric_motor_R_pid.out);

                // 通过 CAN 总线发送摩擦轮电机电流指令
                // Send friction wheel motor current commands via CAN bus
                CAN_cmd_fric(shoot_control.fric_r_current, shoot_control.fric_l_current, 0, 0);
            }
            // 射击标志为 0：停止摩擦轮
            // Shoot flag is 0: Stop friction wheels
            else if (receive_chassis_data.reserve1 == 0)
            {
                // 将电流设为 0，停止摩擦轮电机
                // Set current to 0, stop friction wheel motors
                shoot_control.fric_l_current = 0;
                shoot_control.fric_r_current = 0;

                // 发送停止指令到摩擦轮电机
                // Send stop command to friction wheel motors
                CAN_cmd_fric(0, 0, 0, 0);
            }
        }

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          gimbal cali data, set motor offset encode, max and min relative angle
 * @param[in]      yaw_offse:yaw middle place encode
 * @param[in]      pitch_offset:pitch place encode
 * @param[in]      max_yaw:yaw max relative angle
 * @param[in]      min_yaw:yaw min relative angle
 * @param[in]      max_yaw:pitch max relative angle
 * @param[in]      min_yaw:pitch min relative angle
 * @retval         none
 */
/**
 * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
 * @param[in]      yaw_offse:yaw 中值
 * @param[in]      pitch_offset:pitch 中值
 * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
 * @param[in]      min_yaw:yaw 最小相对角度
 * @param[in]      max_yaw:pitch 最大相对角度
 * @param[in]      min_yaw:pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    // 设置 yaw 电机编码器偏移量（校准中值）
    // Set yaw motor encoder offset (calibrated center value)
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;

    // 设置 yaw 轴最大相对角度限位（单位：弧度）
    // Set yaw axis maximum relative angle limit (unit: radians)
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;

    // 设置 yaw 轴最小相对角度限位（单位：弧度）
    // Set yaw axis minimum relative angle limit (unit: radians)
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    //    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    //    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    //    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}

/**
 * @brief          gimbal cali calculate, return motor offset encode, max and min relative angle
 * @param[out]     yaw_offse:yaw middle place encode
 * @param[out]     pitch_offset:pitch place encode
 * @param[out]     max_yaw:yaw max relative angle
 * @param[out]     min_yaw:yaw min relative angle
 * @param[out]     max_yaw:pitch max relative angle
 * @param[out]     min_yaw:pitch min relative angle
 * @retval         none
 */
/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        // 保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
        // pitch上位機控制回到零點
        //        gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
        //        gimbal_control.gimbal_pitch_motor.max_relative_angle    = *max_pitch;
        //        gimbal_control.gimbal_pitch_motor.min_relative_angle    = *min_pitch;
        gimbal_control.gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          calc motor offset encode, max and min relative angle
 * @param[out]     yaw_offse:yaw middle place encode
 * @param[out]     pitch_offset:pitch place encode
 * @param[out]     max_yaw:yaw max relative angle
 * @param[out]     min_yaw:yaw min relative angle
 * @param[out]     max_yaw:pitch max relative angle
 * @param[out]     min_yaw:pitch min relative angle
 * @retval         none
 */
/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         none
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        //        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
 * @brief          return yaw motor data point
 * @param[in]      none
 * @retval         yaw motor data point
 */
/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          return pitch motor data point
 * @param[in]      none
 * @retval         pitch motor data point
 */
/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
    // pitch的速度环pid
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    // yaw的速度环pid
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    // 电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    // 陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();

    // Pitch4310电机使能，发送ID为0x101 挂载在CAN2
    for (int j = 0; j < 10; j++)
    {
        init_gimbalpitch();
        osDelay(1);
    }

    for (int j = 0; j < 10; j++)
    {

        enable_motor_mode_yaw();
        osDelay(1);
    }

    //	CAN_cmd_4310pitch_pvmode(0.5,aim_speed);

    // 遥控器数据指针获取****************************************************************************************************
    init->gimbal_rc_ctrl = get_remote_control_point();

    init->gimbal_AUTO_ctrl = get_AUTO_control_point(); // 自瞄数据获取
    init->gimbal_yaw_motor.last_auto_data = 1;
    init->gimbal_pitch_motor.last_auto_data = 1;

    // 初始化电机模式 电机原始值控制:GIMBAL_MOTOR_RAW = 0   电机陀螺仪角度控制:GIMBAL_MOTOR_GYRO, 电机编码值角度控制:GIMBAL_MOTOR_ENCONDE,
    // 最一开始都为无力
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    // 初始化yaw电机pid
    // 自瞄low pid
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_low_auto_angle_pid, YAW_AUTO_LOW_ABSOLUTE_PID_MAX_OUT, YAW_AUTO_LOW_ABSOLUTE_PID_MAX_IOUT, YAW_AUTO_LOW_ABSOLUTE_PID_KP, YAW_AUTO_LOW_ABSOLUTE_PID_KI, YAW_AUTO_LOW_ABSOLUTE_PID_KD);
    // yaw绝对角度pid初始化
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    // yaw相对角度pid初始化
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);

    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_auto_angle_pid, YAW_AUTO_ABSOLUTE_PID_MAX_OUT, YAW_AUTO_ABSOLUTE_PID_MAX_IOUT, YAW_AUTO_ABSOLUTE_PID_KP, YAW_AUTO_ABSOLUTE_PID_KI, YAW_AUTO_ABSOLUTE_PID_KD);

    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

    // 初始化pitch电机pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_auto_angle_pid, PITCH_AUTO_ABSOLUTE_PID_MAX_OUT, PITCH_AUTO_ABSOLUTE_PID_MAX_IOUT, PITCH_AUTO_ABSOLUTE_PID_KP, PITCH_AUTO_ABSOLUTE_PID_KI, PITCH_AUTO_ABSOLUTE_PID_KD);

    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    // 清除所有PID
    gimbal_total_pid_clear(init);

    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle; // yaw的绝对角度赋值给绝对角度设定值
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle; // yaw的相对角度赋值给相妒意角度设定值
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
}
/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode, c_fbpara_t *motor)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode, motor);
}
/**
 * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
 * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
int color;
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA.FRAME_HEADER = 0xff;
    if (0 < aim_color_id < 10)
    {
        color = 1;
    }
    else
    {
        color = 0;
    }
    // 判断是否进入自瞄
    if (feedback_update->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO && feedback_update->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA.mode = 0;
    }
    else
    {
        auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA.mode = 0;
    }
    //自瞄数据更新
    float pitch_angle = ((-feedback_update->gimbal_pitch_motor.absolute_angle) - 0.2759) * 1.3492;
    auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA.roll = 0.0f;
    auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA.pitch = pitch_angle;
    auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA.yaw = feedback_update->gimbal_yaw_motor.absolute_angle;
    auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA.blank = 0;
    auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA.FRAME_TAIL = 0x0d;
    
    shoot_control.fricL_speed = shoot_control.fricL_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    shoot_control.fricR_speed = shoot_control.fricR_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

    // Check if INS data pointers are valid
    if (feedback_update->gimbal_INT_angle_point == NULL ||
        feedback_update->gimbal_INT_gyro_point == NULL)
    {
        return; // Skip update if INS data not ready
    }

    // Check if motor measure pointers are valid
    if (feedback_update->gimbal_pitch_motor.gimbal_motor_measure == NULL ||
        feedback_update->gimbal_yaw_motor.gimbal_motor_measure == NULL)
    {
        return; // Skip update if motor data not ready
    }

    // 云台数据更新
    // pitch绝对角度 = 初始化时的角度加上INS_PITCH_ADDRESS_OFFSET
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                    feedback_update->gimbal_pitch_motor.offset_ecd);
#else

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                   feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET); // 陀螺仪

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                  feedback_update->gimbal_yaw_motor.offset_ecd);
#else
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                  feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.absolute_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - arm_sin_f32(feedback_update->gimbal_pitch_motor.absolute_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
 * @brief          calculate the relative angle between ecd and offset_ecd
 * @param[in]      ecd: motor now encode
 * @param[in]      offset_ecd: gimbal offset encode
 * @retval         relative angle, unit rad
 */
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    // yaw电机状态机切换保存数据
    // 上一次的yaw电机模式不等于电机原始值设置 并且 这一次等于 电机原始值设置
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // 电机原始值设置 的电流值等于
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current; //
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    // pitch电机状态机切换保存数据,PITCH新电机是位置速度控制模式，和之前存储会有区别
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    //***************************************************************************************************************
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->pitch_transition_start_angle = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
        gimbal_mode_change->pitch_transition_target_angle = gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set;
        gimbal_mode_change->pitch_transition_start_time = osKernelSysTick();
        gimbal_mode_change->pitch_transition_duration = 1; // 过渡时间，单位ms   理论越小越好
        gimbal_mode_change->pitch_in_transition = 1;
    }
    //*************************************************************************
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control, c_fbpara_t *C_data)
{
    if (set_control == NULL)
    {
        return;
    }

    // fp32 add_yaw_angle = 0.0f;
    // fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control, C_data);

    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle); // add_yaw_angle
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        // enconde模式下，电机编码角度控制
        gimbal_auto_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    // pitch电机模式控制
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle); //////////////////////////////////////////添加限位
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        // enconde模式下，电机编码角度控制
        gimbal_auto_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle); //////////////////////////////////////////添加限位
    }
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_auto_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add) // 有限位
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    // // now angle error
    // // 当前控制误差角度
    // bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    // // relative angle + angle error + add_angle > max_relative angle
    // // 云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    // if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    // {
    //     // 如果是往最大机械角度控制方向
    //     if (add > 0.0f)
    //     {
    //         // calculate max add_angle
    //         // 计算出一个最大的添加角度，
    //         add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
    //     }
    // }
    // // Check if the target angle would exceed the minimum limit
    // // 检查目标角度是否会超出最小限位
    // else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    // {
    //     // If the angle increment is negative (moving downward/leftward)
    //     // 如果角度增量为负（向下/向左移动）
    //     if (add < 0.0f)
    //     {
    //         // Clamp the increment to prevent exceeding the minimum angle limit
    //         // 限制增量以防止超出最小角度限位
    //         add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
    //     }
    // }
    // Store current absolute angle setpoint
    // 保存当前绝对角度设定值
    // angle_set = gimbal_motor->absolute_angle_set;
    // Update the absolute angle setpoint with the clamped increment and normalize to [-π, π]
    // 使用限位后的增量更新绝对角度设定值，并归一化到 [-π, π] 范围
    if(auto_flag)
    {
        gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle + add);
    }
    else
    {
        angle_set = gimbal_motor->absolute_angle_set;
        gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
    }
}

static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add) // 无限位
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //    //now angle error
    //    //当前控制误差角度
    //    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //    //relative angle + angle error + add_angle > max_relative angle
    //    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    //    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    //    {
    //        //如果是往最大机械角度控制方向
    //        if (add > 0.0f)
    //        {
    //            //calculate max add_angle
    //            //计算出一个最大的添加角度，
    //            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
    //        }
    //    }
    //    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    //    {
    //        if (add < 0.0f)
    //        {
    //            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
    //        }
    //    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    // 是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}

extern chassis_behaviour_e chassis_behaviour_mode;
static void gimbal_motor_auto_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    CTRL *ctrl = get_AUTO_control_point();
    // 在开启小陀螺时的较硬自瞄
    if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_auto_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
        gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    }
    // 底盘跟随下的自瞄pid
    else
    {
        gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_low_auto_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
        gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    }
    // 控制值赋值
    // gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
    gimbal_motor->given_current_yaw = (int16_t)(gimbal_motor->current_set);                                                       
}

static void gimbal_motor_auto_angle_control_pitch(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    CTRL *ctrl = get_AUTO_control_point();

    if (gimbal_motor->absolute_angle_set > 0.5)
    {
        gimbal_motor->absolute_angle_set = 0.5;
    }
    else if (gimbal_motor->absolute_angle_set < -0.3)
    {
        gimbal_motor->absolute_angle_set = -0.3;
    }

    // 计算当前角度与目标角度的差值
    angle_error = gimbal_motor->absolute_angle_set - (-(gimbal_motor->absolute_angle) * 1.09756 + 0.064);
    // angle_error = gimbal_motor->absolute_angle_set - (gimbal_motor->absolute_angle);
    // 设置最大速度
    float max_speed = 30.0f;

    // 计算目标速度，距离目标位置越近，速度越小
    aim_speed = max_speed * fabs(angle_error) * 5;

    // 限制目标速度在0到max_speed之间
    if (aim_speed > max_speed)
    {
        aim_speed = max_speed;
    }
    else if (aim_speed < 0)
    {
        aim_speed = 0;
    }
    //CAN_cmd_4310pitch_pvmode(gimbal_motor->absolute_angle_set, aim_speed + 19);
    CAN_cmd_4310pitch_pvmode(gimbal_motor->absolute_angle_set, aim_speed);
}

/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sent to motor
 * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    //拨弹盘控制信息发送
    if(control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        CAN_gimbal_send__to_chassis(&hcan1,control_loop->gimbal_AUTO_ctrl->mode);
    }
    //yaw轴控制
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) // juedui
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) // xiangdui
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        gimbal_motor_auto_angle_control(&control_loop->gimbal_yaw_motor);
    }

    // pitch轴控制
    // Pitch 轴平滑过渡（位置插值/轨迹规划）控制，防止角度突变导致机械冲击
    if (control_loop->pitch_in_transition) // 如果正在过渡状态
    {
        uint32_t current_time = osKernelSysTick();                                        // 获取当前系统时间
        uint32_t elapsed_time = current_time - control_loop->pitch_transition_start_time; // 计算已经过去的时间
        if (elapsed_time < control_loop->pitch_transition_duration)                       // 如果还没到设定的总过渡时间
        {
            // 计算进度比例 (0.0 ~ 1.0)
            fp32 progress = (fp32)elapsed_time / control_loop->pitch_transition_duration;
            // 线性插值计算当前的中间目标角度：当前角度 = 起点角度 + 进度 * (终点角度 - 起点角度)
            control_loop->gimbal_pitch_motor.absolute_angle_set = control_loop->pitch_transition_start_angle + progress * (control_loop->pitch_transition_target_angle - control_loop->pitch_transition_start_angle);
        }
        else // 如果过渡时间已经结束
        {
            // 强制设为最终目标角度
            control_loop->gimbal_pitch_motor.absolute_angle_set = control_loop->pitch_transition_target_angle;
            // 结束过渡状态
            control_loop->pitch_in_transition = 0;
        }
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control_pitch(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        gimbal_motor_auto_angle_control_pitch(&control_loop->gimbal_pitch_motor);
    }
}

/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control_pitch(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    // 角度环，速度环串级pid调试

    if (gimbal_motor->absolute_angle_set > 0.22)
    {
        gimbal_motor->absolute_angle_set = 0.22;
    }
    else if (gimbal_motor->absolute_angle_set < -0.4)
    {
        gimbal_motor->absolute_angle_set = -0.4;
    }

    // 计算差值
    angle_error = gimbal_motor->absolute_angle_set - (-(gimbal_motor->absolute_angle) * 1.09756 + 0.064);

    // 设置最大速度
    float max_speed = 30.0f;

    // 计算目标速度
    aim_speed = max_speed * fabs(angle_error) * 0.6;

    // 限制目标速度
    if (aim_speed > max_speed)
    {
        aim_speed = max_speed;
    }
    else if (aim_speed < 0)
    {
        aim_speed = 0;
    }

    CAN_cmd_4310pitch_pvmode(gimbal_motor->absolute_angle_set, 10);
}

// yaw
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    // 角度环，速度环串级pid调试
    //    gimbal_motor->motor_gyro_set = gimbal2_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    //    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

    //	gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);

    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

    // 控制值赋值
    gimbal_motor->given_current_yaw = (gimbal_motor->current_set);
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    // 角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, 0, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    // 控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

/**
 * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
 *                 data point initialization, and gyro sensor angle point initialization.
 * @param[out]     gimbal_init: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     gimbal_init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

static fp32 gimbal2_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);

    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

/**
 * @brief          gimbal PID clear, clear pid.out, iout.
 * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
