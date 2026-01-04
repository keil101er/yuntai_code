/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "can.h"

#define FLOAT_MAX_INS_YAW 3.5f
#define FLOAT_MIN_INS_YAW -3.5f

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
#define SUPERCAP_CAN hcan2
/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
	CAN_SUPERCAP_ID = 0x211,         //超电的id o
    
	CAN_YAW_MOTOR_ID = 0x09,
	
    CAN_PITCH_MOTOR_ID = 0x05,
	
    CAN_TRIGGER_MOTOR_ID = 0x207,
	
    CAN_GIMBAL_ALL_ID = 0x1FF,
	
	CAN_4310_CONTROL_ID =0x3fe,       //4310一拖四的发送id
	CAN_FRIC_ALL_ID = 0x1FF,
	
	CAN_FRIC_L_ID = 0x206,  //从枪管往前看的左右
	CAN_FRIC_R_ID = 0x205,
	
	
} can_msg_id_e;
	


//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;



typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;



typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Yaw_Motor_t;

//超电
extern void CAN_cmd_SuperCap(uint16_t powerLimit);
/*
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbalyaw(int16_t yaw);
// 发送摩擦轮电机控制电流(0x201,0x202)
extern void CAN_cmd_fric(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

//云台发送函数
extern void CAN_cmd_gimbal(int16_t shoot, int16_t rev);
//pitch轴电机位置速度模式
extern void CAN_cmd_4310pitch_pvmode(float pos,float vel);
//pitch4310初始化
extern void init_gimbalpitch(void);
//发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
extern void CAN_cmd_chassis_reset_ID(void);
//发送电机控制电流(0x201,0x202,0x203,0x204)
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
 //返回yaw 6020电机数据
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
//返回pitch 6020电机数据指针
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
//返回拨弹电机 2006电机数据指针
extern const motor_measure_t *get_trigger_motor_measure_point(void);
//返回摩擦轮电机 3508电机数据指针
extern const motor_measure_t *get_fricL_motor_measure_point(void);

extern const motor_measure_t *get_fricR_motor_measure_point(void);

extern void CAN_gimbal_send__to_chassis(CAN_HandleTypeDef *hcan,float UP_INS_YAW);

extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

extern void enable_motor_mode_yaw(void);
	
extern void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
extern void dm4310_fbdata1(Yaw_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
#endif
