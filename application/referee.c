#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

/* 裁判系统数据帧头 / Referee system data frame header */
frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

/* 比赛状态相关数据 / Game status related data */
game_status_t game_state;
game_result_t game_result;
game_robot_HP_t robot_HP_t;

/* 场地事件相关数据 / Field event related data */
event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
referee_warning_t referee_warning_t1;

/* 机器人状态相关数据 / Robot status related data */
robot_status_t robot_state;
power_heat_data_t power_heat_data_t1;
robot_pos_t game_robot_pos_t;
buff_t buff_musk_t;
air_support_data_t robot_energy_t;
hurt_data_t robot_hurt_t;
shoot_data_t shoot_data_t1;
projectile_allowance_t bullet_remaining_t;
robot_interaction_data_t student_interactive_data_t;
ext_robot_command_t ext_robot_command;

/**
 * @brief  初始化裁判系统数据结构 / Initialize referee system data structures
 * @param  None
 * @retval None
 * @note   将所有裁判系统相关的数据结构清零，在系统启动时调用
 *         Clear all referee system related data structures, called at system startup
 */
void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
   
    memset(&game_state, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&robot_HP_t, 0, sizeof(game_robot_HP_t));


    memset(&field_event, 0, sizeof(event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t1, 0, sizeof(referee_warning_t));


    memset(&robot_state, 0, sizeof(robot_status_t));
    memset(&power_heat_data_t1, 0, sizeof(power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(buff_t));
    memset(&robot_energy_t, 0, sizeof(air_support_data_t));
    memset(&robot_hurt_t, 0, sizeof(hurt_data_t));
    memset(&shoot_data_t1, 0, sizeof(shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(projectile_allowance_t));
    memset(&ext_robot_command, 0, sizeof(ext_robot_command));

    memset(&student_interactive_data_t, 0, sizeof(robot_interaction_data_t));



}

/**
 * @brief  裁判系统数据解析函数 / Referee system data parsing function
 * @param  frame: 接收到的数据帧 / Received data frame
 * @retval None
 * @note   根据命令ID解析裁判系统发送的各类数据并存储到对应的全局变量中
 *         Parse various referee system data according to command ID and store in global variables
 */
void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    /* 复制帧头数据 / Copy frame header */
    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    /* 提取命令ID / Extract command ID */
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    /* 根据命令ID解析不同类型的裁判系统数据 / Parse different referee system data based on command ID */
    switch (cmd_id)
    {
        /* 比赛状态数据 / Game status data (0x0001) */
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(game_status_t));
        }
        break;

        /* 比赛结果数据 / Game result data (0x0002) */
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
        }
        break;

        /* 机器人血量数据 / Robot HP data (0x0003) */
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&robot_HP_t, frame + index, sizeof(game_robot_HP_t));
        }
        break;


        /* 场地事件数据（补给站状态、能量机关等） / Field event data (supply station, power rune, etc.) (0x0101) */
        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;

        /* 补给站动作标识数据 / Supply station action data (0x0102) */
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;

        /* 请求补给站补弹数据 / Supply station booking data (0x0103) */
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
        break;

        /* 裁判警告数据（犯规信息） / Referee warning data (violation info) (0x0104) */
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t1, frame + index, sizeof(referee_warning_t));
        }
        break;

        /* 机器人状态数据（等级、血量、功率上限等） / Robot status data (level, HP, power limit, etc.) (0x0201) */
        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;

        /* 实时功率热量数据（底盘功率、枪口热量） / Real-time power and heat data (chassis power, barrel heat) (0x0202) */
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t1, frame + index, sizeof(power_heat_data_t));
        }
        break;

        /* 机器人位置数据（场地坐标） / Robot position data (field coordinates) (0x0203) */
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;

        /* 机器人增益数据（BUFF状态） / Robot buff data (buff status) (0x0204) */
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;

        /* 空中机器人能量状态数据 / Aerial robot energy data (0x0205) */
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;

        /* 伤害状态数据（受到攻击信息） / Damage data (when robot is hit) (0x0206) */
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;

        /* 实时射击信息（弹速、射频等） / Real-time shooting data (bullet speed, frequency) (0x0207) */
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t1, frame + index, sizeof(shoot_data_t));
        }
        break;

        /* 弹丸剩余发射数数据 / Projectile allowance data (remaining ammo) (0x0208) */
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(projectile_allowance_t));
        }
        break;

        /* 机器人间交互数据（图传链路、自定义数据） / Robot interaction data (custom UI, data sharing) (0x0301) */
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        break;

		/* 云台手自定义数据（已注释） / Gimbal operator custom data (commented) */
		//		 case YUNTAISHOU_DATE:
//        {
//             memcpy(&ext_robot_command, frame + index, sizeof(ext_robot_command));
//        }
        break;

        /* 未知命令ID，不做处理 / Unknown command ID, no action */
        default:
        {
            break;
        }
    }
}

/**
 * @brief  获取底盘功率上限 / Get chassis maximum power limit
 * @param  max_power_limit: 功率上限指针 / Pointer to maximum power limit
 * @retval None
 * @note   从裁判系统获取的底盘功率上限，用于功率控制
 *         Chassis power limit from referee system, used for power control
 */
void get_chassis_max_power(uint16_t *max_power_limit)
{
	*max_power_limit = robot_state.chassis_power_limit;
}

/**
 * @brief  获取底盘实时功率和缓冲能量 / Get chassis real-time power and buffer energy
 * @param  power: 底盘功率指针（W） / Pointer to chassis power (Watts)
 * @param  buffer: 缓冲能量指针（J） / Pointer to buffer energy (Joules)
 * @retval None
 * @note   用于功率控制和超功率保护，防止底盘功率超限
 *         Used for power control and over-power protection
 */
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t1.chassis_power;
    *buffer = power_heat_data_t1.buffer_energy;
    //*buffer =70;*power=20;  /* 测试数据（已注释） / Test data (commented) */
}


/**
 * @brief  获取机器人ID / Get robot ID
 * @param  None
 * @retval 机器人ID（红1=1, 红2=2, ..., 蓝1=101, 蓝2=102, ...）
 *         Robot ID (Red1=1, Red2=2, ..., Blue1=101, Blue2=102, ...)
 * @note   用于识别机器人队伍和编号
 *         Used to identify robot team and number
 */
uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

/**
 * @brief  获取1号枪口热量上限和当前热量 / Get barrel 0 heat limit and current heat
 * @param  heat0_limit: 热量上限指针 / Pointer to heat limit
 * @param  heat0: 当前热量指针 / Pointer to current heat
 * @retval None
 * @note   用于射击热量控制，防止超热量惩罚
 *         Used for shooting heat control to prevent over-heat penalty
 */
void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_barrel_cooling_value;
    *heat0 = power_heat_data_t1.shooter_17mm_2_barrel_heat;
}

/**
 * @brief  获取机器人间通信指令数据 / Get robot command data
 * @param  coordinate_x: 目标X坐标指针 / Pointer to target X coordinate
 * @param  coordinate_y: 目标Y坐标指针 / Pointer to target Y coordinate
 * @param  commd_keyboard: 键盘指令指针 / Pointer to keyboard command
 * @retval None
 * @note   用于云台手向步兵机器人发送目标位置和指令
 *         Used for gimbal operator to send target position and commands to infantry robot
 */
void get_ext_robot_command_date(float *coordinate_x, float  *coordinate_y,uint8_t *commd_keyboard)
{
    *coordinate_x = ext_robot_command.target_position_x;
    *coordinate_y = ext_robot_command.target_position_y;
	  *commd_keyboard=ext_robot_command.commd_keyboard;

//	 *coordinate_x =1.223;       /* 测试数据（已注释） / Test data (commented) */
//    *coordinate_y =1.2223;
//	  *commd_keyboard='A';
}

/**
 * @brief  获取2号枪口热量上限和当前热量 / Get barrel 1 heat limit and current heat
 * @param  heat1_limit: 热量上限指针 / Pointer to heat limit
 * @param  heat1: 当前热量指针 / Pointer to current heat
 * @retval None
 * @note   用于双枪管机器人的第二枪口热量控制
 *         Used for heat control of second barrel in dual-barrel robots
 */
void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_barrel_cooling_value;
    *heat1 = power_heat_data_t1.shooter_17mm_2_barrel_heat;
}

/**
 * @brief  获取比赛状态 / Get game state
 * @param  game_progress: 比赛阶段指针（0-未开始, 1-准备, 2-自检, 3-5秒倒计时, 4-比赛中, 5-结算）
 *         Pointer to game progress (0-not started, 1-preparation, 2-self-check, 3-countdown, 4-in progress, 5-settlement)
 * @param  stage_remain_time: 当前阶段剩余时间指针（秒） / Pointer to stage remaining time (seconds)
 * @retval None
 * @note   用于根据比赛阶段调整机器人行为策略
 *         Used to adjust robot behavior strategy according to game stage
 */
void get_game_state(uint8_t *game_progress, uint16_t *stage_remain_time)
{
    *game_progress =game_state.game_progress ;
    *stage_remain_time = game_state.stage_remain_time;
}

