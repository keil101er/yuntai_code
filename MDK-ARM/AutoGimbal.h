#ifndef AUTOGIMBAL_H
#define AUTOGIMBAL_H
#include "main.h"
#include "referee.h"
#include "string.h"
#define BUFLENGTH  		128//最大接收的数据
#define DATELENGTH		16//有效数据

#define PITCH_AUTO_SEN    0.017f                            //
#define YAW_AUTO_SEN  0.031f                                //0.021f  
#define YAW_AUTO_SEN_WZ  0.031f   



typedef struct
{
  uint8_t FRAME_HEADER ; 
  uint8_t mode;
  float x; 
  float y;
  float distance; 
  uint8_t blank;               //空白帧，视觉要不要校验
  uint8_t FRAME_TAIL ;         //帧尾
} CTRL;

typedef __packed struct
{
	uint8_t FRAME_HEADER ;       //帧头
	uint8_t mode;
	float roll;
	float pitch;
	float yaw;
	uint8_t blank;               //空白帧，视觉要不要校验由视觉决定
	uint8_t FRAME_TAIL ;         //帧尾

}AUTO_SEND_TO_NUC_DATA_t;


typedef union      //共用体
{
AUTO_SEND_TO_NUC_DATA_t   AUTO_SEND_TO_NUC_DATA;  //104字节
uint8_t board_tx_date[16];  
} TX_AUTO_AIM;


typedef struct//发送数据
{
  float x;
  float y;
	uint8_t key_board;
} RX_DATE_t;

typedef union//接收数据
{
	CTRL Rec;
	uint8_t buf[DATELENGTH];
}BUF;

typedef struct//发送比赛状态
{
	 uint16_t game_time;
 uint8_t game_progress;
} GAME_DATE_t;


typedef union
{
//	RX_DATE_t RX_DATE;
	uint8_t rx_date[9];
}TX_DATE;

typedef union      
{
GAME_DATE_t GAME_DATE;  
	uint8_t rx_date[3];  //这里的rx_date是从裁判系统接收的数据
}TX_GAME;

extern void  AUTO_control_init(void);
extern CTRL *get_AUTO_control_point(void);
extern volatile uint32_t vision_last_target_time;
extern void memory_from_buffer(uint8_t *buffer, CTRL *ctrl);
extern void USART1_IDLE_Handler(void);
extern void vision_try_transmit(void);

#define VISION_TX_FRAME_LENGTH 43

#pragma pack(push, 1)
typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint8_t tail[2];
} vision_tx_frame_t;
#pragma pack(pop)

typedef union
{
    vision_tx_frame_t frame;
    uint8_t raw[VISION_TX_FRAME_LENGTH];
} vision_tx_buffer_t;

typedef char vision_tx_frame_size_check[(sizeof(vision_tx_frame_t) == VISION_TX_FRAME_LENGTH) ? 1 : -1];
#endif

