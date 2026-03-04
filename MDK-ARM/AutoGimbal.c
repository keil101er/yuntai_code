#include "AutoGimbal.h"
#include "referee.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "gimbal_task.h"
#define pi 3.1415
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern float pitch_angle;
extern gimbal_control_t gimbal_control;
/* Double buffer for ping-pong DMA reception, minimizes dead time */
/* 双缓冲区乒乓接收，最小化DMA空窗期 */
static uint8_t sbus_rx_double_buf[2][BUFLENGTH];
static uint8_t rx_buf_idx = 0;

uint8_t tx[2]={0,1};
BUF  RresPi;
TX_GAME game_date;//比赛时间数据
TX_DATE gimbaler_date;
TX_AUTO_AIM auto_to_nuc_data;
uint8_t data_length=0;
char txmessage[] = "";
void  AUTO_control_init(void)   //初始化是只在接收到nuc传来的数据后进行，所以调试要先让nuc发数据
{
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能串口空闲中断（两次消息的间隙会触发）
		rx_buf_idx = 0;
		HAL_UART_Receive_DMA(&huart1, sbus_rx_double_buf[rx_buf_idx], BUFLENGTH);
}
 //按字节顺序存储接收到的数据
void memory_from_buffer(uint8_t *buffer, CTRL *ctrl)
	{
    ctrl->FRAME_HEADER = buffer[0];
	//需要的部分
    memcpy(&ctrl->mode ,&buffer[1],1);
    memcpy(&ctrl->y, &buffer[2], 4);
    memcpy(&ctrl->x, &buffer[2 + 4], 4);
    memcpy(&ctrl->distance, &buffer[2 + 2 * 4], 4);
	// 获取 yaw 电机的指针
    const gimbal_motor_t *yaw_motor = get_yaw_motor_point();
    const gimbal_motor_t *pitch_motor = get_pitch_motor_point();
    static float last_x;
	static float last_y;
	// 检查自瞄数据的有效性
    // if (ctrl->x == 0 && ctrl->y == 0)
    // {
    //     // 如果没有识别到目标，保持上一次的目标位置
    //     ctrl->x = last_x;
    //     ctrl->y = last_y;
    // }
    // else
    // {
    //     // 更新上一次的目标位置
    //     last_x = ctrl->x - yaw_motor->absolute_angle +0.02;
    //     last_y = ctrl->y - ((-pitch_motor->absolute_angle)-0.2759)*1.3492;
    // }
	// if (ctrl->x != 0 || ctrl->y != 0)
	// {
	// 	ctrl->x = ctrl->x - yaw_motor->absolute_angle ;
	// 	ctrl->y = ctrl->y - ((-pitch_motor->absolute_angle)-0.2759)*1.3492;      //存在等比例换算关系
	// }
	// else
	// {
	// 	ctrl->x = 0;
    // 	ctrl->y = 0;
	// }
	// 没识别到目标（x==0 && y==0），保持上次的增量
    // if (ctrl->x == 0 && ctrl->y == 0)
    // {
	// 	if(last_x==0&&last_y==0)
	// 	{
	// 		last_x=gimbal_control.gimbal_yaw_motor.absolute_angle;
	// 		last_y=gimbal_control.gimbal_pitch_motor.absolute_angle;	
	// 	}
    //     ctrl->x = last_x;
    //     ctrl->y = last_y;
    // }
    // else
    // {
    //     // 有目标，保存原始增量值
    //     last_x = ctrl->x;
    //     last_y = ctrl->y;
    // }
    ctrl->blank = buffer[2 + 3 * sizeof(float)];
    ctrl->FRAME_TAIL = buffer[2 + 3 * sizeof(float) + 1];
}
void Usart1Receive_IDLE(void)  //接收任务，是串口接收中断里的 ，接收到nuc的数据之后
{
	/* ==== Step 1: Only abort RX DMA, preserve ongoing TX DMA ==== */
	/* ==== 第1步：仅停止接收DMA，保留正在进行的发送DMA ==== */
	/* NOTE: HAL_UART_DMAStop() would kill TX too, causing garbled responses */
	/* 注意：HAL_UART_DMAStop()会同时杀掉TX，导致响应数据残缺 */
	HAL_UART_AbortReceive(&huart1);

	data_length = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

	/* ==== Step 2: Save current buffer pointer, switch to other buffer ==== */
	/* ==== 第2步：保存当前缓冲区指针，切换到另一个缓冲区 ==== */
	uint8_t *process_buf = sbus_rx_double_buf[rx_buf_idx];
	rx_buf_idx ^= 1;  // Toggle buffer index / 切换缓冲区索引

	/* ==== Step 3: Restart DMA ASAP to minimize dead time ==== */
	/* ==== 第3步：尽快重启DMA以最小化数据丢失窗口 ==== */
	/* DMA is now receiving into the OTHER buffer while we process */
	/* 现在DMA往另一个缓冲区接收，我们处理当前缓冲区 */
	HAL_UART_Receive_DMA(&huart1, sbus_rx_double_buf[rx_buf_idx], BUFLENGTH);

	/* ==== Step 4: Process data (DMA is already running again) ==== */
	/* ==== 第4步：处理数据（DMA已经重新运行了） ==== */
	if (data_length >= DATELENGTH) {
		/* Search for valid frame(s) - handles concatenated packets */
		/* 搜索有效帧 - 处理粘包（两帧合并到达）的情况 */
		uint16_t i;
		for (i = 0; i <= data_length - DATELENGTH; i++) {
			if (process_buf[i] == 0xFF && process_buf[i + DATELENGTH - 1] == 0x0D) 
			{
				memory_from_buffer(&process_buf[i], &RresPi.Rec);
				i += DATELENGTH - 1; // Skip past this frame / 跳过已解析的帧
			}
		}
		/* Send response only if TX DMA is idle (avoid aborting previous TX) */
		/* 仅在发送空闲时回复（避免打断上一次发送） */
		if (huart1.gState == HAL_UART_STATE_READY) {
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&auto_to_nuc_data.AUTO_SEND_TO_NUC_DATA, 16);
		}
	}
}



void USART1_IDLE_Handler(void)   //串口接收中断
{
	if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //判断IDLE寄存器是否空闲
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);   //清除标志位                 
			Usart1Receive_IDLE();                          
	}
}


CTRL *get_AUTO_control_point(void)
{ 
	return &RresPi.Rec;
}




// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)//发送回调函数
// {
// 	//**********************************************************************************
// 	if(huart->Instance==USART1)
// 	{
// 		memset(gimbaler_date.rx_date,0,20);
// 	}
// }



	