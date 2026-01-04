#include "ui.h"
#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "string.h"
#include "gimbal_task.h"
#include "referee.h"

#define MAX_SIZE          128    //上传数据最大的长度
#define frameheader_len  5       //帧头长度
#define cmd_len          2       //命令码长度
#define crc_len          2       //CRC16校验
#define LENGTH           1920
#define WIDE             1080
#define N                 7     //图层数
#define true	1
#define false	0

uint8_t seq=0;
ext_student_interactive_header_data_t custom_grapic_draw;			//自定义图像绘制
graphic_data_struct_t G1,G2,G3,G4,G5,G6,G7;
extern gimbal_control_t gimbal_control;
extern robot_status_t robot_state; //读取弹速
int a=50;

//直线描绘
void Line_Draw(graphic_data_struct_t *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!='\0';i++)
   image->graphic_name[2-i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

//发送客户端函数
void Judge_Send_Data(uint8_t *data,uint8_t length)
{
	HAL_UART_Transmit_DMA(&huart6,data,length);
}




//线程函数
void user_task(void const *pvParameters)
 {
	 uint8_t updata[15+15*N];//15+15*n
	 uint16_t length=6+15*N; //数据帧中data长度
	 uint8_t shoot_data[4];
	 memset(&G1,0,sizeof(G1));//整数
	 memset(&shoot_data,0,sizeof(shoot_data));
 static uint8_t seq=0;
	float dis;
	static uint8_t  shoot_speed;
static int num =0;
int j=0;
	 
	
	 
while(1)
  {
   if (j<=20)
	{
			G1.operate_tpye =2;
		j++;
	}
   else
	{
			G1.operate_tpye =1;
	   j=1;
	}
	
		dis = gimbal_control.gimbal_AUTO_ctrl->distance ;//读取视觉传来的距离  
	//shoot_speed = robot_state.shooter_id1_17mm_speed_limit>18?1:0;      //读取弹速
	
		num = (int)(dis/100);  //串口手册要求
		if(seq == 255) seq = 0;
	seq++;
	updata[0] = 0xA5;
	updata[1] = length ;
	updata[2] = (length >> 8);
	updata[3] = seq;
	append_CRC8_check_sum(updata ,5); //帧头crc8校验
	updata[5] = 0x0301 &255;//命令码
	updata[6] = (0x0301 >>8);
	
	updata[7] = UI_Data_ID_Draw7 &255;//内容ID
	updata[8] = UI_Data_ID_Draw7>>8;

	updata[9] =robot_state.robot_id ;
	updata[10] =robot_state.robot_id>> 8;
					
	updata[11] = (robot_state.robot_id|0x100 )&255;
	updata[12] = (robot_state.robot_id|0x100 )>>8;

	G1.graphic_name[0] = 0;
	G1.graphic_name[1] = 0;
	G1.graphic_name[2] = 0;
//	G1.operate_tpye =2;
	G1.graphic_tpye =6;
	G1.layer = 0;
	G1.color =2;
	G1.start_angle =20;
	G1.end_angle =5;
	G1.width =2;
	G1.start_x = 840;
	G1.start_y =640;
	G1.radius =1;
	G1.end_x =2;
	G1.end_y = 3;

//	//画线
//	Line_Draw(&G2,"091",UI_Graph_ADD,9,UI_Color_Green,1,924,WIDE/2+200,924,WIDE/2-300);//中垂线
//	Line_Draw(&G3,"092",UI_Graph_ADD,9,UI_Color_Green,1,724,406,1124,406);//2.5米线
//	Line_Draw(&G4,"093",UI_Graph_ADD,9,UI_Color_Green,1,LENGTH/2-200,WIDE/2+100,LENGTH/2,WIDE/2+100);
//	Line_Draw(&G5,"094",UI_Graph_ADD,9,UI_Color_Pink,1,774,391,1074,391);//3米线
//	Line_Draw(&G6,"095",UI_Graph_ADD,9,UI_Color_Orange,1,824,342,1024,342);//3.5米线
//	Line_Draw(&G7,"076",UI_Graph_ADD,7,UI_Color_Purplish_red,1,824,360,1024,360);//4米线

	//画线
	Line_Draw(&G2,"091",UI_Graph_ADD,9,UI_Color_Green,1,983,WIDE/2+200,983,WIDE/2-300);//中垂线
	Line_Draw(&G3,"092",UI_Graph_ADD,9,UI_Color_Purplish_red,1,783,416,1183,416);//26左右弹速
	Line_Draw(&G4,"093",UI_Graph_ADD,9,UI_Color_Green,1,723,436,1233,436);//14左右弹速（2.5m）
	Line_Draw(&G5,"094",UI_Graph_ADD,9,UI_Color_Orange,1,850,426,1123,426);//(3m）
	Line_Draw(&G6,"095",UI_Graph_ADD,9,UI_Color_Pink,1,873,406,1100,406);	//(3.5m)
	Line_Draw(&G7,"076",UI_Graph_ADD,7,UI_Color_Pink,1,873,386,1100,386);//(4m)

	memcpy(updata+13,&G1,sizeof(G1)-4);//sizeof(G1) = 15;
	memcpy(updata+24,&num,4);
	memcpy(updata+28,&G2,sizeof(G2));
	memcpy(updata+43,&G3,sizeof(G3));//sizeof(G1) = 15;
	memcpy(updata+58,&G4,sizeof(G4));
	memcpy(updata+73,&G5,sizeof(G5));//sizeof(G1) = 15;
  memcpy(updata+88,&G6,sizeof(G6));
	memcpy(updata+103,&G7,sizeof(G7));//sizeof(G1) = 15;
	
	append_CRC16_check_sum(updata,15+15*N);//15+15*n
	Judge_Send_Data(updata,2*(15+15*N));
	
	memcpy(shoot_data,&shoot_speed,4);
		 
	//发送弹速函数
	HAL_UART_Transmit(&huart1,shoot_data,4,100);
	//超级电容等级控制
		{	
			if (robot_state.robot_level == 1 &&PowerData[3]!=50 )
			{
					CAN_cmd_SuperCap(5000);
			}			
			if (robot_state.robot_level == 2 &&PowerData[3]!=74)
			{
					CAN_cmd_SuperCap(7400);
			}
			if (robot_state.robot_level == 3 &&PowerData[3]!=94)
			{
					CAN_cmd_SuperCap(9400);
			}
		}
 	    osDelay(100);//10HZ发送
	 }
 }
