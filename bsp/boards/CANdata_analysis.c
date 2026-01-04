#include "CANdata_analysis.h"
#include "main.h"

#define FLOAT_MAX 3.5f
#define FLOAT_MIN -3.5f

c_fbpara_t  receive_chassis_data;

void gimbal_receive_chassis(c_fbpara_t *receive_chassis, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==8)
	{
		  int16_t yaw_int =  ((int16_t)rx_data[0] << 8) | rx_data[1];
		  int16_t pitch_int =  ((int16_t)rx_data[2] << 8) | rx_data[3];
		  receive_chassis->receive_yaw_ch = yaw_int / 10000.0f;
		  receive_chassis->receive_pitch_ch = pitch_int / 10000.0f; 
		  receive_chassis->mode_flag = rx_data[4];
		  receive_chassis->reserve1 = rx_data[5];
		  receive_chassis->reserve2 = rx_data[6];  
	}
	
}

