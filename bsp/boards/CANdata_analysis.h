#ifndef CANDATA_ANALYSIS_H
#define CANDATA_ANALYSIS_H

#include "struct_typedef.h"

typedef struct 
{

	uint8_t  mode_flag;
	float  receive_yaw_ch;
	float  receive_pitch_ch;
	
	uint8_t   reserve1;
	uint8_t  reserve2;
	
	uint8_t  DUBS_on;  
	
}c_fbpara_t;

extern c_fbpara_t  C_data;

extern void gimbal_receive_chassis(c_fbpara_t *receive_chassis, uint8_t *rx_data,uint32_t data_len);

#endif
