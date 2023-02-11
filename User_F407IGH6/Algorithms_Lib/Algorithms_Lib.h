/*
 * Algorithms_Lib.h
 *
 *  Created on: 2022年9月22日
 *      Author: Phonix
 */

#ifndef ALGORITHMS_LIB_ALGORITHMS_LIB_H_
#define ALGORITHMS_LIB_ALGORITHMS_LIB_H_
#include "main.h"
#include "math.h"
#define PI ((uint16_t)360)
#define Degree_turn_Radian 57.295779513082320876798154814105f

typedef enum{
	IMU_NO_ERROR = 0x00U,
	IMU_ACC_ERROR = 0x01U,
	IMU_GYRO_ERROR = 0x02U,
	IMU_MAG_ERROR = 0x04U,
}IMU_ERROR_Typedef;

typedef enum{
	Status_ERROR = 0x00U,
	Status_OK = 0x01U,
}Status_Typedef;


typedef struct{
	float output;//输出
	float filter_factor;//滤波系数，系数越小收敛越快
	float frame_period;//运算时间间隔，单位 S/秒
}LOW_PASS_FILTER_Typedef;

typedef struct{
	uint16_t  	angle;
	int16_t  	speed;
	int16_t 	current;
	uint8_t     temperature;
	uint16_t  	last_angle;
}motor_Typedef;

typedef struct{
	uint16_t  	angle;//rang:0~8191(0x1FFF)
	uint16_t  	last_angle;
	int16_t  	current_get;//rang:-13000 ~ 13000
	int16_t 	current_set;//rang:-13000 ~ 13000
	uint8_t     hall;//rang:0 ~ 6
	uint16_t  	null;
}Motor_6025_Typedef;
void low_pass_filter_init(LOW_PASS_FILTER_Typedef *filter_type, float filter_factor, float frame_period);
float low_pass_filter(LOW_PASS_FILTER_Typedef *filter_type, float input);

float Average_Filter(float *in_data, float data, uint8_t data_length);

float Angle_Calculate(float absolute_angle, float last_angle);
float invSqrt(float num);
float limit_control(float input, float limit);

#endif /* ALGORITHMS_LIB_ALGORITHMS_LIB_H_ */
