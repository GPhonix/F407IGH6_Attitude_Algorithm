/*
 * IMU_C.h
 *
 *  Created on: 2023年1月30日
 *      Author: Phoenix
 */

#ifndef IMU_C_IMU_C_H_
#define IMU_C_IMU_C_H_
#include "IST8310.h"
#include "BMI088.h"
#include "PID_primary.h"
#include "math.h"

#define IMU_TEMP_PWM_MAX  999U //IMU控制温度的设置TIM的重载值，即给PWM最大为IMU_TEMP_PWM_MAX
#define IMU_MAX_TEMP_SET 40.0f//IMU温度设定最大值


typedef struct{
	int16_t ax_raw;
	int16_t ay_raw;
	int16_t az_raw;
	int16_t gx_raw;
	int16_t gy_raw;
	int16_t gz_raw;
	int16_t mx_raw;
	int16_t my_raw;
	int16_t mz_raw;

	float gx_offset;
	float gy_offset;
	float gz_offset;

	float temperature;
	uint8_t acc_id;
	uint8_t gyr_id;
	uint8_t mag_id;

	float roll;//rad
	float pitch;
	float yaw;

	struct{
		float q0;
		float q1;
		float q2;
		float q3;
	}quat;
}IMU_Typedef;









IMU_ERROR_Typedef IMU_Init(void);
void IMU_Temperature_Compensate(void);
void IMU_Data_Fusion_Mahony(float dt, float *roll, float *pitch, float *yaw);

#endif /* IMU_C_IMU_C_H_ */
