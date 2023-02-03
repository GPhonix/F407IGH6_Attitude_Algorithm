/*
 * BMI088.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Phoenix
 */

#ifndef BMI088_H_
#define BMI088_H_
#include "main.h"
#include "Algorithms_Lib.h"
#include "BMI088_reg.h"
#include "spi.h"

/*--------BMI088的SPI读取协议部分--------*/
#define BMI088_SPI_WRITE_CODE 0x7F//0111 1111
#define BMI088_SPI_READ_CODE 0x80//1000 0000
//温度计算宏
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f
//加速度原始数据单位转换(G/S)转(M^2/S)
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

//陀螺仪原始数据单位转换 弧度制(rad/s)
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f


IMU_ERROR_Typedef BMI088_Init(void);
uint8_t BMI088_Acc_ReadID(void);
float BMI088_Get_Temperature(void);
void BMI088_Getdata_Acc_raw(int16_t *accdata);
void BMI088_Getdata_Gyro_raw(int16_t *gyrodata);
void BMI088_Getdata_Acc(float *accdata);
void BMI088_Getdata_Gyro(float *gyrodata);

#endif /* BMI088_H_ */
