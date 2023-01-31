/*
 * BMI088.c
 *
 *  Created on: Jan 28, 2023
 *      Author: Phoenix
 */

#include "BMI088.h"
//SPI片选宏函数
#define SPI_ACC_ENABLE() 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SPI_ACC_DISABLE() 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define SPI_GYRO_ENABLE() 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define SPI_GYRO_DISABLE() 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

/*--------BMI088基础读写函数--------*/
void BMI088_Acc_WriteReg(uint8_t addr, uint8_t data)
{
	SPI_ACC_ENABLE();

	uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
	HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_Delay(1);
	SPI_ACC_DISABLE();
}

void BMI088_Gyro_WriteReg(uint8_t addr, uint8_t data)
{
	SPI_GYRO_ENABLE();

	uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
	HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);

	SPI_GYRO_DISABLE();
}

void BMI088_Acc_ReadReg(uint8_t addr, uint8_t *pdata, uint8_t len)
{
	SPI_ACC_ENABLE();

	uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);

	HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_SPI_Receive(&hspi1, &pTxData, 1, 1000);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);
	HAL_SPI_Receive(&hspi1, pdata, len, 0XFFFF);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);

	SPI_ACC_DISABLE();

}

void BMI088_Gyro_ReadReg(uint8_t addr, uint8_t *pdata, uint8_t len)
{
	SPI_GYRO_ENABLE();

	uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
	HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
	HAL_SPI_Receive(&hspi1, pdata, len, 1000);

	SPI_GYRO_DISABLE();
}
/*--------BMI088数据读取函数--------*/
uint8_t BMI088_Acc_ReadID(void)
{
	uint8_t ID;
	BMI088_Acc_ReadReg(BMI088_ACC_CHIP_ID, &ID, 1);

	return ID;
}

uint8_t BMI088_Gyro_ReadID(void)
{
	uint8_t ID;
	BMI088_Gyro_ReadReg(BMI088_GYRO_CHIP_ID, &ID, 1);

	return ID;
}

float BMI088_Get_Temperature(void)
{
	uint8_t buff[2];
	uint16_t temp_uint11;
	int16_t temp_int11;

	BMI088_Acc_ReadReg(BMI088_TEMP_M, buff, 2);
	temp_uint11 = (buff[0] << 3) + (buff[1] >> 5);
	if (temp_uint11 > 1023)
	{
		temp_int11 = temp_uint11 - 2048;
	}
	else
	{
		temp_int11 = temp_uint11;
	}

	return (temp_int11 * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET);
}
//raw
void BMI088_Getdata_Acc_raw(int16_t *accdata)
{
	uint8_t buff[6];
	BMI088_Acc_ReadReg(BMI088_ACCEL_XOUT_L, buff, 6);
	accdata[0] = (int16_t)( (buff[1] << 8) | buff[0] );
	accdata[1] = (int16_t)( (buff[3] << 8) | buff[2] );
	accdata[2] = (int16_t)( (buff[5] << 8) | buff[4] );
}
//rad / s
void BMI088_Getdata_Gyro_raw(int16_t *gyrodata)
{
	uint8_t buff[6];
	BMI088_Gyro_ReadReg(BMI088_GYRO_X_L, buff, 6);
	gyrodata[0] = (int16_t)( (buff[1] << 8) | buff[0] );
	gyrodata[1] = (int16_t)( (buff[3] << 8) | buff[2] );
	gyrodata[2] = (int16_t)( (buff[5] << 8) | buff[4] );
}
//m_2/s
void BMI088_Getdata_Acc(float *accdata)
{
	uint8_t buff[6];
	int16_t tempbuff = 0;
	BMI088_Acc_ReadReg(BMI088_ACCEL_XOUT_L, buff, 6);
	tempbuff = (int16_t)( (buff[1] << 8) | buff[0] );
	accdata[0] = tempbuff * BMI088_ACCEL_3G_SEN;
	tempbuff = (int16_t)( (buff[3] << 8) | buff[2] );
	accdata[1] = tempbuff * BMI088_ACCEL_3G_SEN;
	tempbuff = (int16_t)( (buff[5] << 8) | buff[4] );
	accdata[2] = tempbuff * BMI088_ACCEL_3G_SEN;
}
//rad / s
void BMI088_Getdata_Gyro(float *gyrodata)
{
	uint8_t buff[6];
	int16_t tempbuff = 0;
	BMI088_Gyro_ReadReg(BMI088_GYRO_X_L, buff, 6);
	tempbuff = (int16_t)( (buff[1] << 8) | buff[0] );
	gyrodata[0] = tempbuff * BMI088_GYRO_2000_SEN;
	tempbuff = (int16_t)( (buff[3] << 8) | buff[2] );
	gyrodata[1] = tempbuff * BMI088_GYRO_2000_SEN;
	tempbuff = (int16_t)( (buff[5] << 8) | buff[4] );
	gyrodata[2] = tempbuff * BMI088_GYRO_2000_SEN;
}
/*--------BMI088初始化函数--------*/
IMU_ERROR_Typedef BMI088_Acc_Init(void)
{
	uint8_t i;
	uint8_t ID;
	uint8_t BMI088_Acc_Init_Data[6][2] = {
			{BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE},//软件复位，清空所用寄存器
			{BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON},//开启加速度计电源
			{BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE},//加速度正常工作模式
			{BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G},//设置范围为+-3G
			{BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set},//设置采样， 输出频率1600HZ
			{BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF},
	};
    ID = BMI088_Acc_ReadID();
    if (ID != BMI088_ACC_CHIP_ID_VALUE)
    {
    	HAL_Delay(100);
    	ID = BMI088_Acc_ReadID();
    }

    if (ID == BMI088_ACC_CHIP_ID_VALUE)
    {
    	BMI088_Acc_WriteReg(BMI088_Acc_Init_Data[0][0], BMI088_Acc_Init_Data[0][1]);
    	HAL_Delay(50);


    	for(i = 1; i < 5; i++)
    	{
    		BMI088_Acc_WriteReg(BMI088_Acc_Init_Data[i][0], BMI088_Acc_Init_Data[i][1]);
    		HAL_Delay(5);
    	}


    	return IMU_NO_ERROR;
    }
    else
    {
    	return IMU_ACC_ERROR;
    }

}

IMU_ERROR_Typedef BMI088_Gyro_Init(void)
{
	uint8_t i;
	uint8_t ID;
	uint8_t BMI088_Gyro_Init_Data[4][2] = {
			{BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE},//软件复位，清空所用寄存器
			{BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE},//陀螺仪正常工作模式
			{BMI088_GYRO_RANGE, BMI088_GYRO_2000},//设置范围为+-2000°/s
			{BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_532_HZ},//2000Hz输出频率，532Hz滤波器带宽
	};
    ID = BMI088_Gyro_ReadID();
    if (ID != BMI088_GYRO_CHIP_ID_VALUE)
    {
    	HAL_Delay(100);
    	ID = BMI088_Gyro_ReadID();
    }

    if (ID == BMI088_GYRO_CHIP_ID_VALUE)
    {
    	BMI088_Gyro_WriteReg(BMI088_Gyro_Init_Data[0][0], BMI088_Gyro_Init_Data[0][1]);
    	HAL_Delay(50);
    	for(i = 1; i < 4; i++)
    	{
    		BMI088_Gyro_WriteReg(BMI088_Gyro_Init_Data[i][0], BMI088_Gyro_Init_Data[i][1]);
    		HAL_Delay(5);
    	}

    	return IMU_NO_ERROR;
    }
    else
    {
    	return IMU_GYRO_ERROR;
    }
}



IMU_ERROR_Typedef BMI088_Init(void)
{
	uint8_t bmi088_error = IMU_NO_ERROR;
	bmi088_error |= BMI088_Gyro_Init();
	bmi088_error |= BMI088_Acc_Init();
	return bmi088_error;
}
