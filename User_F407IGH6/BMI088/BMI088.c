/*
 * BMI088.c
 *
 *  Created on: Jan 28, 2023
 *      Author: Phoenix
 */

#include "BMI088.h"

/*----------------------SPI片选宏函数----------------------*/
#define SPI_ACC_ENABLE() 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SPI_ACC_DISABLE() 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define SPI_GYRO_ENABLE() 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define SPI_GYRO_DISABLE() 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

/*----------------------BMI088基础寄存器读写函数----------------------*/

/************** 特别说明 :HAL_SPI_Transmit()函数和HAL_SPI_Receive()函数的等待时间不能太短如0xff,否则会通信失败
 *
 */
/**
  * @brief :向BMI088加速度计写入数据
  * @param :reg_reg_addrrr :要写入数据的BMI088加速度计中的地址
  * @param :data :要写入的数据
  * @note  :此函数是基于HAL库的HAL_SPI_Transmit()函数，使用SPI1，请确定是否已经配置完成
  * @retval:无
  */
void BMI088_Acc_WriteReg(uint8_t reg_addrr, uint8_t data)
{
	SPI_ACC_ENABLE();

	uint8_t pTxData = (reg_addrr & BMI088_SPI_WRITE_CODE);
	HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_Delay(1);

	SPI_ACC_DISABLE();
}

/**
  * @brief :向BMI088陀螺仪计写入数据
  * @param :reg_reg_addrrr :要写入数据的BMI088陀螺仪中的地址
  * @param :data :要写入的数据
  * @note  :此函数是基于HAL库的HAL_SPI_Transmit()函数，使用SPI1，请确定是否已经配置完成
  * @retval:无
  */
void BMI088_Gyro_WriteReg(uint8_t reg_addrr, uint8_t data)
{
	SPI_GYRO_ENABLE();

	uint8_t pTxData = (reg_addrr & BMI088_SPI_WRITE_CODE);
	HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);

	SPI_GYRO_DISABLE();
}

/**
  * @brief :从BMI088加速度计中读取数据
  * @param :reg_reg_addrrr :要读取数据的BMI088加速度计中的地址
  * @param :*pdata :存放数据的地址
  * @param :len :读取数据的字节数
  * @note  :此函数是基于HAL库的HAL_SPI_Receive()函数，使用SPI1，请确定是否已经配置完成
  * @retval:无
  */
void BMI088_Acc_ReadReg(uint8_t reg_addrr, uint8_t *pdata, uint8_t len)
{
	SPI_ACC_ENABLE();

	uint8_t pTxData = (reg_addrr | BMI088_SPI_READ_CODE);

	HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
	HAL_SPI_Receive(&hspi1, &pTxData, 1, 1000);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);
	HAL_SPI_Receive(&hspi1, pdata, len, 0XFFFF);
//	while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);

	SPI_ACC_DISABLE();
}

/**
  * @brief :从BMI088陀螺仪中读取数据
  * @param :reg_reg_addrrr :要读取数据的BMI088陀螺仪中的地址
  * @param :*pdata :存放数据的地址
  * @param :len :读取数据的字节数
  * @note  :此函数是基于HAL库的HAL_SPI_Receive()函数，使用SPI1，请确定是否已经配置完成
  * @retval:无
  */
void BMI088_Gyro_ReadReg(uint8_t reg_addrr, uint8_t *pdata, uint8_t len)
{
	SPI_GYRO_ENABLE();

	uint8_t pTxData = (reg_addrr | BMI088_SPI_READ_CODE);
	HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
	HAL_SPI_Receive(&hspi1, pdata, len, 1000);

	SPI_GYRO_DISABLE();
}

/*-------------------BMI088数据读取函数部分-------------------*/

/**
  * @brief :从BMI088加速度计中读取其ID值，用以验证加速度计是否正常工作以及通信是否正常
  * @param :无
  * @note  :此函数使用了宏定义的寄存器地址，请确保已有BMI088_reg.h文件并已包含在此C文件中
  * @retval:BMI088加速度计的ID值
  */
uint8_t BMI088_Acc_ReadID(void)
{
	uint8_t ID;
	BMI088_Acc_ReadReg(BMI088_ACC_CHIP_ID, &ID, 1);

	return ID;
}

/**
  * @brief :从BMI088陀螺仪中读取其ID值，用以验证陀螺仪是否正常工作以及通信是否正常
  * @param :无
  * @note  :此函数使用了宏定义的寄存器地址，请确保已有BMI088_reg.h文件并已包含在此C文件中
  * @retval:BMI088陀螺仪的ID值
  */
uint8_t BMI088_Gyro_ReadID(void)
{
	uint8_t ID;
	BMI088_Gyro_ReadReg(BMI088_GYRO_CHIP_ID, &ID, 1);

	return ID;
}

/**
  * @brief :从BMI088加速度计中读取温度值
  * @param :无
  * @note  :此函数使用前需初始化BMI088加速度计,否则读取的温度值将不会变
  * @retval:BMI088芯片环境温度，单位摄氏度(°C),精度0.125°C,此值每1.28秒更新一次
  */
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

/**
  * @brief :从BMI088加速度计中读取各轴加速度寄存器原始数据
  * @param :*accdata: 存放X,Y,Z三轴加速度的数组地址
  * @note  :1、此函数使用前需初始化BMI088加速度计，否则读取的加速度将不会变
  * 		2、accdata[]数组中数据依次为X轴加速度，Y轴加速度,Z轴加速度
  * @retval:无
  */
void BMI088_Getdata_Acc_raw(int16_t *accdata)
{
	uint8_t buff[6];
	BMI088_Acc_ReadReg(BMI088_ACCEL_XOUT_L, buff, 6);
	accdata[0] = (int16_t)( (buff[1] << 8) | buff[0] );
	accdata[1] = (int16_t)( (buff[3] << 8) | buff[2] );
	accdata[2] = (int16_t)( (buff[5] << 8) | buff[4] );
}

/**
  * @brief :从BMI088陀螺仪中读取各轴角速度寄存器原始数据
  * @param :*gyrodata: 存放X,Y,Z三轴角速度的数组地址
  * @note  :1、此函数使用前需初始化BMI088陀螺仪，否则读取的加速度将不会变
  * 		2、accdata[]数组中数据依次为X轴角速度，Y轴角速度,Z轴角速度
  * @retval:无
  */
void BMI088_Getdata_Gyro_raw(int16_t *gyrodata)
{
	uint8_t buff[6];
	BMI088_Gyro_ReadReg(BMI088_GYRO_X_L, buff, 6);
	gyrodata[0] = (int16_t)( (buff[1] << 8) | buff[0] );
	gyrodata[1] = (int16_t)( (buff[3] << 8) | buff[2] );
	gyrodata[2] = (int16_t)( (buff[5] << 8) | buff[4] );
}

/**
  * @brief :从BMI088加速度计中读取各轴加速度寄存器并处理数据
  * @param :*accdata: 存放X,Y,Z三轴加速度的数组地址
  * @note  :1、此函数使用前需初始化BMI088加速度计，否则读取的加速度将不会变
  * 		2、accdata[]数组中数据依次为X轴加速度，Y轴加速度,Z轴加速度
  * 		3、处理后的数据单位为加速度的国际单位制 m*m/s
  * 		4、此时加速度计采样范围+-3G
  * @retval:无
  */
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

/**
  * @brief :从BMI088陀螺仪计中读取各轴角速度寄存器并处理数据
  * @param :*gyrodata: 存放X,Y,Z三轴角速度的数组地址
  * @note  :1、此函数使用前需初始化BMI088陀螺仪，否则读取的加速度将不会变
  * 		2、accdata[]数组中数据依次为X轴角速度，Y轴角速度,Z轴角速度
  * 		3、处理后的数据单位为角速度的弧度制 rad / s
  * 		4、此时陀螺仪采样范围+-2000°
  * @retval:无
  */
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

/*-------------------BMI088初始化函数部分-------------------*/

/**
  * @brief :BMI088加速度计初始化函数
  * @param :无
  * @note  :IMU_ERROR_Typedef 类型在Algorithms_Lib.h中被定义
  * @retval:IMU_ERROR_Typedef 类型变量，根据此变量可确定加速度计初始化是否成功
  */
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

/**
  * @brief :BMI088陀螺仪初始化函数
  * @param :无
  * @note  :IMU_ERROR_Typedef 类型在Algorithms_Lib.h中被定义
  * @retval:IMU_ERROR_Typedef 类型变量，根据此变量可确定陀螺仪初始化是否成功
  */
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

/**
  * @brief :BMI088总初始化函数
  * @param :无
  * @note  :IMU_ERROR_Typedef 类型在Algorithms_Lib.h中被定义
  * @retval:IMU_ERROR_Typedef 类型变量，根据此变量可确定陀螺仪和加速度计初始化是否成功
  */
IMU_ERROR_Typedef BMI088_Init(void)
{
	IMU_ERROR_Typedef bmi088_error = IMU_NO_ERROR;
	bmi088_error |= BMI088_Gyro_Init();
	bmi088_error |= BMI088_Acc_Init();

	return bmi088_error;
}
