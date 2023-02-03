/*
 * IST8310.c
 *
 *  Created on: Jan 22, 2023
 *      Author: Phoenix
 */
#include "IST8310.h"
extern I2C_HandleTypeDef hi2c3;


/*----------------------IST8310基础寄存器读写函数----------------------*/

/**
  * @brief :向IST8310磁力计写入数据
  * @param :reg_addrr :要写入数据的IST8310磁力计中的寄存器地址
  * @param :reg_dataa :要写入的数据
  * @note  :此函数是基于HAL库的HAL_I2C_Mem_Write函数，使用I2C3外设，请确定是否已经配置完成
  * @retval:无
  */
void IST8310_WriteReg(uint8_t reg_addr,uint8_t reg_data)
{
	HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 0xff);
}

/**
  * @brief :从IST8310寄存器读取数据
  * @param :reg_addr :寄存器地址
  * @param :*pdata :存储数据的缓存区
  * @param :len :要读取的数据量
  * @note  :此函数是基于HAL库的HAL_I2C_Mem_Write函数，使用I2C3外设，请确定是否已经配置完成
  * @retval:无
  */
void IST8310_ReadData(uint8_t reg_addr,uint8_t *pdata,uint8_t len)
{
	HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, len, 0xff);
}

/*-------------------IST8310数据读取函数部分-------------------*/

/**
  * @brief :从IST8310中读取其ID值，用以验证磁力计是否正常工作以及通信是否正常
  * @param :无
  * @note  :此函数使用了宏定义的寄存器地址，请确保已有IST8310_reg.h文件并已包含在此C文件中
  * @retval:IST8310磁力计的ID值
  */
uint8_t IST8310_ReadID(void)
{
	uint8_t ID;
	IST8310_ReadData(IST8310_WHO_AM_I, &ID, 1);
	return ID;
}

/**
  * @brief :从IST8310磁力计中读取各轴磁力寄存器原始数据
  * @param :*magdata: 存放X,Y,Z三轴磁力的数组地址
  * @note  :1、此函数使用前需初始化IST8310加速度计，否则读取的磁力将不会变
  * 		2、magdata[]数组中数据依次为X轴磁力大小，Y轴磁力大小,Z轴磁力大小
  * @retval:无
  */
void IST8310_Getdata_Mag_raw(int16_t *magdata)
{
	uint8_t buff[6];
	IST8310_ReadData(IST8310_DATA_XL_ADDR, buff, 6);
	magdata[0] = (int16_t)( (buff[1] << 8) | buff[0] );
	magdata[1] = (int16_t)( (buff[3] << 8) | buff[2] );
	magdata[2] = (int16_t)( (buff[5] << 8) | buff[4] );
}

/**
  * @brief :从IST8310磁力计中读取各轴磁力寄存器并处理数据
  * @param :*magdata: 存放X,Y,Z三轴磁力的数组地址
  * @note  :1、此函数使用前需初始化IST8310加速度计，否则读取的磁力将不会变
  * 		2、magdata[]数组中数据依次为X轴磁力大小，Y轴磁力大小,Z轴磁力大小
  * 		3、处理后的数据单位为磁力的国际单位制 uT(微特斯拉）
  * @retval:无
  */
void IST8310_Getdata_Mag(float *magdata)
{
	uint8_t buff[6];
	int16_t tempbuff = 0;
	IST8310_ReadData(IST8310_DATA_XL_ADDR, buff, 6);
	tempbuff = (int16_t)( (buff[1] << 8) | buff[0] );
	magdata[0] = tempbuff * MAG_SEN;
	tempbuff = (int16_t)( (buff[3] << 8) | buff[2] );
	magdata[1] = tempbuff * MAG_SEN;
	tempbuff = (int16_t)( (buff[5] << 8) | buff[4] );
	magdata[2] = tempbuff * MAG_SEN;

}

/*-------------------IST8310初始化函数部分-------------------*/

/**
  * @brief :IST8310磁力计初始化函数
  * @param :无
  * @note  :1、IMU_ERROR_Typedef 类型在Algorithms_Lib.h中被定义
  *         2、初始化C板IST8310会用到GPIOG的GPIO_PIN_6引脚，请提前配置好
  * @retval:IMU_ERROR_Typedef 类型变量，根据此变量可确定磁力计初始化是否成功
  */
IMU_ERROR_Typedef IST8310_Init(void)
{
	if(IST8310_ReadID() == IST8310_WHO_AM_I_VALUE)
	{
		//重启IST8310
		HAL_GPIO_WritePin(GPIOG, IST8310_RST_PIN, GPIO_PIN_RESET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOG, IST8310_RST_PIN, GPIO_PIN_SET);
		HAL_Delay(50);
		//不开启中断
		IST8310_WriteReg(IST8310_CNTL2_ADDR, 0x00);
		HAL_Delay(150);
		//四次采样平均
		IST8310_WriteReg(IST8310_AVGCNTL_ADDR, IST8310_AVGCNTL_FOURTH);
		HAL_Delay(150);
		//连续采样，200HZ输出模式
		IST8310_WriteReg(IST8310_CNTL1_ADDR, IST8310_CNTL1_CONTINUE);
		HAL_Delay(150);

    	return IMU_NO_ERROR;
	}
	else
	{
		return IMU_MAG_ERROR;
	}
}










