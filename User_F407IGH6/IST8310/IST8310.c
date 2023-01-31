/*
 * IST8310.c
 *
 *  Created on: Jan 22, 2023
 *      Author: Phoenix
 */
#include "IST8310.h"
extern I2C_HandleTypeDef hi2c3;
/**
  * @brief   写数据到IST8310寄存器
  * @param   reg_add:寄存器地址
	* @param	 reg_data:要写入的数据
  * @retval
  */
void IST8310_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
	HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS << 1, reg_add, I2C_MEMADD_SIZE_8BIT, &reg_dat, 1, 0xff);
}

/**
  * @brief   从IST8310寄存器读取数据
  * @param   reg_add:寄存器地址
	* @param	 *pdata:存储数据的缓存区
	* @param	 len:要读取的数据量
  * @retval
  */
void IST8310_ReadData(uint8_t reg_add,uint8_t *pdata,uint8_t len)
{
	HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1, reg_add, I2C_MEMADD_SIZE_8BIT, pdata, len, 0xff);
}

/**
  * @brief   读取mpu6050的ID
  * @retval  Status_OK正常 | Status_ERROR错误
  */
uint8_t IST8310_ReadID(void)
{
	uint8_t ID;
	IST8310_ReadData(IST8310_WHO_AM_I, &ID, 1);
	return ID;
}

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

void IST8310_Getdata_Mag_raw(int16_t *magdata)
{
	uint8_t buff[6];
	IST8310_ReadData(IST8310_DATA_XL_ADDR, buff, 6);
	magdata[0] = (int16_t)( (buff[1] << 8) | buff[0] );
	magdata[1] = (int16_t)( (buff[3] << 8) | buff[2] );
	magdata[2] = (int16_t)( (buff[5] << 8) | buff[4] );
}

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









