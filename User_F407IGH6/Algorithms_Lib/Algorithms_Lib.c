/*
 * Algorithms_Lib.c
 *
 *  Created on: 2022年9月22日
 *      Author: Phonix
 */
#include "Algorithms_Lib.h"



/**
  * @brief          低通滤波初始化
  * @author         Guo_Hui_Lin
  * @param[in]      一阶低通滤波结构体
  * @param[in]    	滤波系数
  * @param[in]    	运算时间间隔
  * @retval         返回空
  */
void low_pass_filter_init(LOW_PASS_FILTER_Typedef *filter_type, float filter_factor, float frame_period)
{
	filter_type->filter_factor = filter_factor;
	filter_type->frame_period = frame_period;
}

/**
  * @brief          低通滤波计算
  * @author         Guo_Hui_Lin
  * @param[in]      一阶低通滤波结构体
  * @param[in]    	输入
  * @retval         滤波后数据
  */
float low_pass_filter(LOW_PASS_FILTER_Typedef *filter_type, float input)
{
	filter_type->output =
			input * filter_type->frame_period / (filter_type->filter_factor + filter_type->frame_period)
		+	filter_type->output * filter_type->filter_factor / (filter_type->filter_factor + filter_type->frame_period);
	return filter_type->output;

}

/**
  * @brief          平均值滤波计算
  * @author         Guo_Hui_Lin
  * @param[in]      数据数组
  * @param[in]		当前数据
  * @param[in]		采样个数
  * @retval         平均值
  */
float Average_Filter(float *in_data, float data, uint8_t data_length)
{

	float sum = 0;

	for (uint8_t i = 0; i < data_length - 1; i++)
	{
		in_data[i] = in_data[i+1];
		sum = sum + in_data[i];
	}

	in_data[data_length - 1] = data;
	sum = sum + in_data[data_length - 1];

	return (sum / data_length);
}

/**
  * @brief          相对角度计算
  * @author         Guo_Hui_Lin
  * @param[in]      电机角度
  * @param[in]      上次电机角度
  * @retval         相对角度增量[-180, 180]
  */
float Angle_Calculate(float absolute_angle, float last_angle)
{
	float res1, res2, err;

	err = absolute_angle - last_angle;
	if(err < 0)
	{
		res1 = err;
		res2 = err + PI;
	}
	else
	{
		res1 = err;
		res2 = err - PI;
	}
	if(fabs(res1) < fabs(res2))
	{
		return res1;
	}
	else
	{
		return res2;
	}
}

/**
  * @brief          快速开方
  * @author         Guo_Hui_Lin
  * @param[in]      输入
  * @retval         输入值开方的倒数
  */
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief  控制输入参数的绝对值不超过限幅值
  * @param	需要限幅的变量
  * @param	限幅值
  * @retval 限幅值之内的值
  */
float limit_control(float input, float limit)
{
	float limit_output = input;
	if(input > limit) limit_output = limit;
	else if(input < (-1 * limit)) limit_output = (-1 * limit);

	return limit_output;
}
