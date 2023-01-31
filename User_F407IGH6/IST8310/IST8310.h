/*
 * IST8310.h
 *
 *  Created on: Jan 22, 2023
 *      Author: Phoenix
 */
#ifndef IST8310_H_
#define IST8310_H_
#include "main.h"
#include "IST8310_reg.h"
#include "Algorithms_Lib.h"

#define IST8310_GPIOx GPIOG
#define IST8310_RST_PIN GPIO_PIN_6
#define IST8310_DRDY_PIN GPIO_PIN_3

#define MAG_SEN 0.3f



uint8_t IST8310_ReadID(void);
IMU_ERROR_Typedef IST8310_Init(void);
void IST8310_Getdata_Mag_raw(int16_t *magdata);

void IST8310_Getdata_Mag(float *magdata);

#endif /*IST8310_H_*/

