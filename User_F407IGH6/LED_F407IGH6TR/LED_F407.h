/*
 * LED_F407.h
 *
 *  Created on: Nov 26, 2022
 *      Author: Phoenix
 */

#ifndef LED_F407_H_
#define LED_F407_H_

#define LED_B_PORT GPIOH//蓝灯
#define LED_B GPIO_PIN_10

#define LED_G_PORT GPIOH//绿灯
#define LED_G GPIO_PIN_11

#define LED_R_PORT GPIOH//红灯
#define LED_R GPIO_PIN_12

#define LED_TEST()   HAL_GPIO_WritePin(GPIOH, LED_R, GPIO_PIN_SET)

#define LED_ON(X) HAL_GPIO_WritePin(GPIOH, X, GPIO_PIN_SET)
#define LED_OFF(X) HAL_GPIO_WritePin(GPIOH, X, GPIO_PIN_RESET)
#define LED_TOG(X) HAL_GPIO_TogglePin(GPIOH, X)
#endif /* LED_F407IGH6TR_LED_F407_H_ */
