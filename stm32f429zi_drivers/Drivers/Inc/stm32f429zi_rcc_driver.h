/*
 * stm32f429zi_rcc_driver.h
 *
 *  Created on: 12-Sep-2022
 *      Author: Sanjay
 */

#ifndef INC_STM32F429ZI_RCC_DRIVER_H_
#define INC_STM32F429ZI_RCC_DRIVER_H_

#include "STM32F429ZI.h"
//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F429ZI_RCC_DRIVER_H_ */
