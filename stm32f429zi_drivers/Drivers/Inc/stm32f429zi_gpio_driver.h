/*
 * stm32f429zi_gpio_driver.h
 *
 *  Created on: 23-Jun-2022
 *      Author: Sanjay
 */

#ifndef STM32F429ZI_GPIO_DRIVER_H_
#define STM32F429ZI_GPIO_DRIVER_H_

#include "STM32F429ZI.h"
/***************PIN NUMBER******************///@pinnumber
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15

/******************INPUT MODES*****************/  //@modes
#define GPIO_INPUT_MODE			0
#define GPIO_OUTPUT_MODE		1
#define GPIO_ATLFN_MODE			2					//Alternate function Mode
#define GPIO_ANALOG_MODE		3
#define GPIO_INT_FT						4					//Interrupt Falling Edge
#define GPIO_INT_RT						5					//Interrupt Rising Edge
#define GPIO_INT_RFT					6					//Interrupt Rising & Falling Edge


/******************OUTPUT MODES**************/ //@output
#define GPIO_OUTPUT_PP				0					//Push Pull mode
#define GPIO_OUTPUT_OD			1					//Open Drain



/*******************SPEED MODE*****************/ //@speed
#define GPIO_SPEED_LOW			0					//Low speed
#define GPIO_SPEED_MEDIUM		1					//Medium speed
#define GPIO_SPEED_HIGH			2					//High speed
#define GPIO_SPEED_VERYHIGH	3					//Very high speed

/*******************PULL UP/PULL DOWN**************/ //@pullup
#define GPIO_PIN_NO_PUPD				0					//NO PULL UP PULL DOWN
#define GPIO_PIN_PULLUP					1					//Pull up
#define GPIO_PIN_PULLDOWN				2				//Pull down



typedef struct
{
	uint8_t GPIO_PinNumber;					//Valid Types @pinnumber
	uint8_t GPIO_PinMode;						//Valid Types @modes
	uint8_t GPIO_PinSpeed;						//Valid Types @speed
	uint8_t GPIO_PinPuPdControl;				//Valid Type @pullup
	uint8_t GPIO_PinOutType;					//Valid Types @output
	uint8_t GPIO_PinAltFn;
}GPIO_Pin_Config_t;



/***********************HANDLE STRUCTURE FOR GPIO**************************************************/
typedef struct
{
	GPIO_Reg_Def_t  		*pGPIOx;														//Holds the base address of the GPIO Port of the selected pin.
	GPIO_Pin_Config_t		GPIO_PinConfig;											//Holds Pin configuration settings.
}GPIO_Handle_t;

/****************************************************************************************************************************************************************
 * 																							SUPPORTED API																															*
 ****************************************************************************************************************************************************************/
void GPIO_PeriClockControl(GPIO_Reg_Def_t  		*pGPIOx, 	uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

void GPIO_DeInit(GPIO_Reg_Def_t  	*pGPIOx);

uint8_t  GPIO_ReadPin(GPIO_Reg_Def_t  	*pGPIOx, 	uint8_t PinNumber);

uint16_t  GPIO_ReadPort(GPIO_Reg_Def_t  		*pGPIOx);

void GPIO_WritePin(GPIO_Reg_Def_t  *pGPIOx, 	uint8_t PinNumber, 	uint8_t Value);

void GPIO_WritePort(GPIO_Reg_Def_t  	*pGPIOx, 	uint16_t Value);

void GPIO_TogglePin(GPIO_Reg_Def_t  	*pGPIOx, 	uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

void GPIO_IRQPrority_Config(uint8_t IRQNumber, 	uint32_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);









#endif /* STM32F429ZI_GPIO_DRIVER_H_ */
