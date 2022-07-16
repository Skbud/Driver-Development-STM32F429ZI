/*
 * stm32f429zi_gpio_driver.c
 *
 *  Created on: 23-Jun-2022
 *      Author: Sanjay
 */


/* GPIO INTERRUPT FUNCTION HAVE SOME BUGS */

#include "stm32f429zi_gpio_driver.h"

/***********************************************************************************************
 * @Function:							-GPIO_PeriClockControl
 * @Description:						-This function will enable/disable the peripheral clock for the gpio
 * @Parameter[pGPIOx]			-Holds base address of the gpio
 * @Parameter[EnorDi]			-Holds the selection(Enable/Disable) for the gpio. //these are macros
 * @Parameter[]
 * @return								-None
 * @special note:						-None
 */

void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(pGPIOx==GPIOA){
				GPIOA_CLK_EN();
			}else if(pGPIOx==GPIOB){
				GPIOB_CLK_EN();
			}else if(pGPIOx==GPIOC){
				GPIOC_CLK_EN();
			}else if(pGPIOx==GPIOD){
				GPIOD_CLK_EN();
			}else if(pGPIOx==GPIOE){
				GPIOE_CLK_EN();
			}else if(pGPIOx==GPIOF){
				GPIOF_CLK_EN();
			}else if(pGPIOx==GPIOG){
				GPIOG_CLK_EN();
			}else if(pGPIOx==GPIOH){
				GPIOH_CLK_EN();
			}else if(pGPIOx==GPIOI){
				GPIOI_CLK_EN();
			}
	}else
	{
			if(pGPIOx==GPIOA){
				GPIOA_CLK_DI();
				}else if(pGPIOx==GPIOB){
					GPIOB_CLK_DI();
				}else if(pGPIOx==GPIOC){
					GPIOC_CLK_DI();
				}else if(pGPIOx==GPIOD){
					GPIOD_CLK_DI();
				}else if(pGPIOx==GPIOE){
					GPIOE_CLK_DI();
				}else if(pGPIOx==GPIOF){
					GPIOF_CLK_DI();
				}else if(pGPIOx==GPIOG){
					GPIOG_CLK_DI();
				}else if(pGPIOx==GPIOH){
					GPIOH_CLK_DI();
				}else if(pGPIOx==GPIOI){
					GPIOI_CLK_DI();
				}
	}

}



/***********************************************************************************************
 * @Function:							-GPIO_Init
 * @Description:						-This function will initilize the gpio with the parameter defined in GPIOHandle structure
 * @Parameter[pGPIOHandle]-Holds settings
 * @return								-None
 * @special note:						-Needs to be called only after setting up proper handle structure
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	//enable clock
	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);


	uint32_t temp=0;
//Mode Configuration
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_ANALOG_MODE){       //After analog mode(See @modes) all are interrupt mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 		 //multiplying by 2 because each pin needs 2bit field in modeR reg
		pGPIOHandle->pGPIOx->MODER		&=~((0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 								//clearing
		pGPIOHandle->pGPIOx->MODER 	|=temp;																															//Setting
		temp=0;
	}


	else{     //Interrupt mode

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_INT_FT) {
			//configure Falling trigger selection register (EXTI_FTSR)
			EXTI->FTSR		|=			(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR		&=	~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);						//Clearing the rising edge trigger on selected pin.

				} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_INT_RT){
					//Configure Rising trigger selection register (EXTI_RTSR)
					EXTI->RTSR		|=			(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					EXTI->FTSR	&=	~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);						//Clearing the falling  edge trigger on selected pin.

					}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_INT_RFT){
						//Configure both EXTI_FTSR and EXTI_RTSR
						EXTI->FTSR		|=			(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
						EXTI->RTSR		|=			(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					}

		//Configure GPIO Port selection in SYSCFG_EXTICR registers
		uint8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode=GPIO_BASEADD_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[temp1]=portcode<<(4*temp2);

		//Enable EXTI Interrupt Delivery using Interrupt Mask Register(EXTI_IMR)
		EXTI->IMR	|=			(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

}
//Speed Configuration
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR	&=~((0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;
			temp=0;

//Pull up/Pull down settings
			temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->PUPDR	&=~((0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->PUPDR	|=temp;
			temp=0;


//Output type
			temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->OTYPER		&=~((0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->OTYPER		|=temp;
			temp=0;
//Alternate Function
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode	==GPIO_ATLFN_MODE){
				uint8_t temp1,temp2;
				temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;								//To select the Alternate function register AF[0] or AF[1]
				temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;							//To get the bit position which needs to be programmed (4 bits )
				pGPIOHandle->pGPIOx->AFR[temp1]	&=~(0xF<<(4*temp2));							//Clearing
				pGPIOHandle->pGPIOx->AFR[temp1]	|=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFn<<(4*temp2));

			}



}






/***********************************************************************************************
 * @Function:							-GPIO_DeInit
 * @Description:						-This function will reset the gpio port to default values
 * @Parameter[pGPIOx]			-Holds base address of the gpio
 * @return								-None
 * @special note:						-None
 */
void GPIO_DeInit(GPIO_Reg_Def_t  	*pGPIOx){

			if(pGPIOx==GPIOA){
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOB){
				GPIOB_REG_RESET();
			}else if(pGPIOx==GPIOC){
				GPIOC_REG_RESET();
			}else if(pGPIOx==GPIOD){
				GPIOD_REG_RESET();
			}else if(pGPIOx==GPIOE){
				GPIOE_REG_RESET();
			}else if(pGPIOx==GPIOF){
				GPIOF_REG_RESET();
			}else if(pGPIOx==GPIOG){
				GPIOG_REG_RESET();
			}else if(pGPIOx==GPIOH){
				GPIOH_REG_RESET();
			}else if(pGPIOx==GPIOI){
				GPIOI_REG_RESET();
			}

}




/***********************************************************************************************
 * @Function:							-GPIO_ReadPin
 * @Description:						-This function will read data from the gpio pin
 * @Parameter[pGPIOx]			-Holds base address of the gpio
 * @Parameter[PinNumber]	-Holds the pin number
 * @return								-Data received on gpio pin
 * @special note:						-None
 */
uint8_t  GPIO_ReadPin(GPIO_Reg_Def_t  	*pGPIOx, 	uint8_t PinNumber){
		uint8_t value;
		value=(uint8_t)((pGPIOx->IDR>>PinNumber)	&	0x00000001);		//Right shifting the bit by the given pin number so that the data lands at 0th position and then masking and saving the results
		return value;
}




/***********************************************************************************************
 * @Function:							-GPIO_ReadPort
 * @Description:						-This function will read data from entire port
 * @Parameter[pGPIOx]			-Holds base address of the gpio
 *
 * @return								-Data received on port
 * @special note:						-None
 */
uint16_t  GPIO_ReadPort(GPIO_Reg_Def_t  		*pGPIOx){
			uint16_t value;
			value=(uint16_t)(pGPIOx->IDR);
			return value;
}





/***********************************************************************************************
 * @Function:							-GPIO_WritePin
 * @Description:						-This function will write data to the gpio pin
 * @Parameter[pGPIOx]			-Holds base address of the gpio
 * @Parameter[PinNumber]	-Holds the pin number on which data is written
 * @Parameter[Value]				-Data to be written on pin
 * @return								-None
 * @special note:						-None
 */
void GPIO_WritePin(GPIO_Reg_Def_t  *pGPIOx, 	uint8_t PinNumber, 	uint8_t Value){
	if(Value==GPIO_PIN_SET){
		pGPIOx->ODR	|=(1<<PinNumber);
	}else{
		pGPIOx->ODR	&=	~(1<<PinNumber);
	}
}



/***********************************************************************************************
 * @Function:							-GPIO_WritePort
 * @Description:						-This function will write data to entire port
 * @Parameter[pGPIOx]			-Holds base address of the gpio
 * @Parameter[Value]				-Data to be written on the port
 * @Parameter[]
 * @return								-None
 * @special note:						-None
 */
void GPIO_WritePort(GPIO_Reg_Def_t  	*pGPIOx, 	uint16_t Value){
	pGPIOx->ODR=Value;
}




/***********************************************************************************************
 * @Function:							-GPIO_TogglePin
 * @Description:						-This function will toggle the output of the selected gpio pin
 * @Parameter[pGPIOx]			-Holds base address of the gpio
 * @Parameter[PinNumber]	-Holds the Pin which is to be toggled
 * @Parameter[]
 * @return								-None
 * @special note:						-None
 */
void GPIO_TogglePin(GPIO_Reg_Def_t  	*pGPIOx, 	uint8_t PinNumber){
	pGPIOx->ODR	^=	(1<<PinNumber);
}






/***********************************************************************************************
 * @Function:							-GPIO_IRQConfig
 * @Description:						-This function will configure the interrupt on the selected pin(This Function will configure the NVIC registers)
 * @Parameter[IRQNumber]	-Holds the interrupt number(IRQ number)
 * @Parameter[EnorDi]			-Holds the selection(Enable/Disable) for the gpio. //these are macros
 * @return								-None
 * @special note:						-None
 */
void GPIO_IRQConfig(uint8_t IRQNumber,	uint8_t EnorDi){
	if(EnorDi==ENABLE){
			if(IRQNumber<=31){
				//Set ISER0
				*NVIC_ISER0	|=(1<<IRQNumber);
				}else if(IRQNumber>31	&&	IRQNumber<64){
					//Set ISER1
					*NVIC_ISER1	|=(1<<(IRQNumber%32));
				}else if(IRQNumber>=31	&&	IRQNumber<96){
					//Set ISER2
					*NVIC_ISER2	|=(1<<(IRQNumber%64));
				}
				else if(IRQNumber>=96	&&	IRQNumber<101){
					//Set ISER3
					*NVIC_ISER3	|=(1<<(IRQNumber%96));
				}
	}else{
			if(IRQNumber<=31){
				//Set ICER0
					*NVIC_ICER0	|=(1<<IRQNumber);
				}else if(IRQNumber>31	&&	IRQNumber<64){
					//Set ICER1
						*NVIC_ICER1	|=(1<<(IRQNumber%32));
				}else if(IRQNumber>=64	&&	IRQNumber<96){
					//Set ICER2
						*NVIC_ICER2	|=(1<<(IRQNumber%64));

				}else if(IRQNumber>=96	&&	IRQNumber<101){
					//Set ICER3
						*NVIC_ICER3	|=(1<<(IRQNumber%96));
				}
	}

	}



/***********************************************************************************************
 * @Function:							-GPIO_IRQPrority_Config
 * @Description:						-This function is used to set the priority of the interrupt
 *@Parameter[IRQNumber]	-Holds the interrupt number(IRQ number)
 * @Parameter[IRQPriority]	-Holds the interrupt priority for the selected pin
 * @return								-None
 * @special note:						-
 */
void GPIO_IRQPrority_Config(uint8_t IRQNumber, 	uint32_t IRQPriority){
	//Finding IPR register
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	uint8_t shift_amount=	((8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED));
	*(NVIC_PR_BASE_ADD+(iprx))	|=	(IRQPriority<<shift_amount);

}



/***********************************************************************************************
 * @Function:							-GPIO_IRQHandling
 * @Description:						-This function will be triggered when an interrupt happens on the selected pin
 * 											The purpose of this function is to reset the pr register(nvic side)
 * @Parameter[PinNumber]			-Holds pin number on which interrupt is enabled
 * @return								-None
 * @special note:						-Can only be used after setting interrupt on the pin using IRQConfig function
 */
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR	&	(1<<PinNumber)){
		//clear
		EXTI->PR	|=(1<<PinNumber);
	}

}

