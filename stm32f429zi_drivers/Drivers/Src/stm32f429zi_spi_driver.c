/*
 * stm32f429zi_spi_driver.c
 *
 *  Created on: 01-Jul-2022
 *      Author: pc
 */


#include "stm32f429zi_spi_driver.h"
#include "STM32F429ZI.h"
static void  spi_txe_interrupt_handle(SPI_Handle_t	 *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t	 *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t	 *pSPIHandle);





/***********************************************************************************************
 * @Function:							-SPI_GetFlagStatus
 * @Description:						-Gives status of the reqested flag
 * @Parameter[pSPIx]				-Holds base address of the SPI chosen
 * @Parameter[FlagName]		-Flag which needs to be checked
 * @Parameter[]
 * @return								-Flag SET or RESET
 * @special note:						-None
 */

uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t	*pSPIx, uint32_t FlagName){
	if(pSPIx->SR	&	FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/***********************************************************************************************
 * @Function:							-SPI_PeriClockControl
 * @Description:						-This function will enable/disable the peripheral clock for the Selected SPI
 * @Parameter[pSPIx]			-Holds base address of the SPI chosen
 * @Parameter[EnorDi]			-Holds the selection(Enable/Disable) for the SPI. //these are macros
 * @Parameter[]
 * @return								-None
 * @special note:						-None
 */

void SPI_PeriClockControl(SPI_Reg_Def_t  *pSPIx, uint8_t EnorDi){
	if(EnorDi==ENABLE){
				if(pSPIx==SPI1){
					SPI1_CLK_EN();
				}else if(pSPIx==SPI2){
					SPI2_CLK_EN();
				}else if(pSPIx==SPI3){
					SPI3_CLK_EN();
				}else if(pSPIx==SPI4){
					SPI4_CLK_EN();
				}else if(pSPIx==SPI5){
					SPI5_CLK_EN();
				}else if(pSPIx==SPI6){
					SPI6_CLK_EN();
				}
		}else
		{
			if(pSPIx==SPI1){
				}else if(pSPIx==SPI1){
					SPI1_CLK_DI();
				}else if(pSPIx==SPI2){
					SPI2_CLK_DI();
				}else if(pSPIx==SPI3){
					SPI3_CLK_DI();
				}else if(pSPIx==SPI4){
					SPI4_CLK_DI();
				}else if(pSPIx==SPI5){
					SPI5_CLK_DI();
				}else if(pSPIx==SPI6){
					SPI6_CLK_DI();
				}
		}
}


/***********************************************************************************************
 * @Function:							-SPI_Init
 * @Description:						-This function will initilize the SPI with the parameter defined in SPI Handle structure
 * @Parameter[pSPIHandle]-Holds settings
 * @return								-None
 * @special note:						-Needs to be called only after setting up proper handle structure
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//enable clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//Configure SPI_CR1 Register
	uint32_t tempreg=0;
	//1.Config Device mode
	tempreg	|=pSPIHandle->SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;

	//2.config the Bus Config
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD){
			//clear bi direction
		tempreg	&=	~(1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD){
			//Set Bi direction bit
		tempreg	|=(1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// clear Bi Direction
		tempreg	&=	~(1<<SPI_CR1_BIDIMODE);
		//set rx only bit
		tempreg	|=(1<<SPI_CR1_RXONLY);
	}
	pSPIHandle->pSPIx->CR1 =tempreg;
}


/***********************************************************************************************
 * @Function:							-SPI_DeInit
 * @Description:						-This function will reset the selected  SPI to default values
 * @Parameter[pSPIx]					-Holds base address of the SPI
 * @return								-None
 * @special note:						-None
 */

void SPI_DeInit(SPI_Reg_Def_t  	*pSPIx){
					if(pSPIx==SPI1){
						SPI1_REG_RESET();
					}else if(pSPIx==SPI2){
						SPI2_REG_RESET();
					}else if(pSPIx==SPI3){
						SPI3_REG_RESET();
					}else if(pSPIx==SPI4){
						SPI4RST_REG_RESET();
					}else if(pSPIx==SPI5){
						SPI5RST_REG_RESET();
					}else if(pSPIx==SPI6){
						SPI6RST_REG_RESET();
					}
}
/***********************************************************************************************
 * @Function:							-SPI_SendData
 * @Description:						-This function will send the data
 * @Parameter[pSPIx]					-Holds base address of the SPI
 * @Parameter[pTxBuffer]				-Holds base address of the data to be sent
 * @Parameter[Len]						-Holds size of the data to be sent
 * @return								-None
 * @special note:						-Blocking function
 */
void SPI_SendData(SPI_Reg_Def_t 	*pSPIx,	uint8_t *pTxBuffer,uint32_t	Len){
	while(Len>0){
		//1.Wait while TXE Flag is set i.e Transmit buffer empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)==FLAG_RESET);

		//2.check the data frame format 8bit or 16bit
		if((pSPIx->CR1	&	(1<<SPI_CR1_DFF))){
			//16 bit frame format
			//load data in DR register
			pSPIx->DR=	*((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8 bit Frame format
			pSPIx->DR=	*pTxBuffer;
			Len--;
			pTxBuffer++;
		}





	}
}

/***********************************************************************************************
 * @Function:							-SPI_SendDataIT
 * @Description:						-This function will send the data
 * @Parameter[pSPIHandle]		-Holds handle details
 * @Parameter[pTxBuffer]				-Holds base address of the data to be sent
 * @Parameter[Len]						-Holds size of the data to be sent
 * @return								-None
 * @special note:						-NON-Blocking function uses interrupt
 */
uint8_t SPI_SendDataIT(SPI_Handle_t	 *pSPIHandle,	uint8_t *pTxBuffer,uint32_t	Len){
	uint8_t state=pSPIHandle->TxState;
	if(state		!=	SPI_BUSY_IN_TX){
	//1. Save the Tx buffer address and Len in some global variable
		pSPIHandle->pTXBuffer	=pTxBuffer;
		pSPIHandle->TxLen	=Len;
		//2.Make SPI state to busy so that other function cannot overtake the SPI peripheral until the transmission is done
		pSPIHandle->TxState	=SPI_BUSY_IN_TX;
		//3.Enable the TXEIE bit so that interrupt is generated whenever TXE flag is set
		pSPIHandle->pSPIx->CR2	|=(1<<SPI_CR2_TXEIE);
		//4.Data transmission will be done in ISR function
	}
	return state;
}



/***********************************************************************************************
 * @Function:							-SPI_ReceiveData
 * @Description:						-This function will receive the data
 * @Parameter[pSPIx]					-Holds base address of the SPI
 * @Parameter[pRxBuffer]				-Holds base address of the received data
 * @Parameter[Len]						-Holds the size of received data
 * @return								-None
 * @special note:						-Blocking Call
 */
void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx,	uint8_t *pRxBuffer,uint32_t	Len){
	while(Len > 0)
			{
				//1. wait until RXNE is set
				while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

				//2. check the DFF bit in CR1
				if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
				{
					//16 bit DFF
					//1. load the data from DR to Rxbuffer address
					 *((uint16_t*)pRxBuffer) = pSPIx->DR ;
					Len--;
					Len--;
					(uint16_t*)pRxBuffer++;
				}else
				{
					//8 bit DFF
					*(pRxBuffer) = pSPIx->DR ;
					Len--;
					pRxBuffer++;
				}
			}

}

/***********************************************************************************************
 * @Function:							-SPI_ReceiveDataIT
 * @Description:						-This function will receive the data
 * @Parameter[pSPIHandle]		-Holds handle details
 * @Parameter[pRxBuffer]				-Holds base address of the received data
 * @Parameter[Len]						-Holds the size of received data
 * @return								-None
 * @special note:						-NON-Blocking Call uses Interrupt
 */
uint8_t  SPI_ReceiveDataIT(SPI_Handle_t	 *pSPIHandle,	uint8_t *pRxBuffer,uint32_t	Len){
	uint8_t state=pSPIHandle->RxState;
		if(state		!=	SPI_BUSY_IN_RX){
		//1. Save the Tx buffer address and Len in some global variable
			pSPIHandle->pRXBuffer	=pRxBuffer;
			pSPIHandle->RxLen	=Len;
			//2.Make SPI state to busy so that other function cannot overtake the SPI peripheral until the transmission is done
			pSPIHandle->RxState	=SPI_BUSY_IN_RX;
			//3.Enable the TXEIE bit so that interrupt is generated whenever TXE flag is set
			pSPIHandle->pSPIx->CR2	|=(1<<SPI_CR2_RXNEIE);
			//4.Data transmission will be done in ISR function
		}
		return state;
}

/***********************************************************************************************
 * @Function:							-SPI_IRQConfig
 * @Description:						-This function will configure the interrupt on the selected SPI(This Function will configure the NVIC registers)
 * @Parameter[IRQNumber]				-Holds the interrupt number(IRQ number)
 * @Parameter[EnorDi]					-Holds the selection(Enable/Disable) for the SPI. //these are macros
 * @return								-None
 * @special note:						-None
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
 * @Function:							-SPI_IRQPrority_Config
 * @Description:						-This function is used to set the priority of the interrupt
 *@Parameter[IRQNumber]					-Holds the interrupt number(IRQ number)
 * @Parameter[IRQPriority]				-Holds the interrupt priority for the selected SPI
 * @return								-None
 * @special note:						-
 */
void SPI_IRQPrority_Config(uint8_t IRQNumber, 	uint32_t IRQPriority){
	//Finding IPR register
		uint8_t iprx=IRQNumber/4;
		uint8_t iprx_section=IRQNumber%4;
		uint8_t shift_amount=	((8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED));
		*(NVIC_PR_BASE_ADD+(iprx))	|=	(IRQPriority<<shift_amount);
}

/***********************************************************************************************
 * @Function:							-SPI_IRQHandling
 * @Description:						-This function will be triggered when an interrupt happens on the selected pin
 * 										The purpose of this function is to reset the pr register(nvic side)
 * @Parameter[pHandle]					-Holds address of spi handle on which interrupt is enabled
 * @return								-None
 * @special note:						-Can only be used after setting interrupt on the pin using IRQConfig function
 */
void SPI_IRQHandling(SPI_Handle_t	*pHandle){
	uint8_t temp1,temp2;
	//Check what caused the interrupt occured
	 temp1=pHandle->pSPIx->SR	&	(1<<SPI_SR_TXE);
	 temp2=pHandle->pSPIx->CR2	&	(1<<SPI_CR2_TXEIE);
	 if(temp1	&&	temp2){
		 //handle TX
		 spi_txe_interrupt_handle(pHandle);
	 }

	 temp1=pHandle->pSPIx->SR	&	(1<<SPI_SR_RXNE);
	 temp2=pHandle->pSPIx->CR2	&	(1<<SPI_CR2_RXNEIE);
	 	 if(temp1	&&	temp2){
	 		 //handle RX
	 		 spi_rxne_interrupt_handle(pHandle);
	 	 }

	 	 //Checking if overrun is occured
	 	temp1=pHandle->pSPIx->SR	&	(1<<SPI_SR_OVR);
	 	temp2=pHandle->pSPIx->CR2	&	(1<<SPI_CR2_ERRIE);
	 		 if(temp1	&&	temp2){
	 			 //handle overrun error
	 			 spi_ovr_err_interrupt_handle(pHandle);
	 		 }
}


/***********************************************************************************************
 * @Function:							-SPI_PeripheralControl
 * @Description:						-This function will enable the spi peripheral(sets spe bit)
 *
 * @Parameter[pSPIx]					-base address of spi peripheral
 * @return								-None
 * @special note:						-need to be called at last after calling the init function
 */
void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}

}

/***********************************************************************************************
 * @Function:							-SSIConfig
 * @Description:						-This function will set the SSI bit to 1 so that NSS pin is internally connected to Vcc which prevents MODF error in software SSM
 *
 * @Parameter[pSPIx]					-base address of spi peripheral
 * @return								-None
 * @special note:						-need to be called at last after calling the init function
 */
void SSIConfig(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
		}else
		{
			pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
		}
}
/***********************************************************************************************
 * @Function:							-SSOEConfig
 * @Description:						-This function will set the SSOE it to 1 which will make NSS output enable. The NSS pin is now automatically managed by hardware
 * 													when SPE=1,NSS will be pulled to low
 * 													when SPE=0,NSS will be high
 *
 * @Parameter[pSPIx]					-base address of spi peripheral
 * @return								-None
 * @special note:						-need to be called at last after calling the init function
 */
void SSOEConfig(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
		}else
		{
			pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
		}
}

/*****************************SPI Interrupt handles*********************************/
static void  spi_txe_interrupt_handle(SPI_Handle_t	 *pSPIHandle){
	if((pSPIHandle->pSPIx->CR1		&		(1<<SPI_CR1_DFF))){
				//16 bit frame format
				//load data in DR register
		pSPIHandle->pSPIx->DR=	*((uint16_t*)pSPIHandle->pTXBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTXBuffer++;
	}else{
				//8 bit Frame format
		pSPIHandle->pSPIx->DR=	*pSPIHandle->pTXBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTXBuffer++;
	}
	if(!pSPIHandle->TxLen){
		//TxLen is zero, so close the Spi transmission and clear the txeie bit so that no more interrupt is triggered
		SPI_CloseTransmisson(pSPIHandle);
		// !!!USER HAVE TO CREATE THIS FUNCTION
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);  //Returns the status to the application.
	}
}


static void spi_rxne_interrupt_handle(SPI_Handle_t	 *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
		{
			//16 bit
			*((uint16_t*)pSPIHandle->pRXBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRXBuffer++;
			pSPIHandle->pRXBuffer++;

		}else
		{
			//8 bit
			*(pSPIHandle->pRXBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRXBuffer++;
		}

		if(! pSPIHandle->RxLen)
		{
			//reception is complete
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t	 *pSPIHandle){
	uint8_t temp;
		//1. clear the ovr flag
		if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
		{ //if spi peripheral is busy in transmission than this code will not be executed
			temp = pSPIHandle->pSPIx->DR;
			temp = pSPIHandle->pSPIx->SR;
		}
		(void)temp;
		//2. inform the application
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle){
		pSPIHandle->pSPIx->CR2	&=	~(1<<SPI_CR2_TXEIE);
		//reset the buffer and tx length
		pSPIHandle->pTXBuffer=NULL;
		pSPIHandle->TxLen=0;
		pSPIHandle->TxState=SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
		pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRXBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;
	}
void SPI_ClearOVRFlag(SPI_Reg_Def_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}



__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}
