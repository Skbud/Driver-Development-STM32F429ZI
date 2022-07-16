/*
 * stm32f429zi_spi_driver.h
 *
 *  Created on: 01-Jul-2022
 *      Author: pc
 */

#ifndef STM32F429ZI_SPI_DRIVER_H_
#define STM32F429ZI_SPI_DRIVER_H_

#include	"STM32F429ZI.h"
#include	"stm32f429zi_gpio_driver.h"
//CONFIG MACROS

/*****SPI Device MODE******/
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/******Bus Config*********/
#define SPI_BUS_CONFIG_FD				1		//Full Duplex
#define SPI_BUS_CONFIG_HD				2		//HALF DUPLEX
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3		//SIMPLEX RX ONLY


/******CLOCK SPEED*********/
#define SPI_SCLK_SPEED_DIV2				0		//SERIAL CLOCK DIVIDED BY 2
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/********DFF***************/
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1


/*********CPOL**************/
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW 					0

/*********CPHA*************/
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW 					0

/********SSM**************/
#define SPI_SSM_EN						1
#define	SPI_SSM_DI						0

/*******SPI Flags*********/
#define SPI_TXE_FLAG					(1<<SPI_SR_TXE)			//Transmit buffer empty
#define SPI_RXNE_FLAG				(1<<SPI_SR_RXNE)			//Receive buffer not empty
#define SPI_CHSIDE_FLAG				(1<<SPI_SR_CHSIDE)		//Channel side
#define SPI_UDR_FLAG					(1<<SPI_SR_UDR)			//Underrun flag
#define SPI_CRCERR_FLAG			(1<<SPI_SR_CRCERR)		//CRC error flag
#define SPI_MODF_FLAG				(1<<SPI_SR_MODF)		//Mode fault
#define SPI_OVR_FLAG					(1<<SPI_SR_OVR)			//Overrun flag
#define SPI_BSY_FLAG					(1<<SPI_SR_BSY)			//Busy flag
#define SPI_FRE_FLAG					(1<<SPI_SR_FRE)				//Frame format error

/*********SPI Interrupt application macros***/

/*
 * SPI Application State
 */
#define SPI_READY						0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

/*
 * SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4
//Configuration structure for SPI Peripheral
typedef struct
{
	uint8_t SPI_DeviceMode;					//Slave or Master
	uint8_t SPI_BusConfig;						//Half Duplex,Full Duplex or Simplex
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;									//Data frame format 8bit or 16bit
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;									//Slave select mode
}SPI_Config_t;


//Handle Structure for spi peripheral
typedef struct
{
	SPI_Reg_Def_t	 *pSPIx;		//Holds base address of spi peripheral
	SPI_Config_t 	SPIConfig;	//Holds user configurations
	uint8_t 		*pTXBuffer;		//Store the Tx Buffer address for interrupt based application
	uint8_t			*pRXBuffer;		//Store the Rx Buffer address for interrupt based application
	uint32_t 		TxLen;				//Store the Tx data Length   for interrupt based application
	uint32_t		RxLen;				//Store the Rx data Length   for interrupt based application
	uint8_t			TxState;			//Store the Tx State(Busy or not)  for interrupt based application
	uint8_t 		RxState;			//Store the Rx State  for interrupt based application

}SPI_Handle_t;


/****************************************************************************************************************************************************************
 * 																							SUPPORTED API																															*
 ****************************************************************************************************************************************************************/
void SPI_PeriClockControl(SPI_Reg_Def_t  *pSPIx, uint8_t EnorDi);	//Clock Enable/Disable

void SPI_Init(SPI_Handle_t *pSPIHandle);							//Initialization

void SPI_DeInit(SPI_Reg_Def_t  	*pSPIx);							//DeInitialization

void SPI_SendData(SPI_Reg_Def_t *pSPIx,	uint8_t *pTxBuffer,uint32_t	Len);

void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx,	uint8_t *pRxBuffer,uint32_t	Len);

uint8_t SPI_SendDataIT(SPI_Handle_t	 *pSPIHandle,	uint8_t *pTxBuffer,uint32_t	Len);  		//Interrupt based

uint8_t SPI_ReceiveDataIT(SPI_Handle_t	 *pSPIHandle,	uint8_t *pRxBuffer,uint32_t	Len);		//Interrupt based

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

void SPI_IRQPrority_Config(uint8_t IRQNumber, 	uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t	*pHandle);

uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t	*pSPIx, uint32_t FlagName);

void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi);

void SSIConfig(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi);
void SSOEConfig(SPI_Reg_Def_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_Reg_Def_t *pSPIxi);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv); //Application Call back

#endif /* STM32F429ZI_SPI_DRIVER_H_ */
