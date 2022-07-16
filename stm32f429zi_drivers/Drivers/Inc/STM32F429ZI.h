/*
 * STM32F429ZI.h
 *
 *  Created on: 18-Jun-2022
 *      Author: Sanjay
 */

#ifndef STM32F429ZI_H_
#define STM32F429ZI_H_

#include<stdint.h>
#include <stddef.h>

//Generic Macros
#define ENABLE 														1
#define DISABLE 														0
#define SET																ENABLE
#define RESET															DISABLE
#define GPIO_PIN_SET												ENABLE
#define GPIO_PIN_RESET											DISABLE
#define FLAG_SET														ENABLE
#define FLAG_RESET													DISABLE

#define GPIO_BASEADD_TO_CODE(x)						((x==GPIOA)?0:\
																					(x==GPIOB)?1:\
																					(x==GPIOC)?2:\
																					(x==GPIOD)?3:\
																					(x==GPIOE)?4:\
																					(x==GPIOF)?5:\
																					(x==GPIOG)?6:\
																					(x==GPIOH)?7:\
																					(x==GPIOI)?8:0)

//MCU SPECIFIC MACROS
#define NO_PR_BITS_IMPLEMENTED						4					//4 bits is STs microcontroller
#define IRQ_NO_EXTI0												6
#define IRQ_NO_EXTI1												7
#define IRQ_NO_EXTI2												8
#define IRQ_NO_EXTI3												9
#define IRQ_NO_EXTI4												10
#define IRQ_NO_EXTI9_5											23
#define IRQ_NO_EXTI10_15										40
#define IRQ_NO_SPI1													35
#define IRQ_NO_SPI2													36
#define IRQ_NO_SPI3													51
#define IRQ_NO_SPI4													84
#define IRQ_NO_SPI5													85
#define IRQ_NO_SPI6													86

/************************************************************************************************************************************************************************
*										SPI BIT DEFINATION MACROS																										*
*************************************************************************************************************************************************************************/
//SPI control register 1 (SPI_CR1) (not used in I2S mode)
#define SPI_CR1_CPHA								0
#define SPI_CR1_CPOL								1
#define SPI_CR1_MSTR								2
#define SPI_CR1_BR									3
#define SPI_CR1_SPE									6
#define SPI_CR1_LSBFIRST							7
#define SPI_CR1_SSI									8
#define SPI_CR1_SSM									9
#define SPI_CR1_RXONLY								10
#define SPI_CR1_DFF									11
#define SPI_CR1_CRCNEXT								12
#define SPI_CR1_CRCEN								13
#define SPI_CR1_BIDIOE								14
#define SPI_CR1_BIDIMODE							15

//SPI control register 2 (SPI_CR2)
#define SPI_CR2_RXDMAEN								0
#define SPI_CR2_TXDMAEN								1
#define SPI_CR2_SSOE								2
#define SPI_CR2_FRF									4
#define SPI_CR2_ERRIE								5
#define SPI_CR2_RXNEIE								6
#define SPI_CR2_TXEIE								7

//SPI status register (SPI_SR)
#define SPI_SR_RXNE									0
#define SPI_SR_TXE									1
#define SPI_SR_CHSIDE								2
#define SPI_SR_UDR									3
#define SPI_SR_CRCERR								4
#define SPI_SR_MODF									5
#define SPI_SR_OVR									6
#define SPI_SR_BSY									7
#define SPI_SR_FRE									8

//SPI_I2S configuration register (SPI_I2SCFGR)
#define SPI_I2SCFGR_CHLEN								0
#define SPI_I2SCFGR_DATLEN								1
#define SPI_I2SCFGR_CKPOL								3
#define SPI_I2SCFGR_I2SSTD								4
#define SPI_I2SCFGR_PCMSYNC								7
#define SPI_I2SCFGR_I2SCFG								8
#define SPI_I2SCFGR_I2SE								10
#define SPI_I2SCFGR_I2SMOD								11

//SPI_I2S prescaler register (SPI_I2SPR)
#define SPI_I2SPR_I2SDIV								0
#define SPI_I2SPR_ODD									8
#define SPI_I2SPR_MCKOE									9


/************************************************************************************************************************************************************************
*										I2C BIT DEFINATION MACROS																										*
*************************************************************************************************************************************************************************/
//I2C Control register 1 (I2C_CR1)
#define I2C_CR1_PE												0		//Peripheral enable
#define I2C_CR1_SMBUS										1		//SMBus mode
#define I2C_CR1_SMBTYPE									3		//SMBus type
#define I2C_CR1_ENARP										4		//ARP enable
#define I2C_CR1_ENPEC										5		//PEC enable
#define I2C_CR1_ENGC											6		//General call enable
#define I2C_CR1_NOSTRETCH								7		// Clock stretching disable (Slave mode)
#define I2C_CR1_START											8		// Start generation
#define I2C_CR1_STOP											9		//Stop generation
#define I2C_CR1_ACK											10	//Acknowledge enable
#define I2C_CR1_POS												11	//Acknowledge/PEC Position (for data reception)
#define I2C_CR1_PEC												12	// Packet error checking
#define I2C_CR1_ALERT											13	//SMBus alert
#define I2C_CR1_SWRST										15	//Software reset

//I2C Control register 2 (I2C_CR2)
#define I2C_CR2_FREQ											0		// Peripheral clock frequency
#define I2C_CR2_ITERREN										8		//Error interrupt enable
#define I2C_CR2_ITEVTEN										9		//Event interrupt enable
#define I2C_CR2_ITBUFEN										10	//Buffer interrupt enable
#define I2C_CR2_DMAEN										11	//DMA requests enable
#define I2C_CR2_LAST											12	//DMA last transfer

//I2C Own address register 1 (I2C_OAR1)
#define I2C_OAR1_ADD0										0		//Interface address
#define I2C_OAR1_ADD1										1		//Interface address bits 7:1 of address
#define I2C_OAR1_ADD2										8		//[9:8]: Interface address
#define I2C_OAR1_ADDMODE								15	//Addressing mode (slave mode)

//I2C Own address register 2 (I2C_OAR2)
#define I2C_OAR2_ENDUAL									0		//Dual addressing mode enable
#define I2C_OAR2_ADD2										7		//Interface address

//I2C Data register (I2C_DR)
#define I2C_DR_DR												0		//8-bit data register

//I2C Status register 1 (I2C_SR1)
#define I2C_SR1_SB												0		//Start bit (Master mode)
#define I2C_SR1_ADDR											1		//Address sent (master mode)/matched (slave mode)
#define I2C_SR1_BTF												2		//Byte transfer finished
#define I2C_SR1_ADD10										3		//10-bit header sent (Master mode)
#define I2C_SR1_STOPF											4		//Stop detection (slave mode)
#define I2C_SR1_RxNE											6		// Data register not empty (receivers)
#define I2C_SR1_TxE												7		//Data register empty (transmitters)
#define I2C_SR1_BERR											8		// Bus error
#define I2C_SR1_ARLO											9		//Arbitration lost (master mode)
#define I2C_SR1_AF												10	// Acknowledge failure
#define I2C_SR1_OVR											11	//Overrun/Underrun
#define I2C_SR1_PECERR										12	//PEC Error in reception
#define I2C_SR1_TIMEOUT									14	//Timeout or Tlow error
#define I2C_SR1_SMBALERT									15	//SMBus alert

//I2C Status register 2 (I2C_SR2)
#define I2C_SR2_MSL													0		//Master/slave
#define I2C_SR2_BUSY												1		//Bus busy
#define I2C_SR2_TRA													2		//Transmitter/receiver
#define I2C_SR2_GENCALL										4		//General call address (Slave mode)
#define I2C_SR2_SMBDEFAULT									5		//SMBus device default address (Slave mode)
#define I2C_SR2_SMBHOST										6		//SMBus host header (Slave mode)
#define I2C_SR2_DUALF											7		//Dual flag (Slave mode)
#define I2C_SR2_PEC													8		//Packet error checking register

//I2C Clock control register (I2C_CCR)
#define	I2C_CCR_CCR												0		//Clock control register in Fm/Sm mode (Master mode)
#define	I2C_CCR_DUTY											14	//Fm mode duty cycle
#define	I2C_CCR_FS													15	//I2C master mode selection

//I2C TRISE register (I2C_TRISE)
#define	I2C_TRISE_TRISE											0		//Maximum rise time in Fm/Sm mode (Master mode)

//I2C FLTR register (I2C_FLTR)
#define	I2C_FLTR_DNF												0		//Digital noise filter
#define	I2C_FLTR_ANOFF											0		//Analog noise filter OFF


/*********************************************************************************************************************************************************************************************************************
*																												MEMORY ADDRESSES																																											*
************************************************************************************************************************************************************************************************************************/
#define FLASH_BASEADD											0x08000000U
#define SRAM1_BASEADD										0x20000000U				//Primary RAM
#define SRAM2_BASEADD										0x2001C000U
#define SRAM3_BASEADD										0x20020000U
#define SYSTEM_MEMORY_BASEADD						0x1FFF0000U             //ROM
#define SRAM															SRAM1_BASEADD


/*********************************************************************************************************************************************************************************************************************
*																												 BUS	 ADDRESSES																																													*
************************************************************************************************************************************************************************************************************************/
#define PERIPH_BASEADD							  			0x40000000U
#define AHB1_BASEADD											0x40020000U
#define AHB2_BASEADD											0x50000000U
#define AHB3_BASEADD											0xA0000000U
#define APB1_BASEADD											0x40000000U
#define APB2_BASEADD											0x40010000U


/*********************************************************************************************************************************************************************************************************************
*																												Base Address of Peripherals connected to AHB1 BUS																															*
************************************************************************************************************************************************************************************************************************/
#define GPIOA_BASEADD											 ((AHB1_BASEADD)	+	(0x0000))
#define GPIOB_BASEADD											((AHB1_BASEADD)		+	(0x0400))
#define GPIOC_BASEADD											((AHB1_BASEADD)		+	(0x0800))
#define GPIOD_BASEADD											((AHB1_BASEADD)		+	(0x0C00))
#define GPIOE_BASEADD											((AHB1_BASEADD)		+	(0x1000))
#define GPIOF_BASEADD											((AHB1_BASEADD)		+	(0x1400))
#define GPIOG_BASEADD											((AHB1_BASEADD)		+	(0x1800))
#define GPIOH_BASEADD											((AHB1_BASEADD)		+	(0x1C00))
#define GPIOI_BASEADD											((AHB1_BASEADD)		+	(0x2000))
#define GPIOJ_BASEADD											((AHB1_BASEADD)		+	(0x2400))
#define GPIOK_BASEADD											((AHB1_BASEADD)		+	(0x2800))

#define CRC_BASEADD												((AHB1_BASEADD)		+	(0x3000))		/*The CRC (cyclic redundancy check) calculation unit is used to get a
																																					CRC code from a 32-bit data word and a fixed generator polynomial.*/
#define RCC_BASEADD												((AHB1_BASEADD)		+	(0x3800))		//Reset and clock control Register
#define FLASH_INTERFACE_REG_BASEADD				((AHB1_BASEADD)		+	(0x3C00))		/*The Flash access control register is used to enable/disable the acceleration features and
																																				control the Flash memory access time according to CPU frequency.*/
#define BKPSRAM_BASEADD									((AHB1_BASEADD)		+	(0x4000))		//Backup SRAM
#define DMA1_BASEADD											((AHB1_BASEADD)		+	(0x6000))
#define DMA2_BASEADD											((AHB1_BASEADD)		+	(0x6400))
#define ETHERNET_MAC_BASEADD							((AHB1_BASEADD)		+	(0x8000))
#define DMA2D_BASEADD										((AHB1_BASEADD)		+	(0xB000))		/*The Chrom-Art Accelerator™ (DMA2D) is a specialized DMA dedicated to
																																					image manipulation*/
#define USB_OTG_HS_BASEADD								((AHB1_BASEADD)		+	(0x20000))	//USB on-the-go high-speed (OTG_HS)

/*********************************************************************************************************************************************************************************************************************
*																												Base Address of Peripherals connected to AHB2 BUS																															*
************************************************************************************************************************************************************************************************************************/
#define	USB_OTG_FS_BASEADD								((B2_BASEADD)			+	(0x0000))		//USB on-the-go full-speed (OTG_FS)
#define	DCMI_BASEADD											((AHB2_BASEADD)		+	(0x50000))	//Digital camera interface (DCMI)
#define	CRYP_BASEADD											((AHB2_BASEADD)		+	(0x60000))	//The Cryptographic processor (CRYP) performs data encryption and decryption
#define	HASH_BASEADD										((AHB2_BASEADD)		+	(0x60400))	//The hash processor(HASH) is a fully compliant implementation of the secure hash algorithm
#define	RNG_BASEADD											((AHB2_BASEADD)		+	(0x60800))	//Random number generator (RNG)


/*********************************************************************************************************************************************************************************************************************
*																												Base Address of Peripherals connected to AHB3 BUS																															*
************************************************************************************************************************************************************************************************************************/
#define FMC_CONTROL_REG										(AHB3_BASEADD)								//Flexible memory controller (FMC) allows interfacing external memory



/*********************************************************************************************************************************************************************************************************************
*																												Base Address of Peripherals connected to APB1 BUS																															*
************************************************************************************************************************************************************************************************************************/
#define	TIM2_BASEADD											((APB1_BASEADD)	+	(0x0000))
#define	TIM3_BASEADD											((APB1_BASEADD)	+	(0x0400))
#define	TIM4_BASEADD											((APB1_BASEADD)	+	(0x0800))
#define	TIM5_BASEADD											((APB1_BASEADD)	+	(0x0C00))
#define	TIM6_BASEADD											((APB1_BASEADD)	+	(0x1000))
#define	TIM7_BASEADD											((APB1_BASEADD)	+	(0x1400))
#define	TIM12_BASEADD										((APB1_BASEADD)	+	(0x1800))
#define	TIM13_BASEADD										((APB1_BASEADD)	+	(0x1C00))
#define	TIM14_BASEADD										((APB1_BASEADD)	+	(0x2000))
#define	RTC_BKP_REG_BASEADD							((APB1_BASEADD)	+	(0x2800))		//RTC & backup registers
#define	WWDG_BASEADD										((APB1_BASEADD)	+	(0x2C00))		//The window Watch dog(WWDG) is used to detect the occurrence of a software fault
#define	IWDG_BASEADD										((APB1_BASEADD)	+	(0x3000))		//independent Watch Dog (IWDG)
#define	I2S2EXT_BASEADD										((APB1_BASEADD)	+	(0x3400))
#define	SPI2_I2S2_BASEADD									((APB1_BASEADD)	+	(0x3800))
#define	SPI3_I2S3_BASEADD									((APB1_BASEADD)	+	(0x3C00))
#define	I2S3EXT_BASEADD										((APB1_BASEADD)	+	(0x4000))
#define	USART2_BASEADD										((APB1_BASEADD)	+	(0x4400))
#define	USART3_BASEADD										((APB1_BASEADD)	+	(0x4800))
#define	UART4_BASEADD										((APB1_BASEADD)	+	(0x4C00))
#define	UART5_BASEADD										((APB1_BASEADD)	+	(0x5000))
#define	I2C1_BASEADD											((APB1_BASEADD)	+	(0x5400))
#define	I2C2_BASEADD											((APB1_BASEADD)	+	(0x5800))
#define	I2C3_BASEADD											((APB1_BASEADD)	+	(0x5C00))
#define	CAN1_BASEADD											((APB1_BASEADD)	+	(0x6400))
#define	CAN2_BASEADD											((APB1_BASEADD)	+	(0x6800))
#define	PWR_BASEADD											((APB1_BASEADD)	+	(0x7000))		//Power controller (PWR)
#define	DAC_BASEADD											((APB1_BASEADD)	+	(0x7400))
#define	UART7_BASEADD										((APB1_BASEADD)	+	(0x7800))
#define	UART8_BASEADD										((APB1_BASEADD)	+	(0x7C00))


/*********************************************************************************************************************************************************************************************************************
*																												Base Address of Peripherals connected to APB2 BUS																															*
************************************************************************************************************************************************************************************************************************/
#define	TIM1_BASEADD											((APB2_BASEADD)	+	(0x0000))
#define	TIM8_BASEADD											((APB2_BASEADD)	+	(0x0400))
#define	USART1_BASEADD										((APB2_BASEADD)	+	(0x1000))
#define	USART6_BASEADD										((APB2_BASEADD)	+	(0x1400))
#define	ADC1_ADC2_ADC3_BASEADD					((APB2_BASEADD)	+	(0x2000))
#define	SDIO_BASEADD											((APB2_BASEADD)	+	(0x2C00))		/*Secure digital input/output interface (SDIO)  provides an interface between the APB2
																																				peripheral bus and MultiMediaCards (MMCs)*/
#define	SPI1_BASEADD											((APB2_BASEADD)	+	(0x3000))
#define	SPI4_BASEADD											((APB2_BASEADD)	+	(0x3400))
#define	SYSCFG_BASEADD										((APB2_BASEADD)	+	(0x3800))		/*The system configuration controller(SYSCFG) is mainly used to remap the memory accessible in the
																																				code area, select the Ethernet PHY interface and manage the external interrupt line
																																				connection to the GPIOs.*/
#define	EXTI_BASEADD											((APB2_BASEADD)	+	(0x3C00))		//External interrupt/event controller (EXTI)
#define	TIM9_BASEADD											((APB2_BASEADD)	+	(0x4000))
#define	TIM10_BASEADD										((APB2_BASEADD)	+	(0x4400))
#define	TIM11_BASEADD										((APB2_BASEADD)	+	(0x4800))
#define	SPI5_BASEADD											((APB2_BASEADD)	+	(0x5000))
#define	SPI6_BASEADD											((APB2_BASEADD)	+	(0x5400))
#define	SAI1_BASEADD											((APB2_BASEADD)	+	(0x5800))		//Serial Audio Interface (SAI1)
#define	LCD_TFT_BASEADD									((APB2_BASEADD)	+	(0x6800))


/*********************************************************************************************************************************************************************************************************************
*																																	NVIC REGISTER BASE ADDRESSES																																	*
************************************************************************************************************************************************************************************************************************/
#define NVIC_ISER0														(volatile uint32_t*)(0xE000E100) 					//Interrupt Set-enable Registers
#define NVIC_ISER1														(volatile uint32_t*)(0xE000E104)
#define NVIC_ISER2														(volatile uint32_t*)(0xE000E108)
#define NVIC_ISER3														(volatile uint32_t*)(0xE000E10C)
#define NVIC_ISER4														(volatile uint32_t*)(0xE000E110)

#define	NVIC_ICER0														(volatile uint32_t*)(0XE000E180)					//Interrupt Clear-enable Registers
#define	NVIC_ICER1														(volatile uint32_t*)(0XE000E184)
#define	NVIC_ICER2														(volatile uint32_t*)(0XE000E188)
#define	NVIC_ICER3														(volatile uint32_t*)(0XE000E18C)
#define	NVIC_ICER4														(volatile uint32_t*)(0XE000E190)


#define	NVIC_PR_BASE_ADD										(volatile uint32_t*)0xE000E400					//Interrupt Priority Register


/*********************************************************************************************************************************************************************************************************************
*																																GPIO REGISTER DEFINATIONS																																				*
************************************************************************************************************************************************************************************************************************/
typedef struct
{
	volatile	uint32_t	MODER;																											//GPIO port mode register
	volatile	uint32_t	OTYPER;																											//GPIO port output type register
	volatile	uint32_t	OSPEEDR;																										//GPIO port output speed register
	volatile	uint32_t	PUPDR;																											//GPIO port pull-up/pull-down register
	volatile	uint32_t	IDR;																													//PIO port input data register
	volatile	uint32_t	ODR;																												//GPIO port output data register
	volatile	uint32_t	BSRR;																												//GPIO port bit set/reset register
	volatile	uint32_t	LCKR;																												/*GPIO port configuration lock register. This register is used to lock the configuration of the port bits when a
																																								correct write sequence is applied to bit 16 (LCKK). */
	volatile	uint32_t	AFR[2];																												//GPIO alternate function low register.  !NOTE:-   AFR[1]:AFLR   &    AFR[ 2]:AFHR
}GPIO_Reg_Def_t;


#define GPIOA																((GPIO_Reg_Def_t*)GPIOA_BASEADD)
#define GPIOB																((GPIO_Reg_Def_t*)GPIOB_BASEADD)
#define GPIOC																((GPIO_Reg_Def_t*)GPIOC_BASEADD)
#define GPIOD																((GPIO_Reg_Def_t*)GPIOD_BASEADD)
#define GPIOE																((GPIO_Reg_Def_t*)GPIOE_BASEADD)
#define GPIOF																((GPIO_Reg_Def_t*)GPIOF_BASEADD)
#define GPIOG																((GPIO_Reg_Def_t*)GPIOG_BASEADD)
#define GPIOH																((GPIO_Reg_Def_t*)GPIOH_BASEADD)
#define GPIOI																	((GPIO_Reg_Def_t*)GPIOI_BASEADD)
#define GPIOJ																	((GPIO_Reg_Def_t*)GPIOJ_BASEADD)
#define GPIOK																((GPIO_Reg_Def_t*)GPIOK_BASEADD)

/*********************************************************************************************************************************************************************************************************************
*																																RCC REGISTER DEFINATIONS																																				*
************************************************************************************************************************************************************************************************************************/
typedef struct
{
	volatile	uint32_t	CR;																											//clock control register
	volatile	uint32_t	PLLCFGR;																									//PLL configuration register
	volatile	uint32_t	CFGR;																										//clock configuration register
	volatile	uint32_t	CIR;																											//clock interrupt register
	volatile	uint32_t	AHB1RSTR;																								//AHB1 peripheral reset register
	volatile	uint32_t	AHB2RSTR;																								//AHB2 peripheral reset register
	volatile	uint32_t	AHB3RSTR;																								//AHB3 peripheral reset register
	volatile	uint32_t	Reserved0;
	volatile	uint32_t	APB1RSTR;																								//APB1 peripheral reset register
	volatile	uint32_t	APB2RSTR;																								//APB2 peripheral reset register
	volatile	uint32_t	Reserved1;
	volatile	uint32_t	Reserved2;
	volatile	uint32_t	AHB1ENR;																								//AHB1 peripheral clock register
	volatile	uint32_t	AHB2ENR;																								//AHB2 peripheral clock enable register
	volatile	uint32_t	AHB3ENR;																								//AHB3 peripheral clock enable register
	volatile	uint32_t	Reserved3;
	volatile	uint32_t	APB1ENR;																								//APB1 peripheral clock enable register
	volatile	uint32_t	APB2ENR;																								//APB2 peripheral clock enable register
	volatile	uint32_t	Reserved4;
	volatile	uint32_t	Reserved5;
	volatile	uint32_t	AHB1LPENR;																							//AHB1 peripheral clock enable in low power mode register
	volatile	uint32_t	AHB2LPENR;																							//AHB2 peripheral clock enable in low power mode register
	volatile	uint32_t	AHB3LPENR;																							//AHB3 peripheral clock enable in low power mode register
	volatile	uint32_t	Reserved6;
	volatile	uint32_t	APB1LPENR;																							// APB1 peripheral clock enable in low power mode register
	volatile	uint32_t	APB2LPENR;																							//APB2 peripheral clock enabled in low power mode register
	volatile	uint32_t	Reserved7;
	volatile	uint32_t	Reserved8;
	volatile	uint32_t	BDCR;																										// Backup domain control register
	volatile	uint32_t	CSR;																											//clock control & status register
	volatile	uint32_t	Reserved9;
	volatile	uint32_t	Reserved10;
	volatile	uint32_t	SSCGR;																										//spread spectrum clock generation register
	volatile	uint32_t	PLLI2SCFGR;																							//PLLI2S configuration register
	volatile	uint32_t	PLLSAICFGR;																							//PLL configuration register
	volatile	uint32_t	DCKCFGR;																								//Dedicated Clock Configuration Register
}RCC_Reg_Def_t;

#define RCC																((RCC_Reg_Def_t*)RCC_BASEADD	)



/*********************************************************************************************************************************************************************************************************************
*																																EXTI  REGISTER DEFINATIONS																																				*
************************************************************************************************************************************************************************************************************************/
typedef struct{
	volatile	uint32_t 	IMR;																											//Interrupt mask register
	volatile	uint32_t 	EMR;																										//Event mask register
	volatile	uint32_t 	RTSR;																										//Rising trigger selection register
	volatile	uint32_t 	FTSR;																										//Falling trigger selection register
	volatile	uint32_t 	SWIER;																										//Software interrupt event register
	volatile	uint32_t 	PR;																											//Pending register
}EXTI_Reg_Def_t;

#define EXTI																((EXTI_Reg_Def_t*)EXTI_BASEADD)


/*********************************************************************************************************************************************************************************************************************
*																																SYSCFG  REGISTER DEFINATIONS																																				*
************************************************************************************************************************************************************************************************************************/
typedef struct{
	volatile	uint32_t 	MEMRMP;																									//memory remap register
	volatile	uint32_t 	PMC;																											//peripheral mode configuration register
	volatile	uint32_t 	EXTICR[4];																									//external interrupt configuration register 1 to 4
	volatile	uint32_t 	CMPCR;																										//Compensation cell control register
}SYSCFG_Reg_Def_t;

#define SYSCFG															((SYSCFG_Reg_Def_t*)SYSCFG_BASEADD)

/*********************************************************************************************************************************************************************************************************************
*																					SPI REGISTER DEFINATIONS																																				*
************************************************************************************************************************************************************************************************************************/

typedef struct{
	volatile	uint32_t 	CR1;				//SPI control register 1
	volatile	uint32_t 	CR2;				//SPI control register 2
	volatile	uint32_t 	SR;					//SPI status register
	volatile	uint32_t 	DR;					//SPI data register
	volatile	uint32_t 	CRCPR;				//SPI CRC polynomial register
	volatile	uint32_t 	RXCRCR;				//SPI RX CRC register
	volatile	uint32_t 	TXCRCR;				//SPI TX CRC register
	volatile	uint32_t 	I2SCFGR;			//SPI_I2S configuration register
	volatile	uint32_t 	I2SPR;				//SPI_I2S prescaler register
}SPI_Reg_Def_t;

#define SPI1						((SPI_Reg_Def_t*)SPI1_BASEADD)
#define SPI2						((SPI_Reg_Def_t*)SPI2_I2S2_BASEADD)
#define SPI3						((SPI_Reg_Def_t*)SPI3_I2S3_BASEADD)
#define SPI4						((SPI_Reg_Def_t*)SPI4_BASEADD)
#define SPI5						((SPI_Reg_Def_t*)SPI5_BASEADD)
#define SPI6						((SPI_Reg_Def_t*)SPI6_BASEADD)



/*********************************************************************************************************************************************************************************************************************
*																					I2C REGISTER DEFINATIONS																																				*
************************************************************************************************************************************************************************************************************************/

typedef struct{
	volatile	uint32_t 	CR1;				//I2C Control register 1
	volatile	uint32_t 	CR2;				//I2C Control register 2
	volatile	uint32_t 	OAR1;			//I2C Own address register 1 (IT STORES ADDRESS WHEN WORKING AS SLAVE)
	volatile	uint32_t 	OAR2;			//I2C Own address register 2
	volatile	uint32_t 	DR;				//I2C Data register
	volatile	uint32_t 	SR1;				//I2C Status register 1
	volatile	uint32_t 	SR2;				//I2C Status register 2
	volatile	uint32_t 	CCR;				//I2C Clock control register
	volatile	uint32_t 	TRISE;			//I2C TRISE register(Maximum rise time in Fm/Sm mode (Master mode))
	volatile	uint32_t 	FLTR;			//FILTER
}I2C_Reg_Def_t;

#define	I2C1							((I2C_Reg_Def_t*)I2C1_BASEADD)
#define	I2C2							((I2C_Reg_Def_t*)I2C2_BASEADD)
#define	I2C3							((I2C_Reg_Def_t*)I2C3_BASEADD)


/* ************************************************ PERIPHERAL CLOCK ENABLE/DISABLE**********************************************/

/********************************GPIOS**************************************************/
#define GPIOA_CLK_EN()												(RCC->AHB1ENR	|=(1<<0))
#define GPIOA_CLK_DI()												(RCC->AHB1ENR	&=~(1<<0))
#define GPIOB_CLK_EN()												(RCC->AHB1ENR	|=(1<<1))
#define GPIOB_CLK_DI()												(RCC->AHB1ENR	&=~(1<<1))
#define GPIOC_CLK_EN()												(RCC->AHB1ENR	|=(1<<2))
#define GPIOC_CLK_DI()												(RCC->AHB1ENR	&=~(1<<2))
#define GPIOD_CLK_EN()												(RCC->AHB1ENR	|=(1<<3))
#define GPIOD_CLK_DI()												(RCC->AHB1ENR	&=~(1<<3))
#define GPIOE_CLK_EN()												(RCC->AHB1ENR	|=(1<<4))
#define GPIOE_CLK_DI()												(RCC->AHB1ENR	&=~(1<<4))
#define GPIOF_CLK_EN()												(RCC->AHB1ENR	|=(1<<5))
#define GPIOF_CLK_DI()													(RCC->AHB1ENR	&=~(1<<5))
#define GPIOG_CLK_EN()												(RCC->AHB1ENR	|=(1<<6))
#define GPIOG_CLK_DI()												(RCC->AHB1ENR	&=~(1<<6))
#define GPIOH_CLK_EN()												(RCC->AHB1ENR	|=(1<<7))
#define GPIOH_CLK_DI()												(RCC->AHB1ENR	&=~(1<<7))
#define GPIOI_CLK_EN()												(RCC->AHB1ENR	|=(1<<8))
#define GPIOI_CLK_DI()													(RCC->AHB1ENR	&=~(1<<8))


/*****************************OTHER AHB1 PERIPHERALS*****************************/
#define CRC_CLK_EN()													(RCC->AHB1ENR	|=(1<<12))									//CRC clock enable
#define CRC_CLK_DI()													(RCC->AHB1ENR	&=~(1<<12))
#define BKPSRAM_CLK_EN()											(RCC->AHB1ENR	|=(1<<18))									//Backup SRAM interface clock enable
#define BKPSRAM_CLK_DI()											(RCC->AHB1ENR	&=~(1<<18))
#define CCMDATARAM_CLK_EN()									(RCC->AHB1ENR	|=(1<<20))									//CCM data RAM clock enable
#define CCMDATARAM_CLK_DI()									(RCC->AHB1ENR	&=~(1<<20))
#define DMA1_CLK_EN()												(RCC->AHB1ENR	|=(1<<21))									//DMA1 clock enable
#define DMA1_CLK_DI()												(RCC->AHB1ENR	&=~(1<<21))
#define DMA2_CLK_EN()												(RCC->AHB1ENR	|=(1<<22))									// DMA2 clock enable
#define DMA2_CLK_DI()												(RCC->AHB1ENR	&=~(1<<22))
#define DMA2D_CLK_EN()											(RCC->AHB1ENR	|=(1<<23))									//DMA2D clock enable
#define DMA2D_CLK_DI()												(RCC->AHB1ENR	&=~(1<<23))
#define ETHMAC_CLK_EN()											(RCC->AHB1ENR	|=(1<<25))									//Ethernet MAC clock enable
#define ETHMAC_CLK_DI()											(RCC->AHB1ENR	&=~(1<<25))
#define ETHMACTX_CLK_EN()										(RCC->AHB1ENR	|=(1<<26))									//Ethernet Transmission clock enable
#define ETHMACTX_CLK_DI()										(RCC->AHB1ENR	&=~(1<<26))
#define ETHMACRX_CLK_EN()										(RCC->AHB1ENR	|=(1<<27))									//Ethernet Reception clock enable
#define ETHMACRX_CLK_DI()										(RCC->AHB1ENR	&=~(1<<27))
#define ETHMACPTP_CLK_EN()										(RCC->AHB1ENR	|=(1<<28))									//Ethernet PTP clock enable
#define ETHMACPTP_CLK_DI()										(RCC->AHB1ENR	&=~(1<<28))
#define OTGHS_CLK_EN()												(RCC->AHB1ENR	|=(1<<29))									//USB OTG HS clock enable
#define OTGHS_CLK_DI()												(RCC->AHB1ENR	&=~(1<<29))
#define OTGHSULPI_CLK_EN()										(RCC->AHB1ENR	|=(1<<30))									//USB OTG HSULPI clock enable
#define OTGHSULPI_CLK_DI()										(RCC->AHB1ENR	&=~(1<<30))

/*******************************AHB2 PERIPHERALS***********************************/
#define DCMI_CLK_EN()												(RCC->AHB2ENR	|=(1<<0))									//Camera interface enable
#define DCMI_CLK_DI()													(RCC->AHB2ENR	&=~(1<<0))
#define CRYP_CLK_EN()													(RCC->AHB2ENR	|=(1<<4))									//Cryptographic modules clock enable
#define CRYP_CLK_DI()													(RCC->AHB2ENR	&=~(1<<4))
#define HASH_CLK_EN()												(RCC->AHB2ENR	|=(1<<5))									//Hash modules clock enable
#define HASH_CLK_DI()													(RCC->AHB2ENR	&=~(1<<5))
#define RNG_CLK_EN()													(RCC->AHB2ENR	|=(1<<6))									//Random number generator clock enable
#define RNG_CLK_DI()													(RCC->AHB2ENR	&=~(1<<6))
#define OTGFS_CLK_EN()												(RCC->AHB2ENR	|=(1<<7))									//USB OTG FS clock enable
#define OTGFS_CLK_DI()												(RCC->AHB2ENR	&=~(1<<7))

/*****************************AHB3 PERIPHERALS*************************************/
#define FMC_CLK_EN()													(RCC->AHB3ENR	|=(1<<0))									//Flexible memory controller module clock enable
#define FMC_CLK_DI()													(RCC->AHB3ENR	&=~(1<<0))

/****************************APB1 PERIPHERALS**************************************/
#define TIM2_CLK_EN()													(RCC->APB1ENR		|=(1<<0))										//TIM2 clock enable
#define TIM2_CLK_DI()													(RCC->APB1ENR		&=~(1<<0))
#define TIM3_CLK_EN()													(RCC->APB1ENR		|=(1<<1))										//TIM3 clock enable
#define TIM3_CLK_DI()													(RCC->APB1ENR		&=~(1<<1))
#define TIM4_CLK_EN()													(RCC->APB1ENR		|=(1<<2))										//TIM4 clock enable
#define TIM4_CLK_DI()													(RCC->APB1ENR		&=~(1<<2))
#define TIM5_CLK_EN()													(RCC->APB1ENR		|=(1<<3))										//TIM5 clock enable
#define TIM5_CLK_DI()													(RCC->APB1ENR		&=~(1<<3))
#define TIM6_CLK_EN()													(RCC->APB1ENR		|=(1<<4))										// TIM6 clock enable
#define TIM6_CLK_DI()													(RCC->APB1ENR		&=~(1<<4))
#define TIM7_CLK_EN()													(RCC->APB1ENR		|=(1<<5))										//TIM7 clock enable
#define TIM7_CLK_DI()													(RCC->APB1ENR		&=~(1<<5))
#define TIM12_CLK_EN()												(RCC->APB1ENR		|=(1<<6))										//TIM12 clock enable
#define TIM12_CLK_DI()												(RCC->APB1ENR		&=~(1<<6))
#define TIM13_CLK_EN()												(RCC->APB1ENR		|=(1<<7))										//TIM13 clock enable
#define TIM13_CLK_DI()												(RCC->APB1ENR		&=~(1<<7))
#define TIM14_CLK_EN()												(RCC->APB1ENR		|=(1<<8))										//TIM14 clock enable
#define TIM14_CLK_DI()												(RCC->APB1ENR		&=~(1<<8))
#define WWDG_CLK_EN()												(RCC->APB1ENR		|=(1<<11))										//Window watch dog clock enable
#define WWDG_CLK_DI()												(RCC->APB1ENR		&=~(1<<11))
#define SPI2_CLK_EN()													(RCC->APB1ENR		|=(1<<14))										//SPI2 clock enable
#define SPI2_CLK_DI()													(RCC->APB1ENR		&=~(1<<14))
#define SPI3_CLK_EN()													(RCC->APB1ENR		|=(1<<15))										//SPI3 clock enable
#define SPI3_CLK_DI()													(RCC->APB1ENR		&=~(1<<15))
#define USART2_CLK_EN()												(RCC->APB1ENR		|=(1<<17))										//USART2 clock enable
#define USART2_CLK_DI()												(RCC->APB1ENR		&=~(1<<17))
#define USART3_CLK_EN()												(RCC->APB1ENR		|=(1<<18))										//USART3 clock enable
#define USART3_CLK_DI()												(RCC->APB1ENR		&=~(1<<18))
#define UART4_CLK_EN()												(RCC->APB1ENR		|=(1<<19))										//UART4 clock enable
#define UART4_CLK_DI()												(RCC->APB1ENR		&=~(1<<19))
#define UART5_CLK_EN()												(RCC->APB1ENR		|=(1<<20))										//UART5 clock enable
#define UART5_CLK_DI()												(RCC->APB1ENR		&=~(1<<20))
#define I2C1_CLK_EN()													(RCC->APB1ENR		|=(1<<21))										//I2C1 clock enable
#define I2C1_CLK_DI()													(RCC->APB1ENR		&=~(1<<21))
#define I2C2_CLK_EN()													(RCC->APB1ENR		|=(1<<22))										//I2C2 clock enable
#define I2C2_CLK_DI()													(RCC->APB1ENR		&=~(1<<22))
#define I2C3_CLK_EN()													(RCC->APB1ENR		|=(1<<23))										//I2C3 clock enable
#define I2C3_CLK_DI()													(RCC->APB1ENR		&=~(1<<23))
#define CAN1_CLK_EN()												(RCC->APB1ENR		|=(1<<25))										//CAN1 clock enable
#define CAN1_CLK_DI()													(RCC->APB1ENR		&=~(1<<25))
#define CAN2_CLK_EN()												(RCC->APB1ENR		|=(1<<26))										//CAN2 clock enable
#define CAN2_CLK_DI()													(RCC->APB1ENR		&=~(1<<26))
#define PWR_CLK_EN()													(RCC->APB1ENR		|=(1<<28))										//Power interface clock enable
#define PWR_CLK_DI()													(RCC->APB1ENR		&=~(1<<28))
#define DAC_CLK_EN()													(RCC->APB1ENR		|=(1<<29))										//DAC interface clock enable
#define DAC_CLK_DI()													(RCC->APB1ENR		&=~(1<<29))
#define UART7_CLK_EN()												(RCC->APB1ENR		|=(1<<30))										//UART7 clock enable
#define UART7_CLK_DI()												(RCC->APB1ENR		&=~(1<<30))
#define UART8_CLK_EN()												(RCC->APB1ENR		|=(1<<31))										//UART8 clock enable
#define UART8_CLK_DI()												(RCC->APB1ENR		&=~(1<<31))

/***************************APB2 PERIPHERALS**************************************/
#define TIM1_CLK_EN()													(RCC->APB2ENR		|=(1<<0))										//TIM1 clock enable
#define TIM1_CLK_DI()													(RCC->APB2ENR		&=~(1<<0))
#define TIM8_CLK_EN()													(RCC->APB2ENR		|=(1<<1))										//TIM8 clock enable
#define TIM8_CLK_DI()													(RCC->APB2ENR		&=~(1<<1))
#define USART1_CLK_EN()												(RCC->APB2ENR		|=(1<<4))										//USART1 clock enable
#define USART1_CLK_DI()												(RCC->APB2ENR		&=~(1<<4))
#define USART6_CLK_EN()												(RCC->APB2ENR		|=(1<<5))										//USART6 clock enable
#define USART6_CLK_DI()												(RCC->APB2ENR		&=~(1<<5))
#define ADC1_CLK_EN()												(RCC->APB2ENR		|=(1<<8))										//ADC1 clock enable
#define ADC1_CLK_DI()													(RCC->APB2ENR		&=~(1<<8))
#define ADC2_CLK_EN()												(RCC->APB2ENR		|=(1<<9))										//ADC2 clock enable
#define ADC2_CLK_DI()													(RCC->APB2ENR		&=~(1<<9))
#define ADC3_CLK_EN()												(RCC->APB2ENR		|=(1<<10))										//ADC3 clock enable
#define ADC3_CLK_DI()													(RCC->APB2ENR		&=~(1<<10))
#define SDIO_CLK_EN()													(RCC->APB2ENR		|=(1<<11))										//SDIO clock enable
#define SDIO_CLK_DI()													(RCC->APB2ENR		&=~(1<<11))
#define SPI1_CLK_EN()													(RCC->APB2ENR		|=(1<<12))										//SPI1 clock enable
#define SPI1_CLK_DI()													(RCC->APB2ENR		&=~(1<<12))
#define SPI4_CLK_EN()													(RCC->APB2ENR		|=(1<<13))										//SPI4 clock enable
#define SPI4_CLK_DI()													(RCC->APB2ENR		&=~(1<<13))
#define SYSCFG_CLK_EN()												(RCC->APB2ENR		|=(1<<14))										//System configuration controller clock enable
#define SYSCFG_CLK_DI()												(RCC->APB2ENR		&=~(1<<14))
#define TIM9_CLK_EN()													(RCC->APB2ENR		|=(1<<16))										//TIM9 clock enable
#define TIM9_CLK_DI()													(RCC->APB2ENR		&=~(1<<16))
#define TIM10_CLK_EN()												(RCC->APB2ENR		|=(1<<17))										//TIM10 clock enable
#define TIM10_CLK_DI()												(RCC->APB2ENR		&=~(1<<17))
#define TIM11_CLK_EN()												(RCC->APB2ENR		|=(1<<18))										//TIM11 clock enable
#define TIM11_CLK_DI()												(RCC->APB2ENR		&=~(1<<18))
#define SPI5_CLK_EN()													(RCC->APB2ENR		|=(1<<20))										//SPI5 clock enable
#define SPI5_CLK_DI()													(RCC->APB2ENR		&=~(1<<20))
#define SPI6_CLK_EN()													(RCC->APB2ENR		|=(1<<21))										//SPI6 clock enable
#define SPI6_CLK_DI()													(RCC->APB2ENR		&=~(1<<21))
#define SAI1_CLK_EN()													(RCC->APB2ENR		|=(1<<22))										//Serial Audio Interface (SAI1) clock enable
#define SAI1_CLK_DI()													(RCC->APB2ENR		&=~(1<<22))
#define LTDC_CLK_EN()													(RCC->APB2ENR		|=(1<<26))										//LTDC(Display Controller) clock enable
#define LTDC_CLK_DI()													(RCC->APB2ENR		&=~(1<<26))

/*************************************************************************************************************************
 * 																PERIPHERAL RESET																						*
 * ***********************************************************************************************************************/


/************************************RESET MACROS FOR AHB1 PEERIPHERALS**************************************/

#define GPIOA_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<0));		 (RCC->AHB1RSTR		&=	~(1<<0));	}while(0)
#define GPIOB_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<1));		 (RCC->AHB1RSTR		&=	~(1<<1));	}while(0)
#define GPIOC_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<2));		 (RCC->AHB1RSTR		&=	~(1<<2));	}while(0)
#define GPIOD_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<3));		 (RCC->AHB1RSTR		&=	~(1<<3));	}while(0)
#define GPIOE_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<4));		 (RCC->AHB1RSTR		&=	~(1<<4));	}while(0)
#define GPIOF_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<5));		 (RCC->AHB1RSTR		&=	~(1<<5));	}while(0)
#define GPIOG_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<6));		 (RCC->AHB1RSTR		&=	~(1<<6));	}while(0)
#define GPIOH_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<7));		 (RCC->AHB1RSTR		&=	~(1<<7));	}while(0)
#define GPIOI_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<8));		 (RCC->AHB1RSTR		&=	~(1<<8));	}while(0)

#define CRC_REG_RESET()												do{ (RCC->AHB1RSTR		|=(1<<12));		 (RCC->AHB1RSTR		&=	~(1<<12));	}while(0)
#define DMA1_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<21));		 (RCC->AHB1RSTR		&=	~(1<<21));	}while(0)
#define DMA2_REG_RESET()											do{ (RCC->AHB1RSTR		|=(1<<22));		 (RCC->AHB1RSTR		&=	~(1<<22));	}while(0)
#define ETHMAC_REG_RESET()										do{ (RCC->AHB1RSTR		|=(1<<25));		 (RCC->AHB1RSTR		&=	~(1<<29));	}while(0)
#define OTGHS_REG_RESET()										do{ (RCC->AHB1RSTR		|=(1<<29));		 (RCC->AHB1RSTR		&=	~(1<<29));	}while(0)

/************************************RESET MACROS FOR AHB2 PEERIPHERALS**************************************/
#define DCMI_REG_RESET()											do{ (RCC->AHB2RSTR		|=(1<<0));		 (RCC->AHB2RSTR		&=	~(1<<0));	}while(0)
#define CRYP_REG_RESET()											do{ (RCC->AHB2RSTR		|=(1<<4));		 (RCC->AHB2RSTR		&=	~(1<<4));	}while(0)
#define HASH_REG_RESET()											do{ (RCC->AHB2RSTR		|=(1<<5));		 (RCC->AHB2RSTR		&=	~(1<<5));	}while(0)
#define RNG_REG_RESET()											do{ (RCC->AHB2RSTR		|=(1<<6));		 (RCC->AHB2RSTR		&=	~(1<<6));	}while(0)
#define OTGFS_REG_RESET()											do{ (RCC->AHB2RSTR		|=(1<<7));		 (RCC->AHB2RSTR		&=	~(1<<7));	}while(0)

/************************************RESET MACROS FOR AHB3 PEERIPHERALS**************************************/
#define FSMC_REG_RESET()											do{ (RCC->AHB3RSTR		|=(1<<0));		 (RCC->AHB3RSTR		&=	~(1<<0));	}while(0)

/************************************RESET MACROS FOR APB1 PEERIPHERALS**************************************/
#define TIM2_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<0));		 (RCC->APB1RSTR		&=	~(1<<0));	}while(0)
#define TIM3_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<1));		 (RCC->APB1RSTR		&=	~(1<<1));	}while(0)
#define TIM4_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<2));		 (RCC->APB1RSTR		&=	~(1<<2));	}while(0)
#define TIM5_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<3));		 (RCC->APB1RSTR		&=	~(1<<3));	}while(0)
#define TIM6_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<4));		 (RCC->APB1RSTR		&=	~(1<<4));	}while(0)
#define TIM7_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<5));		 (RCC->APB1RSTR		&=	~(1<<5));	}while(0)
#define TIM12_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<6));		 (RCC->APB1RSTR		&=	~(1<<6));	}while(0)
#define TIM13_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<7));		 (RCC->APB1RSTR		&=	~(1<<7));	}while(0)
#define TIM14_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<8));		 (RCC->APB1RSTR		&=	~(1<<8));	}while(0)
#define WWDG_REG_RESET()										do{ (RCC->APB1RSTR		|=(1<<11));		 (RCC->APB1RSTR		&=	~(1<<11));	}while(0)
#define SPI2_REG_RESET()												do{ (RCC->APB1RSTR		|=(1<<14));		 (RCC->APB1RSTR		&=	~(1<<14));	}while(0)
#define SPI3_REG_RESET()												do{ (RCC->APB1RSTR		|=(1<<15));		 (RCC->APB1RSTR		&=	~(1<<15));	}while(0)
#define UART2_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<17));		 (RCC->APB1RSTR		&=	~(1<<17));	}while(0)
#define UART3_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<18));		 (RCC->APB1RSTR		&=	~(1<<18));	}while(0)
#define UART4_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<19));		 (RCC->APB1RSTR		&=	~(1<<19));	}while(0)
#define UART5_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<20));		 (RCC->APB1RSTR		&=	~(1<<20));	}while(0)
#define I2C1_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<21));		 (RCC->APB1RSTR		&=	~(1<<21));	}while(0)
#define I2C2_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<22));		 (RCC->APB1RSTR		&=	~(1<<22));	}while(0)
#define I2C3_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<23));		 (RCC->APB1RSTR		&=	~(1<<23));	}while(0)
#define CAN1_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<25));		 (RCC->APB1RSTR		&=	~(1<<25));	}while(0)
#define CAN2_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<26));		 (RCC->APB1RSTR		&=	~(1<<26));	}while(0)
#define PWR_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<28));		 (RCC->APB1RSTR		&=	~(1<<28));	}while(0)
#define	DAC_REG_RESET()											do{ (RCC->APB1RSTR		|=(1<<29));		 (RCC->APB1RSTR		&=	~(1<<29));	}while(0)

/************************************RESET MACROS FOR APB2 PEERIPHERALS**************************************/
#define	TIM1_REG_RESET()											do{ (RCC->APB2RSTR		|=(1<<0));		 (RCC->APB2RSTR		&=	~(1<<0));	}while(0)
#define	TIM8_REG_RESET()											do{ (RCC->APB2RSTR		|=(1<<1));		 (RCC->APB2RSTR		&=	~(1<<1));	}while(0)
#define	USART1_REG_RESET()										do{ (RCC->APB2RSTR		|=(1<<4));		 (RCC->APB2RSTR		&=	~(1<<4));	}while(0)
#define	USART6_REG_RESET()										do{ (RCC->APB2RSTR		|=(1<<5));		 (RCC->APB2RSTR		&=	~(1<<5));	}while(0)
#define	ADC_REG_RESET()											do{ (RCC->APB2RSTR		|=(1<<8));		 (RCC->APB2RSTR		&=	~(1<<8));	}while(0)
#define	SDIO_REG_RESET()											do{ (RCC->APB2RSTR		|=(1<<11));		 (RCC->APB2RSTR		&=	~(1<<11));	}while(0)
#define	SPI1_REG_RESET()											do{ (RCC->APB2RSTR		|=(1<<12));		 (RCC->APB2RSTR		&=	~(1<<12));	}while(0)
#define	SPI4RST_REG_RESET()										do{ (RCC->APB2RSTR		|=(1<<13));		 (RCC->APB2RSTR		&=	~(1<<13));	}while(0)
#define	SYSCFG_REG_RESET()										do{ (RCC->APB2RSTR		|=(1<<14));		 (RCC->APB2RSTR		&=	~(1<<14));	}while(0)
#define	TIM9_REG_RESET()											do{ (RCC->APB2RSTR		|=(1<<16));		 (RCC->APB2RSTR		&=	~(1<<16));	}while(0)
#define	TIM10_REG_RESET()										do{ (RCC->APB2RSTR		|=(1<<17));		 (RCC->APB2RSTR		&=	~(1<<17));	}while(0)
#define	TIM11_REG_RESET()										do{ (RCC->APB2RSTR		|=(1<<18));		 (RCC->APB2RSTR		&=	~(1<<18));	}while(0)
#define	SPI5RST_REG_RESET()										do{ (RCC->APB2RSTR		|=(1<<20));		 (RCC->APB2RSTR		&=	~(1<<20));	}while(0)
#define	SPI6RST_REG_RESET()										do{ (RCC->APB2RSTR		|=(1<<21));		 (RCC->APB2RSTR		&=	~(1<<21));	}while(0)
#define	SAI1RST_REG_RESET()									do{ (RCC->APB2RSTR		|=(1<<22));		 (RCC->APB2RSTR		&=	~(1<<22));	}while(0)
#define	LTDCRST_REG_RESET()									do{ (RCC->APB2RSTR		|=(1<<26));		 (RCC->APB2RSTR		&=	~(1<<26));	}while(0)









#endif /* STM32F429ZI_H_ */


