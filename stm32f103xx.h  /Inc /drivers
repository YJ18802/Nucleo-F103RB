/*
      This mcu specific header file where memory addresses of peripherals are defined.
      Struct for gpio peripheral, RCC, and some peripheral clock macros and some generic macros.
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include<stdint.h>
#define __vo volatile

//Memory Address

#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U		 //This is SRAM base address we have added 1 to specify sram as there can be multiple secondary sram
#define ROM_BASEADDR						  0x1FFFF000U      //This is system memory (datasheet)
#define SRAM								      SRAM1_BASEADDR   //If there are multiple sram so to highlight the main sram we have made this macro

//Peripheral Base Address

#define PERIPH_BASEADDR					0x4000 0000U     //Peripheral memory address
#define APB1_BASEADDR						PERIPH_BASEADDR  //The API BUS starts from this memory address for peripheral which requires low speed
#define APB2_BASEADDR						0x40010000U  	 //Peripheral 2 address
#define AHB_BASEADDR            0x40020000U      //High speed peripheral bus base address

//AHB Peripheral Base Address

#define RCC_PERIPH_BASEADDR			(AHB_BASEADDR + 0x1000)

//APB1 Peripheral Base Address

#define TIMER2_BASEADDR        	APB1_BASEADDR
#define TIMER3_BASEADDR					(APB1_BASEADDR + 0x0400)
#define TIMER4_BASEADDR					(APB1_BASEADDR + 0x0800)
#define SPI2_BASEADDR						(APB1_BASEADDR + 0x3800)
#define USART2_BASEADDR					(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1_BASEADDR + 0x4800)
#define I2C1_BASEADDR           (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR           (APB1_BASEADDR + 0x5800)

//APB2 Peripheral Base Address

#define AFIO_BASEADDR                       APB2_BASEADDR
#define EXTI_BASEADDR					              (APB2_BASEADDR + 0x0400)
#define GPIOA_BASEADDR                      (APB2_BASEADDR + 0x0800)
#define GPIOB_BASEADDR                      (APB2_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR                      (APB2_BASEADDR + 0x1000)
#define GPIOD_BASEADDR                      (APB2_BASEADDR + 0x1400)
#define GPIOE_BASEADDR                      (APB2_BASEADDR + 0x1800)
#define ADC1_BASEADDR                       (APB2_BASEADDR + 0x2400)
#define ADC2_BASEADDR                       (APB2_BASEADDR + 0x2800)
#define TIMER1_BASEADDR                     (APB2_BASEADDR + 0x2C00)
#define SPI1_BASEADDR                       (APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR                     (APB2_BASEADDR + 0x3800)

/*
 * Peripheral structure containing all the registers for peripheral configuration
 */

//GPIO registers struct
typedef struct {
	__vo uint32_t GPIO_CRL; //Port configuration register low (GPIOx_CRL) Address Offset : 0x00
	__vo uint32_t GPIO_CRH; //Port configuration register high (GPIOx_CRH) (x=A..G) Address offset: 0x04
	__vo uint32_t GPIO_IDR; //Port input data register (GPIOx_IDR) (x=A..G) Address offset: 0x08h
	__vo uint32_t GPIO_ODR;	//Port output data register (GPIOx_ODR) (x=A..G) Address offset: 0x0C
	__vo uint32_t GPIO_BSRR; //Port bit set/reset register (GPIOx_BSRR) (x=A..G) Address offset: 0x10
	__vo uint32_t GPIO_BRR; //Port bit reset register (GPIOx_BRR) (x=A..G) Address offset: 0x14
	__vo uint32_t GPIO_LCKR; //Port configuration lock register (GPIOx_LCKR) (x=A..G) Address Offset : 0x18


} GPIO_RegDef_t;

/*
 * AFIO Reg Def
 */
typedef struct {

	__vo uint32_t	AFIO_EVCR;
	__vo uint32_t	AFIO_MAPR;
	__vo uint32_t	AFIO_EXTICR1;
	__vo uint32_t	AFIO_EXTICR2;
	__vo uint32_t	AFIO_EXTICR3;
	__vo uint32_t	AFIO_EXTICR4;
	__vo uint32_t	AFIO_MAPR2;


} AFIO_Reg_Def_t;

//RCC registers struct

typedef struct {
	__vo uint32_t RCC_CR; //Clock control register (RCC_CR) Address offset: 0x00
	__vo uint32_t RCC_CFGR; //Clock configuration register (RCC_CFGR) Address offset: 0x04
	__vo uint32_t RCC_CIR; //Clock interrupt register (RCC_CIR) Address offset: 0x08
	__vo uint32_t RCC_APB2RSTR; //APB2 peripheral reset register (RCC_APB2RSTR) Address offset: 0x0C
	__vo uint32_t RCC_APB1RSTR; //APB1 peripheral reset register (RCC_APB1RSTR)Address offset: 0x10
	__vo uint32_t RCC_AHBENR; //AHB peripheral clock enable register (RCC_AHBENR) Address offset: 0x14
	__vo uint32_t RCC_APB2ENR; //APB2 peripheral clock enable register (RCC_APB2ENR) Address: 0x18
	__vo uint32_t RCC_APB1ENR; //APB1 peripheral clock enable register (RCC_APB1ENR) Address: 0x1C
	__vo uint32_t RCC_BDCR; //Backup domain control register (RCC_BDCR) Address offset: 0x20
	__vo uint32_t RCC_CSR;     //Control/status register (RCC_CSR) Address: 0x24

} RCC_RegDef_t;



/*
 * Peripheral GPIO Base address definition. Here we will be creating macros for all the gpio and initalizing all the gpio with the struct GPIO_RegDef using pointer
 * GPIO_RegDef_t *Pgpiox = (GPIO_RegDef_t *)Gpiox_BaseAddress = GPIOX , hence we can then use this to access different registers in gpio.
 */

#define GPIOA           ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB           ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC           ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD           ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE           ((GPIO_RegDef_t *)GPIOE_BASEADDR)

/*
 * AFIO
 */
#define AFIO 			((AFIO_Reg_Def_t *)AFIO_BASEADDR) ;

/*
 * RCC Register typecast
 */
#define RCC             ((RCC_RegDef_t *)RCC_PERIPH_BASEADDR)

/*
 * Clock enable Macros for gpio
 */

#define GPIOA_PCLK_EN()              (RCC->RCC_APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()              (RCC->RCC_APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()              (RCC->RCC_APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()              (RCC->RCC_APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()              (RCC->RCC_APB2ENR |= (1 << 6))

/*
 * Clock enable for AFIO
 */

#define AFIO_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 0))

/*
 * Clock enable macros for I2C
 */
#define I2C1_PCLK_EN()               (RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()               (RCC->RCC_APB1ENR |= (1 << 22))

/*
 * Clock enable macros for SPI
 */
#define SPI1_PCLK_EN()               (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()               (RCC->RCC_APB1ENR |= (1 << 14))

/*
 * Clock enable macros for USART
 */
#define USART1_PCLK_EN()              (RCC->RCC_APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()              (RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()              (RCC->RCC_APB1ENR |= (1 << 18))

/*
 * Clock disable Macros for gpio
 */

#define GPIOA_PCLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 6))

/*
 * Clock disable macros for I2C
 */
#define I2C1_PCLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 22))

/*
 * Clock disable macros for SPI
 */
#define SPI1_PCLK_DI()               (RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 14))

/*
 * Clock disable macros for USART
 */
#define USART1_PCLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()              (RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()              (RCC->RCC_APB1ENR &= ~(1 << 18))

/*
 * ResetGPIO Registers
 */

#define GPIOA_Reg_Reset()			 do{ (RCC->RCC_APB2RSTR |=(1<<2)) ; (RCC->RCC_APB2RSTR &= ~(1<<2));} while(0)
#define GPIOB_Reg_Reset()			 do{ (RCC->RCC_APB2RSTR |=(1<<3)) ; (RCC->RCC_APB2RSTR &= ~(1<<3));} while(0)
#define GPIOC_Reg_Reset()			 do{ (RCC->RCC_APB2RSTR |=(1<<4)) ; (RCC->RCC_APB2RSTR &= ~(1<<4));} while(0)
#define GPIOD_Reg_Reset()			 do{ (RCC->RCC_APB2RSTR |=(1<<5)) ; (RCC->RCC_APB2RSTR &= ~(1<<5));} while(0)
#define GPIOE_Reg_Reset()			 do{ (RCC->RCC_APB2RSTR |=(1<<6)) ; (RCC->RCC_APB2RSTR &= ~(1<<6));} while(0)

//Generic Macros

#define ENABLE                         1
#define DISABLE                        0
#define SET							   ENABLE
#define RESET                          DISABLE
#define GPIO_PIN_SET				   SET
#define GPIO_PIN_RESET                 RESET





# include "stm32f103xx_gpio_driver.h"

#endif /* INC_STM32F103XX_H_ */
