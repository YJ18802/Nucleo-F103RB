/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Oct 25, 2025
 *      Author: Dell
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"


/*
 * Pin Config for gpio pin
 */

typedef struct
	{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinPuPdControl;



	}GPIO_PinConfig_t;

typedef struct
	{
		uint8_t AFIO_PinAltFunModeControl;
		uint8_t AFIO_PinAltFunModePort;
		uint8_t AFIO_PinAltFunModePinNumber;
	}AFIO_PinConfig_t;


/*
 * Handle structure for gpio pin
 */
typedef struct
	{
		GPIO_RegDef_t *pGPIOx;
		GPIO_PinConfig_t GPIO_PinConfig;

	} GPIO_Handle_t;

	typedef struct
		{
			AFIO_Reg_Def_t *pAFIO;
			AFIO_PinConfig_t AFIO_PinConfig;

		} AFIO_Handle_t;

/*
 * GPIO Pin Numbers
 */

#define GPIO_PIN_NO_0 				 0
#define GPIO_PIN_NO_1				 1
#define GPIO_PIN_NO_2				 2
#define GPIO_PIN_NO_3				 3
#define GPIO_PIN_NO_4				 4
#define GPIO_PIN_NO_5				 5
#define GPIO_PIN_NO_6				 6
#define GPIO_PIN_NO_7				 7
#define GPIO_PIN_NO_8				 8
#define GPIO_PIN_NO_9				 9
#define GPIO_PIN_NO_10				 10
#define GPIO_PIN_NO_11				 11
#define GPIO_PIN_NO_12				 12
#define GPIO_PIN_NO_13				 13
#define GPIO_PIN_NO_14				 14
#define GPIO_PIN_NO_15				 15

/*
 * GPIO Port
*/

#define PORTA                 0
#define PORTB				  1
#define PORTC			      2
#define PORTD			      3
#define PORTE                 4



/*
 * GPIO Possible modes
 */
//Input Mode
#define GPIO_MODE_IN_Analog   					0b0000
#define GPIO_MODE_IN_Float 						0b0100
#define GPIO_MODE_IN_PullUpDwn 					0b1000

//Output Mode
#define GPIO_MODE_OUT_PushPull					0b0010     //Output speed is 2 Mhz
#define GPIO_MODE_OUT_OpenDrain					0b0110
//Alternate Function Mode
#define GPIO_MODE_ALTFUN_PushPull				9
#define GPIO_MODE_ALTFUN_OpenDrain				13

// For Pull Up Pull Down we need to configure Px_ODR register also
#define GPIO_PinPullUp_ODR						1
#define GPIO_PinPullDown_ODR					0



/**************************************************************************************************************************

                           API for users to work with GPIO

 ***************************************************************************************************************************/

/*
 * Gpio clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);                              //Enable the GPIO clk

/*
 * Gpio initializtion
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);                                                    //Initalize the gpio
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);                                                      //DeInitialize the GPIO
/*
 * AFIO initialization
 */

void AFIO_Init(AFIO_Handle_t *pAFIOHandle);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);	                    //Read from a input pin of a gpio port
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);	                                       //Read from a the whole port of the gpio
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);            //Write to output pin of the gpio
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);                            //Write to output port of the gpio
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);                                //Toggle output pin

/*
 * Interrupt Configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi );                         //IRQ Config , configuring priority,type of interrupt
void GPIO_IRQHandling(uint8_t PinNumber);                                                                        //Whenever interrupt occurs , this function can be used to process the interrupt




#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
