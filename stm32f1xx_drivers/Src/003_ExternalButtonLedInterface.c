/*
 * 003_ExternalButtonLedInterface.c
 *
 *  Created on: Nov 10, 2025
 *      Author: yukta
 */



# include "stm32f103xx.h"

void delay()
{


	for (uint32_t i =0; i <=100000;i++);

}

int main (void)
{

	GPIO_Handle_t GpioLed, GpioBtn ;

	GpioLed.pGPIOx = GPIOC ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PushPull;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7 ;

	GpioBtn.pGPIOx = GPIOB ;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PullUpDwn;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8 ;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PinPullUp_ODR;


	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);


	while (1)
	{

		if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_8) == GPIO_PIN_RESET)
				{
						delay();
						GPIO_TogglePin(GPIOC, GPIO_PIN_NO_7);

				}


		/*if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == GPIO_PIN_RESET)
		{

				GPIO_WriteToOutputPin(GPIOA, 5, 0);

		}
		else
		{
			GPIO_WriteToOutputPin(GPIOA, 5, 1);
		}

		delay();
		*/
		/*else
		{
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, 1);
		}*/


	}



	return 0;
}

