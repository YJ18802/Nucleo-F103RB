/*
 * 002_ButtonLedInterface.c
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

	GpioLed.pGPIOx = GPIOA ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PushPull;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5 ;

	GpioBtn.pGPIOx = GPIOC ;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_Float;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13 ;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);


	while (1)
	{

		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == GPIO_PIN_RESET)
				{
						delay();
						GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);

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
