/*
 * 002_ButtonLedInterface.c
 *
 *  Created on: Nov 10, 2025
 *      Author: yukta
 */



# include "stm32f103xx.h"
#include <string.h>

void delay()
{


	for (uint32_t i =0; i <=500000/2;i++);

}


int main (void)
{

	GPIO_Handle_t GpioLed, GpioBtn ;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));

	GpioLed.pGPIOx = GPIOA ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PushPull;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5 ;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);



	GpioBtn.pGPIOx = GPIOB ;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_Float;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8 ;
	GpioBtn.GPIO_PinConfig.GPIO_InterruptMode = GPIO_MODE_INT_FT;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, IRQPriorityNO15);
	GPIO_IRQInterruptConfig (IRQ_NO_EXTI9_5,ENABLE);
	while(1);
	return 0;

}

void EXTI9_5_IRQHandler(void)
	{
		delay();
		GPIO_IRQHandling(GPIO_PIN_NO_8);
		GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
	}

