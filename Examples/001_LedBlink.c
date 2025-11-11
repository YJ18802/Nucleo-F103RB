
# include "stm32f103xx.h"


void delay()
{


	for (uint32_t i =0; i <=4000000;i++);

}

int main (void)
{

	GPIO_Handle_t GpioLed ;

	GpioLed.pGPIOx = GPIOA ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PushPull;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5 ;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while (1)
	{

		GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
		delay();

	}



	return 0;
}




