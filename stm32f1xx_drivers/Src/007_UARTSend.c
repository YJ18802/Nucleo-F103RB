//I am using putty terminal to view the data.


#include "stm32f103xx.h"
#include<string.h>
#include<stdio.h>
char msg[1024] = "Hello World\n\r";
USART_Handle_t USART2_Config;

void delay() {

	for (uint32_t i = 0; i < 500000 / 2; i++);

}

 void USART2_GPIOInit()
{

	 //PA2 -Tx
	 //PA3 -Rx

	GPIO_Handle_t USART_Pin;


	//Tx
	USART_Pin.pGPIOx = GPIOA ;
	USART_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN_PushPull ;
	USART_Pin.GPIO_PinConfig.GPIO_PinNumber = 2 ;
	GPIO_Init(&USART_Pin);

	//Rx
	USART_Pin.pGPIOx = GPIOA ;
	USART_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_Float ;
	USART_Pin.GPIO_PinConfig.GPIO_PinNumber = 3 ;
	GPIO_Init(&USART_Pin);
}

void USART2_Init()
{


  USART2_Config.pUSARTx = USART2;
  USART2_Config.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
  USART2_Config.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
  USART2_Config.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
  USART2_Config.USART_Config.USART_NoOfStopBits= USART_STOPBITS_1;
  USART2_Config.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
  USART2_Config.USART_Config.USART_BaudRate = USART_STD_BAUD_115200;

  USART_Init(&USART2_Config);

}

void Button_Init()
{
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PullUpDwn;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = 1;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;

	GPIO_Init(&GpioBtn);






}


int main(void) {

	Button_Init();

	USART2_GPIOInit();

	USART2_Init();

	while(1)
	{

		//if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == RESET)
		//{

		//to avoid button de-bouncing related issues 200ms of delay
		delay();
		USART_PeripheralControl(USART2, 1);
		USART_SendData(&USART2_Config,(uint8_t*)msg,strlen(msg));
		delay();
		//USART_PeripheralControl(USART2, 0);

		//}


	}








	return 0;
}

