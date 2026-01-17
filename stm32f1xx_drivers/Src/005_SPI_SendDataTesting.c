/*
 * 005_SPI_SendDataTesting.c
 *
 *  Created on: Nov 19, 2025
 *      Author: yukta
 */
# include "stm32f103xx.h"
# include <string.h>

SPI_Handle_t SPI_1;

char user_data[] = "Hello World";
void delay()
{

	for (uint32_t i =0; i < 500000/2;i++);

}

void SPI1_GPIO_Init(void)
{
	GPIO_Handle_t SPI1_Pin;

	SPI1_Pin.pGPIOx = GPIOA;
	SPI1_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN_PushPull ;
	SPI1_Pin.GPIO_PinConfig.GPIO_AFIOControl =1 ;

	//SCLK
	SPI1_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPI1_Pin);
	//MOSI
	SPI1_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPI1_Pin);


}


void SPI1_Init (void)
{

	SPI_1.pSPIx = SPI1;
	SPI_1.SPI_PinConfig.SPI_BusConfig = SPI_BusConfig_FullDuplex;
	SPI_1.SPI_PinConfig.SPI_DeviceMode = MSTR_Master;
	SPI_1.SPI_PinConfig.SPI_Speed = fPCLK_256;
	SPI_1.SPI_PinConfig.SPI_DFF = DFF_8;
	SPI_1.SPI_PinConfig.SPI_CPOL = 1;
	SPI_1.SPI_PinConfig.SPI_CPHA = 0;
	SPI_1.SPI_PinConfig.SPI_SSM = 1;


	SPI_Init(&SPI_1);


	}


int main(void)
{


  // ButtonInit();

   SPI1_GPIO_Init();

   SPI1_Init();

   EnableSSI_BIT(SPI1,1);

   EnableSpi(SPI1,1);

   SPI_SendData(SPI1,(uint8_t*)user_data, strlen(user_data));

   while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

	SPI_PeriClockControl(SPI1, 0);

	while(1);

	return 0;
}
