/*
 * 006_ArduinoSPISendOnly.c
 *
 *  Created on: 22-Dec-2025
 *      Author: yukta
 */

# include "stm32f103xx.h"
# include <string.h>

SPI_Handle_t SPI_1;
void delay() {

	for (uint32_t i = 0; i < 500000 / 2; i++);

}

void SPI1_GPIO_Init(void) {
	GPIO_Handle_t SPI1_Pin;

	SPI1_Pin.pGPIOx = GPIOA;
	SPI1_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN_PushPull;

	//SCLK
	SPI1_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPI1_Pin);
	//MOSI
	SPI1_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPI1_Pin);

	//NSS

	SPI1_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPI1_Pin);

}

void SPI1_Init(void) {

	SPI_1.pSPIx = SPI1;
	SPI_1.SPI_PinConfig.SPI_BusConfig = SPI_BusConfig_FullDuplex;
	SPI_1.SPI_PinConfig.SPI_DeviceMode = MSTR_Master;
	SPI_1.SPI_PinConfig.SPI_Speed = fPCLK_256;
	SPI_1.SPI_PinConfig.SPI_DFF = DFF_8;
	SPI_1.SPI_PinConfig.SPI_CPOL = 0;
	SPI_1.SPI_PinConfig.SPI_CPHA = 0;
	SPI_1.SPI_PinConfig.SPI_SSM = 0;

	SPI_Init(&SPI_1);

}

void ButtonInit() {
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PullUpDwn;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = 1;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;

	GPIO_Init(&GpioBtn);

}

int main(void) {

	char user_data[] = "Hello World\n";

	SPI1_GPIO_Init();

	ButtonInit();

	SPI1_Init();

	EnableSSOE_BIT(SPI1, 1);

	while (1) {

		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == RESET)
		{

			delay();

			EnableSpi(SPI1, 1);
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI1, &dataLen, 1);
			SPI_SendData(SPI1, (uint8_t*) user_data, strlen(user_data));

			while (SPI_GetFlagStatus(SPI1, SPI_TXE_FLAG));
			while (SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

			EnableSpi(SPI1, 0);
		}

	}

	return 0;
}
