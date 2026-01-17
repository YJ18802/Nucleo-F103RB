/*
 * 008_I2C_ArduinoMasterSend.c
 *
 *  Created on: 28-Dec-2025
 *      Author: yukta
 */
/*
 * I2C PINS
 * PB8 - SCL
 * PB9 - SDA
 *
 *
 *
 */
# include "stm32f103xx.h"
# include <string.h>
# include <stdio.h>

#define Slave_Address	0x68
uint8_t msg[] = "I2C MASTER To Arduino\n";


I2C_Handle_t I2C1_Handle;
void delay()
{
	for (volatile uint32_t i =0; i < 500000/2 ;i++);
}

void I2C1_Handle_Init (void)
{

	I2C1_Handle.pI2Cx =I2C_1 ;
	I2C1_Handle.I2C_Config.I2C_ReMap = 1;
	I2C1_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_EN;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCLSpeed_STD;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress=0x61;

	I2C_Init(&I2C1_Handle);

}



void I2C1_Handle_GPIO_Init(void)
{
	GPIO_Handle_t I2C1_Handle_Pin;

	I2C1_Handle_Pin.pGPIOx = GPIOB;
	I2C1_Handle_Pin.GPIO_PinConfig.GPIO_AFIOControl =1;
	I2C1_Handle_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN_OpenDrain ;

	//SCL
	I2C1_Handle_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&I2C1_Handle_Pin);


	//SDA
	I2C1_Handle_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2C1_Handle_Pin);

}

void PressButton()
{
		GPIO_Handle_t GpioBtn;

		GpioBtn.pGPIOx = GPIOA ;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PullUpDwn;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8 ;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PinPullUp_ODR;

		GPIO_Init(&GpioBtn);



}

int main (void)
{


//Enable I2C1_Handle peripheral pins
I2C1_Handle_GPIO_Init();
//Enable I2C Configuration
I2C1_Handle_Init();
//Enable I2C CR1 Enable bit
EnableI2C(I2C_1,1);



while(1)
{

	I2C_MasterSendData(&I2C1_Handle, msg, strlen((char*)msg), Slave_Address);


}


return 0;
}
