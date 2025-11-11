/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Oct 25, 2025
 *      Author: Dell
 */

#include "stm32f103xx_gpio_driver.h"
/*
 * Gpio clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
	}
	else
	{
		       if (pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
				else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}
				else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}
				else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}
				else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}

	}



}



/*
 * Gpio initializtion
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	// Input or Output mode
           uint32_t temp =0 ;


		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <=7)
			{
				// here for pin number greater than 7 we will be using GPIO_CRL
				pGPIOHandle->pGPIOx->GPIO_CRL &= ~(0xF << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))) ;
             	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
             	 pGPIOHandle->pGPIOx->GPIO_CRL |= temp ;
             	 temp = 0 ;

			}
		else
			{
				// here for pin number greater than 7 we will be using GPIO_CRH
				temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)<<(4*((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8));
				pGPIOHandle->pGPIOx->GPIO_CRH &= ~(0xF << (4*((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8))) ;
			    pGPIOHandle->pGPIOx->GPIO_CRH |= temp ;
			    temp = 0 ;


			}

//PullUp and PullDown

	 	 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_PullUpDwn )
	 	 {
	 		 	 uint16_t temp2 = 0;
	 		 	 //pGPIOHandle->pGPIOx->GPIO_ODR &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ) ;
	 		 	temp2 = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl)<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	 			 pGPIOHandle->pGPIOx->GPIO_ODR |= temp ;
	 			temp2=0;

	 	 }

}





void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)

{
	if (pGPIOx == GPIOA)
			{
				GPIOA_Reg_Reset();
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_Reg_Reset();
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_Reg_Reset();
			}
			else if (pGPIOx == GPIOD)
			{
				GPIOD_Reg_Reset();
			}
			else if (pGPIOx == GPIOE)
			{
				GPIOE_Reg_Reset();
			}

}
//AFIO
void AFIO_Init(AFIO_Handle_t *pAFIOHandle)

{
	AFIO_PCLK_EN();

	uint32_t temp =0;

	//Setting EVOE
	temp = (pAFIOHandle->AFIO_PinConfig.AFIO_PinAltFunModeControl) << 7;
	pAFIOHandle->pAFIO->AFIO_EVCR |= temp;
	temp = 0;

	//Setting Port

	temp = (pAFIOHandle->AFIO_PinConfig.AFIO_PinAltFunModePort)<<4;
	pAFIOHandle->pAFIO->AFIO_EVCR |= temp;
	temp = 0;

	//Setting Pin

	temp =(pAFIOHandle->AFIO_PinConfig.AFIO_PinAltFunModePinNumber)<<0;
	pAFIOHandle->pAFIO->AFIO_EVCR |= temp;
	temp = 0;


}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

		uint8_t value = 0;
		value = (uint8_t)((pGPIOx->GPIO_IDR >> PinNumber) & 0x00000001);

		return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
			uint16_t value = 0;
			value = (uint16_t) (pGPIOx->GPIO_IDR) ;

			return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value) {

	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->GPIO_ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->GPIO_ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

	Value = pGPIOx->GPIO_ODR ;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	pGPIOx->GPIO_ODR = pGPIOx->GPIO_ODR ^ (1 << PinNumber);
}
/*
 * Interrupt Configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}
void GPIO_IRQHandling(uint8_t PinNumber) {
}
