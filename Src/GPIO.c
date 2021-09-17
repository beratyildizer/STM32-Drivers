/*
 * GPIO.c
 *
 *  Created on: 25 Haz 2021
 *      Author: berat
 */

#include "GPIO.h"

/**
  * @brief  GPIO_Init; Configures the port and pin
  * @param  GPIOx; GPIO Port Base Address
  *
  * @param  GPIO_InitTypeDef_t; User Config Structures
  *
  * @retval Void
  */

void GPIO_Init(GPIO_TypeDef_t *GPIOx, GPIO_InitTypeDef_t *GPIO_ConfigStruct)
{
	uint32_t position;
	uint32_t fakePosition=0;
	uint32_t lastPosition=0;
	for(position=0; position<16; position++)
	{
		fakePosition = (0x1 << position);
		lastPosition = (uint32_t)(GPIO_ConfigStruct->pinNumber) & fakePosition;

		if(fakePosition == lastPosition)
		{
			// Mode Config

			uint32_t tempValue = GPIOx->MODER;

			tempValue &= ~(0x3U << (position * 2));
			tempValue |=  (GPIO_ConfigStruct->Mode << (position * 2));

			GPIOx->MODER = tempValue;

			if(GPIO_ConfigStruct->Mode == GPIO_MODE_OUTPUT || GPIO_ConfigStruct->Mode == GPIO_MODE_AF)
			{
				// Output Type Config

				tempValue = GPIOx->OTYPER;
				tempValue &=  ~(0x1U << position);
				tempValue |= (GPIO_ConfigStruct->Otype << position);

				GPIOx->OTYPER = tempValue;

				// Output Speed Config

				tempValue = GPIOx->OSPEEDR;
				tempValue &= ~(0x3U << (position * 2));
				tempValue |= (GPIO_ConfigStruct->Speed << position);

				GPIOx->OSPEEDR = tempValue;

			}

			// Push Pull Config

			tempValue = GPIOx->PUPDR;
			tempValue &= ~(0x3U << (position * 2));
			tempValue |= (GPIO_ConfigStruct->PuPd << (position * 2));

			GPIOx->PUPDR = tempValue;

			// alternate function
			if(GPIO_ConfigStruct->Mode == GPIO_MODE_AF)
				{
					tempValue = GPIOx->AFR[position >> 3U]; // 8 e böldük pin numberi
					tempValue &= ~(0xFU << ((position & 0x7U) * 4)); // 8 e göre modunu aldık ilgili registarda ilgili yeri bulmak için
					tempValue |= (GPIO_ConfigStruct->Alternate << ((position & 0x7U) * 4));
					GPIOx->AFR[position >> 3U] |= tempValue;

				}


		}


	}
}

/**
  * @brief  GPIO_Write_Pin; Makes Pin High or Low
  * @param  GPIOx; GPIO Port Base Address
  *
  * @param  pinNumber; GPIO Pin Numbers from 0 to 15
  *
  * @param  pinState; GPIO Pin Set or Reset
  *
  * @retval Void
  */

void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState)
{
	if(pinState == GPIO_Pin_Set)
	{
		GPIOx->BSRR = pinNumber;

	}
	else
	{
		GPIOx->BSRR = (pinNumber << 16U);
	}
}

/**
  * @brief  GPIO_Read_Pin; Reads the  pin of GPIOX Port
  * @param  GPIOx; GPIO Port Base Address
  *
  * @param  pinNumber; GPIO Pin Numbers from 0 to 15
  *
  * @retval GPIO_PinState_t
  */

GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber)
{
	GPIO_PinState_t bitStatus = GPIO_Pin_Reset;
	if((GPIOx->IDR & pinNumber) != GPIO_Pin_Reset)
	{
		bitStatus = GPIO_Pin_Set;
	}
	return bitStatus;
}

/**
  * @brief  GPIO_Lock_Pin; Locks the pin of GPIOx port
  * @param  GPIOx; GPIO Port Base Address
  *
  * @param  pinNumber; GPIO Pin Numbers from 0 to 15
  *
  * @retval Void
  */

void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber)
{
	uint32_t tempValue = (0x1U << 16U) | pinNumber;
	GPIOx->LCKR = tempValue;        //  LCKR[16] = '1'    LCKR[15:0] = DATA
	GPIOx->LCKR = pinNumber; 		//  LCKR[16] = '0'    LCKR[15:0] = DATA
	GPIOx->LCKR = tempValue;		//  LCKR[16] = '1'    LCKR[15:0] = DATA
	tempValue = GPIOx->LCKR;		//  Read Lock Register
}

/**
  * @brief  GPIO_TogglePin; Toggle the pin of GPIOx port
  * @param  GPIOx; GPIO Port Base Address
  *
  * @param  pinNumber; GPIO Pin Numbers from 0 to 15
  *
  * @retval Void
  */

void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber)
{
	uint32_t tempValue = GPIOx->ODR;
	GPIOx->BSRR = ((tempValue & pinNumber) << 16U) | (~tempValue & pinNumber);
}
