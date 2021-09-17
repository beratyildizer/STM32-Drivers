/*
 * EXTI.c
 *
 *  Created on: 15 Tem 2021
 *      Author: berat
 */


#include "EXTI.h"


/**
  * @brief  EXTI_Init for valid GPIO port and Line number
  * @param  EXTI_InitStruct; User Config structure
  *
  * @retval Void
  */


void EXTI_Init(EXTI_InitTypeDef_t *EXTI_InitStruct)
{
	uint32_t tempValue = 0;

	tempValue = (uint32_t)EXTI_BASE_ADDR; // adresi aldık.

	// IMR ve EMR nin ilgili bitlerini girilecek line numbera göre temizledik.
	EXTI->IMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineMumber);
	EXTI->EMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineMumber);

	// maskeleme yi kaldırma kontrol, yani interrupt veya event oluşumu enable edildi mi
	if(EXTI_InitStruct->EXTI_LineCmd != DISABLE)
	{
		// event veya interrupt seçimini tempValue ye ekliyoruz. Böylece ilgili register adresini int olarak elde ediypruz.
		tempValue += EXTI_InitStruct->EXTI_Mode;
		// Elde edilen adresde bulunan değere pointer ile ulaşıp, maskelemeyi kaldırıyorz. Bu adres ya event ya interrupt mask register oluyor.
		*((_IO uint32_t*)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineMumber);

		tempValue = (uint32_t)EXTI_BASE_ADDR;

		EXTI->RTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineMumber);
		EXTI->FTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineMumber);

		if(EXTI_InitStruct->TriggerSelection == EXTI_Trigger_BothOfThem	)
		{
			EXTI->RTSR |= (0x1U << EXTI_InitStruct->EXTI_LineMumber);
			EXTI->FTSR |= (0x1U << EXTI_InitStruct->EXTI_LineMumber);

		}
		else
		{
			tempValue += EXTI_InitStruct->TriggerSelection;
			*((_IO uint32_t*)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineMumber);
		}

	}
	else
	{
		tempValue += EXTI_InitStruct->EXTI_Mode;

		*((_IO uint32_t*)tempValue) &= ~(0x1U << EXTI_InitStruct->EXTI_LineMumber);


	}

}

/**
  * @brief  EXTI_LineConfig; Configures the port and pin for SYSCFG
  * @param  PortSource; Port Value A-I @def_group PORT_Values
  *
  * @param  LineSource; Line Value 0-15 @def_group LINE_Values
  *
  * @retval Void
  */

void EXTI_LineConfig(uint8_t PortSource, uint8_t LineSource)
{
	// the important point here is the algorithm.
	uint32_t tempValue = SYSCFG->EXTI_CR[LineSource >> 2U]; // line source registere ulaşmak için 4 e böldük.
	tempValue &= ~(0xFU << (LineSource & 0x3U) * 4 );       // bite ulaşmak için 4 e göre mod alıp 4 ile çarptık.
	tempValue |= (PortSource << (LineSource & 0x3U) * 4);
	SYSCFG->EXTI_CR[LineSource >> 2U] = tempValue;
}

void NVIC_EnableInterrupt(EXTI_IRQNumber IRQNumber)
{
	uint32_t tempValue;

	// burada pointer aritmetiği var, NVIC_ISER0 pointer adress tutuyor ve her 1 artışta 4 byte lık artış yapar.
	tempValue  = *((IRQNumber >> 5U) + NVIC_ISER0); // 32 ye böldük
	tempValue &= ~(0x1U << (IRQNumber & 0x1FU)); // 32 ye modunu aldık
	tempValue |= (0x1U << (IRQNumber & 0x1FU));
	*((IRQNumber >> 5U) + NVIC_ISER0) = tempValue;


}


