/*
 * RCC.c
 *
 *  Created on: 21 Haz 2021
 *      Author: berat
 */

#ifndef SRC_RCC_C_
#define SRC_RCC_C_

#include "RCC.h"

const uint8_t AHB_Prescaler[] = {0,0,0,0,0,0,0,0,1,2,3,4,6,7,8,9};
const uint8_t APB_Prescalers[] = {0,0,0,0,1,2,3,4};

uint32_t RCC_GetSystemClock(void)
{
	uint32_t SystemCoreClock = 0;
	uint8_t clkSource = 0;
	clkSource = ( ( RCC->CFGR >> 2U ) & (0x3U) );

	switch(clkSource)
	{
	case 0 : SystemCoreClock = 16000000; break;
	case 1 : SystemCoreClock = 	8000000; break;
	// case 2 : burada pll hesabı var aslında yapmadık.
	default : SystemCoreClock = 16000000;

	}

	return SystemCoreClock;
}

uint32_t RCC_GetHClock(void)
{
	uint32_t AHB_PeriphClock = 0;
	uint32_t SystemCoreClock = 0;
	uint8_t HPRE_Value = 0;
	uint8_t tempValue = 0;

	SystemCoreClock = RCC_GetSystemClock();
	/*aşağıda cfgr nin hpre bitlerinde bulunan değeri almak için bu işlem yapıldı. */
	HPRE_Value =( (RCC->CFGR >> 4U) & 0xFU );
	tempValue = AHB_Prescaler[HPRE_Value];
	 /* yukarıda globalde tanımladığım AHB_PeriphClock CFGR registerında bulunan AHB prescaler
	 bölüm değerlerini elde etmek için kaydırılıcak bit sayısını temsil ediyor.
	 yani 16 ya bölüceksem 4 bit sağa kaydırma işlemi yapmam lazım ve dizi de 16 yerine
	gerekli indisde 4 yazdım*/
	AHB_PeriphClock = (SystemCoreClock >> tempValue); // bölme işlemi, yukarda anlattım.


	return AHB_PeriphClock;
}
uint32_t RCC_GetPClock1(void)
{
	uint32_t APB1_PeriphClock = 0;
	uint32_t Hclock = 0;
	uint8_t HPRE1_Value = 0;
	uint8_t tempValue;


	Hclock = RCC_GetHClock();
	HPRE1_Value = ((RCC->CFGR >> 10U) & 0x7U);
	tempValue = APB_Prescalers[HPRE1_Value];
	APB1_PeriphClock = (Hclock >> tempValue);

	return APB1_PeriphClock;

}
uint32_t RCC_GetPClock2(void)
{
	uint32_t APB2_PeriphClock = 0;
	uint32_t Hclock = 0;
	uint8_t HPRE2_Value = 0;
	uint8_t tempValue = 0;

	Hclock = RCC_GetHClock();
	HPRE2_Value = ((RCC->CFGR >> 13U ) & 0x7U);
	tempValue = APB_Prescalers[HPRE2_Value];
	APB2_PeriphClock = ( Hclock >> tempValue);

	return APB2_PeriphClock;

}


#endif /* SRC_RCC_C_ */
