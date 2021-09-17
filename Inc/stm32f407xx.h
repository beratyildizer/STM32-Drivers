/*
 * stm32f407xx.h
 *
 *  Created on: Jun 19, 2021
 *      Author: berat
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <string.h>
#include <stddef.h>

/*
 *  Microprocessor Defines for NVIC
 */


#define NVIC_ISER0 				((uint32_t *)(0xE000E100)) // NVIC ınterrupt set enable register base adress

#define _IO volatile

#define SET_BIT(REG, BIT)		((REG) |=  (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)		((REG) &   (BIT))
#define unused(x)				(void)x


typedef enum
{
	DISABLE = 0x0U,
	ENABLE = !DISABLE,
}FunctionalState_t;


/*
 *  IRQ Numbers of MCU == Vector Table
 */

typedef enum
{
	EXTI0_IRQNumber = 6,
	EXTI1_IRQNumber = 7,
	EXTI2_IRQNumber = 8,
	EXTI3_IRQNumber = 9,
	EXTI4_IRQNumber = 10,
	EXTI9_5_IRQNumber = 23,
	EXTI10_15_IRQNumber = 40,
	SPI1_IRQNumber = 35,
	USART2_IRQn = 38,

}EXTI_IRQNumber;


/*
 *  Memory Base Adress
 */

#define FLASH_BASE_ADDR			(0x80000000UL)	 // Flash Base Adress (up to 1MB)
#define SRAM1_BASE_ADDR			(0x20000000UL)	 // SRAM1 Base Adress (up to 112KB)
#define SRAM2_BASE_ADDR			(0x2001C000UL) 	 // SRAM1 Base Adress (up to 16KB)

/*
 *  Peripheral Base Adresses
 */

#define PERIPH_BASE_ADDR		(0x40000000UL)   				   // Base Adress for All Peripherals
#define APB1_BASE_ADDR			(PERIPH_BASE_ADDR + 0x00000000UL)  // APB1 Bus Domain Base Adress
#define APB2_BASE_ADDR			(PERIPH_BASE_ADDR + 0X00010000UL)  // APB2 Bus Domain Base Adress
#define AHB1_BASE_ADDR			(PERIPH_BASE_ADDR + 0x00020000UL)  // AHB1 Bus Domain Base Adress
#define AHB2_BASE_ADDR			(PERIPH_BASE_ADDR + 0x10000000UL)  // AHB2 Bus Domain Base Adress

/*
 *  APB1 Peripherals Base Adresses
 */

#define TIM2_BASE_ADDR 			(APB1_BASE_ADDR + 0x00000000UL) // Timer2 Base Adress
#define TIM3_BASE_ADDR 			(APB1_BASE_ADDR + 0x00000400UL) // Timer3 Base Adress
#define TIM4_BASE_ADDR 			(APB1_BASE_ADDR + 0x00000800UL) // Timer4 Base Adress
#define TIM5_BASE_ADDR 			(APB1_BASE_ADDR + 0x00000C00UL) // Timer5 Base Adress
#define TIM6_BASE_ADDR 			(APB1_BASE_ADDR + 0x00001000UL) // Timer6 Base Adress
#define TIM7_BASE_ADDR 			(APB1_BASE_ADDR + 0x00001400UL) // Timer7 Base Adress

#define SPI2_BASE_ADDR			(APB1_BASE_ADDR + 0x00003800UL) // SPI2 Base Adress
#define SPI3_BASE_ADDR			(APB1_BASE_ADDR + 0x00003C00UL) // SPI3 Base Adress

#define USART2_BASE_ADDR 		(APB1_BASE_ADDR + 0x00004400UL) // USART2 Base Adress
#define USART3_BASE_ADDR 		(APB1_BASE_ADDR + 0x00004800UL) // USART3 Base Adress
#define UART4_BASE_ADDR 		(APB1_BASE_ADDR + 0x00004C00UL) // UART4 Base Adress
#define UART5_BASE_ADDR 		(APB1_BASE_ADDR + 0x00005000UL) // UART5 Base Adress
#define USART7_BASE_ADDR 		(APB1_BASE_ADDR + 0x00007800UL) // USART7 Base Adress
#define USART8_BASE_ADDR 		(APB1_BASE_ADDR + 0x00007C00UL) // USART8 Base Adress


#define I2C1_BASE_ADDR			(APB1_BASE_ADDR + 0x00005400UL) // I2C1 Base Adress
#define I2C2_BASE_ADDR			(APB1_BASE_ADDR + 0x00005800UL) // I2C2 Base Adress
#define I2C3_BASE_ADDR			(APB1_BASE_ADDR + 0x00005C00UL) // I2C3 Base Adress

/*
 *  APB2 Peripherals Base Adresses
 */

#define TIM1_BASE_ADDR			(APB2_BASE_ADDR + 0x00000000UL) // TIM1 Base Adress
#define TIM8_BASE_ADDR			(APB2_BASE_ADDR + 0x00000400UL) // TIM8 Base Adress

#define USART1_BASE_ADDR		(APB2_BASE_ADDR + 0x00001000UL) // USART1 Base Adress
#define USART6_BASE_ADDR		(APB2_BASE_ADDR + 0x00001400UL) // USART6 Base Adress

#define SPI1_BASE_ADDR			(APB2_BASE_ADDR + 0x00003000UL) // SPI1 Base Adress
#define SPI4_BASE_ADDR			(APB2_BASE_ADDR + 0x00003400UL) // SPI4 Base Adress

#define SYSCFG_BASE_ADDR		(APB2_BASE_ADDR + 0x00003800UL) // SYSCFG Base Adress
#define EXTI_BASE_ADDR			(APB2_BASE_ADDR + 0x00003C00UL) // EXTI Base Adress

/*
 *  AHB1 Peripherals Base Adresses
 */

#define GPIOA_BASE_ADDR			(AHB1_BASE_ADDR + 0x00000000UL) // GPIOA Base Adress
#define GPIOB_BASE_ADDR			(AHB1_BASE_ADDR + 0x00000400UL) // GPIOB Base Adress
#define GPIOC_BASE_ADDR			(AHB1_BASE_ADDR + 0x00000800UL) // GPIOC Base Adress
#define GPIOD_BASE_ADDR			(AHB1_BASE_ADDR + 0x00000C00UL) // GPIOD Base Adress
#define GPIOE_BASE_ADDR			(AHB1_BASE_ADDR + 0x00001000UL) // GPIOE Base Adress
#define GPIOF_BASE_ADDR			(AHB1_BASE_ADDR + 0x00001400UL) // GPIOF Base Adress
#define GPIOG_BASE_ADDR			(AHB1_BASE_ADDR + 0x00001800UL) // GPIOG Base Adress
#define GPIOH_BASE_ADDR			(AHB1_BASE_ADDR + 0x00001C00UL) // GPIOH Base Adress
#define GPIOI_BASE_ADDR			(AHB1_BASE_ADDR + 0x00002000UL) // GPIOI Base Adress
#define GPIOJ_BASE_ADDR			(AHB1_BASE_ADDR + 0x00002400UL) // GPIOJ Base Adress
#define GPIOK_BASE_ADDR			(AHB1_BASE_ADDR + 0x00002800UL) // GPIOK Base Adress

#define RCC_BASE_ADDR			(AHB1_BASE_ADDR + 0x00003800UL) // RCC Base Adress

/*
 *  Peripheral Structure Definitions
 */

typedef struct
{

	_IO uint32_t MODER;  		// GPIO port mode register 					 Address Offset = 0x0000
	_IO uint32_t OTYPER;    	// GPIO port output type register 			 Address Offset = 0x0004
	_IO uint32_t OSPEEDR;   	// GPIO port output speed register			 Address Offset = 0x0008
	_IO uint32_t PUPDR;     	// GPIO port pull-up/pull-down register		 Address Offset = 0x000C
	_IO uint32_t IDR;       	// GPIO port input data register			 Address Offset = 0x0010
	_IO uint32_t ODR;       	// GPIO port output data register			 Address Offset = 0x0014
	_IO uint32_t BSRR;      	// GPIO port bit set/reset register			 Address Offset = 0x0018
	_IO uint32_t LCKR;      	// GPIO port configuration lock register	 Address Offset = 0x001C
	_IO uint32_t AFR[2];    	// GPIO port alternate function register	 Address Offset = 0x0020

}GPIO_TypeDef_t;

typedef struct
{
	_IO uint32_t CR;        	// RCC clock control register				 					Address Offset = 0x0000
	_IO uint32_t PLLCFGR;		// RCC PLL configuration register			 					Address Offset = 0x0004
	_IO uint32_t CFGR; 			// RCC clock configuration register			 					Address Offset = 0x0008
	_IO uint32_t CIR;			// RCC clock interrupt register				 					Address Offset = 0x000C
	_IO uint32_t AHB1RSTR;		// RCC AHB1 peripheral reset register		 					Address Offset = 0x0010
	_IO uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register		 					Address Offset = 0x0014
	_IO uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register		 					Address Offset = 0x0018
	_IO uint32_t RESERVED0;		// REVERSED0
	_IO uint32_t APB1RSTR;		// RCC APB1 peripheral reset register		 					Address Offset = 0x0020
	_IO uint32_t APB2RSTR;		// RCC APB2 peripheral reset register		 					Address Offset = 0x0024
	_IO uint32_t RESERVED1[2];  // RESERVED1[2]
	_IO uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register 					Address Offset = 0x0030
	_IO uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register 					Address Offset = 0x0034
	_IO uint32_t AHB3ENR;		// RCC AHB3 peripheral clock enable register 					Address Offset = 0x0038
	_IO uint32_t RESERVED2;		// REVERSED2
	_IO uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register 					Address Offset = 0x0040
	_IO uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register 					Address Offset = 0x0044
	_IO uint32_t RESERVED3[2];	// RESERVED3[2]
	_IO uint32_t AHB1LPENR;		// RCC AHB1 peripheral clock enable in low power mode register	Address Offset = 0x0050
	_IO uint32_t AHB2LPENR;		// RCC AHB2 peripheral clock enable in low power mode register	Address Offset = 0x0054
	_IO uint32_t AHB3LPENR;		// RCC AHB3 peripheral clock enable in low power mode register	Address Offset = 0x0058
	_IO uint32_t RESERVED4;		// RESERVED4
	_IO uint32_t APB1LPENR;		// RCC APB1 peripheral clock enable in low power mode register	Address Offset = 0x0060
	_IO uint32_t APB2LPENR;		// RCC APB2 peripheral clock enable in low power mode register	Address Offset = 0x0064
	_IO uint32_t RESERVE5[2];	// RESERVE5[2];
	_IO uint32_t BDCR;			// RCC Backup domain control register		 					Address Offset = 0x0070
	_IO uint32_t CSR;			// RCC clock control & status register		 					Address Offset = 0x0074
	_IO uint32_t RESERVED6[2];	// RESERVED[6]2
	_IO uint32_t SSCGR;			// RCC spread spectrum clock register        					Address Offset = 0x0080
	_IO uint32_t PLLI2SCFGR;	// RCC PLLI2S configuration register		 					Address Offset = 0x0084
}RCC_TypeDef_t;

typedef struct
{
	_IO uint32_t MEMRPM;  		// SYSCFG memory remap register									Address Offset = 0X0000
	_IO uint32_t PMC;			// SYSCFG peripheral mode configuration register				Address Offset = 0X0004
	_IO uint32_t EXTI_CR[4];		// SYSCFG external interrupt configuration registers			Address Offset = 0X0008
	_IO uint32_t RESERVED[2];
	_IO uint32_t CMPCR;			// Compensation cell control register							Address Offset = 0X0020

}SYSCGF_TypeDef_t;

typedef struct
{
	_IO uint32_t IMR;			// Interrupt mask register 										Address Offset = 0X0000
	_IO uint32_t EMR;			// Event mask register											Address Offset = 0X0004
	_IO uint32_t RTSR;			// Rising trigger selection register							Address Offset = 0X0008
	_IO uint32_t FTSR;			// Falling trigger selection register							Address Offset = 0X000C
	_IO uint32_t SWIER;			// Software interrupt event register							Address Offset = 0X0010
	_IO uint32_t PR;			// Pending register												Address Offset = 0X0014

}EXTI_TypeDef_t;

typedef struct
{
	_IO uint32_t CR1;			// SPI control register 1										Address Offset = 0X0000
	_IO uint32_t CR2;			// SPI control register 2										Address Offset = 0X0004
	_IO uint32_t SR;			// SPI status register											Address Offset = 0X0008
	_IO uint32_t DR;			// SPI data register											Address Offset = 0X000C
	_IO uint32_t CRCPR;			// SPI CRC polynomial register									Address Offset = 0X0010
	_IO uint32_t RXCRCR;		// SPI RX CRC register											Address Offset = 0X0014
	_IO uint32_t TXCRCR;		// SPI TX CRC register											Address Offset = 0X0018
	_IO uint32_t I2SCFGR;		// SPI_I2S configuration register								Address Offset = 0X001C
	_IO uint32_t I2SPR;			// SPI_I2S prescaler data register								Address Offset = 0X0020

}SPI_TypeDef_t;

typedef struct
{
	_IO uint32_t SR;			// USART Status Register 										Address Offset = 0X0000
	_IO uint32_t DR;			// USART Data Register											Address Offset = 0X0004
	_IO uint32_t BRR;			// USART Baud Rate Register										Address Offset = 0X0008
	_IO uint32_t CR1;			// USART Control Register 1										Address Offset = 0X000C
	_IO uint32_t CR2;			// USART Control Register 1										Address Offset = 0X0010
	_IO uint32_t CR3;			// USART Control Register 1										Address Offset = 0X0014
	_IO uint32_t GTPR;			// Guard Time and Prescaler Register							Address Offset = 0X0018

}Usart_TypeDef_t;

#define GPIOA					((GPIO_TypeDef_t *)(GPIOA_BASE_ADDR))
#define GPIOB					((GPIO_TypeDef_t *)(GPIOB_BASE_ADDR))
#define GPIOC					((GPIO_TypeDef_t *)(GPIOC_BASE_ADDR))
#define GPIOD					((GPIO_TypeDef_t *)(GPIOD_BASE_ADDR))
#define GPIOE					((GPIO_TypeDef_t *)(GPIOE_BASE_ADDR))
#define GPIOF					((GPIO_TypeDef_t *)(GPIOF_BASE_ADDR))
#define GPIOG					((GPIO_TypeDef_t *)(GPIOG_BASE_ADDR))
#define GPIOH					((GPIO_TypeDef_t *)(GPIOH_BASE_ADDR))
#define GPIOI					((GPIO_TypeDef_t *)(GPIOI_BASE_ADDR))
#define GPIOJ					((GPIO_TypeDef_t *)(GPIOJ_BASE_ADDR))
#define GPIOK					((GPIO_TypeDef_t *)(GPIOK_BASE_ADDR))

#define RCC 					((RCC_TypeDef_t * )(RCC_BASE_ADDR)  )

#define SYSCFG					((SYSCGF_TypeDef_t *)(SYSCFG_BASE_ADDR))

#define EXTI					((EXTI_TypeDef_t *)(EXTI_BASE_ADDR))

#define SPI1					((SPI_TypeDef_t*)(SPI1_BASE_ADDR))
#define SPI2					((SPI_TypeDef_t*)(SPI2_BASE_ADDR))
#define SPI3					((SPI_TypeDef_t*)(SPI3_BASE_ADDR))
#define SPI4					((SPI_TypeDef_t*)(SPI4_BASE_ADDR))

#define USART1					((Usart_TypeDef_t*)(USART1_BASE_ADDR))
#define USART2					((Usart_TypeDef_t*)(USART2_BASE_ADDR))
#define USART3					((Usart_TypeDef_t*)(USART3_BASE_ADDR))
#define UART4					((Usart_TypeDef_t*)(UART4_BASE_ADDR))
#define UART5					((Usart_TypeDef_t*)(UART5_BASE_ADDR))
#define USART6					((Usart_TypeDef_t*)(USART6_BASE_ADDR))
#define USART7					((Usart_TypeDef_t*)(USART7_BASE_ADDR))
#define USART8					((Usart_TypeDef_t*)(USART8_BASE_ADDR))



#define RCC_AHB1ENR_GPIOAEN_Pos					(0U)								// RCC AHB1ENR register GPIOAEN BitPosition
#define RCC_AHB1ENR_GPIOAEN_Msk					(0x1 << RCC_AHB1ENR_GPIOAEN_Pos)    // RCC AHB1ENR register GPIOAEN BitMask
#define RCC_AHB1ENR_GPIOAEN						RCC_AHB1ENR_GPIOAEN_Msk				// RCC AHB1ENR register GPIOAEN Macro

#define RCC_AHB1ENR_GPIOBEN_Pos					(1U)								// RCC AHB1ENR register GPIOBEN BitPosition
#define RCC_AHB1ENR_GPIOBEN_Msk					(0x1 << RCC_AHB1ENR_GPIOBEN_Pos)    // RCC AHB1ENR register GPIOBEN BitMask
#define RCC_AHB1ENR_GPIOBEN						RCC_AHB1ENR_GPIOBEN_Msk				// RCC AHB1ENR register GPIOBEN Macro

#define RCC_AHB1ENR_GPIOCEN_Pos					(2U)								// RCC AHB1ENR register GPIOCEN BitPosition
#define RCC_AHB1ENR_GPIOCEN_Msk					(0x1 << RCC_AHB1ENR_GPIOCEN_Pos)    // RCC AHB1ENR register GPIOCEN BitMask
#define RCC_AHB1ENR_GPIOCEN						RCC_AHB1ENR_GPIOCEN_Msk				// RCC AHB1ENR register GPIOCEN Macro

#define RCC_AHB1ENR_GPIODEN_Pos					(3U)								// RCC AHB1ENR register GPIODEN BitPosition
#define RCC_AHB1ENR_GPIODEN_Msk					(0x1 << RCC_AHB1ENR_GPIODEN_Pos)    // RCC AHB1ENR register GPIODEN BitMask
#define RCC_AHB1ENR_GPIODEN						RCC_AHB1ENR_GPIODEN_Msk				// RCC AHB1ENR register GPIODEN Macro

#define RCC_AHB1ENR_GPIOEEN_Pos					(4U)								// RCC AHB1ENR register GPIOEEN BitPosition
#define RCC_AHB1ENR_GPIOEEN_Msk					(0x1 << RCC_AHB1ENR_GPIOEEN_Pos)    // RCC AHB1ENR register GPIOEEN BitMask
#define RCC_AHB1ENR_GPIOEEN						RCC_AHB1ENR_GPIOEEN_Msk				// RCC AHB1ENR register GPIOEEN Macro

#define RCC_AHB1ENR_GPIOFEN_Pos					(5U)								// RCC AHB1ENR register GPIOFEN BitPosition
#define RCC_AHB1ENR_GPIOFEN_Msk					(0x1 << RCC_AHB1ENR_GPIOFEN_Pos)    // RCC AHB1ENR register GPIOFEN BitMask
#define RCC_AHB1ENR_GPIOFEN						RCC_AHB1ENR_GPIOFEN_Msk				// RCC AHB1ENR register GPIOFEN Macro

#define RCC_AHB1ENR_GPIOGEN_Pos					(6U)								// RCC AHB1ENR register GPIOGEN BitPosition
#define RCC_AHB1ENR_GPIOGEN_Msk					(0x1 << RCC_AHB1ENR_GPIOGEN_Pos)    // RCC AHB1ENR register GPIOGEN BitMask
#define RCC_AHB1ENR_GPIOGEN						RCC_AHB1ENR_GPIOGEN_Msk				// RCC AHB1ENR register GPIOGEN Macro

#define RCC_AHB1ENR_GPIOHEN_Pos					(7U)								// RCC AHB1ENR register GPIOHEN BitPosition
#define RCC_AHB1ENR_GPIOHEN_Msk					(0x1 << RCC_AHB1ENR_GPIOHEN_Pos)    // RCC AHB1ENR register GPIOHEN BitMask
#define RCC_AHB1ENR_GPIOHEN						RCC_AHB1ENR_GPIOHEN_Msk				// RCC AHB1ENR register GPIOHEN Macro

#define RCC_AHB1ENR_GPIOIEN_Pos					(8U)								// RCC AHB1ENR register GPIOIEN BitPosition
#define RCC_AHB1ENR_GPIOIEN_Msk					(0x1 << RCC_AHB1ENR_GPIOIEN_Pos)    // RCC AHB1ENR register GPIOIEN BitMask
#define RCC_AHB1ENR_GPIOIEN						RCC_AHB1ENR_GPIOIEN_Msk				// RCC AHB1ENR register GPIOIEN Macro

#define RCC_APB1ENR_USART2EN_Pos				(17U)								// RCC APB1EN register USART2 BitPosition
#define RCC_APB1ENR_USART2EN_Msk				(0x1U << RCC_APB1ENR_USART2EN_Pos)	// RCC APB1EN register USART2 BitMask
#define RCC_APB1ENR_USART2EN					RCC_APB1ENR_USART2EN_Msk			// RCC APB1EN register USART2 Macro

#define RCC_APB2ENR_SYSCFG_Pos					(14U)								// RCC APB2ENR register SYSCFG BitPosition
#define RCC_APB2ENR_SYSCFG_Msk					(0x1 << RCC_APB2ENR_SYSCFG_Pos)		// RCC APB2ENR register SYSCFG BitMask
#define RCC_APB2ENR_SYSCFG 						RCC_APB2ENR_SYSCFG_Msk				// RCC APB2ENR register SYSCFG Macro

#define RCC_APB2ENR_SPI1_Pos					(12U)								// RCC APB2ENR register SPI1 BitPosition
#define RCC_APB2ENR_SPI1_Msk					(0x1 << RCC_APB2ENR_SPI1_Pos)		// RCC APB2ENR register SPI1 BitMask
#define RCC_APB2ENR_SPI1EN 						RCC_APB2ENR_SPI1_Msk				// RCC APB2ENR register SPI1 Macro

#define RCC_APB1ENR_SPI2_Pos					(14U)								// RCC APB1ENR register SPI2 BitPosition
#define RCC_APB1ENR_SPI2_Msk					(0x1 << RCC_APB1ENR_SPI2_Pos)		// RCC APB1ENR register SPI2 BitMask
#define RCC_APB1ENR_SPI2EN 						RCC_APB1ENR_SPI2_Msk				// RCC APB1ENR register SPI2 Macro

#define RCC_APB1ENR_SPI3_Pos					(15U)								// RCC APB1ENR register SPI3 BitPosition
#define RCC_APB1ENR_SPI3_Msk					(0x1 << RCC_APB1ENR_SPI3_Pos)		// RCC APB1ENR register SPI3 BitMask
#define RCC_APB1ENR_SPI3EN 						RCC_APB1ENR_SPI3_Msk				// RCC APB1ENR register SPI3 Macro

#define SPI_SR_BUSSY							(7U)
#define SPI_SR_Txe								(1U)
#define SPI_SR_Rxe								(0U)

#define SPI_CR1_SPE								(6U)
#define SPI_CR1_DFF								(11U)

#define SPI_CR2_RXEIE							(6U)
#define SPI_CR2_TXEIE							(7U)

#define USART_CR1_UE							(13U)
#define USART_CR1_TxEIE							(7U)
#define USART_CR1_RxNEIE						(5U)
#define UART_CR2_STOP							(12U)

#define USART_SR_Txe							(7U)
#define USART_SR_TC								(6U)

#define USART_SR_RxNE							(5U)





/*
 * Flag Definitions
 */

#define SPI_TxE_FLAG							(0x1U << SPI_SR_Txe)
#define SPI_RxE_FLAG							(0x1U << SPI_SR_Rxe)
#define SPI_Bussy_FLAG							(0x1U << SPI_SR_BUSSY)


#define USART_Txe_FLAG							(0x1U << USART_SR_Txe)
#define USART_TC_FLAG							(0x1U << USART_SR_TC)
#define USART_RxNE_FLAG							(0x1U << USART_SR_RxNE)



#include "RCC.h"
#include "GPIO.h"
#include "EXTI.h"
#include "SPI.h"
#include "USART.h"
















#endif /* INC_STM32F407XX_H_ */
