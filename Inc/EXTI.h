/*
 * EXTI.h
 *
 *  Created on: 15 Tem 2021
 *      Author: berat
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "stm32f407xx.h"


/*
 *  @def_group PORT_Values
 */

#define EXTI_PortSource_GPIOA					((uint8_t)(0x0))
#define EXTI_PortSource_GPIOB					((uint8_t)(0x1))
#define EXTI_PortSource_GPIOC					((uint8_t)(0x2))
#define EXTI_PortSource_GPIOD					((uint8_t)(0x3))
#define EXTI_PortSource_GPIOE					((uint8_t)(0x4))
#define EXTI_PortSource_GPIOF					((uint8_t)(0x5))
#define EXTI_PortSource_GPIOG					((uint8_t)(0x6))
#define EXTI_PortSource_GPIOH					((uint8_t)(0x7))
#define EXTI_PortSource_GPIOI					((uint8_t)(0x8))

/*
 *  @def_group LINE_Values
 */

#define EXTI_LineSource_0						((uint8_t)(0x0))
#define EXTI_LineSource_1						((uint8_t)(0x1))
#define EXTI_LineSource_2						((uint8_t)(0x2))
#define EXTI_LineSource_3						((uint8_t)(0x3))
#define EXTI_LineSource_4						((uint8_t)(0x4))
#define EXTI_LineSource_5						((uint8_t)(0x5))
#define EXTI_LineSource_6						((uint8_t)(0x6))
#define EXTI_LineSource_7						((uint8_t)(0x7))
#define EXTI_LineSource_8						((uint8_t)(0x8))
#define EXTI_LineSource_9						((uint8_t)(0x9))
#define EXTI_LineSource_10						((uint8_t)(0xA))
#define EXTI_LineSource_11						((uint8_t)(0xB))
#define EXTI_LineSource_12						((uint8_t)(0xC))
#define EXTI_LineSource_13						((uint8_t)(0xD))
#define EXTI_LineSource_14						((uint8_t)(0xE))
#define EXTI_LineSource_15						((uint8_t)(0xF))

/*
 * @def_group EXTI_Modes
 */

#define EXTI_Mode_Interrupt		(0x0U)
#define EXTI_Mode_Event			(0x4U)

/*
 * @def_group EXTI_TriggerSelection
 */

#define EXTI_Trigger_RisingEdge		(0x8U)
#define EXTI_Trigger_FallingEdge	(0xCU)
#define EXTI_Trigger_BothOfThem		(0x0U)



typedef struct
{
	uint8_t EXTI_LineMumber;		 // EXTI Line Number for valid GPIO pin  @def_group LINE_Values
	uint8_t TriggerSelection;		 // EXTI Trigger Selection @def_group EXTI_TriggerSelection
	uint8_t EXTI_Mode;				 // EXTI Mode values @def_group EXTI_Modes
	FunctionalState_t EXTI_LineCmd;  // Mask or Unmask the line number

}EXTI_InitTypeDef_t;

void EXTI_Init(EXTI_InitTypeDef_t *EXTI_InitStruct);
void EXTI_LineConfig(uint8_t PortSource, uint8_t LineSource);
void NVIC_EnableInterrupt(EXTI_IRQNumber IRQNumber);



#endif /* INC_EXTI_H_ */
