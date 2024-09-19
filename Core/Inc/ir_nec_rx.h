/*
 * ir_nec_rx.h
 *
 *  Created on: Dec 29, 2023
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IR_IR_NEC_RX_H_
#define IR_IR_NEC_RX_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define IR_POWER        0x45
#define IR_MUTE         0xD6
#define IR_CH_UP        0x99
#define IR_LOVE         0xC1

#define IR_POWER_CHINA  0x4D
#define IR_MUTE_CHINA   0x16
#define IR_CH_UP_CHINA  0x54
#define IR_LOVE_CHINA   0x0C

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void IR_NEC_Init(TIM_HandleTypeDef *tim, uint32_t channel);
uint32_t IR_NEC_GetErrorCnt(void);
__weak void IR_NEC_Parser (uint8_t address, uint8_t command);


#endif /* IR_IR_NEC_RX_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
