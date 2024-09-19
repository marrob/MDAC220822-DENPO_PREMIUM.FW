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
#define IR_POWER        0x070702FD
#define IR_MUTE         0x07070FF0
#define IR_CH_UP        0x070712ED
#define IR_VOLUME_UP    0x070707F8
#define IR_VOLUME_DOWN  0x07070BF4
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void IR_NEC_Init(TIM_HandleTypeDef *tim, uint32_t channel);
uint32_t IR_NEC_GetErrorCnt(void);
__weak void IR_NEC_Parser (uint8_t address, uint8_t command);


#endif /* IR_IR_NEC_RX_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
