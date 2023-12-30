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

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void IR_NEC_Init(TIM_HandleTypeDef *tim, uint32_t channel);
uint32_t IR_NEC_GetErrorCnt(void);
__weak void IR_NEC_Parser (uint8_t address, uint8_t command);


#endif /* IR_IR_NEC_RX_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
