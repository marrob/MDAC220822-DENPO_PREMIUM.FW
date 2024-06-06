/*
 * ir_samsung.h
 *
 *  Created on: Dec 29, 2023
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IR_SAMSUNG_H_
#define IR_SAMSUNG_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define IR_POWER        0x070702FD
#define IR_MUTE         0x07070FF0
#define IR_CH_UP        0x070712ED
#define IR_VOLUME_UP    0x01
#define IR_VOLUME_DOWN  0x02

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void IR_SAM_Init(TIM_HandleTypeDef *tim, uint32_t channel);
#endif /* IR_SAMSUNG_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
