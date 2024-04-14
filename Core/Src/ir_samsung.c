/*
 * ir_samsung.c
 *
 *  Created on: april 14, 2024
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "ir_samsung.h"

/* Private define ------------------------------------------------------------*/
#define MESSAGE_BITS        32

/*
 * Az Capture Timebase is: 100us
 * if LEADING_PERIOD_WIDTH_100US is 90 then period with is 9000us
 */
#define LEADING_PERIOD_WIDTH_100US  90 //-> 9ms
#define HIGH_PERIOD_WIDHT_MIN_100US 20 //-> 2ms


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef *_htim;
static uint32_t _channel;

uint32_t _capture_buffer[MESSAGE_BITS];
uint8_t _decoded_buffer[MESSAGE_BITS / 8];
uint32_t _decoded_cnt = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
__weak void IR_SAM_Parser(uint32_t code)
{

}


void IR_SAM_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
  _htim = htim;
  _channel = channel;
  HAL_TIM_IC_Start_IT(_htim, _channel);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  static uint32_t pre_captrue_value;
  static uint8_t bit_pos;
  static uint8_t message_frame_started;

  if(htim->Instance == _htim->Instance)
  {
    uint32_t curr_captrue_value  = __HAL_TIM_GET_COMPARE(htim, _channel);
    uint32_t period_width = curr_captrue_value - pre_captrue_value;
    pre_captrue_value = curr_captrue_value;

    if(LEADING_PERIOD_WIDTH_100US - 5 < period_width && period_width < LEADING_PERIOD_WIDTH_100US + 5)
    {
      __HAL_TIM_SET_COUNTER(htim, 0);
      pre_captrue_value = 0;
      bit_pos = 0;
      message_frame_started = 1;
      return;
    }

    if(message_frame_started)
    {
      _capture_buffer[bit_pos] = period_width;
      bit_pos++;
      if(bit_pos > MESSAGE_BITS - 1)
      {
        bit_pos = 0;
        message_frame_started = 0;

        for (uint8_t bit = 0; bit < MESSAGE_BITS; bit++)
        {
           if (_capture_buffer[bit] > HIGH_PERIOD_WIDHT_MIN_100US)
             _decoded_buffer[bit / 8] |= 1 << (bit % 8);
           else
             _decoded_buffer[bit / 8] &= ~(1 << (bit % 8));
        }

        _decoded_cnt ++;

        uint32_t code = 0;
        code = _decoded_buffer[3];
        code |= _decoded_buffer[2] << 8;
        code |= _decoded_buffer[1] << 16;
        code |= _decoded_buffer[0] << 24;
        IR_SAM_Parser(code);
      }
    }
  }
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
