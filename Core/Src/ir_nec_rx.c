/*
 * ir_nec_rx.c
 *
 *  Created on: Dec 29, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "ir_nec_rx.h"
#include <stdio.h>
/* Private define ------------------------------------------------------------*/

#define DEBUG_CAPTURE_LEN 35
#define MESSAGE_BITS   32

/*
 * Az Capture Timebase is: 100us
 * if LEADING_PERIOD_WIDTH_100US is 135 then period with is 13500us
 */
#define LEADING_PERIOD_WIDTH_100US 135
#define HIGH_PERIOD_WIDHT_MIN_100US 20


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef *_htim;
static uint32_t _channel;

/*** For Diag ***/
static uint32_t _errorCnt;
static uint32_t _messageFrameCnt;

uint32_t _capture_buffer[MESSAGE_BITS];
uint8_t _decoded_buffer[MESSAGE_BITS / 8];

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void IR_NEC_Init(TIM_HandleTypeDef *htim, uint32_t channel)
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
      bit_pos ++;

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


        uint8_t address = _decoded_buffer[0];

        /*
        uint8_t invers_address =  _decoded_buffer[1];
        */

        //AIR DOG Remote Power On: 0x02 0xBD 0x45 0xBA
        //printf("%02X %02X %02X %02X\r\n", _decoded_buffer[0], _decoded_buffer[1], _decoded_buffer[2], _decoded_buffer[3]);

        /*
        if(address != (uint8_t)~invers_address)
        {
          _errorCnt++;
          return;
        }
        */

        uint8_t command = _decoded_buffer[2];
        uint8_t invers_command =  _decoded_buffer[3];
        if(command != (uint8_t)~invers_command)
        {
          _errorCnt ++;
          return;
        }

        /*** decode completed ***/
        _messageFrameCnt ++;
        IR_NEC_Parser(address, command);
      }
    }
  }
}

/*
__weak void IR_NEC_Parser (uint8_t address, uint8_t command)
{

}
*/
uint32_t IR_NEC_GetErrorCnt(void)
{
  return _errorCnt;
}

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
