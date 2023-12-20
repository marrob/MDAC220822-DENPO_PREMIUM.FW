/*
 * user_leds.c
 *
 *  Created on: Dec 18, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define SPI_TIMEOUT 100
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef *_spi;
static uint16_t _ledState;
/* Private function prototypes -----------------------------------------------*/
void UsrLeds_Update(uint16_t value);
/* Private user code ---------------------------------------------------------*/

void UsrLeds_Init(SPI_HandleTypeDef *spi)
{
  _spi = spi;
  _ledState = 0;
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
 // UsrLeds_On(0x0002); //Debug Only
  UsrLeds_Off(0xFFFF);
  DelayUs(1);
}

void UsrLeds_On(uint16_t usr_led)
{
  _ledState |= usr_led;
  UsrLeds_Update(_ledState);
}

void UsrLeds_Off(uint16_t usr_led)
{
  _ledState &= ~usr_led;
  UsrLeds_Update(_ledState);
}

void UsrLeds_Update(uint16_t value)
{
  HAL_SPI_Transmit(_spi, (uint8_t*)&value, sizeof(value), SPI_TIMEOUT);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
  __NOP();
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
}

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
