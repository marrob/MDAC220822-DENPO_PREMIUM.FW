/*
 * mem.c
 *
 *  Created on: Dec 19, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/*--- 24AA025E48 ---*/
#define EEPRPOM_PAGE_SIZE      16
#define EEPROM_SIZE_BYTE       256

#define EEPROM_DEV_TIMEOUT_MS  100

/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef *_hi2c;
static uint8_t _deviceAddress;

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/*
 * Page határt nem lépheted át
 */

void Eeprom_Init(I2C_HandleTypeDef *hi2c, uint8_t deviceAddress)
{
  _hi2c = hi2c;
  _deviceAddress = deviceAddress;

}

HAL_StatusTypeDef Eeprom_ReadU32(uint8_t address, uint32_t *data)
{
  return HAL_I2C_Mem_Read(_hi2c,
                            _deviceAddress,
                            address,
                            I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)data,
                            sizeof(*data),
                            EEPROM_DEV_TIMEOUT_MS);
}


HAL_StatusTypeDef Eeprom_WriteU32(uint8_t address, uint32_t data)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write( _hi2c,
                            _deviceAddress,
                            address,
                            I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)&data,
                            sizeof(data),
                            EEPROM_DEV_TIMEOUT_MS);

  HAL_Delay(5);
  return status;
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
