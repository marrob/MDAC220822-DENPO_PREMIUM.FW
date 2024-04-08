/*
 * ssd1306.h
 *
 *  Created on: Nov 27, 2023
 *      Author: marrob
 */

#ifndef _SSD1306_H    /* Guard against multiple inclusion */
#define _SSD1306_H

#include "main.h" //-> stm32fxxx_hal.h
#include <stdlib.h>
#include <string.h>
#include "gfx.h"

#define SSD1306_WHITE           0
#define SSD1306_BLACK           1

#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          32
#define SSD1306_BPP             1   //Bits per pixel color

#define SSD1306_OK              0
#define SSD1306_ERROR           1

#define SSD1306_I2C_DEV_ADDRESS 0x78

uint8_t SSD1306_Init(I2C_HandleTypeDef *i2c, uint8_t address);
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void SSD1306_Off(void);
void SSD1306_Update(void);
void SSD1306_Clear(void);


#endif /* _SSD1306_H */

/* *****************************************************************************
 End of File
 */
