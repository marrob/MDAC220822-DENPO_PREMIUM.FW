/*
 * display.h
 *
 *  Created on: Nov 27, 2023
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ILI932X_DISPLAY_H_
#define ILI932X_DISPLAY_H_
/* Includes ------------------------------------------------------------------*/
#include "SSD1306.h"
#include "gfx.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define DISPLAY_OK  0
/* Exported functions ------------------------------------------------------- */

#define DISP_COLOR_BLACK    SSD1306_BLACK
#define DISP_COLOR_WHITE    SSD1306_WHITE

uint8_t DisplayInit(I2C_HandleTypeDef *i2c, uint8_t address);
void DisplayClear(void);
void DisplaySetCursor(uint16_t x, uint16_t y);
void DisplayDrawPixel(uint16_t x, uint16_t y, uint16_t color);
void DisplayDrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void DisplayDrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void DisplayDrawChar(const char ch, const GfxFontTypeDef *font, uint16_t color);
void DisplayDrawString(const char *string, const GfxFontTypeDef *font, uint16_t color);
void DisplayDrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void DisplayUpdate(void);

#endif /* ILI932X_DISPLAY_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
