/*
 * display.c
 *
 *  Created on: Nov 27, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "display.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static GfxTypeDef _gfx;

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

uint8_t DisplayInit(I2C_HandleTypeDef *i2c, uint8_t address)
{
  SSD1306_Init(i2c, address);
  _gfx.DrawPixel = &SSD1306_DrawPixel;
  _gfx.MaxWidth = SSD1306_WIDTH;
  _gfx.MaxHeight = SSD1306_HEIGHT;
  _gfx.TextColor = DISP_COLOR_WHITE;
  _gfx.TextBackgroundColor = DISP_COLOR_BLACK;
  return DISPLAY_OK;
}

void DisplayClear(void)
{
  SSD1306_Clear();
}

void DisplayUpdate(void)
{
  SSD1306_Update();
}

/*
 * x: 0, XXX_WIDTH - 1
 * y: 0, XXX_HEIGHT - 1
 */
void DisplaySetCursor(uint16_t x, uint16_t y)
{
  _gfx.CursorX = x;
  _gfx.CursorY = y;
}

void DisplayDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  SSD1306_DrawPixel(x, y, color);
}

void DisplayDrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
  GfxDrawLine(&_gfx, x0, y0, x1, y1, color);
}

void DisplayDrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
  GfxDrawCircle(&_gfx, x0, y0, r, color);
}

void DisplayDrawChar(const char ch, const GfxFontTypeDef *font, uint16_t color)
{
  GfxDrawChar(&_gfx, ch, font, color);
}

void DisplayDrawString(const char *string, const GfxFontTypeDef *font, uint16_t color)
{
  GfxDrawString(&_gfx, string, font, color);
}

void DisplayDrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
  GfxDrawFilledRectangle(&_gfx, x0, y0, x1, y1, color);
}





/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
