/*
 * gfx.c
 *
 *  Created on: Nov 27, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "gfx.h"
#include <stdlib.h>
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

uint8_t GfxInit(GfxTypeDef *context, int16_t width, int16_t height)
{
  context->MaxWidth = width;
  context->MaxHeight = height;
  return GFX_OK;
}

void GfxDrawLine(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  /*** Bresenham's Line Drawing Algorithm ***/
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steps, k;

  float xIncrement, yIncrement, x = x0, y = y0;

  if (abs(dx) > abs(dy)) {
     steps = abs(dx);
  } else {
     steps = abs(dy);
  }

  xIncrement = (float)dx / (float)steps;
  yIncrement = (float)dy / (float)steps;

  context->DrawPixel((int)x, (int)y, color);

  for (k = 0; k < steps; k++) {
     x += xIncrement;
     y += yIncrement;
     context->DrawPixel((int)x, (int)y, color);
  }
}

void GfxDrawCircle(GfxTypeDef *context, int16_t centerX, int16_t centerY, int16_t radius, uint16_t color)
{
  /*** Midpoint Circle Drawing Algorithm ***/
  int x = radius;
  int y = 0;
  int radiusError = 1 - x;

  while (x >= y)
  {
    context->DrawPixel(centerX + x, centerY - y, color);
    context->DrawPixel(centerX - x, centerY - y, color);
    context->DrawPixel(centerX + x, centerY + y, color);
    context->DrawPixel(centerX - x, centerY + y, color);
    context->DrawPixel(centerX + y, centerY - x, color);
    context->DrawPixel(centerX - y, centerY - x, color);
    context->DrawPixel(centerX + y, centerY + x, color);
    context->DrawPixel(centerX - y, centerY + x, color);

    y++;

    if (radiusError < 0)
    {
       radiusError += 2 * y + 1;
    } else
    {
       x--;
       radiusError += 2 * (y - x + 1);
    }
  }
}

/*
 * const uint8_t FontTable3x6[];
 * GfxFontTypeDef GfxFont3x6  = {
 *   3, //Width
 *   6, //Height
 *   1, //SpaceWidth
 *   FontTable3x6};
 *
 *
 * - A táblézat egy sora egy karakter
 * - A táblázat oszolpai a karakter szélessége (Width)
 * - Egy bájt értékeének az LSB oldala a karakter tetje (ezt az 5C-s ! jelnél látható)
 * - A karakter magassága csak maximum 8 pixel lehet
 *
 *
 * const unsigned char FontTable3x6[]= {
 * 0x00,0x00,0x00, // Espace 0x20
 * 0x00,0x5C,0x00, // !
 * 0x0C,0x00,0x0C, // "
 * 0x7C,0x28,0x7C, // #
 * 0x7C,0x44,0x7C, // 0x
 *
 * ....
 *
 *
 * // 0x00, 0x06, 0x5F, 0x5F, 0x06, 0x00, 0x00, //'!'
 *
 * FontTable7x8[] -> 7
 *
 * Width: 7
 * Height: 8
 */
void GfxDrawChar(GfxTypeDef *context, const char ch, const GfxFontTypeDef *font, uint16_t color)
{
  int pointer,ch_first_index;
  uint8_t kar = ch-0x20;


  /*** New Line if Requied ***/
  if(context->CursorX > ( context->MaxWidth - 1) - (font->Width) )
  {
    context->CursorX = 0;
    context->CursorY += font->Height;
  }

    uint32_t column_data;
    for(int16_t column_index = 0; column_index < font->Width; column_index++)
    {
      //Megnézem hogy egy oszolop hány bájtos, ez alapján keresem meg a karaktert
      uint8_t column_bytes = font->Height / 8;
      if(font->Height % 8 != 0)
        column_bytes ++;

      //Kijelöli az kiválaszott karakter első bájtját
      ch_first_index = kar * (font->Width) * column_bytes;

      //A Width bájtokon lépked
      pointer = ch_first_index + column_index * column_bytes;

      column_data = 0;

      if(column_bytes == 1)
        column_data = (font->FontTable)[pointer];
      else if(column_bytes == 2)
      {
        column_data |= (font->FontTable)[pointer] << 8;
        column_data |= (font->FontTable)[pointer + 1 ];
      }

      uint32_t mask = 0x01;
      //Egy bájtos bit maszkkal megy végig a karkatren, igy bitenként rajzolja a szélességet
      for(int16_t yPtr = 0; yPtr < font->Height; yPtr++)
      {
          if(column_data & mask)
              context->DrawPixel(context->CursorX + column_index , context->CursorY + yPtr, color);
          mask<<=1;
      }
  }
}

void GfxDrawString(GfxTypeDef *context, const char *string, const GfxFontTypeDef *font, uint16_t color)
{
  uint16_t saved_start_x = context->CursorX;

  while(*string != 0)
  {
    if((*string)=='\n')
    {
      /*
       * Uj sor eseteén hozzá adja az Y-hoz a karakter magasságot.
       * Az X-et vissza állitja a kezdo pozicioba
       */
      context->CursorY += font->Height;
      context->CursorX = saved_start_x;
    }
    else
    {
      GfxDrawChar(context,(*string), font, color);
      context->CursorX += font->Width + 1;
    }
    string++;
  }
}

void GfxDrawFilledRectangle(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  for(int16_t x = x0; x < x1; x++ )
    for(int16_t y =y0; y < y1; y++ )
      context->DrawPixel(x, y, color);
}

