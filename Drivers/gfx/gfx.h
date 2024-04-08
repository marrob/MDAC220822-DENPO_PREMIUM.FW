/*
 * gfx.h
 *
 *  Created on: Nov 27, 2023
 *      Author: marrob
 */

#ifndef _GFX_H
#define _GFX_H

#include <stdint.h>
#include <stddef.h>

#define GFX_OK 0

  typedef struct _GfxTypeDef
  {
    uint16_t MaxWidth;
    uint16_t MaxHeight;
    uint16_t CursorX;
    uint16_t CursorY;
    uint16_t TextColor;
    uint16_t TextBackgroundColor;

    void (*DrawPixel)(uint16_t x, uint16_t, uint16_t color);
  }GfxTypeDef;

  typedef struct _GfxFontTypeDef
  {
    uint8_t Width;              /* Character width for storage         */
    uint8_t Height;             /* Character height for storage        */
    uint8_t SpaceWidth;
    const uint8_t *FontTable;       /* Font table start address in memory  */
  } GfxFontTypeDef;

  typedef struct _GfxBitmapTypeDef
  {
    struct _Size
    {
        uint16_t X;
        uint16_t Y;
    }Size;
    uint8_t Data[];
  }GfxBitmapTypeDef;

  extern GfxFontTypeDef GfxFont3x6;
  extern GfxFontTypeDef GfxFont5x8;
  extern GfxFontTypeDef GfxFont7x8;
  extern GfxFontTypeDef GfxFont8x12;

  uint8_t GfxInit(GfxTypeDef *context,  int16_t w, int16_t h);
  void GfxDrawLine(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  void GfxDrawCircle(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void GfxDrawChar(GfxTypeDef *context, const char ch, const GfxFontTypeDef *font, uint16_t color);
  void GfxDrawString(GfxTypeDef *context, const char *string, const GfxFontTypeDef *font, uint16_t color);
  void GfxDrawFilledRectangle(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);


#endif /* _GFX_H */

/* *****************************************************************************
 End of File
 */
