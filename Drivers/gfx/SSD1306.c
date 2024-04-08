/*
 * ssd1306.c
 *
 *  Created on: Nov 27, 2023
 *      Author: marrob
 */
/* Includes ------------------------------------------------------------------*/
#include <SSD1306.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t _frameBuffer[SSD1306_WIDTH * (SSD1306_HEIGHT / 8) + 1]; // + 1 byte for 0x40 Co = 0, D/C = 0
static I2C_HandleTypeDef *_i2c;
static uint8_t  _devAddress;

/* Private function prototypes -----------------------------------------------*/
static inline void Write(uint8_t* wdata, size_t wlength);
static inline void WriteCommand(uint8_t cmd);
static inline void WriteCommands(uint8_t *commands, size_t count);


/* Private user code ---------------------------------------------------------*/
uint8_t SSD1306_Init(I2C_HandleTypeDef *i2c, uint8_t address)
{
  _i2c=i2c;
  _devAddress = address;
  _frameBuffer[0x00] = 0x40; // Co = 0, D/C = 0

  WriteCommand(0xAE);//Display Off

  WriteCommand(0xD5);//SET DISPLAY CLOCK
  WriteCommand(0x80);//105Hz


  WriteCommand(0xA8);//Select Multiplex Ratio
  WriteCommand(0x1F);//Default => 0x3F (1/64 Duty) 0x1F(1/32 Duty)


  WriteCommand(0xD3);//Setting Display Offset
  WriteCommand(0x00);//00H Reset

  WriteCommand(0x40);//Set Display Start Line

  WriteCommand(0x8D);//Set Charge Pump
  WriteCommand(0x14);//Enable Charge Pump

  WriteCommand(0x20); //Set Memory Addressing Mode
  WriteCommand(0xA0); //Horizontal addressing mode

  WriteCommand(0x21); //Set Column Address
  WriteCommand(0x00); //Start: 0
  WriteCommand(0x7F); //End: 127

  WriteCommand(0x22); //Set Page Address
  WriteCommand(0x00); //Start: 0
  WriteCommand(0x04); //End: 4

  WriteCommand(0xA1);//Set Segment Re-Map Default

  WriteCommand(0xC8);//Set COM Output Scan Direction

  WriteCommand(0xDA);//Set COM Hardware Configuration
  WriteCommand(0x02);//Alternative COM Pin---See IC Spec page 34

  WriteCommand(0x81);//Set Contrast Control
  WriteCommand(0x8F);

  WriteCommand(0xD9);//Set Pre-Charge period
  WriteCommand(0x22);

  WriteCommand(0xDB);//Set Pre-Charge period
  WriteCommand(0x40);

  WriteCommand(0xA4);//Entire Display ON

  WriteCommand(0xA6);//Set Normal Display

  WriteCommand(0xAF);//Display ON

  return SSD1306_OK;
}

/*
 * x:0...127
 * y:0...31 
 */
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  switch (color)
  {
    case SSD1306_WHITE:
    {
      //+1 byte for D/C byte
      _frameBuffer[x + (y / 8) * SSD1306_WIDTH + 1] |= (1 << (y & 7));
      break;
    }
    case SSD1306_BLACK:
    {
      _frameBuffer[x + (y / 8) * SSD1306_WIDTH + 1] &= ~(1 << (y & 7));
      break;
    }
  }
}

void SSD1306_Off(void)
{
  WriteCommand(0xAE);//Display Off
  WriteCommand(0x8D);//Set Charge Pump
  WriteCommand(0x10);//Disable Charge Pump
}

void SSD1306_Update(void)
{
  uint8_t cmds[]=
  {
      0x21,  //Set Column Address
      0x00,  //Start: 0
      0x7F,  //End:127
      0x22,  //Set Page Address
      0x00,  //Start: 0
      0x04,  //End: 4
  };
  WriteCommands(cmds, sizeof(cmds));
  uint16_t pixels = SSD1306_WIDTH * (SSD1306_HEIGHT / 8);
  Write(_frameBuffer, pixels + 1);
}

void SSD1306_Clear()
{
  memset(_frameBuffer + 1, 0, SSD1306_WIDTH * (SSD1306_HEIGHT / 8));
}

static inline void WriteCommand(uint8_t cmd)
{
  uint8_t buf[2] = {0x00, cmd }; // Co = 0, D/C = 0
  Write(buf, sizeof(buf));
}

static inline void WriteCommands(uint8_t *commands, size_t count)
{
  uint8_t *buffer = (uint8_t*)malloc(count + 1);
  if(buffer != NULL)
  {
    buffer[0] = 0x00; /* D/C byte -> Command */
    memcpy(buffer + 1, commands, count);
    Write(buffer, count + 1);
    free(buffer);
  }
}

static inline void Write(uint8_t* wdata, size_t wlength)
{
  HAL_I2C_Master_Transmit(_i2c, _devAddress, wdata, wlength, 100);
}

/* *****************************************************************************
 End of File
 */
