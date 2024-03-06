/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "vt100.h"

#include "pcm9211.h"
#include "bd34301.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum _Xtatus_t{
  XMOS_UNKNOWN = 0xFF,
  XMOS_PCM_44_1KHZ = 0x17,
  XMOS_PCM_48_0KHZ = 0x16,
  XMOS_PCM_88_2KHZ = 0x15,
  XMOS_PCM_96_0KHZ = 0x14,
  XMOS_PCM_176_4KHZ = 0x13,
  XMOS_PCM_192_KHZ = 0x12,
  XMOS_PCM_352_8KHZ = 0x11,
  XMOS_PCM_384_KHZ = 0x10,
  XMOS_DSD_64 = 0x02,
  XMOS_DSD_128 = 0x00,
  XMOS_DSD_256 = 0x04
}XmosStatus_t;

typedef enum _DacAudioFormat_t{
  DAC_PCM_32_0KHZ = 0,
  DAC_PCM_44_1KHZ,  //1
  DAC_PCM_48_0KHZ,  //2
  DAC_PCM_88_2KHZ,  //3
  DAC_PCM_96_0KHZ,  //4
  DAC_PCM_176_4KHZ, //5
  DAC_PCM_192_KHZ,  //6
  DAC_PCM_352_8KHZ, //7
  DAC_PCM_384_0KHZ, //8
  DAC_PCM_705_6KHZ, //9
  DAC_PCM_768_0KHZ, //10
  DAC_DSD_64,
  DAC_DSD_128,
  DAC_DSD_256,
  DAC_DSD_512,
}DacAudioFormat_t;

typedef enum _Route_t
{
  ROUTE_USB,
  ROUTE_I2S_HDMI,
  ROUTE_TOS,
  ROUTE_RCA,
  ROUTE_BNC,
  ROUTE_AES_XLR,
}Route_t;

typedef enum _MasterClock_t
{
  CLK_22_5792MHZ,
  CLK_24_575MHZ
}MasterClocks_t;


/*
 * FIGYELEM!
 * AudioTypes_t és AudioFreqArray-nak fedniük kell egymást!
 */
typedef enum _AudioTypes
{
  AUDIO_PCM_32_0KHZ,    //0
  AUDIO_PCM_44_1KHZ,    //1
  AUDIO_PCM_48_0KHZ,    //2
  AUDIO_PCM_88_2KHZ,    //3
  AUDIO_PCM_96_0KHZ,    //4
  AUDIO_PCM_176_4KHZ,   //5
  AUDIO_PCM_192_KHZ,    //6
  AUDIO_PCM_352_8KHZ,   //7
  AUDIO_PCM_384_0KHZ,
  AUDIO_PCM_705_6KHZ,
  AUDIO_DSD_64,
  AUDIO_DSD_128,
  AUDIO_DSD_256,
  AUDIO_DSD_512,
  AUDIO_UNKNOWN,
}AudioTypes_t;

typedef enum _DebugState_Type
{
  SDBG_IDLE,             // 00
  SDBG_MAKE_HARDFAULT,   // 01
  SDBG_HARD_RESET,       // 02
  SDBG_DAC_MUTE_ON,      // 03
  SDBG_DAC_MUTE_OFF,     // 04
  SDBG_DAC_RECONFIG,     // 05
  SDBG_LAST
}DebugState_t;


typedef struct _AppTypeDef
{
  struct _Meas
  {
    __IO uint32_t FreqLRCK_MHz;
    __IO uint32_t FreqBCLK_MHz;
  }Meas;

  struct _AudiType
  {
    AudioTypes_t Pre;
    AudioTypes_t Curr;
  }AudioType;

  MasterClocks_t MasterClock;

  struct _XStatus
  {
    XmosStatus_t Pre;
    XmosStatus_t Curr;
  }XmosStatus;

  struct _Route
  {
    Route_t Pre;
    Route_t Curr;
  }Route;

  struct _Volume
  {
    uint32_t Pre;
    uint32_t Curr;
  }Volume;

  struct _IntExt
  {
    uint8_t Pre;
    uint8_t Curr;
  }IntExt;

  struct _Lock
  {
    uint8_t Pre;
    uint8_t Curr;
  }Lock;


  uint8_t UserIsMute;

  uint8_t RemoteCommand;

  DacAudioFormat_t DacAudioFormat;

  uint32_t UpTimeSec;
  uint8_t IsOn;
  uint8_t Buttons;

  uint32_t IsSleep;
  uint32_t WakeStartTimestamp;

  struct _Diag
  {
    uint8_t  WakeUpFromWdtReset;
    uint32_t RS485ResponseCnt;
    uint32_t RS485RequestCnt;
    uint32_t RS485UnknwonCnt;
    uint32_t RS485NotMyCmdCnt;
    uint32_t UartErrorCnt;
    uint32_t UartDmaDataEmptyErrorCallbackCnt;
    uint32_t AuidoTypeCorrectionCnt;
    uint32_t XmosFormatUnknownCnt;
    uint32_t XmosStatusChangedCnt;
    uint8_t XmosMuteSignaledCnt;
    uint8_t DacReConfgiurationCnt;
    uint32_t SpdifAuidoTypeChangedCnt;
    PCM9211_Frequencies_t PCM9211SamplingFreq;
    uint32_t BootupCnt;
  }Diag;

  DebugState_t DebugState;
}Device_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define DEVICE_NAME             "MDAC220822-DENPO PREMIUM"
#define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
#define DEVICE_FW               "240109_1700"
#define DEVICE_FW_SIZE          sizeof(DEVICDSD_PCME_FW)
#define DEVICE_PCB              "00"
#define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
#define DEVICE_MNF              "KONVOLUCIO"
#define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)

/*--- PCM->I2S PCM9211 ---*/
#define PCM9211_DEVICE_ADDRESS  0x86

/*--- DAC -> BD34 ---*/
#define BD34_DEVICE_ADDRESS     0x38

/*--- EEPROM -> 24AA025E48 ---*/
#define EEPROM_DEVICE_ADDRESS   0xA2

#define DEVICE_GO_SLEEP_SEC  59000

/*-- COM ---*/
#define RS485_TX_HOLD_MS      1
#define RS485_CMD_LENGTH      35
#define RS485_ARG1_LENGTH     35
#define RS485_ARG2_LENGTH     35

#define UART_BUFFER_SIZE       128
#define UART_DMA_BUFFER_SIZE   128
#define UART_TERIMINATION_CHAR  '\r' //0x0D

#define CLIENT_TX_ADDR        0x30
#define CLIENT_RX_ADDR        0x03

/*--- User Leds ---*/
#define USR_LED_POWER     0x0002
#define USR_LED_LOCK      0x0004
#define USR_LED_EXTREF    0x0008
#define USR_LED_AES       0x0200
#define USR_LED_BNC       0x0400
#define USR_LED_RCA       0x0800
#define USR_LED_TOS       0x1000
#define USR_LED_I2S       0x2000
#define USR_LED_USB       0x4000
//Az új MUTE jelző piros LED bekötésével. Az U800 (74HCT595D) Q7 (PIN7) kimenetét fogjuk erre használni.
#define USR_LED_MUTE      0x8000


/*--- EEPROM MAP ---*/
#define EEPROM_ADDR_FIRST_START  0x0000
#define EEPROM_ADDR_BOOTUP_CNT   0x0004
#define EEPROM_ADDR_LAST_ROUTE   0x0008

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/*--- User Leds ---*/
void UsrLeds_Init(SPI_HandleTypeDef *spi);
void UsrLeds_On(uint16_t usr_led);
void UsrLeds_Off(uint16_t usr_led);

/*--- Communication ---*/
void Com_Init(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma);
void Com_Task(void);

void SetMasterClock(MasterClocks_t clk );

/*--- EEPROM ---*/
void Eeprom_Init(I2C_HandleTypeDef *hi2c, uint8_t deviceAddress);
HAL_StatusTypeDef Eeprom_ReadU32(uint8_t address, uint32_t *data);
HAL_StatusTypeDef Eeprom_WriteU32(uint8_t address, uint32_t data);


/* --- FreqMeas ---*/
void FreqMeas_Start(void);
AudioTypes_t FreqMeas_GetAudioType(void);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A0_USB_ISO_Pin GPIO_PIN_13
#define A0_USB_ISO_GPIO_Port GPIOC
#define LIVE_LED_Pin GPIO_PIN_14
#define LIVE_LED_GPIO_Port GPIOC
#define DAC_MUTE_COM_Pin GPIO_PIN_15
#define DAC_MUTE_COM_GPIO_Port GPIOC
#define FREQ_MEAS_LRCK_Pin GPIO_PIN_0
#define FREQ_MEAS_LRCK_GPIO_Port GPIOA
#define DAC_RESETB_Pin GPIO_PIN_1
#define DAC_RESETB_GPIO_Port GPIOA
#define EN_USB_ISO_Pin GPIO_PIN_2
#define EN_USB_ISO_GPIO_Port GPIOA
#define H5_1_ISO_Pin GPIO_PIN_3
#define H5_1_ISO_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define USART1_DIR_Pin GPIO_PIN_6
#define USART1_DIR_GPIO_Port GPIOA
#define H5_3_ISO_Pin GPIO_PIN_0
#define H5_3_ISO_GPIO_Port GPIOB
#define EN_I2S_ISO_Pin GPIO_PIN_1
#define EN_I2S_ISO_GPIO_Port GPIOB
#define RESET_SPD_ISO_Pin GPIO_PIN_2
#define RESET_SPD_ISO_GPIO_Port GPIOB
#define LOCK_PLL_Pin GPIO_PIN_10
#define LOCK_PLL_GPIO_Port GPIOB
#define INT_EXT_PLL_Pin GPIO_PIN_11
#define INT_EXT_PLL_GPIO_Port GPIOB
#define PWR_CTRL_Pin GPIO_PIN_12
#define PWR_CTRL_GPIO_Port GPIOB
#define BTN_PWR_DB_Pin GPIO_PIN_13
#define BTN_PWR_DB_GPIO_Port GPIOB
#define BTN_SEL_DB_Pin GPIO_PIN_14
#define BTN_SEL_DB_GPIO_Port GPIOB
#define RECLKBYPS_Pin GPIO_PIN_15
#define RECLKBYPS_GPIO_Port GPIOB
#define A1_USB_ISO_Pin GPIO_PIN_8
#define A1_USB_ISO_GPIO_Port GPIOA
#define A2_USB_ISO_Pin GPIO_PIN_11
#define A2_USB_ISO_GPIO_Port GPIOA
#define FREQ_MEAS_BCLK_Pin GPIO_PIN_12
#define FREQ_MEAS_BCLK_GPIO_Port GPIOA
#define EN_SPDIF_ISO_Pin GPIO_PIN_15
#define EN_SPDIF_ISO_GPIO_Port GPIOA
#define MUTE_USB_ISO_Pin GPIO_PIN_4
#define MUTE_USB_ISO_GPIO_Port GPIOB
#define DSD_PCM_USB_ISO_Pin GPIO_PIN_5
#define DSD_PCM_USB_ISO_GPIO_Port GPIOB
#define MCLK_SEL_ISO_Pin GPIO_PIN_8
#define MCLK_SEL_ISO_GPIO_Port GPIOB
#define IR_RX_IN_Pin GPIO_PIN_9
#define IR_RX_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
