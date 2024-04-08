/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  *231219
  * User Manual:
  *
  * - A bemenetválasztást a felhasználó választja a SELECTOR nyomógombbal.
  * - A DAC paramétereit forrás frekvenciájának mérésével LRCK/BLCK határozza meg, kivéve az USB forrást.
  *
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "display.h"
#include "ir_nec_rx.h"
#include "LiveLed.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*** DIO ***/
#define DO_EN_I2S_I2C_ISO         ((uint8_t)1<<1)
#define DO_EN_USB_ISO             ((uint8_t)1<<2)
#define DO_EN_SPDIF_ISO           ((uint8_t)1<<3)
#define DO_MUX_PCM                ((uint8_t)1<<4)

#define DI_A0_USB                 ((uint8_t)1<<0)
#define DI_A1_USB                 ((uint8_t)1<<1)
#define DI_A2_USB                 ((uint8_t)1<<2)
#define DI_A3_USB                 ((uint8_t)1<<3)
#define DI_DSD_PCM_USB            ((uint8_t)1<<4)
#define DI_XMOS_MUTE              ((uint8_t)1<<5)


/*** BUTTONS ***/
#define BUTTON_POWER      ((uint8_t)1<<1)
#define BUTTON_SELECTOR   ((uint8_t)1<<2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

Device_t Device;
LiveLED_HnadleTypeDef hLiveLed;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/*** LiveLed ***/
void LiveLedOff(void);
void LiveLedOn(void);


void SetMasterClock(MasterClocks_t clk);

/*** XMOS/USB ***/
XmosStatus_t ReadXmosStaus(void);
uint8_t XmosIsMute(void);
void SetRoute (Route_t route);

/*** Tasks ***/
void UpTimeTask(void);
void DebugTask(DebugState_t dbg);

/*** Mute ***/
void UserMuteOn(void);
void UserMuteOff(void);
uint8_t UserIsMute(void);
void DeviceMuteOff(void);
void DeviceMuteOn(void);

/*** HMI ***/
uint8_t ButtonsPoll(void);
void UpdateLockIntExtStatus(void);
void UpdateSelectorLeds(Route_t route);
void DevicePowerOn(void);
void DevicePowerOff(void);
void DeviceSleep(void);
void DisplayWake(void);
uint8_t IsDisplaySleep(void);
void DisplaySleepTask(void);
uint8_t DeviceIsOn(void);
void ButtonsTask(void);
void RemoteTask(void);

/*** ReClock ***/
void ReClockBypassOff(void);
void ReClockBypassOn(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /*** Check if the system has resumed from WWDG reset ***/
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    Device.Diag.WakeUpFromWdtReset = 1;
  }

  /*** Reset ***/
  BD34301_Reset();

  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);

#if debug
  /*--- Defualt ---*/
  Device.Route.Pre = ROUTE_USB;
  Device.Route.Curr = ROUTE_USB;
  Device.DacAudioFormat = DAC_PCM_44_1KHZ;
  Device.MasterClock = CLK_22_5792MHZ;
  Device.XmosStatus.Pre = XMOS_UNKNOWN;
  Device.Volume.Curr = 100;

  HAL_GPIO_WritePin(EN_I2S_ISO_GPIO_Port, EN_I2S_ISO_Pin, GPIO_PIN_SET); //ON -> Deselect I2S-HDMI        //BCLK ->R529, EN_I2S_IS ->R583
  HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_SET); // ON -> Select USB Input        //BCLK ->R521, EN_USB_ISO ->R579
  HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_RESET); // OFF -> Deselect SPDIF   //BCLK ->R531, EN_SPDIF_ISO ->R591


  /* --- Minden ki ---*/
  HAL_GPIO_WritePin(EN_I2S_ISO_GPIO_Port, EN_I2S_ISO_Pin, GPIO_PIN_SET); //ON -> Deselect I2S-HDMI
  HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); // OFF -> Deselect USB Input
  HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_RESET); // OFF -> Deselect SPDIF

#endif

  /*--- User Leds ---*/
  UsrLeds_Init(&hspi1);

  /*--- EEPROM ---*/
  Eeprom_Init(&hi2c1, EEPROM_DEVICE_ADDRESS);

  uint32_t startSign;
  Eeprom_ReadU32(EEPROM_ADDR_FIRST_START, &startSign);
  if(startSign != 0x55AA)
  {
    /*--- FIRST START ---*/
    Eeprom_WriteU32(EEPROM_ADDR_FIRST_START, 0x55AA);
    Eeprom_WriteU32(EEPROM_ADDR_BOOTUP_CNT, 0);
    Eeprom_WriteU32(EEPROM_ADDR_LAST_ROUTE, ROUTE_USB);
  }

  /*--- BOOTUP COUNTER ---*/
  Eeprom_ReadU32(EEPROM_ADDR_BOOTUP_CNT, &Device.Diag.BootupCnt);
  Device.Diag.BootupCnt++;
  Eeprom_WriteU32(EEPROM_ADDR_BOOTUP_CNT, Device.Diag.BootupCnt);

  /*--- Display ---*/
  DisplayInit(&hi2c1, SSD1306_I2C_DEV_ADDRESS);
  DisplayClear();
  DisplayUpdate();
  DisplaySetCursor(0, 4);
  DisplayDrawString("Hello World", &GfxFont7x8, SSD1306_WHITE );
  DisplayUpdate();

  /*--- FrMeter ---*/
  FreqMeas_Start();
  PCM9211_Init(&hi2c1, PCM9211_DEVICE_ADDRESS);
  BD34301_Init(&hi2c1, BD34_DEVICE_ADDRESS);

  /*--- Device ---*/
  DevicePowerOff();

  /*--- Communication ---*/
  Com_Init(&huart1, &hdma_usart1_rx);

  /*--- Infrared Receiver ---*/
  IR_NEC_Init(&htim4, TIM_CHANNEL_4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    //HAL_WWDG_Refresh(&hwwdg);

    // 22.5792MHz - 24.575MHz
    //         LRCK
    // 352.8KHz -   384KHz    |
    // 176.4KHz -   192KHz    |
    // 88.2KHz  -   96KHz     |
    // 44.1KHz  -   48.0KHz   |   64xLRCK

    /*
     * - Az XLR, HDMI, RCA-ról érkező jelek vezérlik a DAC-ot
     * - A LRCK-BCK mért frekvenciák alapján meghatározom audió jel tipusát pl: AUDIO_PCM_44_1KHZ
     * - Ha pl AUDIO_PCM_44_1KHZ - jön akkor 22.5792MHz a Master Clock kell neki
     * - Ha DSD jön az nem mehet keresztül a SRC, a routot kell változtatni (HDMI vagy XMOS)-n úgy hogy ne menejen keresztül rajta
     * - Ha nem USB-röl jön a jel akkor kiválasszam az OP
     **/

    static uint8_t flag;
    static uint32_t timestamp;

    if(Device.IsOn)
    {
      Device.AudioType.Curr = FreqMeas_GetAudioType();
      if( Device.Route.Curr == ROUTE_I2S_HDMI ||
          Device.Route.Curr == ROUTE_BNC ||
          Device.Route.Curr == ROUTE_RCA ||
          Device.Route.Curr == ROUTE_AES_XLR ||
          Device.Route.Curr == ROUTE_TOS)
      {
        if(Device.AudioType.Pre != Device.AudioType.Curr)
        {
          if(flag == 0)
          {
            flag = 1;
            DeviceMuteOn();
            BD34301_MuteOn();
            Device.Diag.SpdifAuidoTypeChangedCnt++;
            timestamp = HAL_GetTick();
            Device.AudioType.Pre = AUDIO_UNKNOWN; //ez kikényszerití a némítás-visszakapcsolást, abban az esetben is ha pattanás/rövid hiba törétn a stream-ben
          }
          if(flag == 1)
          {
            if(HAL_GetTick() - timestamp > 500)
            {
              flag = 0;

              switch(Device.AudioType.Curr)
              {
                 case AUDIO_PCM_32_0KHZ:{
                   Device.DacAudioFormat = DAC_PCM_32_0KHZ;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
                 }
                 case AUDIO_PCM_44_1KHZ:{
                   Device.DacAudioFormat = DAC_PCM_44_1KHZ;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
                 }
                 case AUDIO_PCM_48_0KHZ:{
                   Device.DacAudioFormat = DAC_PCM_48_0KHZ;
                   Device.MasterClock = CLK_24_575MHZ;
                   break;
                 }
                 case AUDIO_PCM_88_2KHZ:{
                   Device.DacAudioFormat = DAC_PCM_88_2KHZ;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
                 }
                 case AUDIO_PCM_96_0KHZ:{
                   Device.DacAudioFormat = DAC_PCM_96_0KHZ;
                   Device.MasterClock = CLK_24_575MHZ;
                   break;
                 }
                 case AUDIO_PCM_176_4KHZ:{
                   Device.DacAudioFormat = DAC_PCM_176_4KHZ;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
                 }
                 case AUDIO_PCM_192_KHZ:{
                   Device.DacAudioFormat = DAC_PCM_192_KHZ;
                   Device.MasterClock = CLK_24_575MHZ;
                   break;
                 }
                 case AUDIO_PCM_352_8KHZ:{
                     Device.DacAudioFormat = DAC_PCM_352_8KHZ;
                     Device.MasterClock = CLK_22_5792MHZ;
                     break;
                 }
                 case AUDIO_PCM_384_0KHZ:{
                     Device.DacAudioFormat = DAC_PCM_384_0KHZ;
                     Device.MasterClock = CLK_24_575MHZ;
                   break;
                 }
                 case AUDIO_PCM_705_6KHZ:{
                     Device.DacAudioFormat = DAC_PCM_705_6KHZ;
                     Device.MasterClock = CLK_24_575MHZ;
                     break;
                 }
                 case AUDIO_DSD_64:{
                     Device.DacAudioFormat = DAC_DSD_64;
                     Device.MasterClock = CLK_22_5792MHZ;
                     break;
                   }
                 case AUDIO_DSD_128:{
                   Device.DacAudioFormat = DAC_DSD_128;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
                 }
                 case AUDIO_DSD_256:{
                   Device.DacAudioFormat = DAC_DSD_256;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
                 }
                 case AUDIO_DSD_512:
                 {
                   Device.DacAudioFormat = DAC_DSD_512;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
                 }
                 case AUDIO_UNKNOWN: {
                   break;
                 }
              }

              BD34301_DigitalPowerOff();
              BD34301_SoftwareResetOn();

              Device.Diag.DacReConfgiurationCnt++;
              SetMasterClock(Device.MasterClock);
              DelayMs(15); //Kritikus pl 88.2 és 96 váltás között
              BD34301_ModeSwitching(&BD34301_ModeList[Device.DacAudioFormat]);

              BD34301_SoftwareResetOff();
              BD34301_DigitalPowerOn();
              BD34301_RamClear(); //Kritkus, nem szól a PCM ha nincs
              DeviceMuteOff();
              BD34301_MuteOff();
              Device.AudioType.Pre = Device.AudioType.Curr;
            }
          }
        }
      }

      /*
       * Az USB XMOS statuszta mondja meg mit csináljon a DAC
       */
      Device.XmosStatus.Curr = ReadXmosStaus();
      if(Device.Route.Curr == ROUTE_USB)
      {
        if(Device.XmosStatus.Pre != Device.XmosStatus.Curr)
        {
          if(flag == 0)
          {
            flag = 1;
            DeviceMuteOn();
            BD34301_MuteOn();
            Device.Diag.XmosStatusChangedCnt++;
            timestamp = HAL_GetTick();
            Device.AudioType.Pre = XMOS_UNKNOWN; //ez kikényszerití a némítás-visszakapcsolást, abban az esetben is ha pattanás/rövid hiba törétn a stream-ben
          }

          if(flag == 1)
          {
            if(HAL_GetTick() - timestamp > 500)
            {
              flag = 0;
              switch(Device.XmosStatus.Curr)
              {
                case XMOS_PCM_44_1KHZ:{
                    Device.DacAudioFormat = DAC_PCM_44_1KHZ;
                    Device.MasterClock = CLK_22_5792MHZ;
                    break;
                  }
                case XMOS_PCM_48_0KHZ:{
                  Device.DacAudioFormat = DAC_PCM_48_0KHZ;
                  Device.MasterClock = CLK_24_575MHZ;
                  break;
                }
                case XMOS_PCM_88_2KHZ:{
                  Device.DacAudioFormat = DAC_PCM_88_2KHZ;
                  Device.MasterClock = CLK_22_5792MHZ;
                  break;
                }
                case XMOS_PCM_96_0KHZ:{
                  Device.DacAudioFormat = DAC_PCM_96_0KHZ;
                  Device.MasterClock = CLK_24_575MHZ;
                  break;
                }
                case XMOS_PCM_176_4KHZ:{
                  Device.DacAudioFormat = DAC_PCM_176_4KHZ;
                  Device.MasterClock = CLK_22_5792MHZ;
                  break;
                }
                case XMOS_PCM_192_KHZ:{
                  Device.DacAudioFormat = DAC_PCM_192_KHZ;
                  Device.MasterClock = CLK_24_575MHZ;
                  break;
                }
                case XMOS_PCM_352_8KHZ:{
                  Device.DacAudioFormat = DAC_PCM_352_8KHZ;
                  Device.MasterClock = CLK_22_5792MHZ;
                  break;
                }
                case XMOS_PCM_384_KHZ:{
                  Device.DacAudioFormat = DAC_PCM_384_0KHZ;
                  Device.MasterClock = CLK_24_575MHZ;
                  break;
                }
                case XMOS_DSD_64:{
                  Device.DacAudioFormat = DAC_DSD_64;
                  Device.MasterClock = CLK_22_5792MHZ;
                  break;
                }
                case XMOS_DSD_128:{
                  Device.DacAudioFormat = DAC_DSD_128;
                  Device.MasterClock = CLK_22_5792MHZ;
                  break;
                }
                case XMOS_DSD_256:{
                  Device.DacAudioFormat = DAC_DSD_256;
                  Device.MasterClock = CLK_22_5792MHZ;
                  break;
                }
                default:
                {
                  Device.Diag.XmosFormatUnknownCnt++;
                };
              }

              BD34301_DigitalPowerOff();
              BD34301_SoftwareResetOn();

              Device.Diag.DacReConfgiurationCnt++;
              SetMasterClock(Device.MasterClock);
              DelayMs(15); //Kritikus pl 88.2 és 96 váltás között
              BD34301_ModeSwitching(&BD34301_ModeList[Device.DacAudioFormat]);

              BD34301_SoftwareResetOff();
              BD34301_DigitalPowerOn();
              BD34301_RamClear(); //Kritkus, nem szól a PCM ha nincs
              DeviceMuteOff();
              BD34301_MuteOff();
              Device.XmosStatus.Pre = Device.XmosStatus.Curr;
            }
          }
        }
      }

      if(Device.Route.Pre != Device.Route.Curr){
        SetRoute(Device.Route.Curr);
        Device.AudioType.Pre = AUDIO_UNKNOWN;
        Device.XmosStatus.Pre = XMOS_UNKNOWN;
        Device.Route.Pre = Device.Route.Curr;
      }

      /*
       * A Volume 0..100 között értelmezett
       * 0: -110dB Minimum Volume
       * 100: 0dB Maximum Volume
       */
      if(Device.Volume.Pre != Device.Volume.Curr){
        double y = Device.Volume.Curr/100.0;
        double x = (log(y)-log(1E-5))/11.51;
        if(x < 0.1)
          x = 0;
        uint8_t reg = (uint8_t)(255 - x * 255);
        BD34301_RegWrite(0x21, reg);
        BD34301_RegWrite(0x22, reg);
        Device.Volume.Pre = Device.Volume.Curr;
      }




      /*
       * Re-Colck bypass
       *
       *A ReClock a PCM 192-ig müködik, azután Bypassojla...
       *
       * 240109_1609
       * Viktorral átbeszéltük a működést, és arra jutottunk, hogy az I2S és
       * a SPDIF módokban bypass-oljuk a reclockert, az USB módban pedig úgy működik,
       * ahogy eddig kértem.
       *
       */
      if(Device.Route.Curr == ROUTE_USB )
      {
         if(Device.DacAudioFormat == DAC_PCM_352_8KHZ  ||
            Device.DacAudioFormat == DAC_PCM_384_0KHZ  ||
            Device.DacAudioFormat == DAC_PCM_705_6KHZ  ||
            Device.DacAudioFormat == DAC_PCM_768_0KHZ  ||
            Device.DacAudioFormat == DAC_DSD_64  ||
            Device.DacAudioFormat == DAC_DSD_128  ||
            Device.DacAudioFormat == DAC_DSD_256  ||
            Device.DacAudioFormat == DAC_DSD_512
          )
        {
           ReClockBypassOn();
        }
        else
        {
          ReClockBypassOff();
        }
      }
      else
      {
        ReClockBypassOn();
      }
    }

    LiveLedTask(&hLiveLed);
    Com_Task();
    UpTimeTask();
    ButtonsTask();
    UpdateLockIntExtStatus();
    DisplaySleepTask();
    RemoteTask();
    DebugTask(Device.DebugState);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 48000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4799;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DAC_RESETB_Pin|SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_USB_ISO_Pin|USART1_DIR_Pin|EN_SPDIF_ISO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_I2S_ISO_Pin|PWR_CTRL_Pin|RECLKBYPS_Pin|MCLK_SEL_ISO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_SPD_ISO_GPIO_Port, RESET_SPD_ISO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : A0_USB_ISO_Pin */
  GPIO_InitStruct.Pin = A0_USB_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A0_USB_ISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIVE_LED_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIVE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_MUTE_COM_Pin */
  GPIO_InitStruct.Pin = DAC_MUTE_COM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC_MUTE_COM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_RESETB_Pin SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = DAC_RESETB_Pin|SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_USB_ISO_Pin USART1_DIR_Pin EN_SPDIF_ISO_Pin */
  GPIO_InitStruct.Pin = EN_USB_ISO_Pin|USART1_DIR_Pin|EN_SPDIF_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : H5_1_ISO_Pin A1_USB_ISO_Pin A2_USB_ISO_Pin */
  GPIO_InitStruct.Pin = H5_1_ISO_Pin|A1_USB_ISO_Pin|A2_USB_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : H5_3_ISO_Pin LOCK_PLL_Pin INT_EXT_PLL_Pin BTN_PWR_DB_Pin
                           BTN_SEL_DB_Pin MUTE_USB_ISO_Pin DSD_PCM_USB_ISO_Pin */
  GPIO_InitStruct.Pin = H5_3_ISO_Pin|LOCK_PLL_Pin|INT_EXT_PLL_Pin|BTN_PWR_DB_Pin
                          |BTN_SEL_DB_Pin|MUTE_USB_ISO_Pin|DSD_PCM_USB_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_I2S_ISO_Pin PWR_CTRL_Pin RECLKBYPS_Pin MCLK_SEL_ISO_Pin */
  GPIO_InitStruct.Pin = EN_I2S_ISO_Pin|PWR_CTRL_Pin|RECLKBYPS_Pin|MCLK_SEL_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_SPD_ISO_Pin */
  GPIO_InitStruct.Pin = RESET_SPD_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_SPD_ISO_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* Inputs & Outputs ----------------------------------------------------------*/

void SetRoute (Route_t route)
{

  DeviceMuteOn();

  switch(route)
  {
    case ROUTE_USB:{
      HAL_GPIO_WritePin(EN_I2S_ISO_GPIO_Port, EN_I2S_ISO_Pin, GPIO_PIN_SET); //ON -> Deselect I2S-HDMI
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_SET); // ON -> Select USB Input
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_RESET); // OFF -> Deselect SPDIF
      break;
    }
    case ROUTE_I2S_HDMI:{
      HAL_GPIO_WritePin(EN_I2S_ISO_GPIO_Port, EN_I2S_ISO_Pin, GPIO_PIN_RESET); //OFF -> Select I2S-HDMI
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); // OFF -> DeSelect USB Input
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_RESET); //OFF -> DeSelect SPDIF
      break;
    }

    case ROUTE_TOS:{
      HAL_GPIO_WritePin(EN_I2S_ISO_GPIO_Port, EN_I2S_ISO_Pin, GPIO_PIN_SET); //ON -> DeSelect I2S-HDMI
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); //OFF -> DeSelect USB Input
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_SET);//ON -> Select SPDIF -> PCM9211
      PCM9211_SelectSource(PCM9211_RXIN4);
      break;
    }

    case ROUTE_RCA:{
      HAL_GPIO_WritePin(EN_I2S_ISO_GPIO_Port, EN_I2S_ISO_Pin, GPIO_PIN_SET); //ON -> DeSelect I2S-HDMI
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); //OFF -> DeSelect USB Input
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_SET);//ON -> Select SPDIF -> PCM9211
      PCM9211_SelectSource(PCM9211_RXIN1);
      break;
    }

    case ROUTE_BNC:{
      HAL_GPIO_WritePin(EN_I2S_ISO_GPIO_Port, EN_I2S_ISO_Pin, GPIO_PIN_SET); //ON -> DeSelect I2S-HDMI
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); //OFF -> DeSelect USB Input
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_SET); //ON -> Select SPDIF -> PCM9211
      PCM9211_SelectSource(PCM9211_RXIN0);
      break;
    }

    case ROUTE_AES_XLR:{
      HAL_GPIO_WritePin(EN_I2S_ISO_GPIO_Port, EN_I2S_ISO_Pin, GPIO_PIN_SET); //ON -> DeSelect I2S-HDMI
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); //OFF -> DeSelect USB Input
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_SET);//ON -> Select SPDIF -> PCM9211
      PCM9211_SelectSource(PCM9211_RXIN3);
      break;
    }
  }
}

XmosStatus_t ReadXmosStaus(void)
{
  uint8_t status = 0;

  if(HAL_GPIO_ReadPin(A0_USB_ISO_GPIO_Port, A0_USB_ISO_Pin) == GPIO_PIN_SET)
    status |= DI_A0_USB;

  if(HAL_GPIO_ReadPin(A1_USB_ISO_GPIO_Port, A1_USB_ISO_Pin) == GPIO_PIN_SET)
    status |= DI_A1_USB;

  if(HAL_GPIO_ReadPin(A2_USB_ISO_GPIO_Port, A2_USB_ISO_Pin) == GPIO_PIN_SET)
    status |= DI_A2_USB;

  if(HAL_GPIO_ReadPin(DSD_PCM_USB_ISO_GPIO_Port, DSD_PCM_USB_ISO_Pin) == GPIO_PIN_SET)
    status |= DI_DSD_PCM_USB;
  else
    status &= ~DI_A0_USB;
  return (XmosStatus_t)status;
}

uint8_t XmosIsMute(void){
  uint8_t status = HAL_GPIO_ReadPin(MUTE_USB_ISO_GPIO_Port, MUTE_USB_ISO_Pin) == GPIO_PIN_SET;
  return status;
}


void SetMasterClock(MasterClocks_t clk )
{
  if(clk == CLK_24_575MHZ)
    HAL_GPIO_WritePin(MCLK_SEL_ISO_GPIO_Port, MCLK_SEL_ISO_Pin, GPIO_PIN_SET);

  if(clk == CLK_22_5792MHZ)
    HAL_GPIO_WritePin(MCLK_SEL_ISO_GPIO_Port, MCLK_SEL_ISO_Pin, GPIO_PIN_RESET);
}

/* LEDs ----------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

/* HMI -----------------------------------------------------------------------*/
void DevicePowerOn(void)
{
  Device.IsOn = 1;

  /* --- Analóg táp kikapcsolása --- */
  //PWR_CTRL = 0 (ON)
  HAL_GPIO_WritePin(PWR_CTRL_GPIO_Port, PWR_CTRL_Pin, GPIO_PIN_RESET);

  HAL_Delay(500);

  /* --- Bekapcsolom a Power LED-et ---*/
  UsrLeds_On(USR_LED_POWER);

  /*--- Load Last Route ---*/
  uint32_t lastRoute;
  Eeprom_ReadU32(EEPROM_ADDR_LAST_ROUTE, &lastRoute);
  Device.Route.Curr = lastRoute;

  /*--- Bekapcsolom a Rout-hoz tartozó LED-et-- */
  UpdateSelectorLeds(Device.Route.Curr);
  SetRoute (Device.Route.Curr);


  Device.XmosStatus.Pre = XMOS_UNKNOWN;
  Device.Volume.Curr = 100;

  DisplayWake();

  UserMuteOff();
}

void DevicePowerOff(void)
{
  Device.IsOn = 0;

  /* --- Analóg táp kikapcsolása --- */
  HAL_GPIO_WritePin(PWR_CTRL_GPIO_Port, PWR_CTRL_Pin, GPIO_PIN_SET);

  /*--- Minden LED-et kikapcsolok ---*/
  UsrLeds_Off(USR_LED_POWER| USR_LED_AES | USR_LED_BNC | USR_LED_RCA | USR_LED_TOS | USR_LED_I2S | USR_LED_USB | USR_LED_LOCK | USR_LED_EXTREF | USR_LED_MUTE);

  /*--- Minden LED-et kikapcsolok ---*/
  UsrLeds_Off(USR_LED_POWER| USR_LED_AES | USR_LED_BNC | USR_LED_RCA | USR_LED_TOS | USR_LED_I2S | USR_LED_USB | USR_LED_LOCK | USR_LED_EXTREF | USR_LED_MUTE);

  DeviceMuteOn();
}

void DeviceSelectNextRoute(void)
{
  Device.Route.Curr++;

  /*--- Körbe Halad a Route kiválasztása --- */
  if(Device.Route.Curr > ROUTE_AES_XLR)
    Device.Route.Curr = ROUTE_USB;

  /*--- Minden módositás után elmentem az aktuális állapotot ---*/
  Eeprom_WriteU32(EEPROM_ADDR_LAST_ROUTE, Device.Route.Curr);

  DisplayWake();
}

void DeviceSleep(void)
{
  /*--- Minden LED-et kikapcsolok kivéve a POWRER-t  ---*/
  UsrLeds_Off( USR_LED_AES | USR_LED_BNC | USR_LED_RCA | USR_LED_TOS | USR_LED_I2S | USR_LED_USB | USR_LED_LOCK | USR_LED_EXTREF);

  Device.DisplayIsSleep = 1;
}

void DisplayWake(void)
{
  /*--- Innetől ébren van a készülék vagy módositja az ébrenléti timeout kezdetét ---*/
  Device.DisplayIsSleep = 0;
  Device.WakeStartTimestamp = HAL_GetTick();

  /*--- Route akautális LED-jének bekapcsolása ---*/
  UpdateSelectorLeds(Device.Route.Curr);

  /* --- LOCK ---*/
  Device.Lock.Pre = HAL_GPIO_ReadPin(LOCK_PLL_GPIO_Port, LOCK_PLL_Pin);
  if(Device.Lock.Pre)
    UsrLeds_On(USR_LED_LOCK);
  else
    UsrLeds_Off(USR_LED_LOCK);

  /* --- INT_EXT ---*/
  Device.IntExt.Pre = HAL_GPIO_ReadPin(INT_EXT_PLL_GPIO_Port, INT_EXT_PLL_Pin);
  if(Device.IntExt.Pre)
    UsrLeds_On(USR_LED_EXTREF);
  else
    UsrLeds_Off(USR_LED_EXTREF);
}

uint8_t IsDisplaySleep(void)
{
  return Device.DisplayIsSleep;
}

uint8_t DeviceIsOn(void)
{
  return Device.IsOn;
}

void DisplaySleepTask(void)
{
  if(!IsDisplaySleep())
  {
    if(HAL_GetTick() - Device.WakeStartTimestamp > DEVICE_GO_SLEEP_SEC )
    {
      DeviceSleep();
    }
  }
}

/*
 * A nyomogombok GND-re húznak
 * A Debuce áramkor invertál
 * A H szint jelenti a megnyomott állapotot
 */
uint8_t ButtonsPoll (void)
{
  uint8_t retval = 0;
  HAL_GPIO_ReadPin(BTN_PWR_DB_GPIO_Port, BTN_PWR_DB_Pin) ? (retval |= BUTTON_POWER) : (retval &= (~BUTTON_POWER));
  HAL_GPIO_ReadPin(BTN_SEL_DB_GPIO_Port, BTN_SEL_DB_Pin) ? (retval |= BUTTON_SELECTOR) : (retval &= (~BUTTON_SELECTOR));
  return retval;
}

/*
 * Amikor mengyomja a Power nyomógombot, akkor az aktális állpotot invertálja ignorálja a további változást.
 * Szóval a folyamatos nyomvatartás nem kapcsolgatja ki be a készüléket.
 */
void ButtonsTask(void)
{
  static uint8_t powerBtnBlock = 0;
  static uint8_t selectorBtnBlock = 0;

  Device.Buttons = ButtonsPoll();

  /*--- Power Button ---*/
  if((Device.Buttons & BUTTON_POWER) == BUTTON_POWER && !powerBtnBlock)
  {
    powerBtnBlock = 1;
    if(DeviceIsOn())
      DevicePowerOff();
    else
      DevicePowerOn();
  }

  if((Device.Buttons & BUTTON_POWER) != BUTTON_POWER)
    powerBtnBlock = 0;

  /*--- Selector Button ---*/
  if((Device.Buttons & BUTTON_SELECTOR) == BUTTON_SELECTOR && !selectorBtnBlock && DeviceIsOn())
  {
    selectorBtnBlock = 1;
    if(IsDisplaySleep())
      DisplayWake();
    else
      DeviceSelectNextRoute();
  }
  if((Device.Buttons & BUTTON_SELECTOR) != BUTTON_SELECTOR)
    selectorBtnBlock = 0;
}


void UpdateSelectorLeds(Route_t route)
{
  UsrLeds_Off(USR_LED_AES | USR_LED_BNC | USR_LED_RCA | USR_LED_TOS | USR_LED_I2S | USR_LED_USB);
  switch(route)
  {
    case ROUTE_USB : UsrLeds_On(USR_LED_USB); break;
    case ROUTE_I2S_HDMI: UsrLeds_On(USR_LED_I2S); break;
    case ROUTE_TOS: UsrLeds_On(USR_LED_TOS);break;
    case ROUTE_RCA: UsrLeds_On(USR_LED_RCA);break;
    case ROUTE_BNC: UsrLeds_On(USR_LED_BNC);break;
    case ROUTE_AES_XLR: UsrLeds_On(USR_LED_AES);break;
  }
}

/*
 *
 * Visszafelé, ha a külső referenciát lekapcsolom, akkor kijelzi a display, hogy nincs EXTREF és LOCK sem, de ha lockolnak az oszcik, nem jön vissza a LOCK LED a display-en, csak a már említett kézi frissítéssel lehet előcsalogatni! Az EXTREF kijelzése teljesen jó, a LOCK kijelzés előfordul hogy nem az igazat mutatja.
 * Beleépítettem a MUTE piros LED-jét a megadott shift regiszter kimenetére. Mindig világít, ezt szeretném invertálni, tehát ha nincs MUTE, akkor ne világítson, és ha a távvezérlőről lenémítom, akkor azt jelezze ki. A távvezérlő sajnos Viktornál maradt, így ezt most nem tudom korrekt módon tesztelni...
 */

void UpdateLockIntExtStatus(void)
{
  if(DeviceIsOn())
  {
    if(!IsDisplaySleep())
    {
      Device.Lock.Curr = HAL_GPIO_ReadPin(LOCK_PLL_GPIO_Port, LOCK_PLL_Pin);
      if(Device.Lock.Curr != Device.Lock.Pre)
      {
        if(Device.Lock.Curr)
          UsrLeds_On(USR_LED_LOCK);
        else
          UsrLeds_Off(USR_LED_LOCK);

        Device.Lock.Pre = Device.Lock.Curr;
      }
      Device.IntExt.Curr = HAL_GPIO_ReadPin(INT_EXT_PLL_GPIO_Port, INT_EXT_PLL_Pin);
      if(Device.IntExt.Curr != Device.IntExt.Pre)
      {
        if(Device.IntExt.Curr)
          UsrLeds_On(USR_LED_EXTREF);
        else
          UsrLeds_Off(USR_LED_EXTREF);

        Device.IntExt.Pre = Device.IntExt.Curr;
      }
    }
  }
}


/*  Mute---- -----------------------------------------------------------------*/

//A Mute LED érdekes... a MuteOn funkcióban kapcsolja be (elméletileg)  szóval más probléma lehet...
void UserMuteOn(void)
{
  Device.UserIsMute = 1;
  UsrLeds_On(USR_LED_MUTE);
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin, GPIO_PIN_RESET);
}

void UserMuteOff(void)
{
  Device.UserIsMute = 0;
  UsrLeds_Off(USR_LED_MUTE);
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin, GPIO_PIN_SET);
}

uint8_t UserIsMute(void)
{
  return Device.UserIsMute;
}

/*
 * Call by hardwre only
 */
void DeviceMuteOn(void)
{
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin, GPIO_PIN_RESET);
}

/*
 * Call by hardwre only
 *
 * Ha a User némitott és DAC olyan részhez ér ahol némitania kell, majd kikapcsolni a ném itást,
 * akkor megvizsgálja hogy visszakapcsolhatja-e a némitást.
 */
void DeviceMuteOff(void)
{
  if(!UserIsMute())
    HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin, GPIO_PIN_SET);
}

/* IR REMOTE -----------------------------------------------------------------*/
void IR_NEC_Parser (uint8_t address, uint8_t command)
{
  printf("IR REMOTE ADDRESS:0x%02X COMMAND:0x%02X\r\n", address, command);
  Device.RemoteCommand = command;
}

void RemoteTask(void)
{
  switch(Device.RemoteCommand)
  {
    /*--- Power On/Off ---*/
    case 0x4D:
    //case 0x03:
    {
      if(DeviceIsOn())
        DevicePowerOff();
      else
        DevicePowerOn();
      break;
    }

    /*--- Route ---*/
    case 0x54:
    {
      if(Device.IsOn)
      {
        if(IsDisplaySleep())
          DisplayWake();
        else
          DeviceSelectNextRoute();
      }
      break;
    }

    /*--- Mute ---*/
    case 0x16:
    //case 0x07:
    {
      if(Device.IsOn)
      {
        if(UserIsMute())
          UserMuteOff();
        else
          UserMuteOn();
      }
      break;
    }
  }
  Device.RemoteCommand = 0;
}

/* DEBUG ---------------------------------------------------------------------*/
void DebugTask(DebugState_t dbg)
{
  switch(Device.DebugState){
    case SDBG_IDLE:{
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_MAKE_HARDFAULT:{
      *(__IO uint32_t *) 0xA0001000 = 0xFF;
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_HARD_RESET:{
      NVIC_SystemReset();
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_DAC_MUTE_ON:{
      /*** Mute On ***/
      BD34301_RegWrite(0x2A, 0x00);
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_DAC_MUTE_OFF:{
      /*** Mute Off ***/
      BD34301_RegWrite(0x2A, 0x03);
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_DAC_RECONFIG:{
      //DacSoftRstOn();
      //DacSetParams(&DacConfigurations[Device.DacAudioFormat], Device.MasterClock);
      //DacSoftRstOff();
      break;
    }

    case SDBG_LAST:{
      dbg = SDBG_IDLE;
      break;
    }
  }
}

/* printf --------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  //HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

/* Tools----------------------------------------------------------------------*/
void UpTimeTask(void)
{
  static uint32_t timestamp;
  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.UpTimeSec++;
  }
}


/* ReClock--------------------------------------------------------------------*/
void ReClockBypassOn(void)
{
  /*--- BYPASS ACTIVE ---*/
  HAL_GPIO_WritePin(RECLKBYPS_GPIO_Port, RECLKBYPS_Pin, GPIO_PIN_SET);
  Device.ReClockBypassIsActiveStatus = true;
}


void ReClockBypassOff(void)
{
  /*--- RECLOCK ACTIVE ---*/
  HAL_GPIO_WritePin(RECLKBYPS_GPIO_Port, RECLKBYPS_Pin, GPIO_PIN_RESET);

  Device.ReClockBypassIsActiveStatus = false;
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
