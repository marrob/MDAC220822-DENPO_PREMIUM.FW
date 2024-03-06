/*
 * freq_meas.c
 *
 *  Created on: Feb 29, 2024
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define FRMETER_TIM_TIMEBASE      TIM3
#define FRMETER_TIM_LRCK_COUNTER  TIM2
#define FRMETER_TIM_BCLK_COUNTER  TIM1

#define FrMeterTimebaseItEnable()     __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE)
#define FreqMeterTimebaseStart()      __HAL_TIM_ENABLE(&htim3)
#define FreqMeterTimebaseValue        FRMETER_TIM_TIMEBASE->CNT

#define FreqMeterLrckCoutnerStart()    __HAL_TIM_ENABLE(&htim2)
#define FreqMeterLrckCounterValue     FRMETER_TIM_LRCK_COUNTER->CNT

#define FreqMeterBclkCoutnerStart()    __HAL_TIM_ENABLE(&htim1)
#define FreqMeterBclkCounterValue     FRMETER_TIM_BCLK_COUNTER->CNT

/* Private variables ---------------------------------------------------------*/
extern Device_t Device;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

uint32_t AudioFreqArray[] ={
  32000,
  44100,  // -> PCM -> LRCK jel
  48000,  // -> PCM -> LRCK jel
  88200,  // -> PCM -> LRCK jel
  96000,  // -> PCM -> LRCK jel
  176400,
  192000,
  352800,
  384000,
  705600,  // -> PCM -> LRCK jel
  2822400, // -> DSD64 -> BCLK jel
  5644800, // -> DSD128 -> BCLK jel
  11289600,// -> DSD256 -> BCLK jel
};

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* GetAudioType --------------------------------------------------------------*/
/*
 * - Az Audio jel tipusat a jel LRCK BCLK mérésvel határozom meg
 * - Az ismert frekvenciak a RLCKfreqArray definialja
 * - BLCK = 64xLRCK
 */
AudioTypes_t FreqMeas_GetAudioType(void)
{
  float tol = 0.04;

  uint32_t lrck = Device.Meas.FreqLRCK_MHz;
  uint32_t bclk  = Device.Meas.FreqBCLK_MHz;

  uint32_t lLrck = 0, hLrck = 0;
  uint32_t lbclk = 0, hbclk = 0;

  AudioTypes_t result = AUDIO_UNKNOWN;

  for(uint8_t i = AUDIO_PCM_32_0KHZ; i < sizeof(AudioFreqArray)/sizeof(uint32_t); i++)
  {
    uint32_t templrck = AudioFreqArray[i];
    hLrck = templrck + templrck * tol;
    lLrck = templrck - templrck * tol;
    if((lLrck < lrck) && (lrck < hLrck))
    {
      uint32_t tempbclk = AudioFreqArray[i] * 64;
      hbclk = tempbclk + tempbclk * tol;
      lbclk = tempbclk - tempbclk * tol;
      if((lbclk < bclk) && (bclk < hbclk))
      {
        result = i;
        break;
      }
    }
  }

  if(result == AUDIO_UNKNOWN)
  {
    /* Itt már csak DSD lehet most BLCK vonalat kell figyelni csak
     * A táblázatban mos BCLK jelentenek a sorosk
     * */
    for(uint8_t i = AUDIO_DSD_64; i < sizeof(AudioFreqArray)/sizeof(uint32_t); i++){
      uint32_t tempblck = AudioFreqArray[i];
      hbclk = tempblck + tempblck * tol;
      lbclk = tempblck - tempblck * tol;
      if((lbclk < bclk) && (bclk < hbclk)){
        result = i;
        break;
      }
    }
  }
  return result;
}

/* FrMeter -------------------------------------------------------------------*/
void FreqMeas_Start(void)
{
  FreqMeterLrckCounterValue = 0;
  FreqMeterBclkCounterValue = 0;

  FreqMeterTimebaseValue = 0;
  FreqMeterTimebaseStart();
  FreqMeterLrckCoutnerStart();
  FreqMeterBclkCoutnerStart();
  FrMeterTimebaseItEnable();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == FRMETER_TIM_TIMEBASE)
   {
     Device.Meas.FreqBCLK_MHz = (FreqMeterBclkCounterValue) * 1000 * 10;
     Device.Meas.FreqLRCK_MHz = (FreqMeterLrckCounterValue) * 1000;

     FreqMeterBclkCounterValue = 0;
     FreqMeterLrckCounterValue = 0;

     /*
      * Ezeket Timebase teszthez használd, ez mérési tartomány 100Hz-től 4MHz-ig használható
      * 10ms-es időalap estén pl a LIVE_LED-nél 20ms-es periódusidőt kell mérned
      * 2022.02.02 by marrob
      */
     //HAL_GPIO_TogglePin(TIMEBASE_OUT_GPIO_Port, TIMEBASE_OUT_Pin);
   }
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
