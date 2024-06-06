/*
 * com.c
 *
 *  Created on: Dec 18, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *_uart;
static DMA_HandleTypeDef *_dma;

extern Device_t Device;

char  UartRxBuffer[UART_BUFFER_SIZE];
char  UartTxBuffer[UART_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
static char* Parser(char *line);
static inline void DirRx(void);
static inline void DirTx(void);
static void TxTask(void);
static void RxTask(void);
/* Private user code ---------------------------------------------------------*/
void Com_Init(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma)
{
  _uart = uart;
  _dma = dma;

  if(HAL_UART_Receive_DMA(_uart, (uint8_t*)UartRxBuffer, UART_BUFFER_SIZE)!= HAL_OK)
    Device.Diag.UartErrorCnt++;
  __HAL_DMA_DISABLE_IT(_dma, DMA_IT_HT);
}

void Com_Task()
{
  TxTask();
  RxTask();
}

static char* Parser(char *line)
{
  unsigned int addr = 0;
  char buffer[UART_BUFFER_SIZE];
  char cmd[RS485_CMD_LENGTH];
  char arg1[RS485_ARG1_LENGTH];
  char arg2[RS485_ARG2_LENGTH];

  int intarg;

  memset(buffer, 0x00, UART_BUFFER_SIZE);
  memset(cmd,0x00, RS485_CMD_LENGTH);
  memset(arg1,0x00, RS485_ARG1_LENGTH);
  memset(arg2,0x00, RS485_ARG2_LENGTH);

  sscanf(line, "#%x %s",&addr, cmd);
  if(addr != CLIENT_RX_ADDR)
  {
    Device.Diag.RS485NotMyCmdCnt++;
    return NULL;
  }
  Device.Diag.RS485RequestCnt++;

  if(!strcmp(cmd, "*IDN?")){
    sprintf(buffer, "*IDN? %s", DEVICE_NAME);
  }
  else if(!strcmp(cmd, "*OPC?")){
    strcpy(buffer, "*OPC? OK");
  }
  else if(!strcmp(cmd, "*WHOIS?")){
    sprintf(buffer, "*WHOIS? %s", DEVICE_NAME);
  }
  else if(!strcmp(cmd, "FW?")){
    sprintf(buffer, "FW? %s", DEVICE_FW);
  }
  else if(!strcmp(cmd, "UID?")){
    sprintf(buffer, "UID? %4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
  }
  else if(!strcmp(cmd, "PCB?")){
    sprintf(buffer, "PCB? %s", DEVICE_PCB);
  }
  else if(!strcmp(cmd,"UPTIME?")){
     sprintf(buffer, "UPTIME? %08lX", Device.UpTimeSec);
  }
  else if(!strcmp(cmd,"UE?")) {
    sprintf(buffer, "UE? %08lX", Device.Diag.UartErrorCnt);
  }

  /*** CLOCKS ***/
  else if(!strcmp(cmd,"AUDIO?")){
     sprintf(buffer, "AUDIO? %02X", Device.AudioType.Curr);
  }
  else if(!strcmp(cmd,"MASTER:CLK?")){
     sprintf(buffer, "MASTER:CLK? %02X", Device.MasterClock);
  }
  else if(!strcmp(cmd,"FRQ:LRCK?")){
    sprintf(buffer, "FRQ:LRCK? %lX", Device.Meas.FreqLRCK_MHz);
  }
  else if(!strcmp(cmd,"FRQ:BCLK?")){
    sprintf(buffer, "FRQ:BCLK? %lX", Device.Meas.FreqBCLK_MHz);
  }

  /*** XMOS ***/
  else if(!strcmp(cmd,"XMOS:STATUS?")){
    sprintf(buffer, "XMOS:STATUS? %02X", Device.XmosStatus.Curr );
  }

  /*** VOLUME ***/
  else if(!strcmp(cmd,"DAC:VOL"))
  {
    sscanf(line, "#%x %s %x",&addr, cmd, &intarg);
    Device.Volume.Curr = intarg;
    strcpy(buffer, "DAC:VOL OK");
  }
  else if (!strcmp(cmd,"DAC:VOL?")){
    sprintf(buffer, "DAC:VOL? %01X", (uint8_t)Device.Volume.Curr);
  }

  /*** DAC CONFIG ***/
  else if(!strcmp(cmd,"DAC:CONFIG")){
    sscanf(line, "#%x %s %d",&addr, cmd, &intarg);

    Device.DacMode = intarg;

    BD34301_DigitalPowerOff();
    BD34301_SoftwareResetOn();

    BD34301_ModeSwitching(Device.DacMode, Device.DacRollOff);

    BD34301_SoftwareResetOff();
    BD34301_DigitalPowerOn();
    BD34301_RamClear();
    BD34301_MuteOff();

    strcpy(buffer, "DAC:CONFIG OK");
  }
  else if(!strcmp(cmd,"DAC:CONFIG?")){
    sprintf(buffer, "DAC:CONFIG? %02X", (uint8_t)Device.DacMode);
  }

  /*** DAC PARAMS ***/
  else if(!strcmp(cmd,"DAC:PARAMS?")){
    sprintf(buffer, "DAC:PARAMS? %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
      BD34301_ModeList[Device.DacMode].Clock2,
      BD34301_ModeList[Device.DacMode].AudioIf3,
      BD34301_ModeList[Device.DacMode].DsdFilter,
      BD34301_ModeList[Device.DacMode].FirFilter1,
      BD34301_ModeList[Device.DacMode].FirFilter2,
      BD34301_ModeList[Device.DacMode].DeEmph1,
      BD34301_ModeList[Device.DacMode].DeEmph2,
      BD34301_ModeList[Device.DacMode].DeltaSigma);
  }
  else if(!strcmp(cmd,"DAC:PARAMS")){
    int arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8;
      sscanf(line, "#%x %s %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                    &addr, cmd, &arg1, &arg2, &arg3, &arg4, &arg5, &arg6, &arg7, &arg8);
      BD34301_ModeList[Device.DacMode].Clock2 = arg1;
      BD34301_ModeList[Device.DacMode].AudioIf3  = arg2;
      BD34301_ModeList[Device.DacMode].DsdFilter = arg3;
      BD34301_ModeList[Device.DacMode].FirFilter1 = arg4;
      BD34301_ModeList[Device.DacMode].FirFilter2 = arg5;
      BD34301_ModeList[Device.DacMode].DeEmph1 = arg6;
      BD34301_ModeList[Device.DacMode].DeEmph2 = arg7;
      BD34301_ModeList[Device.DacMode].DeltaSigma = arg8;
      strcpy(buffer, "DAC:CONFIG OK");
      Device.Diag.DacReConfgiurationCnt++;
      SetMasterClock(Device.MasterClock);
      BD34301_ModeSwitching(Device.DacMode, Device.DacRollOff);
  }

  /*** ROUTE ***/
  else if(!strcmp(cmd,"ROUTE?")){
     sprintf(buffer, "ROUTE? %02X", Device.Route.Curr);
  }
  else if(!strcmp(cmd,"ROUTE:ACTUAL?")){
     sprintf(buffer, "ROUTE:ACTUAL? %02X", Device.Route.Curr);
  }
  else if(!strcmp(cmd,"ROUTE")){
    sscanf(line, "#%x %s %d",&addr, cmd, &intarg);
    Device.Route.Curr = intarg;
    strcpy(buffer, "ROUTE OK");
  }

  /*** SRC csak kompatiblitás miatt ***/
  else if (!strcmp(cmd,"SRC:PARAMS?")){
    sprintf(buffer, "SRC:PARAMS? %02X:%02X:%02X", 0, 0, 0);
  }
  else if (!strcmp(cmd,"SRC:FSOUT?")){
    sprintf(buffer, "SRC:FSOUT? %02X", 0 );
  }
  else if(!strcmp(cmd,"SRC:FSOUT")){
    sscanf(line, "#%x %s %02X",&addr, cmd, &intarg);
      strcpy(buffer, "SRC:FSOUT OK");
  }
  else if(!strcmp(cmd,"SRC:EN")){
    sscanf(line, "#%x %s %02X", &addr, cmd, &intarg);
      strcpy(buffer, "SRC:EN OK");
  }
  else if(!strcmp(cmd,"SRC:EN?")){
    sprintf(buffer, "SRC:EN? %02X", 0);
  }
  else{
    Device.Diag.RS485UnknwonCnt++;
  }
  static char resp[UART_BUFFER_SIZE + 5];
  memset(resp, 0x00, UART_BUFFER_SIZE);
  sprintf(resp, "#%02X %s", CLIENT_TX_ADDR, buffer);
  return resp;
}

static void TxTask(void)
{
  uint8_t txLen = strlen(UartTxBuffer);
  if(txLen != 0)
  {
    Device.Diag.RS485ResponseCnt++;
    DirTx();
    DelayMs(RS485_TX_HOLD_MS);

    UartTxBuffer[txLen] = UART_TERIMINATION_CHAR;
    UartTxBuffer[txLen + 1] = '\0';

    HAL_UART_Transmit(_uart, (uint8_t*) UartTxBuffer, txLen + 1, 100);
    UartTxBuffer[0] = 0;
    DirRx();
  }
}

static void RxTask(void)
{
  for(uint8_t i=0; i < UART_BUFFER_SIZE; i++)
  {
    if(UartRxBuffer[i]==UART_TERIMINATION_CHAR)
    {
      HAL_UART_DMAStop(_uart);
      strcpy(UartTxBuffer, Parser(UartRxBuffer));
      memset(UartRxBuffer, 0x00, UART_BUFFER_SIZE);
      if(HAL_UART_Receive_DMA(_uart, (uint8_t*)UartRxBuffer, UART_BUFFER_SIZE)!= HAL_OK)
        Device.Diag.UartErrorCnt++;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Device.Diag.UartErrorCnt++;
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
}

void UART_DMAError(UART_HandleTypeDef *huart)
{
  Device.Diag.UartErrorCnt++;
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
}


static inline void DirTx(void)
{
  HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
}

static inline void DirRx(void)
{
  HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
}



/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/


