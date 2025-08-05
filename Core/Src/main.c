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
  */
#include "main.h"
#include "app_lorawan.h"
#include "app_bluenrg_2.h"
#include "w25qxx.h"
#include "lora_app.h"
#include "flash_if.h"
#include "time.h"
#include "stm32_systime.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "sys_app.h"
#include "MS5837.h"
#include "LIS2DW12.h"
#include "STWIN_bc.h"
#include "prod.h"
#include "LoRaMacInterfaces.h"
#include "LoRaMac.h"
#include <stdio.h>
#include <stdlib.h>

ADC_HandleTypeDef hadc; // monitor battery voltage
CRC_HandleTypeDef hcrc; // check OTA firmware update integrity
I2C_HandleTypeDef hi2c2; // pressure sensor and LED FLASH
RTC_HandleTypeDef hrtc; // clock time
SPI_HandleTypeDef hspi1; // SPI communication port
TIM_HandleTypeDef htim2; // used by stbc02 driver
TIM_HandleTypeDef htim16; // used for pwm sound output
TIM_HandleTypeDef htim1; // used for pwm sound output hardware 14 min
TIM_HandleTypeDef htim17; // used for pwm sound output

UART_HandleTypeDef huart1; //back serial port
UART_HandleTypeDef huart2; // GNSS com port
SUBGHZ_HandleTypeDef hsubghz; // LoRA

enum button_action {nop, short_press, long_press , double_click , triple_click, short_long_press};

uint8_t  next_power_state = POWER_ON;
uint32_t  lastActivity;
uint8_t  gpsFix = 0;
uint8_t  gps_lost_cnt = 10;
uint8_t  emergency_mode_en = 0;
uint8_t  battery_level = 0;
uint8_t  blink_power_led = 0;
uint32_t SysTimeUpdateDate = 0;
uint8_t  Flash_cnt = 1;
//uint8_t  GNSS_EN = 0;
//uint8_t  GNSS_On_request = 1;
//uint8_t  GNSS_Off_request = 1;
uint8_t  MSG[100];
uint8_t  aGNSSRxBuffer[1000];
uint8_t  aCOMRxBuffer[1000];
uint8_t  button_status ;
uint32_t button_time[6];
uint32_t button_time_ack ;
gps_data_t gps_data;
uint8_t  EphemerisLoaded=0;

uint8_t newLine[] = "\r\n";
#if HARDWARE_VERSION >= 14
uint8_t aTXGetGNSSModule[] = "PQTMVERNO";
uint8_t aTxSearchGPSonly[] = "PAIR066,1,0,0,0,0,0";
uint8_t aTxColdStartGNSS[] = "PAIR006";
uint8_t aTxFullColdStartGNSS[] = "PAIR007";//reset factory
uint8_t aTxHotStartGNSS[] = "PAIR004";
uint8_t aTxStopGNSS[] = "PAIR003";
uint8_t aTxSearchFull[] = "PAIR064,1,1,1,1,0";
uint8_t aTxWarmStartGNSS[] = "PAIR005";



#else

uint8_t aTxStandbyGNSS[] = "PMTK161,0*28\r\n";
uint8_t aTxStopGNSS[] = "PMTK161,0";
uint8_t aTxEraseFlashGNSS[] = "PMTK127";
uint8_t aTxColdStartGNSS[] = "PMTK104";
uint8_t aTxWarmStartGNSS[] = "PMTK102";
uint8_t aTxHotStartGNSS[] = "PMTK101";
uint8_t aTxSearchGPSonly[] = "PMTK353,1,0,0,0,0";
uint8_t aTxSearchGPSandGALILEO[] = "PMTK353,1,0,1,0,0";
uint8_t aTxSearchGPSandGLONASS[] = "PMTK353,1,1,0,0,0";
uint8_t aTxGNSSNoOutput[] = "PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
uint8_t aTxGNSSFullOutput[] = "PMTK314,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0";
uint8_t aTxGNSSOutput[] = "PMTK314,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0";
uint8_t aTxGNSSDutyCycle[] = "PMTK225,1,10000,60000,30000,1200000";
uint8_t aTxGNSSEphemQuery[] = "PMTK869";
#endif


#define DMA_RX_BUFFER_SIZE 1024
#define GNSS_ACCUM_BUFFER_SIZE 2048

uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];
uint8_t gnss_accum_buffer[GNSS_ACCUM_BUFFER_SIZE];
uint16_t gnss_accum_len = 0;
volatile uint16_t dma_old_pos = 0;
DMA_HandleTypeDef hdma_usart2_rx;


char list_checking[][20] = {"LED_GREEN1","LED_GREEN2","LED_RED1","LED_RED2","LED_FLASH","SOUND","PRESSURE","ACCELEROMETER","BC_STATE","BLE","LORA","GNSS","FLASH","EMERGENCY_TAB"};
uint8_t flag_test = 1;
uint8_t start_flag = 1;
uint8_t start_device = 1;
uint8_t stop_flag = 0;
uint8_t loop_flag = 0;
uint8_t emergency_flag = 0;
uint8_t test_emergency = 0;
uint8_t buttonState = 0;
uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 100;
uint32_t buttonPressedTime = 0;
uint32_t buttonReleasedTime = 0;

//PMTK314,5,5,5,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,0
//PMTK225,1,3000,5000,6000,10000 //
//PMTK225,1,1000,5000,2000,8000 //on 1sec then back up 5sec (2sec then 8sec if not fixed)
//PMTK220,1000 //fix position every 1 sec
//PMTK225,0
//PMTK353,0,0,1,0,0 GALILEO only
//PMTK353,0,0,0,1,0 GALILEO FULL only
//PMTK353,0,0,1,1,0 GALILEO + FULL
//PMTK353,1,0,0,0,0 GPS only
//PMTK353,0,1,0,0,0 GLOSNASS only
//PMTK353,1,0,1,1,0 GPS GALILEO
//PMTK353,1,1,0,0,0 GPS GLOSNASS

static UTIL_TIMER_Object_t timerGPSLed;
static UTIL_TIMER_Object_t timerGPS;
static UTIL_TIMER_Object_t timerFlash;
static UTIL_TIMER_Object_t timerFlashOff;
static UTIL_TIMER_Object_t timerLed;
static UTIL_TIMER_Object_t timerLedOff;


/* USER CODE END PV */

static void SystemClock_Config(void);
static void MX_ADC_Init(void);
void MX_CRC8_Init(void);
void MX_CRC32_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM17_Init(void);
extern void OnSysTimeUpdate(void);


void Scan_I2C(void)
{
  uint8_t i,ret;

  for(i=0; i<127; i++)
  {
      ret = HAL_I2C_IsDeviceReady(&hi2c2, i<<1, 3, 5);
      if (ret != HAL_OK) /* No ACK Received At That Address */
      {
    HAL_Delay(10);
      }
      else if(ret == HAL_OK)
      {
    HAL_Delay(100);
      }
  }
}

void Flash(uint8_t action)
{
  static uint8_t EN = 0;
  uint8_t  data_send[2];
  uint8_t  data_receive[7];

  if (action == 0)
    {
      EN = 0;
    }

  else if (action == 1)
    {
      EN = 1;
    }
  else
    {
      EN = !EN;
    }

  if (EN == 0)
  {
    //turn torch OFF
    data_send[0] = 0x01; //Enable Register;
    data_send[1] = 0x20; //data to write
    HAL_I2C_Master_Transmit(&hi2c2, 0xC8, data_send, 2, 100);
  }
  else
    {
    //turn torch ON
    data_send[0] = 0x01; //Enable Register;
    data_send[1] = 0x22; //data to write
    HAL_I2C_Master_Transmit(&hi2c2, 0xC8, data_send, 2, 100);
    }

  //Read device id register
  data_send[0] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c2, 0xC8, data_send, 1, 100);
  data_receive[0] = 0x0;
  data_receive[1] = 0x0;
  HAL_I2C_Master_Receive(&hi2c2, 0xC8, data_receive, 7, 100);
  if (data_receive[0] != data_send[1])
	  HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_SET);

}

void Flash_conf(void)
{
  uint8_t  data_send[2];
  uint8_t  data_receive[7];

  Flash(0);

// increase light
  data_send[0] = 0x04; //register_pointer;
//  data_send[1] = 0x00;  //  min brightness code 0x00 = 2.4mA
//  data_send[1] = 0x15;  //  low brightness code 0x15 = 64mA
  data_send[1] = 0x4F;  //  max brightness code 0x4F = 240mA
  HAL_I2C_Master_Transmit(&hi2c2, 0xC8, data_send, 2, 100);

  // Boost freq basse, ILIM élevé, IVFM bas
    data_send[0] = 0x02;
    data_send[1] = 0x18;
    HAL_I2C_Master_Transmit(&hi2c2, 0xC8, data_send, 2, 100);


  //Read device id register
  data_send[0] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c2, 0xC8, data_send, 1, 100);
  data_receive[0] = 0x0;
  data_receive[1] = 0x0;
  HAL_I2C_Master_Receive(&hi2c2, 0xC8, data_receive, 7, 100);
  if (data_receive[0] != data_send[1])
	  HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_SET);

}

void Play_Sound(uint16_t freq, uint16_t duration)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(BUZZER_EN_GPIO_Port, BUZZER_EN_Pin, GPIO_PIN_SET);
  __HAL_TIM_CLEAR_IT(&htim16, TIM_IT_UPDATE);

  TIM16->ARR = 10*duration ;
#if HARDWARE_VERSION >= 14
  TIM1->ARR = (uint32_t)((SystemCoreClock / freq) -1 );
  TIM1->CCR1 =  (uint32_t)((SystemCoreClock / freq) -1 )/2;

  //TIM1->CCER |= TIM_CCER_CC3E;
  //TIM1->BDTR = 0x5BDF;
  __HAL_TIM_MOE_ENABLE(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
#else
  TIM17->ARR = (uint32_t)((SystemCoreClock / freq) -1 );
  TIM17->CCR1 =  (uint32_t)((SystemCoreClock / freq) -1 )/2;

  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);
#endif
  HAL_TIM_Base_Start_IT(&htim16);
}

void Play_Start_Music(void)
{
	Play_Sound(1046, 150);
	HAL_Delay(200);
	Play_Sound(1175, 150);
	HAL_Delay(200);
	Play_Sound(1318, 150);
	HAL_Delay(200);
	Play_Sound(1397, 150);
	HAL_Delay(200);
	Play_Sound(1567, 150);
}

void Play_Stop_Music(void)
{
	Play_Sound(1567, 150);
	HAL_Delay(200);
	Play_Sound(1397, 150);
	HAL_Delay(200);
	Play_Sound(1318, 150);
	HAL_Delay(200);
	Play_Sound(1175, 150);
	HAL_Delay(200);
	Play_Sound(1046, 150);
}

unsigned char calc_nmea_checksum(const char* sentence)
{
  unsigned char checksum = 0;
  while (*sentence)
  {
      checksum ^= (unsigned char)*sentence++;
  }
  return checksum;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->ErrorCode & HAL_UART_ERROR_FE) {
    // Reset framing error and relance propre
    __HAL_UART_CLEAR_FEFLAG(huart);

  }
  //Safe_ReceiveToIdle_DMA(huart, dma_rx_buffer, DMA_RX_BUFFER_SIZE);
}


void GNSS_Stop(void)
{
  //APP_LOG(TS_ON, VLEVEL_L, "GNSS STOP\r\n");
  sprintf(MSG,"$%s*%02X\r\n", aTxStopGNSS, calc_nmea_checksum(aTxStopGNSS));
  //HAL_UART_Transmit(&huart2, (uint8_t *)MSG, strlen((char *)MSG),200);
  HAL_Delay(100);
  BSP_BC_CmdSend(GNSS_OFF);
  BSP_BC_CmdSend(GNSS_OFF2);
  HAL_Delay(100);
//  HAL_UART_DeInit(&huart2);
//  GNSS_Off_request = 0;
//  GNSS_EN = 0;
}

void GNSS_Start(uint8_t coldStart)
{

	sprintf((char *)&MSG, "GNSS_Start\r\n\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);

	  BSP_BC_CmdSend(GNSS_ON);
	  BSP_BC_CmdSend(GNSS_ON2);

	  HAL_Delay(1000); // Attendre que le GNSS stabilise et que le TX devienne actif
	  sprintf((char *)&MSG, "GNSS_Started\r\n\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);

	  if (coldStart == 1)
	  {
#if HARDWARE_VERSION >= 14
	      sprintf(MSG, "$%s*%02X\r\n", aTXGetGNSSModule, calc_nmea_checksum(aTXGetGNSSModule));
	      HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		  HAL_UART_Transmit(&huart2, (uint8_t *)MSG, strlen((char *)MSG), 200);
		  HAL_Delay(200);

	    sprintf(MSG, "$%s*%02X\r\n", aTxColdStartGNSS, calc_nmea_checksum(aTxColdStartGNSS));
	    HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
	    HAL_UART_Transmit(&huart2, (uint8_t *)MSG, strlen((char *)MSG), 200);
	    HAL_Delay(200);

	    sprintf(MSG, "$%s*%02X\r\n", aTxSearchGPSonly, calc_nmea_checksum(aTxSearchGPSonly));
	    sprintf(MSG, "$%s*%02X\r\n", aTxSearchFull, calc_nmea_checksum(aTxSearchFull));

	    HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
	    HAL_UART_Transmit(&huart2, (uint8_t *)MSG, strlen((char *)MSG), 200);
	    HAL_Delay(200);

#else

	    sprintf(MSG, "$%s*%02X\r\n", aTxColdStartGNSS, calc_nmea_checksum(aTxColdStartGNSS));
	    HAL_UART_Transmit(&huart2, (uint8_t *)MSG, strlen((char *)MSG), 200);
	    HAL_Delay(200);
	    sprintf(MSG, "$%s*%02X\r\n", aTxSearchGPSonly, calc_nmea_checksum(aTxSearchGPSonly));
	    HAL_UART_Transmit(&huart2, (uint8_t *)MSG, strlen((char *)MSG), 200);
	    HAL_Delay(200);
	    sprintf(MSG, "$%s*%02X\r\n", aTxGNSSOutput, calc_nmea_checksum(aTxGNSSOutput));
	    HAL_UART_Transmit(&huart2, (uint8_t *)MSG, strlen((char *)MSG), 200);
#endif
	  }
	  else
	  {
	    HAL_Delay(200);
	    sprintf(MSG, "$%s*%02X\r\n", aTxHotStartGNSS, calc_nmea_checksum(aTxHotStartGNSS));
	    HAL_UART_Transmit(&huart2, (uint8_t *)MSG, strlen((char *)MSG), 200);
	  }


	  // Démarre la réception avec idle interrupt
	  //HAL_UARTEx_ReceiveToIdle_IT(&huart2, aGNSSRxBuffer, sizeof(aGNSSRxBuffer));

return;

}

void Acc_Write_Reg(uint8_t add, uint8_t data)
{
  uint8_t  data_send[2];

  HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);
  data_send[0] = add;
  data_send[1] = data;
  HAL_SPI_Transmit(&hspi1, data_send, 2, 10);
  HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
}

// Power Down accelerometer
void Acc_PowerDown(void)// refer to DT0101 Design tip doc. (Setting up single-tap and double-tap...
{
  Acc_Write_Reg(0x20, 0x04);
}


// configure accelerometer
// send INT on INT 2 after 1m15 of inactivity
// send INT on INT1 when 80° rotation happens
void Acc_Conf(void)// refer to DT0101 Design tip doc. (Setting up single-tap and double-tap...
{
  // inactivity
  Acc_Write_Reg(0x20, 0x50); // Turn on the accelerometer  ODR = 200 Hz, FS = ±2 g
  Acc_Write_Reg(0x35, 0x4F); // Set duration for inactivity detection  Set duration for wake-up detection
  Acc_Write_Reg(0x34, 0x41); // Set activity/inactivity threshold  Enable activity/inactivity detection
  Acc_Write_Reg(0x24, 0xC0); // SLEEP_state driven to INT2 pin
  Acc_Write_Reg(0x3F, 0x20); // Enable interrupts


  //6D detection
  Acc_Write_Reg(0x25,0x04); //CTRL6 // FS ±2 g, LOW_NOISE enabled
  Acc_Write_Reg(0x30,0x60); //TAP_THS_X // Set 6D threshold (6D_THS[1:0] = 10b = 60 degrees)
  Acc_Write_Reg(0x23,0x80); //CTRL4_INT1_PAD_CTRL // 6D interrupt driven to INT1 pin
}

void Check_Button(int reset)
{
  if (reset == 1)
    {
      button_time_ack = HAL_GetTick();
      button_time[5]=button_time_ack;
      button_time[4]=button_time_ack;
      button_time[3]=button_time_ack;
      button_time[2]=button_time_ack;
      button_time[1]=button_time_ack;
      button_time[0]=button_time_ack;
    }
  //check if buttons has been activated and what kind of push
  // button still pressed = long press
  else if(button_time[5] != button_time_ack && HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin)==GPIO_PIN_SET)
  {
    //short_long
    if(HAL_GetTick() - button_time[5] > 700 && button_time[5] - button_time[4] < 500)
    {
      button_status = short_long_press;
      button_time_ack = button_time[5];
    }
    //long
    else if(HAL_GetTick() - button_time[5] > 700)
    {
      button_status = long_press;
      button_time_ack = button_time[5];
    }
  }
  // button released = short press
  else if(button_time[5] != button_time_ack && HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin)== GPIO_PIN_RESET)
  {
    // three short press
    if(button_time[5] - button_time[3] < 500 && button_time[3] - button_time[1] < 500 && button_time[3]!=0)
    {
      button_status = triple_click;
      button_time_ack = button_time[5];
    // two short press
    }
    else if(HAL_GetTick() - button_time[5] > 500 && button_time[5] - button_time[3] < 500 && button_time[3]!=0)
    {
      button_status = double_click;
      button_time_ack = button_time[5];
    }
    // single short press
    else if(HAL_GetTick() - button_time[5] > 500 && button_time[5] - button_time[4] < 500 && button_time[4]!=0)
    {
      button_status = short_press;
      button_time_ack = button_time[5];
      if (stop_flag==1)
      {
    	  sprintf((char *)&MSG, "Button pressed\r\n\n");
    	  HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
    	  stop_flag=0;
      }
    }
  }
}

void toggle(int nb)
{
  int i = 0;
  HAL_Delay(300);
  while (i<nb)
  {
    HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    i++;
  }
  HAL_Delay(300);
}

/**
  * @brief  System Clock Speed decrease
  *         The system Clock source is shifted from PLL to MSI
  *         while at the same time, MSI range is set to RCC_MSIRANGE_1
  *         to go down to 200 KHz
  * @param  None
  * @retval None
  */
void SystemClock_Decrease(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

  /* Disable PLL to reduce power consumption since MSI is used from that point */
  /* Change MSI frequency */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select MSI as system clock source */
  /* Note: Keep AHB and APB prescaler settings from previous structure initialization */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

static void OnGpsEvent(void *context)
{
}

static void OnGpsledEvent(void *context)
{
  uint16_t period;

  if(gpsFix == 0)
  {
    if (900 < (100*tnl_tmp_gps_data.sat_in_view))
      period = 100;
    else
      period = 1000-(100*(uint16_t)tnl_tmp_gps_data.sat_in_view);
    UTIL_TIMER_SetPeriod(&timerGPSLed,period);
    UTIL_TIMER_Start(&timerGPSLed);
    HAL_GPIO_TogglePin(LED_RED1_GPIO_Port, LED_RED1_Pin);
  }
  else
  {
    UTIL_TIMER_Stop(&timerGPSLed);
    HAL_GPIO_WritePin(LED_RED1_GPIO_Port, LED_RED1_Pin, GPIO_PIN_RESET);
  }
}

static void OnFlashEvent(void *context)
{
  UTIL_TIMER_Start(&timerFlashOff);
  //Flash(1);
}

static void OnFlashEventOff(void *context)
{
  //Flash(0);
  if (Flash_cnt == 3)
  {
    Flash_cnt = 0;
    UTIL_TIMER_SetPeriod(&timerFlash,5000);
    UTIL_TIMER_Start(&timerFlash);
  }
  else if (Flash_cnt == 1)
  {
    UTIL_TIMER_SetPeriod(&timerFlash,250);
    UTIL_TIMER_Start(&timerFlash);
  }
  else
  {
    UTIL_TIMER_Start(&timerFlash);
  }
  Flash_cnt++;
}

static void OnledEvent(void *context)
{
  if (battery_level>=0x5)
  {
    HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_RESET);
  }
  else if (battery_level>0x0)
  {
      HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_SET);
  }

  if (blink_power_led != 0)
    {
      UTIL_TIMER_SetPeriod(&timerLed,600);
      UTIL_TIMER_SetPeriod(&timerLedOff,300);
    }
  else
    {
    UTIL_TIMER_SetPeriod(&timerLed,5000);
    UTIL_TIMER_SetPeriod(&timerLedOff,250);
    }

  UTIL_TIMER_Start(&timerLedOff);
}
/* USER CODE END PrFD_LedEvents */

static void OnledOffEvent(void *context)
{
  if (blink_power_led!=0)
  {
    blink_power_led--;
    if (battery_level>=0x5)
    {
      HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_RESET);
    }
    else if (battery_level>0x0)
    {
      HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_RESET);
    }
  }
  else
  {
    HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_RESET);
  }
}

void BATMSUpdate(void)
{
  uint16_t ADC_value = 0;

  BSP_BC_CmdSend(BATMS_ON);
  MX_ADC_Init();
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_NONE;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

  HAL_Delay(10);
  // Calibrate The ADC On Power-Up For Better Accuracy
//  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 100);
  ADC_value = HAL_ADC_GetValue(&hadc)*118/100;//mV

//  https://lygte-info.dk/review/batteries2012/CommonSmallcomparator.php
//  soshine 16340 CR123 @ .1mA
//  10%  = 3.45 V
//  20%  = 3.55 V
//  30%  = 3.58 V
//  40%  = 3.63 V
//  50%  = 3.68 V
//  60%  = 3.76 V
//  70%  = 3.85 V
//  80%  = 3.95 V
//  90%  = 4.05 V
//  100% = 4.2 V

  if(ADC_value>4050)
    {battery_level = 0x7;} // >90%
  else if(ADC_value>3950)
    {battery_level = 0x6;} // >80%
  else if(ADC_value>3850)
    {battery_level = 0x5;} // >70%
  else if(ADC_value>3760)
    {battery_level = 0x4;} // >60%
  else if(ADC_value>3680)
    {battery_level = 0x3;} // >50%
  else if(ADC_value>3863)
    {battery_level = 0x2;} // >40%
  else if(ADC_value>3580)
    {battery_level = 0x1;} // >30%
  else
    {battery_level = 0x0;} // <30%

  BSP_BC_CmdSend(BATMS_OFF);
  HAL_ADC_DeInit(&hadc);
}

void production_print(uint8_t cnt)
{
	if (start_flag==1)
	{
		flag_test=1;
		loop_flag=0;
		sprintf((char *)&MSG, "<> Test %d : %s\r\n\n", cnt+1,list_checking[cnt]);
		HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		if (cnt<5){
			sprintf((char *)&MSG, "\t-> Check if led %s is on\r\n\n",list_checking[cnt]);
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		}
	}
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
	//HAL_UARTEx_ReceiveToIdle_IT(&huart2, aGNSSRxBuffer, sizeof(aGNSSRxBuffer));
}



void Off_mode(void)
{
	BSP_BC_Init();
	BSP_BC_CmdSend(SHIPPING_MODE);
}


void Safe_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_FLUSH_DRREGISTER(huart);

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(huart);
    }
    SET_BIT(huart2.Instance->CR1, USART_CR1_RE);

    HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_SET);

    HAL_UART_DMAStop(huart);  // Important si le DMA était déjà actif

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, size) != HAL_OK)
    {
        Error_Handler(); // ou retourne une erreur selon ton style
    }

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
 {

    //  /* USER CODE BEGIN 1 */
    __IO uint32_t *regaddr;
    uint32_t regval;

    UTIL_ADV_TRACE_SetVerboseLevel(3); // BH

    //  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();


    /* Configure the system clock */
    SystemClock_Config();
    //configure the clock to use when exiting low power mode
    //  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI); //to check. MSI set in sys_app.c
    /* Initialize all configured peripherals */
    Disable_EXTI10_IRQ();
    MX_PREPARE_GPIO();  //configure GPIO


    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    MX_LoRaWAN_Init();

    BSP_BC_CmdSend(MEM_OFF); //Keep PWR_MEM ON.  spi bus is shared with accelerometer whose power cannot be turned off. Only 1µA in power down
    BSP_BC_CmdSend(BLE_OFF); //turn off PWR_BLE
    GNSS_Stop();

    //Off_mode(); //Stay here if not plug by back port

    FLASH->CR |= 0x00040000;
    MX_DMA_Init();
  	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_TIM16_Init();
#if HARDWARE_VERSION >= 14
	MX_TIM1_Init();
#else
	MX_TIM17_Init();
#endif
	//Wake up music
	Play_Sound(1174, 150);
	HAL_Delay(200);
	Play_Sound(1567, 150);

	HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(VIBRATION_GPIO_Port, VIBRATION_Pin, GPIO_PIN_SET);

	sprintf((char *)&MSG, "Welcome to LYNKX production tests a\r\n\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);

	sprintf((char *)&MSG, "Initializing...\r\n\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);

    //configure PMIC
    BSP_BC_Init();

	Safe_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, DMA_RX_BUFFER_SIZE); // Lance réception DMA circulaire
	//GNSS_Start(1);
	GNSS_Start(0);

	MX_SPI1_Init();
	MX_I2C2_Init();

	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	MX_LoRaWAN_Init2();

	MX_CRC8_Init();




	MX_BlueNRG_2_Init();

	sprintf((char *)&MSG, "Device initialized\r\n\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);

	HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_RESET);


	while(1)
	{
		while(start_flag==1)
		{
			sprintf((char *)&MSG, "\t-> Start of tests procedure\r\n\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);

			Play_Start_Music();
			production_print(0);
			Test_LED_GREEN1();
			production_print(1);
			Test_LED_GREEN2();
			production_print(2);
			Test_LED_RED1();
			production_print(3);
			Test_LED_RED2();
			production_print(4);
			Test_LED_Flash();
			production_print(5);
			Test_Sound();
			production_print(6);
			Test_Pressure();
			production_print(7);
			Test_ACC();
			production_print(8);
			Test_BC_State();
			production_print(9);
			Test_BLE();
			production_print(10);
			Test_LoRa();
			production_print(11);
			Test_GNSS();
			production_print(12);
			Test_Flash();
			production_print(13);
			Test_Emergency();


			//End
			if(stop_flag==1) //End of tests
			{
				sprintf((char *)&MSG, "\nProduction tests completed\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
				Play_Stop_Music();
			}
			else //Tests abort
			{
				sprintf((char *)&MSG, "\nProduction tests abort\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			}
			while(stop_flag == 1)
			{
				Check_Button(0);
			}
			start_flag=0;
		}
		HAL_GPIO_WritePin(LED_RED1_GPIO_Port, LED_RED1_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_RED1_GPIO_Port, LED_RED1_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_RED1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_RED1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
} //end main

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

	  /* DMA controller clock enable */
	  __HAL_RCC_DMAMUX1_CLK_ENABLE();
	  __HAL_RCC_DMA1_CLK_ENABLE();

	  /* DMA interrupt init */
	  /* DMA1_Channel1_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);


}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) //original
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 6;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the HSE Prescaler
  */
  __HAL_RCC_HSE_DIV2_ENABLE();
}

void SystemClock_Config3(void) //switch clock source to HSI oscillator
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
			       |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
			       |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
      Error_Handler();
  }
}

void SystemClock_Config4(void) // reconfigure HSE and PLL before switching clock source to HSE/PLL oscillator
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   /** Initializes the CPU, AHB and APB buses clocks
   */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                               |RCC_OSCILLATORTYPE_LSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.LSEState = RCC_LSE_ON;
   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
   RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV2;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
   RCC_OscInitStruct.PLL.PLLN = 6;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
     Error_Handler();
   }

   /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
   */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                               |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                               |RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
   RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
   {
     Error_Handler();
   }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  ADC_ChannelConfTypeDef sConfig = {0};
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 0;
  sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
}

/**
  * @brief CRC 8 Initialization Function
  * @param None
  * @retval None
  */
void MX_CRC8_Init(void)
{
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 7;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.InitValue = 0;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC 32 Initialization Function
  * @param None
  * @retval None
  */
void MX_CRC32_Init(void)
{
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_32B;
  hcrc.Init.InitValue = 0xFFFFFFFF;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0xA010B3FF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
{
  RTC_AlarmTypeDef sAlarm = {0};

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_PREDIV_A;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_ONLY;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RTC and set the Time and Date
  */
  if (HAL_RTCEx_SetSSRU_IT(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.BinaryAutoClr = RTC_ALARMSUBSECONDBIN_AUTOCLR_NO;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDBINMASK_NONE;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm B
  */
  RTC_AlarmTypeDef sAlarmB = {0};
  sAlarmB.BinaryAutoClr = RTC_ALARMSUBSECONDBIN_AUTOCLR_NO;
  sAlarmB.AlarmTime.SubSeconds = 0xFFFFFFFFu - (5 * 32768 / (RTC_PREDIV_A + 1));
  sAlarmB.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarmB.AlarmSubSecondMask = RTC_ALARMSUBSECONDBINMASK_NONE;
  sAlarmB.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarmB, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  //hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Reconfigure_SPI1_For_Flash_or_ACC(void)
{
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;         // CPHA = 0 → Mode 0
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Reconfigure_SPI1_For_BlueNRG(void)
{
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;         // CPHA = 1 → Mode 1
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static enum { SPI_UNKNOWN, SPI_FOR_BLE, SPI_FOR_FLASH_OR_ACC } spi_owner = SPI_UNKNOWN;

void Ensure_SPI_Config_For_BLE(void)
{
  if (spi_owner != SPI_FOR_BLE) {
    Reconfigure_SPI1_For_BlueNRG();
    spi_owner = SPI_FOR_BLE;
  }
}

void Ensure_SPI_Config_For_Flash_or_Acc(void)
{
  if (spi_owner != SPI_FOR_FLASH_OR_ACC) {
	  Reconfigure_SPI1_For_Flash_or_ACC();
    spi_owner = SPI_FOR_FLASH_OR_ACC;
  }
}
/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
void MX_SUBGHZ_Init(void)
{
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = (uint32_t)(((SystemCoreClock) / (10000)) - 1);
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim16, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim16, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

#if HARDWARE_VERSION >= 14
/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

	__HAL_RCC_TIM1_CLK_ENABLE(); // <<< activation de l'horloge TIM1

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  uint32_t uwTimerPeriod = (uint32_t)((SystemCoreClock / 1000) -1 );

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = uwTimerPeriod;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0; // after 2 round, the DMA will change the CCR
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = uwTimerPeriod/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}
#else

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  uint32_t uwTimerPeriod = (uint32_t)((SystemCoreClock / 1000) -1 );

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = uwTimerPeriod;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0; // after 2 round, the DMA will change the CCR
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim17, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = uwTimerPeriod/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim17);

}
#endif

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
#if HARDWARE_VERSION >= 14
  huart2.Init.BaudRate = 115200;
#else
  huart2.Init.BaudRate = 9600;
#endif
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
  HAL_UART_RegisterRxEventCallback(&huart2, HAL_UARTEx_RxEventCallback);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_PREPARE_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 , GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

   //BUTTON
   GPIO_InitStruct.Pin = GPIO_PIN_0 ;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_1 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   //GNSS TXD
   GPIO_InitStruct.Pin = GPIO_PIN_2 ;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   //GNSS RXD
   GPIO_InitStruct.Pin = GPIO_PIN_3 ;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_4 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_5 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_6 ;
   GPIO_InitStruct.Mode = MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_7    ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   GPIO_InitStruct.Pin =  GPIO_PIN_8;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   //TXD UART1
   GPIO_InitStruct.Pin = GPIO_PIN_9 ;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   //Vibration
   GPIO_InitStruct.Pin = GPIO_PIN_11 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_12  ;
   GPIO_InitStruct.Mode = MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_15;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

   GPIO_InitStruct.Pin = GPIO_PIN_0 ;
   GPIO_InitStruct.Mode = MODE_OUTPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if HARDWARE_VERSION >= 14
   //BLE IRQ pin B4
   GPIO_InitStruct.Pin = GPIO_PIN_4 ;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#else
   //BLE IRQ pin B1
   GPIO_InitStruct.Pin = GPIO_PIN_1 ;
   GPIO_InitStruct.Mode = MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
   GPIO_InitStruct.Pin = GPIO_PIN_2 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   //ACC_INT1
   GPIO_InitStruct.Pin = GPIO_PIN_3 ;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


   GPIO_InitStruct.Pin = GPIO_PIN_5 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_6 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   //BUZZER
#if HARDWARE_VERSION >= 14
   //TIM1_CH3 PWM -> PA10
   GPIO_InitStruct.Pin = GPIO_PIN_10 ;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#else
   //TIM17 PWM -> PB7
   GPIO_InitStruct.Pin = GPIO_PIN_7 ;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.Alternate = GPIO_AF14_TIM17;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

   GPIO_InitStruct.Pin = GPIO_PIN_8    ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   //LED_RED1
   GPIO_InitStruct.Pin = GPIO_PIN_9 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if HARDWARE_VERSION >= 14
   //RX UART1
   GPIO_InitStruct.Pin = GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#else
   GPIO_InitStruct.Pin = GPIO_PIN_10 ;
   GPIO_InitStruct.Mode = MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
   //CHG_FLAG
   GPIO_InitStruct.Pin = GPIO_PIN_10;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


   GPIO_InitStruct.Pin = GPIO_PIN_11 ;
   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_12 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_13 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_14;
   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_15;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 , GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

   GPIO_InitStruct.Pin = GPIO_PIN_0 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_1 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_2 ;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_3 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   //ACC_INT2
   GPIO_InitStruct.Pin = GPIO_PIN_4 ;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_5 ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = GPIO_PIN_6   ;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   //Emergency
   GPIO_InitStruct.Pin = GPIO_PIN_13 ;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0x00);
   HAL_NVIC_EnableIRQ(EXTI0_IRQn);
   HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0x00);
   HAL_NVIC_EnableIRQ(EXTI3_IRQn);
   HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0x00);
   HAL_NVIC_EnableIRQ(EXTI4_IRQn);
   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0x00);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


/* USER CODE BEGIN 4 */


/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	int i;
	uint32_t button_temp;


	//Button
	if (GPIO_Pin==BUTTON_Pin)
	{
		button_temp = HAL_GetTick();
		if (button_temp - button_time[5] > 20 )//debounce
		{
		  for (i=0;i<5;i++)
		  {
			  button_time[i] = button_time[i+1];
		  }
		  button_time[5] = button_temp;
		}
	}

	//Emergency
	if (GPIO_Pin == EMERGENCY_Pin)
	{
		if (test_emergency==1)
		{
			emergency_flag = !emergency_flag;
		}
	}

	//Flag_CHG
	if (GPIO_Pin==GPIO_PIN_10)
	{
		BC_ChgPinFreqGet();
	}

}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  char strbuf[128];
  char str_test[15];
  struct tm heure;
  static char led = 0;

  //UTIL_ADV_TRACE_SetVerboseLevel(3); // BH

  if (huart->Instance == USART2) // GNSS
  {
    // DMA copy
    uint16_t dma_pos = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    uint16_t data_len;

    if (dma_pos >= dma_old_pos)
    {
      data_len = dma_pos - dma_old_pos;
      if (gnss_accum_len + data_len < GNSS_ACCUM_BUFFER_SIZE)
        memcpy(&gnss_accum_buffer[gnss_accum_len], &dma_rx_buffer[dma_old_pos], data_len);
    }
    else
    {
      data_len = DMA_RX_BUFFER_SIZE - dma_old_pos;
      if (gnss_accum_len + data_len < GNSS_ACCUM_BUFFER_SIZE)
        memcpy(&gnss_accum_buffer[gnss_accum_len], &dma_rx_buffer[dma_old_pos], data_len);
      gnss_accum_len += data_len;

      if (gnss_accum_len + dma_pos < GNSS_ACCUM_BUFFER_SIZE)
        memcpy(&gnss_accum_buffer[gnss_accum_len], &dma_rx_buffer[0], dma_pos);

      data_len += dma_pos;
    }

    gnss_accum_len += data_len;
    dma_old_pos = dma_pos;

    // Parsing ligne par ligne
    tnl_tmp_gps_data.sat_in_view = 0;
    while (1)
    {
      char *start = (char *)gnss_accum_buffer;
      char *newline = memchr(start, '\n', gnss_accum_len);
      if (!newline)
    	  break;

      size_t line_len = newline - start + 1;
      if (line_len >= sizeof(aGNSSRxBuffer)) line_len = sizeof(aGNSSRxBuffer) - 1;

      memcpy(aGNSSRxBuffer, gnss_accum_buffer, line_len);
      aGNSSRxBuffer[line_len] = '\0';

      // Décale le buffer
      memmove(gnss_accum_buffer, newline + 1, gnss_accum_len - line_len);
      gnss_accum_len -= line_len;

      APP_LOG(TS_ON, VLEVEL_H, "%s", aGNSSRxBuffer);

      memcpy(str_test, aGNSSRxBuffer, 1);
      str_test[1] = '\0';

      //tnl_tmp_gps_data.sat_in_view = TRK_SearchSatellite(aGNSSRxBuffer);
      //if (!strcmp(str_test, "$GPRMC") || !strcmp(str_test, "$GNRMC"))
      if (!strcmp(str_test, "$"))
      {
        TRK_GpsReadProcessing(aGNSSRxBuffer, strlen((char *)aGNSSRxBuffer));

        bool date_valid = (tnl_tmp_gps_data.year > 21) && (tnl_tmp_gps_data.year < 42);
        bool resync_needed = (SysTimeGetMcuTime().Seconds - SysTimeUpdateDate) > 604800;

        if (tnl_tmp_gps_data.lat != 0)
        {
          sprintf(strbuf, "LAT:%ld %c LONG:%ld %c\r\n",
                  tnl_tmp_gps_data.latitude_fixed, tnl_tmp_gps_data.lat,
                  tnl_tmp_gps_data.longitude_fixed, tnl_tmp_gps_data.lon);
        }
        else
        {
          sprintf(strbuf, "LAT:%ld %d LONG:%ld %d\r\n",
                  tnl_tmp_gps_data.latitude_fixed, tnl_tmp_gps_data.lat,
                  tnl_tmp_gps_data.longitude_fixed, tnl_tmp_gps_data.lon);
        }

        if (date_valid && (SysTimeUpdateDate == 0 || resync_needed))
        {
          heure.tm_sec = tnl_tmp_gps_data.seconds;
          heure.tm_min = tnl_tmp_gps_data.minute;
          heure.tm_hour = tnl_tmp_gps_data.hour;
          heure.tm_mday = tnl_tmp_gps_data.day;
          heure.tm_mon = tnl_tmp_gps_data.month - 1;
          heure.tm_year = tnl_tmp_gps_data.year + 100;

          SysTime_t sysTimeGNSS = {0};
          sysTimeGNSS.Seconds = SysTimeMkTime(&heure);
          sysTimeGNSS.SubSeconds = tnl_tmp_gps_data.milliseconds;
          tnl_tmp_gps_data.timestamp = sysTimeGNSS.Seconds;

          SysTimeSet(sysTimeGNSS);
          OnSysTimeUpdate();
        }

        if (tnl_tmp_gps_data.fix == 1 && gpsFix == 0)
        {
          gpsFix = 1;
          if (gps_lost_cnt == 10)
          {
            // Play_Sound(2400, 100);
          }
          gps_lost_cnt = 0;
        }
        else if (tnl_tmp_gps_data.fix == 0 && gps_lost_cnt < 9)
        {
          gps_lost_cnt++;
        }
        else if (gps_lost_cnt == 9)
        {
          // Play_Sound(2400, 100);
          gps_lost_cnt = 10;
          gpsFix = 0;
        }
      }
    }
  }
  else if (huart->Instance == USART1) // backport
  {
    if (aCOMRxBuffer[0] == 'T') flag_test = 0;
    if (aCOMRxBuffer[0] == 'S') start_flag = 0;
    if (aCOMRxBuffer[0] == 'R') start_flag = 1;

    if (aCOMRxBuffer[0] == 'P' && aCOMRxBuffer[1] == 'M' &&
        aCOMRxBuffer[2] == 'T' && aCOMRxBuffer[3] == 'K')
    {
      sprintf(strbuf, "$%s*%02X\r\n", aCOMRxBuffer, calc_nmea_checksum((char *)aCOMRxBuffer));
      HAL_UART_Transmit(&huart2, (uint8_t *)strbuf, strlen((char *)strbuf), 200);
    }
    else if (!strcmp(aCOMRxBuffer, "0")) UTIL_ADV_TRACE_SetVerboseLevel(0);
    else if (!strcmp(aCOMRxBuffer, "1")) UTIL_ADV_TRACE_SetVerboseLevel(1);
    else if (!strcmp(aCOMRxBuffer, "2")) UTIL_ADV_TRACE_SetVerboseLevel(2);
    else if (!strcmp(aCOMRxBuffer, "3")) UTIL_ADV_TRACE_SetVerboseLevel(3);

    memset(aCOMRxBuffer, 0x0, 1000);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM17)//STBC02_USED_TIM)
  {
    BC_CmdMng();
  }
  if (htim->Instance == TIM16)
  {
    HAL_TIM_Base_Stop(&htim16);
#if HARDWARE_VERSION >= 14
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    //HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
#else
    HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
#endif
    HAL_GPIO_WritePin(BUZZER_EN_GPIO_Port, BUZZER_EN_Pin, GPIO_PIN_RESET);
  }
}


/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
