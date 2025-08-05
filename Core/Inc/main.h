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
#include "stm32wlxx_hal.h"
#include "hci_tl_interface.h"
#include "stm32wlxx_nucleo.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "STWIN_bc.h"
#include "gps.h"
#include "gps.h"
#include "prod.h"
/* USER CODE END Includes */

#define HARDWARE_VERSION 14

#define LORA_EN 1
#define BLE_EN 1

#define FIMRWARE_VERSION_MAJOR 0
#define FIMRWARE_VERSION_MINOR 7
#define LEAP_SECONDS 18


//#define AWAKE_DURATION 864000   // Go to sleep 10 days after last activity
#define AWAKE_DURATION 259200   // Go to sleep 3 days after last activity
//#define AWAKE_DURATION 86400    // Go to sleep 1 day after last activity
//#define AWAKE_DURATION 3600     // Go to sleep 1 hour after last activity
//#define AWAKE_DURATION 30       // Go to sleep XX sec after last activity


/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t MSG[100];
extern uint8_t emergency_mode_en;
extern uint32_t SysTimeUpdateDate;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t next_power_state;
extern uint8_t battery_level;
extern uint8_t GNSS_On_request;
extern SPI_HandleTypeDef hspi1; // SPI communication port


// config which is never updated
typedef struct
{
  uint8_t ProductReference[16];
  uint8_t HardwareVersion_Major;
  uint8_t HardwareVersion_Minor;
  uint8_t MACAddress[8];
  uint8_t RFU0;
  uint8_t RFU1;
  uint8_t RFU2;
  uint8_t RFU3;
  uint8_t RFU4;
} LYNKXConfig_t;


// config which is updated on OTA update
typedef struct
{
  uint8_t  CurrentFirmwareVersion_Major;
  uint8_t  CurrentFirmwareVersion_Minor;
  uint8_t  NewFirmwareVersion_Major;
  uint8_t  NewFirmwareVersion_Minor;
  uint32_t NewFirmwareCRC;
  uint32_t NewFirmwareSize;
  uint8_t  RFU0;  // RFU
  uint8_t  RFU1;  // RFU
  uint8_t  RFU2;  // RFU
  uint8_t  RFU3;  // RFU
  } BootLoaderConfig_t;

void MX_RTC_Init(void);
void MX_SUBGHZ_Init(void);
void MX_USART1_UART_Init(void);
void BATMSUpdate(void);
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)

#define SCK_Pin GPIO_PIN_1
#define SCK_GPIO_Port GPIOA

#if HARDWARE_VERSION >= 14
	#define RXD_Pin GPIO_PIN_7
	#define RXD_GPIO_Port GPIOB
	#define BUZZER_Pin GPIO_PIN_10
	#define BUZZER_GPIO_Port GPIOA
	#define BLE_IRQ_Pin GPIO_PIN_4
	#define BLE_IRQ_GPIO_Port GPIOB
	#define BLE_IRQ_EXTI_IRQn EXTI4_IRQn
#else
	#define RXD_Pin GPIO_PIN_10
	#define RXD_GPIO_Port GPIOA
	#define BUZZER_Pin GPIO_PIN_7
	#define BUZZER_GPIO_Port GPIOB
	#define BLE_MISO_Pin GPIO_PIN_2
	#define BLE_MISO_GPIO_Port GPIOC
	#define BLE_MOSI_Pin GPIO_PIN_3
	#define BLE_MOSI_GPIO_Port GPIOC
	#define BLE_SCK_Pin GPIO_PIN_13
	#define BLE_SCK_GPIO_Port GPIOB
	#define BLE_IRQ_Pin GPIO_PIN_1
	#define BLE_IRQ_GPIO_Port GPIOB
	#define BLE_IRQ_EXTI_IRQn EXTI1_IRQn
#endif

#define EMERGENCY_Pin GPIO_PIN_13
#define EMERGENCY_GPIO_Port GPIOC
#define EMERGENCY_EXTI_IRQn EXTI15_10_IRQn
#define PWR_SW_Pin GPIO_PIN_5
#define PWR_SW_GPIO_Port GPIOB
#define BSP_BUTTON_EXTI_IRQn EXTI0_IRQn
#define BLE_CSN_Pin GPIO_PIN_2
#define BLE_CSN_GPIO_Port GPIOB
#define BLE_NRST_Pin GPIO_PIN_1
#define BLE_NRST_GPIO_Port GPIOC
#define BUZZER_EN_Pin GPIO_PIN_6
#define BUZZER_EN_GPIO_Port GPIOB
#define TXD_Pin GPIO_PIN_9
#define TXD_GPIO_Port GPIOA
#define SW_SEL_cmd_Pin GPIO_PIN_12
#define SW_SEL_cmd_GPIO_Port GPIOB

#define ACC_INT1_Pin GPIO_PIN_3
#define ACC_INT1_GPIO_Port GPIOB
#define ACC_INT2_Pin GPIO_PIN_4
#define ACC_INT2_GPIO_Port GPIOC
#define ACC_INT1_EXTI_IRQn EXTI3_IRQn

#define WP_FLASH_Pin GPIO_PIN_6
#define WP_FLASH_GPIO_Port GPIOC
#define CS_ACC_Pin GPIO_PIN_8
#define CS_ACC_GPIO_Port GPIOA
#define USB_DETECTED_Pin GPIO_PIN_11
#define USB_DETECTED_GPIO_Port GPIOB
#define RXD_GNSS_Pin GPIO_PIN_3
#define RXD_GNSS_GPIO_Port GPIOA
#define TXD_GNSS_Pin GPIO_PIN_2
#define TXD_GNSS_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define CHG_FLAG_Pin GPIO_PIN_10
#define CHG_FLAG_GPIO_Port GPIOB
#define CS_FLASH_Pin GPIO_PIN_5
#define CS_FLASH_GPIO_Port GPIOA
#define VIBRATION_Pin GPIO_PIN_11
#define VIBRATION_GPIO_Port GPIOA
#define PPS_GNSS_Pin GPIO_PIN_12
#define PPS_GNSS_GPIO_Port GPIOA
#define LED_GREEN1_Pin GPIO_PIN_0
#define LED_GREEN1_GPIO_Port GPIOB
#define LED_GREEN2_Pin GPIO_PIN_5
#define LED_GREEN2_GPIO_Port GPIOC
#define LED_RED1_Pin GPIO_PIN_9
#define LED_RED1_GPIO_Port GPIOB
#define LED_RED2_Pin GPIO_PIN_0
#define LED_RED2_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA

#ifndef true
#define true (1)
#endif

#ifndef false
#define false (0)
#endif

#define BLINK_ON 100
#define BLINK_OFF 1000

#define TIMESTAMP                    (SysTimeUpdateDate==0) ? SysTimeGetMcuTime().Seconds : SysTimeGet().Seconds - LEAP_SECONDS
#define TIMESTAMP_ms                   (SysTimeUpdateDate==0) ? SysTimeGetMcuTime().SubSeconds : SysTimeGet().SubSeconds/1024*1000

#define MAX(X, Y) (((X) < (Y)) ? (Y) : (X))

#define MTKEPO_SV_NUMBER 32
#define MTKEPO_RECORD_SIZE 72
#define MTKEPO_SEGMENT_NUM (30 * 4)

//INTERNAL MEMORY
#define APP_START                        ((uint32_t)0x08006000UL)   //Origin + Bootloader size (20kB)
#define INT_MEMORY_END                   ((uint32_t)0x0003FFFFUL)

//EXTERNAL MEMORY
#define FIRMWARE_OTA_BASE_ADDRESS        ((uint32_t)0x00000000UL)
#define LYNKX_CFG_ADDRESS                ((uint32_t)0x00040000UL)
#define BOOTLOADER_CFG_ADDRESS           ((uint32_t)0x00041000UL)
#define LORAWAN_NVM_BASE_ADDRESS         ((uint32_t)0x00042000UL)
#define FIRMWARE_FR_BASE_ADDRESS         ((uint32_t)0x001C0000UL)  // block 28,29,30,31


#define POWER_ON 		0
#define SLEEP_UNTIL_MOVE_ACK  	1
#define SLEEP_UNTIL_PUSH 	2
#define SLEEP_UNTIL_PUSH_ACK 	3
#define POWER_OFF   		4
#define POWER_OFF_ACK  		5
#define RESTART 		6
#define RESTART_ACK 		6

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
