/**
******************************************************************************
* @file    STWIN_bc.h
* @author  SRA
* @version v1.4.6
* @date    22-Oct-2021
* @brief   This file provides code for the configuration of the STBC02
*          battery charger
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the 
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STWIN_BC_H
#define STWIN_BC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "stm32wlxx_hal_tim.h"



/** @addtogroup BSP
 * @{
 */

/** @addtogroup STWIN 
 * @{
 */

/** @addtogroup STWIN_BATTERY_CHARGER 
 * @{
 */

/** @addtogroup STWIN_BATTERY_CHARGER_Public_Defines
 * @{
 */


#define USE_BC_TIM_IRQ_CALLBACK         1U
#define USE_BC_GPIO_IRQ_HANDLER         1U
#define USE_BC_GPIO_IRQ_CALLBACK        1U

#define MAX_VOLTAGE 4225
#define MIN_VOLTAGE 3250
   
#define STBC02_GetTick()                      HAL_GetTick()  //!< Get SysTick macro
   
/*! \name Timer Peripheral
*/
//@{
#define STBC02_USED_TIM_PERIOD                (float)5e-6 // s

#if HARDWARE_VERSION >= 14
#define STBC02_USED_TIM                       TIM17
#define STBC02_USED_TIM_CLK_ENABLE()          __HAL_RCC_TIM17_CLK_ENABLE()
#define STBC02_USED_TIM_CLK_DISABLE()         __HAL_RCC_TIM17_CLK_DISABLE()
#define STBC02_USED_TIM_CLKFreq               (HAL_RCC_GetPCLK1Freq() * (((READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) < 4U) ? 1U : 2U))
#define STBC02_USED_TIM_IRQn                  TIM17_IRQn
#define STBC02_USED_TIM_IRQHandler            TIM17_IRQHandler
#else
#define STBC02_USED_TIM                       TIM1
#define STBC02_USED_TIM_CLK_ENABLE()          __HAL_RCC_TIM1_CLK_ENABLE()
#define STBC02_USED_TIM_CLK_DISABLE()         __HAL_RCC_TIM1_CLK_DISABLE()
#define STBC02_USED_TIM_CLKFreq               (HAL_RCC_GetPCLK1Freq() * (((READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) < 4U) ? 1U : 2U))
#define STBC02_USED_TIM_IRQn                  TIM1_UP_IRQn
#define STBC02_USED_TIM_IRQHandler            TIM1_UP_IRQHandler
#endif

#ifndef STBC02_USED_TIM_IRQ_PP
#define STBC02_USED_TIM_IRQ_PP                3
#endif

#ifndef STBC02_USED_TIM_IRQ_SP 
#define STBC02_USED_TIM_IRQ_SP                0
#endif
//@}

/*! \name ADC Peripheral
*/
//@{
#define STBC02_USED_ADC                       ADC1
#define STBC02_USED_ADC_CLK_ENABLE()          __HAL_RCC_ADC_CLK_ENABLE()
#define STBC02_USED_ADC_CLK_DISABLE()         __HAL_RCC_ADC_CLK_DISABLE()
#define STBC02_USED_ADC_CHANNEL               ADC_CHANNEL_13
//@}
   
/*! \name GPIO for STBC02 Load switch selection input
*/
//@{
#define STBC02_SW_SEL_PIN                     GPIO_PIN_12
//#define STBC02_SW_SEL_PIN                     GPIO_PIN_15 //CMO for test purpose on LED at slow speed
#define STBC02_SW_SEL_GPIO_PORT               GPIOB
#define STBC02_SW_SEL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define STBC02_SW_SEL_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOF_CLK_DISABLE()

//@}

/*! \name GPIO for STBC02 Charging/fault flag
*/
//@{
#define STBC02_CHG_PIN                        GPIO_PIN_10
#define STBC02_CHG_GPIO_PORT                  GPIOB
#define STBC02_CHG_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define STBC02_CHG_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOB_CLK_DISABLE()

//@}

/*! \name GPIO for STBC02 Battery voltage measurement
*/
//@{
#define STBC02_BATMS_PIN                      GPIO_PIN_14
#define STBC02_BATMS_GPIO_PORT                GPIOB
#define STBC02_BATMS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define STBC02_BATMS_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE()
#define STBC02_BATMS_GPIO_MODE                GPIO_MODE_ANALOG_ADC_CONTROL
//@}

/**
 * @}
 */

/** @addtogroup STWIN_BATTERY_CHARGER_Public_Types
 * @{
 */

/**
  * @brief Device state
  */
typedef enum
{
  NotValidInput = 0,
  ValidInput,
  VbatLow,
  EndOfCharge,
  ChargingPhase,
  OverchargeFault,
  ChargingTimeout,
  BatteryVoltageBelowVpre,
  ChargingThermalLimitation,
  BatteryTemperatureFault
} stbc02_ChgState_TypeDef;

/**
  * @brief SW selection pulse number
  */
typedef enum
{
//  SENSOR_OFF              = 1,
//  SENSOR_ON               = 2,
//  GNSS_OFF                = 3,
//  GNSS_ON                 = 4,
//  MEM_OFF                 = 5,
//  MEM_ON                  = 6,
//  BLE_OFF                 = 7,
//  BLE_ON                  = 8,
  GNSS_OFF                 = 1,
  GNSS_ON                  = 2,
  GNSS_OFF2              = 3,
  GNSS_ON2               = 4,
  BLE_OFF                = 5,
  BLE_ON                 = 6,
  MEM_OFF                 = 7,
  MEM_ON                  = 8,
  BATMS_OFF               = 9,
  BATMS_ON                = 10,
  IEND_OFF                = 11,
  IEND_5_PC_IFAST         = 12,
  IEND_2_5_PC_IFAST       = 13,
  IBAT_OCP_900_mA         = 14,
  IBAT_OCP_450_mA         = 15,
  IBAT_OCP_250_mA         = 16,
  IBAT_OCP_100_mA         = 17,
  VFLOAT_ADJ_OFF          = 18,
  VFLOAT_ADJ_PLUS_50_mV   = 19,
  VFLOAT_ADJ_PLUS_100_mV  = 20,
  VFLOAT_ADJ_PLUS_150_mV  = 21,
  VFLOAT_ADJ_PLUS_200_mV  = 22,
  SHIPPING_MODE           = 23,
  AUTORECH_OFF            = 24,
  AUTORECH_ON             = 25,
  WATCHDOG_OFF            = 26,
  WATCHDOG_ON             = 27,
  IFAST_IPRE_50_PC_OFF    = 28,
  IFAST_IPRE_50_PC_ON     = 29
} stbc02_SwCmd_TypeDef;

/**
  * @brief SW pulse state
  */
typedef enum
{
  idle,
  start,
  pulse_l,
  pulse_h,
  stop_l,
  stop_h,
  wait
} stbc02_SwState_TypeDef;

/**
  * @brief Device state structure as name and nominal frequency
  */
typedef struct
{
  char *name;
  float freq;
} stbc02_ChgStateNameAndFreq_TypeDef;

/**
 * @}
 */

/** @addtogroup STWIN_BATTERY_CHARGER_Public_Variables
 * @{
 */
typedef struct
  {
    stbc02_ChgState_TypeDef Id;
    uint8_t Name[32];
  } stbc02_State_TypeDef;

extern TIM_HandleTypeDef hstbc02_UsedTim;

/**
 * @}
 */

/* Public Function_Prototypes -----------------------------------------------*/
int32_t BSP_BC_BatMS_Init(void);
int32_t BSP_BC_BatMS_DeInit(void);
void BSP_BC_Init(void);
void BSP_BC_Chrg_Init(void);
int32_t BSP_BC_CmdSend(stbc02_SwCmd_TypeDef stbc02_SwCmd);
void Print_BC_State(stbc02_State_TypeDef *BC_State);
void BSP_BC_ChgPinHasToggled(void);
void BC_ChgPinFreqGet(void);

void BSP_BC_GetState(stbc02_State_TypeDef *BC_State);
void STBC02_CHG_EXTI_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);

int32_t BSP_BC_GetVoltage(uint32_t *mV);
int32_t BSP_BC_GetVoltageAndLevel(uint32_t *mV,uint32_t *BatteryLevel);
void STBC02_CHG_PIN_Callback(void);

void BC_CmdMng(void);
void TIM2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

void Enable_EXTI10_IRQ(void);
void Disable_EXTI10_IRQ(void);
void Get_BC_State(void);
void Print_BC_State(stbc02_State_TypeDef* BC_State);


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif
#endif /* STWIN_BC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
