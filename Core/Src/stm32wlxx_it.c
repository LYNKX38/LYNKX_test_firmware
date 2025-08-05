/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32wlxx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
extern SUBGHZ_HandleTypeDef hsubghz;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_usart2_rx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WLxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wlxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts.
  */
void TAMP_STAMP_LSECSS_SSRU_IRQHandler(void)
{
  /* USER CODE BEGIN TAMP_STAMP_LSECSS_SSRU_IRQn 0 */

  /* USER CODE END TAMP_STAMP_LSECSS_SSRU_IRQn 0 */
  HAL_RTCEx_SSRUIRQHandler(&hrtc);
  /* USER CODE BEGIN TAMP_STAMP_LSECSS_SSRU_IRQn 1 */

  /* USER CODE END TAMP_STAMP_LSECSS_SSRU_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line 0 Interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
#if USE_BUTTON
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_EXTI_IRQHandler(&H_EXTI_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
#endif /* USE_BUTTON */
#if (!USE_BUTTON)
  HAL_GPIO_EXTI_IRQHandler(BUTTON_Pin);
#endif /* !USE_BUTTON */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line 1 Interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_EXTI_IRQHandler(&H_EXTI_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line 3 Interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ACC_INT1_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI Line 4 Interrupt.
  */
void EXTI4_IRQHandler(void)
{
#if HARDWARE_VERSION >= 14

	  /* Vérifier si c’est le BlueNRG2 qui a déclenché l’IT */
	  if (__HAL_GPIO_EXTI_GET_IT(HCI_TL_SPI_EXTI_PIN) != RESET)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(HCI_TL_SPI_EXTI_PIN);
	    hci_tl_lowlevel_isr(); // BlueNRG2
	  }

	  /* Vérifier si c’est l'accéléromètre qui a déclenché l’IT */
	  if (__HAL_GPIO_EXTI_GET_IT(ACC_INT2_Pin) != RESET)
	  {
		__HAL_GPIO_EXTI_CLEAR_IT(ACC_INT2_Pin);
	    HAL_GPIO_EXTI_Callback(ACC_INT2_Pin); // Accéléro
	  }
#else
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ACC_INT2_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
#endif
}


/**
  * @brief This function handles SPI1 Interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 Channel 1 Interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles USART2 Interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */

  HAL_UART_IRQHandler(&huart2);

  /* USER CODE BEGIN USART2_IRQn 1 */
  /* Rien ici pour l'instant */
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART2 Interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI Lines [15:10] Interrupt.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	uint32_t pendingFlags = EXTI->PR1;

	//CHG_FLG
	if (((pendingFlags>>10) & (0x1)) != 0)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	}

	else if(((pendingFlags>>11) & (0x1)) != 0)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
	}
	else if (((pendingFlags>>12) & (0x1)) != 0)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
	}

	else if (((pendingFlags>>13) & (0x1)) != 0)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
	}
	else if (((pendingFlags>>14) & (0x1)) != 0)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
	}
	else if (((pendingFlags>>15) & (0x1)) != 0)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
	}

  /* USER CODE END EXTI15_10_IRQn 0 */

  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles RTC Alarms (A and B) Interrupt.
  */
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */

  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */

  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/**
  * @brief This function handles SUBGHZ Radio Interrupt.
  */
void SUBGHZ_Radio_IRQHandler(void)
{
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 0 */

  /* USER CODE END SUBGHZ_Radio_IRQn 0 */
  HAL_SUBGHZ_IRQHandler(&hsubghz);
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 1 */

  /* USER CODE END SUBGHZ_Radio_IRQn 1 */
}

/**
  * @brief This function handles TIM16 Global Interrupt.
  */
void TIM16_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim16);

}


