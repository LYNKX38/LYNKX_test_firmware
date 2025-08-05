/* Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "LIS2DW12.h"
#include "MS5837.h"
#include "STWIN_bc.h"
#include "LoRaMacInterfaces.h"
#include "LoRaMac.h"
#include "app_bluenrg_2.h"


/* Variables */
uint8_t  data_send[2];
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern I2C_HandleTypeDef hi2c2;
extern uint8_t flag_test;
extern uint8_t loop_flag;
extern uint8_t emergency_flag;
extern uint8_t test_emergency;
extern uint8_t start_flag;
extern uint8_t stop_flag;
extern uint8_t* aGNSSRxBuffer;
extern uint8_t* aCOMRxBuffer;
//extern gps_data_t tnl_tmp_gps_data;
LoRaMacStatus_t LoRa_status;
data_sensor_t data_sensor; // Data structure
data_sensor_t* ptr_data_sensor = &data_sensor; // Pointer to data structure
stbc02_State_TypeDef BC_State;
stbc02_State_TypeDef* ptr_BC_State = &BC_State;
/* Functions */

void Test_LED_GREEN1(void)
{
	if(start_flag==1)
	{
		HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_SET);
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
		HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_RESET);
	}
}
void Test_LED_GREEN2(void)
{
	if(start_flag==1)
	{
		HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_SET);
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
		HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, GPIO_PIN_RESET);
	}
}
void Test_LED_RED1(void)
{
	if(start_flag==1)
	{
		HAL_GPIO_WritePin(LED_RED1_GPIO_Port, LED_RED1_Pin, GPIO_PIN_SET);
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
		HAL_GPIO_WritePin(LED_RED1_GPIO_Port, LED_RED1_Pin, GPIO_PIN_RESET);
	}
}
void Test_LED_RED2(void)
{
	if(start_flag==1)
	{
		HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_SET);
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
		HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, GPIO_PIN_RESET);
	}
}
void Test_LED_Flash(void)
{
	if(start_flag==1)
	{
	//turn torch ON
		data_send[0] = 0x01; //Enable Register;
		data_send[1] = 0x22; //data to write
		HAL_I2C_Master_Transmit(&hi2c2, 0xC8, data_send, 2, 100);
		HAL_Delay(200);
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
		//turn torch OFF
		data_send[0] = 0x01; //Enable Register;
		data_send[1] = 0x20; //data to write
		HAL_I2C_Master_Transmit(&hi2c2, 0xC8, data_send, 2, 100);
		HAL_Delay(200);
	}
}

void Test_Sound(void)
{
	if(start_flag==1)
	{
		sprintf((char *)&MSG, "\t-> Testing sound\r\n\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		if(flag_test==1)
		{
			HAL_Delay(400);
			Play_Sound(1174, 300);
			HAL_Delay(1200);
			Play_Sound(1567, 300);
			HAL_Delay(1200);
			Play_Sound(1880, 300);
			HAL_Delay(10);

			while(flag_test==1 && start_flag==1)
			{
				HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
			}
		}
	}
}


void Test_Pressure(void)
{
	if(start_flag==1)
	{
	Init_pressure_sensor(ptr_data_sensor);
	sprintf((char *)&MSG, "\t-> No barometer on your device\r\n\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
	Print_environment_caracteristics(ptr_data_sensor);
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
	}
}

void Test_ACC(void)
{
	if(start_flag==1)
	{
		uint16_t res_out_x = 0;
		uint8_t acc_flag = 0;
		uint8_t res_who_am_i;
		res_who_am_i = Read_ACC_WHO_AM_I();
		Acc_Conf();
		if (res_who_am_i==0x44)
		{
			sprintf((char *)&MSG, "\t-> Turn the device in side position\r\n\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			while(start_flag==1 && acc_flag==0)
			{
					res_out_x = (Acc_Read_Reg(0x29)<<8)|Acc_Read_Reg(0x28);
					if (res_out_x<50)
							acc_flag = 1;
			}
			sprintf((char *)&MSG, "\t-> Accelerometer OK\r\n\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);

		}
		else
		{
			sprintf((char *)&MSG, "\t-> Accelerometer NOT OK\r\n\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			start_flag=0;
		}
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
	}
}

void Test_BC_State(void)
{
	if(start_flag==1)
	{
		Get_BC_State(); // battery charger state
		Print_BC_State(ptr_BC_State); //Read structure
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
	}
}



void Test_BLE(void)
{
	if(start_flag==1)
	{
		sprintf((char *)&MSG, "\t-> BLE Initializing...\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		MX_BlueNRG_2_Process(); //do BLE things
		sprintf((char *)&MSG, "\t-> BLE Initialized\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
	}

}

void Test_LoRa(void)
{
	if(start_flag==1)
	{
		sprintf((char *)&MSG, "\t-> LoRa Initializing...\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		LoRa_status = SetTxContinuousWave(5,868000000,20);
		sprintf((char *)&MSG, "\t-> LoRa Initialized\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
	}
}


void Test_GNSS(void)
{
	if(start_flag==1)
	{
		sprintf((char *)&MSG, "\t-> Waiting for satellite...\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		while(tnl_tmp_gps_data.sat_in_view==0 && start_flag==1)
		{
			sprintf((char *)&MSG, "\t-> Satellite in view : %d\r\n",tnl_tmp_gps_data.sat_in_view);
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
			//HAL_UARTEx_ReceiveToIdle_IT(&huart2, aGNSSRxBuffer, sizeof(aGNSSRxBuffer));
			HAL_Delay(3000);
		}
		if (start_flag==1)
		{
			sprintf((char *)&MSG, "\t-> Satellite in view : %d\r\n",tnl_tmp_gps_data.sat_in_view);
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			sprintf((char *)&MSG, "\t-> GNSS OK\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		}
	}
}


void Test_Flash(void)
{
	if(start_flag==1)
	{
		sprintf((char *)&MSG, "\t-> Flash memory testing\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);

		W25qxx_EraseBlock(0);
		uint8_t send_data_flash = 0xA5;
		uint8_t receive_data_flash;
		W25qxx_WriteByte(send_data_flash,0x0);
		uint8_t* ptr = &receive_data_flash;
		W25qxx_ReadByte(ptr,0x0);

		if (receive_data_flash==send_data_flash)
		{
			sprintf((char *)&MSG, "\t-> Flash OK\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
		}
		else
		{
			sprintf((char *)&MSG, "\t-> Flash NOT OK\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			start_flag=0;
		}
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
	}
}

void Test_Emergency(void)
{
	while(start_flag==1 && emergency_flag==0)
	{
		test_emergency = 1;
		if (loop_flag == 0)
		{
			sprintf((char *)&MSG, "\t-> Pull the emergency tab\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			loop_flag++;
		}
	}
	loop_flag=0;
	while(start_flag==1 && emergency_flag==1)
		{
			if(loop_flag==0)
			{
				sprintf((char *)&MSG, "\t-> Push the emergency tab\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
				loop_flag++;
			}
		}
	if (start_flag==1)
	{
		if (emergency_flag==0) //Emergency test OK
		{
			sprintf((char *)&MSG, "\t-> Emergency tab OK\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			stop_flag=1;
		}
		else
		{
			sprintf((char *)&MSG, "\t-> Emergency tab NOT OK\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen((char*)MSG),200);
			start_flag=0;
		}
		while(flag_test==1 && start_flag==1)
		{
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, aCOMRxBuffer, sizeof(aCOMRxBuffer));
		}
	}
}

uint8_t Read_ACC_WHO_AM_I(void)
{
	uint8_t who_am_i = Acc_Read_Reg(0x0F); //Read WHO_AM_I reg
	return who_am_i;
}


/*
void Play_Sound(uint16_t freq, uint16_t duration)
{
  HAL_GPIO_WritePin(BUZZER_EN_GPIO_Port, BUZZER_EN_Pin, GPIO_PIN_SET);
  __HAL_TIM_CLEAR_IT(&htim16, TIM_IT_UPDATE);

  TIM16->ARR = 10*duration ;
  TIM17->ARR = (uint32_t)((SystemCoreClock / freq) -1 );
  TIM17->CCR1 =  (uint32_t)((SystemCoreClock / freq) -1 )/2;

  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim16);
}
*/

