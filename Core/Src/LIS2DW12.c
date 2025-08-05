/* Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "LIS2DW12.h"
#include "main.h"

/* Variables */
//uint8_t tx_data[2];
//uint8_t rx_data[2];
char MSG_ACC[50];
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;

uint32_t retour ;




/* Functions */


uint8_t Acc_Read_Reg(uint8_t add)
{
	uint8_t cmd;
	uint8_t rx_data;
	cmd = add|0x80;

	Ensure_SPI_Config_For_Flash_or_Acc();

	HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &rx_data,1,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
	return rx_data;
}


