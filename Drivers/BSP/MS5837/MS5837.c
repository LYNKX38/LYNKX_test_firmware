/* Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "MS5837.h"
#include "main.h"
#include <errno.h>

/* Variables */
uint8_t data_buffer[3];
uint8_t TEMPORARY_ADDR;
uint8_t MS5837_ADDR = 0x76<<1;
uint8_t MS5837_ADC_READ = 0x00;
uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
uint8_t MS5837_CONVERT_D2_8192 = 0x5A;
uint8_t MS5837_PROM_READ = 0xA0;
uint8_t MS5837_RESET = 0x1E;
char MSG_MS5837[50];

extern FILE* file;
extern I2C_HandleTypeDef hi2c2;

/* Functions */
void Reset_pressure_sensor(void)
{
	HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, &MS5837_RESET, 1, 500);
}

void Store_calibration_data(data_sensor_t* data)
{
	uint8_t i;
	for (i = 0 ; i < 7 ; i++ )
	{
		TEMPORARY_ADDR = MS5837_PROM_READ+(i<<1); /* Get address of calibration coefficients i */

		/* Transmit each READ PROM command */
		HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, &TEMPORARY_ADDR, 1, 500);

		/* Receive C[] values (calibration coefficients) */
		HAL_I2C_Master_Receive(&hi2c2, MS5837_ADDR, data_buffer, 2, 500);

		/* Store coefficients in data structure */
		data->C[i]=(data_buffer[0]<<8)|data_buffer[1];
	}
}

uint32_t Read_digital_data1(void)
{
	uint32_t D1;

	/* Transmit D1 conversion command (8192 Sample Rate) */
	HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, &MS5837_CONVERT_D1_8192, 1, 500);

	HAL_Delay(20); // Max conversion time

	/* Transmit read command */
	HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, &MS5837_ADC_READ, 1, 500);

	/* Receive D1 value in data_buffer */
	HAL_I2C_Master_Receive(&hi2c2, MS5837_ADDR, data_buffer, 3, 500);

	/* Transform data_buffer to uint32_t */
	D1 = (data_buffer[0]<<16)|(data_buffer[1]<<8)|(data_buffer[2]);

	return D1;

}

uint32_t Read_digital_data2(void)
{
	uint32_t D2;

	/* Transmit D2 conversion command (8192 Sample Rate) */
	HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, &MS5837_CONVERT_D2_8192, 1, 500);

	HAL_Delay(20); // Max conversion time

	/* Transmit read command */
	HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, &MS5837_ADC_READ, 1, 500);

	/* Receive D2 value in data_buffer */
	HAL_I2C_Master_Receive(&hi2c2, MS5837_ADDR, data_buffer, 3, 500);

	/* Transform data_buffer to uint32_t */
	D2 = (data_buffer[0]<<16)|(data_buffer[1]<<8)|(data_buffer[2]);

	return D2;
}


void Calculate_values(data_sensor_t* data)
{
	uint64_t D[2];
	//int32_t dT,TEMP,P;
	int64_t OFF,SENS,TEMP,P,dT;

	D[0]=Read_digital_data1(); /*Get D1 value */
	D[1]=Read_digital_data2(); /* Get D2 value */

	/* Calculate first order temperature (TEMP) and pressure (P) */
	dT = (uint64_t)D[1] - (uint64_t)data->C[5]*(uint64_t)pow(2,8);
	TEMP = 2000 + ((int64_t)dT*(int64_t)data->C[6])/(int64_t)pow(2,23);

	OFF = (uint64_t)data->C[2]*(uint64_t)pow(2,17) + (uint64_t)data->C[4]*dT/(uint64_t)pow(2,6);
	SENS = (uint64_t)data->C[1]*(uint64_t)pow(2,16) + (uint64_t)data->C[3]*dT/(uint64_t)pow(2,7);
	P = (D[0]*SENS/(uint64_t)pow(2,21) - OFF)/(uint64_t)pow(2,15);

	/* Low temperature adjustment -> second order values */
	if (TEMP<2000)
	{
		OFF -= 31*(uint64_t)pow(TEMP-2000,2)/(uint64_t)pow(2,3);
		SENS -= 63*(uint64_t)pow(TEMP-2000,2)/(uint64_t)pow(2,5);
		TEMP -= 11*(uint64_t)pow(dT,2)/(uint64_t)pow(2,35);
		P = (D[0]*SENS/(uint64_t)pow(2,21) - OFF)/(uint64_t)pow(2,15);
	}

	/* Store temperature and pressure in data structure */
	data->P = P;
	data->TEMP = TEMP;

}

void Get_values(data_sensor_t* data)
{
	/* Print values in terminal with UART */
	/* Values are 100 times real value to keep precision */
	sprintf((char *)&MSG_MS5837, "\t-> Barometer measured pressure (x100) is  : %ld mbar\r\n",data->P);
	HAL_UART_Transmit(&huart1, (uint8_t*)MSG_MS5837, strlen((char*)MSG_MS5837),200);
}


void Init_pressure_sensor(data_sensor_t* data)
{
	Reset_pressure_sensor(); // Reset
	Store_calibration_data(data); // Store calibration data in structure
}

void Print_environment_caracteristics(data_sensor_t* data)
{
	Calculate_values(data); // Calculate and store pressure and temperature in structure
	Get_values(data); // Read pressure and temperature values in structure
}

