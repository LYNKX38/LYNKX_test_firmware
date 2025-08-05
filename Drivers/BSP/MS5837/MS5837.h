#include <stdio.h>
#include <stdlib.h>

/* Data sensor structure */
typedef struct
{
	uint16_t C[7];
	int32_t TEMP,P;

}data_sensor_t;

/* Functions prototypes */
void Scan_I2C(void);
void Reset_pressure_sensor(void);
void Store_calibration_data(data_sensor_t*);
uint32_t Read_digital_data1(void);
uint32_t Read_digital_data2(void);
void Calculate_values(data_sensor_t*);
void Get_values(data_sensor_t*);
void Init_pressure_sensor(data_sensor_t*);
void Print_environment_caracteristics(data_sensor_t*);
