/*
 * abs_p.c
 *
 *  Created on: 17 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

/*
 * Code based on this library: https://github.com/freetronics/BaroSensor/blob/master/BaroSensor.cpp
 */

#include "inttypes.h"

#include <Sensors/BME280/bme280.h>
#include <Sensors/BNO055/bno055.h>
#include <Sensors/i2c_sensors.h>
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart3;

#define I2C_TIMEOUT 10 //ms

struct bme280_dev bme;
struct bme280_data comp_data;
uint8_t meas_dur;
char buf[300];

//# BNO055
struct bno055_t bno055;
struct bno055_accel_float_t f_accel_xyz;
struct bno055_gyro_float_t f_gyro_xyz;
struct bno055_mag_float_t f_mag_xyz;

int8_t stm32_i2c_read (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t rslt = HAL_I2C_Mem_Read(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	return rslt;
}

int8_t stm32_i2c_write (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t rslt = HAL_I2C_Mem_Write(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	return rslt;
}

void stm32_delay_ms (uint32_t period)
{
	osDelay(period);
}

void initI2cDevices ()
{
	// ######## BME280

	int8_t rslt;

	bme.dev_id = BME280_I2C_ADDR_PRIM;
	bme.intf = BME280_I2C_INTF;
	bme.read = &stm32_i2c_read;
	bme.write = &stm32_i2c_write;
	bme.delay_ms = &stm32_delay_ms;

	rslt = bme280_init(&bme);
	sprintf(buf, "Init ok ? %d\n", rslt);
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);

	/* Always read the current settings before writing, especially when
	* all the configuration is not modified
	*/
	rslt = bme280_get_sensor_settings (&bme);
	/* Check if rslt == BME280_OK, if not, then handle accordingly */
	sprintf(buf, "Config get ok ? %d\n", rslt);
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);

	/* Overwrite the desired settings */
	bme.settings.filter = BME280_FILTER_COEFF_OFF;
	bme.settings.osr_p = BME280_OVERSAMPLING_8X;
	bme.settings.osr_t = BME280_OVERSAMPLING_1X;
	bme.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

	rslt = bme280_set_sensor_settings (BME280_ALL_SETTINGS_SEL, &bme);
	/* Check if rslt == BME280_OK, if not, then handle accordingly */
	sprintf(buf, "Config set ok ? %d\n", rslt);
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);

	/* Always set the power mode after setting the configuration */
	rslt = bme280_set_sensor_mode (BME280_NORMAL_MODE, &bme);
	/* Check if rslt == BME280_OK, if not, then handle accordingly */

	// ######## BMNO055

	bno055.bus_write = &stm32_i2c_write;
	bno055.bus_read = &stm32_i2c_read;
	bno055.delay_msec = &stm32_delay_ms;
	bno055.dev_addr = BNO055_I2C_ADDR1;

	s32 comres = bno055_init (&bno055);

	uint8_t power_mode = BNO055_POWER_MODE_NORMAL;
	comres += bno055_set_power_mode (power_mode);

	comres += bno055_set_operation_mode (BNO055_OPERATION_MODE_AMG);
	bno055_set_accel_range (BNO055_ACCEL_RANGE_16G);
}

void TK_fetch_i2c ()
{
	osDelay (500);

	sprintf(buf, "\nInitializing Sensors...\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);
	initI2cDevices ();

	int8_t rslt;
	uint8_t cntr = 0;

	for (;;)
    {
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
		if (!rslt)
		{
			if(!cntr)
			{
				sprintf (buf, "%"PRId32" %"PRId32" %"PRId32"\n", comp_data.pressure, comp_data.temperature, comp_data.humidity);
				HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);
			}
		}

		rslt = 0;
		rslt += bno055_convert_float_accel_xyz_mg (&f_accel_xyz);
		rslt += bno055_convert_float_mag_xyz_uT (&f_mag_xyz);
		rslt += bno055_convert_float_gyro_xyz_rps (&f_gyro_xyz);

		if(!rslt && !cntr)
		{
			sprintf(buf, "Accel: [%f, %f, %f]\nGyro: [%f, %f, %f]\n Mag: [%f, %f, %f]\n",
				  f_accel_xyz.x, f_accel_xyz.y, f_accel_xyz.z,
				  f_gyro_xyz.x, f_gyro_xyz.y, f_gyro_xyz.z,
				  f_mag_xyz.x, f_mag_xyz.y, f_mag_xyz.z);
//			HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);
//			sprintf(buf, "%f %f %f\n", f_accel_xyz.x, f_accel_xyz.y, f_accel_xyz.z);
//			sprintf(buf, "%f %f %f\n", f_gyro_xyz.x, f_gyro_xyz.y, f_gyro_xyz.z);
//			HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100);
		}

		cntr = ++cntr < 10 ? cntr : 0;
		osDelay(10);
    }
}
