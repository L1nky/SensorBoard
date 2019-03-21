/*
 * ert_lib.c
 *
 *  Created on: 21 Mar 2019
 *      Author: linky
 */

#include "lib/ert_lib.h"
#include "lib/rocket_configuration.h"

#include "string.h"

extern UART_HandleTypeDef huart3;

HAL_StatusTypeDef INFO(char* msg)
{
#ifdef DEBUG
	return HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), UART_TIMEOUT);
#else
	return HAL_OK;
#endif
}
