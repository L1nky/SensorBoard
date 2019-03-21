/*
 * ert_lib.h
 *
 *  Created on: 21 Mar 2019
 *      Author: linky
 */

#ifndef LIB_ERT_LIB_H_
#define LIB_ERT_LIB_H_

#include "stm32f4xx_hal.h"

#define UART_TIMEOUT 10

HAL_StatusTypeDef INFO(char* msg);

#endif /* LIB_ERT_LIB_H_ */
