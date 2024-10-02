/*
 * Systick.h
 *
 *  Created on: Oct 2, 2024
 *      Author: pawan
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "stm32f4xx.h"

void SysTick_Handler(void);
void SysTick_Delay(uint32_t ms);

#endif /* SYSTICK_H_ */
