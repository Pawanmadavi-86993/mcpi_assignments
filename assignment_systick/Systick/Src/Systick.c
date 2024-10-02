/*
 * Systick.c
 *
 *  Created on: Oct 2, 2024
 *      Author: pawan
 */
#include "Systick.h"
#include "stm32f407xx.h"
volatile uint32_t jiffies = 0;

void SysTick_Handler(void) {
	jiffies++;

}
void SysTick_Delay(uint32_t ms) {
	uint32_t until = jiffies + ms;
	while(jiffies < until)
		;
}
