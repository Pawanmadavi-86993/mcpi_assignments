/*
Author: Nilesh Ghule <nilesh@sunbeaminfo.com>
Course: PG-DESD @ Sunbeam Infotech
Date: Sep 21, 2024
*/

#include "switch_intr.h"


void SwitchInit(void) {
	// Enable GPIO clock
    RCC->AHB1ENR |= BV(SWITCH_GPIO_CLKEN);
	// Set gpio mode as input
    GPIOA->MODER &= ~(BV(SWITCH_PIN*2+1) | BV(SWITCH_PIN*2));
	// set no pull-up pull-down registers
    GPIOA->PUPDR &= ~(BV(SWITCH_PIN*2+1) | BV(SWITCH_PIN*2));
	// enable falling edge detection (in FTSR)
    EXTI->FTSR |= BV(SWITCH_EXTI);
	// enable (unmask) exti interrupt (in IMR)
    EXTI->IMR |= BV(SWITCH_EXTI);
	// select exti interrupt (in SYSCFG->EXTICRx) -- EXTI0 --> EXTICR1[3:0] = 0000
    SYSCFG->EXTICR[0] &= ~(BV(3)|BV(2)|BV(1)|BV(0));
	// enable exti in NVIC (ISER or NVIC_EnableIRQ())
    NVIC_EnableIRQ(EXTI0_IRQn); // EXTI0_IRQn
}

// Global SwitchExtiFlag
volatile uint32_t SwitchExtiFlag = 0;

int count=0;

void EXTI0_IRQHandler(void) {
	// acknowledge the interrupt
	EXTI->PR |= BV(SWITCH_EXTI);
	// interrupt handling logic

	SwitchExtiFlag = 1;
	count++;
}









