/*
Author: Nilesh Ghule <nilesh@sunbeaminfo.com>
Course: PG-DESD @ Sunbeam Infotech
Date: Sep 23, 2024
*/

#include <stdio.h>
#include "uart.h"

void UartInit(uint32_t baud) {
	// enable gpio clock -- AHB1ENR.0
	RCC->AHB1ENR |= BV(GPIO_UART_CLKEN);
	// set gpio pins to alt fn 7 (AF7) -- AFRL = 0111
	GPIO_UART->AFR[0] |= BV(14) | BV(13) | BV(12) | BV(10) | BV(9) | BV(8);
	GPIO_UART->AFR[0] &= ~(BV(15) | BV(11));
	// set gpio pins mode to alt fn	-- MODER = 10
	GPIO_UART->MODER &= ~(BV(UART_TX_PIN*2) | BV(UART_RX_PIN*2));
	GPIO_UART->MODER |= (BV(UART_TX_PIN*2+1) | BV(UART_RX_PIN*2+1));

	// enable uart clock -- APB1ENR.17
	RCC->APB1ENR |= BV(UART_CLKEN);
	// enable transmission and reception on uart
	UART->CR1 = BV(USART_CR1_TE_Pos) | BV(USART_CR1_RE_Pos);
	// set word length in CR1 -- M bit = 8-bit data len, Over8 = 0
	UART->CR1 &= ~(BV(USART_CR1_M_Pos) | BV(USART_CR1_OVER8_Pos));
	// set stop bits in CR2 -- 1 stop bit
	UART->CR2 &= ~(USART_CR2_STOP_0 | USART_CR2_STOP_1);
	// set baud rate -- UBRR
	if(baud == 9600)
		UART->BRR = UBRR_9600;
	else if(baud == 38400)
		UART->BRR = UBRR_38400;
	else if(baud == 115200)
			UART->BRR = UBRR_115200;
	// enable USART interrupt in NVIC
	NVIC_EnableIRQ(USART2_IRQn);
	// enable uart in CR1 -- UE bit
	UART->CR1 |= BV(USART_CR1_UE_Pos);
}

char *tx_str;
int tx_index;
volatile int tx_complete=1;

void UartPuts(char str[]) {
	// wait for previous string tx complete
	while(tx_complete == 0);
	// get ready for transmission
	tx_str = str;
	tx_index = 0;
	tx_complete = 0;
	// write first char
	UART->DR = tx_str[tx_index];
	// enable transmission interrupt
	UART->CR1 |= BV(USART_CR1_TXEIE_Pos);
}

void USART2_IRQHandler(void) {
	// check if interrupt is TXE and then handle it
	if(UART->SR & BV(USART_SR_TXE_Pos)) {
		// send next char (if not '\0')
		tx_index++;
		char ch = tx_str[tx_index];
		if(ch != '\0')
			UART->DR = ch;
		// if null char, stop interrupt and mark tx completion
		else {
			UART->CR1 &= ~BV(USART_CR1_TXEIE_Pos);
			tx_complete = 1;
			tx_str = NULL;
		}
	}
}








