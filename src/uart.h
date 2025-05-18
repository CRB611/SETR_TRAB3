/**
 * @file uart.h
 */

#ifndef UART_H
#define UART_H

#include <zephyr/drivers/uart.h>

#define OK 0
#define INIT_ERROR -1

int uart_init(void);


void resetRxBuffer(void);

void resetTxBuffer(void);

int calcChecksum(unsigned char * buf, int nbytes) {
	unsigned int sum = 0 ;
	
	for(unsigned char* i = buf ; i < (buf+nbytes);i++ ){
		sum += (unsigned int)(*i);
	}
	return (sum%256);		
}
