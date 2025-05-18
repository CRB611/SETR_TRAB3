/**
 * @file uart.h
 */
#ifndef UART_H
#define UART_H


#define OK 0
#define INIT_ERROR -1

int uart_init(void);

void resetRxBuffer(void);

void resetTxBuffer(void);

int calcChecksum(unsigned char * buf, int nbytes); 

#endif