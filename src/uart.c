#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "uart.h"

#include <zephyr/drivers/uart.h>

static unsigned char UARTRxBuffer[UART_RX_SIZE];
static unsigned char rxBufLen = 0; 

static unsigned char UARTTxBuffer[UART_TX_SIZE];
static unsigned char txBufLen = 0; 

int init(void){

	resetRxBuffer();
	resetTxBuffer();

	if(txBufLen==0){
		return OK;//vazio
	}else{
		return INIT_ERROR;//Não vazio	
	}
}

void resetRxBuffer(void)
{
	rxBufLen = 0;		
	return;
}

void resetTxBuffer(void)
{
	txBufLen = 0;		
	return;
}


int calcChecksum(unsigned char * buf, int nbytes) {
	unsigned int sum = 0 ;
	
	for(unsigned char* i = buf ; i < (buf+nbytes);i++ ){
		sum += (unsigned int)(*i);
	}
	return (sum%256);		
}

int uart_process(void);

