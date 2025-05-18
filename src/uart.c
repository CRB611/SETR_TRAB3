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

int uart_process(void){
	unsigned int i=0,k=0;
	unsigned char sid;
		
	/* Detect empty cmd string */
	if(rxBufLen == 0)
		return EMPTY_COMMAND; 

	/* Find index of SOF */
	for(i=0; i < rxBufLen; i++) {
		if(UARTRxBuffer[i] == SOF_SYM) {
			break;
		}else if (i == (unsigned int)rxBufLen-1){
			eraseRxBuff(rxBufLen);
			return SOF_ERROR;
		}
	}

	/* Checking correct end of message*/
	for (k = 0; k <= MAX_SIZE; k++)
	{
		if (UARTRxBuffer[k] == EOF_SYM)
		{
			break;

			// Se não acaba com o simbolo que deve dá erro
		}
		else if (k == MAX_SIZE - 1)
		{
			eraseRxBuff(rxBufLen);
			return EOF_ERROR;
		}
	}

	/* Checking correct checksum */
	int chk = calcChecksum(&UARTRxBuffer[i + 1], k - 4); // inclui o tipo, sinal e valor
	int chk_recv = char2num(&UARTRxBuffer[k - 3], 3);	 // os três dígitos ASCII

	// verificar a checksum
	if (chk != chk_recv)
	{
		for (size_t j = 0; j < k; j++)
		{
			printk(UARTRxBuffer[j]);
		}
		printk(" Checksum error.")	//perguntar ao stor
		return CHECKSUM_ERROR;
	}

	/* If a SOF and EOF, and the checksum is correct look for commands */
	if(i < rxBufLen) {
		//checking the command
		switch(UARTRxBuffer[i+1]) { 
			
			case 'M':	
			{	
				/*getting the temperature to be set*/
				int  set_max_temp= char2num(&UARTRxBuffer[i+2], 3);
				
				if (set_max_temp > MAX_TEMP)
				{
					return ERROR_TOO_HOT;
				}
					
				/*SET THE MAX TEMP AQUI*/
				for (size_t j = 0; j < k; j++)
				{
					printk(UARTRxBuffer[j]);
				}
				
				printk(" The max temp was set to: %d, %d is the checksum.\r\n",set_max_temp,chk_recv);
				
				return OK;
			}
			case 'S':
			{
				/*codigo codigo codigo*/
				int  KP= char2num(&UARTRxBuffer[i+2], 3);
				int  TI= char2num(&UARTRxBuffer[i+5], 3);
				int  TD= char2num(&UARTRxBuffer[i+8], 3);
				
				/*codigo codigo codigo*/
				for (size_t j = 0; j < k; j++)
				{
					printk(UARTRxBuffer[j]);
				}
				
				printk(" The controller parameters were set to: Kp=%d, Ti=%d, Td=%d \n%d is the checksum.\r\n",KP,TI,TD,chk_recv);

				return OK;
				
			}	
			case 'C':
			{
				
				txChar("#");
				txChar("c");
				/*resto*/
				txChar("!");
				
				return OK;
			}
			default:
			{
				eraseRxBuff(rxBufLen);
				
				for (size_t j = 0; j < k; j++)
				{
					printk(UARTRxBuffer[j]);
				}
					
				return COMMAND_ERROR;	
			}				
		}
		
		
	}else
	
	/* Cmd string not null and SOF not found */
	return FATAL_ERROR;

}



void num2char(unsigned char *array, int num, char type){
    int i = 0;
	int len=3;

	//checking what type of data it is
	if (type=='t')
	{
		if (num>=0)
		{
			*array='+';
		}else{
			*array='-';
		}
		num=abs(num);
		
		len=2;

		while (i < len) {
			*(array + len-i) = (num % 10) + '0';
			num /= 10;
			i++;
		} 
		return;

	}else if (type=='c'){
		len=5;
	}

	while (i < len) {
		*(array + len-i-1) = (num % 10) + '0';
		num /= 10;
		i++;
	} 
	
    
}

unsigned int char2num(unsigned char ascii [], int length){
	int i = 0, sum = 0, mult = pow(10,length-1);
	int x;
	
	while(i < length){
		x= (ascii[i]-'0') *mult ;

	  	sum += x;
		mult/=10;
	  	i++;
	}
	return sum;
}

/*
 * rxChar
 */
int rxChar(unsigned char car)
{
	/* If rxbuff not full add char to it */
	if (rxBufLen < UART_RX_SIZE) {
		UARTRxBuffer[rxBufLen] = car;
		rxBufLen += 1;
		return OK;		
	}	
	/* If cmd string full return error */
	return FULL_BUFF;
}

/*
 * txChar
 */
int txChar(unsigned char car)
{
	/* If rxbuff not full add char to it */
	if (txBufLen < UART_TX_SIZE) {
		UARTTxBuffer[txBufLen] = car;
		txBufLen += 1;
		return OK;		
	}	
	/* If cmd string full return error */
	return FULL_BUFF;
}
