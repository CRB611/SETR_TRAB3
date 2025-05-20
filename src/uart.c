#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "uart.h"
#include "rtdb.h"
#include "pid.h"

#include <zephyr/drivers/uart.h>

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

int uart_process(uint8_t *UARTRxBuffer,int rxBufLen, uint8_t *UARTTxBuffer,int txBufLen){
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
					
				rtdb_set_maxtemp((uint8_t)set_max_temp);

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
				
				//pid_set();

				for (size_t j = 0; j < k; j++)
				{
					printk(UARTRxBuffer[j]);
				}
				
				printk(" The controller parameters were set to: Kp=%d, Ti=%d, Td=%d \n%d is the checksum.\r\n",KP,TI,TD,chk_recv);

				return OK;
				
			}	
			case 'C':
			{
				char start_msg="#c";	
				err = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
						if (err) {
						return err;
						}
				
				/*resto*/
				uint8_t temp=rtdb_get_cur_temp();
				unsigned char temp_c[3];
				num2char(&temp[0],temp);

				for (int j = 0; j < 3; j++){
					txChar(temp[j]);
				}

				int check = calcChecksum(&UARTTxBuffer[2], 16);
				unsigned char check_c[3];
				num2char(&check_c[0], check, 'h');

				for (int j = 0; j < 3; j++){
					txChar(check_c[j]);
				}

				txChar("!",&UARTTxBuffer,txBufLen);
				
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



void num2char(unsigned char *array, int num){
    int i = 0;
	int len=3;

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

