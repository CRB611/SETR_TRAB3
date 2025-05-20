#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "uart.h"

#include <zephyr/drivers/uart.h>

int calcChecksum(unsigned char * buf, int nbytes) {
	unsigned int sum = 0 ;
	
	for(unsigned char* i = buf ; i < (buf+nbytes);i++ ){
		sum += (unsigned int)(*i);
	}
	return (sum%256);		
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

