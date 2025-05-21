/** 
 * \file uart.c
 * \brief This file contains the implementation of the functions implemented in uart.h.
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug There are no known bugs.
 */
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


float char2float(unsigned char ascii []){
	int i = 0, sum_1 = 0, mult = 10;
	int x;
	printf("start %s\n",ascii);
	while(i < 2){
		x= (ascii[i]-'0') *mult ;

	  	sum_1 += x;
		mult/=10;
	  	i++;
	}
	/*skipping the point*/
	printf("int: %d, float %f\n", sum_1, (float)sum_1);
	i++;
	printf("**.x -> %c\n",ascii[i]);
	x=(ascii[i]-'0');
	float x2=(float)x;
	float sum=(float)sum_1+x2/10;

	return sum;
}
