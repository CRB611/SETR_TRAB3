/**
 * @file uart.h
 */
#ifndef UART_H
#define UART_H


#define MAX_TEMP 100
#define MAX_SIZE 20
 #define UART_RX_SIZE 30 	///< Maximum size of the RX buffer  
 #define UART_TX_SIZE 30 	///< Maximum size of the TX buffer 
 #define SOF_SYM '#'	        ///< Start of Frame Symbol
 #define EOF_SYM '!'         ///< End of Frame Symbol 
#define OK 0
#define INIT_ERROR -1
#define EOF_ERROR -2
#define SOF_ERROR -3
#define EMPTY_COMMAND -4
#define CHECKSUM_ERROR -5
#define ERROR_TOO_HOT -6
#define COMMAND_ERROR -7
#define FATAL_ERROR -8



int uart_init(void);

void resetRxBuffer(void);

void resetTxBuffer(void);

int calcChecksum(unsigned char * buf, int nbytes); 

int uart_process(void);

 /**
  * \brief Converts an integer to a ASCII
  * \param array pointer to where the converted chars will be stored
  * \param num the integer to be converted
  * \param type 't' for temperature, 'h' for humidity, 'c' for c02 or checksum
  */
 void num2char(unsigned char *array, int num);

 /**
  * \brief Converts ASCII to int
  * \param ascii pointer to where the converted chars are stored
  * \param length number of characters in the ascii code
  * \return Returns the int corresponding to the character array
  */
unsigned int char2num(unsigned char ascii [], int length);

 /**
  * \brief Adds a char to the reception buffer
  * \return OK if success
  * \return FULL_BUFF if the buffer is full
  */
 int rxChar(unsigned char car);
 
 /**
  * \brief Adds a char to the transmission buffer
  * \return OK if success
  * \return FULL_BUFF if the buffer is full
  */
 int txChar(unsigned char car);
 
#endif