/** 
 * \file uart.h
 * \brief This file contains the defines and functions needed for the UART related functions.
 *
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug There are no known bugs.
 */
#ifndef UART_H
#define UART_H

#define SOF_SYM '#'	        ///< Start of Frame Symbol
#define EOF_SYM '!'         ///< End of Frame Symbol 
#define EOF_ERROR -1        ///< Return if there's and End-of-File Error
#define SOF_ERROR -2        ///< Return if there's and Start-of-File Error
#define CHECKSUM_ERROR -3   ///< Return if the checksum is wrong
#define ERROR_TOO_HOT -4    ///< Return if the set max temperature is too high
#define COMMAND_ERROR -5    ///< Return if the command does not exist
#define FATAL_ERROR -6      ///< Return if there's another error


/**
  * \brief Computes the checksum  modulo-256 of a given number of chars.
  * \param buf pointer to the buffer
  * \param nbytes number of bytes to check
  */
int calcChecksum(unsigned char * buf, int nbytes); 

 /**
  * \brief Converts an integer to a 3 digit ASCII
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
  * \brief Converts ASCII to float
  * 
  *   The ASCII parameter has to be in pp.f format
  * \param ascii pointer to where the converted chars are stored
  * \return Returns the float corresponding to the character array
  */
float char2float(unsigned char ascii []);


/**
 * \brief Processes the uart commands
 * 
 *   Processes the UART commands by validating the framing and the checksum and
 *  afterwards, computing the received command  
 *  \param cmd Uart receive buffer, where the command is
 *  \param reply Uart transmition buffer, where the responces will be sent 
 *  \return OK if success, Error message in error case.
 */
int process_uart_command(const char *cmd, char *reply);


#endif