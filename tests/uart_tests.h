#ifndef UART_TEST_H_
#define UART_TEST_H_


/**
  * \brief Sets up the UNITY testing
  */
void setup(void);
 
/**
  * \brief closes the unity testing
  */
void teardown(void);

/**
 * \brief Performs a test for the num2char convertion function using UNITY
 * 
 *  This function tests the conversion from int to char array in the temperature, co2 and humidity/checksum case.
 */
void test_num2char(void);

/**
 * \brief Performs a test for the char2num convertion function using UNITY
 */
void test_char2num(void);

/**
 * \brief Performs a test for the char2float convertion function using UNITY
 */
void test_char2float(void);

/**
 * \brief Performs a test for the checksum calculation using UNITY
 */
void test_checksum(void);

/*Até aqui sei que passam a partir daqui i would need a placa e um cmakelist a funcionar pq eu testei estes no trab2*/


void test_commands(void);

void test_command_errors(void);


#endif