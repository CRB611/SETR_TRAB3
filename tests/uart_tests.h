/**
 * @file uart_tests.h
 * @brief Declaration of the unit tests for the UART module.
 *
 * This file contains the declarations of the test functions that validate the UART module
 * behaviour.
 * It uses UNITY framework to test the utilities functions (num2char, char2num, char2float, calcChecksum),
 * and the processing of commands.
 *
 * @authors Simão Ribeiro
 * @date 04/06/2025
 */
#ifndef UART_TESTS_H
#define UART_TESTS_H

#include "unity.h"
#include "../modules/uart.h"

#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief Tests the convertion from integer to 3 digit ASCII.
 */
void test_num2char(void);

/**
 * @brief Tests the convertion from to 3 digit ASCII to integer.
 */
void test_char2num(void);

/**
 * @brief Tests the convertion from string ASCII 'pp.f' to float.
 */
void test_char2float(void);

/**
 * @brief tests the modulo-256 checksum computation.
 */
void test_calcChecksum(void);

/**
 * @brief Tests the processing of the '#MxxxYYY!' command with a value within boundaries.
 */
void test_process_set_max_temp_ok(void);

/**
 * @brief Tests the processing of the '#MxxxYYY!' command with a value out of bounds.
 */
void test_process_set_max_temp_too_hot(void);

/**
 * @brief Tests the processing of the '#Cyyy!' command.
 */
void test_process_get_current_temp(void);

/**
 * @brief Tests an invalid checksum.
 */
void test_process_invalid_checksum(void);

/**
 * @brief Tests an unknown.
 */
void test_process_invalid_command(void);

/**
 * @brief Executes a success of the Uart commands.
 */
void run_uart_tests(void);


#ifdef __cplusplus
}
#endif

#endif // UART_TESTS_H
