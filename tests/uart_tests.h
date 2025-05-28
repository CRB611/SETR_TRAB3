/**
 * @file uart_tests.h
 * @brief Declarações dos testes Unity para o módulo UART.
 *
 * Este ficheiro contém as assinaturas das funções de teste que verificam:
 *   - Utilitários: num2char, char2num, char2float, calcChecksum
 *   - Processamento de comandos UART: process_uart_command
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
 * @brief Testa a conversão de inteiro para array ASCII de três dígitos.
 */
void test_num2char(void);

/**
 * @brief Testa a conversão de ASCII de três dígitos para inteiro.
 */
void test_char2num(void);

/**
 * @brief Testa a conversão de string ASCII 'pp.f' para float.
 */
void test_char2float(void);

/**
 * @brief Testa o cálculo de checksum modulo-256.
 */
void test_calcChecksum(void);

/**
 * @brief Testa o processamento de comando '#MxxxYYY!' com valor dentro do limite.
 */
void test_process_set_max_temp_ok(void);

/**
 * @brief Testa o processamento de comando '#MxxxYYY!' com valor superior ao limite.
 */
void test_process_set_max_temp_too_hot(void);

/**
 * @brief Testa o processamento de comando '#Cyyy!' para obter a temperatura atual.
 */
void test_process_get_current_temp(void);

/**
 * @brief Testa o caso de checksum inválido no comando UART.
 */
void test_process_invalid_checksum(void);

/**
 * @brief Testa o caso de comando desconhecido no frame UART.
 */
void test_process_invalid_command(void);

/**
 * @brief Executa todos os testes de UART.
 */
void run_uart_tests(void);

#ifdef __cplusplus
}
#endif

#endif // UART_TESTS_H
