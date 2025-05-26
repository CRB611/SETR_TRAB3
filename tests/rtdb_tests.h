/**
 * @file test_rtdb.h
 * @brief Declaração de testes unitários para o módulo RTDB (Real-Time Database).
 * 
 * Este ficheiro contém as declarações das funções de teste que validam 
 * o comportamento do módulo `rtdb`. Utiliza o framework Unity para testar
 * valores por omissão, definição e recuperação de parâmetros como PID,
 * temperatura, flags de erro, estado do sistema e setpoint.
 */

#ifndef TEST_RTDB_H
#define TEST_RTDB_H

#include "unity.h"
#include <stdint.h>



/** 
 * @brief Testa os valores por omissão sem inicialização explícita.
 */
void test_default_values_without_init(void);

/** 
 * @brief Testa os valores por omissão após chamada a rtdb_init().
 */
void test_default_values(void);

/** 
 * @brief Testa se a flag de erro está desativada por omissão.
 */
void test_default_error_flag(void);

/** 
 * @brief Testa a ativação e desativação da flag de erro.
 */
void test_set_get_error_flag(void);

/** 
 * @brief Testa se o sistema está desligado por omissão.
 */
void test_default_system_off(void);

/** 
 * @brief Testa a ativação e desativação do sistema.
 */
void test_set_get_system_on(void);

/** 
 * @brief Testa o setpoint por omissão.
 */
void test_default_setpoint(void);

/** 
 * @brief Testa a definição e leitura do setpoint.
 */
void test_set_get_setpoint(void);

/** 
 * @brief Testa os limites mínimo e máximo de temperatura máxima permitida.
 */
void test_max_temp_boundaries(void);

/** 
 * @brief Testa se o reset completo do sistema repõe todos os valores por omissão.
 */
void test_full_reset(void);



#endif // TEST_RTDB_H
