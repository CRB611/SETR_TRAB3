/**
 * @file pid_tests.h
 * @brief Declaração de testes unitários para o módulo de controlo PID.
 * 
 * Este ficheiro define os protótipos das funções de teste para o módulo PID.
 * Os testes verificam a correta implementação das ações proporcional, integral e derivativa,
 * bem como o tratamento de casos particulares como tempos nulos ou negativos, 
 * e a sequência de chamadas a funções do PID.
 */

#ifndef PID_TESTS_H
#define PID_TESTS_H

#include "unity.h"
#include "../modules/pid.h"


/** 
 * @brief Testa os valores por omissão dos parâmetros PID após inicialização.
 */
void test_default_pid_params(void);

/**
 * @brief Testa a contribuição da componente proporcional para a saída do PID.
 */
void test_proportional_action(void);

/**
 * @brief Testa a contribuição da componente integral para a saída do PID.
 */
void test_integral_action(void);

/**
 * @brief Testa a contribuição da componente derivativa para a saída do PID.
 */
void test_derivative_action(void);

/**
 * @brief Testa o reset da componente integral e a ausência da sua contribuição após o reset.
 */
void test_integral_reset(void);

/**
 * @brief Testa o comportamento do PID quando o intervalo de tempo (dt) é zero.
 * 
 * Espera-se que as componentes integral e derivativa não contribuam para a saída.
 */
void test_zero_dt_behaviour(void);

/**
 * @brief Testa a consistência do PID ao ser chamado de forma sequencial 
 * com diferentes combinações de ações (P, PI, PID).
 */
void test_cascading_pid_steps(void);

/**
 * @brief Garante que a derivada no primeiro passo de execução é zero.
 */
void test_derivative_initial_zero(void);

/**
 * @brief Testa o comportamento do PID quando o intervalo de tempo (dt) é negativo.
 * 
 * Espera-se que esse caso seja tratado como dt = 0, ou seja, sem contribuição integral ou derivativa.
 */
void test_negative_dt_behaviour(void);



#endif // PID_TESTS_H
