/**
 * @file heater_tests.h
 * @brief Prototypes e documentação dos testes unitários do módulo Heater
 *
 * Este ficheiro declara as funções de teste para o módulo Heater,
 * utilizando o framework Unity.
 */

#ifndef HEATER_TESTS_H_
#define HEATER_TESTS_H_

#include "unity.h"
#include <stdbool.h>

void test_init_returns_zero_when_device_ready(void);
void test_init_fails_when_device_not_ready(void);
void test_set_power_saturates_and_calls_pwm(void);
void test_set_power_50_percent(void);
void test_set_power_zero(void);

#endif /* HEATER_TESTS_H_ */
