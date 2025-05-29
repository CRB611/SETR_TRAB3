/**
 * @file heater_tests.h
 * @brief Prototypes and documentation of the heater module tests using UNITY 
 *
 * This file has the declaration of the test functionc for the heaters module 
 * using the UNITY framework.
 */

#ifndef HEATER_TESTS_H_
#define HEATER_TESTS_H_

#include "unity.h"
#include <stdbool.h>

/**
 * @brief Tests a successfull heater initialization
 */
void test_init_returns_zero_when_device_ready(void);

/**
 * @brief Tests a failed heater initialization
 */
void test_init_fails_when_device_not_ready(void);

/**
 * @brief Tests the power saturation and the PWM
 */
void test_set_power_saturates_and_calls_pwm(void);

/**
 * @brief Tests the PWM at 50%
 */
void test_set_power_50_percent(void);

/**
 * @brief Tests the PWM at 0%
 */
void test_set_power_zero(void);

#endif /* HEATER_TESTS_H_ */