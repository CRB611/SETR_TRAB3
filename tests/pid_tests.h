/**
 * @file pid_tests.h
 * @brief Declaration of the unit tests for the PID controller module.
 * 
 * This file defines the prototypes of the test functions to the PID controller module.
 * These tests verify the correct implementation of the proportional, integral and derivative 
 * actions, as well as checking particular cases like null or negative terms, and the PID 
 * function calls sequence.
 */

#ifndef PID_TESTS_H
#define PID_TESTS_H

#include "unity.h"
#include "../modules/pid.h"


/** 
 * @brief Tests the default pid parameters.
 */
void test_default_pid_params(void);

/**
 * @brief Tests the contribution of the proportional component for the PID output.
 */
void test_proportional_action(void);

/**
 * @brief Tests the contribution of the integral component for the PID output.
 */
void test_integral_action(void);

/**
 * @brief Tests the contribution of the derivative component for the PID output.
 
 */
void test_derivative_action(void);

/**
 * @brief Tests the integral component reset.
 */
void test_integral_reset(void);

/**
 * @brief Tests the PID behaviour when the time interval (dt) is zero.
 * 
 * Its expected that the integral and  derivative components dont contribute to the output.
 */
void test_zero_dt_behaviour(void);

/**
 * @brief Tests the PID consistency when its called in cascade with diferent actions (P, PI, PID).
 */
void test_cascading_pid_steps(void);

/**
 * @brief Tests the initial derivative.
 * 
 * This initial derivative should be zero.
 */
void test_derivative_initial_zero(void);

/**
 * @brief Tests the PID behaviour when the time interval (dt) is negative.
 * 
 * It's expected to be treateas as if dt = 0, which means the integral and
 *  derivative components dont contribute to the output.
 */
void test_negative_dt_behaviour(void);



#endif // PID_TESTS_H
