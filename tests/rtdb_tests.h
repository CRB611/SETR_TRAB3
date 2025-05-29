/**
 * @file rtdb_tests.h
 * @brief Declaration of the unit tests for the RTDB (Real-Time Database) module.
 * 
 * This file contains the declarations of the test functions that validate the RTDB module
 * behaviour.
 * It uses UNITY framework to test default values, and returning an setting parameters like
 * the PID, temperatures (max, current and desired), error flags, and setpoint.
 */
#ifndef TEST_RTDB_H
#define TEST_RTDB_H

#include "unity.h"
#include <stdint.h>



/** 
 * @brief Test the values without init.
 */
void test_default_values_without_init(void);

/** 
 * @brief Test the values after init.
 */
void test_default_values(void);

/** 
 * @brief Tests the default error flag.
 */
void test_default_error_flag(void);

/** 
 * @brief Tests the error flag activation and deactivation.
 */
void test_set_get_error_flag(void);

/** 
 * @brief Tests the default system state.
 */
void test_default_system_off(void);

/** 
 * @brief Tests the system state change.
 */
void test_set_get_system_on(void);

/** 
 * @brief Tests the default setpoint.
 */
void test_default_setpoint(void);

/** 
 * @brief Tests the setting and returning of the setpoint.
 */
void test_set_get_setpoint(void);

/** 
 * @brief Tests the max temperature upper and lower boundaries.
 */
void test_max_temp_boundaries(void);

/** 
 * @brief Tests the system reset.
 */
void test_full_reset(void);



#endif // TEST_RTDB_H
