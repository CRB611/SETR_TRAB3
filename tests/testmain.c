/**
 * @file testmain.c
 * @brief Main file where the UNITY tests are executed
 *
 */

#include "unity.h"
#include <stdio.h>
#include "pid_tests.h"
#include "heater_tests.h"
#include "uart_tests.h"

// protótipos dos testes

extern void test_default_values(void);
extern void test_set_get_pid(void);
extern void test_set_get_temp(void);
extern void test_default_values_without_init(void);
extern void test_default_error_flag(void);
extern void test_set_get_error_flag(void);
extern void test_default_system_off(void);
extern void test_set_get_system_on(void);
extern void test_default_setpoint(void);
extern void test_set_get_setpoint(void);
extern void test_max_temp_boundaries(void);
extern void test_full_reset(void);



int main(void) {
    UNITY_BEGIN();

    setUp();
    printf("\nTestes para UART\n");
    printf("\n");
    RUN_TEST(test_num2char);
    RUN_TEST(test_char2num);
    RUN_TEST(test_char2float);
    RUN_TEST(test_calcChecksum);
    RUN_TEST(test_process_set_max_temp_ok);
    RUN_TEST(test_process_set_max_temp_too_hot);
    RUN_TEST(test_process_get_current_temp);
    RUN_TEST(test_process_invalid_checksum);
    RUN_TEST(test_process_invalid_command);
    RUN_TEST(run_uart_tests);

    printf("\nTestes para RTDB\n");
    printf("\n");
     setUp();
   // RUN_TEST(test_default_values_without_init);
    RUN_TEST(test_default_values);
    //RUN_TEST(test_set_get_pid);
    //RUN_TEST(test_set_get_temp);
    RUN_TEST(test_default_error_flag);
    RUN_TEST(test_set_get_error_flag);
    RUN_TEST(test_default_system_off);
    RUN_TEST(test_set_get_system_on);
    RUN_TEST(test_default_setpoint);
    RUN_TEST(test_set_get_setpoint);
    RUN_TEST(test_max_temp_boundaries);
    RUN_TEST(test_full_reset);
    printf("\nTestes para PID\n");
    printf("\n");
    RUN_TEST(test_default_pid_params);
    RUN_TEST(test_proportional_action);
    RUN_TEST(test_integral_action);
    RUN_TEST(test_derivative_action);
   /* RUN_TEST(test_integral_reset);*/
    RUN_TEST(test_zero_dt_behaviour);
    RUN_TEST(test_cascading_pid_steps);
    RUN_TEST(test_derivative_initial_zero);
    RUN_TEST(test_negative_dt_behaviour);


    printf("\nTestes para HEATER\n");
    printf("\n");
     setUp();
    RUN_TEST(test_init_returns_zero_when_device_ready);
    RUN_TEST(test_init_fails_when_device_not_ready);
    RUN_TEST(test_set_power_saturates_and_calls_pwm);
    RUN_TEST(test_set_power_50_percent);
    RUN_TEST(test_set_power_zero);
    
    tearDown();

    return UNITY_END();
}
