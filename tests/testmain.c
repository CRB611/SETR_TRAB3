#include "unity.h"
#include <stdio.h>
// protótipos dos testes
extern void test_num2char(void);
extern void test_char2num(void);
extern void test_char2float(void);
extern void test_checksum(void);
extern void test_commands(void);
extern void test_command_errors(void);

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

    printf("\nTestes para UART\n");
    printf("\n");
    RUN_TEST(test_num2char);
    RUN_TEST(test_char2num);
    RUN_TEST(test_char2float);
    RUN_TEST(test_checksum);

    printf("\nTestes para RTDB\n");
    printf("\n");
    RUN_TEST(test_default_values_without_init);
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

    return UNITY_END();
}
