#include "unity.h"

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

int main(void) {
    UNITY_BEGIN();

    // UART
    RUN_TEST(test_num2char);
    RUN_TEST(test_char2num);
    RUN_TEST(test_char2float);
    RUN_TEST(test_checksum);

    // RTDB
    RUN_TEST(test_default_values);
    RUN_TEST(test_set_get_pid);
    RUN_TEST(test_set_get_temp);

    return UNITY_END();
}
