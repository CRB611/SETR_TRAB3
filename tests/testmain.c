#include "../unity/unity.h"
#include "uart_tests.h"

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_num2char);
    RUN_TEST(test_char2num);
    RUN_TEST(test_char2float);
    RUN_TEST(test_checksum);
    /*RUN_TEST(test_commands);
    RUN_TEST(test_command_errors);*/
    return UNITY_END();
}
