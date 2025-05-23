#include "../unity/unity.h"
#include "uart_tests.h"

int main(void)
{
        setUp();
        UNITY_BEGIN();

        printf("tests");
        
        RUN_TEST_TRACKED(test_num2char);
        RUN_TEST_TRACKED(test_char2float);
        RUN_TEST_TRACKED(test_char2num);
        RUN_TEST_TRACKED(test_checksum);
    

        return 0;
}
