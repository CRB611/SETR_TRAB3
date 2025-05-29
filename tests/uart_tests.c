#include "../unity/unity.h"
#include "../modules/uart.h"
#include "../modules/rtdb.h"  // stub para rtdb_get_cur_temp

// -----------------------------------------------------------------------------
// Testes aos utilitários
// -----------------------------------------------------------------------------

void test_num2char(void) {
    unsigned char result[3];
    unsigned char exp[3];

    /* Testa valor 23 -> "023" */
    num2char(result, 23);
    exp[0] = '0'; exp[1] = '2'; exp[2] = '3';
    TEST_ASSERT_EQUAL_CHAR_ARRAY(exp, result, 3);

    /* Testa valor 200 -> "200" */
    num2char(result, 200);
    exp[0] = '2'; exp[1] = '0'; exp[2] = '0';
    TEST_ASSERT_EQUAL_CHAR_ARRAY(exp, result, 3);

    /* Testa valor 5 -> "005" */
    num2char(result, 5);
    exp[0] = '0'; exp[1] = '0'; exp[2] = '5';
    TEST_ASSERT_EQUAL_CHAR_ARRAY(exp, result, 3);
}

void test_char2num(void) {
    unsigned char number[3] = {'1','2','3'};
    unsigned int val = char2num(number, 3);
    TEST_ASSERT_EQUAL_UINT(123, val);
}

void test_char2float(void) {
    unsigned char c1[5] = "30.4";
    unsigned char c2[5] = "09.2";
    unsigned char c3[5] = "53.0";

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.4f, char2float(c1));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.2f,  char2float(c2));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 53.0f, char2float(c3));
}

void test_calcChecksum(void) {
    unsigned char test1[4] = "M100";
    unsigned char test2[14] = "QUERTY1234abcd";
    int expected1 = ('M' + '1' + '0' + '0') % 256;
    int res1 = calcChecksum(test1, 4);
    TEST_ASSERT_EQUAL_INT(expected1, res1);

    int expected2 = 0;
    for (int i = 0; i < 14; i++) expected2 += test2[i];
    expected2 %= 256;
    int res2 = calcChecksum(test2, 14);
    TEST_ASSERT_EQUAL_INT(expected2, res2);
}

// -----------------------------------------------------------------------------
// Testes a process_uart_command()
// -----------------------------------------------------------------------------

void test_process_set_max_temp_ok(void) {
    char reply[16];
    // Comando #M050226!: checksum de 'M050' = 226
    int ret = process_uart_command("#M050226!", reply);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_EQUAL_CHAR('#', reply[0]);
    TEST_ASSERT_EQUAL_CHAR('E', reply[1]);
    TEST_ASSERT_EQUAL_CHAR('o', reply[2]);
    int chk = char2num((unsigned char*)&reply[3], 3);
    TEST_ASSERT_EQUAL_INT(calcChecksum((unsigned char*)"Eo", 2), chk);
}

void test_process_set_max_temp_too_hot(void) {
    char reply[16];
    // Comando #M130225!: checksum de 'M130' = 225
    int ret = process_uart_command("#M130225!", reply);
    TEST_ASSERT_EQUAL_INT(-1, ret);
    TEST_ASSERT_EQUAL_STRING("#Ei000!", reply);
}

void test_process_get_current_temp(void) {
    // Stub: garante que rtdb_get_cur_temp() retorna 42
    rtdb_set_cur_temp(42);
    char reply[16];
    // Comando #C000042105!: checksum de 'C000042' = 105
    int ret = process_uart_command("#C000042105!", reply);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_EQUAL_CHAR('#', reply[0]);
    TEST_ASSERT_EQUAL_CHAR('c', reply[1]);
    char valstr[4] = { reply[2], reply[3], reply[4], '\0' };
    TEST_ASSERT_EQUAL_STRING("042", valstr);
}

void test_process_invalid_checksum(void) {
    char reply[16];
    int ret = process_uart_command("#M050999!", reply);
    TEST_ASSERT_EQUAL_INT(-1, ret);
    TEST_ASSERT_EQUAL_CHAR('#', reply[0]);
    TEST_ASSERT_EQUAL_CHAR('E', reply[1]);
    TEST_ASSERT_EQUAL_CHAR('s', reply[2]);
}

void test_process_invalid_command(void) {
    char reply[16];
    // Comando #Z000000122!: checksum de 'Z000000' = 122
    int ret = process_uart_command("#Z000000122!", reply);
    TEST_ASSERT_EQUAL_INT(-1, ret);
    TEST_ASSERT_EQUAL_STRING("#Ei174!", reply);
}


