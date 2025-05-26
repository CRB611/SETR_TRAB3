#include "unity.h"
#include "../modules/rtdb.h"
#include <stdio.h>

/*
void setUp(void) {
    // garante que partimos dos valores default
    rtdb_reset();
}

void tearDown(void) {
    // nada
}
*/
void test_default_values(void) {
    rtdb_init();
    rtdb_pid p = rtdb_get_pid();

    /* imprime no stdout para veres durante o run 
    //printf("RTDB defaults: Kp=%.2f, Ti=%.2f, Td=%.2f\n",
           p.Kp, p.Ti, p.Td);*/

    TEST_ASSERT_EQUAL_FLOAT(3.0f, p.Kp);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, p.Ti);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.Td);

    uint8_t t = rtdb_get_maxtemp();
    /*printf("RTDB defaults: maxTemp=%u\n", t);*/
    TEST_ASSERT_EQUAL_UINT8(90, t);
}


void test_set_get_pid(void) {
    rtdb_set_pid(2.5f, 1.2f, 0.3f);
    rtdb_pid p = rtdb_get_pid();
    TEST_ASSERT_EQUAL_FLOAT(2.5f, p.Kp);
    TEST_ASSERT_EQUAL_FLOAT(1.2f, p.Ti);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, p.Td);
}

void test_set_get_temp(void) {
    rtdb_set_cur_temp(42);
    uint8_t t = rtdb_get_cur_temp();
    TEST_ASSERT_EQUAL_UINT8(42, t);
}
