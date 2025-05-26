#include "unity.h"
#include "../modules/rtdb.h"
#include <stdint.h>


void test_default_values_without_init(void) {
    
    rtdb_pid p = rtdb_get_pid();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.Kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.Ti);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.Td);
}

void test_default_values(void) {
    rtdb_init();
    rtdb_pid p = rtdb_get_pid();
    TEST_ASSERT_EQUAL_FLOAT(3.0f, p.Kp);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, p.Ti);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.Td);

    uint8_t t = rtdb_get_maxtemp();
    TEST_ASSERT_EQUAL_UINT8(90, t);
}



// 1. Error flag
void test_default_error_flag(void) {
    TEST_ASSERT_FALSE(rtdb_get_error_flag());
}

void test_set_get_error_flag(void) {
    rtdb_set_error_flag(true);
    TEST_ASSERT_TRUE(rtdb_get_error_flag());
    rtdb_set_error_flag(false);
    TEST_ASSERT_FALSE(rtdb_get_error_flag());
}

// 2. System on/off
void test_default_system_off(void) {
    TEST_ASSERT_FALSE(rtdb_get_system_on());
}

void test_set_get_system_on(void) {
    rtdb_set_system_on(false);
    TEST_ASSERT_FALSE(rtdb_get_system_on());
    rtdb_set_system_on(true);
    TEST_ASSERT_TRUE(rtdb_get_system_on());
}

// 3. Setpoint
void test_default_setpoint(void) {
    uint8_t def = 35 ;
    TEST_ASSERT_EQUAL_UINT8(def, rtdb_get_setpoint());
}

void test_set_get_setpoint(void) {
    rtdb_set_setpoint(75);
    TEST_ASSERT_EQUAL_UINT8(75, rtdb_get_setpoint());
    rtdb_set_setpoint(0);
    TEST_ASSERT_EQUAL_UINT8(0, rtdb_get_setpoint());
}

// 4. MaxTemp boundaries
void test_max_temp_boundaries(void) {
    rtdb_set_maxtemp(0);
    TEST_ASSERT_EQUAL_UINT8(0, rtdb_get_maxtemp());

    rtdb_set_maxtemp(255);
    TEST_ASSERT_EQUAL_UINT8(255, rtdb_get_maxtemp());

    // Se o teu módulo validar limites, teste os valores fora de alcance aqui.
}

// 5. Reset completo
void test_full_reset(void) {
    // altera tudo
    rtdb_set_pid(2.2f, 3.3f, 4.4f);
    rtdb_set_cur_temp(99);
    rtdb_set_error_flag(true);
    rtdb_set_system_on(true);
    rtdb_set_setpoint(88);
    rtdb_set_maxtemp(77);

    // faz reset
    rtdb_reset();

    // confirma defaults
    rtdb_pid p = rtdb_get_pid();
    TEST_ASSERT_EQUAL_FLOAT(3.0f, p.Kp);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, p.Ti);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.Td);

    TEST_ASSERT_EQUAL_UINT8(0, rtdb_get_cur_temp());
    TEST_ASSERT_FALSE(rtdb_get_error_flag());
    TEST_ASSERT_FALSE(rtdb_get_system_on());
    TEST_ASSERT_EQUAL_UINT8(35, rtdb_get_setpoint());
    TEST_ASSERT_EQUAL_UINT8(90,  rtdb_get_maxtemp());
}

