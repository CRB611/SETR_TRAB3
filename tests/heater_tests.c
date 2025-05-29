#include "unity.h"
#include "../modules/heater.h"
#include "../stubs/zephyr/drivers/pwm.h"
#include "../stubs/zephyr/device.h"

/* Limpar estado antes de cada teste */
 void setUp(void) {

    last_pwm_dev    = NULL;
    last_pwm_chan   = (uint32_t)-1;
    last_pwm_period = (uint32_t)-1;
    last_pwm_pulse  = (uint32_t)-1;
    last_pwm_flags  = -1;

   
    fake_dev_ready = true;
 }

void tearDown(void){}


/* 1) Inicialização bem sucedida */
void test_init_returns_zero_when_device_ready(void) {
    fake_dev_ready = true;
    int rc = heater_init();
    TEST_ASSERT_EQUAL_INT(0, rc);
}

/* 2) Inicialização falha */
void test_init_fails_when_device_not_ready(void) {
    fake_dev_ready = false;
    int rc = heater_init();
    TEST_ASSERT_LESS_THAN_INT(0, rc); /* devolve -ENODEV */
}

/* 3) Limita potência acima de 100% e chama pwm_set */
void test_set_power_saturates_and_calls_pwm(void) {
    /* força potência >100 */
    heater_set_power(150);
    TEST_ASSERT_NOT_NULL(last_pwm_dev);
    TEST_ASSERT_EQUAL_INT(HEATER_PWM_CHANNEL, last_pwm_chan);
    TEST_ASSERT_EQUAL_UINT32(HEATER_PWM_PERIOD_NS, last_pwm_period);
    /* pulse = period * 100 / 100 */
    TEST_ASSERT_EQUAL_UINT32(HEATER_PWM_PERIOD_NS, last_pwm_pulse);
}

/* 4) Potência intermédia gera pulse certo */
void test_set_power_50_percent(void) {
    heater_set_power(50);
    TEST_ASSERT_EQUAL_UINT32(HEATER_PWM_PERIOD_NS / 2, last_pwm_pulse);
}

/* 5) Potência zero gera pulse zero */
void test_set_power_zero(void) {
    heater_set_power(0);
    TEST_ASSERT_EQUAL_UINT32(0U, last_pwm_pulse);
}

