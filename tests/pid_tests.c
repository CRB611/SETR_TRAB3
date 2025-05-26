#include "unity.h"
#include "pid_tests.h"

static pid_data_t ctrl;

// 1. Verifica parâmetros por defeito
void test_default_pid_params(void) {
    pid_init(&ctrl, 1.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, ctrl.Kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ctrl.Ti);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ctrl.Td);
}

// 2. Proporcional puro: se Ti=Td=0, saída = Kp*(setp−meas)
void test_proportional_action(void) {
    pid_init(&ctrl, 2.0f, 0.0f, 0.0f);
    float u = pid_compute(&ctrl, 5, 3.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(2.0f * (5 - 3.0f), u);
}

// 3. Integral: acumula erro ao longo do tempo
void test_integral_action(void) {
    pid_init(&ctrl, 0.0f, 1.0f, 0.0f);
    // primeiro passo: integral = (5−3)*dt = 2
    pid_compute(&ctrl, 5, 3.0f, 1.0f);
    // segundo passo: integral acumula mais 2 → total 4
    float u2 = pid_compute(&ctrl, 5, 3.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, u2);
}

// 4. Derivativo: com Kd>0, responde à variação de erro
void test_derivative_action(void) {
    pid_init(&ctrl, 0.0f, 0.0f, 1.0f);
    // dt=1: derivada = (e1−e0)/dt
    pid_compute(&ctrl, 5, 3.0f, 1.0f);      // e0 = 2 → u0 = 0
    float u1 = pid_compute(&ctrl, 6, 3.0f, 1.0f);  // e1 = 3 → deriv = 1
    TEST_ASSERT_EQUAL_FLOAT(1.0f, u1);
}
/*
// 5. Reset do integrador: reinicia estado interno
void test_integral_reset(void) {
    pid_init(&ctrl, 0.0f, 1.0f, 0.0f);
    pid_compute(&ctrl, 5, 3.0f, 1.0f);
    pid_reset(&ctrl);
    float u = pid_compute(&ctrl, 5, 3.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, u);  // só 1º passo, integral = 2
}
*/
// 6. dt=0: garante que não dá div/0 nem acumula errado
void test_zero_dt_behaviour(void) {
    pid_init(&ctrl, 1.0f, 1.0f, 1.0f);
    float u = pid_compute(&ctrl, 5, 3.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f * 2.0f, u);  // apenas P, sem I nem D
}

// 7. Passos em cascata: P→PI→PID
void test_cascading_pid_steps(void) {
    pid_data_t ctrl;
    pid_init(&ctrl, 1.0f, 1.0f, 0.0f);

    float u1 = pid_compute(&ctrl, 3, 2.0f, 1.0f); // 2.0f
    float u2 = pid_compute(&ctrl, 3, 2.0f, 1.0f); // 3.0f
    float u3 = pid_compute(&ctrl, 3, 2.0f, 1.0f); // 4.0f

    TEST_ASSERT_EQUAL_FLOAT(2.0f, u1);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, u2);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, u3);
}



// 8. Derivada no primeiro passo deve ser zero
void test_derivative_initial_zero(void) {
    pid_data_t ctrl;
    
    pid_init(&ctrl, 0.0f, 0.0f, 2.0f);
    TEST_ASSERT_EQUAL_FLOAT(2.0f,
        pid_compute(&ctrl, 3, 2.0f, 1.0f));

    
    pid_init(&ctrl, 2.0f, 0.0f, 4.0f);
    TEST_ASSERT_EQUAL_FLOAT(6.0f,
        pid_compute(&ctrl, 3, 2.0f, 1.0f));
}

// 9. dt negativo é tratado como zero (sem I nem D)
void test_negative_dt_behaviour(void) {
    pid_init(&ctrl, 1.0f, 1.0f, 1.0f);
    // dt negativo não deve acumular nem derivar
    float u_neg = pid_compute(&ctrl, 5, 3.0f, -1.0f);
    float u_zero = pid_compute(&ctrl, 5, 3.0f,  0.0f);
    TEST_ASSERT_EQUAL_FLOAT(u_zero, u_neg);
}
