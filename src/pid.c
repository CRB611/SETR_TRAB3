#include "pid.h"

void pid_init(pid_data_t *pid, float Kp, float Ti, float Td)
{
    pid->Kp = Kp;
    pid->Ti = Ti;
    pid->Td = Td;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_set(pid_data_t *pid, float Kp, float Ti, float Td)
{
    pid->Kp = Kp;
    pid->Ti = Ti;
    pid->Td = Td;
}

float pid_compute(pid_data_t *pid, int16_t setp, float meas, float dt_s)
{
    if (dt_s <= 0.0f) {
        return 0.0f;
    }

    float error = (float)setp - meas;

    /* Termo Integral com anti-windup */
    if (pid->Ti > 0.0f) {
        pid->integral += error * dt_s;
        /* Saturação da integral para evitar windup */
        float max_int = 100.0f * pid->Ti;
        if (pid->integral > max_int) {
            pid->integral = max_int;
        } else if (pid->integral < -max_int) {
            pid->integral = -max_int;
        }
    }

    /* Termo Derivativo */
    float deriv = 0.0f;
    if (pid->Td > 0.0f) {
        deriv = (error - pid->prev_error) / dt_s;
    }
    pid->prev_error = error;

    /* Saída PID antes de saturação */
    float output = pid->Kp * error;
    if (pid->Ti > 0.0f) {
        output += pid->integral / pid->Ti;
    }
    if (pid->Td > 0.0f) {
        output += pid->Td * deriv;
    }

    /* Saturar saída entre 0 e 100% */
    if (output > 100.0f) {
        output = 100.0f;
    } else if (output < 0.0f) {
        output = 0.0f;
    }

    return output;
}
