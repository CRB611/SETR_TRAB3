/** 
 * \file pid.h
 * \brief This file contains the defines and functions needed for the PID related functions.
 *
 *  This module computes the PID (proportional–integral–derivative) controller.
 * 
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug There are no known bugs.
 */
#ifndef PID_H
#define PID_H
#ifdef UNIT_TEST
/* stub vazio para host */
#else
#include <zephyr/types.h>
#endif
// #include <zephyr/types.h>

/**
 * @brief PID controller data structure.
 */
typedef struct {
    float Kp;        ///< Proportional Gain
    float Ti;        ///< Integral Term (s); if zero, integral inative */
    float Td;        ///< Derivative Term (s); if zero, derivative inative */
    float integral;  ///< Accumulated Integral Sum */
    float prev_error;///< Previous Iteration Error */
} pid_data_t;

/**
 * @brief Initializes the PID controller.
 *
 * @param pid   Pointer to the pid_data_t struct
 * @param Kp    Proportional gain
 * @param Ti    Integral Term in seconds (>=0)
 * @param Td    Derivative Term in seconds (>=0)
 */
void pid_init(pid_data_t *pid, float Kp, float Ti, float Td);

/**
 * @brief Computes the PID controller output.
 *
 * @param pid    Pointer to the pid_data_t struct
 * @param setp   Desired Setpoint (°C)
 * @param meas   Actual Value (°C)
 * @param dt_s   Time since the last call (s)
 * @return       Control Output (%) between 0 and 100
 */
float pid_compute(pid_data_t *pid, int16_t setp, float meas, float dt_s);

#endif /* PID_H */
