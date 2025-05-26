/** 
 * \file rtdb.h
 * \brief Defines and functions for the Real-Time Database (RTDB).
 *
 * The RTDB stores all the runtime parameters and state of the system:
 *  - Current temperature (curTemp)
 *  - Desired setpoint (setPoint)
 *  - Maximum safety temperature (maxTemp)
 *  - Error flag (errorFlag)
 *  - System ON/OFF state (systemOn)
 *  - PID parameters (Kp, Ti, Td)
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug No known bugs.
 */
#ifndef RTDB_H
#define RTDB_H

 // #include <zephyr/types.h>
#include <stdbool.h>
#include <stdint.h>
/**
 * \brief PID parameters container.
 */
typedef struct {
    float Kp;  ///< Proportional gain
    float Ti;  ///< Integral time (s); if zero, integral inactive
    float Td;  ///< Derivative time (s); if zero, derivative inactive
} rtdb_pid;

/**
 * @brief Initializes the RTDB.
 *
 * Must be called in main() before using any getters or setters.
 */
void rtdb_init(void);

/* -------------------- Current temperature -------------------- */
/**
 * @brief Updates the current temperature in the RTDB.
 *
 * @param t Temperature read from the TC74 sensor (°C).
 */
void    rtdb_set_cur_temp(uint8_t t);

/**
 * @brief Returns the current temperature from the RTDB.
 *
 * @return Current temperature (°C).
 */
uint8_t rtdb_get_cur_temp(void);

/* -------------------- Error flag -------------------- */
/**
 * @brief Sets or clears the error flag.
 *
 * @param err true to signal an error, false to clear it.
 */
void    rtdb_set_error_flag(bool err);

/**
 * @brief Returns the current error flag.
 *
 * @return true if an error is signaled, false otherwise.
 */
bool    rtdb_get_error_flag(void);

/* -------------------- Setpoint -------------------- */
/**
 * @brief Defines the desired setpoint (°C).
 *
 * @param sp Desired temperature setpoint.
 */
void    rtdb_set_setpoint(int16_t sp);

/**
 * @brief Returns the current setpoint (°C).
 *
 * @return Current temperature setpoint.
 */
int16_t rtdb_get_setpoint(void);

/* -------------------- Maximum safety temperature -------------------- */
/**
 * @brief Defines the maximum allowed temperature (°C).
 *
 * @param mt Maximum safety temperature.
 */
void    rtdb_set_maxtemp(uint8_t mt);

/**
 * @brief Returns the current maximum safety temperature (°C).
 *
 * @return Maximum safety temperature.
 */
uint8_t rtdb_get_maxtemp(void);

/* -------------------- System ON/OFF -------------------- */
/**
 * @brief Turns the system ON or OFF.
 *
 * @param on true = system ON, false = system OFF.
 */
void    rtdb_set_system_on(bool on);

/**
 * @brief Returns the current system state.
 *
 * @return true if system is ON, false if OFF.
 */
bool    rtdb_get_system_on(void);

/* -------------------- PID parameters -------------------- */
/**
 * @brief Sets the PID parameters.
 *
 * @param kp Proportional gain.
 * @param ti Integral time (s).
 * @param td Derivative time (s).
 */
void    rtdb_set_pid(float kp, float ti, float td);

/**
 * @brief Returns the current PID parameters.
 *
 * @return A struct containing Kp, Ti, Td.
 */
rtdb_pid rtdb_get_pid(void);

/* -------------------- Utilities -------------------- */
/**
 * @brief Prints the entire RTDB contents to the console.
 */
void    rtdb_print(void);

/**
 * @brief Resets adjustable parameters to their default values.
 *
 * This resets setpoint, maxTemp, PID parameters, and error flag.
 */
void    rtdb_reset(void);

#endif /* RTDB_H */
