/** 
 * \file rtdb.h
 * \brief This file contains the defines and functions needed for the RTDB related functions.
 *
 * The Real-Time Database contains has the values required to run the program.
 * These values are stored so that the internal real-time tasks can be
 *  it synchronized with the I/O interfaces.
 * 
 * Values stored:
 *      <ul>
            <li><i>Current temperature:</i> Current temperature being mesured by the heat sensor; </li>
            <li><i>Current setpoint:</i> Temperature setpoint, default is 35ºC; </li>
            <li><i>Max temperature</i> Max allowed temperature, if the system reaches this temperature an error will be issued, default is 100ºC </li>
            <li><i>Error flag</i> If theres an error flag on. </li>
            <li><i>System State</i> Tells if the system is on or off. </li>
        </ul>
 * 
 * 
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug There are no known bugs.
 */
#ifndef RTDB_H
#define RTDB_H

#include <zephyr/types.h>
#include <stdbool.h>

/**
 * @brief Initializes the RTDB.
 *
 * Must be called in main() before using getters/setters.
 */
void rtdb_init(void);

/**
 * @brief Updates the current temperature in the RTDB.
 *
 * @param t Value read from the TC74 sensor in °C.
 */
void rtdb_set_cur_temp(uint8_t t);

/**
 * @brief Returns the current temperature in the RTDB.
 *
 * @return Current temperature (°C).
 */
uint8_t rtdb_get_cur_temp(void);

/**
 * @brief Signalizes an error in the TC74.
 *
 * @param err True if an error occured, false to clean the flag.
 */
void rtdb_set_error_flag(bool err);

/**
 * @brief Reads the error flag.
 *
 * @return true if error, false if theres no error.
 */
bool rtdb_get_error_flag(void);

// PWM

/**
 * @brief Defines the desired setpoint (°C).
 * 
 * @param sp Desired setpoint
 */
void    rtdb_set_setpoint(int16_t sp);

/**
 * @brief Returns the current setpoint (°C).
 * 
 * \return Current setpoint
 */
int16_t rtdb_get_setpoint(void);

/**
 * @brief Defines the max temperature (°C).
 * 
 * \param mt max temperature.
 */
void rtdb_set_maxtemp(uint8_t mt);

/**
 * @brief Returns the current max temperature (°C).
 * 
 * \return Current max temperature. 
 */
int16_t rtdb_get_maxtemp(void);

/**
 * @brief  Turns the system ON/OFF.
 * \param on on to turn on, off to turn off
 */
void    rtdb_set_system_on(bool on);

/**
 * @brief Returns the system state.
 * \return on if system in on, off is system is off.
 */
bool    rtdb_get_system_on(void);

#endif /* RTDB_H */



