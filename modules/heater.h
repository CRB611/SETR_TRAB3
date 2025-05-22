/** 
 * \file heater.h
 * \brief This file contains the defines and functions needed for the Heater related functions.
 * 
 * The heating system works together with the <i>heater.h</i> module to set the percentual power sent to the 
 * heating system through the PWM node.
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug There are no known bugs.
 */
#ifndef HEATER_H
#define HEATER_H

#include <zephyr/types.h>

/**
 * @brief Initializes the heater driver (PWM).
 *
 * Must be called one time in the begining; returns <0 in case of error.
 *
 * @return  0 in success
 * @return <0  Zephyr error code (e.g. -ENODEV)
 */
int heater_init(void);

/**
 * @brief Adjusts the heater power in percentage (0–100%).
 *
 * @param percent Duty-cycle in percentage: 0 = turned off, 100 = full on.
 */
void heater_set_power(uint8_t percent);

#endif /* HEATER_H */
