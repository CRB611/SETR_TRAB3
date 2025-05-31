/** 
 * \file heater.c
 * \brief This file contains the implementation of the functions implemented in heater.h.
 *
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug There are no known bugs.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include "heater.h"
#include <errno.h>


/* Obter o device a partir do Devicetree */
static const struct device *heater_pwm_dev = DEVICE_DT_GET(HEATER_PWM_NODE);

int heater_init(void)
{
    heater_set_power(0);
    if (!device_is_ready(heater_pwm_dev)) {
        printk("Erro: PWM device '%s' não está pronto\n",
               heater_pwm_dev->name);
        return -ENODEV;
    }
    return 0;
}

void heater_set_power(uint8_t percent)
{
    /*Impedir a saturaçao da percentagem*/
    if (percent > 100U) {
        percent = 100U;
    }

    printk("PWM duty = %u%%\n", percent);

    /*calcular o pulso*/
    uint32_t pulse_ns = (HEATER_PWM_PERIOD_NS * percent) / 100U;

    /*definir o pwm*/
    int ret = pwm_set(heater_pwm_dev,
                      HEATER_PWM_CHANNEL,
                      HEATER_PWM_PERIOD_NS,
                      pulse_ns,
                      PWM_POLARITY_NORMAL);
    if (ret < 0) {
        printk("Erro a definir PWM (canal %u): %d\n",
               HEATER_PWM_CHANNEL, ret);
    }
}
