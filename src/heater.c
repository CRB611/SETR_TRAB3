/* heater.c */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include "heater.h"

/* O nó pwm0 deve estar ativo no overlay */
#define HEATER_PWM_NODE     DT_NODELABEL(pwm0)
#define HEATER_PWM_CHANNEL  0U

/* 1 kHz → 1 ms = 1 000 000 ns */
#define HEATER_PWM_PERIOD_NS 1000000U

/* Obter o device a partir do Devicetree */
static const struct device *heater_pwm_dev = DEVICE_DT_GET(HEATER_PWM_NODE);

int heater_init(void)
{
    if (!device_is_ready(heater_pwm_dev)) {
        printk("Erro: PWM device '%s' não está pronto\n",
               heater_pwm_dev->name);
        return -ENODEV;
    }
    return 0;
}

void heater_set_power(uint8_t percent)
{
    if (percent > 100U) {
        percent = 100U;
    }

  //  printk("PWM duty = %u%%\n", percent);

    uint32_t pulse_ns = (HEATER_PWM_PERIOD_NS * percent) / 100U;

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
