#ifndef ZEPHYR_DRIVERS_PWM_H
#define ZEPHYR_DRIVERS_PWM_H
#include <stdint.h>

/* polaridade – só para compilar */
#define PWM_POLARITY_NORMAL 0

/* stub de pwm_set: guarda os parâmetros em variáveis globais */
extern const struct device *last_pwm_dev;
extern uint32_t         last_pwm_chan;
extern uint32_t         last_pwm_period;
extern uint32_t         last_pwm_pulse;
extern int              last_pwm_flags;

static inline int pwm_set(const struct device *dev,
                          uint32_t chan,
                          uint32_t period,
                          uint32_t pulse,
                          int flags)
{
    last_pwm_dev    = dev;
    last_pwm_chan   = chan;
    last_pwm_period = period;
    last_pwm_pulse  = pulse;
    last_pwm_flags  = flags;
    return 0;
}

#endif /* ZEPHYR_DRIVERS_PWM_H */
