/* stubs/fake_pwm.c */

#include <stddef.h>            /* para NULL */
#include <stdbool.h>
#include "zephyr/device.h"
#include "zephyr/devicetree.h"
#include "zephyr/drivers/pwm.h"

/* flag que o setUp() dos testes manipula */
bool fake_dev_ready = true;

/* o dispositivo “falso” que DEVICE_DT_GET() devolve */
struct device fake_device = { .name = "fake_pwm0" };

/* as variáveis que o stub de pwm_set() preenche */
const struct device *last_pwm_dev    = NULL;
uint32_t             last_pwm_chan   = 0;
uint32_t             last_pwm_period = 0;
uint32_t             last_pwm_pulse  = 0;
int                  last_pwm_flags  = 0;
