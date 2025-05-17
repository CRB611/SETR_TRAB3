/* heater.h */

#ifndef HEATER_H
#define HEATER_H

#include <zephyr/types.h>

/**
 * @brief Inicializa o driver do aquecedor (PWM).
 *
 * Deve ser chamado uma vez no início; devolve <0 em caso de erro.
 *
 * @return  0 em sucesso
 * @return <0 código de erro Zephyr (e.g. -ENODEV)
 */
int heater_init(void);

/**
 * @brief Ajusta a potência do aquecedor em percentual (0–100%).
 *
 * @param percent Duty-cycle em porcentagem: 0 = desligado, 100 = full on.
 */
void heater_set_power(uint8_t percent);

#endif /* HEATER_H */
