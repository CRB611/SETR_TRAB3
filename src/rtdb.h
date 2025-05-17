#ifndef RTDB_H
#define RTDB_H

#include <zephyr/types.h>
#include <stdbool.h>

/**
 * @brief Inicializa a RTDB.
 *
 * Deve ser chamada em main() antes de usar getters/setters.
 */
void rtdb_init(void);

/**
 * @brief Atualiza a temperatura atual na RTDB.
 *
 * @param t Valor em °C lido do TC74.
 */
void rtdb_set_cur_temp(uint8_t t);

/**
 * @brief Lê a temperatura atual guardada na RTDB.
 *
 * @return Temperatura mais recente (°C).
 */
uint8_t rtdb_get_cur_temp(void);

/**
 * @brief Sinaliza um erro na leitura do TC74.
 *
 * @param err true se ocorreu erro, false para limpar o flag.
 */
void rtdb_set_error_flag(bool err);

/**
 * @brief Lê o flag de erro.
 *
 * @return true se houve erro, false caso contrário.
 */
bool rtdb_get_error_flag(void);

// PWM

/**
 * @brief Define o setpoint desejado (°C).
 */
void    rtdb_set_setpoint(int16_t sp);

/**
 * @brief Obtém o setpoint atual (°C).
 */
int16_t rtdb_get_setpoint(void);

/**
 * @brief Liga/desliga o sistema (heater).
 */
void    rtdb_set_system_on(bool on);

/**
 * @brief Indica se o sistema está ligado.
 */
bool    rtdb_get_system_on(void);

#endif /* RTDB_H */



