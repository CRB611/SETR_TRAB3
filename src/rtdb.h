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

#endif /* RTDB_H */
