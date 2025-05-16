/**
 * @file tc74.h
 * @brief Interface para o sensor de temperatura TC74 via I²C.
 *
 * Este módulo fornece funções para inicializar o sensor TC74 e ler
 * a temperatura através do barramento I²C, utilizando a abstração 
 * do Zephyr DeviceTree.
 */

#ifndef TC74_H
#define TC74_H

#include <zephyr/drivers/i2c.h>

/**
 * @def TC74_CMD_RTR
 * @brief Registo interno do TC74 que contém a temperatura atual.
 *
 * Deve ser enviado antes da leitura para colocar o ponteiro 
 * de leitura no registo de temperatura.
 */
#define TC74_CMD_RTR  0x00

/**
 * @def TC74_CMD_RWCR
 * @brief Registo de configuração do TC74.
 *
 * Permite ler ou escrever o registo de configuração do sensor.
 */
#define TC74_CMD_RWCR 0x01

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa o sensor TC74.
 *
 * Verifica se o barramento I²C está pronto para comunicação e envia 
 * uma vez o comando @p TC74_CMD_RTR para assegurar que o ponteiro de 
 * leitura está no registo de temperatura.
 *
 * @return 0 em caso de sucesso, valor negativo (<0) em caso de erro.
 */
int tc74_init(void);

/**
 * @brief Lê a temperatura atual do sensor TC74.
 *
 * Executa uma leitura de 1 byte do registo de temperatura e converte
 * diretamente para graus Celsius (valor inteiro).
 *
 * @param[out] out_temp Ponteiro para variável onde será armazenada
 *                      a temperatura lida (°C).
 * @return 0 em caso de sucesso, valor negativo (<0) em caso de erro.
 */
int tc74_read(uint8_t *out_temp);

#ifdef __cplusplus
}
#endif

#endif /* TC74_H */
