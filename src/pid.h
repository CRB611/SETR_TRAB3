#ifndef PID_H
#define PID_H

#include <zephyr/types.h>

/**
 * @brief Estrutura de dados para controlador PID.
 */
typedef struct {
    float Kp;        /**< Ganho Proporcional */
    float Ti;        /**< Tempo Integral (s); se zero, integral inativa */
    float Td;        /**< Tempo Derivativo (s); se zero, derivada inativa */
    float integral;  /**< Soma integral acumulada */
    float prev_error;/**< Erro da iteração anterior */
} pid_data_t;

/**
 * @brief Inicializa o controlador PID.
 *
 * @param pid   Ponteiro para estrutura pid_data_t
 * @param Kp    Ganho proporcional
 * @param Ti    Tempo integral em segundos (>=0)
 * @param Td    Tempo derivativo em segundos (>=0)
 */
void pid_init(pid_data_t *pid, float Kp, float Ti, float Td);

/**
 * @brief Calcula a saída do controlador PID.
 *
 * @param pid    Ponteiro para estrutura pid_data_t
 * @param setp   Setpoint desejado (°C)
 * @param meas   Medição atual (°C)
 * @param dt_s   Intervalo de tempo desde a última chamada (s)
 * @return       Saída de controlo (%) entre 0 e 100
 */
float pid_compute(pid_data_t *pid, int16_t setp, float meas, float dt_s);

#endif /* PID_H */
