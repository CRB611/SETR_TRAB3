/**
 * @file zephyr/logging/log.h
 * @brief Stub minimal para compilar em host com UNIT_TEST
 */
#ifndef ZEPHYR_LOGGING_LOG_H
#define ZEPHYR_LOGGING_LOG_H

#include <stdio.h>

/**
 * @brief Macro para registar o módulo de logging (stub vazio).
 */
#define LOG_MODULE_REGISTER(name, level)

/**
 * @brief Nível de logging por defeito (stub).
 */
#define CONFIG_LOG_DEFAULT_LEVEL 0

/**
 * @brief Macro para erro de logging.
 */
#define LOG_ERR(fmt, ...)   printf("ERROR: " fmt "\n", ##__VA_ARGS__)

/**
 * @brief Macro para warning de logging.
 */
#define LOG_WRN(fmt, ...)   printf("WARN: " fmt "\n", ##__VA_ARGS__)

#endif /* ZEPHYR_LOGGING_LOG_H */
