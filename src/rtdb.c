#include "rtdb.h"
#include <zephyr/kernel.h>

/* Mutex para proteger todas as variáveis da RTDB */
static struct k_mutex rtdb_mutex;

/* Variáveis partilhadas da RTDB */
static uint8_t cur_temp   = 0;
static bool    error_flag = false;

void rtdb_init(void)
{
    k_mutex_init(&rtdb_mutex);
    cur_temp   = 0;
    error_flag = false;
}

void rtdb_set_cur_temp(uint8_t t)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    cur_temp = t;
    k_mutex_unlock(&rtdb_mutex);
}

uint8_t rtdb_get_cur_temp(void)
{
    uint8_t t;
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    t = cur_temp;
    k_mutex_unlock(&rtdb_mutex);
    return t;
}

void rtdb_set_error_flag(bool err)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    error_flag = err;
    k_mutex_unlock(&rtdb_mutex);
}

bool rtdb_get_error_flag(void)
{
    bool e;
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    e = error_flag;
    k_mutex_unlock(&rtdb_mutex);
    return e;
}
