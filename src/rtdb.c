#include "rtdb.h"
#include <zephyr/kernel.h>

/* Mutex que protege todas as variáveis da RTDB */
static struct k_mutex rtdb_mutex;

/* Variáveis partilhadas da RTDB */
static uint8_t  cur_temp   = 0;
static bool     error_flag = false;
static int16_t  setpoint   = 40;   /* Default = 40°C */
static bool     system_on  = true; /* Sistema ligado por defeito */

void rtdb_init(void)
{
    k_mutex_init(&rtdb_mutex);
    cur_temp   = 0;
    error_flag = false;
    setpoint   = 60;
    system_on  = true;
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

void rtdb_set_setpoint(int16_t sp)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    setpoint = sp;
    k_mutex_unlock(&rtdb_mutex);
}

int16_t rtdb_get_setpoint(void)
{
    int16_t v;
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    v = setpoint;
    k_mutex_unlock(&rtdb_mutex);
    return v;
}

void rtdb_set_system_on(bool on)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    system_on = on;
    k_mutex_unlock(&rtdb_mutex);
}

bool rtdb_get_system_on(void)
{
    bool v;
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    v = system_on;
    k_mutex_unlock(&rtdb_mutex);
    return v;
}
