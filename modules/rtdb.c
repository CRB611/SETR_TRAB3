/**
 * \file rtdb.c
 * \brief Implementação das funções e variáveis do RTDB.
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug Não existem bugs conhecidos.
 */
#include "rtdb.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

/* Mutex que protege todas as variáveis da RTDB */
static struct k_mutex rtdb_mutex;

/* Variáveis partilhadas da RTDB */
static uint8_t   cur_temp   = 0;
static bool      error_flag = false;
static int16_t   setpoint   = 35;   /* Default = 35°C */
static bool      system_on  = false; /* Sistema desligado por defeito */
static uint8_t   max_temp   = 90;    /* Default = 90°C */
static rtdb_pid  PID;                /* Parâmetros PID */

/**
 * @brief Inicializa a RTDB com os valores por defeito.
 */
void rtdb_init(void)
{
    k_mutex_init(&rtdb_mutex);

    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    cur_temp   = 0;
    error_flag = false;
    setpoint   = 35;
    system_on  = false;
    max_temp   = 90;
    PID.Kp     = 3.0f;
    PID.Ti     = 30.0f;
    PID.Td     = 0.0f;
    k_mutex_unlock(&rtdb_mutex);
}

/* -------------------- Current temperature -------------------- */
void rtdb_set_cur_temp(uint8_t t)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    cur_temp = t;
    k_mutex_unlock(&rtdb_mutex);
    // faz sentido fazer um if t== null ?

}

uint8_t rtdb_get_cur_temp(void)
{
    uint8_t t;
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    t = cur_temp;
    k_mutex_unlock(&rtdb_mutex);
    return t;
}

/* -------------------- Error flag -------------------- */
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

/* -------------------- Setpoint -------------------- */
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

/* -------------------- Max temperature -------------------- */
void rtdb_set_maxtemp(uint8_t mt)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    max_temp = mt;
    k_mutex_unlock(&rtdb_mutex);
}

uint8_t rtdb_get_maxtemp(void)
{
    uint8_t v;
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    v = max_temp;
    k_mutex_unlock(&rtdb_mutex);
    return v;
}

/* -------------------- System ON/OFF -------------------- */
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

/* -------------------- PID parameters -------------------- */
void rtdb_set_pid(float kp, float ti, float td)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    PID.Kp = kp;
    PID.Ti = ti;
    PID.Td = td;
    k_mutex_unlock(&rtdb_mutex);
}

rtdb_pid rtdb_get_pid(void)
{
    rtdb_pid v;
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    v = PID;
    k_mutex_unlock(&rtdb_mutex);
    return v;
}

/* -------------------- Utilities -------------------- */
void rtdb_print(void)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    
    /*converter os floats para ints para poder dar print*/
    int Kp_int= floor(PID.Kp);
    int Kp_dec= (PID.Kp-Kp_int) *10;

    int Ti_int= floor(PID.Ti);
    int Ti_dec= (PID.Ti-Ti_int) *10;

     int Td_int= floor(PID.Td);
    int Td_dec= (PID.Td-Td_int) *10;


    printk("__________________________________\n|       VARIABLE         | VALUE |\n");
    printk("|   Current temperature  |%4d   |\n",cur_temp);           
    printk("|        Set Point       |%4d   |\n",setpoint);               
    printk("|     Max Temperature    | %4d  |\n",max_temp);                
    printk("|            Kp          |  %d.%d  |\n",Kp_int,Kp_dec);               
    printk("|            Ti          |  %d.%d |\n",Ti_int,Ti_dec);               
    printk("|            Td          |  %d.%d  |\n",Td_int,Td_dec);              
    printk("|        Error Flag      |%4d   |\n",error_flag);     
    printk("|      System State      |");
    if(system_on == true){
        printk("   on  |\n");              
    }else{
        printk("  off  |\n");        
    }
    printk("\\________________________________/\n");
    k_mutex_unlock(&rtdb_mutex);
}

void rtdb_reset(void)
{
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    cur_temp   = 0;
    error_flag = false;
    setpoint   = 35;
    max_temp   = 90;
    PID.Kp     = 3.0f;
    PID.Ti     = 30.0f;
    PID.Td     = 0.0f;
    system_on  = false;
    k_mutex_unlock(&rtdb_mutex);
}
