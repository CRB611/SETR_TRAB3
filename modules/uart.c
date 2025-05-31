/** 
 * \file uart.c
 * \brief This file contains the implementation of the functions implemented in uart.h.
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug There are no known bugs.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "uart.h"

#ifdef UNIT_TEST
/* stub vazio para host */
#else
#  include <zephyr/drivers/uart.h>
#endif

#include "rtdb.h"
#include "heater.h"
#include "pid.h"
#include "tc74.h"

#define MAX_TEMP 120


int calcChecksum(unsigned char *buf, int nbytes) {
    unsigned int sum = 0;
    for (int i = 0; i < nbytes; i++) {
        sum += (unsigned int)buf[i];
    }
    return sum % 256;
}


void num2char(unsigned char *array, int num) {
    for (int i = 2; i >= 0; i--) {
        array[i] = (num % 10) + '0';
        num /= 10;
    }
}


unsigned int char2num(unsigned char ascii[], int length) {
    unsigned int value = 0;
    for (int i = 0; i < length; i++) {
        value = value * 10 + (ascii[i] - '0');
    }
    return value;
}


float char2float(unsigned char ascii[]) {
    int intpart = (ascii[0] - '0') * 10 + (ascii[1] - '0');
    int frac = (ascii[3] - '0');
    return intpart + frac / 10.0f;
}


int process_uart_command(const char *cmd, char *reply) {
    size_t len = strlen(cmd);
    // Verificação de framing

    if (len == 0 || cmd[0] != '#') {
        int cs = calcChecksum((unsigned char *)"Ef", 2);
        sprintf(reply, "#Ef%03d!", cs);
        return -1;
    }
    if (cmd[len - 1] != '!') {
        int cs = calcChecksum((unsigned char *)"Ef", 2);
        sprintf(reply, "#Ef%03d!", cs);
        return -1;
    }
    if (len < 5) {
        int cs = calcChecksum((unsigned char *)"Ef", 2);
        sprintf(reply, "#Ef%03d!", cs);
        return -1;
    }
    // Verificação de checksum
    unsigned char *chks_ptr = (unsigned char *)&cmd[len - 4];
    int chk_recv = char2num(chks_ptr, 3);
    int chk_calc = calcChecksum((unsigned char *)&cmd[1], (int)(len - 1 - 1 - 3));
    if (chk_calc != chk_recv) {
        sprintf(reply, "#Es%03d!", chk_calc);
        return -1;
    }
    // Identificador de comando
    char id = cmd[1];
    switch (id) {
        case 'M': {
            /*definir max temp*/
            int val = char2num((unsigned char *)&cmd[2], 3);
            if (val > MAX_TEMP) {
                sprintf(reply, "#Ei000!");
                return -1;
            }
            rtdb_set_maxtemp((uint8_t)val);
            int cs = calcChecksum((unsigned char *)"Eo", 2);
            sprintf(reply, "#Eo%03d!", cs);
            return 0;
        }
        case 'C': {
            /*devolver a temp atual*/
            uint8_t temp = rtdb_get_cur_temp();
            unsigned char buf[8] = "#c000000!";
            buf[1] = 'c';
            num2char(&buf[2], temp);
            int cs = calcChecksum(&buf[1], 4);
            num2char(&buf[5], cs);
            memcpy(reply, buf, 8);
            return 0;
        }
        case 'S': {
            /*definir os parametros do PID*/
            float Kp = char2float((unsigned char *)&cmd[2]);
            float Ti = char2float((unsigned char *)&cmd[6]);
            float Td = char2float((unsigned char *)&cmd[10]);
            rtdb_set_pid(Kp, Ti, Td);
            int cs = calcChecksum((unsigned char *)"Eo", 2);
            sprintf(reply, "#Eo%03d!", cs);
            return 0;
        }
        case 'R': {
            /*Reset ás variaveis da RTDB*/
            rtdb_reset();
            int cs = calcChecksum((unsigned char *)"Eo", 2);
            sprintf(reply, "#Eo%03d!", cs);
            return 0;
        }
        case 'D': {
            /*definir o setpoint*/
            int sp = char2num((unsigned char *)&cmd[2], 3);
            rtdb_set_setpoint((uint16_t)sp);
            int cs = calcChecksum((unsigned char *)"Eo", 2);
            sprintf(reply, "#Eo%03d!", cs);
            return 0;
        }
        case 'G': {
            /* Get max_temp: #G000YYY! */
            uint8_t max = rtdb_get_maxtemp();
            unsigned char buf[8] = "#G000000!";
            buf[1] = 'G';
            num2char(&buf[2], max);
            int cs = calcChecksum(&buf[1], 4);
            num2char(&buf[5], cs);
            memcpy(reply, buf, 8);
            return 0;
        }
        default: {
            /*comando inesistente*/
            int cs = calcChecksum((unsigned char *)"Ei", 2);
            sprintf(reply, "#Ei%03d!", cs);
            return -1;
        }
    }
}
