/**
 * \file main.c
 * \brief This file contains all the structures and functions needed for the code as well as the main program.
 *
 *        blablabla
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 04/06/2025
 * \bug There are no known bugs.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <string.h>

/* nosso módulo I²C/TC74 */
#include "tc74.h"
#include "rtdb.h"

/* --- Definições para a thread de leitura do TC74 --- */
#define TC74_SAMPLE_PERIOD_MS   1000    /* período de amostragem, em ms */
#define TC74_THREAD_STACK_SIZE  512     /* tamanho da stack da thread */
#define TC74_THREAD_PRIORITY    5       /* prioridade da thread */

/*Declara a stack da thread */
K_THREAD_STACK_DEFINE(tc74_stack, TC74_THREAD_STACK_SIZE);

/*Objeto para guardar os metadados da thread */
static struct k_thread tc74_thread_data;

/*Protótipo da função que corre na thread */
static void tc74_thread(void *arg1, void *arg2, void *arg3);





/* VARIÁVEIS */
#define SLEEP_TIME_MS 1000   ///< Sleep Time

/* UART RELATED VARIABLES */
#define UART_NODE       DT_NODELABEL(uart0)   ///< UART node ID
#define TBUFF_SIZE      60                    ///< Size of the transmission buffer
#define RBUFF_SIZE      60                    ///< Size of the reception buffer
#define MSG_BUFF_SIZE   100                   ///< Size of the message buffer
#define RECEIVE_TIMEOUT 1000                 ///< Receive timeout

static uint8_t tx_buff[TBUFF_SIZE];
static uint8_t rx_buff[RBUFF_SIZE];
volatile int uart_rxbuf_nchar = 0;           ///< Number of chars currently in the rx buffer 
static const struct device *uart = DEVICE_DT_GET(UART_NODE);

/* Setting up the buttons */
#define SW0_NODE DT_ALIAS(sw0)  ///< NODE ID for Button 1
#define SW1_NODE DT_ALIAS(sw1)  ///< NODE ID for Button 2
#define SW3_NODE DT_ALIAS(sw3)  ///< NODE ID for Button 4

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(SW3_NODE, gpios);

/* Setting up the LEDs */
#define LED0_NODE DT_ALIAS(led0)  ///< NODE ID for LED 1
#define LED1_NODE DT_ALIAS(led1)  ///< NODE ID for LED 2
#define LED2_NODE DT_ALIAS(led2)  ///< NODE ID for LED 3
#define LED3_NODE DT_ALIAS(led3)  ///< NODE ID for LED 4

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

/* Callback variables for the buttons */
static struct gpio_callback button_cb_data0;
static struct gpio_callback button_cb_data1;
static struct gpio_callback button_cb_data3;

/* callback functions for the buttons */
/**
 * @brief Executed when button 1 is pressed
 */
void button_pressed0(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    printk("ON/OFF\r\n");
    gpio_pin_toggle_dt(&led0);
}

/**
 * @brief Executed when button 2 is pressed
 */
void button_pressed1(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    printk("Desired Temperature increased : xxºC\r\n");
}

/**
 * @brief Executed when button 4 is pressed
 */
void button_pressed3(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    printk("Desired Temperature decreased : xxºC\r\n");
}

/* UART callback function */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    switch (evt->type) {  
    case UART_RX_RDY:
        if (evt->data.rx.len == 1) {  
            printk("pressing button = %c \r\n", evt->data.rx.buf[evt->data.rx.offset]);
        }
        break;
    case UART_RX_DISABLED:
        uart_rx_enable(dev, rx_buff, sizeof rx_buff, RECEIVE_TIMEOUT);
        break;  
    default:
        break;
    }
}

int main(void)
{
    int ret;
	 /* Inicializa a RTDB */
    rtdb_init();
    /* --- Inicializar sensor TC74 via I²C --- */
    ret = tc74_init();
    if (ret) {
        printk("Erro ao inicializar TC74 (%d)\n", ret);
        return ret;
    }

    /* --- Inicialização da UART --- */
    if (!device_is_ready(uart)) {
        printk("UART device not ready\r\n");
        return 1;
    }
    uart_callback_set(uart, uart_cb, NULL);
    uart_rx_enable(uart, rx_buff, sizeof rx_buff, RECEIVE_TIMEOUT);

    /* --- Inicialização dos LEDs --- */
    if (!device_is_ready(led0.port) ||
        !device_is_ready(led1.port) ||
        !device_is_ready(led2.port) ||
        !device_is_ready(led3.port)) {
        printk("LEDs are not ready\r\n");
        return -1;
    }
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);

    /* --- Inicialização dos botões e configurações de interrupção --- */
    if (!device_is_ready(button0.port) ||
        !device_is_ready(button1.port) ||
        !device_is_ready(button3.port)) {
        printk("Buttons not ready\r\n");
        return -1;
    }
    gpio_pin_configure_dt(&button0, GPIO_INPUT);
    gpio_pin_configure_dt(&button1, GPIO_INPUT);
    gpio_pin_configure_dt(&button3, GPIO_INPUT);

    gpio_init_callback(&button_cb_data0, button_pressed0, BIT(button0.pin));
    gpio_add_callback(button0.port, &button_cb_data0);
    gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_cb_data1, button_pressed1, BIT(button1.pin));
    gpio_add_callback(button1.port, &button_cb_data1);
    gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_cb_data3, button_pressed3, BIT(button3.pin));
    gpio_add_callback(button3.port, &button_cb_data3);
    gpio_pin_interrupt_configure_dt(&button3, GPIO_INT_EDGE_TO_ACTIVE);

    /* --- Transmissão inicial por UART (opcional) --- */
    // uint8_t welcome_mesg[] = "UART demo: Type a few chars ...\n\r";
    // uart_tx(uart, welcome_mesg, strlen(welcome_mesg), SYS_FOREVER_US);

k_thread_create(&tc74_thread_data,    /* objecto da thread */
                    tc74_stack,           /* stack definida em 1a) */
                    K_THREAD_STACK_SIZEOF(tc74_stack), 
                    tc74_thread,          /* função a correr */
                    NULL, NULL, NULL,     /* args (não usamos) */
                    TC74_THREAD_PRIORITY, /* prioridade */
                    0,                    /* flags */
                    K_NO_WAIT);           /* arranca imediatamente */

    /* --- Loop principal original (pode ficar só a dormir) --- */
    while (1) {
        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}


/* 2) Função que será corrida periodicamente pela thread TC74 */
/* thread de leitura periódico do TC74 */
static void tc74_thread(void *unused1, void *unused2, void *unused3)
{
    uint8_t temp;
    int ret;

    while (1) {
        ret = tc74_read(&temp);
        if (ret == 0) {
            /* Sucesso: guarda na RTDB e opcionalmente imprime */
            rtdb_set_cur_temp(temp);
            printk("TC74-Thread: Temperatura = %d C\n", temp);
        } else {
            /* Erro: sinaliza na RTDB e imprime */
            rtdb_set_error_flag(true);
            printk("TC74-Thread: erro na leitura (%d)\n", ret);
        }
        k_msleep(TC74_SAMPLE_PERIOD_MS);
    }
}


