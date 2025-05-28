/** 
 * \file main.c
 * \brief This file contains the structures and functions needed for the code as well as the main program.
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
#include "../modules/tc74.h"
#include "../modules/rtdb.h"
#include "../modules/heater.h"
#include "../modules/pid.h"
#include "../modules/uart.h"
/* Log error */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Definições gerais */
#define SLEEP_TIME_MS    1000    ///< Sleep Time
#define MAX_TEMP         120     ///< Absolute maximum temperature
#define OK               0       ///< Return if everything is alright

static bool table_flag = false;

/* UART RELATED VARIABLES */
#define UART_NODE              DT_NODELABEL(uart0)   ///< UART node ID
#define RBUFF_SIZE             60                    ///< Size of the reception buffer
#define RECEIVE_TIMEOUT        1000                  ///< Receive timeout
#define UART_SAMPLE_PERIOD_MS  15000                 ///< Sampling period, in miliseconds
#define UART_THREAD_STACK_SIZE 1024                  ///< Stack Size for the UART
#define UART_THREAD_PRIORITY   3                     ///< Thread Priority for UART

static uint8_t  rx_buff[RBUFF_SIZE];
static uint8_t  rx_buff2[RBUFF_SIZE];
static uint8_t  rx_msg[RBUFF_SIZE];
volatile int    uart_rxbuf_nchar = 0;             ///< Number of chars currently in the rx buffer
static const struct device *uart = DEVICE_DT_GET(UART_NODE);
bool buffer = true;                                ///< Which receive buffer is active

const struct uart_config uart_cfg = {
    .baudrate = 115200,
    .parity   = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

K_MUTEX_DEFINE(uart_mutex);
K_THREAD_STACK_DEFINE(uart_stack, UART_THREAD_STACK_SIZE);
static struct k_thread uart_thread_data;
static void uart_thread(void *arg1, void *arg2, void *arg3);
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

/* Threads for TC74 and PID control */
K_THREAD_STACK_DEFINE(tc74_stack, 512);
static struct k_thread tc74_thread_data;
static void tc74_thread(void *arg1, void *arg2, void *arg3);

K_THREAD_STACK_DEFINE(control_stack, 512);
static struct k_thread control_data;
static void control_thread(void *arg1, void *arg2, void *arg3);

/* Button and LED setup */
#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)
#define SW2_NODE DT_ALIAS(sw2)
#define SW3_NODE DT_ALIAS(sw3)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(SW3_NODE, gpios);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

static struct gpio_callback button_cb_data0;
static struct gpio_callback button_cb_data1;
static struct gpio_callback button_cb_data2;
static struct gpio_callback button_cb_data3;

/* Button callbacks */
void button_pressed0(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (rtdb_get_system_on()) {
        rtdb_set_system_on(false);
        heater_set_power(0);
        printk("System turned OFF\r\n");
        gpio_pin_set_dt(&led0,0);
        gpio_pin_set_dt(&led1,0);
        gpio_pin_set_dt(&led2,0);
        gpio_pin_set_dt(&led3,0);
        k_thread_suspend(&tc74_thread_data);
        k_thread_suspend(&uart_thread_data);
        k_thread_suspend(&control_data);
    } else {
        rtdb_set_system_on(true);
        printk("System turned ON\r\n");
        gpio_pin_set_dt(&led0,1);
        k_thread_resume(&tc74_thread_data);
        k_thread_resume(&uart_thread_data);
        k_thread_resume(&control_data);
    }
}

void button_pressed1(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (rtdb_get_system_on()) {
        int16_t curr_sp = rtdb_get_setpoint() + 1;
        rtdb_set_setpoint(curr_sp);
        printk("Desired Temperature increased to %dºC\r\n", curr_sp);
    }
}

void button_pressed2(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (rtdb_get_system_on()) {
        table_flag = !table_flag;
        printk(table_flag ? "RTDB Table ON\n" : "RTDB Table OFF\n");
    }
}

void button_pressed3(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (rtdb_get_system_on()) {
        int16_t curr_sp = rtdb_get_setpoint() - 1;
        rtdb_set_setpoint(curr_sp);
        printk("Desired Temperature decreased to %dºC\r\n", curr_sp);
    }
}

int main(void)
{
    int ret;
    rtdb_init();
    ret = tc74_init();
    if (ret) { printk("Erro ao inicializar TC74 (%d)\n", ret); return ret; }
    ret = heater_init();
    if (ret) { printk("Erro ao inicializar heater PWM (%d)\n", ret); return ret; }
    if (!device_is_ready(uart)) { printk("UART device not ready\n"); return 1; }
    ret = uart_configure(uart, &uart_cfg);
    if (ret == -ENOSYS) { printk("uart_configure() error. Invalid configuration\n"); return -1; }
    uart_callback_set(uart, uart_cb, NULL);
    uart_rx_enable(uart, rx_buff, sizeof(rx_buff), RECEIVE_TIMEOUT);

    /* LEDs init */
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
    /* Buttons init */
    gpio_pin_configure_dt(&button0, GPIO_INPUT);
    gpio_pin_configure_dt(&button1, GPIO_INPUT);
    gpio_pin_configure_dt(&button2, GPIO_INPUT);
    gpio_pin_configure_dt(&button3, GPIO_INPUT);
    gpio_init_callback(&button_cb_data0, button_pressed0, BIT(button0.pin));
    gpio_add_callback(button0.port, &button_cb_data0);
    gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data1, button_pressed1, BIT(button1.pin));
    gpio_add_callback(button1.port, &button_cb_data1);
    gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data2, button_pressed2, BIT(button2.pin));
    gpio_add_callback(button2.port, &button_cb_data2);
    gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data3, button_pressed3, BIT(button3.pin));
    gpio_add_callback(button3.port, &button_cb_data3);
    gpio_pin_interrupt_configure_dt(&button3, GPIO_INT_EDGE_TO_ACTIVE);

    /* Create threads */
    k_thread_create(&tc74_thread_data, tc74_stack, K_THREAD_STACK_SIZEOF(tc74_stack), tc74_thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    k_thread_create(&control_data, control_stack, K_THREAD_STACK_SIZEOF(control_stack), control_thread, NULL, NULL, NULL, 4, 0, K_NO_WAIT);
    k_thread_create(&uart_thread_data, uart_stack, K_THREAD_STACK_SIZEOF(uart_stack), uart_thread, NULL, NULL, NULL, UART_THREAD_PRIORITY, 0, K_NO_WAIT);
    /* Suspend until ON */
    k_thread_suspend(&tc74_thread_data);
    k_thread_suspend(&control_data);
    k_thread_suspend(&uart_thread_data);

    printk("SYSTEM START\n");
    while (1) {
        if (rtdb_get_system_on() && table_flag) rtdb_print();
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}

static void tc74_thread(void *unused1, void *unused2, void *unused3)
{
    uint8_t temp; int ret;
    while (1) {
        ret = tc74_read(&temp);
        if (ret == 0) {
            rtdb_set_cur_temp(temp);
            if (temp > rtdb_get_maxtemp()) {
                rtdb_set_error_flag(true);
                rtdb_set_system_on(false);
                heater_set_power(0);
                LOG_ERR("temperatura %d°C excede maxTemp = %d°C, heater OFF", temp, rtdb_get_maxtemp());
            } else if (!rtdb_get_system_on() && temp < (rtdb_get_maxtemp() - 4)) {
                rtdb_set_error_flag(false);
                rtdb_set_system_on(true);
                LOG_WRN("temperatura %d°C segura, sistema auto-religa", temp);
            }
            int setp = rtdb_get_setpoint();
            printk("TC74-Thread: Temperatura = %d C ; Desejada = %d \n", temp, setp);
            if (temp > setp + 2) { gpio_pin_set_dt(&led1,0); gpio_pin_set_dt(&led2,0); gpio_pin_set_dt(&led3,1); }
            else if (temp < setp - 2) { gpio_pin_set_dt(&led1,0); gpio_pin_set_dt(&led2,1); gpio_pin_set_dt(&led3,0); }
            else { gpio_pin_set_dt(&led1,1); gpio_pin_set_dt(&led2,0); gpio_pin_set_dt(&led3,0); }
        } else { rtdb_set_error_flag(true); printk("TC74-Thread: erro na leitura (%d)\n", ret); }
        k_msleep(1000);
    }
}

static void control_thread(void *a, void *b, void *c)
{
    static pid_data_t pid;
    rtdb_pid gpid = rtdb_get_pid();
    pid_init(&pid, gpid.Kp, gpid.Ti, gpid.Td);
    while (1) {
        if (!rtdb_get_system_on()) heater_set_power(0);
        else {
            int16_t sp = rtdb_get_setpoint(); uint8_t ct = rtdb_get_cur_temp();
            float u = pid_compute(&pid, sp, (float)ct, 500 / 1000.0f);
            heater_set_power((uint8_t)u);
        }
        k_msleep(500);
    }
}

static void uart_thread(void *arg1, void *arg2, void *arg3)
{
    /* Debug: confirma que a thread arranca e o estado inicial */
    printk(">> UART thread arrancou – system_on=%d, initial_chars=%d\n",
           rtdb_get_system_on(), uart_rxbuf_nchar);

    while (1) {
        /* Debug de cada ciclo */
        printk("UART-Thread: ciclo, chars_recebidos = %d\n", uart_rxbuf_nchar);

        k_mutex_lock(&uart_mutex, K_FOREVER);
        if (uart_rxbuf_nchar > 0) {
            /* Processa o frame recebido */
            int ret = uart_process();
            printk("uart_thread: uart_process() retorno = %d\n", ret);
            /* Reset do contador para próxima receção */
            uart_rxbuf_nchar = 0;
        }
        k_mutex_unlock(&uart_mutex);

        /* Aguarda antes do próximo ciclo */
        k_msleep(UART_SAMPLE_PERIOD_MS);
    }
}


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        memcpy(&rx_msg[uart_rxbuf_nchar], &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
        uart_rxbuf_nchar += evt->data.rx.len;
        break;
    case UART_RX_BUF_REQUEST:
        uart_rx_buf_rsp(uart, buffer ? rx_buff : rx_buff2, RBUFF_SIZE);
        buffer = !buffer;
        break;
    case UART_RX_DISABLED:
        uart_rx_enable(uart, rx_buff, sizeof(rx_buff), RECEIVE_TIMEOUT);
        break;
    default:
        break;
    }
}

int uart_process(void)
{
    /* Build frame */
    char frame[RBUFF_SIZE+1];
    memcpy(frame, rx_msg, uart_rxbuf_nchar);
    frame[uart_rxbuf_nchar] = '\0';
    /* Process command */
    char reply[RBUFF_SIZE];
    int result = process_uart_command(frame, reply);
    /* Send reply */
    uart_tx(uart, (const uint8_t *)reply, strlen(reply), SYS_FOREVER_US);
    printk("UART reply: %s\n", reply);
    return result;
}
