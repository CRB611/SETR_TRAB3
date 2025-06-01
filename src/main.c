/** 
 * \file main.c
 * \brief This file contains code as well as the main program function and the threads necessary for the system.
 *
 *  In this file the nrf board drivers are initialized and programed, and the threads are set to ensue a good 
 * Real-time system.
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
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>
/* our modules */
#include "../modules/tc74.h"
#include "../modules/rtdb.h"
#include "../modules/heater.h"
#include "../modules/pid.h"
#include "../modules/uart.h"


/* VARIABLES */
#define SLEEP_TIME_MS    1000    ///< Sleep Time
#define MAX_TEMP         120     ///< Absolute maximum temperature
#define OK               0       ///< Return if everything is alright
#define DELTAT           2       ///< Temperature tolerance
static bool table_flag = false;

/* UART RELATED VARIABLES */
#define UART_NODE              DT_NODELABEL(uart0)   ///< UART node ID
#define RBUFF_SIZE             60                    ///< Size of the reception buffer
#define RECEIVE_TIMEOUT        1000                  ///< Receive timeout
#define UART_SAMPLE_PERIOD_MS  15000                 ///< Sampling period, in miliseconds
#define UART_THREAD_STACK_SIZE 1024                  ///< Stack Size for the UART
#define UART_THREAD_PRIORITY   3                     ///< Thread Priority for UART

/* TC74 RELATED VARIABLES */
#define TC74_SAMPLE_PERIOD_MS  1000                  ///< Sampling period, in miliseconds
#define TC74_THREAD_PRIORITY   3                     ///< Thread Priority for TC74
#define TC74_THREAD_STACK_SIZE 512                  ///< Stack Size for the UART

/* TC74 RELATED VARIABLES */
#define PID_SAMPLE_PERIOD_MS  500                  ///< Sampling period, in miliseconds
#define PID_THREAD_PRIORITY   4                     ///< Thread Priority for the PID Controller
#define PID_THREAD_STACK_SIZE 512                  ///< Stack Size for the UART

/*UART Buffers*/
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
};          ///< UART configuration struct


/*DEFINING MUTEXES AND THREADS*/
/** 
 * \brief function that defines the Uart Mutex
 * \param uart_mutex name for the mutex
 */ 
K_MUTEX_DEFINE(uart_mutex);
/**
 * \brief function that defines the Uart K_thread Stack
 * \param uart_stack name for the stack
 * \param UART_THREAD_STACK_SIZE size of the stack
 */ 
K_THREAD_STACK_DEFINE(uart_stack, UART_THREAD_STACK_SIZE);
static struct k_thread uart_thread_data;
static void uart_thread(void *arg1, void *arg2, void *arg3);

/* Threads for TC74 and PID control */

/** 
 * \brief function that defines the tc74 Mutex
 * \param tc74_mutex name for the mutex
 */ 
K_MUTEX_DEFINE(tc74_mutex);

/**
 * \brief function that defines the tc74 K_thread Stack
 * \param tc74_stack name for the stack
 * \param TC74_THREAD_STACK_SIZE size of the stack
 */ 
K_THREAD_STACK_DEFINE(tc74_stack, TC74_THREAD_STACK_SIZE);
static struct k_thread tc74_thread_data;
static void tc74_thread(void *arg1, void *arg2, void *arg3);

/** 
 * \brief function that defines the PID Mutex
 * \param uart_mutex name for the mutex
 */ 
K_MUTEX_DEFINE(pid_mutex);
/**
 * \brief function that defines the PID control K_thread Stack
 * \param control_stack name for the stack
 * \param PID_THREAD_STACK_SIZE size of the stack
 */ 
K_THREAD_STACK_DEFINE(control_stack, PID_THREAD_STACK_SIZE);
static struct k_thread control_data;
static void control_thread(void *arg1, void *arg2, void *arg3);

/* BUTTON SETUP*/
#define SW0_NODE DT_ALIAS(sw0)                     ///< Node ID for BUTTON 1        
#define SW1_NODE DT_ALIAS(sw1)                     ///< Node ID for BUTTON 2    
#define SW2_NODE DT_ALIAS(sw2)                     ///< Node ID for BUTTON 3    
#define SW3_NODE DT_ALIAS(sw3)                     ///< Node ID for BUTTON 4    
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(SW3_NODE, gpios);

static struct gpio_callback button_cb_data0;
static struct gpio_callback button_cb_data1;
static struct gpio_callback button_cb_data2;
static struct gpio_callback button_cb_data3;

/*LED SETUP*/
#define LED0_NODE DT_ALIAS(led0)                     ///< Node ID for LED 1   
#define LED1_NODE DT_ALIAS(led1)                     ///< Node ID for LED 2   
#define LED2_NODE DT_ALIAS(led2)                     ///< Node ID for LED 3   
#define LED3_NODE DT_ALIAS(led3)                     ///< Node ID for LED 4   
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

/*Log error messages*/
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/*UART callback*/
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

/* Button callbacks */
/**
 * @brief Function executed when the button 1 is pressed
 * 
 *  This function toggles the system state ON/OFF 
 */
void button_pressed0(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    /*Checking system state*/
    if (rtdb_get_system_on()) {
        /*if ON turn off the heater and LED's, set system as off*/
        rtdb_set_system_on(false);
        heater_set_power(0);
        printk("System turned OFF\r\n");
        gpio_pin_set_dt(&led0,0);
        gpio_pin_set_dt(&led1,0);
        gpio_pin_set_dt(&led2,0);
        gpio_pin_set_dt(&led3,0);
        
        /*suspend the threads*/
        k_thread_suspend(&tc74_thread_data);
        k_thread_suspend(&uart_thread_data);
        k_thread_suspend(&control_data);

    } else {
        /*if OFF turn on the system and LED1*/
        rtdb_set_system_on(true);
        printk("System turned ON\r\n");
        gpio_pin_set_dt(&led0,1);

        /*resume the threads*/
        k_thread_resume(&tc74_thread_data);
        k_thread_resume(&uart_thread_data);
        k_thread_resume(&control_data);
    }
}

/**
 * @brief Function executed when the button 2 is pressed
 *
 *  This function increases the desired temperature 1ºC
 */
void button_pressed1(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (rtdb_get_system_on()) {
        /*getting the setpoint and adding 1*/
        int16_t curr_sp = rtdb_get_setpoint() + 1;
        rtdb_set_setpoint(curr_sp);
        printk("Desired Temperature increased to %dºC\r\n", curr_sp);
    }
}

/**
 * @brief Function executed when the button 3 is pressed
 * 
 *  The button 3 is pressed to toggle the printing of the RTDB table,
*/
void button_pressed2(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (rtdb_get_system_on()) {
        /*toggling the flag*/
        table_flag = !table_flag;
        printk(table_flag ? "RTDB Table ON\n" : "RTDB Table OFF\n");
    }
}

/**
 * @brief Function executed when the button 4 is pressed
 *
 *  This function decreases the desired temperature 1ºC
 */
void button_pressed3(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (rtdb_get_system_on()) {
        /*getting the setpoint and removing 1*/
        int16_t curr_sp = rtdb_get_setpoint() - 1;
        rtdb_set_setpoint(curr_sp);
        printk("Desired Temperature decreased to %dºC\r\n", curr_sp);
    }
}

int main(void)
{
    /*Initializing the modules*/
    int ret;
    rtdb_init();
    ret = tc74_init();
    if (ret) { 
        printk("Erro ao inicializar TC74 (%d)\n", ret); 
        return ret; 
    }

    ret = heater_init();
    if (ret) {
         printk("Erro ao inicializar heater PWM (%d)\n", ret); 
         return ret; 
    }

    /*Initializing the UART*/
    if (!device_is_ready(uart)) { 
        printk("UART device not ready\n"); 
        return 1; 
    }

    ret = uart_configure(uart, &uart_cfg);
    if (ret == -ENOSYS) { 
        printk("uart_configure() error. Invalid configuration\n"); 
        return -1; 
    }

    uart_callback_set(uart, uart_cb, NULL);
    /*Enabling UART reception*/
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
    k_thread_create(&tc74_thread_data, tc74_stack, K_THREAD_STACK_SIZEOF(tc74_stack), tc74_thread, NULL, NULL, NULL, TC74_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_create(&control_data, control_stack, K_THREAD_STACK_SIZEOF(control_stack), control_thread, NULL, NULL, NULL, PID_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_create(&uart_thread_data, uart_stack, K_THREAD_STACK_SIZEOF(uart_stack), uart_thread, NULL, NULL, NULL, UART_THREAD_PRIORITY, 0, K_NO_WAIT);
    
    /* Suspend until ON */
    k_thread_suspend(&tc74_thread_data);
    k_thread_suspend(&control_data);
    k_thread_suspend(&uart_thread_data);

    printk("SYSTEM START\n");
    while (1) {
        if (rtdb_get_system_on() && table_flag) {
             rtdb_print();
        }
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}

/*HEAT SENSOR THREAD*/
static void tc74_thread(void *unused1, void *unused2, void *unused3)
{
    uint8_t temp; int ret;
    while (1) {
        /*locking the mutex*/
        k_mutex_lock(&tc74_mutex, K_FOREVER);

        /*getting the sensor temperature*/
        ret = tc74_read(&temp);

        if (ret == 0) {
            /*If the sensor got a temperature value change its value on the RTDB*/
            rtdb_set_cur_temp(temp);

            /*checking the boundaries*/
            if (temp > rtdb_get_maxtemp()) {
                /*If it goes over the max temperature the system turns off and the flag tuns on*/
                rtdb_set_error_flag(true);
                rtdb_set_system_on(false);
                heater_set_power(0);
                LOG_ERR("temperatura %d°C excede maxTemp = %d°C, heater OFF", temp, rtdb_get_maxtemp());
            } else if (!rtdb_get_system_on() && temp < (rtdb_get_maxtemp() - 4)) {
                /*waits until its 4 degrees above the max to turn on again*/
                rtdb_set_error_flag(false);
                rtdb_set_system_on(true);
                LOG_WRN("temperatura %d°C segura, sistema auto-religa", temp);
            }
            int setp = rtdb_get_setpoint();
            printk("TC74-Thread: Temperatura = %d C ; Desejada = %d \n", temp, setp);
            /*Lighting up the correct LEDs*/
            if (temp > setp + DELTAT) { 
                gpio_pin_set_dt(&led1,0);
                gpio_pin_set_dt(&led2,0);
                gpio_pin_set_dt(&led3,1); 
            } else if (temp < setp - DELTAT) {
                gpio_pin_set_dt(&led1,0); 
                gpio_pin_set_dt(&led2,1); 
                gpio_pin_set_dt(&led3,0); 
            }else { 
                gpio_pin_set_dt(&led1,1); 
                gpio_pin_set_dt(&led2,0); 
                gpio_pin_set_dt(&led3,0); 
            }
        } else {
            rtdb_set_error_flag(true); 
            printk("TC74-Thread: erro na leitura (%d)\n", ret); 
        }

        /*unlocking the mutex*/
        k_mutex_unlock(&tc74_mutex, K_FOREVER);

        k_msleep(TC74_SAMPLE_PERIOD_MS);
    }
}

/*PID CONTROLLER THREAD*/
static void control_thread(void *a, void *b, void *c)
{
    static pid_data_t pid;
    rtdb_pid gpid = rtdb_get_pid();
    /*initializing the pid controller*/
    pid_init(&pid, gpid.Kp, gpid.Ti, gpid.Td);
   
    while (1) {
        /*locking the mutex*/
        k_mutex_lock(&pid_mutex, K_FOREVER);

        
        if (!rtdb_get_system_on()) {
            /*if system off turn of the power*/
            heater_set_power(0);
        }else {
            /*if system on set the power to the computed one*/
            int16_t sp = rtdb_get_setpoint(); uint8_t ct = rtdb_get_cur_temp();
            float u = pid_compute(&pid, sp, (float)ct, 500 / 1000.0f);
            heater_set_power((uint8_t)u);
        }
        
        /*unlocking the mutex*/
        k_mutex_unlock(&pid_mutex, K_FOREVER);
        k_msleep(PID_SAMPLE_PERIOD_MS);
    }
}

/*UART THREAD*/
static void uart_thread(void *arg1, void *arg2, void *arg3)
{
    /* check if it turns on */
    printk(">> UART thread arrancou – system_on=%d, initial_chars=%d\n",
           rtdb_get_system_on(), uart_rxbuf_nchar);

    while (1) {
        
        printk("UART-Thread: ciclo, chars_recebidos = %d\n", uart_rxbuf_nchar);

        /*locking te uart mutex*/
        k_mutex_lock(&uart_mutex, K_FOREVER);
        if (uart_rxbuf_nchar > 0) {
            /* processes teh received information */
            int ret = uart_process();
            printk("uart_thread: uart_process() retorno = %d\n", ret);
            
            /* Resets the buffer counter to the next message*/
            uart_rxbuf_nchar = 0;
        }

        /*unlocking the uart mutex*/
        k_mutex_unlock(&uart_mutex);

        /* Aguarda antes do próximo ciclo */
        k_msleep(UART_SAMPLE_PERIOD_MS);
    }
}

/*uart calback function*/
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    /*checks the UART state*/
    switch (evt->type)
    {
    case UART_RX_RDY:
        /*if ready sends the character to the msg buffer*/
        memcpy(&rx_msg[uart_rxbuf_nchar], &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
        uart_rxbuf_nchar += evt->data.rx.len;
        break;
    case UART_RX_BUF_REQUEST:
        /*If the buffer is full it changes buffer*/
        uart_rx_buf_rsp(uart, buffer ? rx_buff : rx_buff2, RBUFF_SIZE);
        buffer = !buffer;
        break;
    case UART_RX_DISABLED:
        /*Disabled message*/
        uart_rx_enable(uart, rx_buff, sizeof(rx_buff), RECEIVE_TIMEOUT);
        break;
    default:
        break;
    }
}


/**
 * \brief function where the Uart commands are processed
 */
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
