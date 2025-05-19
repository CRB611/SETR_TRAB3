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
#include "heater.h"
#include "pid.h"
#include "uart.h"

/* Definições gerais */
#define SLEEP_TIME_MS    1000    ///< Sleep Time in miliseconds

/* UART RELATED VARIABLES */
#define UART_NODE       DT_NODELABEL(uart0)   ///< UART node ID
#define TBUFF_SIZE      60                    ///< Size of the transmission buffer
#define RBUFF_SIZE      60                    ///< Size of the reception buffer
#define MSG_BUFF_SIZE   100                   ///< Size of the message buffer
#define RECEIVE_TIMEOUT 1000                  ///< Receive timeout in miliseconds

static uint8_t tx_buff[TBUFF_SIZE];
static uint8_t rx_buff[RBUFF_SIZE];
static uint8_t rx_msg[RBUFF_SIZE];
volatile int uart_rxbuf_nchar = 0;           ///< Number of chars currently in the rx buffer
volatile int uart_txbuf_nchar = 0;           ///< Number of chars currently in the rx buffer
static const struct device *uart = DEVICE_DT_GET(UART_NODE);

/* --- Definições para a thread de leitura do TC74 --- */
#define TC74_SAMPLE_PERIOD_MS   1000    /* período de amostragem, em ms */
#define TC74_THREAD_STACK_SIZE  512     /* tamanho da stack da thread */
#define TC74_THREAD_PRIORITY    5       /* prioridade da thread */
K_THREAD_STACK_DEFINE(tc74_stack, TC74_THREAD_STACK_SIZE);
static struct k_thread tc74_thread_data;
static void tc74_thread(void *arg1, void *arg2, void *arg3);

/* --- Definições thread de controlo PID→PWM --- */
#define CONTROL_PERIOD_MS 500       ///< PID period in Miliseconds
#define CONTROL_PRIO      4        ///< 
#define CONTROL_STACK_SZ  512       ///< 
K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SZ);
static struct k_thread control_data;
static void control_thread(void *arg1, void *arg2, void *arg3);

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

static struct gpio_callback button_cb_data0;
static struct gpio_callback button_cb_data1;
static struct gpio_callback button_cb_data3;

/* Callback functions for the buttons */
/**
 * @brief Function executed when the button 1 is pressed
 * 
 *  This function toggles the system state ON/OFF 
 */
void button_pressed0(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if(rtdb_get_system_on()== true){
        rtdb_set_system_on(false);
        printk("System turned OFF\r\n");
    }else{
        rtdb_set_system_on(true);
        printk("System turned ON\r\n");
    }

    gpio_pin_toggle_dt(&led0);
}

/**
 * @brief Function executed when the button 2 is pressed
 * 
 *  This function increases the desired temperature 1ºC
 */
void button_pressed1(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int16_t curr_sp=rtdb_get_setpoint()+1;
    rtdb_set_setpoint(curr_sp);
    printk("Desired Temperature increased to %dºC\r\n",curr_sp);
}

/**
 * @brief Function executed when the button 4 is pressed
 *
 *  This function decreases the desired temperature 1ºC
 */
void button_pressed3(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int16_t curr_sp=rtdb_get_setpoint()-1;
    rtdb_set_setpoint(curr_sp);
    printk("Desired Temperature decreased to %dºC\r\n",curr_sp);
}


int main(void)
{
    int ret;

    /* Inicializa a RTDB */
    rtdb_init();

    /* Inicializa o sensor TC74 via I²C */
    ret = tc74_init();
    if (ret) {
        printk("Erro ao inicializar TC74 (%d)\n", ret);
        return ret;
    }

    /* Inicializa o PWM do heater */
    ret = heater_init();
    if (ret) {
        printk("Erro ao inicializar heater PWM (%d)\n", ret);
        return ret;
    }

    /* Inicialização da UART */
    if (!device_is_ready(uart)) {
        printk("UART device not ready\r\n");
        return 1;
    }
    uart_callback_set(uart, uart_cb, NULL);
    uart_rx_enable(uart, rx_buff, sizeof(rx_buff), RECEIVE_TIMEOUT);

    /* Inicialização dos LEDs */
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

    /* Inicialização dos botões */
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

    /* Cria thread de leitura do TC74 */
    k_thread_create(&tc74_thread_data, tc74_stack,
                    K_THREAD_STACK_SIZEOF(tc74_stack),
                    tc74_thread, NULL, NULL, NULL,
                    TC74_THREAD_PRIORITY, 0, K_NO_WAIT);

    /* Cria thread de controlo PID->PWM */
    k_thread_create(&control_data, control_stack,
                    K_THREAD_STACK_SIZEOF(control_stack),
                    control_thread, NULL, NULL, NULL,
                    CONTROL_PRIO, 0, K_NO_WAIT);

    /* Loop principal (sleep) */
    while (1) {
        k_msleep(SLEEP_TIME_MS);
        /*so pra ver se funciona*/
        if(uart_rxbuf_nchar > 0) {
            rx_chars[uart_rxbuf_nchar] = 0; /* Terminate the string */
            uart_rxbuf_nchar = 0;           /* Reset counter */

            sprintf(rep_mesg,"You typed [%s]\n\r",rx_chars);            
            
            err = uart_tx(uart_dev, rep_mesg, strlen(rep_mesg), SYS_FOREVER_MS);
            if (err) {
                printk("uart_tx() error. Error code:%d\n\r",err);
                return FATAL_ERR;
            }
        }

    }

    return 0;
}

/* Função que será corrida periodicamente pela thread TC74 */
static void tc74_thread(void *unused1, void *unused2, void *unused3)
{
    uint8_t temp;
    int ret;

    while (1) {
        ret = tc74_read(&temp);
        if (ret == 0) {
            /* Sucesso: guarda na RTDB e imprime */
            rtdb_set_cur_temp(temp);
            printk("TC74-Thread: Temperatura = %d C\n", temp);
        } else {
            /* Erro: sinaliza e imprime */
            rtdb_set_error_flag(true);
            printk("TC74-Thread: erro na leitura (%d)\n", ret);
        }
        k_msleep(TC74_SAMPLE_PERIOD_MS);
    }
}

/* Função da thread de controlo PID->PWM */
static void control_thread(void *a, void *b, void *c)
{
    static pid_data_t pid;
    pid_init(&pid, 5.0f, 0.0f, 0.0f);  /* Kp=5, P-only */

    while (1) {
        if (!rtdb_get_system_on()) {
            heater_set_power(0);
        } else {
            int16_t sp = rtdb_get_setpoint();
            uint8_t ct = rtdb_get_cur_temp();
            float u = pid_compute(&pid, sp, (float)ct, CONTROL_PERIOD_MS / 1000.0f);
            heater_set_power((uint8_t)u);
        }
        k_msleep(CONTROL_PERIOD_MS);
    }
}


/* UART callback function */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    int err;

    switch (evt->type) {
	
        case UART_TX_DONE:
		    printk("UART_TX_DONE event \n\r");
            break;

    	case UART_TX_ABORTED:
	    	printk("UART_TX_ABORTED event \n\r");
		    break;
		
	    case UART_RX_RDY:
		    printk("UART_RX_RDY event \n\r");
            /* Just copy data to a buffer. */
            /* Simple approach, just for illustration. In most cases it is necessary to use */
            /*    e.g. a FIFO or a circular buffer to communicate with a task that shall process the messages*/
            memcpy(&rx_msg[uart_rxbuf_nchar],&(rx_buff[evt->data.rx.offset]),evt->data.rx.len); 
            uart_rxbuf_nchar += evt->data.rx.len;           
		    break;

	    case UART_RX_BUF_REQUEST:
		    printk("UART_RX_BUF_REQUEST event \n\r");
            /* Should be used to allow continuous reception */
            /* To this end, declare at least two buffers and switch among them here */
            /*      using function uart_rx_buf_rsp() */
		    break;

	    case UART_RX_BUF_RELEASED:
		    printk("UART_RX_BUF_RELEASED event \n\r");
		    break;
		
	    case UART_RX_DISABLED: 
            /* When the RX_BUFF becomes full RX is disabled automaticaly.  */
            /* It must be re-enabled manually for continuous reception */
            printk("UART_RX_DISABLED event \n\r");
		    err =  uart_rx_enable(uart ,rx_buff,sizeof(rx_buff),RECEIVE_TIMEOUT);
            if (err) {
                printk("uart_rx_enable() error. Error code:%d\n\r",err);
                exit(-1);                
            }
		    break;

	    case UART_RX_STOPPED:
		    printk("UART_RX_STOPPED event \n\r");
		    break;
		
	    default:
            printk("UART: unknown event \n\r");
		    break;
    }

}
