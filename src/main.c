/** 
 * \file main.c
 * \brief This file contains all the structures and functions needed for the code as well as the main program.
 *
 *		blablabla
 *
 * \author Simão Ribeiro
 * \author Celina Brito
 * \date 4/6/2025
 * \bug There are no known bugs.
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <string.h>

/*VARIAVEIS*/
#define SLEEP_TIME_MS 1000	///<Sleep Time

/*UART RELATED VARIABLES*/
#define UART_NODE DT_NODELABEL(uart0)   ///<UART node ID
#define TBUFF_SIZE 60					///<Size of the transmition buffer
#define RBUFF_SIZE 60					///<Size of the reception buffer
#define MSG_BUFF_SIZE 100				///<Size of the message buffer
#define RECEIVE_TIMEOUT 1000			///<Receive timeout
static uint8_t tx_buff[TBUFF_SIZE];		
static uint8_t rx_buff[RBUFF_SIZE];
volatile int uart_rxbuf_nchar=0;        ///<Number of chars currently on the rx buffer 
const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));

/* Setting up the buttons */
#define SW0_NODE DT_ALIAS(sw0)			///<NODE ID for the Button 1
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
#define SW1_NODE DT_ALIAS(sw1)			///<NODE ID for the Button 2
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
#define SW3_NODE DT_ALIAS(sw3)			///<NODE ID for the Button 4
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(SW3_NODE, gpios);

/* Setting up the LEDS */
#define LED0_NODE DT_ALIAS(led0)		///<NODE ID for the LED 1
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define LED1_NODE DT_ALIAS(led1)		///<NODE ID for the LED 2
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
#define LED2_NODE DT_ALIAS(led2)		///<NODE ID for the LED 3
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
#define LED3_NODE DT_ALIAS(led3)		///<NODE ID for the LED 4
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

/*callback functions for the buttons */
/**
 * @brief Function executed when the button 1 is pressed
 * @param dev pointer to the device
 * @param cb poiter to the callback variable
 * @param pins pin to be executed
 */
void button_pressed0( const struct device *dev, struct gpio_callback *cb, uint32_t pins){
	printk("ON/OFF\r\n");
	gpio_pin_toggle_dt(&led0);
}
/**
 * @brief Function executed when the button 2 is pressed
 * @param dev pointer to the device
 * @param cb poiter to the callback variable
 * @param pins pin to be executed
 */
void button_pressed1( const struct device *dev, struct gpio_callback *cb, uint32_t pins){
	printk("Desired Temperature increased : xxºC\r\n");
}
/**
 * @brief Function executed when the button 4 is pressed
 * @param dev pointer to the device
 * @param cb poiter to the callback variable
 * @param pins pin to be executed
 */
void button_pressed3( const struct device *dev, struct gpio_callback *cb, uint32_t pins){
	printk("Desired Temperature decreased : xxºC\r\n");
}

/* callback variables*/
static struct gpio_callback button_cb_data0;
static struct gpio_callback button_cb_data1;
static struct gpio_callback button_cb_data3;


/*UART callback Function*/
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
 switch (evt->type) {  
	case UART_RX_RDY:
		if ((evt->data.rx.len) == 1) {  
			printk("pressing button = %c \r\n",evt->data.rx.buf[evt->data.rx.offset]);

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

	
	if (!device_is_ready(uart)){
 		printk("UART device not ready\r\n");
 		return 1 ;
 	}

	if (!device_is_ready(led0.port)) {
 		printk("LEDS are not ready\r\n");
		return -1;
	}

	if (!device_is_ready(button0.port)) {
 		printk("Buttons not ready\r\n");
		return -1;
	}

	/*configurar LEDs a começarem inativos*/
	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return -1;
	}

	/*configurar Botões como inputs*/
	ret = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_configure_dt(&button3, GPIO_INPUT);
	if (ret < 0) {
		return -1;
	}
	/*configurar UART*/
	ret = uart_callback_set(uart, uart_cb, NULL);
 	if (ret) {
		printk("uart_callback_set() error. Error code:%d\n\r",ret);
 		return 1;
	}

	/*Configurara interrupts*/
	/*botoes*/
	ret = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
	ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
	ret = gpio_pin_interrupt_configure_dt(&button3, GPIO_INT_EDGE_TO_ACTIVE);

	 uint8_t welcome_mesg[] = "UART demo: Type a few chars in a row and then pause for a little while ...\n\r"; 
    

	/*Uart*/
	/*transmissao*/
	ret = uart_tx(uart, tx_buff, sizeof(tx_buff), SYS_FOREVER_US);
 		if (ret) {
 	return 1;
 	}
	/*recessao*/

	/* STEP 6 - Initialize the static struct gpio_callback variable   */
	gpio_init_callback(&button_cb_data0, button_pressed0, BIT(button0.pin));
	gpio_init_callback(&button_cb_data1, button_pressed1, BIT(button1.pin));
	gpio_init_callback(&button_cb_data3, button_pressed3, BIT(button3.pin));

	/* STEP 7 - Add the callback function by calling gpio_add_callback()   */
	gpio_add_callback(button0.port, &button_cb_data0);
	gpio_add_callback(button1.port, &button_cb_data1);
	gpio_add_callback(button3.port, &button_cb_data3);

	while (1) {
		k_msleep(SLEEP_TIME_MS);
	}
}