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
/*Log error*/
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Definições gerais */
#define SLEEP_TIME_MS    1000    ///< Sleep Time
#define MAX_TEMP 120             ///< Absolute maximum temperature
#define OK 0                    ///< Return if everything is alright

static bool table_flag=false;

/* UART RELATED VARIABLES */
#define UART_NODE       DT_NODELABEL(uart0)   ///< UART node ID
#define RBUFF_SIZE      60                    ///< Size of the reception buffer
#define RECEIVE_TIMEOUT 1000                  ///< Receive timeout
#define UART_SAMPLE_PERIOD_MS   15000    ///< Sampling period, in miliseconds 
#define UART_THREAD_STACK_SIZE  1024     ///< Stack Size for the UART
#define UART_THREAD_PRIORITY    3       ///< Thread Priority for the UART 

static uint8_t rx_buff[RBUFF_SIZE];
static uint8_t rx_buff2[RBUFF_SIZE];
static uint8_t rx_msg[RBUFF_SIZE];
volatile int uart_rxbuf_nchar = 0;           ///< Number of chars currently in the rx buffer
static const struct device *uart = DEVICE_DT_GET(UART_NODE);
bool buffer=1;                              ///< Variable for defining wich receive buffer is on

const struct uart_config uart_cfg = {          
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};       ///<struct for UART configuration

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

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

/**
 * \brief UART commands processing function
 * 
 *  This function processes the UART received commands, by checking for SOF and EOF symbols,
 * checking the checksum and then processing and anwering the commands.AAR_ENABLE_ENABLE_Msk
 * 
 * \return OK if all went well, EOF_ERROR if the message does not have a EOF symbol, 
 * SOF_ERROR if the message does not have a SOF symbol, CHECKSUM_ERROR if the received checksum is wrong,
 * ERROR_TOO_HOT if the absolute max temperature is exceeded, COMMAND_ERROR if the command does not exist,
 * FATAL_ERROR for any other errors
 */
int uart_process();

/* --- Definições para a thread de leitura do TC74 --- */
#define TC74_SAMPLE_PERIOD_MS   1000    ///< Sampling period, in miliseconds 
#define TC74_THREAD_STACK_SIZE  512     ///< tStack Size for the TC74
#define TC74_THREAD_PRIORITY    5       ///< Thread Priority for the  TC74

/**
 * \brief function that defines the tc74 K_thread Stack
 * \param tc74_stack name for the stack
 * \param TC74_THREAD_STACK_SIZE size of the stack
 */ 
K_THREAD_STACK_DEFINE(tc74_stack, TC74_THREAD_STACK_SIZE);
static struct k_thread tc74_thread_data;
static void tc74_thread(void *arg1, void *arg2, void *arg3);

/* --- Definições thread de controlo PID→PWM --- */
#define CONTROL_PERIOD_MS 500           ///<Sampling period, in miliseconds 
#define CONTROL_PRIO      4            ///< Stack Size for the PID Controler
#define CONTROL_STACK_SZ  512           ///<Thread Priority for the PID Controler

/**
 * \brief function that defines the PID control K_thread Stack
 * \param control_stack name for the stack
 * \param CONTROL_STACK_SZ size of the stack
 */ 
K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SZ);
static struct k_thread control_data;
static void control_thread(void *arg1, void *arg2, void *arg3);


/* Setting up the buttons */
#define SW0_NODE DT_ALIAS(sw0)  ///< NODE ID for Button 1
#define SW1_NODE DT_ALIAS(sw1)  ///< NODE ID for Button 2
#define SW2_NODE DT_ALIAS(sw2)  ///< NODE ID for Button 3
#define SW3_NODE DT_ALIAS(sw3)  ///< NODE ID for Button 4
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
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
static struct gpio_callback button_cb_data2;
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
        heater_set_power(0);       
        printk("System turned OFF\r\n");
        /*turning the leds off*/
        gpio_pin_set_dt(&led0,0);
        gpio_pin_set_dt(&led1,0);
        gpio_pin_set_dt(&led2,0);
        gpio_pin_set_dt(&led3,0);
        /*suspending the threads*/
        k_thread_suspend(&tc74_thread_data);
        k_thread_suspend(&uart_thread_data);
        k_thread_suspend(&control_data);
    }else{
        rtdb_set_system_on(true);
        printk("System turned ON\r\n");
        gpio_pin_set_dt(&led0,1);
        /*resuming the threads*/
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
    
    if(rtdb_get_system_on()== true){
        int16_t curr_sp=rtdb_get_setpoint()+1;
        rtdb_set_setpoint(curr_sp);
        printk("Desired Temperature increased to %dºC\r\n",curr_sp);
    }

    return;
}

/**
 * @brief Function executed when the button 3 is pressed
 * 
 *  The button 3 is pressed to toggle the printing of the RTDB table,
*/
void button_pressed2(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    
    if(rtdb_get_system_on()== true){
        table_flag=!table_flag;

        if(table_flag){
            printk("RTDB Table ON\n");
        }else{
            printk("RTDB Table OFF\n");

        }
    }

    return;
}

/**
 * @brief Function executed when the button 4 is pressed
 *
 *  This function decreases the desired temperature 1ºC
 */
void button_pressed3(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    
    if(rtdb_get_system_on()== true){
    int16_t curr_sp=rtdb_get_setpoint()-1;
    rtdb_set_setpoint(curr_sp);
    printk("Desired Temperature decreased to %dºC\r\n",curr_sp);
    }
     return;
}



/**
 * \brief main function 
 * \return 0 if everithing is alright, error if something is wrong.
 */
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

    /* Configure UART */
    ret = uart_configure(uart, &uart_cfg);
    if (ret == -ENOSYS) { /* If invalid configuration */
        printk("uart_configure() error. Invalid configuration\n\r");
        return -1; 
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
        !device_is_ready(button2.port) ||
        !device_is_ready(button3.port)) {
        printk("Buttons not ready\r\n");
        return -1;
    }
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

     /* Cria thread de leitura UART */
    k_thread_create(&uart_thread_data, uart_stack,
                    K_THREAD_STACK_SIZEOF(uart_stack),
                    uart_thread, NULL, NULL, NULL,
                    UART_THREAD_PRIORITY, 0, K_NO_WAIT);
    
        /*suspending the threads*/
        k_thread_suspend(&tc74_thread_data);
        k_thread_suspend(&uart_thread_data);
        k_thread_suspend(&control_data);

        printk("SYSTEM START\n");
    /* Loop principal (sleep) */
    while (1) {
        if(rtdb_get_system_on()== true){
            if(table_flag==true){
                rtdb_print();}    
        }
        
        k_msleep(SLEEP_TIME_MS);
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
           /* Proteção de maxTemp */
        if (temp > rtdb_get_maxtemp()) {
            rtdb_set_error_flag(true);
            rtdb_set_system_on(false);      /* desliga o sistema */
            printk("System turned OFF\r\n");
            heater_set_power(0);            /* corta o PWM */
        LOG_ERR("temperatura %d°C excede maxTemp = %d°C, heater OFF",
            temp, rtdb_get_maxtemp());
        }else if (!rtdb_get_system_on() && temp < (rtdb_get_maxtemp() - 4)) {
         // se estiver desligado E já arrefeceu 5°C abaixo do limite
        rtdb_set_error_flag(false);
        rtdb_set_system_on(true);
        LOG_WRN("temperatura %d°C segura, sistema auto-religa", temp);
}

            
            int setp= rtdb_get_setpoint();
            printk("\rTC74-Thread: Temperatura = %d C ; Desejada = %d \n", temp,setp);

            if(temp>rtdb_get_setpoint()+2){
                gpio_pin_set_dt(&led1,0);
                gpio_pin_set_dt(&led2,0);
                gpio_pin_set_dt(&led3,1);
            }else if(temp<rtdb_get_setpoint()-2){
                gpio_pin_set_dt(&led1,0);
                gpio_pin_set_dt(&led2,1);
                gpio_pin_set_dt(&led3,0);
            }else{
                gpio_pin_set_dt(&led1,1);
                gpio_pin_set_dt(&led2,0);
                gpio_pin_set_dt(&led3,0);
            }
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
    rtdb_pid gpid= rtdb_get_pid(); 
    pid_init(&pid, gpid.Kp, gpid.Ti, gpid.Td);  

    while (1) {
        printk("Control Thread: ");
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

static void uart_thread(void *arg1, void *arg2, void *arg3){
   
    while(1) {
        printk("UART-Thread:\n");

        /* Computation */
        k_mutex_lock(&uart_mutex, K_FOREVER);
        if (uart_rxbuf_nchar > 0) {
           uart_process();
        
            uart_rxbuf_nchar = 0;
        }
        k_mutex_unlock(&uart_mutex);

        printk("------\n");
        k_msleep(UART_SAMPLE_PERIOD_MS);
    }

}

/* UART callback function */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    int err;

    switch (evt->type) {
	
        case UART_TX_DONE:
		    //printk("UART_TX_DONE event \n\r");
            break;

    	case UART_TX_ABORTED:
	    	printk("UART_TX_ABORTED event \n\r");
		    break;
		
	    case UART_RX_RDY:
		   
           memcpy(&rx_msg[uart_rxbuf_nchar], &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
           uart_rxbuf_nchar  += evt->data.rx.len;

          // printk("%c; nchar: %d \n", evt->data.rx.buf[evt->data.rx.offset],uart_rxbuf_nchar);
           break;

	    case UART_RX_BUF_REQUEST:
		    if(buffer) {
                uart_rx_buf_rsp(uart, rx_buff, RBUFF_SIZE);}
            else{
                uart_rx_buf_rsp(uart, rx_buff2, RBUFF_SIZE);
            }
            buffer = !buffer;
            
		    break;

	    case UART_RX_BUF_RELEASED:
		   // printk("UART_RX_BUF_RELEASED event \n\r");
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



int uart_process(){
	unsigned int i=0,k=0;


	/* Find index of SOF */
	for(i=0; i < uart_rxbuf_nchar; i++) {
		if(rx_msg[i] == SOF_SYM) {
			break;
		}else if (i == (unsigned int)uart_rxbuf_nchar-1){

            for (size_t j = 0; j < uart_rxbuf_nchar; j++)
            {
                printk("%c", rx_msg[j]);
            }
            printk("\n");

            printk(" Missing Start of Message error.\n");	
			return SOF_ERROR;
		}
	}
	/* Checking correct end of message*/
	for (k = 0; k <= uart_rxbuf_nchar; k++)
	{
		if (rx_msg[k] == EOF_SYM)
		{
			break;

			// Se não acaba com o simbolo que deve dá erro
		}
		else if (k == uart_rxbuf_nchar - 1)
		{
            for (size_t j = i; j < uart_rxbuf_nchar; j++)
            {
                printk("%c", rx_msg[j]);
            }
            printk("\n");
            printk(" Missing End of Message error.\n");	
			return EOF_ERROR;
		}
	}

    for (size_t j = i; j <= k; j++)
    {
        printk("%c", rx_msg[j]);
    }
    printk("\n");

    /*getting the message length*/
    int msg_len=k-i;

	/* Checking correct checksum */
	int chk = calcChecksum(&rx_msg[i + 1], msg_len - 4); // inclui o tipo, sinal e valor
	int chk_recv = char2num(&rx_msg[k - 3], 3);	 // os três dígitos ASCII
    
	// verificar a checksum
	if (chk != chk_recv)
	{
		printk(" Checksum error.\n");	
		return CHECKSUM_ERROR;
	}

	/* If a SOF and EOF, and the checksum is correct look for commands */
	if(i < uart_rxbuf_nchar) {
		//checking the command
		switch(rx_msg[i+1]) { 
			
			case 'M':	
			{	
				/*getting the temperature to be set*/
				int  set_max_temp= char2num(&rx_msg[i+2], 3);
				
				if (set_max_temp > MAX_TEMP)
				{
					return ERROR_TOO_HOT;
				}
					
				rtdb_set_maxtemp((uint8_t)set_max_temp);

				
				printk(" The max temp was set to: %d.\r\n",set_max_temp);
				
				return OK;
			}
			case 'S':
			{
				
				float Kp= char2float(&rx_msg[i+2]);
				float Ti= char2float(&rx_msg[i+6]);
			    float Td= char2float(&rx_msg[i+10]);
				
                rtdb_set_pid(Kp,Ti,Td);
                
                printk("The controller parameters were set.\n");
				
				return OK;
				
			}	
			case 'C':
			{
    			printk(" No error.\n");

                unsigned char msg[]="#cxxxchk!";	
				
				uint8_t temp=rtdb_get_cur_temp();
				num2char(&msg[2],temp);

				int check = calcChecksum(&msg[2], 16);
               
				num2char(&msg[5], check);

                int err = uart_tx(uart, msg, sizeof(msg), SYS_FOREVER_US);
                if (err)
                {
                    return err;
                }
                
                printk("\n");
                return OK;
			}
            case 'R':
            {
                rtdb_reset();
    			printk(" The adjustable parameters were reset.\n");
                return OK;
            }  
            
            case 'D':
            {
                /*getting the temperature to be set*/
				int  set_set= char2num(&rx_msg[i+2], 3);
				
				rtdb_set_setpoint((uint16_t)set_set);
			
				printk(" The setpoint was set to: %d.\r\n",set_set);
				
				return OK;
            } 

			default:
			{

                for (size_t j = 0; j < uart_rxbuf_nchar; j++)
                {
                    printk("%c", rx_msg[j]);
                }
                printk(" Wrong command error.\n");

                return COMMAND_ERROR;	
			}				
		}
		
		
	}else
	
	/* Cmd string not null and SOF not found */
	return FATAL_ERROR;

}

