/*
 * main.c
 *
 * Created on: 09.04.2020
 *      Author: silvere
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <errno.h>
#include <unistd.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>


#include "systick.h"
#define STM32L1

int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_clock_config[RCC_CLOCK_VRANGE1_HSI_PLL_32MHZ]);

	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART3);
	rcc_periph_clock_enable(RCC_USART2);
}

static void enable_usart3(void)
{
	nvic_enable_irq(NVIC_USART3_IRQ);
	usart_enable_rx_interrupt(USART3);
	usart_enable(USART3);

}

static void disable_usart3(void)
{
	nvic_disable_irq(NVIC_USART3_IRQ);
	usart_disable_rx_interrupt(USART3);
	usart_disable(USART3);
}

static void uart_setup(void)
{
	/* Setup GPIO pins for USART2/3 TX/RX*/
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);

	/* Setup USART3 TX/RX pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF7, GPIO10);
	gpio_set_af(GPIOB, GPIO_AF7, GPIO11);

	/* Setup UART5 TX/RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO3);

	//USART2 setup for printf commands
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);


	//USART3 setup for printf commands
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable USART2. */
	//usart_enable(UART4);
	usart_enable(USART2);
	usart_enable(USART3);
}

static void gpio_setup(void)
{
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
}

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART2, '\r');
			}
			usart_send_blocking(USART2, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

static void put_s(volatile char *c)
{
	while(*c != '\0'){
		usart_send_blocking(USART3, *c);
		c++;
	}
}

static void put_s2(char *c)
{
	enable_usart3();
	while(*c != '\0'){
		usart_send_blocking(USART3, *c);
		gpio_toggle(GPIOA, GPIO0);
		c++;
		msleep(500);
	}
}

//static char* get_s(void)
//{
//	char* rcv = (char*) malloc(10);
//	int i = 0;
//	do{
//
//		rcv[i] =  usart_recv_blocking(USART3);
//		gpio_toggle(GPIOA, GPIO0);
//		i++;
//	}
//	while(rcv[i] != '0');
//	return rcv;
//}

static volatile char recv[30];
int j = 0;

void usart2_isr(void)
{
	if(usart_get_flag(USART3, USART_SR_RXNE)){
		gpio_toggle(GPIOA, GPIO1);
		recv[j] = usart_recv(USART3);
		j++;
	}
	if(j == 5){
		j = 0;
		disable_usart3();
		put_s(recv);
	}
}

//static void toogle(void)
//{
//	uint32_t i;
//	//gpio_toggle(GPIOA, GPIO1);
//	put_s2("LED ON \r\n\0");
//	msleep(2000);
//	put_s2("LED OFF \r\n\0");
//}

int main(void)
{
	clock_setup();
	systick_ms_setup();
	gpio_setup();
	uart_setup();

	printf("Prog Start! \n");

	while(1){

		put_s2("LED ON \r\n\0");
	}
	return 0;
}
