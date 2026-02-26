#include "peripheral/gpio.h"
#include "peripheral/watchdog.h"
#include "internal/alloc.h"
#include "peripheral/uart.h"
#include "peripheral/pwm.h"
#include "peripheral/spi.h"
#include "peripheral/systick.h"
#include <stdio.h>

void test_uart(){
	uart_config_t config;
	uart_channel_t channel = UART4;
	uart_parity_t parity = UART_PARITY_DISABLED;
	uart_datalength_t data_length = UART_DATALENGTH_8;
	config.channel = channel;
	config.parity = parity;
	config.data_length = data_length;
	config.baud_rate = 9600;
	config.clk_freq = 4000000;

	int n = uart_init(&config, (void*) (0), (void*) (0), (void*) (0));
	asm("BKPT #0");

	uint8_t tx = 0xAA;

	while (1) {
		uart_write_blocking(channel, &tx, 1);
	}
}

void _start() {
	test_uart();
}
