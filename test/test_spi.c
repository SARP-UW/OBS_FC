#include "peripheral/gpio.h"
#include "peripheral/watchdog.h"
#include "internal/alloc.h"
#include "peripheral/uart.h"
#include "peripheral/pwm.h"
#include "peripheral/spi.h"
#include "peripheral/systick.h"
#include <stdio.h>

void test_spi() {
	uint8_t instance = 3;
	int result_init = spi_init(instance);
	asm("BKPT #0");

	uint8_t source = 0b10101010;
	uint8_t dest = 0;

	while (1) {
		int result_transfer = spi_transfer_sync(instance, &source, &dest, 1);
		asm("BKPT #0");
	}
}

void _start() {
	test_spi();
}
