#include "peripheral/spi.h"
#include <stdio.h>

void test_spi() {
	uint8_t inst = 1;
	uint8_t ss_pins[1] = {43};

	spi_init(inst, ss_pins, 1);

	uint8_t src[1] = {0xAA};
	uint8_t dst[1] = {0x00};

	while (1) {
			asm("BKPT #0");
			int result = spi_transfer_sync(inst, ss_pins[0], src, dst, 1);
			asm("BKPT #0");
	}
}

void _start() {
	test_spi();
}
