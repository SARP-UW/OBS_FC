#include "peripheral/gpio.h"
#include "peripheral/watchdog.h"
#include "internal/alloc.h"
#include "peripheral/uart.h"
#include "peripheral/pwm.h"
#include "peripheral/spi.h"
#include "peripheral/systick.h"
#include <stdio.h>

void test_osciloscope() {
	SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);
	WRITE_FIELD(GPIOx_MODER[0], GPIOx_MODER_MODEx[0], 0b01);
	TOGL_FIELD(GPIOx_ODR[0], GPIOx_ODR_ODx[0]);
}

void _start() {
	while (1) {
		test_osciloscope();
	}
}
