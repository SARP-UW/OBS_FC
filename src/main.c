
#include "peripheral/gpio.h"
#include "peripheral/watchdog.h"
#include "internal/alloc.h"
#include "peripheral/uart.h"
#include "peripheral/pwm.h"
#include "peripheral/spi.h"
#include "peripheral/systick.h"
// #include "util/led.h" not here yet?
#include <stdio.h>

#define USR_BUTTON 9
#define GREEN_LED 49
#define YELLOW_LED 139
#define RED_LED 74


/**
 * delay is # of clock cycles, not a time unit
 */
void delay(int delay) {
  for (int i = 0; i < delay; i++) {
    asm("nop");
  }
}

void _start() {
    // Add main loop here!
    return;
}