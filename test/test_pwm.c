#include "peripheral/gpio.h"
#include "peripheral/pwm.h"
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

    struct ti_pwm_config_t pwm_config = {
        .channel = 3,
        .instance = 3,
        .clock_freq = 4000000,
        .freq = 100,
        .duty = 0,
    };

    int32_t dir = 1;
    enum ti_errc_t my_err;
    enum ti_errc_t* errc = &my_err;
    asm("BKPT #0");
    while (true) {
        ti_set_pwm(pwm_config, errc);
        enum ti_errc_t err = *errc;
        // asm("BKPT #0");
        pwm_config.duty += dir;
        if (pwm_config.duty == 1000 || pwm_config.duty == 0) {
            dir *= -1;
        }
        delay(1000);
    }

}