
#include "peripheral/gpio.h"
#include "peripheral/watchdog.h"
#include "internal/alloc.h"
#include "peripheral/uart.h"
#include "peripheral/pwm.h"
#include "peripheral/spi.h"
#include "peripheral/systick.h"
#include "util/led.h"
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

extern uint32_t __heap_start;


void test_spi() {
    spi_config_t config;
    uint8_t instance          = 3;
    config.clk_pin            = 109; // alt mode 4
    config.mosi_pin           = 111; // alt mode 3 //Instace 3
    config.miso_pin           = 110; // alt mode 3
    config.data_size          = 8;
    config.mode               = 0;
    config.baudrate_prescaler = 2;
    config.first_bit          = 0;
    config.priority           = 0;
    config.mutex_timeout      = 0;
    int result_init = spi_init(instance, &config);
    asm("BKPT #0");

    spi_device_t device;
    device.instance = instance;
    device.gpio_pin = 0;

    struct spi_sync_transfer_t transfer;
    uint8_t* source;
    (*source) = 0b10101010;
    transfer.source = source;
    uint8_t* dest;
    transfer.dest = dest;
    size_t size = 1;
    transfer.size = size;
    uint32_t timeout = 10000000;
    transfer.timeout = timeout;
    bool read_inc = false;
    transfer.read_inc = read_inc;
    asm("BKPT #0");

    // bool is_first = false;
    // uint8_t expected_byte = 0xAA; //0b10101010
    // while (1) {
    //     int result = spi_transfer_sync(&transfer);
    //     if (result != 0) {
    //         printf("There was an error in spi_transfer_sync()");
    //     }

    //     printf("Byte Expected: %02x", (is_first) ? 0x00 : expected_byte);
    //     printf("Byte Recieved: %02x", *dest);

    //     is_first = true;
    //     asm("BKPT #0");
    // }

    while (1) {
        int result_transfer = spi_transfer_sync(&transfer);
        asm("BKPT #0");
    }
}


void test_uart(){
    uart_config_t config;
    uart_channel_t channel = UART4;
    uart_parity_t parity = UART_PARITY_DISABLED;
    uart_datalength_t data_length = UART_DATALENGTH_8;
    config.channel = channel;
    config.parity = parity;
    config.data_length = data_length;
    config.baud_rate = 9600;
    config.clk_freq = 4000000; // 60 MHz

    // UART 1-3, 6 is fine (maybe)
    // UART 4-5, probably 7/8 isn't working (maybe)




    // asm("BKPT #0");
    int n = uart_init(&config, (void*) (0), (void*) (0), (void*) (0));
    asm("BKPT #0");
    // int n = 1;

    uint8_t tx = 0xAA;

    while (1) {
        uart_write_blocking(channel, &tx, 1);
    }
}

// void test_pwm(){
//     tal_pwm_pin_init(TIM2_CH1_1, 1000, 30000, (void*)0);

//     //----------
//     // should be done inside the PWM driver.  Here for testing
//     tal_enable_clock(TIM2_CH1_1);
//     SET_FIELD(RCC_APB1LENR, RCC_APB1LENR_TIMxEN[2]); // enable clock for TIM2

//     tal_set_mode(TIM2_CH1_1, 2);
//     tal_alternate_mode(TIM2_CH1_1, 0);
//     //----------


//     tal_pwm_pin_enable(TIM2_CH1_1, (void*)0);
//     asm("BKPT #0");
//     tal_pwm_pin_disable(TIM2_CH1_1, (void*)0);
//     asm("BKPT #0");
// }

void test_pwm_first_principles(){


    tal_enable_clock(GREEN_LED);
    tal_enable_clock(YELLOW_LED);
    tal_set_mode(YELLOW_LED, 1);
    tal_set_pin(YELLOW_LED, 1); // boot successful

    tal_set_mode(GREEN_LED, 2);
    tal_alternate_mode(GREEN_LED, 2);

    uint16_t freqency = 32; // unused atm

    SET_FIELD(RCC_APB1LENR, RCC_APB1LENR_TIMxEN[3]);

    WRITE_FIELD(G_TIMx_ARR[3], G_TIMx_ARR_ARR_L, (1 << 16) - 1);
    WRITE_FIELD(G_TIMx_ARR[3], G_TIMx_ARR_ARR_H, (1 << 16) - 1);
    WRITE_FIELD(G_TIMx_CCR3[3], G_TIMx_CCR3_CCR3_L, (1 << 16) - 1);
    WRITE_FIELD(G_TIMx_CCR3[3], G_TIMx_CCR3_CCR3_H, (1 << 16) - 1);

    field32_t G_TIMx_CCMR2_OUTPUT_OC3M = {
        .msk = (0b111 << 4),
        .pos = 4
    };
    field32_t G_TIMx_CCMR2_OUTPUT_OC3PE = {
        .msk = 0b1 << 3,
        .pos = 3
    };

    WRITE_FIELD(G_TIMx_CCMR2_OUTPUT[3], G_TIMx_CCMR2_OUTPUT_OC3M, 0b0110); // configure as PWM output
    SET_FIELD(G_TIMx_CCMR2_OUTPUT[3], G_TIMx_CCMR2_OUTPUT_OC3PE);
    SET_FIELD(G_TIMx_CR1[3], G_TIMx_CR1_ARPE);

    SET_FIELD(G_TIMx_CCER[3], G_TIMx_CCER_CCxE[3]);
    SET_FIELD(G_TIMx_CR1[3], G_TIMx_CR1_CEN);

    uint16_t duty_cycle = UINT16_MAX;
    int inc = -1;

    while (true) {
        duty_cycle += inc;
        if (duty_cycle == 0 || duty_cycle == UINT16_MAX) {
            inc = -inc;
        }

        WRITE_FIELD(G_TIMx_CCR3[3], G_TIMx_CCR3_CCR3_L, duty_cycle);

        delay(100);
    }


    // CLR_FIELD(G_TIMx_CR1[3], G_TIMx_CR1_CEN);
}

// field32_t G_TIMx_CCMR2_OUTPUT_OCxM[5] = {
//     [3] = {
//         .msk = (0b111 << 4),
//         .pos = 4
//     },
//     [4] = {
//         .msk = (0b111 << 12),
//         .pos = 12
//     }
// };

// field32_t G_TIMx_CCMR2_OUTPUT_OCxPE[5] = {
//     [3] = {
//         .msk = 0b1 << 3,
//         .pos = 3
//     },
//     [4] = {
//         .msk = 0b1 << 11,
//         .pos = 11
//     }
// };

// field32_t G_TIMx_CCMR1_OUTPUT_OCxM[5] = {
//     [1] = {
//         .msk = (0b111 << 4),
//         .pos = 4
//     },
//     [2] = {
//         .msk = (0b111 << 12),
//         .pos = 12
//     }
// };

// field32_t G_TIMx_CCMR1_OUTPUT_OCxPE[5] = {
//     [1] = {
//         .msk = 0b1 << 3,
//         .pos = 3
//     },
//     [2] = {
//         .msk = 0b1 << 11,
//         .pos = 11
//     }
// };

// struct ti_pwm_config_t {
//     int32_t channel;
//     int32_t alt_num;
//     int32_t pin;
//     int32_t freq;
//     int32_t duty;
// };

// # define PWM_CLOCK_FREQ 4000000

// void ti_set_pwm(int32_t pwm_inst, struct ti_pwm_config_t config) {
//     if (config.freq < 0 || config.duty < 0) {
//         // Do error stuff here
//         return;
//     }
//     if (config.freq == 0 || config.duty == 0) {
//         CLR_FIELD(RCC_APB1LENR, RCC_APB1LENR_TIMxEN[pwm_inst]);
//         return;
//     }
//     // Enable peripheral clock
//     SET_FIELD(RCC_APB1LENR, RCC_APB1LENR_TIMxEN[pwm_inst]);

//     // Set up GPIO pin
//     tal_enable_clock(config.pin);
//     tal_alternate_mode(config.pin, config.alt_num);
//     tal_set_mode(config.pin, 2);

//     // Set frequency of timer
//     const int32_t max_val = PWM_CLOCK_FREQ / config.freq;
//     WRITE_FIELD(G_TIMx_ARR[3], G_TIMx_ARR_ARR_L, max_val);

//     // Set duty cycle
//     WRITE_FIELD(G_TIMx_CCR3[3], G_TIMx_CCR3_CCR3_L, (max_val / 1000) * config.duty);

//     // Set to output compare
//     if (config.channel >= 3) {
//         WRITE_FIELD(G_TIMx_CCMR2_OUTPUT[pwm_inst], G_TIMx_CCMR2_OUTPUT_OCxM[config.channel], 0b0110);
//         SET_FIELD(G_TIMx_CCMR2_OUTPUT[pwm_inst], G_TIMx_CCMR2_OUTPUT_OCxPE[config.channel]);
//     } else {
//         WRITE_FIELD(G_TIMx_CCMR1_OUTPUT[pwm_inst], G_TIMx_CCMR1_OUTPUT_OCxM[config.channel], 0b0110);
//         SET_FIELD(G_TIMx_CCMR1_OUTPUT[pwm_inst], G_TIMx_CCMR2_OUTPUT_OCxPE[config.channel]);
//     }
//     // Do some magic
//     SET_FIELD(G_TIMx_CCER[pwm_inst], G_TIMx_CCER_CCxE[config.channel]);
//     SET_FIELD(G_TIMx_CR1[pwm_inst], G_TIMx_CR1_CEN);

//     // Enable PWM output
//     SET_FIELD(G_TIMx_CR1[pwm_inst], G_TIMx_CR1_ARPE);
// }

void test_osciloscope() {
    // Bus clock
    SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);

    // Pin output
    WRITE_FIELD(GPIOx_MODER[0], GPIOx_MODER_MODEx[0], 0b01);

    // Raise high then low then high...
    TOGL_FIELD(GPIOx_ODR[0], GPIOx_ODR_ODx[0]);
}

void _start() {

    // struct ti_pwm_config_t pwm_config = {
    //     .channel = 3,
    //     .alt_num = 2,
    //     .pin = 49,
    //     .freq = 100,
    //     .duty = 0,
    // };

    // int32_t dir = 1;
    // enum ti_errc_t my_err;
    // enum ti_errc_t* errc = &my_err;
    // asm("BKPT #0");
    // while (true) {
    //     ti_set_pwm(3, pwm_config, errc);
    //     enum ti_errc_t err = *errc;
    //     // asm("BKPT #0");
    //     pwm_config.duty += dir;
    //     if (pwm_config.duty == 1000 || pwm_config.duty == 0) {
    //         dir *= -1;
    //     }
    //     delay(1000);
    // }

    // test_pwm_first_principles();
    // tal_enable_clock(GREEN_LED);
    // tal_enable_clock(RED_LED);
    // tal_enable_clock(YELLOW_LED);
    // tal_enable_clock(USR_BUTTON);


    // tal_set_mode(GREEN_LED, 1);
    // tal_set_mode(RED_LED, 1);
    // tal_set_mode(YELLOW_LED, 1);
    // tal_set_mode(USR_BUTTON, 0);

    // HEAP_START = &__heap_start;
    // int heap_status = init_heap();

    // if(heap_status != 1){
    //     asm("BKPT #0"); // heap init failure
    // }

    // tal_set_pin(YELLOW_LED, 1);


    // test_pwm();
    //test_uart();
    test_spi();
    //test_osciloscope();
}