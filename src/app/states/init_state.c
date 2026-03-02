//
// Created by Joshua Beard on 1/17/26.
//

#pragma once
#include <stdint.h>
#include "state.h"
#include "init_state.h"

#include "peripheral/gpio.h"
#include "peripheral/qspi.h"


bool init_state_build() {
    //     ~~~ ( ) initializes everything
    //     ~~~ ( ) Establish radio and umbillical communication
    //     ~~~ ( ) Check battery health and power switching?
    //     ~~~ ( ) Verify all actuators and valves in safe position

    ti_errc_t error =  qspi_init();

    return 1;
}

int init_state_run() {
    return -1;
}

bool init_state_destroy() {
    return 1;
}

state build_init_state() {
    const state init_state = {
        .build = &init_state_build,
        .run = &init_state_run,
        .destroy = &init_state_destroy
    };
    return init_state;
}
