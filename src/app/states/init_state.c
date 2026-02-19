//
// Created by Joshua Beard on 1/17/26.
//

#pragma once
#include <stdint.h>
#include "state.h"
#include "init_state.h"



// ----- STATELESS FUNCTIONS -----
bool do_init_thing_0() {
    __builtin_printf("do_init_thing_0\n");
    return 1;
}

bool do_init_thing_1() {
    __builtin_printf("do_init_thing_1\n");
    return 1;
}
// -------------------------------


// ----- STATEFUL FUNCTIONS -----
bool do_stateful_thing_0(uint8_t* data, uint32_t data_size) {
    __builtin_printf("do_stateful_thing_0\n");
    __builtin_printf((char*)data);
    __builtin_printf("\n");

    return 1;
}
// -------------------------------


// BUILD, RUN, DESTROY
bool init_state_build() {
    __builtin_printf("init_state_build\n");
    return 1;
}

int init_state_run() {
    __builtin_printf("init_state_run\n");

    do_init_thing_0();
    do_init_thing_1();

    char a = 'a';
    do_stateful_thing_0(&a, 1);

    int i = 0;

    while (i < 1000000) { // while state should run, block, do things the state does here
        i++;
    }

    return -1;
}

bool init_state_destroy() {
    __builtin_printf("init_state_destroy\n");

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
