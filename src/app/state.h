//
// Created by Joshua Beard on 1/17/26.
//

#pragma once
#include <stdbool.h>

#define NUM_STATES 8

typedef struct {

    bool (*build)(void); // setup info, if needed; setup info size
    /**
     * blocks until the current state ends
     * The returned int is the index of the state in the state_machine.c file where
     * the program should go to next.
     *
     * returning -1 makes the state machine exit after this state finishes
     */
    int (*run)(void);
    bool (*destroy)(void); // called by state machine
} state;

// State declarations / promises
// Check corresponding .c files for details.
extern const state init_state;
extern const state standby_state;
extern const state filling_state;
extern const state hold_state;
extern const state armed_state;
extern const state firing_state;
extern const state safe_state;
extern const state recover_state;

