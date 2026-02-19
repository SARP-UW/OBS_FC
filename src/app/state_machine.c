//
// Created by Joshua Beard on 1/17/26.
//

// The state declerations are handled in state.h
#include "state.h"

static state states[NUM_STATES];

void setup_states() {
    states[0] = build_init_state();
    // ...additional states
    // TODO: Add the rest
    // TODO: Look into X Macros so we don't have to manually sync this w/ state.h.
}

void run_state_machine() {
    // NOTE: in our final impl, state[0] should be a boot state that also
    // looks in our flash memory to see if we are recovering from a crash,
    // and if so it returns the state to go to (this would be sick)

    state curr = states[0];

    while (1) {
        curr.build();
        const int next_state = curr.run();
        curr.destroy();

        if (next_state == -1) {
            break;
        }
        curr = states[next_state];
    }
}
