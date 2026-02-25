//
// Created by Joshua Beard on 1/17/26.
//

#pragma once
#include <stdbool.h>

typedef struct {
    bool (*build)(void); 
    int (*run)(void);
    bool (*destroy)(void); 
} state;

// State declarations / promises
extern const state init_state;
extern const state standby_state;
extern const state filling_state;
extern const state hold_state;
extern const state armed_state;
extern const state firing_state;
extern const state safe_state;
extern const state recover_state;

//     -- ( ) INITIALIZATION: First State, triggered upon power up
//     ~~~ ( ) initializes everything
//     ~~~ ( ) Establish radio and umbillical communication
//     ~~~ ( ) Check battery health and power switching?
//     ~~~ ( ) Verify all actuators and valves in safe position
// 
//     -- ( ) STANDBY: Rocket on pad awaiting instruction
//     ~~~ ( ) Monitor ambient tank pressure and tempreature
//     ~~~ ( ) Await the start fill command 
//     ~~~ ( ) Remote venting capability is active to safeguard system
// 
//     --- ( ) FILLING: Remote delivery of fuel
//     ~~~ ( ) Actuate GSE Valves to load propellant
//     ~~~ ( ) Monitor mass flow and pressure 
//     ~~~ ( ) If overpressurization is detected, auto abort to SAFE/ABORT
// 
//     - ( ) HOLD: Maintain launch ready state
//     ~~~ ( ) Monitor leaks or pressure drops
//     ~~~ ( ) Keep GSE Connected for power/data
//     ~~~ ( ) Monitoro nitros phase changes
// 
//     -- ( ) ARMED: Final Countdown Sequence and umbilical disconnect
//     ~~~ ( ) switch power from umbilical to battery 
//     ~~~ ( ) disconnect umbillical
// 
//     -- ( ) FIRING: Deliver propelants to manifold
//     ~~~ ( ) Open main propelant valves
//     ~~~ ( ) Execute control loop for engine burn
//     ~~~ ( ) REcord telemetry to Sd card
// 
//     -- ( ) SAFE/ABORT: Venting to abort mission
//     ~~~ ( ) Vent everything 
// 
//     - ( ) AFTER/RECOVERY
//     ~~~ ( ) close all valves to prevent leakages
//     ~~~ ( ) stop sending data to ground, maybe location


