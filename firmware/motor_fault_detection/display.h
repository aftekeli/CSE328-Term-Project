#pragma once
#include "state_machine.h"
#include "ei_inference.h"

void display_init();
void display_update(SystemState state, const EIResult &ei, float temp_c);
void display_fault_screen();   // full-screen flashing FAULT message
