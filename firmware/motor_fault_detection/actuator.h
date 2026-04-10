#pragma once
#include "state_machine.h"

void actuator_init();
void actuator_update(SystemState state);
void actuator_motor_enable();
void actuator_motor_disable();
