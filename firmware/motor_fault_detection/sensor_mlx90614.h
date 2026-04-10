#pragma once
#include <Arduino.h>

bool  mlx90614_init();
float mlx90614_read_celsius();   // returns motor surface (object) temperature
bool  mlx90614_fault(float temp_c);
