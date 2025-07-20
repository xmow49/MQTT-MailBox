#pragma once

#include <Arduino.h>

typedef enum
{
    METER_SOLAR = 0,
    METER_BATTERY = 1,
} meter_t;

void sensor_init();
void sensor_send_values();
void sensor_set_charge(uint32_t value_mWh);
void sensor_send_gates_states();