#pragma once

#include <Arduino.h>
#include "sensor.h"

int mqtt_init();

typedef enum
{
    GATE_LETTER = 0,
    GATE_PARCEL = 1,
    GATE_PIR = 2,
    GATE_UNKNOWN = 3
} gate_t;

void mqtt_sent_gate(gate_t gate, bool state);

void mqtt_send_wifi_infos();

void mqtt_publish_config();

void mqtt_send_loop_state(bool state);

void mqtt_send_temp_hum(float temp, float hum);

void mqtt_publish_meter(meter_t meter, float voltage, float current_ma, float power_mw, float energy_mwh, float storage_energy_mwh);

void mqtt_send_boot_count(uint32_t boot_count);
