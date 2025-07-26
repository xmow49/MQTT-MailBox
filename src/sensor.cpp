#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <DHT.h>
#include <time.h>
#include <ESP32Time.h>
#include <PubSubClient.h>
#include <EEPROM.h>

#include "sensor.h"
#include "config.h"
#include "logs.h"
#include "mqtt.h"

Adafruit_INA219 solar_pannel_meter(0x40);
Adafruit_INA219 battery_meter(0x45);
DHT dht(PIN_DHT, DHTTYPE); // dht temperature sensor

extern ESP32Time rtc;
extern uint32_t last_boot_time;

TaskHandle_t sensor_task_handle = NULL;

typedef struct
{
    bool is_valid;
    float bus_voltage_V;
    float current_mA;
    float power_mW;
} meter_data_t;

RTC_DATA_ATTR bool values_loaded = false; // flag to check if values are loaded from EEPROM
RTC_DATA_ATTR float solar_energy_mWh = 0;
RTC_DATA_ATTR float battery_energy_mWh = 0;

meter_data_t last_solar_data = {0};
meter_data_t last_battery_data = {0};

void sensor_task(void *pvParameters);

void sensor_init()
{
    pinMode(PIN_POWER_INA219, OUTPUT); // Power INA219
    digitalWrite(PIN_POWER_INA219, HIGH);

    pinMode(PIN_PIR, INPUT); // temporary, to avoid data pin as GND

    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for INA219 to power up

    if (!battery_meter.begin())
    {
        logs("Failed to initialize INA219 battery_meter\n");
    }
    if (!solar_pannel_meter.begin())
    {
        logs("Failed to initialize INA219 solar_pannel_meter\n");
    }

    if (!values_loaded)
    {
        logs("Loading energy values from EEPROM...\n");
        if (isnan(config.last_solar_energy_mWh))
        {
            config.last_solar_energy_mWh = 0.0f;
            solar_energy_mWh = config.last_solar_energy_mWh;
            EEPROM.put(0, config); // write config to EEPROM
            EEPROM.commit();
        }
        else
        {
            solar_energy_mWh = config.last_solar_energy_mWh;
        }

        if (isnan(config.last_battery_energy_mWh))
        {
            config.last_battery_energy_mWh = 0.0f;
            battery_energy_mWh = config.last_battery_energy_mWh;
            EEPROM.put(0, config); // write config to EEPROM
            EEPROM.commit();
        }
        else
        {
            battery_energy_mWh = config.last_battery_energy_mWh;
        }

        values_loaded = true;
    }

    solar_pannel_meter.setCalibration_32V_1A();
    battery_meter.setCalibration_32V_1A();

    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 1, &sensor_task_handle, 1);
}

void sensor_send_values()
{
    mqtt_publish_meter(METER_BATTERY, last_battery_data.bus_voltage_V, last_battery_data.current_mA, last_battery_data.power_mW, battery_energy_mWh);
    mqtt_publish_meter(METER_SOLAR, last_solar_data.bus_voltage_V, last_solar_data.current_mA, last_solar_data.power_mW, solar_energy_mWh);

    dht.begin(); // Setup DHT

    logs("Reading Temperatures...\n");
    int hum = dht.readHumidity();       // get temperature
    float temp = dht.readTemperature(); // get humidity

    // Display them:
    logs("  Temperature: %d°C\n", (int)temp);
    logs("  Humidity: %f%%\n", hum);

    // Send them:
    logs("Sending values...\n");
    if (!isnan(temp) && !isnan(hum))
    {
        mqtt_send_temp_hum(temp, hum);
    }
}

void sensor_set_charge(uint32_t value_mWh)
{
    battery_energy_mWh = (float)value_mWh;
    config.last_battery_energy_mWh = battery_energy_mWh;
    EEPROM.put(0, config); // write config to EEPROM
    EEPROM.commit();
}

void sensor_send_gates_states()
{
    static char parcel = -1;
    static char letter = -1;

    char new_parcel = digitalRead(PIN_PARCEL);
    char new_letter = digitalRead(PIN_LETTER);

    if (new_parcel != parcel || parcel == -1)
    {
        logs("Sending parcel state: %d\n", new_parcel);
        parcel = new_parcel;
        mqtt_sent_gate(GATE_PARCEL, new_parcel);
    }

    if (new_letter != letter || letter == -1)
    {
        logs("Sending letter state: %d\n", new_letter);
        letter = new_letter;
        mqtt_sent_gate(GATE_LETTER, new_letter);
    }
}

bool sensor_is_any_gate_open()
{
    bool parcel = digitalRead(PIN_PARCEL);
    bool letter = digitalRead(PIN_LETTER);
    bool pir = digitalRead(PIN_PIR);
    return (parcel || letter || pir);
}

void sensor_task(void *pvParameters)
{
    uint32_t time = rtc.getEpoch() - last_boot_time;
    float battery_deep_sleep_energy_mWh = (POWER_DEEPSLEEP_MW) * ((float)time / (60 * 60)); // time s in deepsleep
    battery_energy_mWh -= battery_deep_sleep_energy_mWh;
    logs("Battery deep sleep energy: %f mWh in %d s; time: %d, last_boot_time: %d\n", battery_deep_sleep_energy_mWh, time, rtc.getEpoch(), last_boot_time);

    // solar energy in deepsleep extrapolation
    float solar_current_mA = solar_pannel_meter.getCurrent_mA();
    float solar_deep_sleep_energy_mWh = (solar_current_mA) * ((float)time / (60 * 60)); // time s in deepsleep
    solar_energy_mWh += solar_deep_sleep_energy_mWh;
    logs("Solar deep sleep energy: %f mWh in %d s; time: %d, last_boot_time: %d\n", solar_energy_mWh, time, rtc.getEpoch(), last_boot_time);

    while (true)
    {
        float battery_busvoltage = battery_meter.getBusVoltage_V();
        float battery_current_mA = battery_meter.getCurrent_mA();
        float battery_power_mW = battery_meter.getPower_mW();

        if (battery_busvoltage < 6.0) // security
        {
            float battery_active_energy_mwh = (battery_power_mW) * (((float)(SENSOR_REFRESH_RATE_MS) / 1000.0f) / (60.0f * 60.0f));
            battery_energy_mWh -= battery_active_energy_mwh;

            last_battery_data.bus_voltage_V = battery_busvoltage;
            last_battery_data.current_mA = battery_current_mA;
            last_battery_data.power_mW = battery_power_mW;
            last_battery_data.is_valid = true;
            config.last_battery_energy_mWh = battery_energy_mWh;

            // logs("Battery: %f V, %f mA, %f mW, energy: %f mWh\n", battery_busvoltage, battery_current_mA, battery_power_mW, battery_energy_mWh);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_REFRESH_RATE_MS / 2)); // Wait for SENSOR_REFRESH_RATE_MS ms before next reading

        // shuntvoltage = solar_pannel_meter.getShuntVoltage_mV();
        float solar_busvoltage = solar_pannel_meter.getBusVoltage_V();
        float solar_current_mA = solar_pannel_meter.getCurrent_mA();
        float solar_power_mW = solar_pannel_meter.getPower_mW();

        if (solar_busvoltage < 6.0)
        {
            float solar_active_energy_mwh = (solar_power_mW) * (((float)(SENSOR_REFRESH_RATE_MS) / 1000.0f) / (60.0f * 60.0f));
            solar_energy_mWh += solar_active_energy_mwh;
            battery_energy_mWh += solar_active_energy_mwh;
            config.last_solar_energy_mWh = solar_energy_mWh;

            last_solar_data.bus_voltage_V = solar_busvoltage;
            last_solar_data.current_mA = solar_current_mA;
            last_solar_data.power_mW = solar_power_mW;
            last_solar_data.is_valid = true;

            // logs("Solar:   %f V, %f mA, %f mW, energy: %f mWh\n", solar_busvoltage, solar_current_mA, solar_power_mW, solar_energy_mWh);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_REFRESH_RATE_MS / 2));
    }

    vTaskDelete(NULL); // Delete the task when done
}