#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <DHT.h>
#include <time.h>
#include <ESP32Time.h>
#include <PubSubClient.h>

#include "sensor.h"
#include "config.h"
#include "logs.h"
#include "mqtt.h"

Adafruit_INA219 solar_pannel_meter(0x40);
Adafruit_INA219 battery_meter(0x45);
DHT dht(PIN_DHT, DHTTYPE); // dht temperature sensor

extern ESP32Time rtc;
extern uint32_t last_boot_time;

RTC_DATA_ATTR float solar_energy_mWh = 0;
RTC_DATA_ATTR float battery_energy_mWh = 0;
RTC_DATA_ATTR float storage_energy_mWh = 0;

void sensor_init()
{
    pinMode(PIN_POWER_INA219, OUTPUT); // Power INA219
    pinMode(PIN_PIR, INPUT);           // temporary, to avoid data pin as GND

    digitalWrite(PIN_POWER_INA219, HIGH);

    if (!solar_pannel_meter.begin())
    {
        logs("Failed to initialize INA219 solar_pannel_meter\n");
    }

    if (!battery_meter.begin())
    {
        logs("Failed to initialize INA219 battery_meter\n");
    }

    solar_pannel_meter.setCalibration_32V_1A();
    battery_meter.setCalibration_32V_1A();
}

void sensor_send_values()
{
    float busvoltage = 0;
    float current_mA = 0;
    float power_mW = 0;
    float newConsommation = 0;

    busvoltage = battery_meter.getBusVoltage_V();
    current_mA = battery_meter.getCurrent_mA();
    power_mW = battery_meter.getPower_mW();

    uint32_t time = rtc.getEpoch() - last_boot_time;
    last_boot_time = rtc.getEpoch();

    if (busvoltage < 6.0) // security
    {
        newConsommation = (POWER_DEEPSLEEP_MW) * ((float)time / (60 * 60)); // time s in deepsleep
        float currentConsommation = (power_mW) * ((float)15 / (60 * 60));   // 15 sec of active
        battery_energy_mWh = battery_energy_mWh + newConsommation + currentConsommation;
        storage_energy_mWh -= newConsommation - currentConsommation;
        mqtt_publish_meter(METER_BATTERY, busvoltage, current_mA, power_mW, battery_energy_mWh, storage_energy_mWh);
    }

    // shuntvoltage = solar_pannel_meter.getShuntVoltage_mV();
    busvoltage = solar_pannel_meter.getBusVoltage_V();
    current_mA = solar_pannel_meter.getCurrent_mA();
    power_mW = solar_pannel_meter.getPower_mW();

    if (busvoltage < 6.0)
    {
        newConsommation = (power_mW) * ((float)time / (60 * 60));
        solar_energy_mWh = solar_energy_mWh + newConsommation;
        storage_energy_mWh += newConsommation;
        mqtt_publish_meter(METER_SOLAR, busvoltage, current_mA, power_mW, solar_energy_mWh, storage_energy_mWh);
    }

    /*float battery = (analogRead(PIN_BATTERY) / (4095 / 3.3) / 0.785714); // Get Battery voltage: analogRead(PIN_BATTERY): 0->4095
                                                                        //                      analogRead(PIN_BATTERY) / (4095 / 3.3): 0V->3.3V of analog pin
                                                                        // 0.785714 is the ratio of the voltage divider bridge with 27k and 100K: 0V->4.2V
    Serial.println("Battery level: " + String(battery) + "V");
    Serial.println("Sending values...");*/

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
    storage_energy_mWh = (float)value_mWh;
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