#include <Arduino.h>
#include <WiFi.h>

#include <PubSubClient.h>
#include <EEPROM.h>

#include "mqtt.h"
#include "config.h"
#include "logs.h"
#include "sensor.h"

WiFiClient espClient;                // Wifi
PubSubClient mqtt_client(espClient); // MQTT

TaskHandle_t mqtt_task_handle = NULL;
static void mqtt_new_message(char *topic, byte *payload, unsigned int length);
static void mqtt_task(void *pvParameters);

int mqtt_init()
{
    mqtt_client.setServer(MQTT_SERVER, 1883);                             // Setup MQTT Server
    if (mqtt_client.connect("BoiteAuxLettres", MQTT_USER, MQTT_PASSWORD)) // Connect to MQTT server
    {
        mqtt_client.setCallback(mqtt_new_message);
        mqtt_client.subscribe(TOPIC_CONFIG_DEEPSLEEP); // Subscribe to configuration topics
        mqtt_client.subscribe(TOPIC_CONFIG_AVOID_MULTIPLE_BOOT);
        mqtt_client.subscribe(TOPIC_CONFIG_BRIGHTNESS);
        mqtt_client.subscribe(TOPIC_CONFIG_CHARGE);
        mqtt_client.subscribe(TOPIC_CONFIG_PIR);
        mqtt_client.subscribe(TOPIC_CONFIG_REBOOT);
        mqtt_client.subscribe(TOPIC_SET_STORAGE);

        xTaskCreatePinnedToCore(mqtt_task, "MQTT", 4096, NULL, 7, &mqtt_task_handle, 1);

        return 0; // Connected to MQTT server
    }
    return mqtt_client.state(); // Failed to connect to MQTT server
}

static void mqtt_new_message(char *topic, byte *payload, unsigned int length)
{
    // Serial.println(topic);
    if (strcmp(topic, TOPIC_CONFIG_DEEPSLEEP) == 0)
    {
        switch ((char)payload[0])
        {
        case '1':
            config.deepSleep = 1;
            break;
        case '0':
            config.deepSleep = 0;
            break;
        default:
            break;
        }
        logs("New deepsleep config: %d\n", config.deepSleep);
    }
    if (strcmp(topic, TOPIC_CONFIG_AVOID_MULTIPLE_BOOT) == 0)
    {
        switch ((char)payload[0])
        {
        case '1':
            config.avoidMultipleBoot = 1;
            break;
        case '0':
            config.avoidMultipleBoot = 0;
            break;
        default:
            break;
        }
        logs("New avoid multiple boot config: %d\n", config.avoidMultipleBoot);
    }
    if (strcmp(topic, TOPIC_CONFIG_BRIGHTNESS) == 0)
    {
        if (length > 3)
            return;
        if (length == 2)
            payload[2] = '\0';
        int value = atoi((char *)payload);
        if (value < 0 || value > 255)
            return;
        config.brightness = value;
        logs("New brightness config: %d\n", config.brightness);
    }
    if (strcmp(topic, TOPIC_CONFIG_CHARGE) == 0)
    {
        if (length > 5)
            return;
        if (length == 2)
            for (int i = 2; i < 5; i++)
                payload[i] = '\0';
        uint32_t value = atoi((char *)payload);
        sensor_set_charge(value);
        logs("New charge config: %d\n", value);
    }
    if (strcmp(topic, TOPIC_CONFIG_PIR) == 0)
    {
        switch ((char)payload[0])
        {
        case '1':
            config.pir_sensor = 1;
            break;
        case '0':
            config.pir_sensor = 0;
            break;
        default:
            break;
        }
        logs("New PIR config: %d\n", config.pir_sensor);
    }
    if (strcmp(topic, TOPIC_CONFIG_REBOOT) == 0)
    {
        switch ((char)payload[0])
        {
        case '1':
            logs("Rebooting...\n");
            ESP.restart();
            break;
        case '0':
            break;
        default:
            break;
        }
    }
    EEPROM.put(0, config); // write config to EEPROM
    EEPROM.commit();
}

void mqtt_sent_gate(gate_t gate, bool state)
{
    const char *topic;
    switch (gate)
    {
    case GATE_LETTER:
        topic = TOPIC_LETTER;
        break;
    case GATE_PARCEL:
        topic = TOPIC_PARCEL;
        break;
    case GATE_PIR:
        topic = TOPIC_PIR;
        break;
    default:
        logs("Unknown gate type: %d\n", gate);
        return;
    }

    mqtt_client.publish(topic, state ? "1" : "0"); // Send ON or OFF to MQTT topic
}

void mqtt_publish_config()
{
    mqtt_client.publish(TOPIC_CONFIG_DEEPSLEEP, String((int)config.deepSleep).c_str());
    mqtt_client.publish(TOPIC_CONFIG_AVOID_MULTIPLE_BOOT, String((int)config.avoidMultipleBoot).c_str());
}

void mqtt_send_wifi_infos()
{
    int8_t rssi = WiFi.RSSI();
    logs("RSSI: %d\n", rssi);

    char payload[50];
    snprintf(payload, sizeof(payload), "%d", rssi);
    mqtt_client.publish(TOPIC_WIFI, payload); // Send RSSI to MQTT topic

    mqtt_client.publish(TOPIC_LOGS, logs_get_buffer(false)); // send logs to MQTT server
    IPAddress ip = WiFi.localIP();

    mqtt_client.publish(TOPIC_IP, ip.toString().c_str());
}

static void mqtt_task(void *pvParameters)
{
    while (true)
    {
        mqtt_client.loop();
        vTaskDelay(100 / portTICK_PERIOD_MS); // Check MQTT messages every 100ms
    }
}

void mqtt_send_loop_state(bool state)
{
    mqtt_client.publish(TOPIC_LOOP_STATE, state ? "1" : "0");
}

void mqtt_send_temp_hum(float temp, float hum)
{
    mqtt_client.publish(TOPIC_TEMP, String(temp).c_str());
    mqtt_client.publish(TOPIC_HUM, String(hum).c_str());
}

void mqtt_publish_meter(meter_t meter, float voltage, float current_ma, float power_mw, float energy_mwh, float storage_energy_mwh)
{
    const char *str_meter = NULL;
    switch (meter)
    {
    case METER_SOLAR:
        str_meter = TOPIC_METER_SOLAR;
        break;
    case METER_BATTERY:
        str_meter = TOPIC_METER_BATTERY;
        break;
    default:
        logs("Unknown meter type: %d\n", meter);
        return;
    }

    char topic[128];
    char value[32];

    snprintf(topic, sizeof(topic), TOPIC_VOLTAGE, str_meter);
    snprintf(value, sizeof(value), "%.2f", voltage);
    mqtt_client.publish(topic, value);

    snprintf(topic, sizeof(topic), TOPIC_CURRENT, str_meter);
    snprintf(value, sizeof(value), "%.2f", current_ma);
    mqtt_client.publish(topic, value);

    snprintf(topic, sizeof(topic), TOPIC_POWER, str_meter);
    snprintf(value, sizeof(value), "%.2f", power_mw);
    mqtt_client.publish(topic, value);

    snprintf(topic, sizeof(topic), TOPIC_ENERGY, str_meter);
    snprintf(value, sizeof(value), "%.2f", energy_mwh);
    mqtt_client.publish(topic, value);

    snprintf(topic, sizeof(topic), TOPIC_CHARGE, str_meter);
    snprintf(value, sizeof(value), "%.2f", storage_energy_mwh);
    mqtt_client.publish(topic, value);
}

void mqtt_send_boot_count(uint32_t boot_count)
{
    mqtt_client.publish(TOPIC_BOOT_COUNT, String(boot_count).c_str());
}