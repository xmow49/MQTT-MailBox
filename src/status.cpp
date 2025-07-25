#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>

#include "status.h"
#include "config.h"

TaskHandle_t status_led_task_handle = NULL;
status_led_state_t current_status_led_state = STATUS_LED_COPY_GATES;
static void status_led_task(void *pvParameters)
{
    pinMode(PIN_PARCEL, INPUT);
    pinMode(PIN_LETTER, INPUT);
    pinMode(PIN_STATUS_LETTER, OUTPUT);
    pinMode(PIN_STATUS_PARCEL, OUTPUT);

    digitalWrite(PIN_STATUS_LETTER, LOW);
    digitalWrite(PIN_STATUS_PARCEL, LOW);

    for (;;)
    {
        switch (current_status_led_state)
        {
        case STATUS_LED_COPY_GATES:

            digitalWrite(PIN_STATUS_PARCEL, digitalRead(PIN_PARCEL));
            digitalWrite(PIN_STATUS_LETTER, digitalRead(PIN_LETTER));
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        case STATUS_LED_CONNECTING:
            digitalWrite(PIN_STATUS_PARCEL, !digitalRead(PIN_STATUS_PARCEL));
            digitalWrite(PIN_STATUS_LETTER, LOW);
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

void status_led_init()
{
    xTaskCreatePinnedToCore(status_led_task, "status_led_task", 2048, NULL, 8, &status_led_task_handle, 1);
    if (status_led_task_handle == NULL)
    {
        Serial.println("Failed to create status LED task");
    }
}

void status_led_set_state(status_led_state_t state)
{
    current_status_led_state = state;
}