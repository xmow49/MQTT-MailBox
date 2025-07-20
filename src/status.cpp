#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>

#include "status.h"
#include "config.h"

TaskHandle_t status_led_task_handle = NULL;

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
        digitalWrite(PIN_STATUS_PARCEL, digitalRead(PIN_PARCEL));
        digitalWrite(PIN_STATUS_LETTER, digitalRead(PIN_LETTER));
        vTaskDelay(pdMS_TO_TICKS(100));
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