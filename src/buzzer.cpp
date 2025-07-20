#include <Arduino.h>

#include "buzzer.h"
#include "config.h"
#include "logs.h"
#include <Tone32.h>

TaskHandle_t buzzer_task = NULL;
static void buzzer_melody_task(void *pvParameters);

void buzzer_init()
{
    pinMode(PIN_BUZZER, OUTPUT);
    ledcAttachPin(PIN_BUZZER, 0);
}

void buzzer_play_melody()
{
    if (buzzer_task != NULL)
    {
        return; // Melody already playing
    }

    logs("Starting buzzer melody\n");

    xTaskCreatePinnedToCore(buzzer_melody_task, "buzzer", 4096, NULL, 10, &buzzer_task, 1);
    if (buzzer_task == NULL)
    {
        logs("Failed to create buzzer task\n");
        return;
    }
}

static void buzzer_melody_task(void *pvParameters)
{
    tone(PIN_BUZZER, 523, 50, 0);
    tone(PIN_BUZZER, 783, 50, 0);
    tone(PIN_BUZZER, 1046, 50, 0);
    tone(PIN_BUZZER, 1568, 50, 0);
    tone(PIN_BUZZER, 2093, 70, 0);

    logs("Buzzer melody finished\n");
    vTaskDelete(NULL);
}