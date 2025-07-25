#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "logs.h"
#include "buzzer.h"

Adafruit_NeoPixel rainbow_led(RAINBOW_LED_COUNT, PIN_RAINBOW_DATA, NEO_GRB + NEO_KHZ800);
TaskHandle_t rainbow_task = NULL;

static void rainbow_animation_task(void *pvParameters);

void rainbow_init()
{
    pinMode(PIN_RAINBOW_DATA, INPUT);             // temporary, to avoid data pin as GND
    pinMode(PIN_RAINBOW_POWER, OUTPUT);           // LED Pin is OUTPUT
    digitalWrite(PIN_RAINBOW_POWER, LOW);         // turn off the power to the rainbow LED
    gpio_hold_dis((gpio_num_t)PIN_RAINBOW_POWER); // disable holding state of GPIO32
    gpio_deep_sleep_hold_dis();                   // disbale holding
}

void rainbow_start()
{
    if (rainbow_task != NULL)
    {
        return; // Rainbow animation already running
    }

    logs("Starting rainbow animation\n");

    xTaskCreatePinnedToCore(rainbow_animation_task, "rainbow", 4096, NULL, 10, &rainbow_task, 1);
    buzzer_play_melody();
    if (rainbow_task == NULL)
    {
        logs("Failed to create rainbow animation task\n");
        return;
    }
}

static void rainbow_animation_task(void *pvParameters)
{
    long firstPixelHue = 0;
    digitalWrite(PIN_RAINBOW_POWER, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));

    rainbow_led.begin();
    rainbow_led.setBrightness(config.brightness);

    uint32_t i = 0;
    for (;;)
    {
        rainbow_led.rainbow(firstPixelHue);
        if (i < rainbow_led.numPixels())
        {
            for (int j = i + 1; j < rainbow_led.numPixels(); j++)
            {
                rainbow_led.setPixelColor(j, 0, 0, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        rainbow_led.show(); // Update strip with new contents
        firstPixelHue += 256;
        i++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void rainbow_stop()
{
    if (rainbow_task == NULL)
    {
        return; // No task to stop
    }
    logs("Stopping rainbow animation\n");

    vTaskDelete(rainbow_task);
    rainbow_task = NULL;
    rainbow_led.setBrightness(0);
    rainbow_led.clear();
    rainbow_led.show();
    delay(100);
    digitalWrite(PIN_RAINBOW_POWER, LOW);
    digitalWrite(PIN_RAINBOW_DATA, LOW);
    pinMode(PIN_RAINBOW_DATA, INPUT); // disable data led to cut the power (use the data pin as GND)
}