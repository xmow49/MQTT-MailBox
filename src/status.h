#pragma once

typedef enum
{
    STATUS_LED_COPY_GATES,
    STATUS_LED_CONNECTING,
    STATUS_LED_CONNECTED,
    STATUS_LED_BROWNOUT,
} status_led_state_t;

void status_led_init();
void status_led_set_state(status_led_state_t state);
