#pragma once
#include <Arduino.h>

#define DEBUG true

void logs_init();

void logs(const char *format, ...);

char *logs_get_buffer(bool clear);

void logs_enable_telnet(bool enable);
