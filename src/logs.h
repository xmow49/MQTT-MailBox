#pragma once
#include <Arduino.h>

void logs_init();

void logs(const char *format, ...);

char *logs_get_buffer(bool clear);

void logs_enable_telnet(bool enable);
