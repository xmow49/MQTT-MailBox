#include "logs.h"
#include <stdarg.h>
#include <TelnetStream.h>

static char logs_buffer[8 * 1024];
static uint32_t logs_index = 0;

static bool telnet_enabled = false;

void logs_init()
{
#if DEBUG
    Serial.begin(115200);
#endif
}

void logs(const char *format, ...)
{
    char buffer[512];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

#if DEBUG
    Serial.print(buffer);
#endif

    if (telnet_enabled && TelnetStream.available())
    {
        TelnetStream.print(buffer);
    }

    if (logs_index + strlen(buffer) < sizeof(logs_buffer) - 1)
    {
        strcpy(&logs_buffer[logs_index], buffer);
        logs_index += strlen(buffer);
    }
    else
    {
#if DEBUG
        Serial.println("Log buffer overflow, resetting logs");
#endif
        logs_index = 0;
        strcpy(logs_buffer, buffer);
    }
}

char *logs_get_buffer(bool clear)
{
    logs_buffer[logs_index] = '\0';
    if (clear)
    {
        logs_index = 0; // Reset index after getting the buffer
    }
    return logs_buffer;
}

void logs_enable_telnet(bool enable)
{
    telnet_enabled = enable;
}