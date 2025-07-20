
#include <ArduinoOTA.h>

#include "ota.h"
#include "logs.h"
#include "config.h"

TaskHandle_t ota_task_handle = NULL;

void ota_task(void *pvParameters);

void ota_start_server()
{
    ArduinoOTA.setHostname("BoiteAuxLettres"); // Set hostname for OTA
    ArduinoOTA.setPassword(OTA_PASSWORD);      // Set password for OTA
    ArduinoOTA
        .onStart([]()
                 {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
        .onEnd([]()
               { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
    ArduinoOTA.begin();
    xTaskCreatePinnedToCore(ota_task, "OTA", 4096, NULL, 8, &ota_task_handle, 1);
}

void ota_task(void *pvParameters)
{
    logs("OTA server started\n");

    while (true)
    {
        ArduinoOTA.handle();            // Handle OTA requests
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay to avoid blocking
    }
}