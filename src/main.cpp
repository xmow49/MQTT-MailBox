#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>

#include <esp_sleep.h>
#include <EEPROM.h>
#include <ESP32Time.h>
#include <TelnetStream.h>

#include "status.h"
#include "rainbow.h"
#include "logs.h"
#include "sensor.h"
#include "buzzer.h"
#include "ota.h"
#include "mqtt.h"

#include "time.h"
#include "config.h" //pins file

ESP32Time rtc(GMT_OFFSET_SEC); // offset in seconds GMT+1

RTC_DATA_ATTR uint32_t last_boot_time;
RTC_DATA_ATTR uint32_t boot_count;
RTC_DATA_ATTR Config config;
RTC_DATA_ATTR gate_t gate_event_from_loop = GATE_UNKNOWN;

bool already_open = false;
static unsigned long loop_time_ms = 0;
static unsigned long end_boot_time_ms = 0;

void go_deep_sleep()
{
  mqtt_send_loop_state(false);
  rainbow_stop();
  digitalWrite(PIN_POWER_INA219, LOW);
  if (config.pir_sensor == 1)
  {
    esp_sleep_enable_ext1_wakeup(BUTTON_AND_PIR_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // set pin to wake, here GPIO27 and 15. More info in config.h in "BUTTON_AND_PIR_BITMASK"
  }
  else
  {
    esp_sleep_enable_ext1_wakeup(BUTTON_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // set pin to wake, here GPIO27 and 15. More info in config.h in "BUTTON_BITMASK"
  }

  logs("Going to deepsleep...\n");
  delay(500);
  gpio_hold_en(GPIO_NUM_32); // hold the current state of pin 32 durring the deepsleep (LED)
  gpio_deep_sleep_hold_en(); // enable it

  esp_sleep_enable_timer_wakeup(10 * 60 * 1000000); // Every 10 minutes, send temperature, battery ...
  esp_deep_sleep_start();                           // Start the deepsleep
}

void avoid_multiple_boot()
{
  if (rtc.getYear() == 1970)
  {
    return; // RTC not initialized, we cannot avoid multiple boot
  }

  if (last_boot_time + 60 > rtc.getEpoch())
  {
    logs("Restart in 1 minute\n");
    gpio_hold_en(GPIO_NUM_32); // hold the current state of pin 32 durring the deepsleep (LED)
    gpio_deep_sleep_hold_en(); // enable it
    digitalWrite(PIN_POWER_INA219, LOW);
    esp_sleep_enable_timer_wakeup(1 * 60 * 1000000); // Every 10 minutes, send temperature, battery ...
    esp_deep_sleep_start();
  }
  last_boot_time = rtc.getEpoch();
}

void setup()
{
  logs_init();
  rainbow_init();
  sensor_init();
  buzzer_init();
  status_led_init();
  EEPROM.begin(512);
  EEPROM.get(0, config); // read config from EEPROM

  boot_count++;

  if (config.avoidMultipleBoot == 1 || config.avoidMultipleBoot == -1)
  {
    avoid_multiple_boot();
  }

  char wake_GPIO = log(esp_sleep_get_ext1_wakeup_status()) / log(2);
  if (wake_GPIO >= 32)
  {
    logs("Wake GPIO is not valid: %d\n", wake_GPIO);
    wake_GPIO = 0;
  }

  if (wake_GPIO == PIN_LETTER || wake_GPIO == PIN_PARCEL)
  {
    rainbow_start();
  }

  logs("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Connect to the Wifi
  int wifi_timeout = millis() + 20000;  // Get time before begin to connect
  while (WiFi.status() != WL_CONNECTED)
  {
    logs(".");
    // digitalWrite(PIN_STATUS_PARCEL, !digitalRead(PIN_STATUS_PARCEL));
    if (millis() > wifi_timeout) // Timeout of 30s
    {
      go_deep_sleep(); // stop trying to connect and go deepsleep
    }
    delay(500);
  }

  IPAddress ip = WiFi.localIP();
  logs("\nWiFi connected: %s\n", ip.toString().c_str());
  TelnetStream.begin();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  struct tm timeinfo;
  if (rtc.getYear() == 1970)
  {
    logs("RTC not set, getting time from NTP server...\n");
    logs("Getting time...\n");
    if (getLocalTime(&timeinfo))
    {
      rtc.setTimeStruct(timeinfo);
      last_boot_time = rtc.getEpoch();
    }
  }
  else
  {
    last_boot_time = rtc.getEpoch();
  }

  logs("RTC set: %s\n", rtc.getTime("%A, %B %d %Y %H:%M:%S").c_str());

  int mqtt_state = mqtt_init();
  if (mqtt_state == 0)
  {
    logs("Connected to MQTT Server\n");
    logs("Boot OK \n");
  }
  else
  {
    logs("MQTT ERROR: %d bye\n", mqtt_state);
    go_deep_sleep(); // error, go deepsleep
  }

  mqtt_send_boot_count(boot_count);

  gate_t gate = GATE_UNKNOWN;
  logs("Wake GPIO: %d\n", wake_GPIO);
  switch (wake_GPIO)
  {
  case PIN_LETTER:
    gate = GATE_LETTER;
    break;
  case PIN_PARCEL:
    gate = GATE_PARCEL;
    break;
  case PIN_PIR:
    gate = GATE_PIR;
    break;
  default:
    if (gate_event_from_loop < GATE_UNKNOWN)
    {
      logs("Wake event from loop: %d\n", gate_event_from_loop);
      gate = gate_event_from_loop;
    }
    break;
  }

  logs("Gate: %d\n", gate);

  if (gate != GATE_UNKNOWN)
  {
    mqtt_sent_gate(gate, false); // Send OFF to MQTT topic
    delay(200);
    mqtt_sent_gate(gate, true); // Send ON to MQTT topic
    delay(200);
    mqtt_sent_gate(gate, false); // Send OFF to MQTT topic
  }
  else // Wake with reset button, or every 10 min for the monitoring
  {
    logs("No Notif\n");
  }

  sensor_send_values();   // send sensor values to MQTT server
  mqtt_send_wifi_infos(); // send wifi infos to MQTT server
  mqtt_publish_config();
  delay(3000); // wait a bit before going to deepsleep

  if (config.deepSleep == 1 || config.deepSleep == -1) // If deepsleep is enabled or config not loaded
  {
    go_deep_sleep(); // return to deepsleep
    return;          // never reached
  }

  rainbow_stop();
  ota_start_server();
  mqtt_send_loop_state(true);

  if (digitalRead(PIN_LETTER) || digitalRead(PIN_PARCEL))
  {
    already_open = true;
  }

  end_boot_time_ms = millis();
}

void loop()
{
  bool parcel = digitalRead(PIN_PARCEL);
  bool letter = digitalRead(PIN_LETTER);
  if ((letter || parcel) && !already_open) // if any detection, restart to process
  {
    gate_event_from_loop = (letter ? GATE_LETTER : GATE_PARCEL);
    logs("Detected %s, restarting...\n", (letter ? "letter" : "parcel"));
    ESP.restart();
  }

  sensor_send_gates_states(); // send gates states to MQTT server

  if (millis() > loop_time_ms) // every 10s
  {
    logs("Loop\n");
    sensor_send_values();
    mqtt_send_wifi_infos();
    loop_time_ms = millis() + 10000;

    if (config.deepSleep == 1 || config.deepSleep == -1) // if deepsleep is enabled
    {
      go_deep_sleep(); // return to deepsleep
    }

    if (millis() - end_boot_time_ms > LOOP_TIMEOUT_MIN * 60 * 1000)
    {
      logs("Loop timeout, restarting...\n");
      ESP.restart(); // Restart if loop is too long
    }
  }

  if (TelnetStream.available()) // if there is data on TelnetStream
  {
    char c = TelnetStream.read();
    logs_enable_telnet(true);
    switch (c)
    {
    case '\n':
      TelnetStream.println("BoiteAuxLettres");
      TelnetStream.println("L: Show startup logs");
      TelnetStream.println("R: Start rainbow");
      TelnetStream.println("S: Stop rainbow");
      break;
    case 'L':
      TelnetStream.println(logs_get_buffer(false));
      break;
    case 'R':
      rainbow_start();
      break;
    case 'S':
      rainbow_stop();
      break;
    default:
      break;
    }
  }
}