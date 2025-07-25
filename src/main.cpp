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
#include "esp_wifi.h"

#include "time.h"
#include "config.h" //pins file

ESP32Time rtc(GMT_OFFSET_SEC); // offset in seconds GMT+1

RTC_DATA_ATTR uint32_t last_boot_time;
RTC_DATA_ATTR uint32_t boot_count;
RTC_DATA_ATTR Config config;
RTC_DATA_ATTR gate_t gate_event_from_loop = GATE_UNKNOWN;
RTC_DATA_ATTR bool last_gate_open = false;

bool already_open = false;
static unsigned long loop_time_ms = 0;
static unsigned long end_boot_time_ms = 0;

TaskHandle_t watchdog_task_handle = NULL;

void set_max_power(bool state)
{
  if (state)
  {
    pinMode(PIN_MAX_POWER, OUTPUT); // LED Pin is OUTPUT
    digitalWrite(PIN_MAX_POWER, HIGH);
    gpio_hold_dis((gpio_num_t)PIN_MAX_POWER); // disable holding state of GPIO32
  }
  else
  {
    pinMode(PIN_MAX_POWER, OUTPUT); // LED Pin is OUTPUT
    digitalWrite(PIN_MAX_POWER, LOW);
    gpio_hold_en((gpio_num_t)PIN_MAX_POWER); // hold the current state of pin 32 durring the deepsleep (LED)
  }
}

void go_deep_sleep()
{
  logs("Going to deepsleep...\n");
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

  last_gate_open = sensor_is_any_gate_open();

  delay(500);
  set_max_power(false);
  gpio_deep_sleep_hold_en();

  esp_sleep_enable_timer_wakeup(PERIODIC_SEND_MINUTES * 60 * 1000000); // Every 10 minutes, send temperature, battery ...
  esp_deep_sleep_start();                                              // Start the deepsleep
}

gate_t get_boot_gate()
{
  char wake_GPIO = log(esp_sleep_get_ext1_wakeup_status()) / log(2);
  if (wake_GPIO >= 32)
  {
    logs("Wake GPIO is not valid: %d\n", wake_GPIO);
    wake_GPIO = 0;
  }

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
      gate_event_from_loop = GATE_UNKNOWN; // reset the event from loop
    }
    break;
  }

  if (gate == GATE_UNKNOWN)
  {
    bool letter = digitalRead(PIN_LETTER);
    bool parcel = digitalRead(PIN_PARCEL);
    bool pir = digitalRead(PIN_PIR);
    logs("No gate detected, using pins read: letter=%d, parcel=%d, pir=%d\n", letter, parcel, pir);
    if (parcel)
    {
      gate = GATE_PARCEL;
    }
    else if (letter)
    {
      gate = GATE_LETTER;
    }
    else if (pir)
    {
      gate = GATE_PIR;
    }
  }

  return gate;
}

void avoid_multiple_boot()
{
  if (last_gate_open && sensor_is_any_gate_open())
  {
    logs("Avoiding multiple boot: a gate is open, going to deepsleep...\n");
    set_max_power(false);
    gpio_deep_sleep_hold_en();
    digitalWrite(PIN_POWER_INA219, LOW);
    esp_sleep_enable_timer_wakeup(1 * 60 * 1000000);
    esp_deep_sleep_start();
  }
  else
  {
    logs("Avoiding multiple boot: no gate is open, continuing...\n");
  }

  if ((rtc.getYear() != 1970 && (last_boot_time + 60 > rtc.getEpoch())))
  {
    logs("Avoiding multiple boot by rtc, going to deepsleep...\n");
    set_max_power(false);
    gpio_deep_sleep_hold_en();
    digitalWrite(PIN_POWER_INA219, LOW);
    esp_sleep_enable_timer_wakeup(1 * 60 * 1000000);
    esp_deep_sleep_start();
  }

  last_boot_time = rtc.getEpoch();
}

void watchdog_task(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(WATCHDOG_SETUP_TIMEOUT_S * 1000));
  logs("Watchdog task triggered, going to deepsleep...\n");
  go_deep_sleep();   // if the watchdog is triggered, go to deepsleep
  vTaskDelete(NULL); // delete the task
}

void setup()
{
  set_max_power(true);
  logs_init();
  if (config.avoidMultipleBoot == 1 || config.avoidMultipleBoot == -1)
  {
    avoid_multiple_boot();
  }

  xTaskCreatePinnedToCore(watchdog_task, "watchdog", 4096, NULL, 23, &watchdog_task_handle, 1);

  rainbow_init();
  sensor_init();
  buzzer_init();
  status_led_init();
  EEPROM.begin(512);
  EEPROM.get(0, config); // read config from EEPROM

  boot_count++;

  if (esp_reset_reason() == ESP_RST_BROWNOUT)
  {
    logs("Brownout detected, setting status LED to BROWNOUT\n");
    status_led_set_state(STATUS_LED_BROWNOUT);
    vTaskDelay(pdMS_TO_TICKS(3000)); // wait a bit to show the brownout state
  }
  else
  {
    vTaskDelay(pdMS_TO_TICKS(500)); // wait a bit before starting the rainbow animation
  }

  gate_t gate = get_boot_gate();
  logs("Gate: %d %d\n", gate, gate_event_from_loop);

  logs("Connecting to %s", WIFI_SSID);
  esp_wifi_set_max_tx_power(32);        // Set max TX power to 46 dBm (default is 20 dBm --> 86)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Connect to the Wifi
  int wifi_timeout = millis() + 20000;  // Get time before begin to connect
  status_led_set_state(STATUS_LED_CONNECTING);

  if (gate <= GATE_PARCEL)
  {
    rainbow_start();
  }

  while (WiFi.status() != WL_CONNECTED)
  {
    logs(".");
    if (millis() > wifi_timeout) // Timeout of 30s
    {
      go_deep_sleep(); // stop trying to connect and go deepsleep
    }
    delay(500);
  }
  status_led_set_state(STATUS_LED_COPY_GATES);

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

  if (gate != GATE_UNKNOWN)
  {
    logs("Sending gate state to MQTT: %d\n", gate);
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
  // mqtt_publish_config();

  delay(5000); // wait a bit before going to deepsleep

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

  vTaskSuspend(watchdog_task_handle); // suspend the watchdog task, we don't need it anymore
}

void loop()
{
  bool parcel = digitalRead(PIN_PARCEL);
  bool letter = digitalRead(PIN_LETTER);
  if ((letter || parcel) && !already_open) // if any detection, restart to process
  {
    gate_event_from_loop = (letter ? GATE_LETTER : GATE_PARCEL);
    logs("Detected %s, restarting...\n", (letter ? "letter" : "parcel"));
    // ESP.restart();
    esp_sleep_enable_timer_wakeup(100000); // 100ms restart to process the detection
    esp_deep_sleep_start();
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
      TelnetStream.println("=====================");
      TelnetStream.println("BoiteAuxLettres");
      TelnetStream.println("L: Show startup logs");
      TelnetStream.println("R: Start rainbow");
      TelnetStream.println("S: Stop rainbow");
      TelnetStream.println("O: Restart ESP");
      TelnetStream.println("=====================");
      TelnetStream.println();

      break;
    case 'L':
      TelnetStream.println("-----------------------------------------------");
      TelnetStream.println(logs_get_buffer(false));
      TelnetStream.println("-----------------------------------------------");
      break;
    case 'R':
      rainbow_start();
      break;
    case 'S':
      rainbow_stop();
      break;
    case 'O':
      ESP.restart();
      break;
    default:
      break;
    }
  }
}