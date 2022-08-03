#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <Tone32.h>
#include <esp_sleep.h>
#include <EEPROM.h>
#include <ESP32Time.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include "time.h"
#include "config.h" //pins file

AsyncWebServer server(80);

DHT dht(dhtPin, DHTTYPE); // dht temperature sensor

WiFiClient espClient;           // Wifi
PubSubClient client(espClient); // MQTT

Adafruit_INA219 solarMeter(0x40);
Adafruit_INA219 batteryMeter(0x45);

Adafruit_NeoPixel ledMerci(34, dataLedPin, NEO_GRB + NEO_KHZ800);

TaskHandle_t statusLEDS;
TaskHandle_t animationTask;
TaskHandle_t MQTTTask;

ESP32Time rtc(3600); // offset in seconds GMT+1

const char *ntpServer = "192.168.2.50";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

struct Config
{
  char deepSleep = -1;
  int brightness = -1;
  char avoidMultipleBoot = -1;
};

RTC_DATA_ATTR unsigned long lastBoot;
RTC_DATA_ATTR unsigned long bootCount;
RTC_DATA_ATTR Config config;

RTC_DATA_ATTR float solar_energy_mWh = 0;
RTC_DATA_ATTR float battery_energy_mWh = 0;

bool animationState = false;

void turnOFFLedMerci()
{
  if (animationState)
  {
    vTaskDelete(animationTask);
    animationState = false;
  }

  ledMerci.setBrightness(0);
  ledMerci.clear();
  ledMerci.show();
  delay(100);
  digitalWrite(powerLedPin, LOW);
  digitalWrite(dataLedPin, LOW);
  pinMode(dataLedPin, INPUT); // disable data led to cut the power (use the data pin as GND)
}

void go_deepSleep()
{
  turnOFFLedMerci();
  delay(100);
  // vTaskDelete(statusLEDS);
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // set pin to wake, here GPIO2 and 15. More info in config.h in "BUTTON_PIN_BITMASK"
  Serial.println("DeepSleep");
  delay(500);
  gpio_hold_en(GPIO_NUM_32);                        // hold the current state of pin 32 durring the deepsleep (LED)
  gpio_deep_sleep_hold_en();                        // enable it
  esp_sleep_enable_timer_wakeup(10 * 60 * 1000000); // Every 10 minutes, send temperature, battery ...
  esp_deep_sleep_start();                           // Start the deepsleep
}

void melody()
{
  tone(buzzerPin, 523, 50, 0);
  delay(50);
  tone(buzzerPin, 783, 50, 0);
  delay(50);
  tone(buzzerPin, 1046, 50, 0);
  delay(50);
  tone(buzzerPin, 1568, 50, 0);
  delay(50);
  tone(buzzerPin, 2093, 70, 0);
}

void statusLEDSTask(void *pvParameters)
{
  while (1)
  {
    digitalWrite(parcelStatusPin, digitalRead(parcelPin));
    digitalWrite(letterStatusPin, digitalRead(letterPin));
    delay(100);
  }
}

void sendPowerMeter()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = batteryMeter.getShuntVoltage_mV();
  busvoltage = batteryMeter.getBusVoltage_V();
  current_mA = batteryMeter.getCurrent_mA();
  power_mW = batteryMeter.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  int time = rtc.getEpoch() - lastBoot;
  battery_energy_mWh = battery_energy_mWh + (power_mW) * (time / (60 * 60)); // 1s
  lastBoot = rtc.getEpoch();

  client.publish(battery_topic, String(busvoltage).c_str()); // Send it
  client.publish("boiteAuxLettres/battery/busvoltage", (String(busvoltage) + "V").c_str());
  // client.publish("boiteAuxLettres/battery/shuntvoltage", (String(shuntvoltage) + "mV").c_str());
  // client.publish("boiteAuxLettres/battery/loadvoltage", (String(loadvoltage) + "V").c_str());
  // client.publish("boiteAuxLettres/battery/current_mA", (String(current_mA) + "mA").c_str());
  client.publish("boiteAuxLettres/battery/power_mW", (String(power_mW) + "mW").c_str());
  client.publish("boiteAuxLettres/battery/energy_mWh", (String(battery_energy_mWh) + "mWh").c_str());

  shuntvoltage = solarMeter.getShuntVoltage_mV();
  busvoltage = solarMeter.getBusVoltage_V();
  current_mA = solarMeter.getCurrent_mA();
  power_mW = solarMeter.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  solar_energy_mWh = solar_energy_mWh + (power_mW) * (1 / (60 * 60)); // 1s

  client.publish("boiteAuxLettres/solar/busvoltage", (String(busvoltage) + "V").c_str());
  // client.publish("boiteAuxLettres/solar/shuntvoltage", (String(shuntvoltage) + "mV").c_str());
  // client.publish("boiteAuxLettres/solar/loadvoltage", (String(loadvoltage) + "V").c_str());
  client.publish("boiteAuxLettres/solar/current_mA", (String(current_mA) + "mA").c_str());
  client.publish("boiteAuxLettres/solar/power_mW", (String(power_mW) + "mW").c_str());
  client.publish("boiteAuxLettres/solar/energy_mWh", (String(solar_energy_mWh) + "mWh").c_str());

  /*float battery = (analogRead(batteryPin) / (4095 / 3.3) / 0.785714); // Get Battery voltage: analogRead(batteryPin): 0->4095
                                                                      //                      analogRead(batteryPin) / (4095 / 3.3): 0V->3.3V of analog pin
                                                                      // 0.785714 is the ratio of the voltage divider bridge with 27k and 100K: 0V->4.2V
  Serial.println("Battery level: " + String(battery) + "V");
  Serial.println("Sending values...");*/
}

void rainbow(void *pvParameters)
{
  digitalWrite(powerLedPin, HIGH);
  Serial.println("Rainbow");
  ledMerci.begin();
  ledMerci.setBrightness(120);
  int i = 0;
  unsigned long nextMillis = 0;
  while (1)
  {
    for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256, i++)
    {
      ledMerci.rainbow(firstPixelHue);
      if (i < ledMerci.numPixels())
      {
        for (int j = i + 1; j < ledMerci.numPixels(); j++)
        {
          ledMerci.setPixelColor(j, 0, 0, 0);
        }
        delay(50);
      }
      ledMerci.show(); // Update strip with new contents
      if (millis() > nextMillis)
      {
        nextMillis = millis() + 1000;
        sendPowerMeter();
      }
      delay(10);
    }
  }
}

void avoidMultipleBoot()
{
  if (rtc.getYear() != 1970)
  {    
    if (lastBoot + 60 > rtc.getEpoch())
    {
      Serial.println("restart in 1 minute");
      gpio_hold_en(GPIO_NUM_32);                        // hold the current state of pin 32 durring the deepsleep (LED)
      gpio_deep_sleep_hold_en();                        // enable it
      esp_sleep_enable_timer_wakeup(1 * 60 * 1000000); // Every 10 minutes, send temperature, battery ...
      esp_deep_sleep_start();
    }
    lastBoot = rtc.getEpoch();
  }
}

void checkMQTTMessage(void *pvParameters)
{
  while (1)
  {
    client.loop();
    delay(100);
  }
}

void publishConfig()
{
  client.publish(config_deepsleep_topic, String((int)config.deepSleep).c_str());
  client.publish(config_avoidMultipleBoot_topic, String((int)config.avoidMultipleBoot).c_str());
}

void newMQTTMessage(char *topic, byte *payload, unsigned int length)
{
  if (strcmp(topic, config_deepsleep_topic) == 0)
  {
    switch ((char)payload[0])
    {
    case '1':
      config.deepSleep = 1;
      break;
    case '0':
      config.deepSleep = 0;
      break;
    default:
      break;
    }
    // Serial.print("DeepSleep: ");
    // Serial.println(config.deepSleep, DEC);
  }
  if (strcmp(topic, config_avoidMultipleBoot_topic) == 0)
  {
    switch ((char)payload[0])
    {
    case '1':
      config.avoidMultipleBoot = 1;
      break;
    case '0':
      config.avoidMultipleBoot = 0;
      break;
    default:
      break;
    }
    // Serial.print("AvoidMultipleBoot: ");
    // Serial.println(config.avoidMultipleBoot, DEC);
  }
  EEPROM.put(0, config); // write config to EEPROM
  EEPROM.commit();
}

void sendGatesStates()
{
  client.publish(parcel_topic, (digitalRead(parcelPin) ? "1" : "0"));
  client.publish(letter_topic, (digitalRead(letterPin) ? "1" : "0"));
}

void sendTemperature()
{
  dht.begin(); // Setup DHT

  Serial.println("Reading Temperatures...");
  int hum = dht.readHumidity();       // get temperature
  float temp = dht.readTemperature(); // get humidity

  // Display them:
  Serial.println("Temperature: " + String(temp) + "°C");
  Serial.println("Humidity: " + String(hum) + "%");

  // Send them:
  Serial.println("Sending values...");
  client.publish(temp_topic, String(temp).c_str());
  client.publish(hum_topic, String(hum).c_str());
}

void sendWifiInfos()
{
  Serial.println("Getting RSSI...");
  client.publish(wifi_topic, String(WiFi.RSSI()).c_str()); // Get RSSI (wifi strength)
  Serial.println(WiFi.RSSI());                             // Send it
}

void startOTAServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! I am ESP32."); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}

void setup()
{
  String logs;
  bootCount++;

  Serial.begin(115200);
  if (config.avoidMultipleBoot == 1 || config.avoidMultipleBoot == -1) //
  {
    avoidMultipleBoot();
  }
  lastBoot = rtc.getEpoch();

  uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status(); // get the reason of wake
  int wake_GPIO = (log(GPIO_reason)) / log(2);

  gpio_hold_dis(GPIO_NUM_32); // disable holding state of GPIO32
  gpio_deep_sleep_hold_dis(); // disbale holding

  pinMode(parcelPin, INPUT_PULLUP);
  pinMode(letterPin, INPUT_PULLUP);
  pinMode(dataLedPin, INPUT); // temporary, to avoid data pin as GND

  pinMode(parcelStatusPin, OUTPUT);
  pinMode(letterStatusPin, OUTPUT);
  pinMode(powerLedPin, OUTPUT); // LED Pin is OUTPUT
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(parcelStatusPin, LOW);
  digitalWrite(letterStatusPin, LOW);
  digitalWrite(powerLedPin, LOW);

  ledcAttachPin(buzzerPin, 0);

  xTaskCreatePinnedToCore(
      statusLEDSTask, /* Task function. */
      "Task1",        /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &statusLEDS,    /* Task handle to keep track of created task */
      0);             /* pin task to core 0 */

  if (digitalRead(letterPin))
    wake_GPIO = letterPin;
  else if (digitalRead(parcelPin))
    wake_GPIO = parcelPin;

  if (!solarMeter.begin())
    logs += "Failed to initialize INA219 solarMeter\n";
  if (!batteryMeter.begin())
    logs += "Failed to initialize INA219 batteryMeter\n";
  solarMeter.setCalibration_32V_1A();
  batteryMeter.setCalibration_32V_1A();

  EEPROM.begin(512);
  EEPROM.get(0, config); // read config from EEPROM

  if (wake_GPIO == letterPin || wake_GPIO == parcelPin)
  {
    // if the GPIO2 or 15 wake the ESP32, There is a mail
    xTaskCreatePinnedToCore(
        rainbow,        /* Task function. */
        "Task2",        /* name of task. */
        10000,          /* Stack size of task */
        NULL,           /* parameter of the task */
        1,              /* priority of the task */
        &animationTask, /* Task handle to keep track of created task */
        1);
    animationState = true;
    melody(); // play sound
  }

  Serial.println("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password); // Connect to the Wifi

  int beforeConnect = millis(); // Get time before begin to connect
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (beforeConnect + 30000 < millis()) // Timeout of 30s
    {
      go_deepSleep(); // stop trying to connect and go deepsleep
    }
  }
  Serial.println("WiFi connected");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (rtc.getYear() == 1970)
  {
    logs += "RTC not set\n";
    Serial.println("Getting time...");
    if (getLocalTime(&timeinfo))
    {
      rtc.setTimeStruct(timeinfo);
      lastBoot = rtc.getEpoch();
    }
  }
  logs += "RTC set: " + rtc.getTime("%A, %B %d %Y %H:%M:%S") + "\n";
  Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));

  client.setServer(mqtt_server, 1883);                             // Setup MQTT Server
  if (client.connect("BoiteAuxLettres", mqtt_user, mqtt_password)) // Connect to MQTT server
  {
    client.setCallback(newMQTTMessage);
    client.subscribe(config_deepsleep_topic);
    client.subscribe(config_avoidMultipleBoot_topic);

    Serial.println("connected to MQTT Server");
    logs += "Boot OK \n";
    xTaskCreatePinnedToCore(
        checkMQTTMessage, /* Task function. */
        "Task3",          /* name of task. */
        10000,            /* Stack size of task */
        NULL,             /* parameter of the task */
        2,                /* priority of the task */
        &MQTTTask,        /* Task handle to keep track of created task */
        0);
  }
  else
  {
    Serial.print("MQTT ERROR: ");
    Serial.println(client.state());
    go_deepSleep(); // error, go deepsleep
  }

  if (wake_GPIO == letterPin) // Letter GPIO, there is a letter
  {
    Serial.println("Letter");
    client.publish(letter_topic, "1"); // Send ON to MQTT topic
    delay(200);
    client.publish(letter_topic, "0"); // Send OFF to MQTT topic
    logs += "Letter\n";
  }
  else if (wake_GPIO == parcelPin) // Parcel GPIO, there is a parcel
  {
    Serial.println("Parcel");
    client.publish(parcel_topic, "1"); // Send ON to MQTT topic
    delay(200);
    client.publish(parcel_topic, "0"); // Send OFF to MQTT topic
    logs += "Parcel\n";
  }
  else // Wake with reset button, or every 10 min for the monitoring
  {
    Serial.println("No Notif");
    logs += "No Notif\n";
    sendTemperature(); // send temperature to MQTT server
  }
  sendPowerMeter(); // send power meter to MQTT server
  sendWifiInfos();  // send wifi infos to MQTT server

  client.publish("boiteAuxLettres/log", logs.c_str());
  
  if (config.deepSleep == 1 || config.deepSleep == -1) // If deepsleep is enabled or config not loaded
  {
    while (millis() < 10000)
    {
      delay(10);
    }
    publishConfig();
    go_deepSleep(); // return to deepsleep
  }

  else
    startOTAServer(); // start OTA server
}

void loop()
{
  if (millis() > 15000 && animationState) // if 30s passed and animation is running, stop it
  {
    turnOFFLedMerci();
  }
  static unsigned long time = 0;
  client.loop(); // MQTT loop
  if (millis() > time)
  {
    sendPowerMeter();
    sendGatesStates();
    time = millis() + 1000;
    if (config.deepSleep == 1 || config.deepSleep == -1)
      go_deepSleep();
  }
}