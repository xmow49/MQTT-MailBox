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
#include <TelnetStream.h>

#include <ArduinoOTA.h>

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
  char pir_sensor = -1;
  int brightness = 100;
  char avoidMultipleBoot = -1;
};

RTC_DATA_ATTR unsigned long lastBoot;
RTC_DATA_ATTR unsigned long bootCount;
RTC_DATA_ATTR Config config;

RTC_DATA_ATTR float solar_energy_mWh = 0;
RTC_DATA_ATTR float battery_energy_mWh = 0;
RTC_DATA_ATTR float storage_energy_mWh = 0;

bool animationState = false;
bool alreadyOpen = false;

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
  client.publish(loopState_topic, "0");
  turnOFFLedMerci();
  digitalWrite(powerINA219, LOW);
  // vTaskDelete(statusLEDS);
  if (config.pir_sensor == 1)
  {
    esp_sleep_enable_ext1_wakeup(BUTTON_AND_PIR_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // set pin to wake, here GPIO27 and 15. More info in config.h in "BUTTON_AND_PIR_BITMASK"
  }
  else
  {
    esp_sleep_enable_ext1_wakeup(BUTTON_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // set pin to wake, here GPIO27 and 15. More info in config.h in "BUTTON_BITMASK"
  }
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
  float busvoltage = 0;
  float current_mA = 0;
  float power_mW = 0;
  float newConsommation = 0;

  busvoltage = batteryMeter.getBusVoltage_V();
  current_mA = batteryMeter.getCurrent_mA();
  power_mW = batteryMeter.getPower_mW();

  int time = rtc.getEpoch() - lastBoot;
  lastBoot = rtc.getEpoch();

  if (busvoltage < 6.0) // security
  {
    newConsommation = (power_mW) * ((float)time / (60 * 60));
    battery_energy_mWh = battery_energy_mWh + newConsommation;
    storage_energy_mWh -= newConsommation;
    client.publish(battery_voltage_topic, (String(busvoltage)).c_str());
    client.publish(battery_current_topic, (String(current_mA)).c_str());
    client.publish(battery_power_topic, (String(power_mW)).c_str());
    client.publish(battery_energy_topic, (String(battery_energy_mWh)).c_str());
  }

  // shuntvoltage = solarMeter.getShuntVoltage_mV();
  busvoltage = solarMeter.getBusVoltage_V();
  current_mA = solarMeter.getCurrent_mA();
  power_mW = solarMeter.getPower_mW();

  if (busvoltage < 6.0)
  {
    newConsommation = (power_mW) * ((float)time / (60 * 60));
    solar_energy_mWh = solar_energy_mWh + newConsommation;
    storage_energy_mWh += newConsommation;
    client.publish(solar_voltage_topic, (String(busvoltage)).c_str());
    client.publish(solar_current_topic, (String(current_mA)).c_str());
    client.publish(solar_power_topic, (String(power_mW)).c_str());
    client.publish(solar_energy_topic, (String(solar_energy_mWh)).c_str());

    client.publish(battery_charge_topic, (String(storage_energy_mWh)).c_str());
  }

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
  ledMerci.setBrightness(config.brightness);
  int i = 0;
  unsigned long nextMillis = 0;
  while (1)
  {
    for (long firstPixelHue = 0; firstPixelHue < 5 * 65536 && animationState && millis() < 15000; firstPixelHue += 256, i++)
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
        if (rtc.getYear() != 1970)
          sendPowerMeter();
      }
      delay(10);
    }
    if (millis() >= 15000 && animationState) // if 30s passed and animation is running, stop it
    {
      turnOFFLedMerci();
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
      gpio_hold_en(GPIO_NUM_32);                       // hold the current state of pin 32 durring the deepsleep (LED)
      gpio_deep_sleep_hold_en();                       // enable it
      digitalWrite(powerINA219, LOW);
      esp_sleep_enable_timer_wakeup(1 * 60 * 1000000); // Every 10 minutes, send temperature, battery ...
      esp_deep_sleep_start();
    }
    lastBoot = rtc.getEpoch();
  }
}

void checkMQTTMessage()
{
  client.loop();
  yield();
}

void publishConfig()
{
  client.publish(config_deepsleep_topic, String((int)config.deepSleep).c_str());
  client.publish(config_avoidMultipleBoot_topic, String((int)config.avoidMultipleBoot).c_str());
}

void newMQTTMessage(char *topic, byte *payload, unsigned int length)
{
  // Serial.println(topic);
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
  if (strcmp(topic, config_brightness_topic) == 0)
  {
    if (length > 3)
      return;
    if (length == 2)
      payload[2] = '\0';
    int value = atoi((char *)payload);
    if (value < 0 || value > 255)
      return;
    config.brightness = value;
    Serial.print("Brightness: ");
    Serial.println(config.brightness, DEC);
  }
  if (strcmp(topic, config_charge_topic) == 0)
  {
    if (length > 5)
      return;
    if (length == 2)
      for (int i = 2; i < 5; i++)
        payload[i] = '\0';
    int value = atoi((char *)payload);
    storage_energy_mWh = (float)value;
    Serial.print("Charge: ");
    Serial.println(storage_energy_mWh, DEC);
  }
  if (strcmp(topic, config_pir_topic) == 0)
  {
    switch ((char)payload[0])
    {
    case '1':
      config.pir_sensor = 1;
      break;
    case '0':
      config.pir_sensor = 0;
      break;
    default:
      break;
    }
    Serial.print("PIR: ");
    Serial.println(config.pir_sensor, DEC);
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
  if (!isnan(temp) && !isnan(hum))
  {
    client.publish(temp_topic, String(temp).c_str());
    client.publish(hum_topic, String(hum).c_str());
  }
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

  ArduinoOTA.setHostname("BoiteAuxLettres"); // Set hostname for OTA
  ArduinoOTA.setPassword(OTA_password);      // Set password for OTA
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

  Serial.println("HTTP server started");
}

void setup()
{
  pinMode(parcelPin, INPUT);
  pinMode(letterPin, INPUT);
  pinMode(dataLedPin, INPUT); // temporary, to avoid data pin as GND
  pinMode(pirPin, INPUT);     // temporary, to avoid data pin as GND

  pinMode(parcelStatusPin, OUTPUT);
  digitalWrite(parcelStatusPin, LOW);

  pinMode(letterStatusPin, OUTPUT);
  digitalWrite(letterStatusPin, LOW);

  pinMode(powerLedPin, OUTPUT); // LED Pin is OUTPUT
  digitalWrite(powerLedPin, LOW);

  pinMode(buzzerPin, OUTPUT);

  pinMode(powerINA219, OUTPUT); // Power INA219
  digitalWrite(powerINA219, HIGH);

  String logs;
  bootCount++;

  Serial.begin(115200);
  if (config.avoidMultipleBoot == 1 || config.avoidMultipleBoot == -1) //
  {
    avoidMultipleBoot();
  }
  lastBoot = rtc.getEpoch();
  // uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status(); // get the reason of wake

  // gpio_hold_dis(GPIO_NUM_32); // disable holding state of GPIO32
  // gpio_deep_sleep_hold_dis(); // disbale holding

  ledcAttachPin(buzzerPin, 0);
  // tskIDLE_PRIORITY
  //  xTaskCreatePinnedToCore(
  //      statusLEDSTask,   /* Task function. */
  //      "StatusLED",          /* name of task. */
  //      10000,            /* Stack size of task */
  //      NULL,             /* parameter of the task */
  //      32, /* priority of the task */
  //      &statusLEDS,      /* Task handle to keep track of created task */
  //      0);               /* pin task to core 0 */

  auto WakeUP = log(esp_sleep_get_ext1_wakeup_status()) / log(2);
  char wake_GPIO = 0;
  if (WakeUP < 32)
  {
    wake_GPIO = (char)WakeUP;
  }

  // if (digitalRead(letterPin))
  //   wake_GPIO = letterPin;
  // else if (digitalRead(parcelPin))
  //   wake_GPIO = parcelPin;

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
        "Rainbow",      /* name of task. */
        10000,          /* Stack size of task */
        NULL,           /* parameter of the task */
        1,              /* priority of the task */
        &animationTask, /* Task handle to keep track of created task */
        0);
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
    if (beforeConnect + 20000 < millis()) // Timeout of 30s
    {
      go_deepSleep(); // stop trying to connect and go deepsleep
    }
  }
  TelnetStream.begin();
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
    client.subscribe(config_brightness_topic);
    client.subscribe(config_charge_topic);
    client.subscribe(config_pir_topic);

    Serial.println("connected to MQTT Server");
    logs += "Boot OK \n";
    // xTaskCreatePinnedToCore(
    //     checkMQTTMessage, /* Task function. */
    //     "MQTTLoop",       /* name of task. */
    //     10000,            /* Stack size of task */
    //     NULL,             /* parameter of the task */
    //     0,                /* priority of the task */
    //     &MQTTTask,        /* Task handle to keep track of created task */
    //     0);
    client.publish(bootCount_topic, String(bootCount).c_str());
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
    client.publish(letter_topic, "0");
    delay(200);
    client.publish(letter_topic, "1"); // Send ON to MQTT topic
    delay(200);
    client.publish(letter_topic, "0"); // Send OFF to MQTT topic
    logs += "Letter\n";
  }
  else if (wake_GPIO == parcelPin) // Parcel GPIO, there is a parcel
  {
    Serial.println("Parcel");
    client.publish(parcel_topic, "0"); // Send ON to MQTT topic
    delay(200);
    client.publish(parcel_topic, "1"); // Send ON to MQTT topic
    delay(200);
    client.publish(parcel_topic, "0"); // Send OFF to MQTT topic
    logs += "Parcel\n";
  }
  else if (wake_GPIO == pirPin) // Parcel GPIO, there is a parcel
  {
    Serial.println("PIR");
    client.publish(pir_topic, "0"); // Send ON to MQTT topic
    delay(200);
    client.publish(pir_topic, "1"); // Send ON to MQTT topic
    delay(200);
    client.publish(pir_topic, "0"); // Send OFF to MQTT topic
    logs += "PIR\n";
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

  if (animationState || wake_GPIO == pirPin)
  {
    while (millis() < 15000) // whait end of animation
    {
      checkMQTTMessage();
      delay(50);
    }
  }

  if (config.deepSleep == 1 || config.deepSleep == -1) // If deepsleep is enabled or config not loaded
  {
    publishConfig();
    go_deepSleep(); // return to deepsleep
  }

  else
  {
    turnOFFLedMerci();
    startOTAServer(); // start OTA server
    client.publish(loopState_topic, "1");
    if (digitalRead(letterPin) || digitalRead(parcelPin))
    {
      alreadyOpen = true;
    }
  }
  yield();
}

void loop()
{
  static unsigned long time = 0;
  digitalWrite(parcelStatusPin, digitalRead(parcelPin)); // Update parcel status
  digitalWrite(letterStatusPin, digitalRead(letterPin)); // Update letter status
  client.loop();                                         // MQTT loop
  ArduinoOTA.handle();                                   // OTA loop

  if ((digitalRead(letterPin) || digitalRead(parcelPin)) && !alreadyOpen) // if any detection, restart to process
  {
    ESP.restart();
  }
  if (millis() > time) // every 10s
  {
    sendGatesStates();                                   // send gates states to MQTT server
    sendPowerMeter();                                    // send power meter to MQTT server
    sendTemperature();                                   // send temperature to MQTT server
    time = millis() + 10000;                             // next time
    if (config.deepSleep == 1 || config.deepSleep == -1) // if deepsleep is enabled
      go_deepSleep();                                    // return to deepsleep
  }
}