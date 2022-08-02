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
#include "time.h"

#include "config.h" //pins file

DHT dht(dhtPin, DHTTYPE); // dht temperature sensor

WiFiClient espClient;           // Wifi
PubSubClient client(espClient); // MQTT

Adafruit_INA219 solarMeter(0x40);
Adafruit_INA219 batteryMeter(0x45);

Adafruit_NeoPixel ledMerci(34, dataLedPin, NEO_GRB + NEO_KHZ800);

TaskHandle_t statusLEDS;
TaskHandle_t animationTask;

ESP32Time rtc(3600); // offset in seconds GMT+1

const char *ntpServer = "192.168.2.50";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

struct Config
{
  char deepSleep = -1;
  int brightness = -1;
};

RTC_DATA_ATTR unsigned long lastBoot;
RTC_DATA_ATTR unsigned long bootCount;
RTC_DATA_ATTR Config config;

bool animationState = false;

void go_deepSleep()
{
  if (animationState)
    vTaskDelete(animationTask);

  ledMerci.setBrightness(0);
  ledMerci.clear();
  ledMerci.show();
  delay(100);
  digitalWrite(powerLedPin, LOW);
  digitalWrite(dataLedPin, LOW);
  pinMode(dataLedPin, INPUT); // disable data led to cut the power (use the data pin as GND)
  delay(100);
  // vTaskDelete(statusLEDS);
  Serial.println("LED OFF");

  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); // set pin to wake, here GPIO2 and 15. More info in config.h in "BUTTON_PIN_BITMASK"
  Serial.println("DeepSleep");
  delay(1000);
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

void rainbow(void *pvParameters)
{
  digitalWrite(powerLedPin, HIGH);
  Serial.println("Rainbow");
  ledMerci.begin();
  static long time = 0;
  time = millis() + 1000;
  ledMerci.setBrightness(120);
  for (long firstPixelHue = 0, i = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256, i++)
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
    delay(10);
  }
}

void avoidMultipleBoot()
{
  if (rtc.getYear() != 1970)
  {
    if (lastBoot + 60 > rtc.getEpoch())
    {
      go_deepSleep();
    }
    lastBoot = rtc.getEpoch();
  }
}

void sendPowerMeter()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;
  static float energy_mWh = 0;

  shuntvoltage = batteryMeter.getShuntVoltage_mV();
  busvoltage = batteryMeter.getBusVoltage_V();
  current_mA = batteryMeter.getCurrent_mA();
  power_mW = batteryMeter.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  energy_mWh = energy_mWh + (power_mW) * (1 / (60 * 60)); // 1s

  client.publish("boiteAuxLettres/Batterybusvoltage", (String(busvoltage) + "V").c_str());
  client.publish("boiteAuxLettres/Batteryshuntvoltage", (String(shuntvoltage) + "mV").c_str());
  client.publish("boiteAuxLettres/Batteryloadvoltage", (String(loadvoltage) + "V").c_str());
  client.publish("boiteAuxLettres/Batterycurrent_mA", (String(current_mA) + "mA").c_str());
  client.publish("boiteAuxLettres/Batterypower_mW", (String(power_mW) + "mW").c_str());
  client.publish("boiteAuxLettres/Batteryenergy_mWh", (String(energy_mWh) + "mWh").c_str());

  shuntvoltage = solarMeter.getShuntVoltage_mV();
  busvoltage = solarMeter.getBusVoltage_V();
  current_mA = solarMeter.getCurrent_mA();
  power_mW = solarMeter.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  client.publish("boiteAuxLettres/Solarbusvoltage", (String(busvoltage) + "V").c_str());
  client.publish("boiteAuxLettres/Solarshuntvoltage", (String(shuntvoltage) + "mV").c_str());
  client.publish("boiteAuxLettres/Solarloadvoltage", (String(loadvoltage) + "V").c_str());
  client.publish("boiteAuxLettres/Solarcurrent_mA", (String(current_mA) + "mA").c_str());
  client.publish("boiteAuxLettres/Solarpower_mW", (String(power_mW) + "mW").c_str());
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
    Serial.print("DeepSleep: ");
    Serial.println(config.deepSleep, DEC);
  }
  EEPROM.put(0, config); // write config to EEPROM
  EEPROM.commit();
}

void setup()
{
  String logs;
  bootCount++;
  // avoidMultipleBoot();

  Serial.begin(115200);
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

  if (digitalRead(2))
    wake_GPIO = 2;
  else if (digitalRead(15))
    wake_GPIO = 15;

  if (!solarMeter.begin())
    logs += "Failed to initialize INA219 solarMeter\n";
  if (!batteryMeter.begin())
    logs += "Failed to initialize INA219 batteryMeter\n";
  solarMeter.setCalibration_32V_1A();
  batteryMeter.setCalibration_32V_1A();

  EEPROM.begin(512);
  EEPROM.get(0, config); // read config from EEPROM

  if (wake_GPIO == 2 || wake_GPIO == 15)
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
    Serial.println("connected to MQTT Server");
    logs += "Boot OK \n";
  }
  else
  {
    Serial.print("MQTT ERROR: ");
    Serial.println(client.state());
    go_deepSleep(); // error, go deepsleep
  }

  if (wake_GPIO == 2) // Letter GPIO, there is a letter
  {
    Serial.println("Letter");
    client.publish(letter_topic, "ON"); // Send ON to MQTT topic
    delay(200);
    client.publish(letter_topic, "OFF"); // Send OFF to MQTT topic
    logs += "Letter\n";
  }
  else if (wake_GPIO == 15) // Parcel GPIO, there is a parcel
  {
    Serial.println("Parcel");
    client.publish(parcel_topic, "ON"); // Send ON to MQTT topic
    delay(200);
    client.publish(parcel_topic, "OFF"); // Send OFF to MQTT topic
    logs += "Parcel\n";
  }
  else // Wake with reset button, or every 10 min for the monitoring
  {
    Serial.println("No Notif");
    logs += "No Notif\n";
  }

  dht.begin(); // Setup DHT

  Serial.println("Reading Temperatures...");
  int hum = dht.readHumidity();     // get temperature
  int temp = dht.readTemperature(); // get humidity

  // Display them:
  Serial.println("Temperature: " + String(temp) + "°C");
  Serial.println("Humidity: " + String(hum) + "%");

  // Send them:
  Serial.println("Sending values...");
  client.publish(temp_topic, String(temp).c_str());
  client.publish(hum_topic, String(hum).c_str());

  // Battery
  Serial.println("Getting battery level...");
  float battery = (analogRead(batteryPin) / (4095 / 3.3) / 0.785714); // Get Battery voltage: analogRead(batteryPin): 0->4095
                                                                      //                      analogRead(batteryPin) / (4095 / 3.3): 0V->3.3V of analog pin
                                                                      // 0.785714 is the ratio of the voltage divider bridge with 27k and 100K: 0V->4.2V
  Serial.println("Battery level: " + String(battery) + "V");
  Serial.println("Sending values...");
  client.publish(battery_topic, String(battery).c_str()); // Send it

  Serial.println("Getting RSSI...");
  client.publish(wifi_topic, String(WiFi.RSSI()).c_str()); // Get RSSI (wifi strength)
  Serial.println(WiFi.RSSI());                             // Send it

  client.publish("boiteAuxLettres/log", logs.c_str());

  client.loop();                                       // Loop to check MQTT messages
  if (config.deepSleep == 1 || config.deepSleep == -1) // If deepsleep is enabled or config not loaded
    go_deepSleep();                                    // return to deepsleep
}

void loop()
{
  static unsigned long time = 0;
  client.loop();
  if (millis() > time)
  {
    sendPowerMeter();
    time = millis() + 1000;
  }
}