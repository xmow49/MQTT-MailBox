#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#include <config.h>

DHT dht(dhtPin, DHTTYPE);


WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

void reconnect()
{
  while (!client.connected())
  {
    if (client.connect("ESP32Client", mqtt_user, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("error, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void go_deepSleep()
{
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
  Serial.println("DeepSleep");
  delay(1000);
  esp_deep_sleep_start();
}

void ledBlink()
{
  pinMode(ledPin, OUTPUT);
  for (int i = 0; i <= 2; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}

void setup()
{
  Serial.begin(115200);

  uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  int wake_GPIO = (log(GPIO_reason)) / log(2);

  if(wake_GPIO == 2 || wake_GPIO == 15){
    ledBlink();
  }

  Serial.println("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  int beforeConnect = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (beforeConnect + 60000 < millis())
    {
      go_deepSleep();
    }
  }
  Serial.println("WiFi connected");

  client.setServer(mqtt_server, 1883);
  reconnect();
  if (wake_GPIO == 2) //Courrier
  {
    Serial.println("Courrier");
    client.publish(courrier_topic, "ON");
    delay(200);
    client.publish(courrier_topic, "OFF");
  }
  else if (wake_GPIO == 15) //Colis
  {
    Serial.println("Colis");
    client.publish(colis_topic, "ON");
    delay(200);
    client.publish(colis_topic, "OFF");
  }
  else
  {
    Serial.println("No Notif");
  }

  dht.begin();

  Serial.println("Reading Temperatures...");
  int hum = dht.readHumidity();
  int temp = dht.readTemperature();
  Serial.println("Temperature: " + String(temp) + "°C");
  Serial.println("Humidity: " + String(hum) + "%");

  Serial.println("Sending values...");
  client.publish(temp_topic, String(temp).c_str());
  client.publish(hum_topic, String(hum).c_str());

  Serial.println("Getting battery level...");
  float battery = map(analogRead(batteryPin), 0.0f, 4095.0f, 0.0f, 3.3f);
  Serial.println("Battery level: " + String(battery) + "V");
  Serial.println("Sending values...");
  client.publish(battery_topic, String(battery).c_str());

  Serial.println("Getting RSSI...");
  client.publish(wifi_topic, String(WiFi.RSSI()).c_str());
  Serial.println(WiFi.RSSI());

  go_deepSleep();
}
void loop() {}