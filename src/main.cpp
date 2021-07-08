#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "esp_sleep.h"

#include <config.h> //pins file

DHT dht(dhtPin, DHTTYPE); //dht temperature sensor

WiFiClient espClient;           //Wifi
PubSubClient client(espClient); //MQTT


void go_deepSleep()
{
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); //set pin to wake, here GPIO2 and 15. More info in config.h in "BUTTON_PIN_BITMASK"
  Serial.println("DeepSleep");
  delay(1000);
  gpio_hold_en(GPIO_NUM_5);  //hold the current state of pin 5 durring the deepsleep (LED)
  gpio_deep_sleep_hold_en(); //enable it
  esp_deep_sleep_start();    //Start the deepsleep
}

void ledBlink()
{
  //Blink the LED
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
  gpio_hold_dis(GPIO_NUM_5);  //disable holding state of GPIO5
  gpio_deep_sleep_hold_dis(); //disbale holding

  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);   //LED Pin is OUTPUT
  digitalWrite(ledPin, LOW); //LED to 0

  uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status(); //get the reason of wake
  int wake_GPIO = (log(GPIO_reason)) / log(2);

  if (wake_GPIO == 2 || wake_GPIO == 15)
  {             //if the GPIO2 or 15 wake the ESP32, There is a mail
    ledBlink(); //LED to thanks the postman
  }

  Serial.println("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password); //Connect to the Wifi

  int beforeConnect = millis(); //Get time before begin to connect
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (beforeConnect + 60000 < millis()) //Timeout of 1min
    {
      go_deepSleep(); //stop trying to connect and go deepsleep
    }
  }
  Serial.println("WiFi connected");

  client.setServer(mqtt_server, 1883);                         //Setup MQTT Server
  if (client.connect("ESP32Client", mqtt_user, mqtt_password)) //Connect to MQTT server
  {
    Serial.println("connected to MQTT Server");
  }
  else
  {
    Serial.print("error, rc=");
    Serial.print(client.state());
    go_deepSleep(); //error, go deepsleep
  }

  if (wake_GPIO == 2) //Letter GPIO, there is a letter
  {
    Serial.println("Letter");
    client.publish(letter_topic, "ON"); //Send ON to MQTT topic
    delay(200);
    client.publish(letter_topic, "OFF"); //Send OFF to MQTT topic
  }
  else if (wake_GPIO == 15) //Parcel GPIO, there is a parcel
  {
    Serial.println("Parcel");
    client.publish(parcel_topic, "ON"); //Send ON to MQTT topic
    delay(200);
    client.publish(parcel_topic, "OFF"); //Send OFF to MQTT topic
  }
  else //Wake with reset button, or every 10 min for the monitoring
  {
    Serial.println("No Notif");
  }

  dht.begin(); //Setup DHT

  Serial.println("Reading Temperatures...");
  int hum = dht.readHumidity();     //get temperature
  int temp = dht.readTemperature(); //get humidity

  //Display them:
  Serial.println("Temperature: " + String(temp) + "°C");
  Serial.println("Humidity: " + String(hum) + "%");

  //Send them:
  Serial.println("Sending values...");
  client.publish(temp_topic, String(temp).c_str());
  client.publish(hum_topic, String(hum).c_str());

  //Battery
  Serial.println("Getting battery level...");
  float battery = (analogRead(batteryPin) / (4095 / 3.3) / 0.785714); //Get Battery voltage: analogRead(batteryPin): 0->4095
                                                                      //                     analogRead(batteryPin) / (4095 / 3.3): 0V->3.3V of analog pin
                                                                      //0.785714 is the ratio of the voltage divider bridge with 27k and 100K: 0V->4.2V
  Serial.println("Battery level: " + String(battery) + "V");
  Serial.println("Sending values...");
  client.publish(battery_topic, String(battery).c_str()); //Send it

  Serial.println("Getting RSSI...");
  client.publish(wifi_topic, String(WiFi.RSSI()).c_str()); //Get RSSI (wifi strength)
  Serial.println(WiFi.RSSI());                             //Send it

  esp_sleep_enable_timer_wakeup(10 * 60 * 1000000); // Every 10 minutes, send temperature, battery ...
  go_deepSleep(); //return to deepsleep
}
void loop()
{
}