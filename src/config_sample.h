#define PIN_BATTERY 33
#define ledPin 5
#define dhtPin 13
#define buzzer 18

#define DHTTYPE DHT22

#define BUTTON_PIN_BITMASK 0x8004 // GPIOs 2 and 15 (2^2 + 2^15)

#define wifi_ssid "WIFI"
#define wifi_password "PASSWORD"

#define mqtt_server "192.168.XX.XX"
#define mqtt_user "admin"
#define mqtt_password "password"

#define letter_topic "mailBox/letter"
#define parcel_topic "mailBox/parcel"
#define battery_topic "mailBox/battery"
#define temp_topic "mailBox/temperature"
#define hum_topic "mailBox/humidity"
#define wifi_topic "mailBox/wifi"