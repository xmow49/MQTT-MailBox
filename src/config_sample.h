#define batteryPin 33
#define ledPin 12
#define dhtPin 13

#define DHTTYPE DHT22

#define BUTTON_PIN_BITMASK 0x8004 // GPIOs 2 and 15 (2^2 + 2^15)

#define wifi_ssid "Wifi"
#define wifi_password "Password"

#define mqtt_server "192.168.2.50"
#define mqtt_user "user"
#define mqtt_password "password"

#define courrier_topic "boiteAuxLettres/courrier"
#define colis_topic "boiteAuxLettres/colis"
#define battery_topic "boiteAuxLettres/battery"
#define temp_topic "boiteAuxLettres/temperature"
#define hum_topic "boiteAuxLettres/humidity"
#define wifi_topic "boiteAuxLettres/wifi"