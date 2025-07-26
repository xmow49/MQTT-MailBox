#define PIN_PARCEL 15
#define PIN_STATUS_PARCEL 26
#define PIN_LETTER 27
#define PIN_STATUS_LETTER 25

#define PIN_PIR 4

#define PIN_RAINBOW_DATA 5
#define PIN_MAX_POWER 32
#define RAINBOW_LED_COUNT 34

#define PIN_DHT 13
#define PIN_BUZZER 18

#define PIN_POWER_INA219 19

#define POWER_DEEPSLEEP_MW 13.2

#define DHTTYPE DHT22

#define BUTTON_BITMASK 0x8008000         // GPIOs 27 and 15 (2^27 + 2^15)
#define BUTTON_AND_PIR_BITMASK 0x8008010 // GPIOs 27, 15 and 4 (2^27 + 2^15 + 2^4)

#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"
#define OTA_PASSWORD "OTA_PASSWORD"

#define MQTT_SERVER "MQTT_SERVER_IP"
#define MQTT_USER "MQTT_USER"
#define MQTT_PASSWORD "MQTT_PASSWORD"

#define NTP_SERVER "NTP_SERVER_IP"
#define GMT_OFFSET_SEC 0
#define DAYLIGHT_OFFSET_SEC 3600

#define TOPIC_LETTER "mailbox/courrier"
#define TOPIC_PARCEL "mailbox/colis"
#define TOPIC_PIR "mailbox/pir"
#define TOPIC_BATTERY "mailbox/battery"
#define TOPIC_TEMP "mailbox/temperature"
#define TOPIC_HUM "mailbox/humidity"
#define TOPIC_WIFI "mailbox/wifi"
#define TOPIC_OTHER "mailbox/other"

#define TOPIC_CONFIG_DEEPSLEEP "mailbox/config/deepsleep"
#define TOPIC_CONFIG_AVOID_MULTIPLE_BOOT "mailbox/config/avoidMultipleBoot"
#define TOPIC_CONFIG_BRIGHTNESS "mailbox/config/brightness"
#define TOPIC_CONFIG_CHARGE "mailbox/config/charge"
#define TOPIC_CONFIG_PIR "mailbox/config/pir"
#define TOPIC_CONFIG_REBOOT "mailbox/config/reboot"

#define TOPIC_BOOT_COUNT "mailbox/bootCount"
#define TOPIC_LOOP_STATE "mailbox/loopState"
#define TOPIC_SET_STORAGE "mailbox/setStorage"
#define TOPIC_IP "mailbox/IP"

#define TOPIC_METER_SOLAR "solar"
#define TOPIC_METER_BATTERY "battery"

#define TOPIC_VOLTAGE "mailbox/%s/busvoltage"
#define TOPIC_CURRENT "mailbox/%s/current_mA"
#define TOPIC_POWER "mailbox/%s/power_mW"
#define TOPIC_ENERGY "mailbox/%s/energy_mWh"

#define TOPIC_LOGS "mailbox/eeprom_log"

#define LOOP_TIMEOUT_MIN 10

#define PERIODIC_SEND_MINUTES 15

#define WATCHDOG_SETUP_TIMEOUT_S 60

#define SENSOR_REFRESH_RATE_MS (100)

#define DEBUG false

struct Config
{
    char deepSleep = -1;
    char pir_sensor = -1;
    int brightness = 100;
    char avoidMultipleBoot = -1;
    float last_battery_energy_mWh = 0.0f;
    float last_solar_energy_mWh = 0.0f;
};

extern Config config;