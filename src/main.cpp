#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <WiFiSettings.h>
#include <MQTT.h>
#include <PMS.h>
#include <MHZ19.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <NeoPixelBus.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <list>
using namespace std;

#include "setting.h"

#define Sprintf(f, ...) ({ char* s; asprintf(&s, f, __VA_ARGS__); String r = s; free(s); r; })

#define ESPMAC (Sprintf("%06" PRIx64, ESP.getEfuseMac() >> 24)).c_str()
#define HOST_NAME ROOM_NAME "-" FIRMWARE_NAME

#define every(t) for (static uint32_t _lasttime; (uint32_t)((uint32_t) millis() - _lasttime) >= (t); _lasttime = millis())

unsigned long interval;
const int buttonpin  = 15;
const int i2c_sda = 23;
const int i2c_scl = 13;
const int ledpin = 26;
int brightness;
bool ota_enabled;

MQTTClient mqtt;
String topic_prefix;
bool add_units;

void retain(String topic, String message) {
    Serial.printf("%s %s\n", topic.c_str(), message.c_str());
    mqtt.publish(topic, message, true, 0);
}

#define FN function<void()>
#define LM function<void(SnuffelSensor&)>
struct SnuffelSensor {
    bool    enabled;
    String  id;
    String  description;
    String  topic_suffix;
    FN      settings;
    FN      init;
    FN      prepare;
    LM      fetch;

    void publish(list<pair<const char*, String>> mapping, String value, String unit) {
        // String topic = topic_prefix + topic_suffix;
        String topic = topic_prefix + topic_suffix;
        String name = topic_suffix;
        for (auto i : mapping) topic.replace(i.first, i.second);
        for (auto i : mapping) name.replace(i.first, i.second);
        retain(topic, name + ",room=study value=" + (add_units ? (value + " " + unit) : value));
    }
    void publish(String value, String unit) {
        publish({ }, value, unit);
    }
};

list<SnuffelSensor> snuffels;

void setup_sensors() {
    {
        static int gpio = 4;
        static OneWire ds(gpio);
        static DallasTemperature sensors(&ds);

        struct SnuffelSensor s = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "DS18B20",
            description: "temperature sensor(s)",
            topic_suffix: "temperature/{index}",
            settings: NULL,
            init: []() {
                sensors.begin();
                sensors.setWaitForConversion(false);
            },
            prepare: []() {
                sensors.requestTemperatures();
            },
            fetch: [](SnuffelSensor& self) {
                unsigned int timeout = millis() + 500;
                while (millis() < timeout && !sensors.isConversionComplete()) { }

                for (int i = 0; i < 100 /* arbitrary maximum */; i++) {
                    float C = sensors.getTempCByIndex(i);
                    if (C == DEVICE_DISCONNECTED_C) break;
                    if (C == 85) break;  // sensor returns 85.0 on errors
                    self.publish({ { "{index}", String(i) } }, String(C), "°C");
                }
            },
        };
        snuffels.push_back(s);
    }

    {
        static HardwareSerial hwserial(2);
        static MHZ19 mhz;
        static int rx = 22, tx = 21;
        static int alarm_level;

        struct SnuffelSensor s = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "MH-Z19",
            description: "CO2 sensor",
            topic_suffix: "co2",
            settings: []() {
                alarm_level = WiFiSettings.integer("MH-Z19_alarm", 0, 5000, 800, "CO2 warning level [PPM]");
            },
            init: []() {
                hwserial.begin(9600, SERIAL_8N1, rx, tx);
                mhz.begin(hwserial);
                mhz.setFilter(true, true); // Filter out erroneous readings (set to 0)
                mhz.autoCalibration();
            },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                int CO2 = mhz.getCO2();
                if (!CO2) return;

                self.publish(String(CO2), "PPM");
            }
        };
        snuffels.push_back(s);
    }

    {
        static HardwareSerial hwserial(1);
        static PMS pms(hwserial);
        static PMS::DATA data;
        static int rx = 25, tx = 32;

        struct SnuffelSensor s = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "PMS7003",
            description: "dust sensor",
            topic_suffix: "dust-PM{size}",
            settings: NULL,
            init: []() {
                hwserial.begin(9600, SERIAL_8N1, rx, tx);
                pms.passiveMode();
            },
            prepare: []() {
                pms.requestRead();
            },
            fetch: [](SnuffelSensor& self) {
                if (! pms.readUntil(data)) return;
                self.publish({ { "{size}",  "1.0" } }, String(data.PM_AE_UG_1_0),  "µg/m3");
                self.publish({ { "{size}",  "2.5" } }, String(data.PM_AE_UG_2_5),  "µg/m3");
                self.publish({ { "{size}", "10.0" } }, String(data.PM_AE_UG_10_0), "µg/m3");
            }
        };
        snuffels.push_back(s);
    }

    {
        static Adafruit_BME280 bme;  // repeated .begin()s seems to be fine
        static int i2c_address = 0x76;

        struct SnuffelSensor rh = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "BME280_RH",
            description: "relative humidity sensor",
            topic_suffix: "humidity",
            settings: NULL,
            init: []() { bme.begin(i2c_address); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                self.publish(String(bme.readHumidity()), "%");
            }
        };
        snuffels.push_back(rh);

        struct SnuffelSensor bp = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "BME280_BP",
            description: "barometric pressure sensor",
            topic_suffix: "pressure",
            settings: NULL,
            init: []() { bme.begin(i2c_address); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                self.publish(String(bme.readPressure() / 100.0F), "hPa");
            }
        };
        snuffels.push_back(bp);
    }
}

void setup_ota() {
    ArduinoOTA.setHostname(WiFiSettings.hostname.c_str());
    ArduinoOTA.setPassword(WiFiSettings.password.c_str());
    ArduinoOTA.begin();
}

void check_button() {
    if (digitalRead(buttonpin)) return;
    delay(50);
    if (digitalRead(buttonpin)) return;
    WiFiSettings.portal();
}

void setup() {
    Serial.begin(115200);
    SPIFFS.begin(true);
    Wire.begin(i2c_sda, i2c_scl);
    pinMode(buttonpin, INPUT);

    setup_sensors();

    ota_enabled   = WiFiSettings.checkbox("snuffelaar_ota", false, "Enable remote programming through ArduinoOTA. (Uses portal password!)");
    String server = WiFiSettings.string("mqtt_server", 64, "test.mosquitto.org", "MQTT broker");
    int port      = WiFiSettings.integer("mqtt_port", 0, 65535, 1883, "MQTT broker TCP port");
    topic_prefix  = WiFiSettings.string("snuffelaar_mqtt_prefix", "snuffelaar/", "MQTT topic prefix (ending with '/' strongly advised)");
    interval      = 1000UL * WiFiSettings.integer("snuffelaar_interval", 1, 3600, 5, "Publish interval [s]");
    add_units     = WiFiSettings.checkbox("snuffelaar_add_units", true, "Add units of measurement to MQTT messages");
    brightness    = WiFiSettings.integer("snuffelaar_brightness", 0, 255, 80, "Brightness for startup and idle LED");

    for (auto& s : snuffels) {
        String label = "Enable " + s.id + " " + s.description;
        s.enabled = WiFiSettings.checkbox(s.id + "_enabled", true, label);
        s.topic_suffix = WiFiSettings.string(s.id + "_topic", 1, 128, s.topic_suffix, s.id + " MQTT topic suffix");
        if (s.settings) s.settings();
    }

    if (ota_enabled) WiFiSettings.onPortal = setup_ota;

    WiFiSettings.onWaitLoop = []() {
        check_button();
        return 50;
    };
    WiFiSettings.onPortalWaitLoop = []() {
        if (ota_enabled) ArduinoOTA.handle();
    };
    if (!WiFiSettings.connect(false)) ESP.restart();

    for (auto& s : snuffels) if (s.enabled && s.init) s.init();

    static WiFiClient wificlient;
    Serial.print("Connecting MQTT");
    Serial.println(server);
    mqtt.begin(server.c_str(), port, wificlient);

    if (ota_enabled) setup_ota();
}

void loop() {
    unsigned long start = millis();

    if (ota_enabled) ArduinoOTA.handle();

    if (!mqtt.connected()) mqtt.connect("");  // ignore failures

    for (auto& s : snuffels) if (s.enabled && s.prepare) s.prepare();
    for (auto& s : snuffels) if (s.enabled && s.fetch)   s.fetch(s);

    while (millis() < start + interval) check_button();
}



