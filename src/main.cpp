#ifdef ARDUINO
#include <Arduino.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>
#include <LittleFS.h>
#include <WebSerial.h>
#include <WiFi.h>
#include <esp_timer.h>
#include <stdlib.h>

#include "config.h"

#ifdef UDP_STAT_HOST
#include <WiFiUdp.h>
#endif  // UDP_STAT_HOST

#include <Thermostat.h>

#define SENSOR_READ_PERIOD_IN_S 10

class ArduinoThermostatHardware : public Thermostat::Hardware {
 public:
  ArduinoThermostatHardware(uint8_t pin) : pin_(pin) {}
  void TurnOn() override { digitalWrite(pin_, HIGH); }
  void TurnOff() override { digitalWrite(pin_, LOW); }
  void Initialize() {
    pinMode(pin_, OUTPUT);
    TurnOff();  // Just in case it was latched on somehow.
  }

 private:
  uint8_t pin_;
};

class EspTimerTimestampProvider : public Thermostat::TimestampProvider {
 public:
  EspTimerTimestampProvider() { esp_timer_early_init(); }
  int64_t GetCurrentTimestamp() override { return esp_timer_get_time() / 1000; }
};

// Hardware/Services
DHT main_dht(MAIN_TEMP_PIN, DHT22);
float last_temp_read_in_c = NAN;
DHT heater_dht(HEATER_TEMP_PIN, DHT22);
float last_heater_temp_read_in_c = NAN;
int consecutive_ignored_reads = 0;
AsyncWebServer server(80);
#ifdef UDP_STAT_HOST
WiFiUDP Udp;
#endif  // UDP_STAT_HOST
ArduinoThermostatHardware furnace_hw(FURNACE_RELAY_PIN);
ArduinoThermostatHardware fan_hw(FAN_RELAY_PIN);
#ifdef LED_PIN
ArduinoThermostatHardware led_hw(LED_PIN);
#endif  // LED_PIN
#ifdef TEMP_POWER_PIN
ArduinoThermostatHardware temp_power_hw(TEMP_POWER_PIN);
#endif  // TEMP_POWER_PIN
EspTimerTimestampProvider timestamp_provider;
Thermostat thermostat(furnace_hw, fan_hw, timestamp_provider);

// State
long last_sample_timestamp = 0;
long last_sample_sent = 0;
long last_wifi_connect_attempt = 0;

void ConnectWiFi() {
  last_wifi_connect_attempt = millis();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print("Disconnected from WiFi access point. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Attempting to reconnect.");
  ConnectWiFi();
}

void HandleVarDump(AsyncWebServerRequest *request) {
  AsyncResponseStream *response =
      request->beginResponseStream("application/json");
  thermostat.DumpVarsTo(*response);
  request->send(response);
}

void HandleDebugDump(AsyncWebServerRequest *request) {
  AsyncResponseStream *response =
      request->beginResponseStream("application/json");
  JsonDocument json_doc;
  json_doc["build_time"] = BUILD_TIME;
  serializeJsonPretty(json_doc, *response);
  request->send(response);
}

void HandleSet(AsyncWebServerRequest *request) {
  if (request->hasParam("target_temp")) {
    thermostat.SetTempTarget(
        request->getParam("target_temp")->value().toFloat());
  }
  if (request->hasParam("hysteresis_offset")) {
    thermostat.SetHysteresisOffset(
        request->getParam("hysteresis_offset")->value().toFloat());
  }
  if (request->hasParam("opportunistic_heat_offset")) {
    thermostat.SetOpportunisticHeatOffset(
        request->getParam("opportunistic_heat_offset")->value().toFloat());
  }
  if (request->hasParam("mode")) {
    switch (request->getParam("mode")->value().toInt()) {
      case 1:
        thermostat.SetMode(Thermostat::ENGINE);
        break;
      case 2:
        thermostat.SetMode(Thermostat::FURNACE);
        break;
      default:
        thermostat.SetMode(Thermostat::OFF);
        break;
    }
  }

  // Write complete. Do a variable dump, requiring a read lock.
  HandleVarDump(request);
}

void HandleFurnaceOn(AsyncWebServerRequest *request) {
  thermostat.TurnOnFurnace();

  // Write complete. Do a variable dump, requiring a read lock.
  HandleVarDump(request);
}

void HandleSerialMessage(uint8_t *data, size_t len) {
  WebSerial.println("Received data. Ignoring.");
}

void setup() {
  Serial.begin(115200);
  // Wait for serial console to open.
  while (!Serial) {
    delay(10);
  }
  delay(2000);  // Wait for monitor to open.
  Serial.println("HyHeat initializing...");

  // Relays
#ifdef LED_PIN
  led_hw.Initialize();
  led_hw.TurnOn();
#endif  // LED_PIN
#ifdef TEMP_POWER_PIN
  Serial.print("Initializing temp sensor power...");
  temp_power_hw.Initialize();
  temp_power_hw.TurnOn();
  delay(1000);
  Serial.println(" Initialized.");
#endif  // TEMP_POWER_PIN
  furnace_hw.Initialize();
  fan_hw.Initialize();

  // Sensors
  main_dht.begin();
  heater_dht.begin();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);
  WiFi.onEvent(WiFiStationDisconnected,
               WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  ConnectWiFi();
  Serial.printf("Connecting to SSID '%s'...", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED &&
         millis() < WIFI_CONNECT_WAIT_IN_S * 1000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nConnected, IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.printf("\nUnable to connect after %ds. Continuing.\n",
                  WIFI_CONNECT_WAIT_IN_S);
  }

  // mDNS
  if (!MDNS.begin(HOSTNAME)) {
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.printf("mDNS responder started at %s.local\n", HOSTNAME);

  // SPI Flash Files System
  LittleFS.begin();

  // WebSerial
  WebSerial.begin(&server);
  WebSerial.onMessage(HandleSerialMessage);

  // OTA Updates
  ElegantOTA.begin(&server, OTA_USER, OTA_PASS);

  // Web Server
  server.on("/var_dump", HandleVarDump);
  server.on("/debug_dump", HandleDebugDump);
  server.on("/set", HandleSet);
  server.on("/furnace_on", HandleFurnaceOn);
  server.serveStatic("/", LittleFS, "/htdocs/")
      .setDefaultFile("thermostat.html");
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "404: Not found");
  });
  server.begin();

#ifdef LED_PIN
  led_hw.TurnOff();
#endif  // LED_PIN

  Serial.println("Initialized.");
}

void loop() {
  // Check if we're connected (or recently attempted reconnecting to) WiFi.
  if (WiFi.status() != WL_CONNECTED &&
      millis() - last_wifi_connect_attempt > WIFI_CONNECT_WAIT_IN_S * 1000) {
    Serial.println("WiFi still appears disconnected. Attempting to reconnect.");
    WiFi.disconnect();
    ConnectWiFi();
  }

  float temp_in_c = main_dht.readTemperature();
  float heater_temp_in_c = heater_dht.readTemperature();
  long sample_timestamp = millis();
  WebSerial.printf("Temp: %2.1fC Heater: %2.1f %ld ms\n", temp_in_c,
                   heater_temp_in_c, sample_timestamp - last_sample_timestamp);
  last_sample_timestamp = sample_timestamp;

  // Latch our heater/fan state so that we make sure to send stats on change.
  bool fan_on = thermostat.fan_on();
  bool furnace_on = thermostat.furnace_on();

  if (fabs(temp_in_c - last_temp_read_in_c) < TEMP_SENSOR_NOISE_IN_C &&
      fabs(heater_temp_in_c - last_heater_temp_read_in_c) <
          TEMP_SENSOR_NOISE_IN_C) {
    consecutive_ignored_reads = 0;
    thermostat.OnTempSamples(temp_in_c + MAIN_TEMP_OFFSET, heater_temp_in_c);
  } else {
    ++consecutive_ignored_reads;
    WebSerial.printf("Values over noise threshold (%fC), ignoring.\n",
                     TEMP_SENSOR_NOISE_IN_C);
#if defined(RESET_AFTER_IGNORED_READS) && defined(TEMP_POWER_PIN)
    if (consecutive_ignored_reads > RESET_AFTER_IGNORED_READS) {
      WebSerial.printf(
          "Too many consecutive ignored reads (%d), resetting sensors.\d",
          consecutive_ignored_reads);
      temp_power_hw.TurnOff();
      // Wait 5s to drain any capacitors.
      delay(5000);
      temp_power_hw.TurnOn();
      // Wait at least 1s before sending data to the sensor.
      if (SENSOR_READ_PERIOD_IN_S < 1) {
        delay(1000);
      }
    }
#endif  // RESET_AFTER_IGNORED_READS && TEMP_POWER_PIN
#ifdef UDP_STAT_HOST
    Udp.beginPacket(UDP_STAT_HOST, UDP_STAT_PORT);
    JsonDocument json_doc;
    json_doc["ignored_temp"] = temp_in_c;
    json_doc["last_temp"] = last_temp_read_in_c;
    json_doc["ignored_heater_temp"] = heater_temp_in_c;
    json_doc["last_heater_temp"] = last_heater_temp_read_in_c;
    serializeJsonPretty(json_doc, Udp);
    Udp.endPacket();
    WebSerial.println("Sent ignored values via UDP.");
#endif
  }

  last_temp_read_in_c = temp_in_c;
  last_heater_temp_read_in_c = heater_temp_in_c;

  // Fire UDP to influx every reporting period, or if fan/furance state changed.
#ifdef UDP_STAT_HOST
  if (last_sample_timestamp - last_sample_sent >
          STAT_REPORTING_PERIOD_IN_S * 1000 ||
      fan_on != thermostat.fan_on() || furnace_on != thermostat.furnace_on()) {
    Udp.beginPacket(UDP_STAT_HOST, UDP_STAT_PORT);
    thermostat.DumpVarsTo(Udp);
    Udp.endPacket();
    last_sample_sent = last_sample_timestamp;
    WebSerial.println("Sent sample via UDP.");
  }
#endif  // UDP_STAT_HOST

  // Handle updates.
  ElegantOTA.loop();

  // Sleep for a bit to save power and wasted effort.
  delay(SENSOR_READ_PERIOD_IN_S * 1000);
}
#else
int main(int arc, char** argv) {
  // Nothing to do for non-arduino environments.
}
#endif  // ARDUINO