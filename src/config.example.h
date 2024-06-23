// Connectivity.
#define HOSTNAME "hyheat"
#define WIFI_SSID "..."
#define WIFI_PASS "..."
#define WIFI_CONNECT_WAIT_IN_S 10

// Hardware pins.
#define MAIN_TEMP_PIN GPIO_NUM_18
#define HEATER_TEMP_PIN GPIO_NUM_19
#define TEMP_POWER_PIN GPIO_NUM_21
#define FURNACE_RELAY_PIN GPIO_NUM_16
#define FAN_RELAY_PIN GPIO_NUM_17
#define LED_PIN \
  GPIO_NUM_23  // Optional. LED on while initializing. Leave undefined to
               // disable.

// Temperature sensor offset. Useful if your temperature sensor is mounted in a
// location that is consistently warmer or cooler than the rest of the
// environment.
#define MAIN_TEMP_OFFSET 0.0
// Temperature sensor noise threshold in C. If a temperature reading differs
// from its previous value by this amount or more, do not apply thermostat
// logic. This can help with temperature sensors that occasionally report bogus
// values.
#define TEMP_SENSOR_NOISE_IN_C 5.0
// Reset temp sensors after this many bogus temp readings.
#define RESET_AFTER_IGNORED_READS 10  

// Over-The-Air firmware updates are password-protected.
#define OTA_USER "..."
#define OTA_PASS "..."  // Note these credentials are sent unencrypted!

// Stats pushed over UDP in JSON. Optional. Comment out (leave undefined) to
// disable. Stats reported once per fan/furance state change, or per reporting
// period, whichever is first.
#define UDP_STAT_HOST "192.168.8.1"
#define UDP_STAT_PORT 8094
#define STAT_REPORTING_PERIOD_IN_S 10