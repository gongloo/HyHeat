#include <ArduinoJson.h>
#include <Thermostat.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::Return;
using ::testing::ReturnPointee;

class MockThermostatHardware : public Thermostat::Hardware {
 public:
  MOCK_METHOD(void, TurnOn, (), (override));
  MOCK_METHOD(void, TurnOff, (), (override));
};

class MockTimestampProvider : public Thermostat::TimestampProvider {
 public:
  MOCK_METHOD(int64_t, GetCurrentTimestamp, (), (override));
};

class UninitializedThermostatTest : public ::testing::Test {
 protected:
  UninitializedThermostatTest()
      : t_(furnace_hw_, fan_hw_, timestamp_provider_){};
  StaticJsonDocument<1024> GetVars() {
    StaticJsonDocument<1024> vars;
    std::string dump;
    t_.DumpVarsTo(dump);
    deserializeJson(vars, dump);
    return vars;
  }

  ::testing::StrictMock<MockThermostatHardware> furnace_hw_;
  ::testing::StrictMock<MockThermostatHardware> fan_hw_;
  MockTimestampProvider timestamp_provider_;
  Thermostat t_;
};

class ThermostatTest : public UninitializedThermostatTest {
 protected:
  void SetUp() override {
    // Set current temps right at temp target to start.
    t_.OnTempSamples(19, 19);
  }
};

TEST_F(UninitializedThermostatTest, DumpValues) {
  auto vars = GetVars();
  EXPECT_TRUE(vars["temp"].isNull());
  EXPECT_TRUE(vars["heater_temp"].isNull());
  EXPECT_EQ(0, vars["mode"]);
  EXPECT_EQ(19, vars["target_temp"]);
  EXPECT_EQ(1.5, vars["hysteresis_offset"]);
  EXPECT_EQ(0.5, vars["min_hysteresis_offset"]);
  EXPECT_EQ(5.0, vars["opportunistic_heat_offset"]);
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(0, vars["fan_on_ms"]);
  EXPECT_EQ(900000, vars["min_furnace_runtime_ms"]);
  EXPECT_EQ(0, vars["uptime_ms"]);
}

TEST_F(UninitializedThermostatTest, NanTempsCauseShutdown) {
  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);
  t_.OnTempSamples(NAN, NAN);
}

TEST_F(ThermostatTest, SavesTemps) {
  // Use default set point, so no thermostat logic employed.
  t_.OnTempSamples(19.01, 19.02);

  auto vars = GetVars();
  EXPECT_FLOAT_EQ(19.01, vars["temp"]);
  EXPECT_FLOAT_EQ(19.02, vars["heater_temp"]);
}

TEST_F(ThermostatTest, NanTempsDoNotSave) {
  t_.OnTempSamples(NAN, 10);
  auto vars = GetVars();
  EXPECT_EQ(19, vars["temp"]);
  EXPECT_EQ(10, vars["heater_temp"]);

  t_.OnTempSamples(10, NAN);
  vars = GetVars();
  EXPECT_EQ(10, vars["temp"]);
  EXPECT_EQ(10, vars["heater_temp"]);
}

TEST_F(ThermostatTest, BasicFurnaceOnOff) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(furnace_hw_, TurnOn()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 10);  // Furnace should turn on.

  cur_timestamp += Thermostat::kMinFurnaceRuntimeInS * 1000 + 1;
  auto vars = GetVars();
  EXPECT_EQ(cur_timestamp, vars["furnace_on_ms"]);

  ++cur_timestamp;
  // Nothing happens since we're in the hysteresis range.
  t_.OnTempSamples(20, 50);
  vars = GetVars();
  EXPECT_EQ(cur_timestamp, vars["furnace_on_ms"]);

  EXPECT_CALL(furnace_hw_, TurnOff()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  ++cur_timestamp;
  t_.OnTempSamples(30, 30);  // Furnace should turn off.
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
}

TEST_F(ThermostatTest, OffModeTurnsOffFan) {
  t_.SetMode(Thermostat::ENGINE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 55);  // Fan should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(1, vars["fan_on_ms"]);

  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  t_.SetMode(Thermostat::OFF);
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
}

TEST_F(ThermostatTest, OffModeTurnsOffFurnace) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(furnace_hw_, TurnOn()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 10);  // Furnace should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(1, vars["furnace_on_ms"]);

  EXPECT_CALL(furnace_hw_, TurnOff()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  cur_timestamp += Thermostat::kMinFurnaceRuntimeInS * 1000 + 1;
  t_.SetMode(Thermostat::OFF);
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
}

TEST_F(ThermostatTest, SetTempTargetImmediateEffect) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(furnace_hw_, TurnOn()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.SetTempTarget(30);  // Furnace should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(1, vars["furnace_on_ms"]);
}

TEST_F(ThermostatTest, SetHysteresisTakesImmediateEffect) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(furnace_hw_, TurnOn()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  // Nothing should happen since we're within hysteresis.
  t_.OnTempSamples(18, 18);
  // Furnace should turn on.
  t_.SetHysteresisOffset(Thermostat::kMinHysteresisOffsetInC);

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(1, vars["furnace_on_ms"]);
}

TEST_F(ThermostatTest, SetOpportunisticHeatOffsetTakesImmediateEffect) {
  t_.SetMode(Thermostat::ENGINE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 55);  // Fan should turn on.
  // Fan should remain on since we're within opportunistic heat offset.
  t_.OnTempSamples(23.5, 50);
  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(1, vars["fan_on_ms"]);

  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);
  t_.SetOpportunisticHeatOffset(1);  // Fan should turn off.

  ++cur_timestamp;
  vars = GetVars();
  EXPECT_EQ(0, vars["fan_on_ms"]);
}

TEST_F(ThermostatTest, SetTempTargetBounds) {
  t_.SetTempTarget(0);
  auto vars = GetVars();
  EXPECT_EQ(19, vars["target_temp"]);

  t_.SetTempTarget(100);
  vars = GetVars();
  EXPECT_EQ(19, vars["target_temp"]);
}

TEST_F(ThermostatTest, SetHysteresisBounds) {
  t_.SetHysteresisOffset(0);
  auto vars = GetVars();
  EXPECT_EQ(1.5, vars["hysteresis_offset"]);

  t_.SetHysteresisOffset(20);
  vars = GetVars();
  EXPECT_EQ(1.5, vars["hysteresis_offset"]);
}

TEST_F(ThermostatTest, SetOpportunisticHeatOffsetBounds) {
  t_.SetOpportunisticHeatOffset(-0.001);
  auto vars = GetVars();
  EXPECT_EQ(5, vars["opportunistic_heat_offset"]);

  t_.SetOpportunisticHeatOffset(20);
  vars = GetVars();
  EXPECT_EQ(5, vars["opportunistic_heat_offset"]);
}

TEST_F(ThermostatTest, EngineModeTurnsOffFurnace) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(furnace_hw_, TurnOn()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 10);  // Furnace should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(1, vars["furnace_on_ms"]);

  EXPECT_CALL(furnace_hw_, TurnOff()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  cur_timestamp += Thermostat::kMinFurnaceRuntimeInS * 1000 + 1;
  t_.SetMode(Thermostat::ENGINE);
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
}

TEST_F(ThermostatTest, TurnsOffFurnaceWhenSafe) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(furnace_hw_, TurnOn()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 10);  // Furnace should turn on.
  ++cur_timestamp;  // Not enough time has passed to turn off the furnace.
  // Furnace should not turn off yet even though it's warm enough.
  t_.OnTempSamples(30, 30);

  auto vars = GetVars();
  EXPECT_EQ(1, vars["furnace_on_ms"]);

  EXPECT_CALL(furnace_hw_, TurnOff()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  cur_timestamp += Thermostat::kMinFurnaceRuntimeInS * 1000;
  t_.OnTempSamples(30, 30);  // Furnace should turn off.
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
}

TEST_F(ThermostatTest, BasicFanOnOff) {
  t_.SetMode(Thermostat::ENGINE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 55);  // Fan should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(cur_timestamp, vars["fan_on_ms"]);

  ++cur_timestamp;
  // Nothing happens since we're in the hysteresis range.
  t_.OnTempSamples(23.9, 50);
  vars = GetVars();
  EXPECT_EQ(cur_timestamp, vars["fan_on_ms"]);

  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  ++cur_timestamp;
  t_.OnTempSamples(30, 50);  // Fan should turn off.
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(0, vars["fan_on_ms"]);
}

TEST_F(ThermostatTest, FanTurnsOffWhenCold) {
  t_.SetMode(Thermostat::ENGINE);
  t_.SetTempTarget(38);
  t_.SetHysteresisOffset(0.5);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));

  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 55);  // Fan should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(1, vars["fan_on_ms"]);

  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  // Fan should turn off as we don't meet the minimum residual heat.
  t_.OnTempSamples(10, Thermostat::kMinResidualHeatInC - Thermostat::kMinHysteresisOffsetInC);
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(0, vars["fan_on_ms"]);
}

TEST_F(ThermostatTest, FurnaceModePrefersFan) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(fan_hw_, TurnOn()).Times(1);

  t_.OnTempSamples(10, 55);  // Fan should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(cur_timestamp, vars["fan_on_ms"]);

  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  ++cur_timestamp;
  t_.OnTempSamples(30, 30);  // Fan should turn off.
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(0, vars["fan_on_ms"]);
}

TEST_F(ThermostatTest, FurnaceModeFanThenFurnace) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(fan_hw_, TurnOn()).Times(2);

  t_.OnTempSamples(10, 55);  // Fan should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(cur_timestamp, vars["fan_on_ms"]);

  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);
  EXPECT_CALL(furnace_hw_, TurnOn()).Times(1);

  ++cur_timestamp;
  t_.OnTempSamples(10, 10);  // Furnace should turn on.
  ++cur_timestamp;
  vars = GetVars();
  EXPECT_EQ(1, vars["furnace_on_ms"]);
  EXPECT_EQ(0, vars["fan_on_ms"]);
}

TEST_F(ThermostatTest, FurnaceModeFurnaceThenFan) {
  t_.SetMode(Thermostat::FURNACE);

  int64_t cur_timestamp = 0;
  EXPECT_CALL(timestamp_provider_, GetCurrentTimestamp())
      .WillRepeatedly(ReturnPointee(&cur_timestamp));
  EXPECT_CALL(furnace_hw_, TurnOn()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOn()).Times(2);

  t_.OnTempSamples(10, 10);  // Furnace should turn on.

  ++cur_timestamp;
  auto vars = GetVars();
  EXPECT_EQ(cur_timestamp, vars["furnace_on_ms"]);
  EXPECT_EQ(0, vars["fan_on_ms"]);

  EXPECT_CALL(furnace_hw_, TurnOff()).Times(1);
  EXPECT_CALL(fan_hw_, TurnOff()).Times(1);

  cur_timestamp += Thermostat::kMinFurnaceRuntimeInS * 1000 + 1;
  t_.OnTempSamples(23, 30);  // Furnace should turn off.
  ++cur_timestamp;
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(0, vars["fan_on_ms"]);

  ++cur_timestamp;
  t_.OnTempSamples(10, 55);  // Fan should turn on.
  ++cur_timestamp;
  vars = GetVars();
  EXPECT_EQ(0, vars["furnace_on_ms"]);
  EXPECT_EQ(1, vars["fan_on_ms"]);
}

#if defined(ARDUINO)
#include <Arduino.h>

void setup() {
  // should be the same value as for the `test_speed` option in "platformio.ini"
  // default value is test_speed=115200
  Serial.begin(115200);

  ::testing::InitGoogleTest();
  // if you plan to use GMock, replace the line above with
  // ::testing::InitGoogleMock();
}

void loop() {
  // Run tests
  if (RUN_ALL_TESTS())
    ;

  // sleep for 1 sec
  delay(1000);
}

#else
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // if you plan to use GMock, replace the line above with
  // ::testing::InitGoogleMock(&argc, argv);

  if (RUN_ALL_TESTS())
    ;

  // Always return zero-code and allow PlatformIO to parse results
  return 0;
}
#endif