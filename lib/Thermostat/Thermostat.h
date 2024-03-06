#ifndef __THERMOSTAT_H__
#define __THERMOSTAT_H__

#include <ArduinoJson.h>
#include <math.h>

#include <mutex>
#include <shared_mutex>

class ThermostatTest;

class Thermostat {
 public:
  class Hardware {
   public:
    virtual void TurnOn() = 0;
    virtual void TurnOff() = 0;
  };

  class TimestampProvider {
   public:
    virtual int64_t GetCurrentTimestamp() = 0;
  };

  enum Mode {
    OFF,     // Don't manage anything, turn everything off when safe.
    ENGINE,  // Use only (free) engine heat.
    FURNACE  // Use furnace heat as necessary, but prefer engine heat.
  };

  // Hardcoded config
  // Recommended minimum runtime is 15 minutes.
  static constexpr unsigned int kMinFurnaceRuntimeInS = 15 * 60;
  // Absolute value.
  static constexpr float kMinResidualHeatInC = 50;
  // +/- minimum hysteresis offset.

  // Bounds
  static constexpr float kMinTempTargetInC = 5;
  static constexpr float kMaxTempTargetInC = 38;
  static constexpr float kMinHysteresisOffsetInC = 0.5;
  static constexpr float kMaxHysteresisOffsetInC = 10;
  static constexpr float kMaxOpportunisticHeatOffsetInC =
      kMaxHysteresisOffsetInC;

  Thermostat(Hardware& furnace_hw, Hardware& fan_hw,
             TimestampProvider& timestamp_provider);

  // Apply thermostat logic given new temperature readings.
  void OnTempSamples(float main_temp_in_c, float heater_temp_in_c);

  // Setters. Applies logic immediately.
  void SetTempTarget(float temp_target_in_c);
  void SetHysteresisOffset(float hysteresis_offset_in_c);
  void SetOpportunisticHeatOffset(float opportunistic_heat_offset_in_c);
  void SetMode(Mode mode);

  // Manual forced controls.
  void TurnOnFurnace();

  template <typename Writer>
  void DumpVarsTo(Writer& w) {
    read_lock l(m_);
    DynamicJsonDocument json_doc(1024);
    auto now_in_ms = timestamp_provider_.GetCurrentTimestamp();
    json_doc["temp"] = last_temp_in_c_;
    json_doc["heater_temp"] = last_heater_temp_in_c_;
    json_doc["mode"] = static_cast<int>(mode_);
    json_doc["target_temp"] = temp_target_in_c_;
    json_doc["hysteresis_offset"] = hysteresis_offset_in_c_;
    json_doc["min_hysteresis_offset"] = kMinHysteresisOffsetInC;
    json_doc["opportunistic_heat_offset"] = opportunistic_heat_offset_in_c_;
    json_doc["furnace_on_ms"] =
        furnace_on_ ? now_in_ms - furnace_on_timestamp_ : 0;
    json_doc["fan_on_ms"] = fan_on_ ? now_in_ms - fan_on_timestamp_ : 0;
    json_doc["min_furnace_runtime_ms"] = kMinFurnaceRuntimeInS * 1000;
    json_doc["uptime_ms"] = now_in_ms;
    serializeJsonPretty(json_doc, w);
  }

  bool furnace_on() { return furnace_on_; }
  bool fan_on() { return fan_on_; }

 private:
  typedef std::lock_guard<std::shared_mutex> write_lock;
  typedef std::shared_lock<std::shared_mutex> read_lock;

  void ApplyLogic(const write_lock&);
  void TurnOnFurnace(const write_lock&);
  void TurnOffFurnaceIfSafe(const write_lock&);
  void TurnOnFan(const write_lock&);
  void TurnOffFan(const write_lock&);

  // True if we no longer want to opportunistically heat even when heat is free.
  bool TooDamnHot(const write_lock&);
  // True if we've reached the furnace heat max.
  bool WantLessFurnace(const write_lock&);
  // True if we're too cold.
  bool NeedMoreHeat(const write_lock&);
  // True if we're happy to heat a bit further, especially if it's free.
  bool WantMoreHeat(const write_lock&);
  bool HasSufficientResidualHeat(const write_lock&);

  // Devices
  Hardware& furnace_hw_;
  Hardware& fan_hw_;
  TimestampProvider& timestamp_provider_;

  // Config
  float temp_target_in_c_ = 19;
  float hysteresis_offset_in_c_ = 1.0;
  float opportunistic_heat_offset_in_c_ = 5.0;

  // State
  Mode mode_ = OFF;
  bool furnace_on_ = false;
  int64_t furnace_on_timestamp_ = 0;
  bool fan_on_ = false;
  int64_t fan_on_timestamp_ = 0;
  float last_temp_in_c_ = NAN;
  float last_heater_temp_in_c_ = NAN;
  std::shared_mutex m_;
};

#endif  // __THERMOSTAT_H__