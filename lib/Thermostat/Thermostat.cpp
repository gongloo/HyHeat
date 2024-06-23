#include <ArduinoJson.h>
#include <Thermostat.h>
#include <math.h>

Thermostat::Thermostat(Hardware& furnace_hw, Hardware& fan_hw,
                       TimestampProvider& timestamp_provider)
    : furnace_hw_(furnace_hw),
      fan_hw_(fan_hw),
      timestamp_provider_(timestamp_provider) {}

void Thermostat::OnTempSamples(float main_temp_in_c, float heater_temp_in_c) {
  write_lock l(m_);
  if (!isnan(main_temp_in_c)) {
    last_temp_in_c_ = main_temp_in_c;
  }
  if (!isnan(heater_temp_in_c)) {
    last_heater_temp_in_c_ = heater_temp_in_c;
  }
  // TODO: Count NaN samples and put ourselves in an error state.
  ApplyLogic(l);
}

void Thermostat::ApplyLogic(const write_lock& l) {
  // Fail safe if our temperature readings are bogus.
  if (isnan(last_temp_in_c_) || isnan(last_heater_temp_in_c_)) {
    TurnOffFan(l);
    TurnOffFurnaceIfSafe(l);
    return;
  }

  // Make sure the fan is off if we're not supposed to be automatically managing
  // it.
  if (mode_ < ENGINE && fan_on_) {
    TurnOffFan(l);
  }

  // Make sure the furnace is off if we're not supposed to be automatically
  // managing it.
  if (mode_ < FURNACE && furnace_on_) {
    TurnOffFurnaceIfSafe(l);
  }

  // Furnace thermostat logic.
  if (mode_ >= FURNACE) {
    if (furnace_on_) {
      if (WantLessFurnace(l)) {
        // Turn off the furnace when we're warm enough.
        TurnOffFurnaceIfSafe(l);
      }
    } else if (NeedMoreHeat(l) && !HasSufficientResidualHeat(l)) {
      // Turn on the furnace when we're too cold and have no residual heat.
      TurnOnFurnace(l);
    }
  }

  // Fan thermostat logic.
  if (mode_ >= ENGINE) {  // Use residual heat in any non-off mode.
    if (fan_on_) {
      // Turn off the fan if we're hot enough, or don't have any additional heat
      // to impart.
      if (TooDamnHot(l) || !HasSufficientResidualHeat(l)) {
        TurnOffFan(l);
      }
    } else if (!furnace_on_) {  // The furnace automatically manages the fan.
      // Turn on the fan if we could use a little more heat.
      if (WantMoreHeat(l) && HasSufficientResidualHeat(l)) {
        TurnOnFan(l);
      }
    }
  }
}

void Thermostat::SetTempTarget(float temp_target_in_c) {
  if (temp_target_in_c < kMinTempTargetInC ||
      temp_target_in_c > kMaxTempTargetInC) {
    // Out of bounds. Do nothing.
    return;
  }

  write_lock l(m_);
  temp_target_in_c_ = temp_target_in_c;
  ApplyLogic(l);
}

void Thermostat::SetHysteresisOffset(float hysteresis_offset_in_c) {
  if (hysteresis_offset_in_c < kMinHysteresisOffsetInC ||
      hysteresis_offset_in_c > kMaxHysteresisOffsetInC) {
    // Out of bounds. Do nothing.
    return;
  }

  write_lock l(m_);
  hysteresis_offset_in_c_ =
      std::max(hysteresis_offset_in_c, kMinHysteresisOffsetInC);
  ApplyLogic(l);
}

void Thermostat::SetOpportunisticHeatOffset(
    float opportunistic_heat_offset_in_c) {
  if (opportunistic_heat_offset_in_c < 0 ||
      opportunistic_heat_offset_in_c > kMaxOpportunisticHeatOffsetInC) {
    // Out of bounds. Do nothing.
    return;
  }

  write_lock l(m_);
  opportunistic_heat_offset_in_c_ = opportunistic_heat_offset_in_c;
  ApplyLogic(l);
}

void Thermostat::SetMode(Mode mode) {
  write_lock l(m_);
  mode_ = mode;
  ApplyLogic(l);
}

void Thermostat::TurnOnFurnace() {
  write_lock l(m_);
  TurnOnFurnace(l);
}

void Thermostat::TurnOnFan() {
  write_lock l(m_);
  TurnOnFan(l);
}

void Thermostat::TurnOnFurnace(const write_lock&) {
  furnace_hw_.TurnOn();
  fan_hw_.TurnOn();
  furnace_on_timestamp_ = timestamp_provider_.GetCurrentTimestamp();
  furnace_on_ = true;
}

void Thermostat::TurnOffFurnaceIfSafe(const write_lock&) {
  if (timestamp_provider_.GetCurrentTimestamp() - furnace_on_timestamp_ <
      kMinFurnaceRuntimeInS * 1000) {
    // Not yet safe.
    return;
  }

  furnace_hw_.TurnOff();
  fan_hw_.TurnOff();
  furnace_on_ = false;
}

void Thermostat::TurnOnFan(const write_lock&) {
  fan_hw_.TurnOn();
  fan_on_ = true;
  fan_on_timestamp_ = timestamp_provider_.GetCurrentTimestamp();
}

void Thermostat::TurnOffFan(const write_lock&) {
  fan_hw_.TurnOff();
  fan_on_ = false;
}

bool Thermostat::TooDamnHot(const write_lock&) {
  return last_temp_in_c_ > temp_target_in_c_ + opportunistic_heat_offset_in_c_ +
                               kMinHysteresisOffsetInC;
}

bool Thermostat::WantLessFurnace(const write_lock&) {
  return last_temp_in_c_ > temp_target_in_c_ + hysteresis_offset_in_c_;
}

bool Thermostat::NeedMoreHeat(const write_lock&) {
  return last_temp_in_c_ <= temp_target_in_c_ - hysteresis_offset_in_c_;
}

bool Thermostat::WantMoreHeat(const write_lock&) {
  return last_temp_in_c_ <= temp_target_in_c_ +
                                opportunistic_heat_offset_in_c_ -
                                kMinHysteresisOffsetInC;
}

bool Thermostat::HasSufficientResidualHeat(const write_lock&) {
  if (fan_on_) {
    return last_heater_temp_in_c_ >
           kMinResidualHeatInC - kMinHysteresisOffsetInC;
  }
  return last_heater_temp_in_c_ > kMinResidualHeatInC + kMinHysteresisOffsetInC;
}
