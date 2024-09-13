#pragma once

#include "esphome/core/component.h"
#include "esphome/components/canbus/canbus.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace virtual_can_bms {

struct SmaCanMessage0x0351 {
  uint16_t ChargeVoltage;
  int16_t MaxChargingCurrent;
  int16_t MaxDischargingCurrent;
  uint16_t DischargeVoltageLimit;
};

struct SmaCanMessage0x0355 {
  uint16_t StateOfCharge;
  uint16_t StateOfHealth;
  uint16_t StateOfChargeHighRes;
};

struct SmaCanMessage0x0356 {
  int16_t BatteryVoltage;
  int16_t BatteryCurrent;
  int16_t BatteryTemperature;
};

struct SmaCanMessage0x035A {
  uint32_t AlarmBitmask;
  uint32_t WarningBitmask;
};

struct SmaCanMessage0x035E {
  char Model[8];
};

struct SmaCanMessage0x035F {
  uint16_t CellChemistry;
  uint8_t HardwareVersion[2];
  uint16_t NominalCapacity;
  uint8_t SoftwareVersion[2];
};

struct SmaCanMessage0x0370 {
  char Manufacturer[8];
};

struct SmaCanMessage0x0373 {
  uint16_t MinCellvoltage;
  uint16_t MaxCellvoltage;
  uint16_t MinTemperature;
  uint16_t MaxTemperature;
};

class VirtualCanBms : public Component {
 public:
  void set_canbus(canbus::Canbus *canbus) { this->canbus = canbus; }

  void set_charge_voltage_sensor(sensor::Sensor *charge_voltage_sensor) {
    charge_voltage_sensor_ = charge_voltage_sensor;
  }
  void set_charge_current_limit_sensor(sensor::Sensor *charge_current_limit_sensor) {
    charge_current_limit_sensor_ = charge_current_limit_sensor;
  }
  void set_discharge_current_limit_sensor(sensor::Sensor *discharge_current_limit_sensor) {
    discharge_current_limit_sensor_ = discharge_current_limit_sensor;
  }
  void set_discharge_voltage_limit_sensor(sensor::Sensor *discharge_voltage_limit_sensor) {
    discharge_voltage_limit_sensor_ = discharge_voltage_limit_sensor;
  }
  void set_state_of_charge_sensor(sensor::Sensor *state_of_charge_sensor) {
    state_of_charge_sensor_ = state_of_charge_sensor;
  }
  void set_state_of_health_sensor(sensor::Sensor *state_of_health_sensor) {
    state_of_health_sensor_ = state_of_health_sensor;
  }
  void set_hires_state_of_charge_sensor(sensor::Sensor *hires_state_of_charge_sensor) {
    hires_state_of_charge_sensor_ = hires_state_of_charge_sensor;
  }
  void set_battery_voltage_sensor(sensor::Sensor *battery_voltage_sensor) {
    battery_voltage_sensor_ = battery_voltage_sensor;
  }
  void set_battery_current_sensor(sensor::Sensor *battery_current_sensor) {
    battery_current_sensor_ = battery_current_sensor;
  }
  void set_battery_temperature_sensor(sensor::Sensor *battery_temperature_sensor) {
    battery_temperature_sensor_ = battery_temperature_sensor;
  }

  void dump_config() override;
  void loop() override;
  void setup() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  canbus::Canbus *canbus;

 protected:
  sensor::Sensor *charge_voltage_sensor_;
  sensor::Sensor *charge_current_limit_sensor_;
  sensor::Sensor *discharge_current_limit_sensor_;
  sensor::Sensor *discharge_voltage_limit_sensor_;
  sensor::Sensor *state_of_charge_sensor_;
  sensor::Sensor *state_of_health_sensor_;
  sensor::Sensor *hires_state_of_charge_sensor_;
  sensor::Sensor *battery_voltage_sensor_;
  sensor::Sensor *battery_current_sensor_;
  sensor::Sensor *battery_temperature_sensor_;

  void send_frame_0x0351_();
  void send_frame_0x0355_();
  void send_frame_0x0356_();
  void send_frame_0x035a_();
  void publish_state_(sensor::Sensor *sensor, float value);

  enum class State {
    SEND_0X0351,
    SEND_0X0355,
    SEND_0X0356,
    SEND_0X035A,
    IDLE
  };

  State current_state_{State::SEND_0X0351};
  uint32_t last_frame_time_{0};
  static constexpr uint32_t FRAME_INTERVAL_MS = 200;
};

}  // namespace virtual_can_bms
}  // namespace esphome
