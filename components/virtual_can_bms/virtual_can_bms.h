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
  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  canbus::Canbus *canbus;

 protected:
  void register_sensor_callbacks_();

  void send_frame_0x0351_();
  void send_frame_0x0355_();
  void send_frame_0x0356_();
  void send_frame_0x035a_();

  void build_frame_0x0351_(SmaCanMessage0x0351 &message);
  void build_frame_0x0355_(SmaCanMessage0x0355 &message);
  void build_frame_0x0356_(SmaCanMessage0x0356 &message);
  void build_frame_0x035a_(SmaCanMessage0x035A &message);

  sensor::Sensor *charge_voltage_sensor_{nullptr};
  sensor::Sensor *charge_current_limit_sensor_{nullptr};
  sensor::Sensor *discharge_current_limit_sensor_{nullptr};
  sensor::Sensor *discharge_voltage_limit_sensor_{nullptr};
  sensor::Sensor *state_of_charge_sensor_{nullptr};
  sensor::Sensor *state_of_health_sensor_{nullptr};
  sensor::Sensor *hires_state_of_charge_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *battery_current_sensor_{nullptr};
  sensor::Sensor *battery_temperature_sensor_{nullptr};

  bool sensor_0x0351_updated_{false};
  bool sensor_0x0355_updated_{false};
  bool sensor_0x0356_updated_{false};
  bool sensor_0x035a_updated_{false};

  uint32_t last_frame_time_{0};
  uint32_t last_mandatory_frame_time_{0};

  SmaCanMessage0x0351 last_frame_0x0351_{};
  SmaCanMessage0x0355 last_frame_0x0355_{};
  SmaCanMessage0x0356 last_frame_0x0356_{};
  SmaCanMessage0x035A last_frame_0x035a_{};

  static constexpr uint32_t FRAME_INTERVAL_MS = 200;
  static constexpr uint32_t MANDATORY_FRAME_INTERVAL_MS = 10000;  // 10 seconds
};

 private:
  // Add variables to store the last known state of each sensor
  float last_charge_voltage_ = 0;
  float last_charge_current_limit_ = 0;
  float last_discharge_current_limit_ = 0;
  float last_discharge_voltage_limit_ = 0;
  float last_state_of_charge_ = 0;
  float last_state_of_health_ = 0;
  float last_hires_state_of_charge_ = 0;
  float last_battery_voltage_ = 0;
  float last_battery_current_ = 0;
  float last_battery_temperature_ = 0;

}  // namespace virtual_can_bms
}  // namespace esphome
