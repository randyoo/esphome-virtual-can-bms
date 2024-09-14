#include "virtual_can_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace virtual_can_bms {

static const char *const TAG = "virtual_can_bms";

void VirtualCanBms::setup() {
  this->last_frame_time_ = millis();
  this->last_frame_0x0351_time_ = millis();
  this->last_frame_0x0355_time_ = millis();
  this->last_frame_0x0356_time_ = millis();
  this->last_frame_0x035a_time_ = millis();
  this->register_sensor_callbacks_();
}

void VirtualCanBms::register_sensor_callbacks_() {
  if (this->charge_voltage_sensor_ != nullptr)
    this->charge_voltage_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_charge_voltage_) {
        this->last_charge_voltage_ = state;
        this->sensor_0x0351_updated_ = true;
      }
    });
  
  if (this->charge_current_limit_sensor_ != nullptr)
    this->charge_current_limit_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_charge_current_limit_) {
        this->last_charge_current_limit_ = state;
        this->sensor_0x0351_updated_ = true;
      }
    });
  
  if (this->discharge_current_limit_sensor_ != nullptr)
    this->discharge_current_limit_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_discharge_current_limit_) {
        this->last_discharge_current_limit_ = state;
        this->sensor_0x0351_updated_ = true;
      }
    });
  
  if (this->discharge_voltage_limit_sensor_ != nullptr)
    this->discharge_voltage_limit_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_discharge_voltage_limit_) {
        this->last_discharge_voltage_limit_ = state;
        this->sensor_0x0351_updated_ = true;
      }
    });
  
  if (this->state_of_charge_sensor_ != nullptr)
    this->state_of_charge_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_state_of_charge_) {
        this->last_state_of_charge_ = state;
        this->sensor_0x0355_updated_ = true;
      }
    });
  
  if (this->state_of_health_sensor_ != nullptr)
    this->state_of_health_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_state_of_health_) {
        this->last_state_of_health_ = state;
        this->sensor_0x0355_updated_ = true;
      }
    });
  
  if (this->hires_state_of_charge_sensor_ != nullptr)
    this->hires_state_of_charge_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_hires_state_of_charge_) {
        this->last_hires_state_of_charge_ = state;
        this->sensor_0x0355_updated_ = true;
      }
    });
  
  if (this->battery_voltage_sensor_ != nullptr)
    this->battery_voltage_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_battery_voltage_) {
        this->last_battery_voltage_ = state;
        this->sensor_0x0356_updated_ = true;
      }
    });
  
  if (this->battery_current_sensor_ != nullptr)
    this->battery_current_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_battery_current_) {
        this->last_battery_current_ = state;
        this->sensor_0x0356_updated_ = true;
      }
    });
  
  if (this->battery_temperature_sensor_ != nullptr)
    this->battery_temperature_sensor_->add_on_state_callback([this](float state) {
      if (state != this->last_battery_temperature_) {
        this->last_battery_temperature_ = state;
        this->sensor_0x0356_updated_ = true;
      }
    });
}

void VirtualCanBms::dump_config() { ESP_LOGCONFIG(TAG, "VirtualCanBms:"); }

void VirtualCanBms::loop() {
  uint32_t now = millis();
  if (now - this->last_frame_time_ < FRAME_INTERVAL_MS) {
    return;  // Respect minimum interval between frames
  }

  bool frame_sent = false;

  // Check and send 0x0351 frame
  if (this->sensor_0x0351_updated_ || (now - this->last_frame_0x0351_time_ >= MANDATORY_FRAME_INTERVAL_MS)) {
    this->send_frame_0x0351_();
    this->sensor_0x0351_updated_ = false;
    this->last_frame_0x0351_time_ = now;
    frame_sent = true;
  }

  // Check and send 0x0355 frame
  if (!frame_sent && (this->sensor_0x0355_updated_ || (now - this->last_frame_0x0355_time_ >= MANDATORY_FRAME_INTERVAL_MS))) {
    this->send_frame_0x0355_();
    this->sensor_0x0355_updated_ = false;
    this->last_frame_0x0355_time_ = now;
    frame_sent = true;
  }

  // Check and send 0x0356 frame
  if (!frame_sent && (this->sensor_0x0356_updated_ || (now - this->last_frame_0x0356_time_ >= MANDATORY_FRAME_INTERVAL_MS))) {
    this->send_frame_0x0356_();
    this->sensor_0x0356_updated_ = false;
    this->last_frame_0x0356_time_ = now;
    frame_sent = true;
  }

  // Check and send 0x035A frame
  if (!frame_sent && (this->sensor_0x035a_updated_ || (now - this->last_frame_0x035a_time_ >= MANDATORY_FRAME_INTERVAL_MS))) {
    this->send_frame_0x035a_();
    this->sensor_0x035a_updated_ = false;
    this->last_frame_0x035a_time_ = now;
    frame_sent = true;
  }

  if (frame_sent) {
    this->last_frame_time_ = now;
  }
}

void VirtualCanBms::send_frame_0x0351_() {
  SmaCanMessage0x0351 message;
  this->build_frame_0x0351_(message);
  auto *ptr = reinterpret_cast<uint8_t *>(&message);
  this->canbus->send_data(0x0351, false, false, std::vector<uint8_t>(ptr, ptr + sizeof message));
  this->last_frame_0x0351_ = message;
  this->last_frame_0x0351_time_ = millis();  // Update the timer
  ESP_LOGI(TAG, "Sent frame 0x0351");
}

void VirtualCanBms::send_frame_0x0355_() {
  SmaCanMessage0x0355 message;
  this->build_frame_0x0355_(message);
  auto *ptr = reinterpret_cast<uint8_t *>(&message);
  this->canbus->send_data(0x0355, false, false, std::vector<uint8_t>(ptr, ptr + sizeof message));
  this->last_frame_0x0355_ = message;
  this->last_frame_0x0355_time_ = millis();  // Update the timer
  ESP_LOGI(TAG, "Sent frame 0x0355");
}

void VirtualCanBms::send_frame_0x0356_() {
  SmaCanMessage0x0356 message;
  this->build_frame_0x0356_(message);
  auto *ptr = reinterpret_cast<uint8_t *>(&message);
  this->canbus->send_data(0x0356, false, false, std::vector<uint8_t>(ptr, ptr + sizeof message));
  this->last_frame_0x0356_ = message;
  this->last_frame_0x0356_time_ = millis();  // Update the timer
  ESP_LOGI(TAG, "Sent frame 0x0356");
}

void VirtualCanBms::send_frame_0x035a_() {
  SmaCanMessage0x035A message;
  this->build_frame_0x035a_(message);
  auto *ptr = reinterpret_cast<uint8_t *>(&message);
  this->canbus->send_data(0x035A, false, false, std::vector<uint8_t>(ptr, ptr + sizeof message));
  this->last_frame_0x035a_ = message;
  this->last_frame_0x035a_time_ = millis();  // Update the timer
  ESP_LOGI(TAG, "Sent frame 0x035A");
}

void VirtualCanBms::build_frame_0x0351_(SmaCanMessage0x0351 &message) {
  if (this->charge_voltage_sensor_ == nullptr || this->charge_current_limit_sensor_ == nullptr ||
      this->discharge_current_limit_sensor_ == nullptr || this->discharge_voltage_limit_sensor_ == nullptr) {
    ESP_LOGW(TAG, "One of the required sensors for frame 0x0351 is missing");
    return;
  }

  float charge_voltage = this->charge_voltage_sensor_->get_state();
  float charge_current_limit = this->charge_current_limit_sensor_->get_state();
  float discharge_current_limit = this->discharge_current_limit_sensor_->get_state();
  float discharge_voltage_limit = this->discharge_voltage_limit_sensor_->get_state();

  message.ChargeVoltage = ((charge_voltage) * 10.0f);
  message.MaxChargingCurrent = (charge_current_limit * 10.0f);
  message.MaxDischargingCurrent = (discharge_current_limit * 10.0f);
  message.DischargeVoltageLimit = (discharge_voltage_limit * 10.0f);
}

void VirtualCanBms::build_frame_0x0355_(SmaCanMessage0x0355 &message) {
  if (this->state_of_charge_sensor_ == nullptr || this->state_of_health_sensor_ == nullptr) {
    ESP_LOGW(TAG, "One of the required sensors for frame 0x0355 is missing");
    return;
  }

  float state_of_charge = this->state_of_charge_sensor_->get_state();
  float state_of_health = this->state_of_health_sensor_->get_state();
  float hires_state_of_charge = this->hires_state_of_charge_sensor_ != nullptr
                                    ? this->hires_state_of_charge_sensor_->get_state()
                                    : 65535;  // Invalid unsigned if sensor not available

  message.StateOfCharge = state_of_charge;
  message.StateOfHealth = state_of_health;
  message.StateOfChargeHighRes = (hires_state_of_charge * 100.0f);
}

void VirtualCanBms::build_frame_0x0356_(SmaCanMessage0x0356 &message) {
  if (this->battery_voltage_sensor_ == nullptr || this->battery_current_sensor_ == nullptr ||
      this->battery_temperature_sensor_ == nullptr) {
    ESP_LOGW(TAG, "One of the required sensors for frame 0x0356 is missing");
    return;
  }

  float battery_voltage = this->battery_voltage_sensor_->get_state();
  float battery_current = this->battery_current_sensor_->get_state();
  float battery_temperature = this->battery_temperature_sensor_->get_state();

  message.BatteryVoltage = (battery_voltage * 100.0f);
  message.BatteryCurrent = (battery_current * 10.0f);
  message.BatteryTemperature = (battery_temperature * 10.0f);
}

void VirtualCanBms::build_frame_0x035a_(SmaCanMessage0x035A &message) {
  message.AlarmBitmask = 0x00000000;
  message.WarningBitmask = 0x00000000;
}

}  // namespace virtual_can_bms
}  // namespace esphome
