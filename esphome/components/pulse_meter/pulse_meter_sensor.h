#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/helpers.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


namespace esphome {
namespace pulse_meter {

struct pulse_data_t {
   uint32_t ts_value;
};

class PulseMeterSensor : public sensor::Sensor, public Component {
 public:
  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
  void set_filter_us(uint32_t filter) { this->filter_us_ = filter; }
  void set_timeout_us(uint32_t timeout) { this->timeout_us_ = timeout; }
  void set_total_sensor(sensor::Sensor *sensor) { this->total_sensor_ = sensor; }

  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void dump_config() override;

 protected:
  static void gpio_intr(PulseMeterSensor *sensor);

  InternalGPIOPin *pin_ = nullptr;
  ISRInternalGPIOPin isr_pin_;
  uint32_t filter_us_ = 0;
  uint32_t timeout_us_ = 1000000UL * 60UL * 5UL;
  sensor::Sensor *total_sensor_ = nullptr;

  Deduplicator<uint32_t> pulse_width_dedupe_;
  Deduplicator<uint32_t> total_dedupe_;

  volatile uint32_t total_pulses_ = 0;
  QueueHandle_t pulse_q_;
  volatile uint32_t last_irq = 0;
  volatile uint32_t last_pulse;
};

}  // namespace pulse_meter
}  // namespace esphome
