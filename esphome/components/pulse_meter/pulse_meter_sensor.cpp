#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

void PulseMeterSensor::setup() {
  this->pulse_q_ = xQueueCreate(5, sizeof(pulse_data_t *));
  this->pin_->setup();
  this->isr_pin_ = pin_->to_isr();
  this->pin_->attach_interrupt(PulseMeterSensor::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);

}

void PulseMeterSensor::loop() {
    const uint32_t now = micros();

    uint32_t pulse_width;
    pulse_data_t pdata;
    if (xQueueReceive(this->pulse_q_, &pdata, 1L) == pdTRUE)
    {
     //ESP_LOGD(TAG, "Got pulse at %us",pdata->ts_value);

     if (last_pulse != 0)
      {
         pulse_width = pdata.ts_value-last_pulse;
         if (pulse_width > this->filter_us_) 
           {
           ESP_LOGD(TAG, "A: Pulses/Min %f",(60.0 * 1000.0) / (pulse_width / 1000));
           const uint32_t pulse_width_ms = pulse_width / 1000;
           if (this->pulse_width_dedupe_.next(pulse_width_ms)) {
             // Calculate pulses/min from the pulse width in ms
             this->publish_state((60.0 * 1000.0) / pulse_width_ms);
           }
  

           this->last_pulse = pdata.ts_value;
           this->total_pulses_++;
           const uint32_t total = this->total_pulses_;
           if (this->total_dedupe_.next(total)) {
             this->total_sensor_->publish_state(total);
           }
         }
      } else
      {
        this->last_pulse = pdata.ts_value;
      }
     //delete pdata;
    } else
    {
    if (this->last_pulse != 0)
     {
      const uint32_t time_since_valid_edge_us = now - this->last_pulse;
      if  (time_since_valid_edge_us > this->timeout_us_) {
        ESP_LOGD(TAG, "No pulse detected for %us, assuming 0 pulses/min", time_since_valid_edge_us / 1000000);
        this->last_pulse=0;
        this->publish_state(0);
       }
     }
    }
  return;
}

void PulseMeterSensor::set_total_pulses(uint32_t pulses) { this->total_pulses_ = pulses; }

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %u µs", this->filter_us_);
  ESP_LOGCONFIG(TAG, "  Assuming 0 pulses/min after not receiving a pulse for %us", this->timeout_us_ / 1000000);
}

void IRAM_ATTR PulseMeterSensor::gpio_intr(PulseMeterSensor *sensor) {
  // This is an interrupt handler - we can't call any virtual method from this method

  // Get the current time before we do anything else so the measurements are consistent
  const uint32_t now = micros();

  // We only look at rising edges
  if (!sensor->isr_pin_.digital_read()) {
    return;
  }

  if ((now - sensor->last_irq)/1000 > 100) // 100mSec lockout
   { 
     pulse_data_t pdata;// = new pulse_data_t;
     pdata.ts_value = now;
     xQueueSendFromISR(sensor->pulse_q_, &pdata, NULL);
   }
   
  sensor->last_irq = now;
}

}  // namespace pulse_meter
}  // namespace esphome
