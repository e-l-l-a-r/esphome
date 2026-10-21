#pragma once

#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/one_wire/one_wire.h"

namespace esphome {
namespace ds18b20 {

class DS18B20Sensor : public PollingComponent, public sensor::Sensor, public one_wire::OneWireDevice {
public:
    void setup() override;
    void update() override;
    void dump_config() override;

    /// Повторный поиск устройств на шине 1-Wire.
    void rescan();

    /// Список адресов найденных на шине устройств в виде "0x28ff...".
    std::vector<std::string> get_devices();

    /// Set the resolution for this sensor.
    void set_resolution(uint8_t resolution) { this->resolution_ = resolution; }

    /// Сменить адрес датчика в рантайме (строка hex, с префиксом "0x" или без).
    void set_str_address(const std::string &address);
    void set_offset(float offset);

protected:
    uint8_t resolution_{12};
    uint8_t scratch_pad_[9] = {0};
    float offset_{0.0f};

    /// Get the number of milliseconds we have to wait for the conversion phase.
    uint16_t millis_to_wait_for_conversion_() const;
    bool read_scratch_pad_();
    bool check_scratch_pad_();
    float get_temp_c_();
};

} // namespace ds18b20
} // namespace esphome
