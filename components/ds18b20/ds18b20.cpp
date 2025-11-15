//
// Created by ellar on 15.11.25.
//

#include "ds18b20.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ds18b20 {

    static const char *const TAG = "ds18b20.temp.sensor";

    void DS18B20Sensor::dump_config() {
        dallas_temp::DallasTemperatureSensor::dump_config();
    }

    void DS18B20Sensor::update() {
        dallas_temp::DallasTemperatureSensor::update();
    }

    void DS18B20Sensor::rescan() {
        this->bus_->search();
    }

    const std::vector<std::string> DS18B20Sensor::get_devices()
    {
        std::vector<std::string> devices_;
        for( auto item:this->bus_->get_devices() )
            devices_.push_back( std::string("0x") + format_hex(item) );
        return devices_;
    }

} // ds18b20
} // esphome