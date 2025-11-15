//
// Created by ellar on 15.11.25.
//

#pragma once

#include "esphome/components/dallas_temp/dallas_temp.h"

namespace esphome {
namespace ds18b20 {

class DS18B20Sensor : public dallas_temp::DallasTemperatureSensor {
public:
    void setup() override;
    void update() override;
    void dump_config() override;

    void rescan();

    const std::vector<std::string> get_devices();
};

} // ds18b20
} // esphome

