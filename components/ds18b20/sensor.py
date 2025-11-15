import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import one_wire, sensor
from esphome.const import (
    CONF_RESOLUTION,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

ds18b20_ns = cg.esphome_ns.namespace("ds18b20")

DS18B20Sensor = ds18b20_ns.class_(
    "DS18B20Sensor",
    cg.PollingComponent,
    sensor.Sensor,
    one_wire.OneWireDevice,
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        DS18B20Sensor,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Optional(CONF_RESOLUTION, default=12): cv.int_range(min=9, max=12),
        }
    )
    .extend(one_wire.one_wire_device_schema())
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await one_wire.register_one_wire_device(var, config)

    cg.add(var.set_resolution(config[CONF_RESOLUTION]))
    #
    # cg.add(var.set_foo(config[CONF_FOO]))
    # if bar := config.get(CONF_BAR):
    #     cg.add(var.set_bar(bar))
    # if baz := config.get(CONF_BAZ):
    #     cg.add(var.set_baz(baz))
