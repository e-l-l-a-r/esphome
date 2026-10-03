#include "ds18b20.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstdlib>

namespace esphome {
namespace ds18b20 {

  static const char *const TAG = "ds18b20.temp.sensor";

  // Имя таймаута чтения. Планировщик ESPHome различает таймауты по паре
  // (компонент, имя), поэтому статической строки достаточно для каждого экземпляра.
  // С 2026.7.0 set_timeout() принимает только const char* со статическим временем жизни
  // или числовой id — перегрузка с std::string удалена.
  static const char *const READ_TIMEOUT_NAME = "read";

  static const uint8_t DALLAS_MODEL_DS18S20 = 0x10;
  static const uint8_t DALLAS_MODEL_DS18B20 = 0x28;
  static const uint8_t DALLAS_COMMAND_START_CONVERSION = 0x44;
  static const uint8_t DALLAS_COMMAND_READ_SCRATCH_PAD = 0xBE;
  static const uint8_t DALLAS_COMMAND_WRITE_SCRATCH_PAD = 0x4E;
  static const uint8_t DALLAS_COMMAND_COPY_SCRATCH_PAD = 0x48;

  uint16_t DS18B20Sensor::millis_to_wait_for_conversion_() const {
    switch (this->resolution_) {
      case 9:
        return 94;
      case 10:
        return 188;
      case 11:
        return 375;
      default:
        return 750;
    }
  }

  void DS18B20Sensor::dump_config() {
    ESP_LOGCONFIG(TAG, "Dallas Temperature Sensor:");
    if (this->address_ == 0) {
      ESP_LOGW(TAG, "  Unable to select an address");
      return;
    }
    LOG_ONE_WIRE_DEVICE(this);
    ESP_LOGCONFIG(TAG, "  Resolution: %u bits", this->resolution_);
    ESP_LOGCONFIG(TAG, "  Offset: %.1f", this->offset_);
    LOG_UPDATE_INTERVAL(this);
  }

  void DS18B20Sensor::update() {
    if (this->address_ == 0)
      return;

    this->status_clear_warning();

    this->send_command_(DALLAS_COMMAND_START_CONVERSION);

    this->set_timeout(READ_TIMEOUT_NAME, this->millis_to_wait_for_conversion_(), [this] {
      if (!this->read_scratch_pad_() || !this->check_scratch_pad_()) {
        this->publish_state(NAN);
        return;
      }

      float tempc = this->get_temp_c_();
      if (std::isnan(tempc)) {
        this->publish_state(NAN);
        return;
      }
      ESP_LOGD(TAG, "'%s': Got Temperature=%.1f°C; Offset = %.1f", this->get_name().c_str(), tempc, this->offset_);
      this->publish_state(tempc + this->offset_);
    });
  }

  bool DS18B20Sensor::read_scratch_pad_() {
    bool success = this->send_command_(DALLAS_COMMAND_READ_SCRATCH_PAD);
    if (success) {
      for (uint8_t &i : this->scratch_pad_) {
        i = this->bus_->read8();
      }
    } else {
      ESP_LOGW(TAG, "'%s' - reading scratch pad failed bus reset", this->get_name().c_str());
      this->status_set_warning(LOG_STR("bus reset failed"));
    }
    return success;
  }

  void DS18B20Sensor::setup() {
    // Определяет адрес по index: или автоматически (если на шине одно устройство).
    // Без этого вызова датчик, заданный через index: или без address:, никогда не опрашивался.
    if (!this->check_address_or_index_())
      return;
    if (!this->read_scratch_pad_())
      return;
    if (!this->check_scratch_pad_())
      return;

    if ((this->address_ & 0xff) == DALLAS_MODEL_DS18S20) {
      // DS18S20 doesn't support resolution.
      ESP_LOGW(TAG, "DS18S20 doesn't support setting resolution");
      return;
    }

    uint8_t res;
    switch (this->resolution_) {
      case 12:
        res = 0x7F;
        break;
      case 11:
        res = 0x5F;
        break;
      case 10:
        res = 0x3F;
        break;
      case 9:
      default:
        res = 0x1F;
        break;
    }

    if (this->scratch_pad_[4] == res)
      return;
    this->scratch_pad_[4] = res;

    if (this->send_command_(DALLAS_COMMAND_WRITE_SCRATCH_PAD)) {
      this->bus_->write8(this->scratch_pad_[2]);  // high alarm temp
      this->bus_->write8(this->scratch_pad_[3]);  // low alarm temp
      this->bus_->write8(this->scratch_pad_[4]);  // resolution
    }

    // write value to EEPROM
    this->send_command_(DALLAS_COMMAND_COPY_SCRATCH_PAD);
  }

  bool DS18B20Sensor::check_scratch_pad_() {
    bool chksum_validity = (crc8(this->scratch_pad_, 8) == this->scratch_pad_[8]);

  #ifdef ESPHOME_LOG_LEVEL_VERY_VERBOSE
    ESP_LOGVV(TAG, "Scratch pad: %02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X (%02X)", this->scratch_pad_[0],
              this->scratch_pad_[1], this->scratch_pad_[2], this->scratch_pad_[3], this->scratch_pad_[4],
              this->scratch_pad_[5], this->scratch_pad_[6], this->scratch_pad_[7], this->scratch_pad_[8],
              crc8(this->scratch_pad_, 8));
  #endif
    if (!chksum_validity) {
      this->status_set_warning(LOG_STR("scratch pad checksum invalid"));
      ESP_LOGD(TAG, "Scratch pad: %02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X (%02X)", this->scratch_pad_[0],
               this->scratch_pad_[1], this->scratch_pad_[2], this->scratch_pad_[3], this->scratch_pad_[4],
               this->scratch_pad_[5], this->scratch_pad_[6], this->scratch_pad_[7], this->scratch_pad_[8],
               crc8(this->scratch_pad_, 8));
    }
    return chksum_validity;
  }

  float DS18B20Sensor::get_temp_c_() {
    int16_t temp = (this->scratch_pad_[1] << 8) | this->scratch_pad_[0];
    if ((this->address_ & 0xff) == DALLAS_MODEL_DS18S20) {
      // Защита от деления на ноль: scratch pad из одних нулей проходит проверку CRC.
      if (this->scratch_pad_[7] == 0)
        return NAN;
      return (temp >> 1) + (this->scratch_pad_[7] - this->scratch_pad_[6]) / float(this->scratch_pad_[7]) - 0.25f;
    }
    switch (this->resolution_) {
      case 9:
        temp &= 0xfff8;
        break;
      case 10:
        temp &= 0xfffc;
        break;
      case 11:
        temp &= 0xfffe;
        break;
      case 12:
      default:
        break;
    }
    // 85 °C — значение по включении питания: датчик сбросился и ещё не измерял.
    // https://github.com/cpetrich/counterfeit_DS18B20#solution-to-the-85-c-problem
    if ((this->address_ & 0xff) == DALLAS_MODEL_DS18B20) {
      if ((temp == 85 * 16) && (this->scratch_pad_[6] == 0x0c)) {
        ESP_LOGD(TAG, "dropping reading caused by sensor reset");
        return NAN;
      }
    }
    return temp / 16.0f;
  }

  void DS18B20Sensor::rescan() {
    this->bus_->search();
  }

  std::vector<std::string> DS18B20Sensor::get_devices() {
    std::vector<std::string> result;
    // format_hex(uint64_t) устарел и будет удалён в 2026.11.0 — используем буфер на стеке.
    char buf[format_hex_prefixed_size(sizeof(uint64_t))];
    for (uint64_t item : this->bus_->get_devices())
      result.emplace_back(format_hex_prefixed_to(buf, item));
    return result;
  }

  void DS18B20Sensor::set_str_address(const std::string &address) {
    // std::stoull бросает исключение на некорректной строке, а прошивки ESPHome
    // собираются без исключений — это приводило к перезагрузке. strtoull безопасен.
    const char *str = address.c_str();
    char *end = nullptr;
    uint64_t addr = strtoull(str, &end, 16);  // префикс "0x" допускается
    if (end == str || *end != '\0' || addr == 0) {
      ESP_LOGW(TAG, "Invalid address: '%s'", str);
      return;
    }
    // set_address() сбрасывает кешированное имя адреса (используется в логах);
    // прямое присваивание address_ оставляло в логах старый адрес.
    this->set_address(addr);
    ESP_LOGD(TAG, "Set address to: %s", this->get_address_name().c_str());
    this->setup();
    this->update();
  }

  void DS18B20Sensor::set_offset(float offset) {
    this->offset_ = offset;
    ESP_LOGD(TAG, "Set offset to: %.1f", this->offset_);
  }

} // namespace ds18b20
} // namespace esphome
