#include "ds18b20.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ds18b20 {

  static const char *const TAG = "ds18b20.temp.sensor";

  static const uint8_t DALLAS_MODEL_DS18S20 = 0x10;
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
    LOG_UPDATE_INTERVAL(this);
  }

  void DS18B20Sensor::update() {
    if (this->address_ == 0)
      return;

    this->status_clear_warning();

    this->send_command_(DALLAS_COMMAND_START_CONVERSION);

    this->set_timeout(this->get_address_name(), this->millis_to_wait_for_conversion_(), [this] {
      if (!this->read_scratch_pad_() || !this->check_scratch_pad_()) {
        this->publish_state(NAN);
        return;
      }

      float tempc = this->get_temp_c_();
      ESP_LOGD(TAG, "'%s': Got Temperature=%.1f°C; Offset = %.1f", this->get_name().c_str(), tempc, offset_);
      this->publish_state(tempc + offset_);
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
    if (this->address_ == 0)
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
      return (temp >> 1) + (this->scratch_pad_[7] - this->scratch_pad_[6]) / float(this->scratch_pad_[7]) - 0.25;
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

    return temp / 16.0f;
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

  void DS18B20Sensor::set_str_address(const std::string &address){
    this->address_ = std::stoull(address, nullptr, 16);
    ESP_LOGD(TAG, "Set address to: 0x%s", format_hex(this->address_).c_str());
    setup();
    update();
  }

  void DS18B20Sensor::set_offset(const float offset)
  {
    offset_ = offset;
    ESP_LOGD(TAG, "Set offset to: %.1f", offset_);
  }

} // ds18b20
} // esphome