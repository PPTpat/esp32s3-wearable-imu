#include "hal/i2c_bus.hpp"

#include <array>
#include <cstring>

namespace hal {

Status I2cBus::mapError(esp_err_t err)
{
    switch (err) {
    case ESP_OK:
        return Status::Ok;
    case ESP_ERR_INVALID_ARG:
        return Status::ErrArg;
    case ESP_ERR_TIMEOUT:
        return Status::ErrTimeout;
    default:
        return Status::ErrBus;
    }
}

Status I2cBus::init(const Config &cfg)
{
    if (cfg.sdaPin < 0 || cfg.sclPin < 0 || cfg.address == 0 || bus_ != nullptr) {
        return Status::ErrArg;
    }

    i2c_master_bus_config_t busCfg{};
    busCfg.i2c_port = cfg.port;
    busCfg.sda_io_num = static_cast<gpio_num_t>(cfg.sdaPin);
    busCfg.scl_io_num = static_cast<gpio_num_t>(cfg.sclPin);
    busCfg.clk_source = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt = 7;
    busCfg.flags.enable_internal_pullup = true; // fallback only; use external 4.7k pull-ups
    if (const auto st = mapError(i2c_new_master_bus(&busCfg, &bus_)); st != Status::Ok) {
        return st;
    }

    i2c_device_config_t devCfg{};
    devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    devCfg.device_address = cfg.address;
    devCfg.scl_speed_hz = cfg.freqHz;
    if (const auto st = mapError(i2c_master_bus_add_device(bus_, &devCfg, &dev_)); st != Status::Ok) {
        return st;
    }

    timeoutMs_ = static_cast<int>(cfg.timeoutMs);
    return Status::Ok;
}

Status I2cBus::read(std::uint8_t reg, std::span<std::uint8_t> out)
{
    if (dev_ == nullptr || out.empty()) {
        return Status::ErrArg;
    }
    return mapError(i2c_master_transmit_receive(dev_, &reg, 1, out.data(), out.size(), timeoutMs_));
}

Status I2cBus::write(std::uint8_t reg, std::span<const std::uint8_t> data)
{
    if (dev_ == nullptr || data.empty() || data.size() > kMaxWriteBytes) {
        return Status::ErrArg;
    }
    std::array<std::uint8_t, kMaxWriteBytes + 1> frame{};
    frame[0] = reg;
    std::memcpy(&frame[1], data.data(), data.size());
    return mapError(i2c_master_transmit(dev_, frame.data(), data.size() + 1, timeoutMs_));
}

} // namespace hal