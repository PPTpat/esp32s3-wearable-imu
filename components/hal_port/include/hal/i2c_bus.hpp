/**
 * @file i2c_bus.hpp
 * @brief hal::IBus implementation on the ESP-IDF I2C master driver (ESP-IDF
 * only).
 */
#pragma once

#include <cstdint>

#include "driver/i2c_master.h"
#include "hal/bus.hpp"

namespace hal {

/**
 * @brief Register bus for one I2C device (7-bit address).
 *
 * Not thread-safe: use it from one task at a time (init, then the acquisition
 * task). Register writes are limited to kMaxWriteBytes of payload (stack
 * buffer, no heap).
 */
class I2cBus final : public IBus {
public:
  static constexpr std::size_t kMaxWriteBytes = 16;

  /** @brief Bus and device settings. */
  struct Config {
    i2c_port_num_t port = I2C_NUM_0;
    int sdaPin = -1;
    int sclPin = -1;
    std::uint8_t address = 0;      ///< 7-bit device address.
    std::uint32_t freqHz = 400000; ///< SCL frequency.
    std::uint32_t timeoutMs = 20;  ///< Timeout of a single transaction.
  };

  /** @brief Create the I2C bus and add the device. @return Ok, ErrArg or
   * ErrBus. */
  Status init(const Config &cfg);

  Status read(std::uint8_t reg, std::span<std::uint8_t> out) override;
  Status write(std::uint8_t reg, std::span<const std::uint8_t> data) override;

  /**
   * @brief Check whether any device acknowledges @p address (wiring
   * diagnostics).
   * @return Ok if a device answered, ErrBus if nobody did, ErrArg before
   * init().
   */
  Status probe(std::uint8_t address) const;

  /** @brief Raw ESP-IDF result of the most recent transaction (diagnostics
   * only). */
  esp_err_t lastError() const { return lastError_; }

private:
  static Status mapError(esp_err_t err);
  Status record(esp_err_t err) const;

  i2c_master_bus_handle_t bus_ = nullptr;
  i2c_master_dev_handle_t dev_ = nullptr;
  int timeoutMs_ = 20;
  mutable esp_err_t lastError_ = ESP_OK;
};

} // namespace hal