/**
 * @file bus.hpp
 * @brief Register-oriented bus abstraction (SPI / I2C / host mock).
 *
 * Drivers depend only on hal::IBus, so they can be unit-tested on a PC with
 * a mock bus and ported to any transport without modification.
 */
#pragma once

#include <cstdint>
#include <span>

namespace hal {

/** @brief Result of a HAL operation. */
enum class [[nodiscard]] Status : std::int8_t {
  Ok = 0,          ///< Operation succeeded.
  ErrBus = -1,     ///< Transport-level failure (NACK, timeout, CRC...).
  ErrArg = -2,     ///< Invalid argument (empty buffer, address out of range).
  ErrTimeout = -3, ///< Device did not reach the expected state in time.
  ErrDevice = -4,  ///< Wrong or missing device (e.g. unexpected WHO_AM_I).
};

/**
 * @brief Abstract register bus.
 *
 * Multi-byte transfers use register auto-increment. Implementations must not
 * throw and should not allocate memory.
 */
class IBus {
public:
  virtual ~IBus() = default;

  /**
   * @brief Read consecutive registers.
   * @param reg  Address of the first register.
   * @param out  Destination buffer; its size is the number of bytes read.
   * @return Status::Ok on success.
   */
  virtual Status read(std::uint8_t reg, std::span<std::uint8_t> out) = 0;

  /**
   * @brief Write consecutive registers.
   * @param reg   Address of the first register.
   * @param data  Bytes to write.
   * @return Status::Ok on success.
   */
  virtual Status write(std::uint8_t reg,
                       std::span<const std::uint8_t> data) = 0;

  /** @brief Read a single register. */
  Status readU8(std::uint8_t reg, std::uint8_t &value) {
    return read(reg, std::span<std::uint8_t>(&value, 1));
  }

  /** @brief Write a single register. */
  Status writeU8(std::uint8_t reg, std::uint8_t value) {
    return write(reg, std::span<const std::uint8_t>(&value, 1));
  }
};

} // namespace hal