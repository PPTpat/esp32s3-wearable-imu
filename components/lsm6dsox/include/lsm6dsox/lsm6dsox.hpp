/**
 * @file lsm6dsox.hpp
 * @brief Register-level LSM6DSOX 6-axis IMU driver (bus-agnostic).
 */
#pragma once

#include <array>
#include <cstdint>

#include "hal/bus.hpp"

namespace lsm6dsox {

/** @brief Output / batch data rate. Values match the register encoding. */
enum class Odr : std::uint8_t {
  PowerDown = 0,
  Hz12_5 = 1,
  Hz26 = 2,
  Hz52 = 3,
  Hz104 = 4,
  Hz208 = 5,
  Hz416 = 6,
  Hz833 = 7,
  Hz1667 = 8,
  Hz3333 = 9,
  Hz6667 = 10,
};

/** @brief Accelerometer full scale. Values match the register encoding. */
enum class AccelRange : std::uint8_t { G2 = 0, G16 = 1, G4 = 2, G8 = 3 };

/** @brief Gyroscope full scale. Values match the register encoding. */
enum class GyroRange : std::uint8_t {
  Dps250 = 0,
  Dps500 = 1,
  Dps1000 = 2,
  Dps2000 = 3
};

/** @brief Sensor configuration. */
struct Config {
  Odr accelOdr = Odr::Hz104;
  AccelRange accelRange = AccelRange::G4;
  Odr gyroOdr = Odr::Hz104;
  GyroRange gyroRange = GyroRange::Dps500;
};

/** @brief FIFO configuration. */
struct FifoConfig {
  Odr accelBatch =
      Odr::Hz104; ///< Accelerometer batch rate (PowerDown = not batched).
  Odr gyroBatch =
      Odr::Hz104; ///< Gyroscope batch rate (PowerDown = not batched).
  std::uint16_t watermark = 64; ///< Threshold in FIFO words (1..511).
  bool routeWatermarkToInt1 =
      true; ///< Raise INT1 when the watermark is reached.
};

/** @brief FIFO status snapshot. */
struct FifoStatus {
  std::uint16_t unread = 0; ///< Number of unread 7-byte words.
  bool watermark = false;   ///< Watermark reached.
  bool overrun = false;     ///< FIFO overrun occurred (data lost).
  bool full = false;        ///< FIFO full.
};

/** @brief Sensor type carried by a FIFO word. */
enum class FifoTag : std::uint8_t {
  Gyro = 0x01,
  Accel = 0x02,
  Temperature = 0x03,
  Timestamp = 0x04,
  Unknown = 0xFF,
};

/** @brief One decoded FIFO word. */
struct FifoWord {
  FifoTag tag = FifoTag::Unknown;
  std::array<std::int16_t, 3>
      xyz{}; ///< Raw X, Y, Z (little-endian from the device).
};

/** @brief Blocking delay provided by the platform (e.g. wraps vTaskDelay). */
using DelayFn = void (*)(std::uint32_t ms);

/** @brief Step of Driver::init(); on failure tells where it stopped. */
enum class InitStep : std::uint8_t {
  WhoAmI = 0,     ///< Reading WHO_AM_I.
  ResetWrite = 1, ///< Writing SW_RESET.
  ResetPoll = 2,  ///< Waiting for the reset to finish.
  Finalize = 3,   ///< Writing BDU / IF_INC.
  Done = 4,       ///< init() succeeded.
};

/** @brief LSM6DSOX driver. Does not own the bus, never allocates, never throws.
 */
class Driver {
public:
  /**
   * @param bus   Register bus connected to the sensor; must outlive the driver.
   * @param delay Optional delay used while waiting for the sensor to reboot
   * after reset. Without it the reset is polled back-to-back.
   */
  explicit Driver(hal::IBus &bus, DelayFn delay = nullptr)
      : bus_(bus), delay_(delay) {}

  /**
   * @brief Verify the device, reset it and enable BDU and register
   * auto-increment.
   *
   * The sensor may NACK for a short time while it reboots after SW_RESET, so
   * bus errors during the reset poll are retried; only the last one is
   * reported.
   *
   * @return Ok, ErrDevice (wrong WHO_AM_I), ErrTimeout (reset did not finish)
   * or ErrBus.
   */
  hal::Status init();

  /** @brief Last step reached by init() (InitStep::Done after success). */
  InitStep failedStep() const { return step_; }

  /** @brief Set ODR and full scale of both sensors. */
  hal::Status configure(const Config &cfg);

  /**
   * @brief Configure FIFO in continuous mode and optionally route the watermark
   * to INT1.
   * @return Ok, ErrArg (watermark out of range) or ErrBus.
   */
  hal::Status configureFifo(const FifoConfig &cfg);

  /** @brief Read FIFO fill level and flags. */
  hal::Status readFifoStatus(FifoStatus &out);

  /** @brief Pop the oldest FIFO word (one 7-byte transaction). */
  hal::Status readFifoWord(FifoWord &out);

  /** @brief Accelerometer sensitivity of the current range, in mg/LSB. */
  float accelSensitivityMgPerLsb() const;

  /** @brief Gyroscope sensitivity of the current range, in mdps/LSB. */
  float gyroSensitivityMdpsPerLsb() const;

private:
  hal::IBus &bus_;
  DelayFn delay_ = nullptr;
  InitStep step_ = InitStep::WhoAmI;
  AccelRange accelRange_ = AccelRange::G4;
  GyroRange gyroRange_ = GyroRange::Dps500;
};

} // namespace lsm6dsox