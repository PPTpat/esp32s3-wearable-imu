/**
 * @file registers.hpp
 * @brief LSM6DSOX register map and bit fields (subset used by the driver).
 *
 * Source: ST DS12814, LSM6DSOX datasheet.
 */
#pragma once

#include <cstdint>

namespace lsm6dsox {

/** @brief 7-bit I2C addresses, selected by the SA0 pin. */
constexpr std::uint8_t kI2cAddrSa0Low = 0x6A;
constexpr std::uint8_t kI2cAddrSa0High = 0x6B;

/** @brief Expected WHO_AM_I value. */
constexpr std::uint8_t kWhoAmIValue = 0x6C;

/** @brief Register addresses. */
namespace reg {
constexpr std::uint8_t FIFO_CTRL1 = 0x07; ///< Watermark threshold, bits [7:0].
constexpr std::uint8_t FIFO_CTRL2 = 0x08; ///< Watermark bit 8, FIFO options.
constexpr std::uint8_t FIFO_CTRL3 = 0x09; ///< Batch data rates (gyro, accel).
constexpr std::uint8_t FIFO_CTRL4 =
    0x0A; ///< FIFO mode, timestamp/temperature batching.
constexpr std::uint8_t INT1_CTRL = 0x0D; ///< INT1 pin routing.
constexpr std::uint8_t WHO_AM_I = 0x0F;  ///< Device ID.
constexpr std::uint8_t CTRL1_XL = 0x10;  ///< Accelerometer ODR and full scale.
constexpr std::uint8_t CTRL2_G = 0x11;   ///< Gyroscope ODR and full scale.
constexpr std::uint8_t CTRL3_C = 0x12; ///< Reset, BDU, address auto-increment.
constexpr std::uint8_t FIFO_STATUS1 = 0x3A; ///< Unread words, bits [7:0].
constexpr std::uint8_t FIFO_STATUS2 =
    0x3B; ///< FIFO flags, unread words bits [9:8].
constexpr std::uint8_t FIFO_DATA_OUT_TAG =
    0x78; ///< Tag of the oldest FIFO word.
constexpr std::uint8_t FIFO_DATA_OUT_X_L =
    0x79; ///< First data byte of the oldest FIFO word.
} // namespace reg

/** @brief CTRL3_C bits. */
namespace ctrl3_c {
constexpr std::uint8_t SW_RESET = 1U << 0;
constexpr std::uint8_t IF_INC = 1U << 2;
constexpr std::uint8_t BDU = 1U << 6;
} // namespace ctrl3_c

/** @brief INT1_CTRL bits. */
namespace int1_ctrl {
constexpr std::uint8_t FIFO_TH = 1U << 3;
constexpr std::uint8_t FIFO_OVR = 1U << 4;
constexpr std::uint8_t FIFO_FULL = 1U << 5;
} // namespace int1_ctrl

/** @brief FIFO_STATUS2 bits. */
namespace fifo_status2 {
constexpr std::uint8_t DIFF_FIFO_HI_MASK = 0x03;
constexpr std::uint8_t FULL = 1U << 5;
constexpr std::uint8_t OVERRUN = 1U << 6;
constexpr std::uint8_t WATERMARK = 1U << 7;
} // namespace fifo_status2

/** @brief FIFO_CTRL2 bits. */
namespace fifo_ctrl2 {
constexpr std::uint8_t WTM8 = 1U << 0;
} // namespace fifo_ctrl2

/** @brief FIFO_CTRL4 FIFO_MODE field value: continuous (overwrites oldest
 * data). */
constexpr std::uint8_t kFifoModeContinuous = 0x06;

} // namespace lsm6dsox