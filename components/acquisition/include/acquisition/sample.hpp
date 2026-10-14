/**
 * @file sample.hpp
 * @brief One paired accelerometer + gyroscope sample.
 */
#pragma once

#include <array>
#include <cstdint>

namespace acquisition {

/**
 * @brief Raw IMU sample (sensor LSBs, scale with Driver::*Sensitivity*()).
 *
 * `seq` increments by one per sample, so a gap in `seq` means lost data.
 * Time of a sample is `seq / ODR` until hardware timestamps are added.
 */
struct Sample {
    std::uint32_t seq = 0;
    std::array<std::int16_t, 3> accel{};
    std::array<std::int16_t, 3> gyro{};
};

} // namespace acquisition