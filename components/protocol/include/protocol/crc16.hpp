/**
 * @file crc16.hpp
 * @brief CRC-16/CCITT-FALSE used to protect BLE frames.
 */
#pragma once

#include <cstdint>
#include <span>

namespace protocol {

/**
 * @brief Compute CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR).
 *
 * Check value: crc16("123456789") == 0x29B1.
 *
 * @param data Bytes to checksum.
 * @param init Initial value; pass a previous result to continue a running checksum.
 * @return The 16-bit checksum.
 */
std::uint16_t crc16(std::span<const std::uint8_t> data, std::uint16_t init = 0xFFFF);

} // namespace protocol