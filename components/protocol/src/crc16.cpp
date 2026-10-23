#include "protocol/crc16.hpp"

#include <array>

namespace protocol {

namespace {

constexpr std::array<std::uint16_t, 256> makeTable() {
  std::array<std::uint16_t, 256> table{};
  for (unsigned i = 0; i < table.size(); ++i) {
    unsigned crc = i << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = ((crc & 0x8000U) != 0) ? ((crc << 1) ^ 0x1021U) : (crc << 1);
    }
    table[i] = static_cast<std::uint16_t>(crc & 0xFFFFU);
  }
  return table;
}

constexpr auto kTable = makeTable(); // built at compile time, 512 B of flash

} // namespace

std::uint16_t crc16(std::span<const std::uint8_t> data, std::uint16_t init) {
  unsigned crc = init;
  for (const std::uint8_t byte : data) {
    crc = ((crc << 8) ^ kTable[((crc >> 8) ^ byte) & 0xFFU]) & 0xFFFFU;
  }
  return static_cast<std::uint16_t>(crc);
}

} // namespace protocol