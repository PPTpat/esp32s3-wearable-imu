/**
 * @file frame.hpp
 * @brief Binary wire format: sample frames, device info and commands.
 *
 * All multi-byte fields are little-endian. See docs/ble_protocol.md.
 *
 * Sample frame (notified on the data characteristic):
 * | offset | size | field                                    |
 * |--------|------|------------------------------------------|
 * | 0      | 1    | version (1)                              |
 * | 1      | 1    | type (0x01 = samples)                    |
 * | 2      | 2    | frame_seq (wraps at 65536)               |
 * | 4      | 4    | first_sample_seq                         |
 * | 8      | 1    | count (1..16)                            |
 * | 9      | 1    | reserved (0)                             |
 * | 10     | 12*n | samples: ax ay az gx gy gz (int16 each)  |
 * | 10+12n | 2    | CRC-16/CCITT-FALSE of all bytes before it|
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "acquisition/sample.hpp"

namespace protocol {

constexpr std::uint8_t kProtocolVersion = 1;

/** @brief Frame type byte. */
enum class FrameType : std::uint8_t { Samples = 0x01 };

constexpr std::size_t kHeaderBytes = 10;
constexpr std::size_t kSampleBytes = 12;
constexpr std::size_t kCrcBytes = 2;
constexpr std::size_t kFrameOverhead = kHeaderBytes + kCrcBytes;
constexpr std::size_t kMaxSamplesPerFrame = 16;
constexpr std::size_t kMaxFrameBytes =
    kFrameOverhead + kMaxSamplesPerFrame * kSampleBytes;

/**
 * @brief How many samples fit in one notification.
 * @param payloadBytes Usable ATT payload (negotiated MTU - 3).
 * @return 0 if not even one sample fits, otherwise at most kMaxSamplesPerFrame.
 */
constexpr std::size_t maxSamplesForPayload(std::size_t payloadBytes) {
  if (payloadBytes < kFrameOverhead + kSampleBytes) {
    return 0;
  }
  const std::size_t n = (payloadBytes - kFrameOverhead) / kSampleBytes;
  return n < kMaxSamplesPerFrame ? n : kMaxSamplesPerFrame;
}

/** @brief Decoded frame header. */
struct FrameHeader {
  std::uint8_t version = 0;
  FrameType type = FrameType::Samples;
  std::uint16_t frameSeq = 0;
  std::uint32_t firstSampleSeq = 0;
  std::uint8_t count = 0;
};

/**
 * @brief Serialise consecutive samples into one frame.
 *
 * Sample i is stored without its own `seq`; the receiver derives it as
 * `first_sample_seq + i`, so samples must have consecutive `seq` values.
 *
 * @param out       Destination buffer (kMaxFrameBytes is always enough).
 * @param frameSeq  Frame counter value to store.
 * @param samples   1..kMaxSamplesPerFrame samples with consecutive seq.
 * @return Number of bytes written, or 0 if the input is empty, too long,
 *         not consecutive, or @p out is too small.
 */
std::size_t encodeSamplesFrame(std::span<std::uint8_t> out,
                               std::uint16_t frameSeq,
                               std::span<const acquisition::Sample> samples);

/** @brief Result of decodeSamplesFrame(). */
enum class DecodeStatus : std::uint8_t {
  Ok,
  TooShort,   ///< Fewer bytes than header + CRC.
  BadCrc,     ///< Checksum mismatch.
  BadVersion, ///< Unsupported protocol version.
  BadType,    ///< Unknown frame type.
  BadLength,  ///< count disagrees with the frame size or exceeds the maximum.
  OutputTooSmall, ///< @p out cannot hold `count` samples.
};

/**
 * @brief Validate and parse a sample frame.
 * @param in   Raw frame bytes.
 * @param hdr  Receives the header (valid when Ok).
 * @param out  Receives `hdr.count` samples with `seq` reconstructed.
 */
DecodeStatus decodeSamplesFrame(std::span<const std::uint8_t> in,
                                FrameHeader &hdr,
                                std::span<acquisition::Sample> out);

/** @brief Static device description, readable once after connecting. */
struct DeviceInfo {
  std::uint16_t odrHz = 0;     ///< Sample rate of the stream.
  float accelMgPerLsb = 0.0F;  ///< Accelerometer scale.
  float gyroMdpsPerLsb = 0.0F; ///< Gyroscope scale.
};

constexpr std::size_t kInfoBytes = 12;

/**
 * @brief Serialise DeviceInfo: version u8, reserved u8, odr_hz u16, accel f32,
 * gyro f32.
 * @return kInfoBytes, or 0 if @p out is too small.
 */
std::size_t encodeInfo(std::span<std::uint8_t> out, const DeviceInfo &info);

/** @brief Parse DeviceInfo. @return false if the size or version is wrong. */
bool decodeInfo(std::span<const std::uint8_t> in, DeviceInfo &info);

/** @brief Commands written to the control characteristic (one byte). */
enum class Command : std::uint8_t {
  Start = 0x01, ///< Begin streaming samples.
  Stop = 0x02,  ///< Stop streaming.
};

/** @brief Parse a control write. @return the command, or nothing if invalid. */
std::optional<Command> parseCommand(std::span<const std::uint8_t> in);

} // namespace protocol