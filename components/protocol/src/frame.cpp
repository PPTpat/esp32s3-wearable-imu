#include "protocol/frame.hpp"

#include <bit>

#include "protocol/crc16.hpp"

namespace protocol {

namespace {

/** @brief Little-endian writer over a pre-checked buffer. */
class Writer {
public:
  explicit Writer(std::span<std::uint8_t> buf) : buf_(buf) {}
  void u8(std::uint8_t v) { buf_[pos_++] = v; }
  void u16(std::uint16_t v) {
    u8(static_cast<std::uint8_t>(v & 0xFFU));
    u8(static_cast<std::uint8_t>(v >> 8));
  }
  void u32(std::uint32_t v) {
    u16(static_cast<std::uint16_t>(v & 0xFFFFU));
    u16(static_cast<std::uint16_t>(v >> 16));
  }
  void i16(std::int16_t v) { u16(static_cast<std::uint16_t>(v)); }
  std::size_t pos() const { return pos_; }

private:
  std::span<std::uint8_t> buf_;
  std::size_t pos_ = 0;
};

/** @brief Little-endian reader over a pre-checked buffer. */
class Reader {
public:
  explicit Reader(std::span<const std::uint8_t> buf) : buf_(buf) {}
  std::uint8_t u8() { return buf_[pos_++]; }
  std::uint16_t u16() {
    const unsigned lo = u8();
    const unsigned hi = u8();
    return static_cast<std::uint16_t>(lo | (hi << 8));
  }
  std::uint32_t u32() {
    const std::uint32_t lo = u16();
    const std::uint32_t hi = u16();
    return lo | (hi << 16);
  }
  std::int16_t i16() { return static_cast<std::int16_t>(u16()); }

private:
  std::span<const std::uint8_t> buf_;
  std::size_t pos_ = 0;
};

} // namespace

std::size_t encodeSamplesFrame(std::span<std::uint8_t> out,
                               std::uint16_t frameSeq,
                               std::span<const acquisition::Sample> samples) {
  if (samples.empty() || samples.size() > kMaxSamplesPerFrame) {
    return 0;
  }
  const std::size_t total = kFrameOverhead + samples.size() * kSampleBytes;
  if (out.size() < total) {
    return 0;
  }
  for (std::size_t i = 1; i < samples.size(); ++i) {
    if (samples[i].seq != samples[i - 1].seq + 1U) {
      return 0;
    }
  }

  Writer w(out);
  w.u8(kProtocolVersion);
  w.u8(static_cast<std::uint8_t>(FrameType::Samples));
  w.u16(frameSeq);
  w.u32(samples[0].seq);
  w.u8(static_cast<std::uint8_t>(samples.size()));
  w.u8(0);
  for (const auto &s : samples) {
    for (const auto v : s.accel) {
      w.i16(v);
    }
    for (const auto v : s.gyro) {
      w.i16(v);
    }
  }
  w.u16(crc16(out.first(w.pos())));
  return w.pos();
}

DecodeStatus decodeSamplesFrame(std::span<const std::uint8_t> in,
                                FrameHeader &hdr,
                                std::span<acquisition::Sample> out) {
  if (in.size() < kFrameOverhead) {
    return DecodeStatus::TooShort;
  }
  const auto body = in.first(in.size() - kCrcBytes);
  Reader crcReader(in.last(kCrcBytes));
  if (crc16(body) != crcReader.u16()) {
    return DecodeStatus::BadCrc;
  }

  Reader r(body);
  hdr.version = r.u8();
  if (hdr.version != kProtocolVersion) {
    return DecodeStatus::BadVersion;
  }
  const std::uint8_t type = r.u8();
  if (type != static_cast<std::uint8_t>(FrameType::Samples)) {
    return DecodeStatus::BadType;
  }
  hdr.type = FrameType::Samples;
  hdr.frameSeq = r.u16();
  hdr.firstSampleSeq = r.u32();
  hdr.count = r.u8();
  r.u8(); // reserved

  if (hdr.count == 0 || hdr.count > kMaxSamplesPerFrame ||
      in.size() !=
          kFrameOverhead + static_cast<std::size_t>(hdr.count) * kSampleBytes) {
    return DecodeStatus::BadLength;
  }
  if (out.size() < hdr.count) {
    return DecodeStatus::OutputTooSmall;
  }
  for (std::size_t i = 0; i < hdr.count; ++i) {
    auto &s = out[i];
    s.seq = hdr.firstSampleSeq + static_cast<std::uint32_t>(i);
    for (auto &v : s.accel) {
      v = r.i16();
    }
    for (auto &v : s.gyro) {
      v = r.i16();
    }
  }
  return DecodeStatus::Ok;
}

std::size_t encodeInfo(std::span<std::uint8_t> out, const DeviceInfo &info) {
  if (out.size() < kInfoBytes) {
    return 0;
  }
  Writer w(out);
  w.u8(kProtocolVersion);
  w.u8(0);
  w.u16(info.odrHz);
  w.u32(std::bit_cast<std::uint32_t>(info.accelMgPerLsb));
  w.u32(std::bit_cast<std::uint32_t>(info.gyroMdpsPerLsb));
  return w.pos();
}

bool decodeInfo(std::span<const std::uint8_t> in, DeviceInfo &info) {
  if (in.size() != kInfoBytes) {
    return false;
  }
  Reader r(in);
  if (r.u8() != kProtocolVersion) {
    return false;
  }
  r.u8(); // reserved
  info.odrHz = r.u16();
  info.accelMgPerLsb = std::bit_cast<float>(r.u32());
  info.gyroMdpsPerLsb = std::bit_cast<float>(r.u32());
  return true;
}

std::optional<Command> parseCommand(std::span<const std::uint8_t> in) {
  if (in.size() != 1) {
    return std::nullopt;
  }
  switch (in[0]) {
  case static_cast<std::uint8_t>(Command::Start):
    return Command::Start;
  case static_cast<std::uint8_t>(Command::Stop):
    return Command::Stop;
  default:
    return std::nullopt;
  }
}

} // namespace protocol