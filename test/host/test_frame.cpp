#include <array>
#include <vector>

#include "protocol/crc16.hpp"
#include "protocol/frame.hpp"
#include "unity.h"

using acquisition::Sample;
using namespace protocol;

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

/** Frame built independently in Python (tools/test_protocol.py uses the same
 * bytes). */
static const std::array<std::uint8_t, 36> kGolden = {
    0x01, 0x01, 0x02, 0x01, 0x64, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00,
    0xfe, 0xff, 0x03, 0x00, 0xfc, 0xff, 0x05, 0x00, 0xfa, 0xff, 0x00, 0x01,
    0x00, 0xff, 0xff, 0x7f, 0x00, 0x80, 0x00, 0x00, 0x07, 0x00, 0xf7, 0x10};

static std::array<Sample, 2> goldenSamples() {
  std::array<Sample, 2> s{};
  s[0].seq = 100;
  s[0].accel = {1, -2, 3};
  s[0].gyro = {-4, 5, -6};
  s[1].seq = 101;
  s[1].accel = {256, -256, 32767};
  s[1].gyro = {-32768, 0, 7};
  return s;
}

static void test_encode_matches_golden_bytes() {
  std::array<std::uint8_t, kMaxFrameBytes> buf{};
  const auto s = goldenSamples();
  const auto n = encodeSamplesFrame(buf, 0x0102, s);
  TEST_ASSERT_EQUAL_UINT(kGolden.size(), n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(kGolden.data(), buf.data(), n);
}

static void test_decode_golden_frame() {
  FrameHeader hdr;
  std::array<Sample, kMaxSamplesPerFrame> out{};
  TEST_ASSERT_EQUAL(DecodeStatus::Ok, decodeSamplesFrame(kGolden, hdr, out));
  TEST_ASSERT_EQUAL_UINT16(0x0102, hdr.frameSeq);
  TEST_ASSERT_EQUAL_UINT32(100, hdr.firstSampleSeq);
  TEST_ASSERT_EQUAL_UINT8(2, hdr.count);
  TEST_ASSERT_EQUAL_UINT32(101, out[1].seq);
  TEST_ASSERT_EQUAL_INT16(-32768, out[1].gyro[0]);
  TEST_ASSERT_EQUAL_INT16(32767, out[1].accel[2]);
}

static void test_roundtrip_max_frame() {
  std::array<Sample, kMaxSamplesPerFrame> in{};
  for (std::size_t i = 0; i < in.size(); ++i) {
    in[i].seq =
        0xFFFFFFF0U + static_cast<std::uint32_t>(i); // crosses the 32-bit wrap
    in[i].accel = {static_cast<std::int16_t>(i), 0, 0};
  }
  std::array<std::uint8_t, kMaxFrameBytes> buf{};
  const auto n = encodeSamplesFrame(buf, 7, in);
  TEST_ASSERT_EQUAL_UINT(kMaxFrameBytes, n);

  FrameHeader hdr;
  std::array<Sample, kMaxSamplesPerFrame> out{};
  TEST_ASSERT_EQUAL(DecodeStatus::Ok,
                    decodeSamplesFrame(
                        std::span<const std::uint8_t>(buf).first(n), hdr, out));
  TEST_ASSERT_EQUAL_UINT32(in[15].seq, out[15].seq);
  TEST_ASSERT_EQUAL_INT16(15, out[15].accel[0]);
}

static void test_encode_rejects_bad_input() {
  std::array<std::uint8_t, kMaxFrameBytes> buf{};
  auto s = goldenSamples();
  TEST_ASSERT_EQUAL_UINT(0,
                         encodeSamplesFrame(buf, 0, std::span<const Sample>{}));
  s[1].seq = 105; // gap
  TEST_ASSERT_EQUAL_UINT(0, encodeSamplesFrame(buf, 0, s));
  s = goldenSamples();
  std::array<std::uint8_t, 10> small{};
  TEST_ASSERT_EQUAL_UINT(0, encodeSamplesFrame(small, 0, s));
  std::array<Sample, kMaxSamplesPerFrame + 1> tooMany{};
  TEST_ASSERT_EQUAL_UINT(0, encodeSamplesFrame(buf, 0, tooMany));
}

static void test_decode_detects_corruption() {
  FrameHeader hdr;
  std::array<Sample, kMaxSamplesPerFrame> out{};
  std::vector<std::uint8_t> bad(kGolden.begin(), kGolden.end());
  bad[14] ^= 0x01;
  TEST_ASSERT_EQUAL(DecodeStatus::BadCrc, decodeSamplesFrame(bad, hdr, out));
  TEST_ASSERT_EQUAL(
      DecodeStatus::TooShort,
      decodeSamplesFrame(std::span<const std::uint8_t>(kGolden).first(5), hdr,
                         out));
}

static void test_decode_rejects_wrong_version() {
  FrameHeader hdr;
  std::array<Sample, kMaxSamplesPerFrame> out{};
  std::vector<std::uint8_t> f(kGolden.begin(), kGolden.end());
  f[0] = 2; // unsupported version, but with a valid CRC
  const auto crc = crc16(std::span<const std::uint8_t>(f).first(f.size() - 2));
  f[f.size() - 2] = static_cast<std::uint8_t>(crc & 0xFF);
  f[f.size() - 1] = static_cast<std::uint8_t>(crc >> 8);
  TEST_ASSERT_EQUAL(DecodeStatus::BadVersion, decodeSamplesFrame(f, hdr, out));
}

static void test_decode_output_too_small() {
  FrameHeader hdr;
  std::array<Sample, 1> out{};
  TEST_ASSERT_EQUAL(DecodeStatus::OutputTooSmall,
                    decodeSamplesFrame(kGolden, hdr, out));
}

static void test_max_samples_for_payload() {
  TEST_ASSERT_EQUAL_UINT(0, maxSamplesForPayload(20)); // default MTU 23
  TEST_ASSERT_EQUAL_UINT(0, maxSamplesForPayload(23));
  TEST_ASSERT_EQUAL_UINT(1, maxSamplesForPayload(24));
  TEST_ASSERT_EQUAL_UINT(2, maxSamplesForPayload(36));
  TEST_ASSERT_EQUAL_UINT(16, maxSamplesForPayload(244)); // MTU 247
}

static void test_info_roundtrip() {
  DeviceInfo in{104, 0.122F, 17.5F};
  std::array<std::uint8_t, kInfoBytes> buf{};
  TEST_ASSERT_EQUAL_UINT(kInfoBytes, encodeInfo(buf, in));
  DeviceInfo out;
  TEST_ASSERT_TRUE(decodeInfo(buf, out));
  TEST_ASSERT_EQUAL_UINT16(104, out.odrHz);
  TEST_ASSERT_EQUAL_FLOAT(0.122F, out.accelMgPerLsb);
  TEST_ASSERT_EQUAL_FLOAT(17.5F, out.gyroMdpsPerLsb);
}

static void test_parse_command() {
  const std::array<std::uint8_t, 1> start{0x01};
  const std::array<std::uint8_t, 1> stop{0x02};
  const std::array<std::uint8_t, 1> junk{0x7F};
  const std::array<std::uint8_t, 2> longer{0x01, 0x00};
  TEST_ASSERT_TRUE(parseCommand(start) == Command::Start);
  TEST_ASSERT_TRUE(parseCommand(stop) == Command::Stop);
  TEST_ASSERT_FALSE(parseCommand(junk).has_value());
  TEST_ASSERT_FALSE(parseCommand(longer).has_value());
  TEST_ASSERT_FALSE(parseCommand(std::span<const std::uint8_t>{}).has_value());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_encode_matches_golden_bytes);
  RUN_TEST(test_decode_golden_frame);
  RUN_TEST(test_roundtrip_max_frame);
  RUN_TEST(test_encode_rejects_bad_input);
  RUN_TEST(test_decode_detects_corruption);
  RUN_TEST(test_decode_rejects_wrong_version);
  RUN_TEST(test_decode_output_too_small);
  RUN_TEST(test_max_samples_for_payload);
  RUN_TEST(test_info_roundtrip);
  RUN_TEST(test_parse_command);
  return UNITY_END();
}