#include <array>

#include "bus_mock.hpp"
#include "unity.h"

static mocks::BusMock mock;

extern "C" void setUp(void) { mock = mocks::BusMock{}; }
extern "C" void tearDown(void) {}

static void test_write_then_read_u8() {
  TEST_ASSERT_EQUAL(hal::Status::Ok, mock.writeU8(0x10, 0xA5));
  std::uint8_t v = 0;
  TEST_ASSERT_EQUAL(hal::Status::Ok, mock.readU8(0x10, v));
  TEST_ASSERT_EQUAL_HEX8(0xA5, v);
}

static void test_burst_roundtrip() {
  const std::array<std::uint8_t, 4> in{1, 2, 3, 4};
  std::array<std::uint8_t, 4> out{};
  TEST_ASSERT_EQUAL(hal::Status::Ok, mock.write(0x20, in));
  TEST_ASSERT_EQUAL(hal::Status::Ok, mock.read(0x20, out));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(in.data(), out.data(), in.size());
}

static void test_out_of_range_rejected() {
  std::array<std::uint8_t, 8> buf{};
  TEST_ASSERT_EQUAL(hal::Status::ErrArg, mock.read(0xFC, buf));
}

static void test_empty_buffer_rejected() {
  TEST_ASSERT_EQUAL(hal::Status::ErrArg,
                    mock.read(0, std::span<std::uint8_t>{}));
}

static void test_fault_injection() {
  std::uint8_t v = 0;
  mock.failAfter(1);
  TEST_ASSERT_EQUAL(hal::Status::Ok, mock.readU8(0, v));
  TEST_ASSERT_EQUAL(hal::Status::ErrBus, mock.readU8(0, v));
  TEST_ASSERT_EQUAL_UINT(2, mock.readCalls);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_write_then_read_u8);
  RUN_TEST(test_burst_roundtrip);
  RUN_TEST(test_out_of_range_rejected);
  RUN_TEST(test_empty_buffer_rejected);
  RUN_TEST(test_fault_injection);
  return UNITY_END();
}