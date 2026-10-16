#include <array>

#include "protocol/crc16.hpp"
#include "unity.h"

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static void test_check_value()
{
    const std::array<std::uint8_t, 9> data{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, protocol::crc16(data));
}

static void test_empty_returns_init()
{
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, protocol::crc16(std::span<const std::uint8_t>{}));
}

static void test_running_checksum_matches_one_shot()
{
    const std::array<std::uint8_t, 6> data{1, 2, 3, 4, 5, 6};
    const auto part = protocol::crc16(std::span<const std::uint8_t>(data).first(3));
    const auto whole = protocol::crc16(std::span<const std::uint8_t>(data).last(3), part);
    TEST_ASSERT_EQUAL_HEX16(protocol::crc16(data), whole);
}

static void test_single_bit_flip_is_detected()
{
    std::array<std::uint8_t, 4> data{0xDE, 0xAD, 0xBE, 0xEF};
    const auto good = protocol::crc16(data);
    data[2] ^= 0x01;
    TEST_ASSERT_NOT_EQUAL(good, protocol::crc16(data));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_check_value);
    RUN_TEST(test_empty_returns_init);
    RUN_TEST(test_running_checksum_matches_one_shot);
    RUN_TEST(test_single_bit_flip_is_detected);
    return UNITY_END();
}