#include "acquisition/sample_assembler.hpp"
#include "unity.h"

using acquisition::Sample;
using acquisition::SampleAssembler;
using lsm6dsox::FifoTag;
using lsm6dsox::FifoWord;

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static FifoWord word(FifoTag tag, std::int16_t x, std::int16_t y, std::int16_t z)
{
    FifoWord w;
    w.tag = tag;
    w.xyz = {x, y, z};
    return w;
}

static void test_gyro_then_accel_emits_sample()
{
    SampleAssembler a;
    Sample s;
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Gyro, 1, 2, 3), s));
    TEST_ASSERT_TRUE(a.push(word(FifoTag::Accel, 4, 5, 6), s));
    TEST_ASSERT_EQUAL_INT16(4, s.accel[0]);
    TEST_ASSERT_EQUAL_INT16(6, s.accel[2]);
    TEST_ASSERT_EQUAL_INT16(1, s.gyro[0]);
    TEST_ASSERT_EQUAL_INT16(3, s.gyro[2]);
}

static void test_accel_then_gyro_emits_sample()
{
    SampleAssembler a;
    Sample s;
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Accel, 4, 5, 6), s));
    TEST_ASSERT_TRUE(a.push(word(FifoTag::Gyro, 1, 2, 3), s));
    TEST_ASSERT_EQUAL_INT16(5, s.accel[1]);
    TEST_ASSERT_EQUAL_INT16(2, s.gyro[1]);
}

static void test_sequence_increments()
{
    SampleAssembler a;
    Sample s;
    for (std::uint32_t i = 0; i < 3; ++i) {
        TEST_ASSERT_FALSE(a.push(word(FifoTag::Gyro, 0, 0, 0), s));
        TEST_ASSERT_TRUE(a.push(word(FifoTag::Accel, 0, 0, 0), s));
        TEST_ASSERT_EQUAL_UINT32(i, s.seq);
    }
}

static void test_repeated_kind_counts_unpaired_and_keeps_latest()
{
    SampleAssembler a;
    Sample s;
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Accel, 1, 1, 1), s));
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Accel, 9, 9, 9), s));
    TEST_ASSERT_EQUAL_UINT32(1, a.unpaired());
    TEST_ASSERT_TRUE(a.push(word(FifoTag::Gyro, 2, 2, 2), s));
    TEST_ASSERT_EQUAL_INT16(9, s.accel[0]);
}

static void test_other_tags_are_ignored()
{
    SampleAssembler a;
    Sample s;
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Temperature, 1, 1, 1), s));
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Timestamp, 1, 1, 1), s));
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Unknown, 1, 1, 1), s));
    TEST_ASSERT_EQUAL_UINT32(0, a.unpaired());
}

static void test_reset_drops_partial_pair()
{
    SampleAssembler a;
    Sample s;
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Gyro, 1, 1, 1), s));
    a.reset();
    TEST_ASSERT_FALSE(a.push(word(FifoTag::Accel, 2, 2, 2), s));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_gyro_then_accel_emits_sample);
    RUN_TEST(test_accel_then_gyro_emits_sample);
    RUN_TEST(test_sequence_increments);
    RUN_TEST(test_repeated_kind_counts_unpaired_and_keeps_latest);
    RUN_TEST(test_other_tags_are_ignored);
    RUN_TEST(test_reset_drops_partial_pair);
    return UNITY_END();
}