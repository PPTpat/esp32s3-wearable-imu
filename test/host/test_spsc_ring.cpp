#include <cstdint>
#include <thread>

#include "unity.h"
#include "util/spsc_ring.hpp"

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static void test_push_pop_fifo_order()
{
    util::SpscRing<int, 4> q;
    for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_TRUE(q.push(i));
    }
    for (int i = 0; i < 3; ++i) {
        int v = -1;
        TEST_ASSERT_TRUE(q.pop(v));
        TEST_ASSERT_EQUAL_INT(i, v);
    }
}

static void test_pop_empty_fails()
{
    util::SpscRing<int, 4> q;
    int v = 0;
    TEST_ASSERT_FALSE(q.pop(v));
}

static void test_push_full_fails_and_uses_full_capacity()
{
    util::SpscRing<int, 4> q;
    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_TRUE(q.push(i));
    }
    TEST_ASSERT_FALSE(q.push(99));
    TEST_ASSERT_EQUAL_UINT(4, q.size());
}

static void test_wraparound()
{
    util::SpscRing<int, 4> q;
    int v = 0;
    for (int i = 0; i < 100; ++i) {
        TEST_ASSERT_TRUE(q.push(i));
        TEST_ASSERT_TRUE(q.pop(v));
        TEST_ASSERT_EQUAL_INT(i, v);
    }
    TEST_ASSERT_EQUAL_UINT(0, q.size());
}

static void test_two_threads_preserve_order()
{
    static util::SpscRing<std::uint32_t, 64> q;
    constexpr std::uint32_t kCount = 500000;

    std::thread producer([] {
        for (std::uint32_t i = 0; i < kCount;) {
            if (q.push(i)) {
                ++i;
            }
        }
    });

    std::uint32_t expected = 0;
    bool ordered = true;
    while (expected < kCount) {
        std::uint32_t v = 0;
        if (q.pop(v)) {
            ordered = ordered && (v == expected);
            ++expected;
        }
    }
    producer.join();
    TEST_ASSERT_TRUE(ordered);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_push_pop_fifo_order);
    RUN_TEST(test_pop_empty_fails);
    RUN_TEST(test_push_full_fails_and_uses_full_capacity);
    RUN_TEST(test_wraparound);
    RUN_TEST(test_two_threads_preserve_order);
    return UNITY_END();
}