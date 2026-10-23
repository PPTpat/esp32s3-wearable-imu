/**
 * @file spsc_ring.hpp
 * @brief Lock-free single-producer / single-consumer ring buffer.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace util {

/**
 * @brief Fixed-capacity SPSC queue; wait-free, no allocation, no locks.
 *
 * Exactly one thread may call push() and exactly one (other) thread may call
 * pop(). Indices are free-running counters, so the full capacity is usable.
 *
 * @tparam T Element type (must be trivially copyable for ISR/task use).
 * @tparam N Capacity; must be a power of two.
 */
template <typename T, std::size_t N> class SpscRing {
  static_assert(N >= 2 && (N & (N - 1)) == 0,
                "capacity must be a power of two");

public:
  /** @brief Number of elements the queue can hold. */
  static constexpr std::size_t capacity() { return N; }

  /**
   * @brief Append an element (producer side).
   * @return false if the queue is full (element not stored).
   */
  bool push(const T &value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail == N) {
      return false;
    }
    buf_[head & (N - 1)] = value;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  /**
   * @brief Remove the oldest element (consumer side).
   * @param[out] out Receives the element.
   * @return false if the queue is empty.
   */
  bool pop(T &out) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);
    if (head == tail) {
      return false;
    }
    out = buf_[tail & (N - 1)];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  /** @brief Approximate number of stored elements (exact when called from
   * either end). */
  std::size_t size() const {
    return head_.load(std::memory_order_acquire) -
           tail_.load(std::memory_order_acquire);
  }

private:
  std::array<T, N> buf_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};

} // namespace util