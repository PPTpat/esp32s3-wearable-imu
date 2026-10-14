/**
 * @file acquisition.hpp
 * @brief Interrupt-driven FIFO acquisition (ESP-IDF only).
 *
 * Data path:  INT1 edge -> ISR (notify only) -> acquisition task -> FIFO drain
 *             over I2C -> SampleAssembler -> SPSC queue -> consumer task.
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "acquisition/sample.hpp"
#include "acquisition/sample_assembler.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lsm6dsox/lsm6dsox.hpp"
#include "util/spsc_ring.hpp"

namespace acquisition {

/** @brief Capacity of the sample queue between acquisition and consumers. */
constexpr std::size_t kQueueCapacity = 256;

/** @brief Queue carrying samples from the acquisition task to the consumer. */
using SampleQueue = util::SpscRing<Sample, kQueueCapacity>;

/** @brief Acquisition settings. */
struct Config {
    gpio_num_t intPin = GPIO_NUM_NC;     ///< GPIO wired to sensor INT1.
    UBaseType_t taskPriority = 10;       ///< Priority of the acquisition task.
    std::uint32_t taskStackBytes = 4096; ///< Stack size of the acquisition task.
};

/** @brief Runtime counters. */
struct Stats {
    std::uint32_t samples = 0;             ///< Samples pushed to the queue.
    std::uint32_t queueDrops = 0;          ///< Samples lost because the queue was full.
    std::uint32_t fifoOverruns = 0;        ///< FIFO overrun flag observed (sensor lost data).
    std::uint32_t busErrors = 0;           ///< Failed I2C transactions.
    std::uint32_t unpairedWords = 0;       ///< FIFO words that never found their pair.
    std::uint32_t stackHighWaterBytes = 0; ///< Minimum free stack of the task so far.
};

/** @brief Owns the acquisition task and the INT1 interrupt. */
class Acquisition {
  public:
    /**
     * @param driver Initialised and configured sensor driver; must outlive this object.
     * @param queue  Destination queue; must outlive this object.
     */
    Acquisition(lsm6dsox::Driver &driver, SampleQueue &queue) : driver_(driver), queue_(queue) {}

    /**
     * @brief Create the task and enable the INT1 interrupt.
     * @return ESP_OK on success, an esp_err_t otherwise.
     */
    esp_err_t start(const Config &cfg);

    /** @brief Snapshot of the counters (safe to call from any task). */
    Stats stats() const;

  private:
    static void isr(void *arg);
    static void taskEntry(void *arg);
    void run();
    void drainFifo();

    lsm6dsox::Driver &driver_;
    SampleQueue &queue_;
    SampleAssembler assembler_;
    TaskHandle_t task_ = nullptr;

    std::atomic<std::uint32_t> samples_{0};
    std::atomic<std::uint32_t> queueDrops_{0};
    std::atomic<std::uint32_t> fifoOverruns_{0};
    std::atomic<std::uint32_t> busErrors_{0};
    std::atomic<std::uint32_t> unpaired_{0};
};

} // namespace acquisition