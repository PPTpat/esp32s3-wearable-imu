#include "acquisition/acquisition.hpp"

#include "esp_attr.h"
#include "esp_log.h"

namespace acquisition {

namespace {
constexpr const char *kTag = "acq";
constexpr TickType_t kIdleTimeoutTicks = pdMS_TO_TICKS(1000);
} // namespace

// Keep the ISR minimal: it only wakes the task. All bus traffic happens in task context.
void IRAM_ATTR Acquisition::isr(void *arg)
{
    auto *self = static_cast<Acquisition *>(arg);
    self->interrupts_.store(self->interrupts_.load(std::memory_order_relaxed) + 1U, std::memory_order_relaxed);
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(self->task_, &woken);
    portYIELD_FROM_ISR(woken);
}

void Acquisition::taskEntry(void *arg)
{
    static_cast<Acquisition *>(arg)->run();
}

esp_err_t Acquisition::start(const Config &cfg)
{
    if (cfg.intPin == GPIO_NUM_NC || task_ != nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xTaskCreate(taskEntry, "acq", cfg.taskStackBytes, this, cfg.taskPriority, &task_) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io{};
    io.pin_bit_mask = 1ULL << static_cast<unsigned>(cfg.intPin);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_POSEDGE;
    if (const esp_err_t err = gpio_config(&io); err != ESP_OK) {
        return err;
    }

    // The ISR service may already be installed by another component.
    const esp_err_t svc = gpio_install_isr_service(0);
    if (svc != ESP_OK && svc != ESP_ERR_INVALID_STATE) {
        return svc;
    }
    if (const esp_err_t err = gpio_isr_handler_add(cfg.intPin, isr, this); err != ESP_OK) {
        return err;
    }

    // INT1 may already be high (no edge to catch), so kick the task once.
    xTaskNotifyGive(task_);
    ESP_LOGI(kTag, "started, INT1 on GPIO%d", static_cast<int>(cfg.intPin));
    return ESP_OK;
}

void Acquisition::run()
{
    for (;;) {
        // Also wake up on timeout so a missed edge cannot stall acquisition forever.
        ulTaskNotifyTake(pdTRUE, kIdleTimeoutTicks);
        drainFifo();
    }
}

void Acquisition::drainFifo()
{
    lsm6dsox::FifoStatus status;
    if (driver_.readFifoStatus(status) != hal::Status::Ok) {
        busErrors_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (status.overrun) {
        fifoOverruns_.fetch_add(1, std::memory_order_relaxed);
        assembler_.reset(); // the stream has a hole: do not pair words across it
    }

    for (std::uint16_t i = 0; i < status.unread; ++i) {
        lsm6dsox::FifoWord word;
        if (driver_.readFifoWord(word) != hal::Status::Ok) {
            busErrors_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        Sample sample;
        if (!assembler_.push(word, sample)) {
            continue;
        }
        if (queue_.push(sample)) {
            samples_.fetch_add(1, std::memory_order_relaxed);
        } else {
            queueDrops_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    unpaired_.store(assembler_.unpaired(), std::memory_order_relaxed);
}

Stats Acquisition::stats() const
{
    Stats s;
    s.interrupts = interrupts_.load(std::memory_order_relaxed);
    s.samples = samples_.load(std::memory_order_relaxed);
    s.queueDrops = queueDrops_.load(std::memory_order_relaxed);
    s.fifoOverruns = fifoOverruns_.load(std::memory_order_relaxed);
    s.busErrors = busErrors_.load(std::memory_order_relaxed);
    s.unpairedWords = unpaired_.load(std::memory_order_relaxed);
    s.stackHighWaterBytes = task_ != nullptr ? static_cast<std::uint32_t>(uxTaskGetStackHighWaterMark(task_)) : 0;
    return s;
}

} // namespace acquisition