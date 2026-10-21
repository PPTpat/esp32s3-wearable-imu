/**
 * @file main.cpp
 * @brief Firmware entry point: IMU acquisition streamed over BLE.
 */
#include <cinttypes>

#include "acquisition/acquisition.hpp"
#include "ble_service/ble_service.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_bus.hpp"
#include "lsm6dsox/lsm6dsox.hpp"
#include "lsm6dsox/registers.hpp"
#include "nvs_flash.h"
#include "protocol/frame.hpp"

namespace {

constexpr const char *kTag = "app";
constexpr const char *kDeviceName = "ESP32S3-IMU";

// The advertised sample rate must match lsm6dsox::Config{} below.
constexpr std::uint16_t kOdrHz = 104;
static_assert(lsm6dsox::Config{}.accelOdr == lsm6dsox::Odr::Hz104 &&
                  lsm6dsox::Config{}.gyroOdr == lsm6dsox::Odr::Hz104,
              "kOdrHz must match the default sensor configuration");

// Static storage: no heap allocation in the data path.
/** @brief Blocking delay for the sensor driver. */
void delayMs(std::uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

hal::I2cBus gBus;
lsm6dsox::Driver gImu(gBus, delayMs);
acquisition::SampleQueue gQueue;
acquisition::Acquisition gAcq(gImu, gQueue);
ble_service::BleService gBle(gQueue); // the only consumer of gQueue

/** @brief Log an error and park the task; a real fault handler comes later. */
[[noreturn]] void fatal(const char *what)
{
    ESP_LOGE(kTag, "fatal: %s", what);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Measure how reliably the sensor ACKs: 20 probes, 10 ms apart.
 *
 * 20/20 means the bus is healthy; anything between 1 and 19 points to a loose
 * contact; 0 means nothing is listening at all.
 */
void measureBusHealth(std::uint8_t address)
{
    constexpr unsigned kProbes = 20;
    unsigned acks = 0;
    for (unsigned i = 0; i < kProbes; ++i) {
        if (gBus.probe(address) == hal::Status::Ok) {
            ++acks;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(kTag, "probe 0x%02X: ack %u/%u (last result: %s)", static_cast<unsigned>(address), acks, kProbes,
             esp_err_to_name(gBus.lastError()));
}

/** @brief Explain a failed sensor init: status codes, WHO_AM_I and every I2C address that answers. */
void diagnoseImu(hal::Status initStatus)
{
    ESP_LOGE(kTag, "imu init failed, status=%d (-1 bus/NACK, -3 timeout, -4 wrong WHO_AM_I), step=%d "
                   "(0 WHO_AM_I, 1 reset write, 2 reset poll, 3 finalize)",
             static_cast<int>(initStatus), static_cast<int>(gImu.failedStep()));

    std::uint8_t id = 0;
    const auto rd = gBus.readU8(lsm6dsox::reg::WHO_AM_I, id);
    if (rd == hal::Status::Ok) {
        ESP_LOGE(kTag, "WHO_AM_I = 0x%02X (expected 0x%02X)", static_cast<unsigned>(id),
                 static_cast<unsigned>(lsm6dsox::kWhoAmIValue));
    } else {
        // A failed read leaves garbage in the buffer, so no value is printed.
        ESP_LOGE(kTag, "WHO_AM_I read failed (esp_err=%s)", esp_err_to_name(gBus.lastError()));
    }

    unsigned found = 0;
    for (std::uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (gBus.probe(addr) == hal::Status::Ok) {
            ESP_LOGW(kTag, "I2C device answers at 0x%02X", static_cast<unsigned>(addr));
            ++found;
        }
    }
    if (found == 0) {
        ESP_LOGE(kTag, "no I2C device answers: check power, SDA/SCL, solder joints");
    }
}

void initNvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "wearable-imu booting");
    initNvs(); // required by the BLE stack

    hal::I2cBus::Config busCfg;
    busCfg.sdaPin = CONFIG_IMU_I2C_SDA_GPIO;
    busCfg.sclPin = CONFIG_IMU_I2C_SCL_GPIO;
    busCfg.freqHz = CONFIG_IMU_I2C_FREQ_HZ;
#ifdef CONFIG_IMU_ADDR_SA0_HIGH
    busCfg.address = lsm6dsox::kI2cAddrSa0High;
#else
    busCfg.address = lsm6dsox::kI2cAddrSa0Low;
#endif
    if (gBus.init(busCfg) != hal::Status::Ok) {
        fatal("I2C init failed");
    }
    measureBusHealth(busCfg.address);

    // A flaky contact can make a single attempt fail, so retry before giving up.
    hal::Status initStatus = hal::Status::ErrBus;
    for (int attempt = 1; attempt <= 5; ++attempt) {
        initStatus = gImu.init();
        if (initStatus == hal::Status::Ok) {
            break;
        }
        ESP_LOGW(kTag, "imu init attempt %d failed (status=%d, step=%d, esp_err=%s)", attempt,
                 static_cast<int>(initStatus), static_cast<int>(gImu.failedStep()), esp_err_to_name(gBus.lastError()));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (initStatus != hal::Status::Ok) {
        diagnoseImu(initStatus);
        fatal("LSM6DSOX not found (check wiring, address, pull-ups)");
    }

    lsm6dsox::FifoConfig fifo;
    fifo.watermark = 32;
    if (gImu.configureFifo(fifo) != hal::Status::Ok || gImu.configure(lsm6dsox::Config{}) != hal::Status::Ok) {
        fatal("sensor configuration failed");
    }

    acquisition::Config acqCfg;
    acqCfg.intPin = static_cast<gpio_num_t>(CONFIG_IMU_INT1_GPIO);
    if (gAcq.start(acqCfg) != ESP_OK) {
        fatal("acquisition start failed");
    }

    protocol::DeviceInfo info;
    info.odrHz = kOdrHz;
    info.accelMgPerLsb = gImu.accelSensitivityMgPerLsb();
    info.gyroMdpsPerLsb = gImu.gyroSensitivityMdpsPerLsb();
    if (gBle.start(info, kDeviceName) != ESP_OK) {
        fatal("BLE start failed");
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        const auto a = gAcq.stats();
        const auto b = gBle.stats();
        ESP_LOGI(kTag,
                 "acq: smp=%" PRIu32 " irq=%" PRIu32 " drops=%" PRIu32 " ovr=%" PRIu32 " bus_err=%" PRIu32 " stack_free=%" PRIu32
                 "B | ble: conn=%d stream=%d mtu=%u frames=%" PRIu32 " notify_err=%" PRIu32 " lost=%" PRIu32,
                 a.samples, a.interrupts, a.queueDrops, a.fifoOverruns, a.busErrors, a.stackHighWaterBytes,
                 static_cast<int>(b.connected), static_cast<int>(b.streaming), static_cast<unsigned>(b.mtu),
                 b.framesSent, b.notifyErrors, b.samplesLost);
    }
}