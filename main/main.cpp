/**
 * @file main.cpp
 * @brief Firmware entry point: IMU acquisition streamed over BLE.
 */
#include <cinttypes>

#include "acquisition/acquisition.hpp"
#include "ble_service/ble_service.hpp"
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
hal::I2cBus gBus;
lsm6dsox::Driver gImu(gBus);
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
    if (gImu.init() != hal::Status::Ok) {
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
                 "acq: smp=%" PRIu32 " drops=%" PRIu32 " ovr=%" PRIu32 " bus_err=%" PRIu32 " stack_free=%" PRIu32
                 "B | ble: conn=%d stream=%d mtu=%u frames=%" PRIu32 " notify_err=%" PRIu32 " lost=%" PRIu32,
                 a.samples, a.queueDrops, a.fifoOverruns, a.busErrors, a.stackHighWaterBytes,
                 static_cast<int>(b.connected), static_cast<int>(b.streaming), static_cast<unsigned>(b.mtu),
                 b.framesSent, b.notifyErrors, b.samplesLost);
    }
}