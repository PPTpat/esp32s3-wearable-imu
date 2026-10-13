/**
 * @file main.cpp
 * @brief Firmware entry point.
 */
#include "esp_log.h"

namespace {
constexpr const char *kTag = "app";
}

/** @brief ESP-IDF entry point (called from the main task). */
extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "wearable-imu booting");
}