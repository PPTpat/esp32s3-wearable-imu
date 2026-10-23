/**
 * @file ble_service.hpp
 * @brief BLE GATT peripheral that streams IMU samples (ESP-IDF, NimBLE).
 *
 * GATT layout (UUIDs in protocol/uuids.hpp):
 *  - Data    : notify, carries sample frames
 *  - Control : write, one-byte commands (start / stop)
 *  - Info    : read, DeviceInfo (sample rate and scales)
 *
 * This class is the only consumer of the SampleQueue: a streaming task drains
 * it periodically, batches samples into frames and notifies the client.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <span>

#include "acquisition/acquisition.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "protocol/frame.hpp"

struct ble_gap_event;
struct ble_gatt_access_ctxt;

namespace ble_service {

/** @brief Runtime counters. */
struct Stats {
  std::uint32_t framesSent = 0; ///< Frames accepted by the BLE stack.
  std::uint32_t notifyErrors =
      0; ///< Failed notifications (e.g. out of buffers).
  std::uint32_t samplesLost =
      0;                 ///< Samples dropped because a notification failed.
  std::uint16_t mtu = 0; ///< Negotiated ATT MTU (23 = not negotiated yet).
  bool connected = false;
  bool streaming = false; ///< Client sent Start and enabled notifications.
};

/** @brief GATT server plus streaming task. */
class BleService {
public:
  /** @param queue Samples to stream; this service becomes its only consumer. */
  explicit BleService(acquisition::SampleQueue &queue) : queue_(queue) {}

  /**
   * @brief Start the BLE stack, register the GATT service and begin
   * advertising.
   * @param info       Sent to the client through the Info characteristic.
   * @param deviceName Advertised name (max about 20 characters).
   * @return ESP_OK on success.
   */
  esp_err_t start(const protocol::DeviceInfo &info, const char *deviceName);

  /** @brief Snapshot of the counters (safe to call from any task). */
  Stats stats() const;

private:
  static void hostTask(void *arg);
  static void onSync();
  static void onReset(int reason);
  static int gapEvent(ble_gap_event *event, void *arg);
  static int accessCb(std::uint16_t connHandle, std::uint16_t attrHandle,
                      ble_gatt_access_ctxt *ctxt, void *arg);
  static void streamEntry(void *arg);

  void buildGattTable();
  void startAdvertising();
  void streamLoop();
  void flush(std::span<const acquisition::Sample> samples);

  acquisition::SampleQueue &queue_;
  protocol::DeviceInfo info_{};
  std::array<std::uint8_t, protocol::kInfoBytes> infoBytes_{};
  const char *name_ = "";

  std::uint8_t ownAddrType_ = 0;
  std::uint16_t dataHandle_ = 0;
  std::uint16_t frameSeq_ = 0;

  std::atomic<std::uint16_t> connHandle_{0};
  std::atomic<std::uint16_t> mtu_{23};
  std::atomic<bool> connected_{false};
  std::atomic<bool> notifyEnabled_{false};
  std::atomic<bool> startRequested_{false};

  std::atomic<std::uint32_t> framesSent_{0};
  std::atomic<std::uint32_t> notifyErrors_{0};
  std::atomic<std::uint32_t> samplesLost_{0};
};

} // namespace ble_service