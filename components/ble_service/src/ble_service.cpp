#include "ble_service/ble_service.hpp"

#include <algorithm>
#include <cstring>

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "protocol/uuids.hpp"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace ble_service {

namespace {

constexpr const char *kTag = "ble";
constexpr std::uint32_t kStreamPeriodMs =
    50; ///< Frame cadence: about 20 frames/s.
constexpr std::uint32_t kStreamStackBytes = 4096;
constexpr UBaseType_t kStreamPriority = 5;
constexpr std::uint16_t kAttOverhead =
    3; ///< ATT notification header (opcode + handle).

BleService *gSelf =
    nullptr; // NimBLE sync/reset callbacks carry no user argument.

ble_uuid128_t gSvcUuid;
ble_uuid128_t gDataUuid;
ble_uuid128_t gControlUuid;
ble_uuid128_t gInfoUuid;
ble_gatt_chr_def gChars[4];
ble_gatt_svc_def gSvcs[2];

/** @brief Fill a NimBLE UUID (little-endian) from our canonical big-endian
 * array. */
void setUuid(ble_uuid128_t &dst, const protocol::uuid::Uuid128 &src) {
  dst.u.type = BLE_UUID_TYPE_128;
  std::reverse_copy(src.begin(), src.end(), dst.value);
}

} // namespace

esp_err_t BleService::start(const protocol::DeviceInfo &info,
                            const char *deviceName) {
  if (gSelf != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  gSelf = this;
  info_ = info;
  name_ = deviceName;
  if (protocol::encodeInfo(infoBytes_, info_) == 0) {
    return ESP_FAIL;
  }

  if (const esp_err_t err = nimble_port_init(); err != ESP_OK) {
    return err;
  }
  ble_hs_cfg.reset_cb = onReset;
  ble_hs_cfg.sync_cb = onSync;

  ble_svc_gap_init();
  ble_svc_gatt_init();
  buildGattTable();
  if (ble_gatts_count_cfg(gSvcs) != 0 || ble_gatts_add_svcs(gSvcs) != 0) {
    return ESP_FAIL;
  }
  if (ble_svc_gap_device_name_set(name_) != 0) {
    return ESP_FAIL;
  }

  nimble_port_freertos_init(hostTask);
  if (xTaskCreate(streamEntry, "ble_stream", kStreamStackBytes, this,
                  kStreamPriority, nullptr) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

void BleService::buildGattTable() {
  setUuid(gSvcUuid, protocol::uuid::kService);
  setUuid(gDataUuid, protocol::uuid::kData);
  setUuid(gControlUuid, protocol::uuid::kControl);
  setUuid(gInfoUuid, protocol::uuid::kInfo);

  // Zero-initialised entries terminate both arrays.
  gChars[0].uuid = &gDataUuid.u;
  gChars[0].access_cb = accessCb;
  gChars[0].arg = this;
  gChars[0].flags = BLE_GATT_CHR_F_NOTIFY;
  gChars[0].val_handle = &dataHandle_;

  gChars[1].uuid = &gControlUuid.u;
  gChars[1].access_cb = accessCb;
  gChars[1].arg = this;
  gChars[1].flags = BLE_GATT_CHR_F_WRITE;

  gChars[2].uuid = &gInfoUuid.u;
  gChars[2].access_cb = accessCb;
  gChars[2].arg = this;
  gChars[2].flags = BLE_GATT_CHR_F_READ;

  gSvcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
  gSvcs[0].uuid = &gSvcUuid.u;
  gSvcs[0].characteristics = gChars;
}

void BleService::hostTask(void *) {
  nimble_port_run(); // returns only when nimble_port_stop() is called
  nimble_port_freertos_deinit();
}

void BleService::onReset(int reason) {
  ESP_LOGW(kTag, "host reset, reason=%d", reason);
}

void BleService::onSync() {
  if (ble_hs_util_ensure_addr(0) != 0 ||
      ble_hs_id_infer_auto(0, &gSelf->ownAddrType_) != 0) {
    ESP_LOGE(kTag, "no usable BLE address");
    return;
  }
  gSelf->startAdvertising();
}

void BleService::startAdvertising() {
  ble_hs_adv_fields fields;
  std::memset(&fields, 0, sizeof(fields));
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.name = reinterpret_cast<const std::uint8_t *>(name_);
  fields.name_len = static_cast<std::uint8_t>(std::strlen(name_));
  fields.name_is_complete = 1;
  if (ble_gap_adv_set_fields(&fields) != 0) {
    ESP_LOGE(kTag, "adv fields rejected (name too long?)");
    return;
  }

  ble_gap_adv_params params;
  std::memset(&params, 0, sizeof(params));
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  const int rc = ble_gap_adv_start(ownAddrType_, nullptr, BLE_HS_FOREVER,
                                   &params, gapEvent, this);
  if (rc != 0) {
    ESP_LOGE(kTag, "adv start failed rc=%d", rc);
  }
}

int BleService::gapEvent(ble_gap_event *event, void *arg) {
  auto *self = static_cast<BleService *>(arg);
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      self->connHandle_ = event->connect.conn_handle;
      self->connected_ = true;
      ble_gattc_exchange_mtu(event->connect.conn_handle, nullptr, nullptr);
      ESP_LOGI(kTag, "connected");
    } else {
      self->startAdvertising();
    }
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    self->connected_ = false;
    self->notifyEnabled_ = false;
    self->startRequested_ = false;
    self->mtu_ = 23;
    ESP_LOGI(kTag, "disconnected");
    self->startAdvertising();
    break;
  case BLE_GAP_EVENT_MTU:
    self->mtu_ = event->mtu.value;
    ESP_LOGI(kTag, "mtu=%u", static_cast<unsigned>(event->mtu.value));
    break;
  case BLE_GAP_EVENT_SUBSCRIBE:
    if (event->subscribe.attr_handle == self->dataHandle_) {
      self->notifyEnabled_ = event->subscribe.cur_notify != 0;
    }
    break;
  default:
    break;
  }
  return 0;
}

int BleService::accessCb(std::uint16_t, std::uint16_t,
                         ble_gatt_access_ctxt *ctxt, void *arg) {
  auto *self = static_cast<BleService *>(arg);

  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR &&
      ble_uuid_cmp(ctxt->chr->uuid, &gInfoUuid.u) == 0) {
    const auto len = static_cast<std::uint16_t>(self->infoBytes_.size());
    return os_mbuf_append(ctxt->om, self->infoBytes_.data(), len) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR &&
      ble_uuid_cmp(ctxt->chr->uuid, &gControlUuid.u) == 0) {
    std::array<std::uint8_t, 4> buf{};
    const std::uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0 || len > buf.size()) {
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (os_mbuf_copydata(ctxt->om, 0, len, buf.data()) != 0) {
      return BLE_ATT_ERR_UNLIKELY;
    }
    const auto cmd =
        protocol::parseCommand(std::span<const std::uint8_t>(buf.data(), len));
    if (!cmd.has_value()) {
      return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    self->startRequested_ = (*cmd == protocol::Command::Start);
    ESP_LOGI(kTag, "command: %s", self->startRequested_ ? "start" : "stop");
    return 0;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

void BleService::streamEntry(void *arg) {
  static_cast<BleService *>(arg)->streamLoop();
}

void BleService::streamLoop() {
  std::array<acquisition::Sample, protocol::kMaxSamplesPerFrame> batch{};
  std::size_t n = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(kStreamPeriodMs));

    const bool active = connected_ && notifyEnabled_ && startRequested_;
    const std::size_t cap =
        active ? protocol::maxSamplesForPayload(
                     static_cast<std::size_t>(mtu_.load()) - kAttOverhead)
               : 0;

    acquisition::Sample s;
    if (cap == 0) {
      // Idle: discard so the next Start begins with fresh data and the queue
      // never backs up.
      while (queue_.pop(s)) {
      }
      n = 0;
      continue;
    }

    while (queue_.pop(s)) {
      // A hole in seq (queue overflow upstream) must not be hidden inside one
      // frame.
      if (n > 0 && s.seq != batch[n - 1].seq + 1U) {
        flush(std::span<const acquisition::Sample>(batch.data(), n));
        n = 0;
      }
      batch[n++] = s;
      if (n == cap) {
        flush(std::span<const acquisition::Sample>(batch.data(), n));
        n = 0;
      }
    }
    if (n > 0) {
      flush(std::span<const acquisition::Sample>(batch.data(), n));
      n = 0;
    }
  }
}

void BleService::flush(std::span<const acquisition::Sample> samples) {
  std::array<std::uint8_t, protocol::kMaxFrameBytes> frame{};
  const std::size_t len =
      protocol::encodeSamplesFrame(frame, frameSeq_, samples);
  os_mbuf *om =
      len != 0
          ? ble_hs_mbuf_from_flat(frame.data(), static_cast<std::uint16_t>(len))
          : nullptr;

  // ble_gatts_notify_custom() takes ownership of the mbuf, also on failure.
  if (om == nullptr ||
      ble_gatts_notify_custom(connHandle_, dataHandle_, om) != 0) {
    notifyErrors_.fetch_add(1, std::memory_order_relaxed);
    samplesLost_.fetch_add(static_cast<std::uint32_t>(samples.size()),
                           std::memory_order_relaxed);
    return;
  }
  ++frameSeq_; // only advance on success: a gap in frame_seq then means a frame
               // was lost in transit
  framesSent_.fetch_add(1, std::memory_order_relaxed);
}

Stats BleService::stats() const {
  Stats s;
  s.framesSent = framesSent_.load(std::memory_order_relaxed);
  s.notifyErrors = notifyErrors_.load(std::memory_order_relaxed);
  s.samplesLost = samplesLost_.load(std::memory_order_relaxed);
  s.mtu = mtu_.load(std::memory_order_relaxed);
  s.connected = connected_.load(std::memory_order_relaxed);
  s.streaming = s.connected && notifyEnabled_.load(std::memory_order_relaxed) &&
                startRequested_.load(std::memory_order_relaxed);
  return s;
}

} // namespace ble_service