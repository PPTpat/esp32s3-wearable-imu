#include "bus_mock.hpp"
#include "lsm6dsox/lsm6dsox.hpp"
#include "lsm6dsox/registers.hpp"
#include "unity.h"

using namespace lsm6dsox;

static mocks::BusMock mock;

extern "C" void setUp(void) {
  mock = mocks::BusMock{};
  mock.regs[reg::WHO_AM_I] = kWhoAmIValue;
  mock.setAutoClear(reg::CTRL3_C, ctrl3_c::SW_RESET);
}
extern "C" void tearDown(void) {}

static void test_init_ok_sets_bdu_and_auto_increment() {
  Driver d(mock);
  TEST_ASSERT_EQUAL(hal::Status::Ok, d.init());
  TEST_ASSERT_EQUAL_HEX8(ctrl3_c::BDU | ctrl3_c::IF_INC,
                         mock.regs[reg::CTRL3_C]);
}

static void test_init_rejects_wrong_who_am_i() {
  mock.regs[reg::WHO_AM_I] = 0x00;
  Driver d(mock);
  TEST_ASSERT_EQUAL(hal::Status::ErrDevice, d.init());
  TEST_ASSERT_EQUAL_UINT(0, mock.writeCalls);
}

static void test_init_times_out_when_reset_never_finishes() {
  mock.setAutoClear(reg::CTRL3_C, 0x00);
  Driver d(mock);
  TEST_ASSERT_EQUAL(hal::Status::ErrTimeout, d.init());
}

static unsigned gDelayCalls = 0;
static void fakeDelay(std::uint32_t) { ++gDelayCalls; }

static void test_init_survives_nack_while_device_reboots() {
  gDelayCalls = 0;
  mock.failAfterWrite(reg::CTRL3_C,
                      2); // device NACKs twice right after SW_RESET
  Driver d(mock, fakeDelay);
  TEST_ASSERT_EQUAL(hal::Status::Ok, d.init());
  TEST_ASSERT_TRUE(d.failedStep() == InitStep::Done);
  TEST_ASSERT_EQUAL_UINT(
      3, gDelayCalls); // two failed polls, then the successful one
}

static void test_init_reports_step_of_failure() {
  {
    Driver d(mock);
    mock.failAfter(0);
    TEST_ASSERT_EQUAL(hal::Status::ErrBus, d.init());
    TEST_ASSERT_TRUE(d.failedStep() == InitStep::WhoAmI);
  }
  setUp();
  {
    Driver d(mock);
    mock.failAfter(1); // WHO_AM_I read passes, the reset write fails
    TEST_ASSERT_EQUAL(hal::Status::ErrBus, d.init());
    TEST_ASSERT_TRUE(d.failedStep() == InitStep::ResetWrite);
  }
}

static void test_init_reports_last_bus_error_if_reset_never_settles() {
  mock.failAfterWrite(reg::CTRL3_C, 1000);
  Driver d(mock);
  TEST_ASSERT_EQUAL(hal::Status::ErrBus, d.init());
  TEST_ASSERT_TRUE(d.failedStep() == InitStep::ResetPoll);
}

static void test_configure_writes_odr_and_range() {
  Driver d(mock);
  TEST_ASSERT_EQUAL(hal::Status::Ok,
                    d.configure(Config{})); // 104 Hz, 4 g / 500 dps
  TEST_ASSERT_EQUAL_HEX8(0x48, mock.regs[reg::CTRL1_XL]);
  TEST_ASSERT_EQUAL_HEX8(0x44, mock.regs[reg::CTRL2_G]);
}

static void test_sensitivity_follows_range() {
  Driver d(mock);
  Config cfg;
  cfg.accelRange = AccelRange::G8;
  cfg.gyroRange = GyroRange::Dps2000;
  TEST_ASSERT_EQUAL(hal::Status::Ok, d.configure(cfg));
  TEST_ASSERT_EQUAL_FLOAT(0.244F, d.accelSensitivityMgPerLsb());
  TEST_ASSERT_EQUAL_FLOAT(70.0F, d.gyroSensitivityMdpsPerLsb());
}

static void test_configure_fifo_encodes_registers() {
  Driver d(mock);
  FifoConfig cfg;
  cfg.accelBatch = Odr::Hz104;
  cfg.gyroBatch = Odr::Hz208;
  cfg.watermark = 300; // needs WTM8
  TEST_ASSERT_EQUAL(hal::Status::Ok, d.configureFifo(cfg));
  TEST_ASSERT_EQUAL_HEX8(300 & 0xFF, mock.regs[reg::FIFO_CTRL1]);
  TEST_ASSERT_EQUAL_HEX8(fifo_ctrl2::WTM8, mock.regs[reg::FIFO_CTRL2]);
  TEST_ASSERT_EQUAL_HEX8(
      0x54, mock.regs[reg::FIFO_CTRL3]); // gyro 208 Hz, accel 104 Hz
  TEST_ASSERT_EQUAL_HEX8(kFifoModeContinuous, mock.regs[reg::FIFO_CTRL4]);
  TEST_ASSERT_EQUAL_HEX8(int1_ctrl::FIFO_TH, mock.regs[reg::INT1_CTRL]);
}

static void test_configure_fifo_rejects_bad_watermark() {
  Driver d(mock);
  FifoConfig cfg;
  cfg.watermark = 0;
  TEST_ASSERT_EQUAL(hal::Status::ErrArg, d.configureFifo(cfg));
  cfg.watermark = 512;
  TEST_ASSERT_EQUAL(hal::Status::ErrArg, d.configureFifo(cfg));
}

static void test_read_fifo_status_decodes_fields() {
  mock.regs[reg::FIFO_STATUS1] = 0x2C;
  mock.regs[reg::FIFO_STATUS2] =
      fifo_status2::WATERMARK | fifo_status2::OVERRUN | 0x01;
  Driver d(mock);
  FifoStatus s;
  TEST_ASSERT_EQUAL(hal::Status::Ok, d.readFifoStatus(s));
  TEST_ASSERT_EQUAL_UINT16(300, s.unread);
  TEST_ASSERT_TRUE(s.watermark);
  TEST_ASSERT_TRUE(s.overrun);
  TEST_ASSERT_FALSE(s.full);
}

static void test_read_fifo_word_decodes_tag_and_samples() {
  mock.regs[reg::FIFO_DATA_OUT_TAG] = 0x02 << 3; // accelerometer
  mock.regs[0x79] = 0x34;
  mock.regs[0x7A] = 0x12; // X = 0x1234
  mock.regs[0x7B] = 0xFF;
  mock.regs[0x7C] = 0xFF; // Y = -1
  mock.regs[0x7D] = 0x00;
  mock.regs[0x7E] = 0x80; // Z = -32768
  Driver d(mock);
  FifoWord w;
  TEST_ASSERT_EQUAL(hal::Status::Ok, d.readFifoWord(w));
  TEST_ASSERT_EQUAL(FifoTag::Accel, w.tag);
  TEST_ASSERT_EQUAL_INT16(0x1234, w.xyz[0]);
  TEST_ASSERT_EQUAL_INT16(-1, w.xyz[1]);
  TEST_ASSERT_EQUAL_INT16(-32768, w.xyz[2]);
}

static void test_bus_error_is_propagated() {
  Driver d(mock);
  mock.failAfter(0);
  TEST_ASSERT_EQUAL(hal::Status::ErrBus, d.configure(Config{}));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_init_ok_sets_bdu_and_auto_increment);
  RUN_TEST(test_init_rejects_wrong_who_am_i);
  RUN_TEST(test_init_times_out_when_reset_never_finishes);
  RUN_TEST(test_init_survives_nack_while_device_reboots);
  RUN_TEST(test_init_reports_step_of_failure);
  RUN_TEST(test_init_reports_last_bus_error_if_reset_never_settles);
  RUN_TEST(test_configure_writes_odr_and_range);
  RUN_TEST(test_sensitivity_follows_range);
  RUN_TEST(test_configure_fifo_encodes_registers);
  RUN_TEST(test_configure_fifo_rejects_bad_watermark);
  RUN_TEST(test_read_fifo_status_decodes_fields);
  RUN_TEST(test_read_fifo_word_decodes_tag_and_samples);
  RUN_TEST(test_bus_error_is_propagated);
  return UNITY_END();
}