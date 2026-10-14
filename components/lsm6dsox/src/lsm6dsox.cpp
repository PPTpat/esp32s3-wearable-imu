#include "lsm6dsox/lsm6dsox.hpp"

#include "lsm6dsox/registers.hpp"

namespace lsm6dsox {

namespace {

constexpr unsigned kResetPollAttempts = 100;
constexpr std::uint16_t kMaxWatermark = 511;
constexpr std::size_t kFifoWordSize = 7;

constexpr std::array<float, 4> kAccelMgPerLsb = {0.061F, 0.488F, 0.122F, 0.244F}; // 2g,16g,4g,8g
constexpr std::array<float, 4> kGyroMdpsPerLsb = {8.75F, 17.5F, 35.0F, 70.0F};

constexpr std::uint8_t u8(unsigned v)
{
    return static_cast<std::uint8_t>(v);
}

constexpr std::int16_t toS16(std::uint8_t lo, std::uint8_t hi)
{
    return static_cast<std::int16_t>(static_cast<std::uint16_t>(lo | (hi << 8)));
}

FifoTag decodeTag(std::uint8_t raw)
{
    switch (raw >> 3) {
    case 0x01:
        return FifoTag::Gyro;
    case 0x02:
        return FifoTag::Accel;
    case 0x03:
        return FifoTag::Temperature;
    case 0x04:
        return FifoTag::Timestamp;
    default:
        return FifoTag::Unknown;
    }
}

} // namespace

hal::Status Driver::init()
{
    std::uint8_t id = 0;
    if (const auto st = bus_.readU8(reg::WHO_AM_I, id); st != hal::Status::Ok) {
        return st;
    }
    if (id != kWhoAmIValue) {
        return hal::Status::ErrDevice;
    }

    if (const auto st = bus_.writeU8(reg::CTRL3_C, ctrl3_c::SW_RESET); st != hal::Status::Ok) {
        return st;
    }

    // SW_RESET clears itself when the reset is done; poll with a bounded loop.
    std::uint8_t ctrl3 = ctrl3_c::SW_RESET;
    for (unsigned i = 0; i < kResetPollAttempts && (ctrl3 & ctrl3_c::SW_RESET) != 0; ++i) {
        if (const auto st = bus_.readU8(reg::CTRL3_C, ctrl3); st != hal::Status::Ok) {
            return st;
        }
    }
    if ((ctrl3 & ctrl3_c::SW_RESET) != 0) {
        return hal::Status::ErrTimeout;
    }

    // BDU: output registers update only after both bytes were read.
    // IF_INC: address auto-increment for burst reads.
    return bus_.writeU8(reg::CTRL3_C, u8(ctrl3_c::BDU | ctrl3_c::IF_INC));
}

hal::Status Driver::configure(const Config &cfg)
{
    const auto xl =
        u8((static_cast<unsigned>(cfg.accelOdr) << 4) | (static_cast<unsigned>(cfg.accelRange) << 2));
    const auto g = u8((static_cast<unsigned>(cfg.gyroOdr) << 4) | (static_cast<unsigned>(cfg.gyroRange) << 2));

    if (const auto st = bus_.writeU8(reg::CTRL1_XL, xl); st != hal::Status::Ok) {
        return st;
    }
    if (const auto st = bus_.writeU8(reg::CTRL2_G, g); st != hal::Status::Ok) {
        return st;
    }
    accelRange_ = cfg.accelRange;
    gyroRange_ = cfg.gyroRange;
    return hal::Status::Ok;
}

hal::Status Driver::configureFifo(const FifoConfig &cfg)
{
    if (cfg.watermark == 0 || cfg.watermark > kMaxWatermark) {
        return hal::Status::ErrArg;
    }

    const auto wtmLow = u8(cfg.watermark & 0xFFU);
    const auto ctrl2 = u8((cfg.watermark & 0x100U) != 0 ? fifo_ctrl2::WTM8 : 0U);
    const auto ctrl3 = u8((static_cast<unsigned>(cfg.gyroBatch) << 4) | static_cast<unsigned>(cfg.accelBatch));
    // Timestamp and temperature batching stay off for now (DEC_TS_BATCH = ODR_T_BATCH = 0).
    const auto ctrl4 = kFifoModeContinuous;
    const auto int1 = u8(cfg.routeWatermarkToInt1 ? int1_ctrl::FIFO_TH : 0U);

    if (const auto st = bus_.writeU8(reg::FIFO_CTRL1, wtmLow); st != hal::Status::Ok) {
        return st;
    }
    if (const auto st = bus_.writeU8(reg::FIFO_CTRL2, ctrl2); st != hal::Status::Ok) {
        return st;
    }
    if (const auto st = bus_.writeU8(reg::FIFO_CTRL3, ctrl3); st != hal::Status::Ok) {
        return st;
    }
    if (const auto st = bus_.writeU8(reg::FIFO_CTRL4, ctrl4); st != hal::Status::Ok) {
        return st;
    }
    return bus_.writeU8(reg::INT1_CTRL, int1);
}

hal::Status Driver::readFifoStatus(FifoStatus &out)
{
    std::array<std::uint8_t, 2> raw{};
    if (const auto st = bus_.read(reg::FIFO_STATUS1, raw); st != hal::Status::Ok) {
        return st;
    }
    const std::uint8_t s2 = raw[1];
    out.unread = static_cast<std::uint16_t>(raw[0] | ((s2 & fifo_status2::DIFF_FIFO_HI_MASK) << 8));
    out.watermark = (s2 & fifo_status2::WATERMARK) != 0;
    out.overrun = (s2 & fifo_status2::OVERRUN) != 0;
    out.full = (s2 & fifo_status2::FULL) != 0;
    return hal::Status::Ok;
}

hal::Status Driver::readFifoWord(FifoWord &out)
{
    std::array<std::uint8_t, kFifoWordSize> raw{};
    if (const auto st = bus_.read(reg::FIFO_DATA_OUT_TAG, raw); st != hal::Status::Ok) {
        return st;
    }
    out.tag = decodeTag(raw[0]);
    out.xyz = {toS16(raw[1], raw[2]), toS16(raw[3], raw[4]), toS16(raw[5], raw[6])};
    return hal::Status::Ok;
}

float Driver::accelSensitivityMgPerLsb() const
{
    return kAccelMgPerLsb[static_cast<std::size_t>(accelRange_)];
}

float Driver::gyroSensitivityMdpsPerLsb() const
{
    return kGyroMdpsPerLsb[static_cast<std::size_t>(gyroRange_)];
}

} // namespace lsm6dsox