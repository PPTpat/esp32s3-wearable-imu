/**
 * @file bus_mock.hpp
 * @brief In-memory register file implementing hal::IBus, with fault injection.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "hal/bus.hpp"

namespace mocks {

/** @brief Fake bus backed by a 256-byte register file (host tests only). */
class BusMock final : public hal::IBus {
  public:
    static constexpr std::size_t kRegCount = 256;

    std::array<std::uint8_t, kRegCount> regs{}; ///< Register contents, directly editable by tests.
    unsigned readCalls = 0;                     ///< Number of read() calls.
    unsigned writeCalls = 0;                    ///< Number of write() calls.

    /**
     * @brief Make the bus fail permanently after @p n more successful calls.
     * @param n Number of calls allowed before failing; negative disables injection.
     */
    void failAfter(int n) { failAfter_ = n; }

    /**
     * @brief Make selected bits of a register clear themselves after every write
     *        (emulates self-clearing bits such as SW_RESET).
     * @param reg  Register address.
     * @param mask Bits that read back as zero after a write.
     */
    void setAutoClear(std::uint8_t reg, std::uint8_t mask) { autoClear_[reg] = mask; }

    /**
     * @brief Emulate a device that is busy after a write to @p reg: the next @p count calls
     *        fail with ErrBus (one-shot, e.g. a sensor rebooting after SW_RESET).
     */
    void failAfterWrite(std::uint8_t reg, unsigned count)
    {
        busyReg_ = reg;
        busyCount_ = count;
        busyArmed_ = true;
    }

    hal::Status read(std::uint8_t reg, std::span<std::uint8_t> out) override
    {
        ++readCalls;
        if (const auto st = check(reg, out.size()); st != hal::Status::Ok) {
            return st;
        }
        std::memcpy(out.data(), &regs[reg], out.size());
        return hal::Status::Ok;
    }

    hal::Status write(std::uint8_t reg, std::span<const std::uint8_t> data) override
    {
        ++writeCalls;
        if (const auto st = check(reg, data.size()); st != hal::Status::Ok) {
            return st;
        }
        std::memcpy(&regs[reg], data.data(), data.size());
        for (std::size_t i = 0; i < data.size(); ++i) {
            regs[reg + i] &= static_cast<std::uint8_t>(~autoClear_[reg + i]);
        }
        if (busyArmed_ && reg == busyReg_) {
            busyArmed_ = false;
            failNext_ = busyCount_;
        }
        return hal::Status::Ok;
    }

  private:
    hal::Status check(std::uint8_t reg, std::size_t len)
    {
        if (len == 0 || static_cast<std::size_t>(reg) + len > kRegCount) {
            return hal::Status::ErrArg;
        }
        if (failNext_ > 0) {
            --failNext_;
            return hal::Status::ErrBus;
        }
        if (failAfter_ == 0) {
            return hal::Status::ErrBus;
        }
        if (failAfter_ > 0) {
            --failAfter_;
        }
        return hal::Status::Ok;
    }

    std::array<std::uint8_t, kRegCount> autoClear_{};
    int failAfter_ = -1;
    unsigned failNext_ = 0;
    unsigned busyCount_ = 0;
    std::uint8_t busyReg_ = 0;
    bool busyArmed_ = false;
};

} // namespace mocks