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
        return hal::Status::Ok;
    }

  private:
    hal::Status check(std::uint8_t reg, std::size_t len)
    {
        if (len == 0 || static_cast<std::size_t>(reg) + len > kRegCount) {
            return hal::Status::ErrArg;
        }
        if (failAfter_ == 0) {
            return hal::Status::ErrBus;
        }
        if (failAfter_ > 0) {
            --failAfter_;
        }
        return hal::Status::Ok;
    }

    int failAfter_ = -1;
};

} // namespace mocks