#include "acquisition/sample_assembler.hpp"

namespace acquisition {

bool SampleAssembler::push(const lsm6dsox::FifoWord &word, Sample &out)
{
    switch (word.tag) {
    case lsm6dsox::FifoTag::Accel:
        if (hasAccel_) {
            ++unpaired_;
        }
        pending_.accel = word.xyz;
        hasAccel_ = true;
        break;
    case lsm6dsox::FifoTag::Gyro:
        if (hasGyro_) {
            ++unpaired_;
        }
        pending_.gyro = word.xyz;
        hasGyro_ = true;
        break;
    default:
        return false;
    }

    if (!(hasAccel_ && hasGyro_)) {
        return false;
    }
    pending_.seq = nextSeq_++;
    out = pending_;
    hasAccel_ = false;
    hasGyro_ = false;
    return true;
}

void SampleAssembler::reset()
{
    hasAccel_ = false;
    hasGyro_ = false;
}

} // namespace acquisition