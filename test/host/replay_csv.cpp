/**
 * @file replay_csv.cpp
 * @brief Host tool: run a recorded session (tools/ble_logger.py CSV) through
 * the DSP algorithms.
 *
 * Usage: replay_csv session.csv [--events]
 * CSV columns: seq,t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps (first line is a
 * header).
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "dsp/fall_detector.hpp"
#include "dsp/madgwick.hpp"
#include "dsp/step_counter.hpp"

namespace {

struct Row {
  double t = 0.0;
  dsp::Vec3 accel;
  dsp::Vec3 gyro;
};

/** @brief Parse "a,b,c,..." into exactly @p count numbers; false on any
 * malformed field. */
bool parseFields(const std::string &line, double *out, int count) {
  const char *p = line.c_str();
  for (int i = 0; i < count; ++i) {
    char *end = nullptr;
    out[i] = std::strtod(p, &end);
    if (end == p) {
      return false;
    }
    p = end;
    if (i + 1 < count) {
      if (*p != ',') {
        return false;
      }
      ++p;
    }
  }
  return true;
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s session.csv [--events]\n", argv[0]);
    return 2;
  }
  const bool showEvents = argc > 2 && std::strcmp(argv[2], "--events") == 0;

  std::ifstream in(argv[1]);
  if (!in) {
    std::fprintf(stderr, "cannot open %s\n", argv[1]);
    return 1;
  }

  std::vector<Row> rows;
  std::string line;
  unsigned skipped = 0;
  std::getline(in, line); // header
  while (std::getline(in, line)) {
    double f[8];
    if (!parseFields(line, f, 8)) {
      ++skipped;
      continue;
    }
    Row r;
    r.t = f[1];
    r.accel = dsp::Vec3{static_cast<float>(f[2]), static_cast<float>(f[3]),
                        static_cast<float>(f[4])};
    r.gyro = dsp::Vec3{static_cast<float>(f[5]), static_cast<float>(f[6]),
                       static_cast<float>(f[7])};
    rows.push_back(r);
  }
  if (rows.size() < 2) {
    std::fprintf(stderr,
                 "not enough samples (%zu), skipped %u malformed lines\n",
                 rows.size(), skipped);
    return 1;
  }

  const double duration = rows.back().t - rows.front().t;
  const double rate = static_cast<double>(rows.size() - 1) / duration;

  dsp::StepCounterConfig stepCfg;
  stepCfg.sampleRateHz = static_cast<float>(rate);
  dsp::FallDetectorConfig fallCfg;
  fallCfg.sampleRateHz = static_cast<float>(rate);
  dsp::StepCounter steps(stepCfg);
  dsp::FallDetector falls(fallCfg);
  dsp::Madgwick filter;
  const float dt = static_cast<float>(1.0 / rate);

  for (const Row &r : rows) {
    const float mag = dsp::norm(r.accel);
    if (steps.update(mag) && showEvents) {
      std::printf("step %u at %.2f s\n", steps.steps(), r.t);
    }
    if (falls.update(mag)) {
      std::printf("FALL detected at %.2f s\n", r.t);
    }
    const dsp::Vec3 gyroRad{r.gyro.x * dsp::kDegToRad,
                            r.gyro.y * dsp::kDegToRad,
                            r.gyro.z * dsp::kDegToRad};
    filter.update(gyroRad, r.accel, dt);
  }

  const dsp::Euler e = filter.euler();
  std::printf("file: %s\n", argv[1]);
  std::printf(
      "samples: %zu (skipped %u malformed), duration: %.1f s, rate: %.1f Hz\n",
      rows.size(), skipped, duration, rate);
  std::printf("steps: %u\n", steps.steps());
  std::printf("falls: %u\n", falls.falls());
  std::printf("final orientation: roll %.1f, pitch %.1f, yaw %.1f deg\n",
              static_cast<double>(e.rollDeg), static_cast<double>(e.pitchDeg),
              static_cast<double>(e.yawDeg));
  return 0;
}