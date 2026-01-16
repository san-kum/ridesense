#pragma once

#include "capture/stream.h"
#include "common/logger.h"
#include <atomic>
#include <random>

namespace ridersense {

// Mock GPS for now - TODO: replace with real NMEA parser
class GpsStream : public Stream {
public:
  explicit GpsStream(int rate_hz);
  ~GpsStream() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

private:
  void generate_loop();

  int rate_hz_;
  std::shared_ptr<spdlog::logger> logger_;

  std::atomic<bool> running_{false};
  std::thread thread_;
  uint64_t sample_count_{0};

  double lat_base_{37.7749};
  double lon_base_{-122.4194};
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0.0, 0.00001};
};

} // namespace ridersense
