#pragma once

#include "../capture/stream.h"
#include "../common/logger.h"
#include <atomic>
#include <cstdint>
#include <memory>
#include <random>
#include <thread>

namespace ridersense {

class ImuStream : public Stream {
public:
  explicit ImuStream(int rate_hz);
  ~ImuStream() override;

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

  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0.0, 0.1};
};
} // namespace ridersense
