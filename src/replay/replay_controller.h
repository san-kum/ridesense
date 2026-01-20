#pragma once

#include "common/logger.h"
#include "replay/replay_reader.h"
#include "services/service.h"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace ridersense {

enum class ReplayState { STOPPED, PLAYING, PAUSED };

using ImuCallback = std::function<void(std::shared_ptr<ImuFrame>)>;
using GpsCallback = std::function<void(std::shared_ptr<GpsFrame>)>;

class ReplayController : public Service {
public:
  ReplayController(const std::string &data_dir, double speed = 1.0);
  ~ReplayController() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void play();
  void pause();
  void step(); 
  void set_speed(double speed);

  ReplayState get_state() const { return state_; }
  double get_speed() const { return speed_; }

  void set_imu_callback(ImuCallback cb) { imu_callback_ = cb; }
  void set_gps_callback(GpsCallback cb) { gps_callback_ = cb; }

  struct Stats {
    uint64_t imu_frames_played{0};
    uint64_t gps_frames_played{0};
    uint64_t current_ts{0};
  };
  Stats get_stats() const;

private:
  void playback_loop();
  void emit_next_frame();

  std::string data_dir_;
  double speed_;
  std::shared_ptr<spdlog::logger> logger_;

  ReplayReader reader_;
  ReplayState state_{ReplayState::STOPPED};

  std::atomic<bool> running_{false};
  std::thread playback_thread_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool step_requested_{false};

  ImuCallback imu_callback_;
  GpsCallback gps_callback_;

  Stats stats_;
  uint64_t start_replay_ts_{0};
  std::chrono::steady_clock::time_point start_wall_time_;
};

} // namespace ridersense
