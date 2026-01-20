#include "replay/replay_controller.h"
#include <algorithm>

namespace ridersense {

ReplayController::ReplayController(const std::string &data_dir, double speed)
    : Service("replay_controller"), data_dir_(data_dir), speed_(speed),
      logger_(Logger::get("replay")), reader_(data_dir) {}

ReplayController::~ReplayController() { shutdown(); }

bool ReplayController::init() {
  if (!reader_.open()) {
    LOG_ERROR(logger_, "failed to open replay data");
    return false;
  }

  LOG_INFO(logger_, "replay controller initialized: {}", data_dir_);
  LOG_INFO(logger_, "playback speed: {}x", speed_);
  return true;
}

bool ReplayController::start() {
  running_ = true;
  state_ = ReplayState::PAUSED; 
  playback_thread_ = std::thread(&ReplayController::playback_loop, this);
  LOG_INFO(logger_, "replay controller started (paused)");
  return true;
}

bool ReplayController::stop() {
  LOG_INFO(logger_, "stopping replay controller");
  running_ = false;
  state_ = ReplayState::STOPPED;
  cv_.notify_all();

  if (playback_thread_.joinable()) {
    playback_thread_.join();
  }

  auto s = get_stats();
  LOG_INFO(logger_, "replay stats - IMU: {}, GPS: {}", s.imu_frames_played,
           s.gps_frames_played);
  return true;
}

bool ReplayController::shutdown() {
  reader_.close();
  LOG_INFO(logger_, "replay controller shutdown");
  return true;
}

void ReplayController::play() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == ReplayState::PAUSED) {
    state_ = ReplayState::PLAYING;
    start_wall_time_ = std::chrono::steady_clock::now();
    cv_.notify_all();
    LOG_INFO(logger_, "replay: playing");
  }
}

void ReplayController::pause() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == ReplayState::PLAYING) {
    state_ = ReplayState::PAUSED;
    LOG_INFO(logger_, "replay: paused");
  }
}

void ReplayController::step() {
  std::lock_guard<std::mutex> lock(mutex_);
  step_requested_ = true;
  cv_.notify_all();
}

void ReplayController::set_speed(double speed) {
  std::lock_guard<std::mutex> lock(mutex_);
  speed_ = std::clamp(speed, 0.1, 10.0);
  LOG_INFO(logger_, "replay speed: {}x", speed_);
}

ReplayController::Stats ReplayController::get_stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void ReplayController::playback_loop() {
  auto imu_ts = reader_.peek_next_imu_ts();
  auto gps_ts = reader_.peek_next_gps_ts();

  if (!imu_ts && !gps_ts) {
    LOG_WARN(logger_, "no frames to replay");
    return;
  }

  start_replay_ts_ = std::min(imu_ts.value_or(UINT64_MAX),
                               gps_ts.value_or(UINT64_MAX));
  start_wall_time_ = std::chrono::steady_clock::now();

  while (running_) {
    std::unique_lock<std::mutex> lock(mutex_);

    cv_.wait(lock, [this] {
      return !running_ || state_ == ReplayState::PLAYING || step_requested_;
    });

    if (!running_) {
      break;
    }

    bool do_step = step_requested_;
    step_requested_ = false;
    lock.unlock();

    auto next_imu = reader_.peek_next_imu_ts();
    auto next_gps = reader_.peek_next_gps_ts();

    if (!next_imu && !next_gps) {
      LOG_INFO(logger_, "replay complete");
      std::lock_guard<std::mutex> l(mutex_);
      state_ = ReplayState::STOPPED;
      break;
    }

    uint64_t next_ts = std::min(next_imu.value_or(UINT64_MAX),
                                 next_gps.value_or(UINT64_MAX));

    if (!do_step && state_ == ReplayState::PLAYING) {
      uint64_t elapsed_replay = next_ts - start_replay_ts_;
      auto target_wall = start_wall_time_ +
                         std::chrono::nanoseconds(
                             static_cast<int64_t>(elapsed_replay / speed_));
      auto now = std::chrono::steady_clock::now();

      if (target_wall > now) {
        std::this_thread::sleep_until(target_wall);
      }
    }

    if (next_imu && (!next_gps || *next_imu <= *next_gps)) {
      if (auto frame = reader_.read_next_imu()) {
        if (imu_callback_) {
          imu_callback_(*frame);
        }
        std::lock_guard<std::mutex> l(mutex_);
        stats_.imu_frames_played++;
        stats_.current_ts = *next_imu;
      }
    } else if (next_gps) {
      if (auto frame = reader_.read_next_gps()) {
        if (gps_callback_) {
          gps_callback_(*frame);
        }
        std::lock_guard<std::mutex> l(mutex_);
        stats_.gps_frames_played++;
        stats_.current_ts = *next_gps;
      }
    }

    if (do_step) {
      std::lock_guard<std::mutex> l(mutex_);
      state_ = ReplayState::PAUSED;
    }
  }
}

void ReplayController::emit_next_frame() {
  // helper for step mode - handled in playback_loop
}

} // namespace ridersense
