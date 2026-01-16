#pragma once

#include "capture/frame.h"
#include "common/logger.h"
#include "services/service.h"
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

namespace ridersense {

// TODO: use proper serialization format (protobuf/flatbuffers/etc)
class FrameLogger : public Service {
public:
  explicit FrameLogger(const std::string &output_dir);
  ~FrameLogger() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void log_frame(std::shared_ptr<Frame> frame);

private:
  void writer_loop();
  void write_image_frame(const ImageFrame *frame);
  void write_imu_frame(const ImuFrame *frame);
  void write_gps_frame(const GpsFrame *frame);

  std::string output_dir_;
  std::shared_ptr<spdlog::logger> logger_;

  std::ofstream image_stream_;
  std::ofstream imu_stream_;
  std::ofstream gps_stream_;

  std::queue<std::shared_ptr<Frame>> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  std::atomic<bool> running_{false};
  std::thread writer_thread_;

  uint64_t frames_written_{0};
};

} // namespace ridersense
