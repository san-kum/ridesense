#pragma once

#include "../capture/stream.h"
#include "../common/logger.h"
#include <atomic>
#include <cstdint>
#include <memory>
#include <opencv2/videoio.hpp>
#include <spdlog/logger.h>

namespace ridersense {
class CameraStream : public Stream {
public:
  CameraStream(int device_id, int width, int height, int fps);
  ~CameraStream() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

private:
  void capture_loop();
  int device_id_;
  int width_;
  int height_;
  int fps_;

  cv::VideoCapture capture_;
  std::shared_ptr<spdlog::logger> logger_;

  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  uint64_t frame_count_{0};
};

} // namespace ridersense
