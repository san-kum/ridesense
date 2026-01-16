#include "capture/camera_stream.h"
#include "capture/frame.h"
#include "capture/stream.h"
#include "common/logger.h"
#include <chrono>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <thread>

namespace ridersense {

CameraStream::CameraStream(int device_id, int width, int height, int fps)
    : Stream("camera_stream"), device_id_(device_id), width_(width),
      height_(height), fps_(fps), logger_(Logger::get("camera")) {}

CameraStream::~CameraStream() {
  if (running_)
    stop();
}

bool CameraStream::init() {
  LOG_INFO(logger_, "opening camera device {}", device_id_);
  if (!capture_.open(device_id_)) {
    LOG_ERROR(logger_, "failed to open camera {}", device_id_);
    return false;
  }

  capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  capture_.set(cv::CAP_PROP_FPS, fps_);

  int actual_w = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
  int actual_h = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
  int actual_fps = capture_.get(cv::CAP_PROP_FPS);

  LOG_INFO(logger_, "camera opened: {}x{} @ {}fps", actual_w, actual_h,
           actual_fps);
  return true;
}

bool CameraStream::start() {
  if (running_)
    return true;

  running_ = true;
  capture_thread_ = std::thread(&CameraStream::capture_loop, this);
  LOG_INFO(logger_, "camera capture started");
  return true;
}

bool CameraStream::stop() {
  if (!running_)
    return true;

  LOG_INFO(logger_, "stopping camera capture");
  running_ = false;

  if (capture_thread_.joinable())
    capture_thread_.join();

  return true;
}

bool CameraStream::shutdown() {
  stop();
  if (capture_.isOpened()) {
    capture_.release();
    LOG_INFO(logger_, "camera released");
  }
  return true;
}

void CameraStream::capture_loop() {
  cv::Mat frame;

  while (running_) {
    if (!capture_.read(frame)) {
      LOG_WARN(logger_, "failed to read frame");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto img_frame = std::make_shared<ImageFrame>(
        std::chrono::high_resolution_clock::now(), frame_count_++, device_id_);

    img_frame->image = frame.clone();
    publish(img_frame);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps_));
  }

  LOG_ERROR(logger_, "capture loop exited, {} frames captured", frame_count_);
}

} // namespace ridersense
