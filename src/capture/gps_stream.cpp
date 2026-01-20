#include "capture/gps_stream.h"
#include <cmath>

namespace ridersense {

GpsStream::GpsStream(int rate_hz)
    : Stream("gps_stream"), rate_hz_(rate_hz), logger_(Logger::get("gps")),
      rng_(std::random_device{}()) {}

GpsStream::~GpsStream() {
  if (running_) {
    stop();
  }
}

bool GpsStream::init() {
  LOG_INFO(logger_, "GPS stream initialized (mock mode)");
  return true;
}

bool GpsStream::start() {
  if (running_) {
    return true;
  }

  running_ = true;
  thread_ = std::thread(&GpsStream::generate_loop, this);

  LOG_INFO(logger_, "GPS stream started at {} Hz", rate_hz_);
  return true;
}

bool GpsStream::stop() {
  if (!running_)
    return true;

  LOG_INFO(logger_, "stopping GPS stream");
  running_ = false;

  if (thread_.joinable())
    thread_.join();

  return true;
}

bool GpsStream::shutdown() {
  stop();
  LOG_INFO(logger_, "GPS stream shutdown complete");
  return true;
}

void GpsStream::generate_loop() {
  auto interval = std::chrono::milliseconds(1000 / rate_hz_);
  auto next_time = std::chrono::high_resolution_clock::now();

  while (running_) {
    auto frame = std::make_shared<GpsFrame>(
        std::chrono::high_resolution_clock::now(), sample_count_++);

    double t = sample_count_ * 0.01;
    frame->latitude = lat_base_ + 0.001 * std::sin(t) + noise_(rng_);
    frame->longitude = lon_base_ + 0.001 * std::cos(t) + noise_(rng_);
    frame->altitude = 50.0 + noise_(rng_) * 10;
    frame->speed = 15.0;
    frame->heading = std::fmod(t * 10, 360.0);
    frame->num_satellites = 12;

    publish(frame);

    next_time += interval;
    std::this_thread::sleep_until(next_time);
  }

  LOG_INFO(logger_, "GPS loop exited, {} samples", sample_count_);
}

} // namespace ridersense
