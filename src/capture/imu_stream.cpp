#include "capture/imu_stream.h"

namespace ridersense {

ImuStream::ImuStream(int rate_hz)
    : Stream("imu_stream"), rate_hz_(rate_hz), logger_(Logger::get("imu")),
      rng_(std::random_device{}()) {}

ImuStream::~ImuStream() {
  if (running_) {
    stop();
  }
}

bool ImuStream::init() {
  LOG_INFO(logger_, "IMU stream initialized (mock mode)");
  return true;
}

bool ImuStream::start() {
  if (running_) {
    return true;
  }

  running_ = true;
  thread_ = std::thread(&ImuStream::generate_loop, this);

  LOG_INFO(logger_, "IMU stream started at {} Hz", rate_hz_);
  return true;
}

bool ImuStream::stop() {
  if (!running_) {
    return true;
  }

  LOG_INFO(logger_, "stopping IMU stream");
  running_ = false;

  if (thread_.joinable()) {
    thread_.join();
  }
  return true;
}

bool ImuStream::shutdown() {
  stop();
  LOG_INFO(logger_, "IMU stream shutdown");
  return true;
}

void ImuStream::generate_loop() {
  auto interval = std::chrono::microseconds(1000000 / rate_hz_);
  auto next_time = std::chrono::high_resolution_clock::now();

  while (running_) {
    auto frame = std::make_shared<ImuFrame>(
        std::chrono::high_resolution_clock::now(), sample_count_++);

    frame->accel_x = noise_(rng_);
    frame->accel_y = noise_(rng_);
    frame->accel_z = -9.81 + noise_(rng_);

    frame->gyro_x = noise_(rng_) * 0.01;
    frame->gyro_y = noise_(rng_) * 0.01;
    frame->gyro_z = noise_(rng_) * 0.01;

    publish(frame);

    next_time += interval;
    std::this_thread::sleep_until(next_time);
  }

  LOG_INFO(logger_, "IMU loop exited, {} samples", sample_count_);
}

} // namespace ridersense
