#include "storage/frame_logger.h"
#include <filesystem>
#include <opencv2/imgcodecs.hpp>

namespace ridersense {

FrameLogger::FrameLogger(const std::string &output_dir)
    : Service("frame_logger"), output_dir_(output_dir),
      logger_(Logger::get("storage")) {}

FrameLogger::~FrameLogger() {
  if (running_) {
    stop();
  }
}

bool FrameLogger::init() {
  std::filesystem::create_directories(output_dir_);

  std::string image_path = output_dir_ + "/images.bin";
  std::string imu_path = output_dir_ + "/imu.bin";
  std::string gps_path = output_dir_ + "/gps.bin";

  image_stream_.open(image_path, std::ios::binary);
  imu_stream_.open(imu_path, std::ios::binary);
  gps_stream_.open(gps_path, std::ios::binary);

  if (!image_stream_ || !imu_stream_ || !gps_stream_) {
    LOG_ERROR(logger_, "failed to open output streams");
    return false;
  }

  LOG_INFO(logger_, "frame logger initialized: {}", output_dir_);
  return true;
}

bool FrameLogger::start() {
  if (running_) {
    return true;
  }

  running_ = true;
  writer_thread_ = std::thread(&FrameLogger::writer_loop, this);

  LOG_INFO(logger_, "frame logger started");
  return true;
}

bool FrameLogger::stop() {
  if (!running_) {
    return true;
  }

  LOG_INFO(logger_, "stopping frame logger");
  running_ = false;
  queue_cv_.notify_all();

  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  LOG_INFO(logger_, "wrote {} frames total", frames_written_);
  return true;
}

bool FrameLogger::shutdown() {
  stop();
  image_stream_.close();
  imu_stream_.close();
  gps_stream_.close();

  LOG_INFO(logger_, "frame logger shutdown");
  return true;
}

void FrameLogger::log_frame(std::shared_ptr<Frame> frame) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  queue_.push(frame);
  queue_cv_.notify_one();
}

void FrameLogger::writer_loop() {
  while (running_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(lock, [this] { return !queue_.empty() || !running_; });

    while (!queue_.empty()) {
      auto frame = queue_.front();
      queue_.pop();
      lock.unlock();

      switch (frame->type) {
      case FrameType::IMAGE:
        write_image_frame(static_cast<ImageFrame *>(frame.get()));
        break;
      case FrameType::IMU:
        write_imu_frame(static_cast<ImuFrame *>(frame.get()));
        break;
      case FrameType::GPS:
        write_gps_frame(static_cast<GpsFrame *>(frame.get()));
        break;
      default:
        break;
      }

      frames_written_++;

      lock.lock();
    }
  }
}

void FrameLogger::write_image_frame(const ImageFrame *frame) {
  uint64_t ts = to_nanoseconds(frame->timestamp);

  image_stream_.write(reinterpret_cast<const char *>(&ts), sizeof(ts));
  image_stream_.write(reinterpret_cast<const char *>(&frame->sequence),
                      sizeof(frame->sequence));

  int rows = frame->image.rows;
  int cols = frame->image.cols;
  int type = frame->image.type();

  image_stream_.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
  image_stream_.write(reinterpret_cast<const char *>(&cols), sizeof(cols));
  image_stream_.write(reinterpret_cast<const char *>(&type), sizeof(type));

  size_t data_size = frame->image.total() * frame->image.elemSize();
  image_stream_.write(reinterpret_cast<const char *>(frame->image.data),
                      data_size);
}

void FrameLogger::write_imu_frame(const ImuFrame *frame) {
  uint64_t ts = to_nanoseconds(frame->timestamp);

  imu_stream_.write(reinterpret_cast<const char *>(&ts), sizeof(ts));
  imu_stream_.write(reinterpret_cast<const char *>(&frame->sequence),
                    sizeof(frame->sequence));
  imu_stream_.write(reinterpret_cast<const char *>(&frame->accel_x),
                    sizeof(double));
  imu_stream_.write(reinterpret_cast<const char *>(&frame->accel_y),
                    sizeof(double));
  imu_stream_.write(reinterpret_cast<const char *>(&frame->accel_z),
                    sizeof(double));
  imu_stream_.write(reinterpret_cast<const char *>(&frame->gyro_x),
                    sizeof(double));
  imu_stream_.write(reinterpret_cast<const char *>(&frame->gyro_y),
                    sizeof(double));
  imu_stream_.write(reinterpret_cast<const char *>(&frame->gyro_z),
                    sizeof(double));
}

void FrameLogger::write_gps_frame(const GpsFrame *frame) {
  uint64_t ts = to_nanoseconds(frame->timestamp);

  gps_stream_.write(reinterpret_cast<const char *>(&ts), sizeof(ts));
  gps_stream_.write(reinterpret_cast<const char *>(&frame->sequence),
                    sizeof(frame->sequence));
  gps_stream_.write(reinterpret_cast<const char *>(&frame->latitude),
                    sizeof(double));
  gps_stream_.write(reinterpret_cast<const char *>(&frame->longitude),
                    sizeof(double));
  gps_stream_.write(reinterpret_cast<const char *>(&frame->altitude),
                    sizeof(double));
  gps_stream_.write(reinterpret_cast<const char *>(&frame->speed),
                    sizeof(double));
  gps_stream_.write(reinterpret_cast<const char *>(&frame->heading),
                    sizeof(double));
  gps_stream_.write(reinterpret_cast<const char *>(&frame->num_satellites),
                    sizeof(int));
}

} // namespace ridersense
