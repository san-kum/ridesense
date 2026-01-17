#include "../sync/time_aligner.h"

namespace ridersense {

TimeAligner::TimeAligner(std::chrono::milliseconds max_time_diff,
                         size_t buffer_size)
    : Service("time_aligner"), max_time_diff_(max_time_diff),
      buffer_size_(buffer_size), logger_(Logger::get("sync")),
      imu_resampler_(buffer_size), gps_resampler_(buffer_size) {}

TimeAligner::~TimeAligner() {}

bool TimeAligner::init() {
  ts_sync_.register_stream("camera");
  ts_sync_.register_stream("imu");
  ts_sync_.register_stream("gps");

  LOG_INFO(logger_, "time aligner initialized");
  LOG_INFO(logger_, "max time diff: {} ms", max_time_diff_.count());
  LOG_INFO(logger_, "buffer size: {}", buffer_size_);

  return true;
}

bool TimeAligner::start() {
  LOG_INFO(logger_, "time aligner started");
  return true;
}

bool TimeAligner::stop() {
  LOG_INFO(logger_, "stopping time aligner");
  LOG_INFO(logger_,
           "Stats - images: {}, IMU: {}, GPS: {}, Bundles: {}, Dropped: {}",
           stats_.images_received, stats_.imu_received, stats_.gps_received,
           stats_.bundles_emitted, stats_.images_dropped);
  return true;
}

bool TimeAligner::shutdown() {
  LOG_INFO(logger_, "time aligner shutdown");
  return true;
}

void TimeAligner::on_image(std::shared_ptr<ImageFrame> frame) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);

  stats_.images_received++;

  image_buffer_.push_back(frame);

  if (image_buffer_.size() > buffer_size_) {
    image_buffer_.pop_front();
    stats_.images_dropped++;
  }

  try_emit();
}

void TimeAligner::on_imu(std::shared_ptr<ImuFrame> frame) {
  stats_.imu_received++;

  imu_resampler_.add_sample(frame);
}

void TimeAligner::on_gps(std::shared_ptr<GpsFrame> frame) {
  stats_.gps_received++;

  gps_resampler_.add_sample(frame);
}

void TimeAligner::try_emit() {

  if (image_buffer_.empty()) {
    return;
  }

  auto img = image_buffer_.front();
  Timestamp target_time = img->timestamp;

  auto imu = imu_resampler_.interpolate_at(target_time);
  auto gps = gps_resampler_.interpolate_at(target_time);

  if (imu && gps) {
    SyncedFrames bundle;
    bundle.timestamp = target_time;
    bundle.image = img;
    bundle.imu = *imu;
    bundle.gps = *gps;

    image_buffer_.pop_front();
    stats_.bundles_emitted++;

    if (callback_) {
      callback_(bundle);
    }
  } else {
    if (image_buffer_.size() > buffer_size_ / 2) {
      LOG_WARN(logger_, "dropping old image frame, no matching sensor data");
      image_buffer_.pop_front();
      stats_.images_dropped++;
    }
  }
}

TimeAligner::Stats TimeAligner::get_stats() const { return stats_; }

} // namespace ridersense
