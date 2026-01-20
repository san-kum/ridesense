#include "replay/replay_reader.h"
#include <filesystem>

namespace ridersense {

ReplayReader::ReplayReader(const std::string &data_dir)
    : data_dir_(data_dir), logger_(Logger::get("replay")) {}

ReplayReader::~ReplayReader() { close(); }

bool ReplayReader::open() {
  std::string imu_path = data_dir_ + "/imu.bin";
  std::string gps_path = data_dir_ + "/gps.bin";

  if (!std::filesystem::exists(imu_path)) {
    LOG_WARN(logger_, "imu.bin not found: {}", imu_path);
  } else {
    imu_stream_.open(imu_path, std::ios::binary);
    if (!imu_stream_) {
      LOG_ERROR(logger_, "failed to open imu.bin");
      return false;
    }
  }

  if (!std::filesystem::exists(gps_path)) {
    LOG_WARN(logger_, "gps.bin not found: {}", gps_path);
  } else {
    gps_stream_.open(gps_path, std::ios::binary);
    if (!gps_stream_) {
      LOG_ERROR(logger_, "failed to open gps.bin");
      return false;
    }
  }

  LOG_INFO(logger_, "replay reader opened: {}", data_dir_);
  return true;
}

void ReplayReader::close() {
  if (imu_stream_.is_open()) {
    imu_stream_.close();
  }
  if (gps_stream_.is_open()) {
    gps_stream_.close();
  }
  cached_imu_ts_.reset();
  cached_gps_ts_.reset();
}

void ReplayReader::reset() {
  if (imu_stream_.is_open()) {
    imu_stream_.clear();
    imu_stream_.seekg(0);
  }
  if (gps_stream_.is_open()) {
    gps_stream_.clear();
    gps_stream_.seekg(0);
  }
  cached_imu_ts_.reset();
  cached_gps_ts_.reset();
}

bool ReplayReader::has_more_imu() const {
  return imu_stream_.is_open() && imu_stream_.good() && !imu_stream_.eof();
}

bool ReplayReader::has_more_gps() const {
  return gps_stream_.is_open() && gps_stream_.good() && !gps_stream_.eof();
}

std::optional<uint64_t> ReplayReader::peek_next_imu_ts() {
  if (cached_imu_ts_) {
    return cached_imu_ts_;
  }

  if (!has_more_imu()) {
    return std::nullopt;
  }

  uint64_t ts;
  auto pos = imu_stream_.tellg();
  imu_stream_.read(reinterpret_cast<char *>(&ts), sizeof(ts));

  if (imu_stream_.gcount() < static_cast<std::streamsize>(sizeof(ts))) {
    return std::nullopt;
  }

  imu_stream_.seekg(pos); 
  cached_imu_ts_ = ts;
  return ts;
}

std::optional<uint64_t> ReplayReader::peek_next_gps_ts() {
  if (cached_gps_ts_) {
    return cached_gps_ts_;
  }

  if (!has_more_gps()) {
    return std::nullopt;
  }

  uint64_t ts;
  auto pos = gps_stream_.tellg();
  gps_stream_.read(reinterpret_cast<char *>(&ts), sizeof(ts));

  if (gps_stream_.gcount() < static_cast<std::streamsize>(sizeof(ts))) {
    return std::nullopt;
  }

  gps_stream_.seekg(pos);
  cached_gps_ts_ = ts;
  return ts;
}

std::optional<std::shared_ptr<ImuFrame>> ReplayReader::read_next_imu() {
  if (!has_more_imu()) {
    return std::nullopt;
  }

  uint64_t ts;
  uint64_t seq;

  imu_stream_.read(reinterpret_cast<char *>(&ts), sizeof(ts));
  imu_stream_.read(reinterpret_cast<char *>(&seq), sizeof(seq));

  if (imu_stream_.gcount() < static_cast<std::streamsize>(sizeof(seq))) {
    return std::nullopt;
  }

  auto frame = std::make_shared<ImuFrame>(from_nanoseconds(ts), seq);

  imu_stream_.read(reinterpret_cast<char *>(&frame->accel_x), sizeof(double));
  imu_stream_.read(reinterpret_cast<char *>(&frame->accel_y), sizeof(double));
  imu_stream_.read(reinterpret_cast<char *>(&frame->accel_z), sizeof(double));
  imu_stream_.read(reinterpret_cast<char *>(&frame->gyro_x), sizeof(double));
  imu_stream_.read(reinterpret_cast<char *>(&frame->gyro_y), sizeof(double));
  imu_stream_.read(reinterpret_cast<char *>(&frame->gyro_z), sizeof(double));

  cached_imu_ts_.reset();
  return frame;
}

std::optional<std::shared_ptr<GpsFrame>> ReplayReader::read_next_gps() {
  if (!has_more_gps()) {
    return std::nullopt;
  }

  uint64_t ts;
  uint64_t seq;

  gps_stream_.read(reinterpret_cast<char *>(&ts), sizeof(ts));
  gps_stream_.read(reinterpret_cast<char *>(&seq), sizeof(seq));

  if (gps_stream_.gcount() < static_cast<std::streamsize>(sizeof(seq))) {
    return std::nullopt;
  }

  auto frame = std::make_shared<GpsFrame>(from_nanoseconds(ts), seq);

  gps_stream_.read(reinterpret_cast<char *>(&frame->latitude), sizeof(double));
  gps_stream_.read(reinterpret_cast<char *>(&frame->longitude), sizeof(double));
  gps_stream_.read(reinterpret_cast<char *>(&frame->altitude), sizeof(double));
  gps_stream_.read(reinterpret_cast<char *>(&frame->speed), sizeof(double));
  gps_stream_.read(reinterpret_cast<char *>(&frame->heading), sizeof(double));
  gps_stream_.read(reinterpret_cast<char *>(&frame->num_satellites),
                   sizeof(int));

  cached_gps_ts_.reset();
  return frame;
}

} // namespace ridersense
