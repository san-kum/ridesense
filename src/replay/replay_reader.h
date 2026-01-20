#pragma once

#include "capture/frame.h"
#include "common/logger.h"
#include <fstream>
#include <memory>
#include <optional>
#include <string>

namespace ridersense {

class ReplayReader {
public:
  explicit ReplayReader(const std::string &data_dir);
  ~ReplayReader();

  bool open();
  void close();

  std::optional<std::shared_ptr<ImuFrame>> read_next_imu();
  std::optional<std::shared_ptr<GpsFrame>> read_next_gps();

  bool has_more_imu() const;
  bool has_more_gps() const;

  std::optional<uint64_t> peek_next_imu_ts();
  std::optional<uint64_t> peek_next_gps_ts();

  void reset();

private:
  std::string data_dir_;
  std::shared_ptr<spdlog::logger> logger_;

  std::ifstream imu_stream_;
  std::ifstream gps_stream_;

  std::optional<uint64_t> cached_imu_ts_;
  std::optional<uint64_t> cached_gps_ts_;
};

} // namespace ridersense
