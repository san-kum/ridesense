#pragma once

#include "../capture/frame.h"
#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <string>

namespace ridersense {

class TimestampSync {
public:
  TimestampSync();

  void register_stream(const std::string &stream_id);
  void update_offset(const std::string &stream_id, Timestamp local,
                     Timestamp master);

  Timestamp sync_timestamp(const std::string &stream_id, Timestamp local);

  std::chrono::nanoseconds get_offset(const std::string &stream_id);

  struct Stats {
    double mean_offset_ns;
    double std_offset_ns;
    size_t sample_count;
  };

  Stats get_stats(const std::string &stream_id);

private:
  struct StreamSync {
    std::deque<std::chrono::nanoseconds> offset_history;
    std::chrono::nanoseconds current_offset{0};
    static constexpr size_t HISTORY_SIZE = 100;
  };

  std::map<std::string, StreamSync> streams_;
  std::mutex mutex_;
};

} // namespace ridersense
