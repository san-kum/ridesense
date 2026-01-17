#include "sync/timestamp_sync.h"
#include <chrono>
#include <cmath>
#include <mutex>
#include <numeric>

namespace ridersense {
TimestampSync::TimestampSync() {}

void TimestampSync::register_stream(const std::string &stream_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  streams_[stream_id] = StreamSync{};
}

void TimestampSync::update_offset(const std::string &stream_id, Timestamp local,
                                  Timestamp master) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = streams_.find(stream_id);
  if (it == streams_.end())
    return;

  auto &sync = it->second;

  auto offset = std::chrono::duration_cast<std::chrono::nanoseconds>(
      master.time_since_epoch() - local.time_since_epoch());
  sync.offset_history.push_back(offset);
  if (sync.offset_history.size() > StreamSync::HISTORY_SIZE)
    sync.offset_history.pop_front();

  if (!sync.offset_history.empty()) {
    auto sum =
        std::accumulate(sync.offset_history.begin(), sync.offset_history.end(),
                        std::chrono::nanoseconds{0});
    sync.current_offset = sum / sync.offset_history.size();
  }
}

Timestamp TimestampSync::sync_timestamp(const std::string &stream_id,
                                        Timestamp local) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = streams_.find(stream_id);
  if (it == streams_.end())
    return local;

  return local + it->second.current_offset;
}

std::chrono::nanoseconds
TimestampSync::get_offset(const std::string &stream_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = streams_.find(stream_id);
  if (it == streams_.end())
    return std::chrono::nanoseconds{0};

  return it->second.current_offset;
}

TimestampSync::Stats TimestampSync::get_stats(const std::string &stream_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  Stats stats{0.0, 0.0, 0};
  auto it = streams_.find(stream_id);
  if (it == streams_.end() || it->second.offset_history.empty())
    return stats;

  auto &history = it->second.offset_history;
  stats.sample_count = history.size();

  auto sum = std::accumulate(history.begin(), history.end(),
                             std::chrono::nanoseconds{0});
  stats.mean_offset_ns = static_cast<double>(sum.count()) / history.size();

  double variance = 0.0;
  for (const auto &offset : history) {
    double diff = offset.count() - stats.mean_offset_ns;
    variance += diff * diff;
  }
  stats.std_offset_ns = std::sqrt(variance / history.size());
  return stats;
}

} // namespace ridersense
