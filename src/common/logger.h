#pragma once

#include <memory>
#include <spdlog/spdlog.h>
#include <string>

// MACROS
#define LOG_TRACE(logger, ...) logger->trace(__VA_ARGS__)
#define LOG_DEBUG(logger, ...) logger->debug(__VA_ARGS__)
#define LOG_INFO(logger, ...) logger->info(__VA_ARGS__)
#define LOG_WARN(logger, ...) logger->warn(__VA_ARGS__)
#define LOG_ERROR(logger, ...) logger->error(__VA_ARGS__)

namespace ridersense {
class Logger {
public:
  static void init(const std::string &level, const std::string &log_dir);
  static std::shared_ptr<spdlog::logger> get(const std::string &name);

private:
  static bool initialized_;
};
} // namespace ridersense
