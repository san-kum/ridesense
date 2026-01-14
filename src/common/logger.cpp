#include "../common/logger.h"
#include <filesystem>
#include <memory>
#include <spdlog/common.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/ansicolor_sink.h>
#include <spdlog/spdlog-inl.h>
#include <vector>

namespace ridersense {

bool Logger::initialized_ = false;

void Logger::init(const std::string &level, const std::string &log_dir) {
  if (initialized_)
    return;

  std::filesystem::create_directories(log_dir);

  if (level == "trace")
    spdlog::set_level(spdlog::level::trace);
  if (level == "debug")
    spdlog::set_level(spdlog::level::debug);
  if (level == "info")
    spdlog::set_level(spdlog::level::info);
  if (level == "warn")
    spdlog::set_level(spdlog::level::warn);
  if (level == "error")
    spdlog::set_level(spdlog::level::err);

  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%t] [%^%l%$] [%n] %v");

  initialized_ = true;
}

std::shared_ptr<spdlog::logger> Logger::get(const std::string &name) {
  auto logger = spdlog::get(name);
  if (!logger) {
    auto console_sink =
        std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>();

    std::vector<spdlog::sink_ptr> sinks{console_sink};
    logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    spdlog::register_logger(logger);
  }

  return logger;
}

} // namespace ridersense
