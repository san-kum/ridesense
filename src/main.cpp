#include "common/config.h"
#include "common/logger.h"
#include "services/service_manager.h"
#include <atomic>
#include <csignal>
#include <filesystem>
#include <iostream>

namespace {
std::atomic<bool> running{true};

void signal_handler(int signal) { running = false; }
} // namespace

int main(int argc, char **argv) {
  using namespace ridersense;

  std::string config_path = "config/default.yaml";
  if (argc > 1) {
    config_path = argv[1];
  } else {
    if (!std::filesystem::exists(config_path))
      config_path = "../config/default.yaml";
  }

  try {
    Config::instance().load(config_path);
  } catch (const std::exception &e) {
    std::cerr << "config error: " << e.what() << std::endl;
    return 1;
  }

  Logger::init(Config::instance().log_level(), Config::instance().log_dir());

  auto logger = Logger::get("main");
  LOG_INFO(logger, "ridersense starting");
  LOG_INFO(logger, "config loaded from: {}", config_path);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  ServiceManager service_mgr;

  // TODO: add services here when implemented

  if (!service_mgr.init_all()) {
    LOG_ERROR(logger, "service initialization failed");
    return 1;
  }

  if (!service_mgr.start_all()) {
    LOG_ERROR(logger, "service start failed");
    service_mgr.shutdown_all();
    return 1;
  }

  LOG_INFO(logger, "all services running");

  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  LOG_INFO(logger, "shutting down...");
  service_mgr.stop_all();
  service_mgr.shutdown_all();

  LOG_INFO(logger, "ridersense stopped");
  return 0;
}
