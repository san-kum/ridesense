#include "capture/camera_stream.h"
#include "capture/gps_stream.h"
#include "capture/imu_stream.h"
#include "common/config.h"
#include "common/logger.h"
#include "services/service_manager.h"
#include "storage/frame_logger.h"
#include "sync/time_aligner.h"
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
    if (!std::filesystem::exists(config_path)) {
      config_path = "../config/default.yaml";
    }
  }

  try {
    Config::instance().load(config_path);
  } catch (const std::exception &e) {
    std::cerr << "Config error: " << e.what() << std::endl;
    return 1;
  }

  Logger::init(Config::instance().log_level(), Config::instance().log_dir());

  auto logger = Logger::get("main");
  LOG_INFO(logger, "ridersense starting");
  LOG_INFO(logger, "config loaded from: {}", config_path);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  ServiceManager service_mgr;

  std::shared_ptr<FrameLogger> frame_logger;
  if (Config::instance().service_enabled("frame_logger")) {
    auto data_dir =
        Config::instance().root()["system"]["data_dir"].as<std::string>();
    frame_logger = std::make_shared<FrameLogger>(data_dir);
    service_mgr.add(frame_logger);
  }

  std::shared_ptr<TimeAligner> time_aligner;
  if (Config::instance().service_enabled("time_aligner")) {
    auto sync_cfg = Config::instance().root()["sync"];
    int max_diff_ms = sync_cfg["max_time_diff_ms"].as<int>(50);
    int buffer_size = sync_cfg["buffer_size"].as<int>(100);

    time_aligner = std::make_shared<TimeAligner>(
        std::chrono::milliseconds(max_diff_ms), buffer_size);

    if (frame_logger) {
      time_aligner->set_callback([frame_logger](const SyncedFrames &bundle) {
        frame_logger->log_frame(bundle.image);
        frame_logger->log_frame(bundle.imu);
        frame_logger->log_frame(bundle.gps);
      });
    }

    service_mgr.add(time_aligner);
  }

  if (Config::instance().service_enabled("camera")) {
    auto cam_cfg = Config::instance().root()["services"]["camera"];
    int device_id = cam_cfg["device_id"].as<int>(0);
    int width = cam_cfg["width"].as<int>(640);
    int height = cam_cfg["height"].as<int>(480);
    int fps = cam_cfg["fps"].as<int>(30);

    auto camera = std::make_shared<CameraStream>(device_id, width, height, fps);

    if (time_aligner) {
      camera->set_callback([time_aligner](auto frame) {
        auto img = std::static_pointer_cast<ImageFrame>(frame);
        time_aligner->on_image(img);
      });
    } else if (frame_logger) {
      camera->set_callback(
          [frame_logger](auto frame) { frame_logger->log_frame(frame); });
    }

    service_mgr.add(camera);
  }

  if (Config::instance().service_enabled("imu")) {
    auto imu_cfg = Config::instance().root()["services"]["imu"];
    int rate_hz = imu_cfg["rate_hz"].as<int>(100);

    auto imu = std::make_shared<ImuStream>(rate_hz);

    if (time_aligner) {
      imu->set_callback([time_aligner](auto frame) {
        auto imu_frame = std::static_pointer_cast<ImuFrame>(frame);
        time_aligner->on_imu(imu_frame);
      });
    } else if (frame_logger) {
      imu->set_callback(
          [frame_logger](auto frame) { frame_logger->log_frame(frame); });
    }

    service_mgr.add(imu);
  }

  if (Config::instance().service_enabled("gps")) {
    auto gps_cfg = Config::instance().root()["services"]["gps"];
    int rate_hz = gps_cfg["rate_hz"].as<int>(10);

    auto gps = std::make_shared<GpsStream>(rate_hz);

    if (time_aligner) {
      gps->set_callback([time_aligner](auto frame) {
        auto gps_frame = std::static_pointer_cast<GpsFrame>(frame);
        time_aligner->on_gps(gps_frame);
      });
    } else if (frame_logger) {
      gps->set_callback(
          [frame_logger](auto frame) { frame_logger->log_frame(frame); });
    }

    service_mgr.add(gps);
  }

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

  auto last_report = std::chrono::steady_clock::now();
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_report)
            .count() >= 5) {
      if (time_aligner) {
        auto stats = time_aligner->get_stats();
        LOG_INFO(logger, "sync stats - bundles: {}, images: {}, dropped: {}",
                 stats.bundles_emitted, stats.images_received,
                 stats.images_dropped);
      }
      last_report = now;
    }
  }

  LOG_INFO(logger, "Shutting down...");
  service_mgr.stop_all();
  service_mgr.shutdown_all();

  LOG_INFO(logger, "ridersense stopped");
  return 0;
}
