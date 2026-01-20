#include "capture/camera_stream.h"
#include "capture/gps_stream.h"
#include "capture/imu_stream.h"
#include "common/config.h"
#include "common/logger.h"
#include "services/service_manager.h"
#include "storage/frame_logger.h"
#include "sync/time_aligner.h"

#include "perception/lane_detector.h"
#include "perception/object_detector.h"
#include "perception/slam_processor.h"

#include "fusion/pose_fusion.h"
#include "fusion/vehicle_state.h"

#include "analytics/event_detector.h"
#include "analytics/metrics_collector.h"

#include "network/state_broadcaster.h"
#include "network/state_server.h"
#include "network/video_server.h"

#include "replay/replay_controller.h"

#include <atomic>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> running{true};

void signal_handler(int signal) { running = false; }
} // namespace

int main(int argc, char **argv) {
  using namespace ridersense;

  std::string config_path = "config/default.yaml";
  std::string replay_dir = "";
  bool replay_mode = false;

  // parse args
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--replay" && i + 1 < argc) {
      replay_dir = argv[++i];
      replay_mode = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: ridersense [config.yaml] [--replay <data_dir>]\n";
      return 0;
    } else {
      config_path = arg;
    }
  }

  if (!std::filesystem::exists(config_path)) {
    config_path = "../config/default.yaml";
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

    service_mgr.add(time_aligner);
  }

  std::shared_ptr<SlamProcessor> slam_proc;
  std::shared_ptr<ObjectDetector> obj_detector;
  std::shared_ptr<LaneDetector> lane_detector;

  if (Config::instance().service_enabled("slam")) {
    auto slam_cfg = Config::instance().root()["perception"]["slam"];
    std::string vocab_path = slam_cfg["vocabulary_path"].as<std::string>();
    std::string mode = slam_cfg["mode"].as<std::string>();

    slam_proc = std::make_shared<SlamProcessor>(vocab_path, mode);
    service_mgr.add(slam_proc);
  }

  if (Config::instance().service_enabled("detector")) {
    auto det_cfg = Config::instance().root()["perception"]["detector"];
    std::string model_path = det_cfg["model_path"].as<std::string>();
    float conf_thresh = det_cfg["confidence_threshold"].as<float>();
    float nms_thresh = det_cfg["nms_threshold"].as<float>();
    int input_size = det_cfg["input_size"].as<int>();

    obj_detector = std::make_shared<ObjectDetector>(model_path, conf_thresh,
                                                    nms_thresh, input_size);
    service_mgr.add(obj_detector);
  }

  if (Config::instance().service_enabled("lane_detector")) {
    auto lane_cfg = Config::instance().root()["perception"]["lane"];
    float roi_top = lane_cfg["roi_top_ratio"].as<float>();
    int canny_low = lane_cfg["canny_low"].as<int>();
    int canny_high = lane_cfg["canny_high"].as<int>();
    int hough_thresh = lane_cfg["hough_threshold"].as<int>();

    lane_detector = std::make_shared<LaneDetector>(roi_top, canny_low,
                                                   canny_high, hough_thresh);
    service_mgr.add(lane_detector);
  }

  std::shared_ptr<PoseFusion> pose_fusion;
  std::shared_ptr<VehicleState> vehicle_state;

  if (Config::instance().service_enabled("pose_fusion")) {
    auto fusion_cfg = Config::instance().root()["fusion"]["pose_fusion"];

    pose_fusion = std::make_shared<PoseFusion>(
        fusion_cfg["process_noise_position"].as<double>(),
        fusion_cfg["process_noise_velocity"].as<double>(),
        fusion_cfg["process_noise_orientation"].as<double>(),
        fusion_cfg["slam_position_noise"].as<double>(),
        fusion_cfg["gps_position_noise"].as<double>(),
        fusion_cfg["imu_noise"].as<double>());

    service_mgr.add(pose_fusion);
  }

  if (Config::instance().service_enabled("vehicle_state")) {
    auto vs_cfg = Config::instance().root()["fusion"]["vehicle_state"];

    vehicle_state = std::make_shared<VehicleState>(
        vs_cfg["wheelbase"].as<double>(), vs_cfg["cg_height"].as<double>(),
        vs_cfg["max_lean_angle"].as<double>());

    service_mgr.add(vehicle_state);
  }

  // analytics
  std::shared_ptr<EventDetector> event_detector;
  std::shared_ptr<MetricsCollector> metrics_collector;

  if (Config::instance().service_enabled("event_detector")) {
    auto ev_cfg = Config::instance().root()["analytics"]["event_detector"];
    EventDetectorConfig cfg;
    cfg.hard_brake_threshold = ev_cfg["hard_brake_threshold"].as<double>(-6.0);
    cfg.hard_accel_threshold = ev_cfg["hard_accel_threshold"].as<double>(4.0);
    cfg.max_lean_threshold = ev_cfg["max_lean_threshold"].as<double>(45.0);
    cfg.close_object_distance = ev_cfg["close_object_distance"].as<double>(5.0);
    cfg.high_slip_threshold = ev_cfg["high_slip_threshold"].as<double>(10.0);
    cfg.cooldown_seconds = ev_cfg["cooldown_seconds"].as<double>(2.0);

    event_detector = std::make_shared<EventDetector>(cfg);
    service_mgr.add(event_detector);
  }

  if (Config::instance().service_enabled("metrics_collector")) {
    auto met_cfg = Config::instance().root()["analytics"]["metrics"];
    double window_sec = met_cfg["window_seconds"].as<double>(60.0);

    metrics_collector = std::make_shared<MetricsCollector>(window_sec);
    service_mgr.add(metrics_collector);

    // wire event detector to metrics
    if (event_detector) {
      event_detector->set_callback([metrics_collector](const Event &ev) {
        metrics_collector->on_event(ev);
      });
    }
  }

  // network
  std::shared_ptr<StateServer> state_server;
  std::shared_ptr<StateBroadcaster> state_broadcaster;

  if (Config::instance().service_enabled("state_server")) {
    auto net_cfg = Config::instance().root()["network"]["state_server"];
    int port = net_cfg["port"].as<int>(9000);
    int max_clients = net_cfg["max_clients"].as<int>(10);

    state_server = std::make_shared<StateServer>(port, max_clients);
    service_mgr.add(state_server);
  }

  if (Config::instance().service_enabled("state_broadcaster") && state_server) {
    auto bc_cfg = Config::instance().root()["network"]["broadcaster"];
    double rate_hz = bc_cfg["rate_hz"].as<double>(10.0);

    state_broadcaster = std::make_shared<StateBroadcaster>(state_server, rate_hz);
    service_mgr.add(state_broadcaster);

    // wire event detector to broadcaster
    if (event_detector) {
      event_detector->set_callback([metrics_collector, state_broadcaster](const Event &ev) {
        if (metrics_collector) {
          metrics_collector->on_event(ev);
        }
        if (state_broadcaster) {
          state_broadcaster->on_event(ev);
        }
      });
    }

    // wire metrics to broadcaster
    if (metrics_collector) {
      metrics_collector->set_callback([state_broadcaster](const RidingMetrics &m) {
        if (state_broadcaster) {
          state_broadcaster->on_metrics(m);
        }
      });
    }
  }

  // video server
  std::shared_ptr<VideoServer> video_server;
  if (Config::instance().service_enabled("video_server")) {
    auto vid_cfg = Config::instance().root()["network"]["video_server"];
    int port = vid_cfg["port"].as<int>(9002);
    int quality = vid_cfg["quality"].as<int>(80);

    video_server = std::make_shared<VideoServer>(port, quality);
    service_mgr.add(video_server);
  }

  if (time_aligner) {
    time_aligner->set_callback([&](const SyncedFrames &bundle) {
      std::shared_ptr<PoseFrame> pose;
      std::shared_ptr<DetectionFrame> detections;
      std::shared_ptr<LaneFrame> lanes;

      if (slam_proc) {
        pose = slam_proc->process(bundle.image, bundle.imu);
      }

      if (obj_detector) {
        detections = obj_detector->detect(bundle.image);
      }

      if (lane_detector) {
        lanes = lane_detector->detect(bundle.image);
      }

      std::shared_ptr<FusionFrame> fusion_result;

      if (pose_fusion) {
        fusion_result = pose_fusion->fuse(pose, bundle.imu, bundle.gps);

        if (vehicle_state) {
          vehicle_state->estimate(fusion_result);
        }

        // analytics processing
        if (event_detector) {
          event_detector->process(fusion_result, detections, lanes);
        }
        if (metrics_collector) {
          metrics_collector->on_fusion(fusion_result);
        }
        // broadcast state
        if (state_broadcaster) {
          state_broadcaster->on_fusion(fusion_result, bundle.gps);
        }
      }

      if (frame_logger) {
        frame_logger->log_frame(bundle.image);
        frame_logger->log_frame(bundle.imu);
        frame_logger->log_frame(bundle.gps);

        if (pose)
          frame_logger->log_frame(pose);
        if (detections)
          frame_logger->log_frame(detections);
        if (lanes)
          frame_logger->log_frame(lanes);
        if (fusion_result)
          frame_logger->log_frame(fusion_result);
      }
    });
  }

  // replay mode or live sensors
  std::shared_ptr<ReplayController> replay_controller;

  if (replay_mode) {
    LOG_INFO(logger, "REPLAY MODE: {}", replay_dir);

    replay_controller = std::make_shared<ReplayController>(replay_dir, 1.0);
    service_mgr.add(replay_controller);

    // wire replay to time aligner
    if (time_aligner) {
      replay_controller->set_imu_callback([time_aligner](auto frame) {
        time_aligner->on_imu(frame);
      });
      replay_controller->set_gps_callback([time_aligner](auto frame) {
        time_aligner->on_gps(frame);
      });
    }
  } else {
    // live sensor streams
    if (Config::instance().service_enabled("camera")) {
      auto cam_cfg = Config::instance().root()["services"]["camera"];
      int device_id = cam_cfg["device_id"].as<int>(0);
      int width = cam_cfg["width"].as<int>(640);
      int height = cam_cfg["height"].as<int>(480);
      int fps = cam_cfg["fps"].as<int>(30);

      auto camera = std::make_shared<CameraStream>(device_id, width, height, fps);

      camera->set_callback([time_aligner, frame_logger, video_server](auto frame) {
        auto img = std::static_pointer_cast<ImageFrame>(frame);
        
        if (time_aligner) {
          time_aligner->on_image(img);
        }
        if (frame_logger && !time_aligner) {
          frame_logger->log_frame(frame);
        }
        if (video_server) {
          video_server->on_frame(img);
        }
      });

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

  LOG_INFO(logger, "shutting down...");
  service_mgr.stop_all();
  service_mgr.shutdown_all();

  LOG_INFO(logger, "ridersense stopped");
  return 0;
}
