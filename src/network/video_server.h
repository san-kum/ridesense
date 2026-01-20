#pragma once

#include "capture/frame.h"
#include "common/logger.h"
#include "services/service.h"
#include <atomic>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <vector>

namespace ridersense {

class VideoServer : public Service {
public:
  VideoServer(int port, int quality = 80);
  ~VideoServer() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void on_frame(std::shared_ptr<ImageFrame> frame);

private:
  void accept_loop();
  void handle_client(int client_fd);
  void send_frame(int client_fd, const std::vector<uchar> &jpeg_data);

  int port_;
  int quality_;
  int server_fd_{-1};

  std::shared_ptr<spdlog::logger> logger_;

  std::atomic<bool> running_{false};
  std::thread accept_thread_;

  mutable std::mutex frame_mutex_;
  std::vector<uchar> current_frame_;

  mutable std::mutex clients_mutex_;
  std::vector<int> client_fds_;
};

} // namespace ridersense
