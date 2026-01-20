#include "network/video_server.h"
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>

namespace ridersense {

static const char* MJPEG_HEADER = 
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: close\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "\r\n";

static const char* FRAME_HEADER = 
    "--frame\r\n"
    "Content-Type: image/jpeg\r\n"
    "Content-Length: ";

VideoServer::VideoServer(int port, int quality)
    : Service("video_server"), port_(port), quality_(quality),
      logger_(Logger::get("video")) {}

VideoServer::~VideoServer() { shutdown(); }

bool VideoServer::init() {
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    LOG_ERROR(logger_, "failed to create socket: {}", strerror(errno));
    return false;
  }

  int opt = 1;
  setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port_);

  if (bind(server_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    LOG_ERROR(logger_, "bind failed on port {}: {}", port_, strerror(errno));
    close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  if (listen(server_fd_, 5) < 0) {
    LOG_ERROR(logger_, "listen failed: {}", strerror(errno));
    close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  int flags = fcntl(server_fd_, F_GETFL, 0);
  fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);

  LOG_INFO(logger_, "video server initialized on port {}", port_);
  return true;
}

bool VideoServer::start() {
  running_ = true;
  accept_thread_ = std::thread(&VideoServer::accept_loop, this);
  LOG_INFO(logger_, "video server started");
  return true;
}

bool VideoServer::stop() {
  LOG_INFO(logger_, "stopping video server");
  running_ = false;

  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int fd : client_fds_) {
      close(fd);
    }
    client_fds_.clear();
  }

  return true;
}

bool VideoServer::shutdown() {
  if (server_fd_ >= 0) {
    close(server_fd_);
    server_fd_ = -1;
  }
  LOG_INFO(logger_, "video server shutdown");
  return true;
}

void VideoServer::accept_loop() {
  while (running_) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(server_fd_, &read_fds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    int result = select(server_fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (result <= 0) continue;

    if (FD_ISSET(server_fd_, &read_fds)) {
      struct sockaddr_in client_addr {};
      socklen_t client_len = sizeof(client_addr);
      int client_fd = accept(server_fd_, (struct sockaddr *)&client_addr, &client_len);

      if (client_fd < 0) continue;

      char buf[1024];
      recv(client_fd, buf, sizeof(buf), 0);

      send(client_fd, MJPEG_HEADER, strlen(MJPEG_HEADER), MSG_NOSIGNAL);

      {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_fds_.push_back(client_fd);
      }

      LOG_INFO(logger_, "video client connected");
    }
  }
}

void VideoServer::on_frame(std::shared_ptr<ImageFrame> frame) {
  if (!frame || frame->image.empty()) return;

  std::vector<uchar> jpeg_data;
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality_};
  cv::imencode(".jpg", frame->image, jpeg_data, params);

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    current_frame_ = std::move(jpeg_data);
  }

  std::vector<uchar> frame_copy;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_copy = current_frame_;
  }

  std::vector<int> dead_clients;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int fd : client_fds_) {
      std::ostringstream header;
      header << FRAME_HEADER << frame_copy.size() << "\r\n\r\n";
      std::string header_str = header.str();

      ssize_t sent = send(fd, header_str.c_str(), header_str.size(), MSG_NOSIGNAL);
      if (sent < 0) {
        dead_clients.push_back(fd);
        continue;
      }

      sent = send(fd, frame_copy.data(), frame_copy.size(), MSG_NOSIGNAL);
      if (sent < 0) {
        dead_clients.push_back(fd);
        continue;
      }

      send(fd, "\r\n", 2, MSG_NOSIGNAL);
    }

    for (int fd : dead_clients) {
      close(fd);
      client_fds_.erase(std::remove(client_fds_.begin(), client_fds_.end(), fd), client_fds_.end());
    }
  }
}

} // namespace ridersense
