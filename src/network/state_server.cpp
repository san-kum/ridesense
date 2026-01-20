#include "network/state_server.h"
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ridersense {

StateServer::StateServer(int port, int max_clients)
    : Service("state_server"), port_(port), max_clients_(max_clients),
      logger_(Logger::get("network")) {}

StateServer::~StateServer() { shutdown(); }

bool StateServer::init() {
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    LOG_ERROR(logger_, "failed to create socket: {}", strerror(errno));
    return false;
  }

  int opt = 1;
  if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    LOG_WARN(logger_, "setsockopt SO_REUSEADDR failed");
  }

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

  if (listen(server_fd_, max_clients_) < 0) {
    LOG_ERROR(logger_, "listen failed: {}", strerror(errno));
    close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  int flags = fcntl(server_fd_, F_GETFL, 0);
  fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);

  LOG_INFO(logger_, "state server initialized on port {}", port_);
  return true;
}

bool StateServer::start() {
  running_ = true;
  accept_thread_ = std::thread(&StateServer::accept_loop, this);
  LOG_INFO(logger_, "state server started");
  return true;
}

bool StateServer::stop() {
  LOG_INFO(logger_, "stopping state server");
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

bool StateServer::shutdown() {
  if (server_fd_ >= 0) {
    close(server_fd_);
    server_fd_ = -1;
  }
  LOG_INFO(logger_, "state server shutdown");
  return true;
}

void StateServer::accept_loop() {
  while (running_) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(server_fd_, &read_fds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms timeout

    int result = select(server_fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (result < 0) {
      if (errno != EINTR) {
        LOG_ERROR(logger_, "select error: {}", strerror(errno));
      }
      continue;
    }

    if (result == 0) {
      continue; // timeout
    }

    if (FD_ISSET(server_fd_, &read_fds)) {
      struct sockaddr_in client_addr {};
      socklen_t client_len = sizeof(client_addr);
      int client_fd =
          accept(server_fd_, (struct sockaddr *)&client_addr, &client_len);

      if (client_fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          LOG_ERROR(logger_, "accept failed: {}", strerror(errno));
        }
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        if (static_cast<int>(client_fds_.size()) >= max_clients_) {
          LOG_WARN(logger_, "max clients reached, rejecting connection");
          close(client_fd);
          continue;
        }
        client_fds_.push_back(client_fd);
      }

      char client_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
      LOG_INFO(logger_, "client connected: {}:{}", client_ip,
               ntohs(client_addr.sin_port));
    }
  }
}

void StateServer::broadcast(const std::string &message) {
  std::string msg_with_newline = message + "\n";

  std::lock_guard<std::mutex> lock(clients_mutex_);
  std::vector<int> dead_clients;

  for (int fd : client_fds_) {
    ssize_t sent =
        send(fd, msg_with_newline.c_str(), msg_with_newline.size(), MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EPIPE || errno == ECONNRESET) {
        dead_clients.push_back(fd);
      }
    }
  }

  for (int fd : dead_clients) {
    LOG_INFO(logger_, "client disconnected (fd: {})", fd);
    close(fd);
    client_fds_.erase(
        std::remove(client_fds_.begin(), client_fds_.end(), fd),
        client_fds_.end());
  }
}

int StateServer::client_count() const {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  return static_cast<int>(client_fds_.size());
}

void StateServer::remove_client(int fd) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  close(fd);
  client_fds_.erase(std::remove(client_fds_.begin(), client_fds_.end(), fd),
                    client_fds_.end());
}

} // namespace ridersense
