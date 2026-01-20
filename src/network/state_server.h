#pragma once

#include "common/logger.h"
#include "network/state_message.h"
#include "services/service.h"
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

namespace ridersense {

class StateServer : public Service {
public:
  StateServer(int port, int max_clients);
  ~StateServer() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void broadcast(const std::string &message);
  int client_count() const;

private:
  void accept_loop();
  void remove_client(int fd);

  int port_;
  int max_clients_;
  int server_fd_{-1};

  std::shared_ptr<spdlog::logger> logger_;

  std::atomic<bool> running_{false};
  std::thread accept_thread_;

  mutable std::mutex clients_mutex_;
  std::vector<int> client_fds_;
};

} // namespace ridersense
