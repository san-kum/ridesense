#pragma once

#include "../common/logger.h"
#include "../services/service.h"
#include <memory>
#include <vector>

namespace ridersense {

class ServiceManager {
public:
  void add(std::shared_ptr<Service> service) { services_.push_back(service); }

  bool init_all() {
    auto logger = Logger::get("service_manager");
    for (auto &svc : services_) {
      LOG_INFO(logger, "initializing service: {}", svc->name());
      if (!svc->init()) {
        LOG_ERROR(logger, "failed to init service: {}", svc->name());
        return false;
      }
    }
    return true;
  }
  bool start_all() {
    auto logger = Logger::get("service_manager");
    for (auto &svc : services_) {
      LOG_INFO(logger, "starting service: {}", svc->name());
      if (!svc->start()) {
        LOG_ERROR(logger, "failed to start service: {}", svc->name());
        return false;
      }
    }
    return true;
  }

  void stop_all() {
    auto logger = Logger::get("service_manager");
    for (auto it = services_.rbegin(); it != services_.rend(); ++it) {
      LOG_INFO(logger, "stopping service: {}", (*it)->name());
      (*it)->stop();
    }
  }

  void shutdown_all() {
    auto logger = Logger::get("service_manager");
    for (auto it = services_.rbegin(); it != services_.rend(); ++it) {
      LOG_INFO(logger, "shutting down service: {}", (*it)->name());
      (*it)->shutdown();
    }
  }

private:
  std::vector<std::shared_ptr<Service>> services_;
};

} // namespace ridersense
