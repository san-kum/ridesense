#pragma once

#include "../capture/frame.h"
#include "../services/service.h"
#include <functional>
#include <memory>
#include <mutex>

namespace ridersense {

using FrameCallback = std::function<void(std::shared_ptr<Frame>)>;

class Stream : public Service {
public:
  explicit Stream(const std::string &name) : Service(name) {}
  virtual ~Stream() = default;

  void set_callback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = cb;
  }

protected:
  void publish(std::shared_ptr<Frame> frame) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_)
      callback_(frame);
  }

  std::mutex callback_mutex_;
  FrameCallback callback_;
};

} // namespace ridersense
