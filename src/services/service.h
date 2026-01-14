#pragma once

#include <string>

namespace ridersense {
class Service {
public:
  explicit Service(const std::string &name) : name_(name) {}
  virtual ~Service() = default;

  virtual bool init() = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual bool shutdown() = 0;

  const std::string &name() const { return name_; }

protected:
  std::string name_;
};
} // namespace ridersense
