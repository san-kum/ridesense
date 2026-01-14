#pragma once

#include <yaml-cpp/yaml.h>

namespace ridersense {
class Config {
public:
  static Config &instance();

  void load(const std::string &path);

  YAML::Node root() const { return root_; }

  std::string system_name() const;
  std::string log_level() const;
  std::string log_dir() const;

  bool service_enabled(const std::string &name) const;

private:
  Config() = default;
  YAML::Node root_;
};

} // namespace ridersense
