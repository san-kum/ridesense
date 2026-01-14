#include "../common/config.h"
#include <string>

namespace ridersense {

Config &Config::instance() {
  static Config cfg;
  return cfg;
}

void Config::load(const std::string &path) {
  try {
    root_ = YAML::LoadFile(path);
  } catch (const YAML::Exception &e) {
    throw std::runtime_error("Failed to load config: " + std::string(e.what()));
  }
}

std::string Config::system_name() const {
  return root_["system"]["name"].as<std::string>("ridersense");
}

std::string Config::log_level() const {
  return root_["system"]["log_level"].as<std::string>("info");
}

std::string Config::log_dir() const {
  return root_["system"]["log_dir"].as<std::string>("/tmp/ridersense/logs");
}

bool Config::service_enabled(const std::string &name) const {
  if (root_["services"] && root_["services"][name])
    return root_["services"][name]["enabled"].as<bool>(false);
  return false;
}

} // namespace ridersense
