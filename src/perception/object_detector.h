#pragma once

#include "capture/frame.h"
#include "common/logger.h"
#include "services/service.h"
#include <opencv2/dnn.hpp>

namespace ridersense {

class ObjectDetector : public Service {
public:
  ObjectDetector(const std::string &model_path, float conf_threshold,
                 float nms_threshold, int input_size);
  ~ObjectDetector() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  std::shared_ptr<DetectionFrame> detect(std::shared_ptr<ImageFrame> image);

private:
  void load_classes();
  std::vector<DetectionFrame::Detection>
  postprocess(const cv::Mat &frame, const std::vector<cv::Mat> &outs);

  std::string model_path_;
  float conf_threshold_;
  float nms_threshold_;
  int input_size_;

  std::shared_ptr<spdlog::logger> logger_;

  cv::dnn::Net net_;
  std::vector<std::string> classes_;
  std::vector<std::string> output_names_;

  uint64_t detection_sequence_{0};
};

} // namespace ridersense
