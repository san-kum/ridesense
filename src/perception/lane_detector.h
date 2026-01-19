#pragma once

#include "capture/frame.h"
#include "common/logger.h"
#include "services/service.h"

namespace ridersense {

class LaneDetector : public Service {
public:
  LaneDetector(float roi_top_ratio, int canny_low, int canny_high,
               int hough_threshold);
  ~LaneDetector() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  std::shared_ptr<LaneFrame> detect(std::shared_ptr<ImageFrame> image);

private:
  cv::Mat preprocess(const cv::Mat &image);
  std::vector<cv::Vec4i> detect_lines(const cv::Mat &edges);
  void cluster_lanes(const std::vector<cv::Vec4i> &lines, LaneFrame &result);

  float roi_top_ratio_;
  int canny_low_;
  int canny_high_;
  int hough_threshold_;

  std::shared_ptr<spdlog::logger> logger_;
  uint64_t lane_sequence_{0};
};

} // namespace ridersense
