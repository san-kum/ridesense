#include "perception/lane_detector.h"
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace ridersense {

LaneDetector::LaneDetector(float roi_top_ratio, int canny_low, int canny_high,
                           int hough_threshold)
    : Service("lane_detector"), roi_top_ratio_(roi_top_ratio),
      canny_low_(canny_low), canny_high_(canny_high),
      hough_threshold_(hough_threshold), logger_(Logger::get("lanes")) {}

LaneDetector::~LaneDetector() {}

bool LaneDetector::init() {
  LOG_INFO(logger_, "Lane detector initialized");
  return true;
}

bool LaneDetector::start() {
  LOG_INFO(logger_, "Lane detector started");
  return true;
}

bool LaneDetector::stop() {
  LOG_INFO(logger_, "Stopping lane detector");
  return true;
}

bool LaneDetector::shutdown() {
  LOG_INFO(logger_, "Lane detector shutdown");
  return true;
}

std::shared_ptr<LaneFrame>
LaneDetector::detect(std::shared_ptr<ImageFrame> image) {
  auto result = std::make_shared<LaneFrame>(image->timestamp, lane_sequence_++);

  cv::Mat processed = preprocess(image->image);

  auto lines = detect_lines(processed);

  cluster_lanes(lines, *result);

  return result;
}

cv::Mat LaneDetector::preprocess(const cv::Mat &image) {
  cv::Mat gray, edges;

  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

  cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
  int roi_top = static_cast<int>(gray.rows * roi_top_ratio_);
  std::vector<cv::Point> roi_points = {
      cv::Point(0, gray.rows), cv::Point(0, roi_top),
      cv::Point(gray.cols, roi_top), cv::Point(gray.cols, gray.rows)};
  cv::fillConvexPoly(mask, roi_points, cv::Scalar(255));

  cv::Canny(gray, edges, canny_low_, canny_high_);

  cv::bitwise_and(edges, mask, edges);

  return edges;
}

std::vector<cv::Vec4i> LaneDetector::detect_lines(const cv::Mat &edges) {
  std::vector<cv::Vec4i> lines;

  cv::HoughLinesP(edges, lines, 1, CV_PI / 180, hough_threshold_, 50, 10);

  return lines;
}

void LaneDetector::cluster_lanes(const std::vector<cv::Vec4i> &lines,
                                 LaneFrame &result) {
  if (lines.empty()) {
    return;
  }

  std::vector<cv::Vec4i> left_lines, right_lines;

  for (const auto &line : lines) {
    float dx = line[2] - line[0];
    float dy = line[3] - line[1];

    if (std::abs(dx) < 1e-6)
      continue;

    float slope = dy / dx;

    if (std::abs(slope) < 0.5)
      continue;

    if (slope < 0) {
      left_lines.push_back(line);
    } else {
      right_lines.push_back(line);
    }
  }

  if (!left_lines.empty()) {
    LaneFrame::Lane left_lane;
    left_lane.side = LaneFrame::Lane::Side::LEFT;
    left_lane.type = LaneFrame::Lane::Type::UNKNOWN;
    left_lane.is_valid = true;

    for (const auto &line : left_lines) {
      left_lane.points.push_back(cv::Point2f(line[0], line[1]));
      left_lane.points.push_back(cv::Point2f(line[2], line[3]));
    }

    result.lanes.push_back(left_lane);
  }

  if (!right_lines.empty()) {
    LaneFrame::Lane right_lane;
    right_lane.side = LaneFrame::Lane::Side::RIGHT;
    right_lane.type = LaneFrame::Lane::Type::UNKNOWN;
    right_lane.is_valid = true;

    for (const auto &line : right_lines) {
      right_lane.points.push_back(cv::Point2f(line[0], line[1]));
      right_lane.points.push_back(cv::Point2f(line[2], line[3]));
    }

    result.lanes.push_back(right_lane);
  }

  // TODO: fit polynomial, estimate lateral offset
}

} // namespace ridersense
