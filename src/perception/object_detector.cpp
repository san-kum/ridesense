#include "perception/object_detector.h"
#include <filesystem>
#include <opencv2/imgproc.hpp>

namespace ridersense {

ObjectDetector::ObjectDetector(const std::string &model_path,
                               float conf_threshold, float nms_threshold,
                               int input_size)
    : Service("object_detector"), model_path_(model_path),
      conf_threshold_(conf_threshold), nms_threshold_(nms_threshold),
      input_size_(input_size), logger_(Logger::get("detector")) {}

ObjectDetector::~ObjectDetector() {}

bool ObjectDetector::init() {
  LOG_INFO(logger_, "Object detector initializing");

  if (!std::filesystem::exists(model_path_)) {
    LOG_WARN(logger_, "Model not found: {}", model_path_);
    LOG_WARN(logger_, "Object detection will be stubbed");
    return true;
  }

  try {
    net_ = cv::dnn::readNet(model_path_);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    output_names_ = net_.getUnconnectedOutLayersNames();

    LOG_INFO(logger_, "Loaded model: {}", model_path_);
  } catch (const cv::Exception &e) {
    LOG_ERROR(logger_, "Failed to load model: {}", e.what());
    return false;
  }

  load_classes();

  return true;
}

bool ObjectDetector::start() {
  LOG_INFO(logger_, "Object detector started");
  return true;
}

bool ObjectDetector::stop() {
  LOG_INFO(logger_, "Stopping object detector");
  return true;
}

bool ObjectDetector::shutdown() {
  LOG_INFO(logger_, "Object detector shutdown");
  return true;
}

void ObjectDetector::load_classes() {
  // COCO classes
  classes_ = {"person",        "bicycle",      "car",
              "motorcycle",    "airplane",     "bus",
              "train",         "truck",        "boat",
              "traffic light", "fire hydrant", "stop sign",
              "parking meter", "bench",        "bird",
              "cat",           "dog",          "horse",
              "sheep",         "cow",          "elephant",
              "bear",          "zebra",        "giraffe",
              "backpack",      "umbrella",     "handbag",
              "tie",           "suitcase",     "frisbee",
              "skis",          "snowboard",    "sports ball",
              "kite",          "baseball bat", "baseball glove",
              "skateboard",    "surfboard",    "tennis racket",
              "bottle",        "wine glass",   "cup",
              "fork",          "knife",        "spoon",
              "bowl",          "banana",       "apple",
              "sandwich",      "orange",       "broccoli",
              "carrot",        "hot dog",      "pizza",
              "donut",         "cake",         "chair",
              "couch",         "potted plant", "bed",
              "dining table",  "toilet",       "tv",
              "laptop",        "mouse",        "remote",
              "keyboard",      "cell phone",   "microwave",
              "oven",          "toaster",      "sink",
              "refrigerator",  "book",         "clock",
              "vase",          "scissors",     "teddy bear",
              "hair drier",    "toothbrush"};
}

std::shared_ptr<DetectionFrame>
ObjectDetector::detect(std::shared_ptr<ImageFrame> image) {
  auto result =
      std::make_shared<DetectionFrame>(image->timestamp, detection_sequence_++);
  result->image_width = image->image.cols;
  result->image_height = image->image.rows;

  if (net_.empty()) {
    // No model loaded - return empty detections
    return result;
  }

  try {
    // Prepare input blob
    cv::Mat blob;
    cv::dnn::blobFromImage(image->image, blob, 1.0 / 255.0,
                           cv::Size(input_size_, input_size_), cv::Scalar(),
                           true, false);

    net_.setInput(blob);

    // Forward pass
    std::vector<cv::Mat> outs;
    net_.forward(outs, output_names_);

    // Postprocess
    result->detections = postprocess(image->image, outs);

  } catch (const cv::Exception &e) {
    LOG_ERROR(logger_, "Detection failed: {}", e.what());
  }

  return result;
}

std::vector<DetectionFrame::Detection>
ObjectDetector::postprocess(const cv::Mat &frame,
                            const std::vector<cv::Mat> &outs) {
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  for (const auto &out : outs) {
    auto *data = (float *)out.data;
    for (int j = 0; j < out.rows; ++j, data += out.cols) {
      cv::Mat scores = out.row(j).colRange(5, out.cols);
      cv::Point class_id_point;
      double confidence;
      cv::minMaxLoc(scores, 0, &confidence, 0, &class_id_point);

      if (confidence > conf_threshold_) {
        int center_x = (int)(data[0] * frame.cols);
        int center_y = (int)(data[1] * frame.rows);
        int width = (int)(data[2] * frame.cols);
        int height = (int)(data[3] * frame.rows);
        int left = center_x - width / 2;
        int top = center_y - height / 2;

        class_ids.push_back(class_id_point.x);
        confidences.push_back((float)confidence);
        boxes.push_back(cv::Rect(left, top, width, height));
      }
    }
  }

  // NMS
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_,
                    indices);

  std::vector<DetectionFrame::Detection> detections;
  for (int idx : indices) {
    DetectionFrame::Detection det;
    det.class_id = class_ids[idx];
    det.class_name =
        (det.class_id < classes_.size()) ? classes_[det.class_id] : "unknown";
    det.confidence = confidences[idx];
    det.bbox = boxes[idx];
    detections.push_back(det);
  }

  return detections;
}

} // namespace ridersense
