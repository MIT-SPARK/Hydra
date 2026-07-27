#include "hydra/input/input_conversion.h"

#include <glog/logging.h>

#include <opencv2/imgproc.hpp>

#include "hydra/common/global_info.h"
#include "hydra/input/input_packet.h"
#include "hydra/input/sensor.h"

namespace hydra::conversions {

namespace {

inline std::string showTypeInfo(const cv::Mat& mat) {
  std::stringstream ss;
  ss << "{depth: " << mat.depth() << ", channels: " << mat.channels() << "}";
  return ss.str();
}

bool convertLabels(InputData& data) {
  if (data.label_image.empty()) {
    return false;
  }

  // Enforcing requirement for int32_t at this point
  if (data.label_image.type() != CV_32SC1) {
    cv::Mat new_label_image(data.label_image.size(), CV_32SC1);
    data.label_image.convertTo(new_label_image, CV_32SC1);
    data.label_image = new_label_image;
  }

  const auto remap = GlobalInfo::instance().getLabelRemapper();
  if (!remap.empty()) {
    for (int r = 0; r < data.label_image.rows; ++r) {
      for (int c = 0; c < data.label_image.cols; ++c) {
        const auto pixel = data.label_image.at<int32_t>(r, c);
        data.label_image.at<int32_t>(r, c) = remap.remapLabel(pixel).value_or(-1);
      }
    }
  }

  const auto label_type = data.label_image.type();
  if (label_type == CV_32SC1) {
    return true;
  }

  if (label_type != CV_8UC1 && label_type != CV_16UC1 && label_type != CV_8SC1 &&
      label_type != CV_16SC1) {
    LOG(ERROR) << "label image must be integer type, not "
               << showTypeInfo(data.label_image);
    return false;
  }

  if (label_type == CV_16SC1 || label_type == CV_8SC1) {
    LOG_FIRST_N(WARNING, 5)
        << "signed to unsigned conversion of labels may not do what you want!";
  }

  cv::Mat label_converted;
  data.label_image.convertTo(label_converted, CV_32SC1);
  data.label_image = label_converted;
  return true;
}

bool convertDepth(InputData& data) {
  if (data.depth_image.empty()) {
    return true;
  }

  if (data.depth_image.channels() != 1) {
    LOG(ERROR) << "depth image must be single-channel";
    return false;
  }

  if (data.depth_image.type() == CV_32FC1) {
    return true;  // nothing else to do
  }

  if (data.depth_image.type() != CV_16UC1) {
    LOG(ERROR) << "only CV_32FC1 or CV_16UC1 formats supported, not "
               << showTypeInfo(data.depth_image);
    return false;
  }

  cv::Mat depth_converted;
  data.depth_image.convertTo(depth_converted, CV_32FC1, 1.0e-3);
  data.depth_image = depth_converted;
  return true;
}

bool convertColor(InputData& data) {
  if (data.color_image.empty()) {
    return true;
  }

  if (data.color_image.type() != CV_8UC3) {
    LOG(ERROR) << "only 3-channel rgb images supported";
    return false;
  }
  return true;
}

}  // namespace

std::unique_ptr<InputData> parseInputPacket(const InputPacket& input_packet,
                                            bool vertices_in_world_frame,
                                            bool normalize_labels) {
  if (!input_packet.sensor_input) {
    LOG(ERROR) << "[Input Conversion] Input packet has no sensor input.";
    return nullptr;
  }

  const auto& name = input_packet.sensor_input->sensor_name;
  auto sensor = GlobalInfo::instance().getSensor(name);
  if (!sensor) {
    LOG(ERROR) << "[Input Conversion] Missing sensor '" << name
               << "' for input packet @ " << input_packet.timestamp_ns << " [ns]";
    return nullptr;
  }

  auto data = std::make_unique<InputData>(sensor);
  if (!input_packet.fillInputData(*data)) {
    LOG(ERROR) << "[Input Conversion] Unable to fill input data from input packet.";
    return nullptr;
  }

  if (!normalizeData(*data, normalize_labels)) {
    LOG(ERROR) << "[Input Conversion] Unable to normalize data.";
    return nullptr;
  }

  if (!data->getSensor().finalizeRepresentations(*data)) {
    LOG(ERROR) << "[Input Conversion] Unable to compute inputs for integration";
    return nullptr;
  }

  convertVertexMap(*data, vertices_in_world_frame);
  return data;
}

bool normalizeData(InputData& data, bool normalize_labels) {
  if (!convertDepth(data)) {
    return false;
  }

  if (!convertColor(data)) {
    return false;
  }

  if (normalize_labels && !convertLabels(data)) {
    return false;
  }

  if (!data.vertex_map.empty() && data.vertex_map.type() != CV_32FC3) {
    LOG(ERROR) << "pointcloud must be CV_32FC3, not " << showTypeInfo(data.vertex_map);
    return false;
  }

  return true;
}

void convertVertexMap(InputData& data, bool in_world_frame) {
  if (data.points_in_world_frame == in_world_frame) {
    return;
  }

  Eigen::Isometry3f transform = data.getSensorPose().cast<float>();  // world_T_sensor
  if (!in_world_frame) {
    transform = transform.inverse();  // Instead get sensor_T_world
  }

  for (int r = 0; r < data.vertex_map.rows; ++r) {
    for (int c = 0; c < data.vertex_map.cols; ++c) {
      auto& point = data.vertex_map.at<cv::Vec3f>(r, c);
      Eigen::Vector3f point_eigen(point[0], point[1], point[2]);
      point_eigen = transform * point_eigen;
      point[0] = point_eigen.x();
      point[1] = point_eigen.y();
      point[2] = point_eigen.z();
    }
  }

  data.points_in_world_frame = in_world_frame;
}

}  // namespace hydra::conversions
