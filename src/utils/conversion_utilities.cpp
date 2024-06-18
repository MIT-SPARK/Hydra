#include "hydra/utils/conversion_utilities.h"

#include <glog/logging.h>

#include <opencv2/imgproc.hpp>

#include "hydra/common/global_info.h"
#include "hydra/common/semantic_color_map.h"
#include "hydra/input/input_packet.h"
#include "hydra/input/sensor.h"

namespace hydra {

std::string showTypeInfo(const cv::Mat& mat) {
  std::stringstream ss;
  ss << "{depth: " << mat.depth() << ", channels: " << mat.channels() << "}";
  return ss.str();
}

bool conversions::inputPacketToData(InputData& input_data,
                                    const InputPacket& input_packet) {
  if (!input_packet.fillInputData(input_data)) {
    LOG(ERROR) << "[Hydra Conversions] unable to fill InputData structure.";
    return false;
  }
  if (!conversions::normalizeData(input_data)) {
    LOG(ERROR) << "[Hydra Conversions] unable to normalize data.";
    return false;
  }
  const Sensor& sensor =
      *GlobalInfo::instance().getSensor(input_packet.sensor_input->sensor_id);
  if (!sensor.finalizeRepresentations(input_data)) {
    LOG(ERROR) << "[Hydra Conversions] unable to compute inputs for integration";
    return false;
  }
  return true;
}

bool conversions::normalizeDepth(InputData& data) { return conversions::convertDepth(data); }

bool conversions::normalizeData(InputData& data, bool normalize_labels) {
  if (!conversions::convertDepth(data)) {
    return false;
  }

  if (!conversions::convertColor(data)) {
    return false;
  }

  // must come after convertColor as it uses color image
  if (normalize_labels && !conversions::convertLabels(data)) {
    return false;
  }

  if (!data.vertex_map.empty() && data.vertex_map.type() != CV_32FC3) {
    LOG(ERROR) << "pointcloud must be of type CV_32FC3, not "
               << showTypeInfo(data.vertex_map);
    return false;
  }

  return true;
}

bool conversions::colorToLabels(cv::Mat& label_image, const cv::Mat& colors) {
  if (colors.empty() || colors.channels() != 3) {
    LOG(ERROR) << "color image required to decode semantic labels";
    return false;
  }

  CHECK_EQ(colors.type(), CV_8UC3);

  const auto colormap_ptr = GlobalInfo::instance().getSemanticColorMap();
  if (!colormap_ptr || !colormap_ptr->isValid()) {
    LOG(ERROR)
        << "label colormap not valid, but required for converting colors to labels!";
    return false;
  }

  cv::Mat new_label_image(colors.size(), CV_32SC1);
  for (int r = 0; r < colors.rows; ++r) {
    for (int c = 0; c < colors.cols; ++c) {
      const auto& pixel = colors.at<cv::Vec3b>(r, c);
      Color color(pixel[0], pixel[1], pixel[2]);
      // this is lazy, but works out to the same invalid label we normally use
      new_label_image.at<int32_t>(r, c) =
          colormap_ptr->getLabelFromColor(color).value_or(-1);
    }
  }

  label_image = new_label_image;
  return true;
}

bool conversions::convertLabels(InputData& data) {
  if (data.label_image.empty()) {
    return conversions::colorToLabels(data.label_image, data.color_image);
  }

  if (data.label_image.channels() != 1) {
    return conversions::colorToLabels(data.label_image, data.label_image);
  }

  // Enforcing requirement for int32_t at this point
  if (data.label_image.type() != CV_32SC1) {
    cv::Mat new_label_image(data.label_image.size(), CV_32SC1);
    data.label_image.convertTo(new_label_image, CV_32SC1);
    data.label_image = new_label_image;
  }

  LabelRemapper label_remapper = GlobalInfo::instance().getLabelRemapper();
  if (!label_remapper.empty()) {
    
    for (int r = 0; r < data.label_image.rows; ++r) {
      for (int c = 0; c < data.label_image.cols; ++c) {
        // TODO(marcus): any reason to cache image and reassign with a new one?
        const auto& pixel = data.label_image.at<int32_t>(r, c);
        data.label_image.at<int32_t>(r, c) =
            label_remapper.remapLabel(pixel).value_or(-1);
      }
    }
  }

  const auto label_type = data.label_image.type();
  if (label_type == CV_32SC1) {
    return true;
  }

  if (label_type != CV_8UC1 && label_type != CV_16UC1 && label_type != CV_8SC1 &&
      label_type != CV_16SC1) {
    LOG(ERROR) << "label image must be integer type, not " << showTypeInfo(data.label_image);
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

bool conversions::convertDepth(InputData& data) {
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

bool conversions::convertColor(InputData& data) {
  if (data.color_image.empty()) {
    return true;
  }

  if (data.color_image.type() != CV_8UC3) {
    LOG(ERROR) << "only 3-channel rgb images supported";
    return false;
  }

  if (data.color_is_bgr) {
    // TODO(marcus): should this do an assignment at the end?
    cv::Mat rgb_image;
    cv::cvtColor(data.color_image, rgb_image, cv::COLOR_BGR2RGB);
    data.color_image = rgb_image;
  }

  return true;
}

}  // namespace hydra
