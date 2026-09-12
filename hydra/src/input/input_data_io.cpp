/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/input/input_data_io.h"

#include <config_utilities/parsing/yaml.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/validation.h>
#include <config_utilities/virtual_config.h>

#include <array>
#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <utility>

#include "hydra/common/config_utilities.h"

namespace hydra::input {
namespace {

struct ImageField {
  const char* name;
  cv::Mat InputData::* member;
  int type;
  const char* extension;
};

const std::array<ImageField, 8> kImages{{
    {"color", &InputData::color_image, CV_8UC3, ".png"},
    {"color_mask", &InputData::color_mask, CV_8UC1, ".png"},
    {"depth", &InputData::depth_image, CV_32FC1, ".exr"},
    {"range", &InputData::range_image, CV_32FC1, ".exr"},
    {"labels", &InputData::label_image, CV_32SC1, ".tiff"},
    {"instances", &InputData::instance_image, CV_16SC1, ".tiff"},
    {"vertices", &InputData::vertex_map, CV_32FC3, ".exr"},
    {"traversability", &InputData::traversability_image, CV_32FC1, ".exr"},
}};

const ImageField kSensorMask{"sensor_mask", nullptr, CV_8UC3, ".png"};

YAML::Node writeFeature(const FeatureVector& feature) {
  auto node =
      feature.size() ? YAML::Node(feature) : YAML::Node(YAML::NodeType::Sequence);
  node.SetStyle(YAML::EmitterStyle::Flow);
  return node;
}

FeatureVector readFeature(const YAML::Node& node) {
  if (!node.IsSequence()) {
    throw std::runtime_error("feature must be a sequence");
  }

  FeatureVector feature(node.size());
  YAML::convert<FeatureVector>::decode(node, feature);
  return feature;
}

YAML::Node writePose(const Eigen::Isometry3d& pose) {
  if (!pose.matrix().allFinite()) {
    throw std::runtime_error("non-finite pose");
  }

  const auto rotation = Eigen::Quaterniond(pose.rotation());
  const auto& p = pose.translation();
  YAML::Node node;
  node["translation_m"] = Eigen::Vector3d(p);
  node["rotation_wxyz"] =
      std::array<double, 4>{rotation.w(), rotation.x(), rotation.y(), rotation.z()};
  node["translation_m"].SetStyle(YAML::EmitterStyle::Flow);
  node["rotation_wxyz"].SetStyle(YAML::EmitterStyle::Flow);
  return node;
}

Eigen::Isometry3d readPose(const YAML::Node& record) {
  const auto p = record["translation_m"].as<std::array<double, 3>>();
  const auto q = record["rotation_wxyz"].as<std::array<double, 4>>();
  const auto rotation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
  if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 1.0e-6) {
    throw std::runtime_error("invalid pose quaternion");
  }

  auto result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.toRotationMatrix();
  result.translation() = Eigen::Vector3d(p.data());
  if (!result.matrix().allFinite()) {
    throw std::runtime_error("non-finite pose");
  }

  return result;
}

std::string readEntryName(const YAML::Node& description) {
  const auto name = description["file"].as<std::string>();
  if (name.empty() || name.find('/') != std::string::npos ||
      name.find('\\') != std::string::npos || name == "." || name == "..") {
    throw std::runtime_error("invalid input entry name");
  }

  return name;
}

bool isValidImageType(const ImageField& field, int type) {
  // Sensor masks are read as BGR by Sensor, but an in-memory mask may be scalar.
  if (!field.member) {
    return type == CV_8UC1 || type == CV_8UC3;
  }

  return type == field.type;
}

std::vector<std::string> getChannelOrder(const ImageField& field, int type) {
  if (field.member == &InputData::vertex_map) {
    return {"X", "Y", "Z"};
  }

  if (field.member == &InputData::color_image) {
    return {"R", "G", "B"};
  }

  if (type == CV_8UC3) {
    return {"B", "G", "R"};
  }

  return {"scalar"};
}

std::vector<int> getEncodingOptions(const cv::Mat& image) {
  if (image.depth() != CV_32F) {
    return {};
  }

  return {cv::IMWRITE_EXR_TYPE,
          cv::IMWRITE_EXR_TYPE_FLOAT,
          cv::IMWRITE_EXR_COMPRESSION,
          cv::IMWRITE_EXR_COMPRESSION_ZIP};
}

YAML::Node writeImage(const ImageField& field,
                      const cv::Mat& image,
                      const WriteEntry& write) {
  if (image.empty()) {
    return YAML::Node(YAML::NodeType::Null);
  }

  try {
    if (image.dims != 2 || !isValidImageType(field, image.type())) {
      throw std::runtime_error("unsupported matrix type or dimensions");
    }

    cv::Mat encoded_image;
    if (field.member == &InputData::color_image) {
      cv::cvtColor(image, encoded_image, cv::COLOR_RGB2BGR);
    } else {
      encoded_image = image;
    }

    Bytes bytes;
    const auto options = getEncodingOptions(image);
    if (!cv::imencode(field.extension, encoded_image, bytes, options)) {
      throw std::runtime_error("image encoding failed");
    }

    const auto filename = std::string(field.name) + field.extension;
    write(filename, bytes);
    YAML::Node node;
    node["file"] = filename;
    node["rows"] = image.rows;
    node["cols"] = image.cols;
    node["opencv_type"] = image.type();
    node["channel_order"] = getChannelOrder(field, image.type());
    node.SetStyle(YAML::EmitterStyle::Flow);
    return node;
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string(field.name) + ": " + e.what());
  }
}

cv::Mat readImage(const ImageField& field,
                  const YAML::Node& description,
                  const ReadEntry& read) {
  if (description.IsNull()) {
    return {};
  }

  try {
    const auto type = description["opencv_type"].as<int>();
    if (!isValidImageType(field, type)) {
      throw std::runtime_error("unsupported matrix type");
    }

    if (description["channel_order"].as<std::vector<std::string>>() !=
        getChannelOrder(field, type)) {
      throw std::runtime_error("unsupported channel order");
    }

    const auto rows = description["rows"].as<int>();
    const auto cols = description["cols"].as<int>();
    const auto bytes = read(readEntryName(description));
    auto image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
    if (image.empty() || image.type() != type || image.rows != rows ||
        image.cols != cols) {
      throw std::runtime_error("decoded image does not match metadata");
    }

    if (field.member == &InputData::color_image) {
      cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    return image;
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string(field.name) + ": " + e.what());
  }
}

YAML::Node writeSensor(const Sensor& sensor) {
  auto node = sensor.dump();
  if (!node["type"] || node["type"].as<std::string>().empty()) {
    throw std::runtime_error("sensor dump must include its registered factory type");
  }

  ParamSensorExtrinsics::Config extrinsics;
  const auto transform = sensor.body_T_sensor();
  extrinsics.body_p_sensor = transform.translation();
  extrinsics.body_R_sensor = Eigen::Quaterniond(transform.rotation());
  node["extrinsics"] =
      config::toYaml(config::VirtualConfig<SensorExtrinsics>(extrinsics));

  // The effective mask is an archive entry, never a dependency on an external path.
  node["static_mask_fp"] = "";

  YAML::Node record;
  record["name"] = sensor.name;
  record["config"] = node;
  return record;
}

Sensor::Ptr readSensor(const YAML::Node& record) {
  const auto node = record["config"];
  if (!node["extrinsics"] || node["extrinsics"]["type"].as<std::string>() != "param" ||
      node["static_mask_fp"].as<std::string>() != "") {
    throw std::runtime_error("sensor configuration is not self-contained");
  }

  const auto cfg = config::fromYaml<config::VirtualConfig<Sensor>>(node);
  if (!cfg || !config::isValid(cfg)) {
    throw std::runtime_error("invalid sensor configuration");
  }

  auto sensor = cfg.create(record["name"].as<std::string>());
  if (!sensor) {
    throw std::runtime_error("could not construct sensor");
  }

  return sensor;
}

YAML::Node writeMetadata(const InputData& input) {
  YAML::Node record;
  record["format"] = "hydra_input";
  record["version"] = 2;
  record["timestamp_ns"] = input.timestamp_ns;
  record["world_T_body"] = writePose(input.world_T_body);
  record["sensor"] = writeSensor(input.getSensor());
  record["points_in_world_frame"] = input.points_in_world_frame;
  record["min_range_m"] = input.min_range;
  record["max_range_m"] = input.max_range;
  record["feature"] = writeFeature(input.feature);

  YAML::Node features(YAML::NodeType::Sequence);
  for (const auto& [label, feature] : input.label_features) {
    YAML::Node entry;
    entry["label"] = label;
    entry["feature"] = writeFeature(feature);
    entry.SetStyle(YAML::EmitterStyle::Flow);
    features.push_back(entry);
  }

  record["label_features"] = features;
  return record;
}

void readMetadata(const YAML::Node& record, InputData& input) {
  input.timestamp_ns = record["timestamp_ns"].as<TimeStamp>();
  input.world_T_body = readPose(record["world_T_body"]);
  input.points_in_world_frame = record["points_in_world_frame"].as<bool>();
  input.min_range = record["min_range_m"].as<float>();
  input.max_range = record["max_range_m"].as<float>();
  input.feature = readFeature(record["feature"]);

  const auto features = record["label_features"];
  if (!features.IsSequence()) {
    throw std::runtime_error("label_features must be a sequence");
  }

  for (const auto& entry : features) {
    const auto label = entry["label"].as<int>();
    auto feature = readFeature(entry["feature"]);
    const auto inserted =
        input.label_features.emplace(label, std::move(feature)).second;
    if (!inserted) {
      throw std::runtime_error("duplicate label feature");
    }
  }
}

}  // namespace

void writeInputData(const InputData& input, const WriteEntry& write) {
  try {
    auto record = writeMetadata(input);
    auto images = record["images"];
    for (const auto& field : kImages) {
      images[field.name] = writeImage(field, input.*field.member, write);
    }

    const auto& mask = input.getSensor().getStaticMask();
    images[kSensorMask.name] = writeImage(kSensorMask, mask, write);

    const auto text = YAML::Dump(record);
    write("metadata.yaml", Bytes(text.begin(), text.end()));
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("input serialization: ") + e.what());
  }
}

InputData::Ptr readInputData(const ReadEntry& read) {
  try {
    const auto bytes = read("metadata.yaml");
    const auto record = YAML::Load(std::string(bytes.begin(), bytes.end()));
    if (record["format"].as<std::string>() != "hydra_input" ||
        record["version"].as<int>() != 2) {
      throw std::runtime_error("unsupported input format/version");
    }

    auto sensor = readSensor(record["sensor"]);
    auto input = std::make_shared<InputData>(sensor);
    readMetadata(record, *input);

    const auto& images = record["images"];
    for (const auto& field : kImages) {
      (*input).*field.member = readImage(field, images[field.name], read);
    }

    const auto mask = readImage(kSensorMask, images[kSensorMask.name], read);
    sensor->setStaticMask(mask);
    return input;
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("input deserialization: ") + e.what());
  }
}

}  // namespace hydra::input
