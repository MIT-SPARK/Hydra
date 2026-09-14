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

#include <config_utilities/config.h>
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

namespace hydra::input {
namespace {

YAML::Node writeFeature(const FeatureVector& x) {
  auto node = x.size() ? YAML::Node(x) : YAML::Node(YAML::NodeType::Sequence);
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

  const Eigen::Quaterniond q(pose.rotation());
  YAML::Node node;
  node["translation_m"] = Eigen::Vector3d(pose.translation());
  node["rotation_wxyz"] = std::array<double, 4>{q.w(), q.x(), q.y(), q.z()};
  node["translation_m"].SetStyle(YAML::EmitterStyle::Flow);
  node["rotation_wxyz"].SetStyle(YAML::EmitterStyle::Flow);
  return node;
}

Eigen::Isometry3d readPose(const YAML::Node& record) {
  const auto p = record["translation_m"].as<std::array<double, 3>>();
  const auto q = record["rotation_wxyz"].as<std::array<double, 4>>();
  const Eigen::Quaterniond rot(q[0], q[1], q[2], q[3]);
  const Eigen::Vector3d vec(p[0], p[1], p[2]);
  const Eigen::Isometry3d result = Eigen::Translation<double, 3>(vec) * rot;
  if (!result.matrix().allFinite()) {
    throw std::runtime_error("non-finite pose");
  }

  return result;
}

std::vector<int> getEncodingOptions(const std::string& extension,
                                    const SaveOptions& options) {
  if (extension == ".png") {
    // Setting the level otherwise changes OpenCV's default RLE strategy.
    return {cv::IMWRITE_PNG_COMPRESSION,
            options.png_compression,
            cv::IMWRITE_PNG_STRATEGY,
            cv::IMWRITE_PNG_STRATEGY_RLE};
  }

  if (extension != ".exr") {
    return {};
  }

  int compression = cv::IMWRITE_EXR_COMPRESSION_ZIP;
  switch (options.float_compression) {
    case SaveOptions::FloatCompression::NONE:
      compression = cv::IMWRITE_EXR_COMPRESSION_NO;
      break;
    case SaveOptions::FloatCompression::RLE:
      compression = cv::IMWRITE_EXR_COMPRESSION_RLE;
      break;
    case SaveOptions::FloatCompression::ZIP:
      break;
  }

  return {cv::IMWRITE_EXR_TYPE,
          cv::IMWRITE_EXR_TYPE_FLOAT,
          cv::IMWRITE_EXR_COMPRESSION,
          compression};
}

void writeImage(const std::string& name,
                const std::string& extension,
                const SaveOptions& options,
                const cv::Mat& image,
                const WriteEntry& write,
                YAML::Node& record) {
  if (image.empty()) {
    return;
  }

  if (image.dims != 2) {
    throw std::runtime_error(name + ": unsupported matrix type or dimensions");
  }

  Bytes bytes;
  const auto encoding = getEncodingOptions(extension, options);
  if (!cv::imencode(extension, image, bytes, encoding)) {
    throw std::runtime_error(name + ": image encoding failed");
  }

  const auto filename = name + extension;
  write(filename, bytes);

  auto node = record[name];
  node["file"] = filename;
  node["rows"] = image.rows;
  node["cols"] = image.cols;
  node["opencv_type"] = image.type();
  node.SetStyle(YAML::EmitterStyle::Flow);
}

cv::Mat readImage(const std::string& name,
                  const YAML::Node& record,
                  const ReadEntry& read) {
  if (!record[name]) {
    return {};
  }

  const auto& description = record[name];
  const auto filename = description["file"].as<std::string>();
  const auto type = description["opencv_type"].as<int>();
  const auto rows = description["rows"].as<int>();
  const auto cols = description["cols"].as<int>();

  const auto bytes = read(filename);
  const auto image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
  if (image.empty() || image.type() != type || image.rows != rows ||
      image.cols != cols) {
    throw std::runtime_error(name + ": decoded image does not match metadata");
  }

  return image;
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

Sensor::Ptr readSensor(const YAML::Node& record, const cv::Mat& mask) {
  const auto node = record["config"];
  if (!node["extrinsics"] || node["extrinsics"]["type"].as<std::string>() != "param" ||
      node["static_mask_fp"].as<std::string>() != "") {
    throw std::runtime_error("sensor configuration is not self-contained");
  }

  auto sensor = Sensor::fromRecord(node, record["name"].as<std::string>(), mask);
  if (!sensor) {
    throw std::runtime_error("could not construct sensor");
  }

  return sensor;
}

YAML::Node writeMetadata(const InputData& input) {
  YAML::Node record;
  record["version"] = 1;
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

void declare_config(SaveOptions& config) {
  using namespace config;
  name("InputData::SaveOptions");
  enum_field(config.float_compression, "float_compression", {"none", "rle", "zip"});
  field(config.png_compression, "png_compression");
  field(config.archive, "archive");
  checkInRange(config.png_compression, 0, 9, "png_compression");
}

void writeInputData(const InputData& input,
                    const WriteEntry& write,
                    const SaveOptions& opts) {
  config::checkValid(opts);
  auto record = writeMetadata(input);

  auto images = record["images"];
  writeImage("color", ".png", opts, input.color_image, write, images);
  writeImage("color_mask", ".png", opts, input.color_mask, write, images);
  writeImage("depth", ".exr", opts, input.depth_image, write, images);
  writeImage("range", ".exr", opts, input.range_image, write, images);
  writeImage("labels", ".tiff", opts, input.label_image, write, images);
  writeImage("instances", ".tiff", opts, input.instance_image, write, images);
  writeImage("vertices", ".exr", opts, input.vertex_map, write, images);
  writeImage("traversability", ".exr", opts, input.traversability_image, write, images);

  const auto& mask = input.getSensor().getStaticMask();
  writeImage("sensor_mask", ".png", opts, mask, write, images);

  const auto text = YAML::Dump(record);
  write("metadata.yaml", Bytes(text.begin(), text.end()));
}

InputData::Ptr readInputData(const ReadEntry& read) {
  const auto bytes = read("metadata.yaml");
  const auto record = YAML::Load(std::string(bytes.begin(), bytes.end()));
  if (record["version"].as<int>() != 1) {
    throw std::runtime_error("unsupported input version");
  }

  const auto& images = record["images"];
  const auto mask = readImage("sensor_mask", images, read);
  const auto sensor = readSensor(record["sensor"], mask);

  auto input = std::make_shared<InputData>(sensor);
  readMetadata(record, *input);
  input->color_image = readImage("color", images, read);
  input->color_mask = readImage("color_mask", images, read);
  input->depth_image = readImage("depth", images, read);
  input->range_image = readImage("range", images, read);
  input->label_image = readImage("labels", images, read);
  input->instance_image = readImage("instances", images, read);
  input->vertex_map = readImage("vertices", images, read);
  input->traversability_image = readImage("traversability", images, read);
  return input;
}

}  // namespace hydra::input
