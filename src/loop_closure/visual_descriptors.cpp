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
#include "hydra/loop_closure/visual_descriptors.h"

#include <DBoW2/DBoW2.h>
#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include "hydra/common/common.h"

namespace hydra::lcd {

void declare_config(OrbFeatureDetector::Config& config) {
  using namespace config;
  name("OrbFeatureDetector::Config");
  field(config.num_features, "num_features");
  field(config.edge_threshold, "edge_threshold");
  field(config.fast_threshold, "fast_threshold");
  check(config.num_features, GT, 0, "num_features");
  check(config.edge_threshold, GT, 0, "edge_threshold");
  check(config.fast_threshold, GT, 0, "fast_threshold");
}

void declare_config(DBoWFactory::Config& config) {
  using namespace config;
  name("DBoWFactory::Config");
  base<SensorDescriptorFactory::Config>(config);
  field<Path>(config.vocabulary_path, "vocabulary_path");
  check<Path::Exists>(config.vocabulary_path, "vocabulary_path");
}

OrbFeatureDetector::OrbFeatureDetector(const Config& config) {
  detector = cv::ORB::create(config.num_features);
  detector->setEdgeThreshold(config.edge_threshold);
  detector->setFastThreshold(config.fast_threshold);
}

SensorFeatures::Ptr OrbFeatureDetector::detect(const Sensor&,
                                               const FrameData& data) const {
  if (data.color_image.empty()) {
    LOG(WARNING) << "Color image required for DBoW descriptor1";
    return nullptr;
  }

  if (data.vertex_map.empty()) {
    LOG(ERROR) << "Frame data must be finalized!";
    return nullptr;
  }

  if (data.points_in_world_frame) {
    LOG(ERROR) << "Frame data point cloud must not be in world frame";
    return nullptr;
  }

  auto features = std::make_shared<SensorFeatures>();
  cv::Mat mask;
  std::vector<cv::KeyPoint> keypoints;
  detector->detectAndCompute(data.color_image, mask, keypoints, features->descriptors);

  features->features = Eigen::MatrixXd(keypoints.size(), 3);
  auto& feature_points = features->features;
  for (size_t i = 0; i < keypoints.size(); ++i) {
    const auto& pt = keypoints[i].pt;
    const auto u = static_cast<int>(pt.x);
    const auto v = static_cast<int>(pt.y);
    CHECK_GE(u, 0);
    CHECK_LT(u, data.vertex_map.cols);
    CHECK_GE(v, 0);
    CHECK_LT(v, data.vertex_map.rows);
    const auto p = data.vertex_map.at<cv::Vec3f>(v, u);
    feature_points(i, 0) = p[0];
    feature_points(i, 1) = p[1];
    feature_points(i, 2) = p[2];
    VLOG(VLEVEL_DETAILED) << "(u, v) = (" << u << ", " << v << ") -> [" << p[0] << ", "
                          << p[1] << ", " << p[2] << "]";
  }

  return features;
}

struct DBoWFactory::Impl {
  Impl(const DBoWFactory::Config& config) : config(config) {
    LOG(INFO) << "Loading vocabulary from '" << config.vocabulary_path.string()
              << "'...";
    vocab = std::make_unique<OrbVocabulary>(config.vocabulary_path.string());
    LOG(INFO) << "Finished loading vocabulary";
  }

  Descriptor::Ptr construct(const DynamicSceneGraphNode& node,
                            const cv::Mat& descriptors) const {
    std::vector<cv::Mat> descriptor_list(descriptors.rows);
    for (int i = 0; i < descriptors.rows; ++i) {
      descriptor_list[i] = descriptors.row(i);
    }

    DBoW2::BowVector bow_vec;
    vocab->transform(descriptor_list, bow_vec);

    auto descriptor = std::make_unique<Descriptor>();
    descriptor->root_position = node.attributes().position;
    descriptor->nodes.insert(node.id);
    descriptor->timestamp = node.timestamp;
    descriptor->normalized = true;

    const size_t num_words = bow_vec.size();
    descriptor->words = decltype(descriptor->words)::Zero(num_words, 1);
    descriptor->values = decltype(descriptor->values)::Zero(num_words, 1);
    int i = 0;
    for (auto&& [word, value] : bow_vec) {
      descriptor->words(i, 0) = word;
      descriptor->values(i, 0) = value;
      ++i;
    }

    return descriptor;
  }

  const DBoWFactory::Config config;
  std::unique_ptr<OrbVocabulary> vocab;
};

DBoWFactory::DBoWFactory(const DBoWFactory::Config& config)
    : SensorDescriptorFactory(config), config(config::checkValid(config)) {
  impl_ = std::make_unique<Impl>(config);
  auto orb_detector = dynamic_cast<OrbFeatureDetector*>(detector_.get());
  CHECK(orb_detector) << "ORB Feature detector required for DBoW descriptors";
}

DBoWFactory::~DBoWFactory() {}

Descriptor::Ptr DBoWFactory::describe(const Sensor&,
                                      const DynamicSceneGraphNode& node,
                                      const FrameData&,
                                      const SensorFeatures* features) const {
  if (!features) {
    LOG(ERROR) << "Features required for DBoW descriptors";
    return nullptr;
  }

  return impl_->construct(node, features->descriptors);
}

}  // namespace hydra::lcd
