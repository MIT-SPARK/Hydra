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
#include "hydra/places/traversability_clustering.h"

#include <config_utilities/config.h>
#include <spark_dsg/node_attributes.h>

namespace hydra::places {

using Label = spark_dsg::SemanticNodeAttributes::Label;

void declare_config(SemanticLabelConfig& config) {
  using namespace config;
  name("SemanticLabelConfig");
  field(config.project_labels_to_ground, "project_labels_to_ground");
  field(config.robot_height, "robot_height");
  field(config.label_depth_tolerance, "label_depth_tolerance");
  field(config.label_use_const_weight, "label_use_const_weight");
}

void extractSemanticLabels(const SemanticLabelConfig& config,
                           float current_robot_height,
                           const ActiveWindowOutput& msg,
                           spark_dsg::DynamicSceneGraph& graph) {
  const auto& tsdf = msg.map().getTsdfLayer();
  const auto& sensor = msg.sensor_data->getSensor();
  const Eigen::Isometry3f sensor_T_world =
      msg.sensor_data->getSensorPose().cast<float>().inverse();

  for (const auto& [id, node] :
       graph.getLayer(spark_dsg::DsgLayers::TRAVERSABILITY).nodes()) {
    auto& attrs = node->attributes<spark_dsg::TraversabilityNodeAttributes>();
    if (!attrs.is_active) continue;

    Eigen::Vector3f pos_W = attrs.position.cast<float>();
    if (config.project_labels_to_ground) {
      if (config.robot_height > 0.0f) {
        pos_W.z() = current_robot_height - config.robot_height;
      } else {
        while (true) {
          const auto voxel = tsdf.getVoxelPtr(pos_W);
          if (!voxel) {
            pos_W = attrs.position.cast<float>();
            break;
          }
          if (voxel->distance <= 0.0) break;
          pos_W.z() -= tsdf.voxel_size;
        }
      }
    }

    const Eigen::Vector3f pos_C = sensor_T_world * pos_W;
    int u, v;
    if (!sensor.projectPointToImagePlane(pos_C, u, v)) continue;
    const float img_range =
        msg.sensor_data->depth_image.at<InputData::RangeType>(v, u);
    const float place_range = pos_C.norm();
    if (place_range > img_range + config.label_depth_tolerance) continue;
    const int label =
        msg.sensor_data->label_image.at<InputData::LabelType>(v, u);
    float weight = 1.0f;
    if (!config.label_use_const_weight) {
      weight /= (place_range * place_range);
    }
    attrs.label_weights[static_cast<Label>(label)] += weight;
  }
}

}  // namespace hydra::places
