#pragma once
#include "kimera_scene_graph/common.h"

#include <kimera_dsg/scene_graph.h>
#include <kimera_semantics/common.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_skeleton/skeleton.h>

#include <glog/logging.h>

namespace voxblox {

// TODO(nathan) fix

std_msgs::ColorRGBA getVertexColor(const Mesh& mesh,
                                   const ColorMode& color_mode,
                                   const size_t index);

}  // namespace voxblox

namespace kimera {

namespace utils {

void convertLayerToSkeleton(const SceneGraphLayer& scene_graph_layer,
                            vxb::SparseSkeletonGraph* skeleton);

void fillLayerFromSkeleton(const vxb::SparseSkeletonGraph& skeleton,
                           SceneGraph* scene_graph);

}  // namespace utils

}  // namespace kimera
