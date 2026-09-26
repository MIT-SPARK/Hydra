#include "hydra_multi/backend/update_crisp_objects_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/backend/backend_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/mesh_utilities.h>
#include <hydra/utils/timing_utilities.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>

namespace hydra_multi {

using hydra::timing::ScopedTimer;
using spark_dsg::ObjectNodeAttributes;

NodeAttributes::Ptr mergeCrispObjectAttributes(const DynamicSceneGraph& graph,
                                               const std::vector<NodeId>& nodes) {
  if (nodes.empty()) {
    return nullptr;
  }

  auto iter = nodes.begin();
  auto attrs_ptr = graph.getNode(*iter).attributes().clone();
  auto& new_attrs =
      *CHECK_NOTNULL(dynamic_cast<ObjectNodeAttributes*>(attrs_ptr.get()));
  std::vector<Eigen::MatrixXf> crisp_features;
  size_t total_columns = 0;
  // Technically crisp_features should be of single column. Just have this now to
  // maintain generality.
  while (iter != nodes.end()) {
    const auto& attrs = graph.getNode(*iter).attributes<ObjectNodeAttributes>();
    crisp_features.push_back(attrs.semantic_feature);
    total_columns += attrs.semantic_feature.cols();
    ++iter;
  }

  // Combine into single matrix
  Eigen::MatrixXf combined(crisp_features.front().rows(), total_columns);
  int current_col = 0;
  for (const auto& mat : crisp_features) {
    combined.block(0, current_col, mat.rows(), mat.cols()) = mat;
    current_col += mat.cols();
  }

  new_attrs.semantic_feature = combined.rowwise().mean();

  return attrs_ptr;
}

void declare_config(UpdateCrispObjectsFunctor::Config& config) {
  using namespace config;
  name("UpdateCrispObjectsFunctor::Config");
  field(config.allow_feature_merging, "allow_feature_merging");
  field(config.merge_proposer, "merge_proposer");
  field(config.merge_require_same_label, "merge_require_same_label");
  field(config.check_feature_similarity, "check_feature_similarity");
  field(config.min_feature_similarity, "min_feature_similarity");
}

UpdateCrispObjectsFunctor::UpdateCrispObjectsFunctor(const Config& config)
    : config(config::checkValid(config)), merge_proposer(config.merge_proposer) {}

hydra::UpdateFunctor::Hooks UpdateCrispObjectsFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.find_merges = [this](const auto& graph, const auto& info) {
    return findMerges(graph, info);
  };

  if (config.allow_feature_merging) {
    my_hooks.merge = &mergeCrispObjectAttributes;
  }
  return my_hooks;
}

void UpdateCrispObjectsFunctor::call(const DynamicSceneGraph& unmerged,
                                     SharedDsgInfo& dsg,
                                     const UpdateInfo::ConstPtr& info) const {
  ScopedTimer spin_timer("backend/update_crisp_objects", info->timestamp_ns);
  if (!unmerged.hasLayer(DsgLayers::OBJECTS)) {
    VLOG(5) << "Skipping khronos object update due to missing layer";
    return;
  }

  // we want to use the unmerged graph for most things
  const auto& objects = unmerged.getLayer(DsgLayers::OBJECTS);
  // we want to iterate over the unmerged graph
  const auto new_loopclosure = info->loop_closure_detected;
  active_tracker.clear();  // reset from previous pass
  LayerView view = new_loopclosure ? LayerView(objects) : active_tracker.view(objects);

  if (!info->pgmo_values) {
    return;
  }

  const auto& pgmo_values = *info->pgmo_values;
  for (const auto& node : view) {
    if (!pgmo_values.exists(node.id)) {
      VLOG(10) << "[Hydra Backend] missing object " << NodeSymbol(node.id).str()
               << " in deformation graph.";
      continue;
    }

    auto& attrs = node.attributes<ObjectNodeAttributes>();
    attrs.position = pgmo_values.at<gtsam::Pose3>(node.id).translation();
    attrs.world_R_object =
        pgmo_values.at<gtsam::Pose3>(node.id).rotation().toQuaternion();
    attrs.bounding_box.world_P_center = attrs.position.cast<float>();
    attrs.bounding_box.world_R_center = attrs.world_R_object.cast<float>();
    dsg.graph->setNodeAttributes(node.id, attrs.clone());
  }
}

MergeList UpdateCrispObjectsFunctor::findMerges(
    const DynamicSceneGraph& graph, const UpdateInfo::ConstPtr& info) const {
  // TODO(Yun) copied from Khronos from now. Update.
  if (!graph.hasLayer(DsgLayers::OBJECTS)) {
    return {};
  }

  const auto new_lcd = info->loop_closure_detected;
  const auto& objects = graph.getLayer(DsgLayers::OBJECTS);
  // freeze layer view to avoid messing with tracker
  LayerView view = new_lcd ? LayerView(objects) : active_tracker.view(objects, true);

  MergeList proposals;
  merge_proposer.findMerges(
      objects,
      view,
      [this](const SceneGraphNode& lhs, const SceneGraphNode& rhs) {
        const auto lhs_attrs = lhs.tryAttributes<ObjectNodeAttributes>();
        const auto rhs_attrs = rhs.tryAttributes<ObjectNodeAttributes>();

        if (!lhs_attrs || !rhs_attrs) {
          return false;
        }

        if (config.merge_require_same_label) {
          if (lhs_attrs->semantic_label != rhs_attrs->semantic_label) {
            return false;
          }
        }

        if (config.check_feature_similarity) {
          auto lhs_sem_norm = lhs_attrs->semantic_feature.colwise().normalized();
          auto rhs_sem_norm = rhs_attrs->semantic_feature.colwise().normalized();

          Eigen::VectorXf similarities(lhs_sem_norm.cols());
          for (int i = 0; i < lhs_sem_norm.cols(); ++i) {
            similarities(i) = lhs_sem_norm.col(i).dot(rhs_sem_norm.col(i));
          }
          if (similarities.maxCoeff() < config.min_feature_similarity) {
            return false;
          }
        }

        // IOU check not supported for oriented bbox
        return (lhs_attrs->bounding_box.contains(rhs_attrs->position) ||
                rhs_attrs->bounding_box.contains(lhs_attrs->position));
      },
      proposals);
  return proposals;
}

}  // namespace hydra_multi
