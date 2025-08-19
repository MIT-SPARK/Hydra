#include "hydra/frontend/view_database.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>
#include <spark_dsg/printing.h>

#include "hydra/common/pipeline_queues.h"
#include "hydra/input/input_data.h"

namespace hydra {

void declare_config(ViewDatabase::Config& config) {
  using namespace config;
  name("ViewDatabase::Config");
  field(config.view_selection_method, "view_selection_method");
  field(config.inflation_distance, "inflation_distance");
  field(config.layers, "layers");
  field(config.verbosity, "verbosity");
}

using NodeSet = std::unordered_set<NodeId>;

ViewDatabase::ViewDatabase(const Config& config)
    : config(config),
      view_selector_(config::create<ViewSelector>(config.view_selection_method)) {
  CHECK(view_selector_);
  for (const auto& layer : config.layers) {
    trackers_.emplace(layer, ActiveWindowTracker());
  }
}

ViewDatabase::~ViewDatabase() {}

void ViewDatabase::updateAssignments(const DynamicSceneGraph& graph,
                                     const ArchivalCheck& should_archive) const {
  auto& queue = PipelineQueues::instance().input_features_queue;
  size_t new_views = 0;
  while (!queue.empty()) {
    ++new_views;
    views_.push_back(queue.pop());
  }

  // clean up views
  auto iter = views_.begin();
  while (iter != views_.end()) {
    const auto& view = *iter;
    if (!view) {
      iter = views_.erase(iter);
      LOG_IF(INFO, config.verbosity >= 2) << "Removed empty view!";
      continue;
    }

    const Eigen::Vector3d view_pos = view->sensor_T_world.inverse().translation();
    if (should_archive(view_pos, view->timestamp_ns)) {
      LOG_IF(INFO, config.verbosity >= 2)
          << "Archived view @ " << view->timestamp_ns << " [ns]";
      iter = views_.erase(iter);
      continue;
    }

    ++iter;
  }

  LOG_IF(INFO, config.verbosity >= 1) << "Got " << new_views << " new views!";
  if (views_.empty()) {
    LOG_IF(INFO, config.verbosity >= 1) << "No views assigned!";
    return;
  }

  LOG_IF(INFO, config.verbosity >= 1)
      << "Assigning features with " << views_.size() << " active view(s)";
  for (auto& [layer_name, layer_tracker] : trackers_) {
    auto layer = graph.findLayer(layer_name);
    if (!layer) {
      LOG(WARNING) << "Skipping unknown layer: '" << layer_name << "'";
      continue;
    }

    size_t num_assigned = 0;
    layer_tracker.clear();
    const auto layer_view = layer_tracker.view(*layer);
    for (const auto& node : layer_view) {
      auto attrs = node.tryAttributes<SemanticNodeAttributes>();
      if (!attrs) {
        LOG(ERROR) << "Invalid node " << NodeSymbol(node.id).str()
                   << " when assigning views!";
        continue;
      }

      ++num_assigned;
      view_selector_->selectFeature(views_, config.inflation_distance, *attrs);
    }

    LOG_IF(INFO, config.verbosity >= 1) << "Assigned features for " << num_assigned
                                        << " nodes for layer '" << layer_name << "'";
  }
}

}  // namespace hydra
