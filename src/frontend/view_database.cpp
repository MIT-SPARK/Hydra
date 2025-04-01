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
      continue;
    }

    const auto view_pos = view->sensor_T_world.inverse().translation();
    if (should_archive(view_pos, view->timestamp_ns)) {
      iter = views_.erase(iter);
      continue;
    }

    ++iter;
  }

  VLOG(2) << "Got " << new_views << " new views!";
  if (views_.empty()) {
    VLOG(2) << "No views assigned!";
    return;
  }

  VLOG(2) << "Assigning features with " << views_.size() << " active view(s)";
  for (const auto& [layer_name, layer_tracker] : trackers_) {
    auto layer = graph.findLayer(layer_name);
    if (!layer) {
      LOG(WARNING) << "Skipping unknown layer: '" << layer_name << "'";
      continue;
    }

    size_t num_assigned = 0;
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

    VLOG(2) << "Assigned features for " << num_assigned << " nodes for layer '"
            << layer_name << "'";
  }
}

}  // namespace hydra
