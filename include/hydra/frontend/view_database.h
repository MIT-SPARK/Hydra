#pragma once
#include "hydra/common/dsg_types.h"
#include "hydra/frontend/view_selector.h"
#include "hydra/utils/active_window_tracker.h"

namespace hydra {

struct InputData;

class ViewDatabase {
 public:
  using Ptr = std::shared_ptr<ViewDatabase>;
  using ArchivalCheck =
      std::function<bool(const Eigen::Vector3d pos, uint64_t time_ns)>;
  struct Config {
    //! @brief Method to control mapping from views to resulting feature
    std::string view_selection_method = "fusion";
    //! @brief Amount to inflate field-of-view by
    double inflation_distance = 0.0;
    //! @brief Layers to assign views for
    std::vector<std::string> layers{DsgLayers::PLACES, DsgLayers::MESH_PLACES};
    //! @brief Verbosity for database
    int verbosity = 0;
  } const config;

  explicit ViewDatabase(const Config& config);

  ~ViewDatabase();

  void updateAssignments(const DynamicSceneGraph& graph,
                         const ArchivalCheck& should_archive) const;

 protected:
  mutable ViewSelector::FeatureList views_;
  std::unique_ptr<ViewSelector> view_selector_;
  mutable std::map<std::string, ActiveWindowTracker> trackers_;
};

void declare_config(ViewDatabase::Config& config);

}  // namespace hydra
