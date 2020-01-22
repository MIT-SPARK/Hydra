#pragma once

#include <memory>

#include <glog/logging.h>

#include <pcl/PolygonMesh.h>

#include "kimera_scene_graph/min_cut_solvers/stoer_wagner_min_cut.h"

namespace kimera {

class StoerWagnerMinCutWrapper {
 public:
  StoerWagnerMinCutWrapper() : min_cut_(nullptr) {}

  void solve(const pcl::PolygonMesh& polgyon_mesh) {
    // Build problem

    // Solve

    // Return
  }

 private:
  std::unique_ptr<StoerWagnerMinCut> min_cut_;
};

}  // namespace kimera
