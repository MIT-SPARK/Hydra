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
#pragma once
#include "hydra_dsg_builder/incremental_types.h"

#include <hydra_utils/dsg_types.h>
#include <kimera_pgmo/utils/CommonStructs.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/time.h>

#include <atomic>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace hydra {
namespace incremental {

struct SharedModuleState {
  using Ptr = std::shared_ptr<SharedModuleState>;

  NodeIdSet latest_places;
  NodeIdSet archived_places;

  std::mutex mesh_mutex;
  bool have_new_mesh;
  std::shared_ptr<pcl::PolygonMesh> latest_mesh;
  std::shared_ptr<std::vector<ros::Time>> mesh_vertex_stamps;
  std::shared_ptr<std::vector<int>> mesh_vertex_graph_indices;
  std::list<pose_graph_tools::PoseGraph::ConstPtr> deformation_graphs;

  std::map<NodeId, size_t> agent_key_map;

  std::mutex lcd_mutex;
  std::queue<lcd::DsgRegistrationSolution> loop_closures;
};

}  // namespace incremental
}  // namespace hydra
