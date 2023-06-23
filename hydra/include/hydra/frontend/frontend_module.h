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
#include <kimera_pgmo/MeshFrontendInterface.h>
#include <kimera_pgmo/compression/DeltaCompression.h>
#include <spark_dsg/scene_graph_logger.h>

#include <memory>
#include <mutex>
#include <thread>

#include "hydra/common/common.h"
#include "hydra/common/input_queue.h"
#include "hydra/common/robot_prefix_config.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/frontend/frontend_config.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/reconstruction/reconstruction_output.h"
#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra {

class FrontendModule {
 public:
  using FrontendInputQueue = InputQueue<ReconstructionOutput::Ptr>;
  using InputCallback = std::function<void(const ReconstructionOutput&)>;
  using OutputCallback = std::function<void(
      const DynamicSceneGraph& graph, const BackendInput& backend_input, uint64_t)>;
  using DynamicLayer = DynamicSceneGraphLayer;

  FrontendModule(const RobotPrefixConfig& prefix,
                 const FrontendConfig& config,
                 const SharedDsgInfo::Ptr& dsg,
                 const SharedModuleState::Ptr& state);

  virtual ~FrontendModule();

  void start();

  void stop();

  void save(const std::string& output_path);

  void spin();

  bool spinOnce();

  inline FrontendInputQueue::Ptr getQueue() const { return queue_; }

  void addOutputCallback(const OutputCallback& callback);

  size_t maxSemanticLabel() const;

 protected:
  void spinOnce(const ReconstructionOutput& input);

  void updateMeshAndObjects(const ReconstructionOutput& input);

  void updateDeformationGraph(const ReconstructionOutput& input);

  void updatePlaces(const ReconstructionOutput& input);

  void updatePoseGraph(const ReconstructionOutput& input);

 protected:
  void filterPlaces(const SceneGraphLayer& places,
                    NodeIdSet& objects_to_check,
                    NodeIdSet& active_places,
                    const NodeIdSet& active_neighborhood);

  void deletePlaceNode(NodeId node_id, NodeIdSet& objects_to_check);

  void archivePlaces(const NodeIdSet active_places);

  void invalidateMeshEdges(const kimera_pgmo::MeshDelta& delta);

  void addPlaceObjectEdges(uint64_t timestamp_ns,
                           NodeIdSet* extra_objects_to_check = nullptr);

  void addPlaceAgentEdges(uint64_t timestamp_ns);

  void assignBowVectors(const DynamicLayer& agents);

  void updatePlaceMeshMapping(const ReconstructionOutput& input);

 protected:
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<std::thread> spin_thread_;
  FrontendInputQueue::Ptr queue_;

  LcdInput::Ptr lcd_input_;
  BackendInput::Ptr backend_input_;

  RobotPrefixConfig prefix_;
  FrontendConfig config_;
  std::unique_ptr<kimera::SemanticLabel2Color> label_map_;
  SharedDsgInfo::Ptr dsg_;
  SharedModuleState::Ptr state_;
  std::vector<ros::Time> mesh_timestamps_;

  kimera_pgmo::MeshFrontendInterface mesh_frontend_;
  std::unique_ptr<kimera_pgmo::DeltaCompression> mesh_compression_;
  std::shared_ptr<kimera_pgmo::VoxbloxIndexMapping> mesh_remapping_;

  std::unique_ptr<MeshSegmenter> segmenter_;
  SceneGraphLogger frontend_graph_logger_;

  std::unique_ptr<NearestNodeFinder> places_nn_finder_;
  NodeIdSet unlabeled_place_nodes_;
  NodeIdSet previous_active_places_;
  std::map<NodeId, size_t> agent_key_map_;
  std::set<NodeId> deleted_agent_edge_indices_;
  std::map<LayerPrefix, size_t> last_agent_edge_index_;
  std::list<pose_graph_tools::BowQuery::ConstPtr> cached_bow_messages_;

  std::vector<InputCallback> input_callbacks_;
  std::vector<OutputCallback> output_callbacks_;
};

}  // namespace hydra
