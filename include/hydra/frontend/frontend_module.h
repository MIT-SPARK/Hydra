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
#include <config_utilities/factory.h>
#include <kimera_pgmo/MeshFrontendInterface.h>
#include <spark_dsg/scene_graph_logger.h>

#include <memory>
#include <mutex>
#include <thread>

#include "hydra/common/common.h"
#include "hydra/common/input_queue.h"
#include "hydra/common/module.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/frontend/frontend_config.h"
#include "hydra/frontend/place_extractor_interface.h"
#include "hydra/reconstruction/reconstruction_output.h"
#include "hydra/utils/log_utilities.h"

namespace kimera_pgmo {
class DeltaCompression;
}  // namespace kimera_pgmo

namespace hydra {

class NearestNodeFinder;
class MeshSegmenter;

class FrontendModule : public Module {
 public:
  using Ptr = std::shared_ptr<FrontendModule>;
  using FrontendInputQueue = InputQueue<ReconstructionOutput::Ptr>;
  using InputCallback = std::function<void(const ReconstructionOutput&)>;
  using OutputCallback = std::function<void(
      const DynamicSceneGraph& graph, const BackendInput& backend_input, uint64_t)>;
  using DynamicLayer = DynamicSceneGraphLayer;
  using PositionMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using PlaceVizCallback = std::function<void(uint64_t,
                                              const voxblox::Layer<places::GvdVoxel>&,
                                              const places::GraphExtractorInterface*)>;

  FrontendModule(const FrontendConfig& config,
                 const SharedDsgInfo::Ptr& dsg,
                 const SharedModuleState::Ptr& state,
                 const LogSetup::Ptr& logs = nullptr);

  virtual ~FrontendModule();

  void start() override;

  void stop() override;

  void save(const LogSetup& log_setup) override;

  std::string printInfo() const override;

  virtual void spin();

  bool spinOnce();

  inline FrontendInputQueue::Ptr getQueue() const { return queue_; }

  void addOutputCallback(const OutputCallback& callback);

  void addPlaceVisualizationCallback(const PlaceVizCallback& callback);

  // takes in a 3xN matrix
  std::vector<bool> inFreespace(const PositionMatrix& positions,
                                double freespace_distance_m) const;

 protected:
  virtual void initCallbacks();

  void spinOnce(const ReconstructionOutput& msg);

 protected:
  void updateMesh(const ReconstructionOutput& msg);

  void updateObjects(const ReconstructionOutput& msg);

  void updatePlaces(const ReconstructionOutput& msg);

  void updateDeformationGraph(const ReconstructionOutput& msg);

  void updatePoseGraph(const ReconstructionOutput& msg);

 protected:
  void assignBowVectors(const DynamicLayer& agents);

  void invalidateMeshEdges(const kimera_pgmo::MeshDelta& delta);

  void archivePlaces(const NodeIdSet active_places);

  void addPlaceObjectEdges(uint64_t timestamp_ns);

  void addPlaceAgentEdges(uint64_t timestamp_ns);

  void updatePlaceMeshMapping(const ReconstructionOutput& input);

 protected:
  const FrontendConfig config_;

  bool initialized_ = false;
  mutable std::mutex gvd_mutex_;
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<std::thread> spin_thread_;
  FrontendInputQueue::Ptr queue_;

  LcdInput::Ptr lcd_input_;
  BackendInput::Ptr backend_input_;

  SharedDsgInfo::Ptr dsg_;
  SharedModuleState::Ptr state_;
  kimera_pgmo::MeshDelta::Ptr last_mesh_update_;

  kimera_pgmo::MeshFrontendInterface mesh_frontend_;
  std::unique_ptr<kimera_pgmo::DeltaCompression> mesh_compression_;
  std::shared_ptr<kimera_pgmo::VoxbloxIndexMapping> mesh_remapping_;

  std::unique_ptr<MeshSegmenter> segmenter_;
  std::unique_ptr<PlaceExtractorInterface> place_extractor_;
  SceneGraphLogger frontend_graph_logger_;
  LogSetup::Ptr logs_;

  std::unique_ptr<NearestNodeFinder> places_nn_finder_;
  NodeIdSet unlabeled_place_nodes_;
  NodeIdSet previous_active_places_;
  std::map<NodeId, size_t> agent_key_map_;
  std::map<LayerPrefix, size_t> last_agent_edge_index_;
  std::map<LayerPrefix, std::set<size_t>> active_agent_nodes_;
  std::list<pose_graph_tools_msgs::BowQuery::ConstPtr> cached_bow_messages_;

  std::vector<InputCallback> input_callbacks_;
  std::vector<InputCallback> post_mesh_callbacks_;
  std::vector<OutputCallback> output_callbacks_;

   // TODO(lschmid): This mutex currently simply locks all data for manipulation.
  std::mutex mutex_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<FrontendModule,
                                     FrontendModule,
                                     FrontendConfig,
                                     SharedDsgInfo::Ptr,
                                     SharedModuleState::Ptr,
                                     LogSetup::Ptr>("FrontendModule");
};

}  // namespace hydra
