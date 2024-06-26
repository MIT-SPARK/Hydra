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
#include <kimera_pgmo/hashing.h>
#include <spark_dsg/scene_graph_logger.h>

#include <memory>
#include <mutex>
#include <thread>

#include "hydra/common/common.h"
#include "hydra/common/input_queue.h"
#include "hydra/common/module.h"
#include "hydra/common/output_sink.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/frontend/freespace_places_interface.h"
#include "hydra/frontend/frontier_places_interface.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/frontend/surface_places_interface.h"
#include "hydra/odometry/pose_graph_from_odom.h"
#include "hydra/reconstruction/reconstruction_output.h"
#include "hydra/utils/log_utilities.h"

namespace kimera_pgmo {
class DeltaCompression;
class MeshCompression;
}  // namespace kimera_pgmo

namespace hydra {

class NearestNodeFinder;

class FrontendModule : public Module {
 public:
  using Ptr = std::shared_ptr<FrontendModule>;
  using FrontendInputQueue = InputQueue<ReconstructionOutput::Ptr>;
  using DynamicLayer = DynamicSceneGraphLayer;
  using PositionMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using InputCallback = std::function<void(const ReconstructionOutput&)>;
  using Sink = OutputSink<uint64_t, const DynamicSceneGraph&, const BackendInput&>;

  struct Config {
    size_t min_object_vertices = 20;
    bool lcd_use_bow_vectors = true;
    struct DeformationConfig {
      double mesh_resolution = 0.1;
      double d_graph_resolution = 1.5;
      double time_horizon = 10.0;
    } pgmo;
    MeshSegmenter::Config object_config;
    config::VirtualConfig<PoseGraphTracker> pose_graph_tracker{
        PoseGraphFromOdom::Config()};
    config::VirtualConfig<SurfacePlacesInterface> surface_places;
    config::VirtualConfig<FreespacePlacesInterface> freespace_places;
    bool use_frontiers = false;
    config::VirtualConfig<FrontierPlacesInterface> frontier_places;
    std::vector<Sink::Factory> sinks;
  } const config;

  FrontendModule(const Config& config,
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

  void addSink(const Sink::Ptr& sink);

 protected:
  virtual void initCallbacks();

  void dispatchSpin(ReconstructionOutput::Ptr msg);

  void spinOnce(const ReconstructionOutput::Ptr& msg);

  virtual void updateImpl(const ReconstructionOutput::Ptr& msg);

 protected:
  void updateMesh(const ReconstructionOutput& msg);

  void updateObjects(const ReconstructionOutput& msg);

  void updateFrontiers(const ReconstructionOutput& msg);

  void updatePlaces(const ReconstructionOutput& msg);

  void updatePlaces2d(const ReconstructionOutput& msg);

  void updateDeformationGraph(const ReconstructionOutput& msg);

  void updatePoseGraph(const ReconstructionOutput& msg);

 protected:
  void assignBowVectors(const DynamicLayer& agents);

  void invalidateMeshEdges(const kimera_pgmo::MeshDelta& delta);

  void archivePlaces2d(const NodeIdSet active_places);

  void addPlaceObjectEdges(uint64_t timestamp_ns);

  void addPlaceAgentEdges(uint64_t timestamp_ns);

  void processNextInput(const ReconstructionOutput& msg);

  void updatePlaceMeshMapping(const ReconstructionOutput& input);

 protected:
  using InputPtrCallback = std::function<void(const ReconstructionOutput::Ptr&)>;

  bool initialized_ = false;
  mutable std::mutex gvd_mutex_;
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<std::thread> spin_thread_;
  FrontendInputQueue::Ptr queue_;
  std::atomic<bool> spin_finished_;

  LcdInput::Ptr lcd_input_;
  BackendInput::Ptr backend_input_;

  SharedDsgInfo::Ptr dsg_;
  SharedModuleState::Ptr state_;
  kimera_pgmo::MeshDelta::Ptr last_mesh_update_;

  kimera_pgmo::Graph deformation_graph_;
  std::unique_ptr<kimera_pgmo::DeltaCompression> mesh_compression_;
  std::unique_ptr<kimera_pgmo::MeshCompression> deformation_compression_;
  kimera_pgmo::HashedIndexMapping deformation_remapping_;
  std::shared_ptr<kimera_pgmo::HashedIndexMapping> mesh_remapping_;

  std::unique_ptr<MeshSegmenter> segmenter_;
  std::unique_ptr<PoseGraphTracker> tracker_;
  std::unique_ptr<SurfacePlacesInterface> surface_places_;
  std::unique_ptr<FreespacePlacesInterface> freespace_places_;
  std::unique_ptr<FrontierPlacesInterface> frontier_places_;

  SceneGraphLogger frontend_graph_logger_;
  LogSetup::Ptr logs_;

  std::unique_ptr<NearestNodeFinder> places_nn_finder_;
  NodeIdSet unlabeled_place_nodes_;
  NodeIdSet previous_active_places_;
  NodeIdSet previous_active_places_2d_;
  std::map<NodeId, size_t> agent_key_map_;
  std::map<LayerPrefix, size_t> last_agent_edge_index_;
  std::map<LayerPrefix, std::set<size_t>> active_agent_nodes_;
  std::list<pose_graph_tools::BowQuery::ConstPtr> cached_bow_messages_;

  std::vector<InputCallback> input_callbacks_;
  std::vector<InputPtrCallback> input_dispatches_;
  std::vector<InputCallback> post_mesh_callbacks_;
  Sink::List sinks_;

  // TODO(lschmid): This mutex currently simply locks all data for manipulation.
  std::mutex mutex_;

 private:
  void stopImpl();

  inline static const auto registration_ =
      config::RegistrationWithConfig<FrontendModule,
                                     FrontendModule,
                                     Config,
                                     SharedDsgInfo::Ptr,
                                     SharedModuleState::Ptr,
                                     LogSetup::Ptr>("FrontendModule");
};

void declare_config(FrontendModule::Config& config);

}  // namespace hydra
