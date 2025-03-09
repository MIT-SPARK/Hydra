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
#include <config_utilities/virtual_config.h>
#include <kimera_pgmo/hashing.h>
#include <pose_graph_tools/bow_query.h>
#include <spark_dsg/scene_graph_logger.h>

#include <memory>
#include <mutex>
#include <thread>

#include "hydra/active_window/active_window_output.h"
#include "hydra/backend/backend_input.h"
#include "hydra/common/message_queue.h"
#include "hydra/common/module.h"
#include "hydra/common/output_sink.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/frontend/freespace_places_interface.h"
#include "hydra/frontend/graph_connector.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/frontend/surface_places_interface.h"
#include "hydra/frontend/view_database.h"
#include "hydra/loop_closure/lcd_input.h"
#include "hydra/odometry/pose_graph_from_odom.h"
#include "hydra/utils/log_utilities.h"

namespace kimera_pgmo {
class DeltaCompression;
class MeshCompression;
}  // namespace kimera_pgmo

namespace hydra {

class FrontierExtractor;
class NearestNodeFinder;
struct VolumetricWindow;

class GraphBuilder : public Module {
 public:
  using Ptr = std::shared_ptr<GraphBuilder>;
  using InputQueue = MessageQueue<ActiveWindowOutput::Ptr>;
  using InputCallback = std::function<void(const ActiveWindowOutput&)>;
  using Sink = OutputSink<uint64_t, const DynamicSceneGraph&, const BackendInput&>;

  struct Config {
    bool lcd_use_bow_vectors = true;
    struct DeformationConfig {
      double mesh_resolution = 0.1;
      double d_graph_resolution = 1.5;
      double time_horizon = 10.0;
    } pgmo;
    GraphUpdater::Config graph_updater{{{"objects", {'O', std::nullopt}}}};
    GraphConnector::Config graph_connector;
    bool enable_mesh_objects = true;
    MeshSegmenter::Config object_config;
    config::VirtualConfig<PoseGraphTracker> pose_graph_tracker{
        PoseGraphFromOdom::Config()};
    config::VirtualConfig<SurfacePlacesInterface> surface_places;
    config::VirtualConfig<FreespacePlacesInterface> freespace_places;
    bool use_frontiers = false;
    config::VirtualConfig<FrontierExtractor> frontier_places;
    ViewDatabase::Config view_database;
    std::vector<Sink::Factory> sinks;
  } const config;

  GraphBuilder(const Config& config,
               const SharedDsgInfo::Ptr& dsg,
               const SharedModuleState::Ptr& state,
               const LogSetup::Ptr& logs = nullptr);

  virtual ~GraphBuilder();

  void start() override;

  void stop() override;

  void save(const LogSetup& log_setup) override;

  std::string printInfo() const override;

  virtual void spin();

  bool spinOnce();

  inline InputQueue::Ptr queue() const { return queue_; }

  void addSink(const Sink::Ptr& sink);

 protected:
  void addInputCallback(InputCallback callback);

  void addPostMeshCallback(InputCallback callback);

  void dispatchSpin(ActiveWindowOutput::Ptr msg);

  void spinOnce(const ActiveWindowOutput::Ptr& msg);

  virtual void updateImpl(const ActiveWindowOutput::Ptr& msg);

 protected:
  void updateMesh(const ActiveWindowOutput& msg);

  void updateObjects(const ActiveWindowOutput& msg);

  void updateFrontiers(const ActiveWindowOutput& msg);

  void updatePlaces(const ActiveWindowOutput& msg);

  void updatePlaces2d(const ActiveWindowOutput& msg);

  void updateDeformationGraph(const ActiveWindowOutput& msg);

  void updatePoseGraph(const ActiveWindowOutput& msg);

 protected:
  void assignBowVectors(const DynamicSceneGraphLayer& agents);

  void archivePlaces2d(const NodeIdSet active_places);

  void processNextInput(const ActiveWindowOutput& msg);

  void updatePlaceMeshMapping(const ActiveWindowOutput& input);

 protected:
  uint64_t sequence_number_;
  mutable std::mutex gvd_mutex_;
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<std::thread> spin_thread_;
  InputQueue::Ptr queue_;
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

  GraphUpdater graph_updater_;
  GraphConnector graph_connector_;

  std::unique_ptr<VolumetricWindow> map_window_;
  std::unique_ptr<MeshSegmenter> segmenter_;
  std::unique_ptr<PoseGraphTracker> tracker_;
  std::unique_ptr<SurfacePlacesInterface> surface_places_;
  std::unique_ptr<FreespacePlacesInterface> freespace_places_;
  std::unique_ptr<FrontierExtractor> frontier_places_;
  ViewDatabase view_database_;

  SceneGraphLogger frontend_graph_logger_;
  LogSetup::Ptr logs_;
  MessageQueue<PoseGraphPacket> pose_graph_updates_;

  NodeIdSet previous_active_places_;
  NodeIdSet previous_active_places_2d_;
  std::map<NodeId, size_t> agent_key_map_;
  std::list<pose_graph_tools::BowQuery::ConstPtr> cached_bow_messages_;

  Sink::List sinks_;

  // TODO(lschmid): This mutex currently simply locks all data for manipulation.
  std::mutex mutex_;

 private:
  void stopImpl();

  std::vector<std::function<void(ActiveWindowOutput::Ptr)>> input_callbacks_;
  std::vector<std::function<void(const ActiveWindowOutput&)>> post_mesh_callbacks_;
};

void declare_config(GraphBuilder::Config& config);

}  // namespace hydra
