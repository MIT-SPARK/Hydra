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
#include "hydra/frontend/frontend_module.h"

#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <kimera_pgmo/compression/DeltaCompression.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <pose_graph_tools_ros/conversions.h>
#include <spark_dsg/pgmo_mesh_traits.h>
#include <tf2_eigen/tf2_eigen.h>

#include <fstream>

#include "hydra/common/hydra_config.h"
#include "hydra/frontend/gvd_place_extractor.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/utils/display_utilities.h"
#include "hydra/utils/mesh_interface.h"
#include "hydra/utils/nearest_neighbor_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;

using LabelClusters = MeshSegmenter::LabelClusters;

FrontendModule::FrontendModule(const FrontendConfig& config,
                               const SharedDsgInfo::Ptr& dsg,
                               const SharedModuleState::Ptr& state,
                               const LogSetup::Ptr& logs)
    : config_(config::checkValid(config)),
      queue_(std::make_shared<FrontendInputQueue>()),
      dsg_(dsg),
      state_(state),
      pose_graph_tracker_(new PoseGraphTracker(config.pose_graphs)) {
  kimera_pgmo::MeshFrontendConfig pgmo_config = config_.pgmo_config;
  const auto& prefix = HydraConfig::instance().getRobotPrefix();
  pgmo_config.robot_id = prefix.id;

  CHECK(dsg_ != nullptr);
  CHECK(dsg_->graph != nullptr);
  dsg_->graph->setMesh(std::make_shared<spark_dsg::Mesh>());
  dsg_->graph->createDynamicLayer(DsgLayers::AGENTS, prefix.key);

  const auto mesh_resolution = pgmo_config.mesh_resolution;
  mesh_compression_.reset(new kimera_pgmo::DeltaCompression(mesh_resolution));

  if (logs && logs->valid()) {
    logs_ = logs;

    const auto frontend_dir = logs->getLogDir("frontend");
    VLOG(VLEVEL_INFO) << "[Hydra Frontend] logging to " << frontend_dir;
    frontend_graph_logger_.setOutputPath(frontend_dir);
    frontend_graph_logger_.setLayerName(DsgLayers::OBJECTS, "objects");
    frontend_graph_logger_.setLayerName(DsgLayers::PLACES, "places");

    pgmo_config.log_output = true;
    pgmo_config.log_path = logs->getLogDir("frontend/pgmo");
  } else {
    pgmo_config.log_output = false;
  }

  CHECK(mesh_frontend_.initialize(pgmo_config));
  segmenter_.reset(new MeshSegmenter(config_.object_config));
  place_extractor_ = config_.places.create();
}

FrontendModule::~FrontendModule() {
  // intentionally the private implementation to avoid calling virtual method
  stopImpl();
}

void FrontendModule::initCallbacks() {
  initialized_ = true;
  input_callbacks_.clear();
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updateMesh, this, std::placeholders::_1));
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updateDeformationGraph, this, std::placeholders::_1));
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updatePoseGraph, this, std::placeholders::_1));

  post_mesh_callbacks_.clear();
  post_mesh_callbacks_.push_back(
      std::bind(&FrontendModule::updateObjects, this, std::placeholders::_1));

  if (!place_extractor_) {
    return;
  }

  input_callbacks_.push_back(
      std::bind(&FrontendModule::updatePlaces, this, std::placeholders::_1));
}

void FrontendModule::start() {
  initCallbacks();
  spin_thread_.reset(new std::thread(&FrontendModule::spin, this));
  LOG(INFO) << "[Hydra Frontend] started!";
}

void FrontendModule::stop() { stopImpl(); }

void FrontendModule::stopImpl() {
  should_shutdown_ = true;

  if (spin_thread_) {
    VLOG(VLEVEL_TRACE) << "[Hydra Frontend] stopping frontend!";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(VLEVEL_TRACE) << "[Hydra Frontend] stopped!";
  }

  VLOG(VLEVEL_TRACE) << "[Hydra Frontend]: " << queue_->size() << " messages left";
}

void FrontendModule::save(const LogSetup& log_setup) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto output_path = log_setup.getLogDir("frontend");
  dsg_->graph->save(output_path + "/dsg.json", false);
  dsg_->graph->save(output_path + "/dsg_with_mesh.json");

  const auto mesh = dsg_->graph->mesh();
  if (mesh && !mesh->empty()) {
    kimera_pgmo::WriteMesh(output_path + "/mesh.ply", *mesh);
  }

  if (place_extractor_) {
    place_extractor_->save(log_setup);
  }
}

std::string FrontendModule::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config_);
  return ss.str();
}

void FrontendModule::spin() {
  bool should_shutdown = false;
  spin_finished_ = true;

  ReconstructionOutput::Ptr input;
  while (!should_shutdown) {
    if (input && spin_finished_) {
      // start a spin to process input independent of this thread of
      // execution. spin_finished_ will flip to true once the thread terminates
      spin_finished_ = false;
      std::thread spin_thread(&FrontendModule::dispatchSpin, this, input);
      spin_thread.detach();
      input.reset();
    }

    bool has_data = queue_->poll();
    if (HydraConfig::instance().force_shutdown() || !has_data) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    // from this point on, we build an input packet by collating the maps together of
    // subsequent outputs. This doesn't take effect until multiple packets from the
    // reconstruction module start arriving between frontend updates
    if (!input) {
      input = queue_->front();
    } else {
      input->updateFrom(*queue_->front(), false);
    }

    processNextInput(*queue_->front());
    queue_->pop();
  }

  while (!spin_finished_) {
    // wait for current spin to finish before shutting down
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void FrontendModule::processNextInput(const ReconstructionOutput& msg) {
  // start critical section for modifying sensor data
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  const auto keyframe_id = pose_graph_tracker_->update(msg);
  if (!keyframe_id) {
    return;
  }

  if (!state_->lcd_queue) {
    return;
  }

  const auto& prefix = HydraConfig::instance().getRobotPrefix();
  const NodeSymbol node_id(prefix.key, *keyframe_id);
  sensor_cache_[node_id] = {msg.sensor_data, msg.sensor};
}

bool FrontendModule::spinOnce() {
  bool has_data = queue_->poll();
  if (!has_data) {
    return false;
  }

  ReconstructionOutput::Ptr input = queue_->front();
  processNextInput(*input);
  queue_->pop();

  spinOnce(input);
  return true;
}

void FrontendModule::addOutputCallback(const OutputCallback& callback) {
  output_callbacks_.push_back(callback);
}

void FrontendModule::addObjectVisualizationCallback(const ObjectVizCallback& cb) {
  if (segmenter_) {
    segmenter_->addVisualizationCallback(cb);
  }
}

void FrontendModule::addPlaceVisualizationCallback(const PlaceVizCallback& cb) {
  if (place_extractor_) {
    place_extractor_->addVisualizationCallback(cb);
  }
}

std::vector<bool> FrontendModule::inFreespace(const PositionMatrix& positions,
                                              double freespace_distance_m) const {
  if (!place_extractor_) {
    return std::vector<bool>(positions.cols(), false);
  }

  return place_extractor_->inFreespace(positions, freespace_distance_m);
}

void FrontendModule::dispatchSpin(ReconstructionOutput::Ptr msg) {
  spinOnce(msg);
  spin_finished_ = true;
}

void FrontendModule::spinOnce(const ReconstructionOutput::Ptr& msg) {
  if (!initialized_) {
    initCallbacks();
  }

  if (input_dispatches_.size() != input_callbacks_.size()) {
    input_dispatches_.clear();
    for (size_t i = 0; i < input_callbacks_.size(); ++i) {
      input_dispatches_.push_back(
          [this, i](const auto& msg_ptr) { input_callbacks_.at(i)(*msg_ptr); });
    }
  }

  VLOG(VLEVEL_FILE) << "[Hydra Frontend] Popped input packet @ " << msg->timestamp_ns
                    << " [ns]";
  std::lock_guard<std::mutex> lock(mutex_);
  ScopedTimer timer("frontend/spin", msg->timestamp_ns);

  if (dsg_->graph && backend_input_) {
    for (const auto& callback : output_callbacks_) {
      callback(*dsg_->graph, *backend_input_, msg->timestamp_ns);
    }
  }

  backend_input_.reset(new BackendInput());
  backend_input_->timestamp_ns = msg->timestamp_ns;

  if (state_->lcd_queue) {
    lcd_input_.reset(new LcdInput());
    lcd_input_->timestamp_ns = msg->timestamp_ns;
  }

  updateImpl(msg);

  // TODO(nathan) ideally make the copy lighter-weight
  // we need to copy over the latest updates to the backend and to LCD
  // no fancy threading: we just mark the update time and copy all changes in one go
  {  // start critical section
    std::unique_lock<std::mutex> lock(state_->backend_graph->mutex);
    state_->backend_graph->last_update_time = msg->timestamp_ns;
    state_->backend_graph->graph->mergeGraph(*dsg_->graph);
  }  // end critical section

  if (state_->lcd_queue) {
    {  // start critical section
      std::unique_lock<std::mutex> lock(state_->lcd_graph->mutex);
      state_->lcd_graph->last_update_time = msg->timestamp_ns;
      state_->lcd_graph->graph->mergeGraph(*dsg_->graph);
    }  // end critical section
  }

  backend_input_->mesh_update = last_mesh_update_;
  state_->backend_queue.push(backend_input_);
  if (state_->lcd_queue) {
    state_->lcd_queue->push(lcd_input_);
  }

  if (logs_) {
    // mutex not required because nothing is modifying the graph
    frontend_graph_logger_.logGraph(dsg_->graph);
  }
}

void FrontendModule::updateImpl(const ReconstructionOutput::Ptr& msg) {
  launchCallbacks(input_dispatches_, msg);
  if (place_extractor_) {
    updatePlaceMeshMapping(*msg);
  }
}

void FrontendModule::updateMesh(const ReconstructionOutput& input) {
  {  // start timing scope
    ScopedTimer timer("frontend/mesh_archive", input.timestamp_ns, true, 1, false);
    VLOG(VLEVEL_FILE) << "[Hydra Frontend] Clearing " << input.archived_blocks.size()
                      << " blocks from mesh";
    if (!input.archived_blocks.empty()) {
      mesh_compression_->clearArchivedBlocks(input.archived_blocks);
    }
  }  // end timing scope

  // TODO(nathan) prune archived blocks from input?

  const auto& input_mesh = input.map().getMeshLayer();
  {
    ScopedTimer timer("frontend/mesh_compression", input.timestamp_ns, true, 1, false);
    mesh_remapping_.reset(new kimera_pgmo::VoxbloxIndexMapping());
    auto mesh = input_mesh.getActiveMesh(input.archived_blocks);
    VLOG(VLEVEL_FILE) << "[Hydra Frontend] Updating mesh with " << mesh->numBlocks()
                      << " blocks";
    auto interface = getMeshInterface(*mesh);
    last_mesh_update_ =
        mesh_compression_->update(interface, input.timestamp_ns, mesh_remapping_.get());
  }  // end timing scope

  {  // start timing scope
    // TODO(nathan) we should probably have a mutex before modifying the mesh, but
    // nothing else uses it at the moment
    ScopedTimer timer("frontend/mesh_update", input.timestamp_ns, true, 1, false);
    last_mesh_update_->updateMesh(*dsg_->graph->mesh());
    invalidateMeshEdges(*last_mesh_update_);
  }  // end timing scope

  launchCallbacks(post_mesh_callbacks_, input);
}

void FrontendModule::updateObjects(const ReconstructionOutput& input) {
  if (!last_mesh_update_) {
    LOG(ERROR) << "Cannot detect objects without valid mesh";
    return;
  }

  const auto clusters =
      segmenter_->detect(input.timestamp_ns, *last_mesh_update_, std::nullopt);

  {  // start dsg critical section
    std::unique_lock<std::mutex> lock(dsg_->mutex);
    segmenter_->updateGraph(input.timestamp_ns,
                            clusters,
                            last_mesh_update_->getTotalArchivedVertices(),
                            *dsg_->graph);
    addPlaceObjectEdges(input.timestamp_ns);
  }  // end dsg critical section
}

void FrontendModule::updateDeformationGraph(const ReconstructionOutput& input) {
  {  // start timing scope
    ScopedTimer timer(
        "frontend/dgraph_compresssion", input.timestamp_ns, true, 1, false);
    const auto time_ns = std::chrono::nanoseconds(input.timestamp_ns);
    double time_s = std::chrono::duration_cast<std::chrono::seconds>(time_ns).count();
    auto interface = getMeshInterface(input.map().getMeshLayer());
    mesh_frontend_.processMeshGraph(interface, time_s);
  }  // end timing scope

  if (backend_input_) {
    backend_input_->deformation_graph.reset(new pose_graph_tools_msgs::PoseGraph(
        mesh_frontend_.getLastProcessedMeshGraph()));
  }
}

void FrontendModule::updatePlaces(const ReconstructionOutput& input) {
  if (!place_extractor_) {
    return;
  }

  NodeIdSet active_nodes;
  place_extractor_->detect(input);
  {  // start graph critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    place_extractor_->updateGraph(input.timestamp_ns, *dsg_->graph);

    active_nodes = place_extractor_->getActiveNodes();
    const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);
    places_nn_finder_.reset(new NearestNodeFinder(places, active_nodes));
    addPlaceAgentEdges(input.timestamp_ns);
    addPlaceObjectEdges(input.timestamp_ns);
    state_->latest_places = active_nodes;
  }  // end graph update critical section

  archivePlaces(active_nodes);
  previous_active_places_ = active_nodes;
}

void FrontendModule::updatePoseGraph(const ReconstructionOutput& input) {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  ScopedTimer timer("frontend/update_posegraph", input.timestamp_ns);
  const auto& prefix = HydraConfig::instance().getRobotPrefix();

  std::vector<uint64_t> new_nodes;
  {  // start pose-graph-tracker critical section
    // we need to lock here because the pose graph tracker is updated from
    // reconstruction in a different thread
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    new_nodes = pose_graph_tracker_->addAgentNodes(*dsg_->graph, prefix);
    pose_graph_tracker_->fillPoseGraphs(*backend_input_, new_nodes);

    if (lcd_input_) {
      lcd_input_->new_agent_nodes = new_nodes;
      for (const auto new_node : new_nodes) {
        auto iter = sensor_cache_.find(new_node);
        CHECK(iter != sensor_cache_.end())
            << "Unable to find sensor data for node " << printNodeId(new_node);
        lcd_input_->new_sensor_data.push_back(iter->second.data);
        lcd_input_->new_sensors.push_back(iter->second.sensor);
        sensor_cache_.erase(iter);
      }
    }
  }  // end critical section for parsing sensor data

  VLOG(VLEVEL_FILE) << "[Hydra Frontend] Added " << new_nodes.size() << " new nodes @ "
                    << input.timestamp_ns << " [ns] for prefix '" << prefix.key << "'";
  addPlaceAgentEdges(input.timestamp_ns);
}

void FrontendModule::invalidateMeshEdges(const kimera_pgmo::MeshDelta& delta) {
  if (config_.min_object_vertices == 0) {
    return;
  }

  std::unique_lock<std::mutex> lock(dsg_->mutex);

  std::vector<NodeId> objects_to_delete;
  const auto& objects = dsg_->graph->getLayer(DsgLayers::OBJECTS);
  for (const auto& id_node_pair : objects.nodes()) {
    auto& attrs = id_node_pair.second->attributes<ObjectNodeAttributes>();

    auto iter = attrs.mesh_connections.begin();
    while (iter != attrs.mesh_connections.end()) {
      if (delta.deleted_indices.count(*iter)) {
        iter = attrs.mesh_connections.erase(iter);
        continue;
      }

      auto map_iter = delta.prev_to_curr.find(*iter);
      if (map_iter != delta.prev_to_curr.end()) {
        *iter = map_iter->second;
      }

      ++iter;
    }

    if (attrs.mesh_connections.size() < config_.min_object_vertices) {
      objects_to_delete.push_back(id_node_pair.first);
    }
  }

  for (const auto& node : objects_to_delete) {
    dsg_->graph->removeNode(node);
  }
}

void FrontendModule::archivePlaces(const NodeIdSet active_places) {
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

    // find node ids that are valid, but outside active place window
    for (const auto& prev : previous_active_places_) {
      if (active_places.count(prev)) {
        continue;
      }

      const auto has_prev_node = dsg_->graph->getNode(prev);
      if (!has_prev_node) {
        continue;
      }

      const SceneGraphNode& prev_node = has_prev_node.value();
      prev_node.attributes().is_active = false;
      if (lcd_input_) {
        lcd_input_->archived_places.insert(prev);
      }
    }

  }  // end graph update critical section
}

void FrontendModule::addPlaceObjectEdges(uint64_t timestamp_ns) {
  ScopedTimer timer("frontend/place_object_edges", timestamp_ns);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  for (const auto& object_id : segmenter_->getActiveNodes()) {
    const auto object_opt = dsg_->graph->getNode(object_id);
    if (!object_opt) {
      continue;
    }

    const SceneGraphNode& object_node = *object_opt;
    const auto parent_opt = object_node.getParent();
    if (parent_opt) {
      dsg_->graph->removeEdge(object_id, *parent_opt);
    }

    const Eigen::Vector3d object_position = dsg_->graph->getPosition(object_id);
    places_nn_finder_->find(
        object_position, 1, false, [&](NodeId place_id, size_t, double) {
          dsg_->graph->insertEdge(place_id, object_id);
        });
  }
}

void FrontendModule::addPlaceAgentEdges(uint64_t timestamp_ns) {
  ScopedTimer timer("frontend/place_agent_edges", timestamp_ns);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  for (const auto& pair : dsg_->graph->dynamicLayersOfType(DsgLayers::AGENTS)) {
    const LayerPrefix prefix = pair.first;
    const auto& layer = *pair.second;

    if (!last_agent_edge_index_.count(prefix)) {
      last_agent_edge_index_[prefix] = 0;
      active_agent_nodes_[prefix] = {};
    }

    auto& curr_active = active_agent_nodes_[prefix];
    for (size_t i = last_agent_edge_index_[prefix]; i < layer.numNodes(); ++i) {
      curr_active.insert(i);
    }

    auto iter = curr_active.begin();
    while (iter != curr_active.end()) {
      const auto& node = layer.getNodeByIndex(*iter)->get();
      const auto prev_parent = node.getParent();
      if (prev_parent) {
        dsg_->graph->removeEdge(node.id, *prev_parent);
      }

      bool found = false;
      NodeId parent = 0;
      places_nn_finder_->find(
          node.attributes().position, 1, false, [&](NodeId place_id, size_t, double) {
            CHECK(dsg_->graph->insertEdge(place_id, prefix.makeId(*iter)));
            found = true;
            parent = place_id;
          });

      if (!found) {
        ++iter;
        continue;
      }

      const auto& parent_attrs = dsg_->graph->getNode(parent)->get().attributes();
      if (!parent_attrs.is_active) {
        iter = curr_active.erase(iter);
        continue;
      }

      ++iter;
    }

    last_agent_edge_index_[prefix] = layer.numNodes();
  }
}

size_t remapConnections(const kimera_pgmo::VoxbloxIndexMapping& remapping,
                        const voxblox::IndexSet& archived_blocks,
                        const std::vector<NearestVertexInfo>& connections,
                        std::vector<size_t>& indices) {
  size_t num_invalid = 0;
  for (const auto& connection : connections) {
    voxblox::BlockIndex idx = Eigen::Map<const voxblox::BlockIndex>(connection.block);
    if (archived_blocks.count(idx)) {
      continue;
    }

    const auto iter = remapping.find(idx);
    if (iter == remapping.end()) {
      num_invalid++;
      continue;
    }

    const auto viter = iter->second.find(connection.vertex);
    if (viter == iter->second.end()) {
      num_invalid++;
    } else {
      indices.push_back(viter->second);
    }
  }
  return num_invalid;
}

using MeshIndexMap = voxblox::AnyIndexHashMapType<size_t>::type;

// TODO(nathan) maybe move this somewhere else...
void FrontendModule::updatePlaceMeshMapping(const ReconstructionOutput& input) {
  ScopedTimer timer("frontend/place_mesh_mapping", input.timestamp_ns);
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);
  const auto& graph_mapping = mesh_frontend_.getVoxbloxMsgToGraphMapping();

  voxblox::BlockIndexList allocated_list;
  input.map().getMeshLayer().getAllocatedBlockIndices(allocated_list);

  voxblox::IndexSet allocated(allocated_list.begin(), allocated_list.end());
  voxblox::IndexSet archived(input.archived_blocks.begin(),
                             input.archived_blocks.end());

  size_t num_missing = 0;
  size_t num_deform_invalid = 0;
  size_t num_mesh_invalid = 0;
  size_t num_semantic_invalid = 0;
  for (const auto& id_node_pair : places.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (!attrs.is_active) {
      continue;
    }

    if (attrs.voxblox_mesh_connections.empty()) {
      ++num_missing;
      continue;
    }

    // reset connections
    attrs.deformation_connections.clear();
    attrs.pcl_mesh_connections.clear();
    attrs.mesh_vertex_labels.clear();
    num_deform_invalid += remapConnections(graph_mapping,
                                           archived,
                                           attrs.voxblox_mesh_connections,
                                           attrs.deformation_connections);
    if (mesh_remapping_) {
      num_mesh_invalid += remapConnections(*mesh_remapping_,
                                           archived,
                                           attrs.voxblox_mesh_connections,
                                           attrs.pcl_mesh_connections);
    }
  }

  if (config_.validate_vertices) {
    CHECK_EQ(num_deform_invalid, 0u);
    CHECK_EQ(num_mesh_invalid, 0u);
    CHECK_EQ(num_missing, 0u);
    CHECK_EQ(num_semantic_invalid, 0u);
  }
}

}  // namespace hydra
