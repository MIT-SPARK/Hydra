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

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <kimera_pgmo/compression/block_compression.h>
#include <kimera_pgmo/compression/delta_compression.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <kimera_pgmo/utils/mesh_io.h>

#include <fstream>

#include "hydra/common/global_info.h"
#include "hydra/frontend/frontier_extractor.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/frontend/place_2d_segmenter.h"
#include "hydra/frontend/place_mesh_connector.h"
#include "hydra/utils/display_utilities.h"
#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/nearest_neighbor_utilities.h"
#include "hydra/utils/pgmo_mesh_interface.h"
#include "hydra/utils/pgmo_mesh_traits.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;

void declare_config(FrontendModule::Config& config) {
  using namespace config;
  name("FrontendModule::Config");
  field(config.min_object_vertices, "min_object_vertices");
  field(config.min_object_vertices, "min_object_vertices");
  field(config.lcd_use_bow_vectors, "lcd_use_bow_vectors");

  {
    NameSpace ns("pgmo");
    field(config.pgmo.mesh_resolution, "mesh_resolution");
    field(config.pgmo.d_graph_resolution, "d_graph_resolution");
    field(config.pgmo.time_horizon, "time_horizon");
  }

  field(config.object_config, "objects");
  config.pose_graph_tracker.setOptional();
  field(config.pose_graph_tracker, "pose_graph_tracker");
  // surface (i.e., 2D) places
  config.surface_places.setOptional();
  field(config.surface_places, "surface_places");
  // freespace (i.e., 3D) places
  config.freespace_places.setOptional();
  field(config.freespace_places, "freespace_places");
  // frontier (i.e. 3D, boundary to unknown space) places
  field(config.use_frontiers, "use_frontiers");
  config.frontier_places.setOptional();
  field(config.frontier_places, "frontier_places");
}

FrontendModule::FrontendModule(const Config& config,
                               const SharedDsgInfo::Ptr& dsg,
                               const SharedModuleState::Ptr& state,
                               const LogSetup::Ptr& logs)
    : config(config::checkValid(config)),
      queue_(std::make_shared<FrontendInputQueue>()),
      dsg_(dsg),
      state_(state),
      segmenter_(new MeshSegmenter(config.object_config)),
      tracker_(config.pose_graph_tracker.create()),
      surface_places_(config.surface_places.create()),
      freespace_places_(config.freespace_places.create()),
      frontier_places_(config.frontier_places.create()),
      sinks_(Sink::instantiate(config.sinks)) {
  if (!config.use_frontiers) {
    frontier_places_.reset();
  }

  if (config.lcd_use_bow_vectors) {
    state_->bow_queue = std::make_shared<SharedModuleState::BowQueue>();
  }

  CHECK(dsg_ != nullptr);
  CHECK(dsg_->graph != nullptr);
  dsg_->graph->setMesh(std::make_shared<spark_dsg::Mesh>());
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  dsg_->graph->createDynamicLayer(DsgLayers::AGENTS, prefix.key);

  mesh_compression_.reset(
      new kimera_pgmo::DeltaCompression(config.pgmo.mesh_resolution));
  deformation_compression_.reset(
      new kimera_pgmo::BlockCompression(config.pgmo.d_graph_resolution));

  if (logs && logs->valid()) {
    logs_ = logs;

    const auto frontend_dir = logs->getLogDir("frontend");
    VLOG(1) << "[Hydra Frontend] logging to " << frontend_dir;
    frontend_graph_logger_.setOutputPath(frontend_dir);
    frontend_graph_logger_.setLayerName(DsgLayers::OBJECTS, "objects");
    frontend_graph_logger_.setLayerName(DsgLayers::PLACES, "places");
    frontend_graph_logger_.setLayerName(DsgLayers::MESH_PLACES, "places 2d");
  }
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
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updatePlaces, this, std::placeholders::_1));
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updateFrontiers, this, std::placeholders::_1));

  post_mesh_callbacks_.clear();
  post_mesh_callbacks_.push_back(
      std::bind(&FrontendModule::updateObjects, this, std::placeholders::_1));
  post_mesh_callbacks_.push_back(
      std::bind(&FrontendModule::updatePlaces2d, this, std::placeholders::_1));
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
    VLOG(2) << "[Hydra Frontend] stopping frontend!";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Hydra Frontend] stopped!";
  }

  VLOG(2) << "[Hydra Frontend]: " << queue_->size() << " messages left";
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

  if (freespace_places_) {
    freespace_places_->save(log_setup);
  }
}

std::string FrontendModule::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config);
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
    if (GlobalInfo::instance().force_shutdown() || !has_data) {
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

void FrontendModule::processNextInput(const ReconstructionOutput& /*msg*/) {
  // TODO(nathan) cache information if required
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

void FrontendModule::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void FrontendModule::dispatchSpin(ReconstructionOutput::Ptr msg) {
  spinOnce(msg);
  spin_finished_ = true;
}

void FrontendModule::spinOnce(const ReconstructionOutput::Ptr& msg) {
  if (!initialized_) {
    initCallbacks();
  }

  VLOG(5) << "[Hydra Frontend] Popped input packet @ " << msg->timestamp_ns << " [ns]";
  std::lock_guard<std::mutex> lock(mutex_);
  ScopedTimer timer("frontend/spin", msg->timestamp_ns);

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
    ScopedTimer merge_timer("frontend/merge_graph", msg->timestamp_ns);
    state_->backend_graph->last_update_time = msg->timestamp_ns;
    state_->backend_graph->graph->mergeGraph(*dsg_->graph);
  }  // end critical section

  if (state_->lcd_queue) {
    // n.b., critical section in this scope!
    std::unique_lock<std::mutex> lock(state_->lcd_graph->mutex);
    ScopedTimer merge_timer("frontend/merge_lcd_graph", msg->timestamp_ns);
    state_->lcd_graph->last_update_time = msg->timestamp_ns;
    state_->lcd_graph->graph->mergeGraph(*dsg_->graph);
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

  if (dsg_->graph && backend_input_) {
    ScopedTimer sink_timer("frontend/sinks", msg->timestamp_ns);
    Sink::callAll(sinks_, msg->timestamp_ns, *dsg_->graph, *backend_input_);
  }
}

void FrontendModule::updateImpl(const ReconstructionOutput::Ptr& msg) {
  // TODO(nathan) make this more formal
  if (input_dispatches_.size() != input_callbacks_.size()) {
    // initialize dispatch versions of callbacks if not initialized
    input_dispatches_.clear();
    for (size_t i = 0; i < input_callbacks_.size(); ++i) {
      input_dispatches_.push_back(
          [this, i](const auto& msg_ptr) { input_callbacks_.at(i)(*msg_ptr); });
    }
  }

  {  // start timing scope
    ScopedTimer timer("frontend/launch_callbacks", msg->timestamp_ns, true, 1, false);
    launchCallbacks(input_dispatches_, msg);
  }
  updatePlaceMeshMapping(*msg);
}

void FrontendModule::updateMesh(const ReconstructionOutput& input) {
  {  // start timing scope
    ScopedTimer timer("frontend/mesh_archive", input.timestamp_ns, true, 1, false);
    VLOG(5) << "[Hydra Frontend] Clearing " << input.archived_blocks.size()
            << " blocks from mesh";
    if (!input.archived_blocks.empty()) {
      mesh_compression_->clearArchivedBlocks(input.archived_blocks);
    }
  }  // end timing scope

  // TODO(nathan) prune archived blocks from input?

  const auto& input_mesh = input.map().getMeshLayer();
  {
    ScopedTimer timer("frontend/mesh_compression", input.timestamp_ns, true, 1, false);
    mesh_remapping_ = std::make_shared<kimera_pgmo::HashedIndexMapping>();
    auto mesh = getActiveMesh(input_mesh, input.archived_blocks);
    VLOG(5) << "[Hydra Frontend] Updating mesh with " << mesh->numBlocks() << " blocks";
    auto interface = PgmoMeshLayerInterface(*mesh);
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

  ScopedTimer timer(
      "frontend/launch_postmesh_callbacks", input.timestamp_ns, true, 1, false);
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

using PgmoCloud = pcl::PointCloud<pcl::PointXYZRGBA>;

void FrontendModule::updateDeformationGraph(const ReconstructionOutput& input) {
  ScopedTimer timer("frontend/dgraph_compresssion", input.timestamp_ns, true, 1, false);
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto time_ns = std::chrono::nanoseconds(input.timestamp_ns);
  double time_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(time_ns).count();
  auto interface = PgmoMeshLayerInterface(input.map().getMeshLayer());

  PgmoCloud new_vertices;
  std::vector<size_t> new_indices;
  std::vector<pcl::Vertices> new_triangles;
  deformation_compression_->pruneStoredMesh(time_s - config.pgmo.time_horizon);
  deformation_compression_->compressAndIntegrate(interface,
                                                 new_vertices,
                                                 new_triangles,
                                                 new_indices,
                                                 deformation_remapping_,
                                                 time_s);

  PgmoCloud::Ptr vertices(new PgmoCloud());
  deformation_compression_->getVertices(vertices);

  std::vector<kimera_pgmo::Edge> new_edges;
  if (new_indices.size() > 0 && new_triangles.size() > 0) {
    // Add nodes and edges to graph
    new_edges = deformation_graph_.addPointsAndSurfaces(new_indices, new_triangles);
  }

  if (backend_input_) {
    backend_input_->deformation_graph = kimera_pgmo::makePoseGraph(
        prefix.id, time_s, new_edges, new_indices, *vertices);
  }
}

void FrontendModule::updateFrontiers(const ReconstructionOutput& input) {
  if (!frontier_places_) {
    return;
  }

  frontier_places_->updateRecentBlocks(input.world_t_body, input.map().blockSize());

  {  // start graph critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

    {  // start timing scope
      ScopedTimer timer("frontend/frontiers", input.timestamp_ns, true, 1, false);
      NodeIdSet active_nodes = freespace_places_->getActiveNodes();

      const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);
      places_nn_finder_.reset(new NearestNodeFinder(places, active_nodes));

      frontier_places_->detectFrontiers(input, *dsg_->graph, *places_nn_finder_);
      frontier_places_->addFrontiers(
          input.timestamp_ns, *dsg_->graph, *places_nn_finder_);
    }  // end timing scope
  }    // end graph update critical section
}

void FrontendModule::updatePlaces(const ReconstructionOutput& input) {
  if (!freespace_places_) {
    return;
  }

  NodeIdSet active_nodes;
  freespace_places_->detect(input);
  {  // start graph critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    freespace_places_->updateGraph(input.timestamp_ns, *dsg_->graph);

    // TODO(nathan) fold this into updateGraph and return archived instead
    // find node ids that are valid, but outside active place window
    active_nodes = freespace_places_->getActiveNodes();
    for (const auto& prev : previous_active_places_) {
      if (active_nodes.count(prev)) {
        continue;
      }

      const auto has_prev_node = dsg_->graph->findNode(prev);
      if (!has_prev_node) {
        continue;
      }

      const auto& prev_node = *has_prev_node;
      prev_node.attributes().is_active = false;
      if (lcd_input_) {
        lcd_input_->archived_places.insert(prev);
      }

      if (frontier_places_) {
        frontier_places_->archived_places_.push_back(prev);
      }
    }
    previous_active_places_ = active_nodes;

    const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);
    places_nn_finder_.reset(new NearestNodeFinder(places, active_nodes));
    addPlaceAgentEdges(input.timestamp_ns);
    addPlaceObjectEdges(input.timestamp_ns);
    state_->latest_places = active_nodes;
  }  // end graph update critical section
}

void FrontendModule::updatePlaces2d(const ReconstructionOutput& input) {
  if (!surface_places_) {
    return;
  }

  if (!last_mesh_update_) {
    LOG(ERROR) << "Cannot detect places without valid mesh";
    return;
  }

  NodeIdSet active_nodes;

  {  // start timing scope
    ScopedTimer timer("frontend/places_2d", input.timestamp_ns, true, 1, false);
    surface_places_->detect(input, *last_mesh_update_, *dsg_->graph);
    {  // start graph critical section
      std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

      surface_places_->updateGraph(input.timestamp_ns, input, *dsg_->graph);
      // TODO(nathan) unify places so that active places get populated correctly
      // depending on run configuration
    }  // end graph update critical section

    archivePlaces2d(active_nodes);
    previous_active_places_2d_ = active_nodes;
  }  // end timing scope
}

void FrontendModule::updatePoseGraph(const ReconstructionOutput& input) {
  if (!tracker_) {
    LOG_FIRST_N(WARNING, 1)
        << "PoseGraphTracker disabled, no agent layer will be created";
    return;
  }

  std::unique_lock<std::mutex> lock(dsg_->mutex);
  ScopedTimer timer("frontend/update_posegraph", input.timestamp_ns);
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto& agents = dsg_->graph->getLayer(DsgLayers::AGENTS, prefix.key);

  if (lcd_input_) {
    lcd_input_->new_agent_nodes.clear();
  }

  const auto packet = tracker_->update(input.timestamp_ns, input.world_T_body());
  if (backend_input_) {
    backend_input_->agent_updates = packet;
  }

  for (const auto& pose_graph : packet.pose_graphs) {
    if (!pose_graph || pose_graph->nodes.empty()) {
      continue;
    }

    for (const auto& node : pose_graph->nodes) {
      if (node.key < agents.numNodes()) {
        continue;
      }

      Eigen::Vector3d position = node.pose.translation();
      Eigen::Quaterniond rotation(node.pose.linear());

      // TODO(nathan) implicit assumption that pgmo ids are sequential starting at 0
      // TODO(nathan) implicit assumption that gtsam symbol and dsg node symbol are
      // same
      NodeSymbol pgmo_key(prefix.key, node.key);

      const std::chrono::nanoseconds stamp(node.stamp_ns);
      VLOG(5) << "[Hydra Frontend] Adding agent " << agents.nodes().size() << " @ "
              << stamp.count() << " [ns] for layer " << agents.prefix.str()
              << " (key: " << node.key << ")";
      auto attrs = std::make_unique<AgentNodeAttributes>(rotation, position, pgmo_key);
      if (!dsg_->graph->emplaceNode(
              agents.id, agents.prefix, stamp, std::move(attrs))) {
        VLOG(1) << "[Hydra Frontend] repeated timestamp " << stamp.count()
                << "[ns] found";
        continue;
      }

      // TODO(nathan) save key association for lcd
      const size_t last_index = agents.nodes().size() - 1;
      agent_key_map_[pgmo_key] = last_index;
      if (lcd_input_) {
        lcd_input_->new_agent_nodes.push_back(agents.prefix.makeId(last_index));
      }
    }
  }

  addPlaceAgentEdges(input.timestamp_ns);
  assignBowVectors(agents);
}

void FrontendModule::assignBowVectors(const DynamicLayer& agents) {
  if (!state_->bow_queue) {
    return;
  }

  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  // TODO(nathan) take care of synchronization better
  // lcd_input_->new_agent_nodes.clear();

  // add bow messages received since last spin
  auto& bow_queue = *state_->bow_queue;
  while (!bow_queue.empty()) {
    cached_bow_messages_.push_back(bow_queue.pop());
  }

  const auto prior_size = cached_bow_messages_.size();

  auto iter = cached_bow_messages_.begin();
  while (iter != cached_bow_messages_.end()) {
    const auto& msg = *iter;
    if (static_cast<int>(msg->robot_id) != prefix.id) {
      VLOG(1) << "[Hydra Frontend] rejected bow message from robot " << msg->robot_id;
      iter = cached_bow_messages_.erase(iter);
    }

    const NodeSymbol pgmo_key(prefix.key, msg->pose_id);

    auto agent_index = agent_key_map_.find(pgmo_key);
    if (agent_index == agent_key_map_.end()) {
      ++iter;
      continue;
    }

    const auto& node = agents.getNodeByIndex(agent_index->second);
    VLOG(5) << "[Hydra Frontend] assigned bow vector of " << pgmo_key.getLabel()
            << " to dsg node " << NodeSymbol(node.id).getLabel();
    // lcd_input_->new_agent_nodes.push_back(node.id);

    auto& attrs = node.attributes<AgentNodeAttributes>();
    attrs.dbow_ids = Eigen::Map<const AgentNodeAttributes::BowIdVector>(
        msg->bow_vector.word_ids.data(), msg->bow_vector.word_ids.size());
    attrs.dbow_values = Eigen::Map<const Eigen::VectorXf>(
        msg->bow_vector.word_values.data(), msg->bow_vector.word_values.size());

    iter = cached_bow_messages_.erase(iter);
  }

  size_t num_assigned = prior_size - cached_bow_messages_.size();
  VLOG(3) << "[Hydra Frontend] assigned " << num_assigned << " bow vectors of "
          << prior_size << " original";
}

void FrontendModule::invalidateMeshEdges(const kimera_pgmo::MeshDelta& delta) {
  if (config.min_object_vertices == 0) {
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

    if (attrs.mesh_connections.size() < config.min_object_vertices) {
      objects_to_delete.push_back(id_node_pair.first);
    }
  }

  for (const auto& node : objects_to_delete) {
    dsg_->graph->removeNode(node);
  }
}

void FrontendModule::archivePlaces2d(const NodeIdSet active_places) {
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

    // find node ids that are valid, but outside active place window
    for (const auto& prev : previous_active_places_2d_) {
      if (active_places.count(prev)) {
        continue;
      }

      const auto has_prev_node = dsg_->graph->findNode(prev);
      if (!has_prev_node) {
        continue;
      }

      const auto& prev_node = *has_prev_node;
      prev_node.attributes().is_active = false;
    }
  }  // end graph update critical section
}

void FrontendModule::addPlaceObjectEdges(uint64_t timestamp_ns) {
  ScopedTimer timer("frontend/place_object_edges", timestamp_ns);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  // TODO(nathan) fix active window behavior for objects and places
  for (const auto& object_id : segmenter_->getActiveNodes()) {
    const auto node = dsg_->graph->findNode(object_id);
    if (!node) {
      continue;
    }

    places_nn_finder_->find(
        node->attributes().position, 1, false, [&](NodeId place_id, size_t, double) {
          dsg_->graph->insertParentEdge(place_id, object_id);
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
      const auto& node = layer.getNodeByIndex(*iter);

      const auto parent = node.getParent();
      if (parent) {
        const auto& parent_attrs = dsg_->graph->getNode(*parent).attributes();
        if (!parent_attrs.is_active) {
          iter = curr_active.erase(iter);
          continue;
        }
      }

      places_nn_finder_->find(
          node.attributes().position, 1, false, [&](NodeId place_id, size_t, double) {
            const auto agent_id = prefix.makeId(*iter);
            dsg_->graph->insertParentEdge(place_id, agent_id);
          });

      ++iter;
    }

    last_agent_edge_index_[prefix] = layer.numNodes();
  }
}

void FrontendModule::updatePlaceMeshMapping(const ReconstructionOutput& input) {
  const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);
  if (places.numNodes() == 0) {
    // avoid doing work by making the kdtree lookup if we don't have places
    return;
  }

  ScopedTimer timer("frontend/place_mesh_mapping", input.timestamp_ns, true, 1);
  CHECK(last_mesh_update_);
  CHECK(mesh_remapping_);

  // TODO(nathan) we can maybe put this somewhere else
  const auto num_active = last_mesh_update_->vertex_updates->size();
  std::vector<size_t> deformation_mapping(num_active,
                                          std::numeric_limits<size_t>::max());
  for (const auto& [block, indices] : *mesh_remapping_) {
    const auto block_iter = deformation_remapping_.find(block);
    if (block_iter == deformation_remapping_.end()) {
      LOG(WARNING) << "Missing block " << block.transpose() << " from graph mapping!";
      continue;
    }

    const auto& block_mapping = block_iter->second;
    for (const auto& [block_idx, mesh_idx] : indices) {
      const auto vertex_iter = block_mapping.find(block_idx);
      if (vertex_iter == block_mapping.end()) {
        continue;
      }

      const auto local_idx = last_mesh_update_->getLocalIndex(mesh_idx);
      CHECK_LT(local_idx, num_active);
      deformation_mapping[local_idx] = vertex_iter->second;
    }
  }

  PlaceMeshConnector connector(last_mesh_update_);
  const auto num_missing = connector.addConnections(places, deformation_mapping);

  VLOG_IF(1, num_missing > 0) << "[Frontend] " << num_missing
                              << " places missing basis points @ " << input.timestamp_ns
                              << " [ns]";
}

}  // namespace hydra
