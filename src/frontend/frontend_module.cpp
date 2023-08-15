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
#include <glog/logging.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <fstream>

#include "hydra/common/hydra_config.h"
#include "hydra/places/gvd_integrator.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;
using places::GvdIntegrator;
using places::GvdVoxel;
using pose_graph_tools::PoseGraph;
using voxblox::Layer;

/*
void ReconstructionModule::addPlacesToOutput(ReconstructionOutput& output) {
  output.places.reset(new ActiveLayerInfo());
  auto& places = *output.places;
  // non-const, as clearDeletedNodes modifies internal state
  std::unordered_set<NodeId> active_nodes = graph_extractor_->getActiveNodes();
  const auto& removed_nodes = graph_extractor_->getDeletedNodes();
  const auto& removed_edges = graph_extractor_->getDeletedEdges();

  const auto& graph = graph_extractor_->getGraph();
  std::list<NodeId> invalid_nodes;
  for (const auto& active_id : active_nodes) {
    const auto& node = graph.getNode(active_id);
    const auto& attrs = node->get().attributes<PlaceNodeAttributes>();
    if (std::isnan(attrs.distance)) {
      invalid_nodes.push_back(active_id);
      continue;
    }

    if (attrs.position.hasNaN()) {
      invalid_nodes.push_back(active_id);
      continue;
    }

    for (const auto& info : attrs.voxblox_mesh_connections) {
      if (std::isnan(info.voxel_pos[0]) || std::isnan(info.voxel_pos[1]) ||
          std::isnan(info.voxel_pos[2])) {
        invalid_nodes.push_back(active_id);
        continue;
      }
    }
  }

  places.deleted_nodes.insert(
      places.deleted_nodes.begin(), removed_nodes.begin(), removed_nodes.end());
  places.deleted_edges.insert(
      places.deleted_edges.begin(), removed_edges.begin(), removed_edges.end());

  if (!invalid_nodes.empty()) {
    LOG(ERROR) << "Nodes found with invalid attributes: "
               << displayNodeSymbolContainer(invalid_nodes);
    for (const auto& invalid_id : invalid_nodes) {
      active_nodes.erase(invalid_id);
      // TODO(nathan) think about this some more
      places.deleted_nodes.push_back(invalid_id);
    }
  }

  places.fillFromGraph(graph, active_nodes);
  graph_extractor_->clearDeleted();

  VLOG(5) << "[Hydra Reconstruction] exporting " << active_nodes.size()
          << " active and " << removed_nodes.size() << " deleted nodes";
}

*/

using LabelClusters = MeshSegmenter::LabelClusters;

template <typename Func, typename... Args>
void launchCallbacks(const std::vector<Func>& callbacks, Args... args) {
  std::list<std::thread> threads;
  for (const auto& callback : callbacks) {
    threads.emplace_back(callback, args...);
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

FrontendModule::FrontendModule(const FrontendConfig& config,
                               const RobotPrefixConfig& prefix,
                               const SharedDsgInfo::Ptr& dsg,
                               const SharedModuleState::Ptr& state,
                               const LogSetup::Ptr& logs)
    : config_(config),
      queue_(std::make_shared<FrontendInputQueue>()),
      prefix_(prefix),
      dsg_(dsg),
      state_(state) {
  kimera_pgmo::MeshFrontendConfig pgmo_config = config_.pgmo_config;
  pgmo_config.robot_id = prefix_.id;

  CHECK(dsg_ != nullptr);
  CHECK(dsg_->graph != nullptr);
  dsg_->graph->initMesh(true);
  dsg_->graph->createDynamicLayer(DsgLayers::AGENTS, prefix_.key);

  const auto mesh_resolution = pgmo_config.mesh_resolution;
  mesh_compression_.reset(new kimera_pgmo::DeltaCompression(mesh_resolution));

  if (logs && logs->valid()) {
    logs_ = logs;

    const auto frontend_dir = logs->getLogDir("frontend");
    VLOG(1) << "[Hydra Frontend] logging to " << frontend_dir;
    frontend_graph_logger_.setOutputPath(frontend_dir);
    frontend_graph_logger_.setLayerName(DsgLayers::OBJECTS, "objects");
    frontend_graph_logger_.setLayerName(DsgLayers::PLACES, "places");

    pgmo_config.log_output = true;
    pgmo_config.log_path = logs->getLogDir("frontend/pgmo");
  } else {
    pgmo_config.log_output = false;
  }

  CHECK(mesh_frontend_.initialize(pgmo_config));
  segmenter_.reset(new MeshSegmenter(config_.object_config,
                                     dsg_->graph->getMeshVertices(),
                                     dsg_->graph->getMeshLabels()));

  graph_extractor_ = config_.graph_extractor.create();
  if (!config_.graph_extractor) {
    LOG(WARNING) << "Places graph extraction disabled";
  }
}

FrontendModule::~FrontendModule() { stop(); }

void FrontendModule::initCallbacks() {
  input_callbacks_.clear();
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updateMesh, this, std::placeholders::_1));
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updateDeformationGraph, this, std::placeholders::_1));
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updatePoseGraph, this, std::placeholders::_1));
  input_callbacks_.push_back(
      std::bind(&FrontendModule::updatePlaces, this, std::placeholders::_1));

  post_mesh_callbacks_.clear();
  post_mesh_callbacks_.push_back(
      std::bind(&FrontendModule::detectObjects, this, std::placeholders::_1));
}

void FrontendModule::start() {
  initCallbacks();
  spin_thread_.reset(new std::thread(&FrontendModule::spin, this));
  LOG(INFO) << "[Hydra Frontend] started!";
}

void FrontendModule::stop() {
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
  const auto output_path = log_setup.getLogDir("frontend");
  dsg_->graph->save(output_path + "/dsg.json", false);
  dsg_->graph->save(output_path + "/dsg_with_mesh.json");

  if (!dsg_->graph->isMeshEmpty()) {
    pcl::PolygonMesh mesh = dsg_->graph->getMesh();
    kimera_pgmo::WriteMeshWithStampsToPly(
        output_path + "/mesh.ply", mesh, mesh_timestamps_);
  }

  if (graph_extractor_) {
    const auto& original_places = graph_extractor_->getGraph();
    auto places = original_places.clone();

    std::unique_ptr<DynamicSceneGraph::Edges> edges(new DynamicSceneGraph::Edges());
    for (const auto& id_edge_pair : places->edges()) {
      edges->emplace(std::piecewise_construct,
                     std::forward_as_tuple(id_edge_pair.first),
                     std::forward_as_tuple(id_edge_pair.second.source,
                                           id_edge_pair.second.target,
                                           id_edge_pair.second.info->clone()));
    }

    DynamicSceneGraph::Ptr graph(new DynamicSceneGraph());
    graph->updateFromLayer(*places, std::move(edges));
    graph->save(output_path + "/places.json", false);
  }
}

std::string FrontendModule::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config_);
  return ss.str();
}

void FrontendModule::spin() {
  bool should_shutdown = false;
  while (!should_shutdown) {
    bool has_data = queue_->poll();
    if (HydraConfig::instance().force_shutdown() || !has_data) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    spinOnce(*queue_->front());
    queue_->pop();
  }
}

bool FrontendModule::spinOnce() {
  bool has_data = queue_->poll();
  if (!has_data) {
    return false;
  }

  spinOnce(*queue_->front());
  queue_->pop();
  return true;
}

void FrontendModule::addOutputCallback(const OutputCallback& callback) {
  output_callbacks_.push_back(callback);
}

void FrontendModule::addPlaceVisualizationCallback(const PlaceVizCallback& cb) {
  places_visualization_callbacks_.push_back(cb);
}

std::vector<bool> FrontendModule::inFreespace(const PositionMatrix& positions,
                                              double freespace_distance_m) const {
  if (positions.cols() < 1) {
    return {};
  }

  std::vector<bool> flags(positions.cols(), false);
  // starting lock on tsdf update
  std::unique_lock<std::mutex> lock(gvd_mutex_);
  for (int i = 0; i < positions.cols(); ++i) {
    const auto* voxel = gvd_->getVoxelPtrByCoordinates(positions.col(i).cast<float>());
    if (!voxel) {
      continue;
    }

    flags[i] = voxel->observed && voxel->distance > freespace_distance_m;
  }

  return flags;
}

void FrontendModule::spinOnce(const ReconstructionOutput& msg) {
  VLOG(5) << "[Hydra Frontend] Popped input packet @ " << msg.timestamp_ns << " [ns]";
  ScopedTimer timer("frontend/spin", msg.timestamp_ns);

  if (dsg_->graph && backend_input_) {
    for (const auto& callback : output_callbacks_) {
      callback(*dsg_->graph, *backend_input_, msg.timestamp_ns);
    }
  }

  backend_input_.reset(new BackendInput());
  backend_input_->pose_graphs = msg.pose_graphs;
  backend_input_->timestamp_ns = msg.timestamp_ns;

  lcd_input_.reset(new LcdInput);
  lcd_input_->timestamp_ns = msg.timestamp_ns;

  // TODO(nathan) this might potentially starve the backend and LCD
  // We want to make sure that the backend and LCD catch the scene graph when:
  // - last_update_time_ns is still set to the last packet sent time
  // - some threads have done work
  // - the newest output has not been pushed yet
  // so we set the last update time before modifying anything!
  dsg_->last_update_time = msg.timestamp_ns;
  dsg_->updated = true;

  launchCallbacks(input_callbacks_, msg);
  backend_input_->mesh_update = last_mesh_update_;
  updatePlaceMeshMapping(msg);

  state_->backend_queue.push(backend_input_);
  if (state_->lcd_queue) {
    state_->lcd_queue->push(lcd_input_);
  }

  if (logs_) {
    // mutex not required because nothing is modifying the graph
    frontend_graph_logger_.logGraph(dsg_->graph);
  }
}

void FrontendModule::updateMesh(const ReconstructionOutput& input) {
  {  // start timing scope
    ScopedTimer timer("frontend/mesh_archive", input.timestamp_ns, true, 1, false);
    VLOG(5) << "[Hydra Frontend] Clearing " << input.archived_blocks.size()
            << " blocks from mesh";
    mesh_compression_->clearArchivedBlocks(input.archived_blocks);
  }  // end timing scope

  // TODO(nathan) prune archived blocks from input?

  {
    ScopedTimer timer("frontend/mesh_compression", input.timestamp_ns, true, 1, false);
    mesh_remapping_.reset(new kimera_pgmo::VoxbloxIndexMapping());
    VLOG(5) << "[Hydra Frontend] Updating mesh with" << input.mesh->numBlocks()
            << " blocks";
    auto interface = input.mesh->getMeshInterface();
    last_mesh_update_ =
        mesh_compression_->update(interface, input.timestamp_ns, mesh_remapping_.get());
  }  // end timing scope

  {  // start timing scope
    // TODO(nathan) we should probably have a mutex before modifying the mesh, but
    // nothing else uses it at the moment
    ScopedTimer timer("frontend/mesh_update", input.timestamp_ns, true, 1, false);
    last_mesh_update_->updateMesh(*dsg_->graph->getMeshVertices(),
                                  mesh_timestamps_,
                                  *dsg_->graph->getMeshFaces(),
                                  dsg_->graph->getMeshLabels().get());
    invalidateMeshEdges(*last_mesh_update_);
  }  // end timing scope

  launchCallbacks(post_mesh_callbacks_, input);
}

void FrontendModule::detectObjects(const ReconstructionOutput& input) {
  if (!last_mesh_update_) {
    LOG(ERROR) << "Cannot detect objects without valid mesh";
    return;
  }

  LabelClusters object_clusters;
  {  // timing scope
    ScopedTimer timer("frontend/object_detection", input.timestamp_ns, true, 1, false);
    object_clusters = segmenter_->detect(last_mesh_update_->getActiveIndices(),
                                         input.current_position);
  }

  {  // start dsg critical section
    ScopedTimer timer("frontend/object_graph_update", input.timestamp_ns);
    std::unique_lock<std::mutex> lock(dsg_->mutex);
    segmenter_->updateGraph(*dsg_->graph, object_clusters, input.timestamp_ns);
    addPlaceObjectEdges(input.timestamp_ns);
  }  // end dsg critical section
}

void FrontendModule::updateDeformationGraph(const ReconstructionOutput& input) {
  {  // start timing scope
    ScopedTimer timer(
        "frontend/dgraph_compresssion", input.timestamp_ns, true, 1, false);
    const auto time_ns = std::chrono::nanoseconds(input.timestamp_ns);
    double time_s = std::chrono::duration_cast<std::chrono::seconds>(time_ns).count();
    auto interface = input.mesh->getMeshInterface();
    mesh_frontend_.processMeshGraph(interface, time_s);
  }  // end timing scope

  if (backend_input_) {
    backend_input_->deformation_graph.reset(
        new PoseGraph(mesh_frontend_.getLastProcessedMeshGraph()));
  }
}

void FrontendModule::filterPlaces(const SceneGraphLayer& places,
                                  NodeIdSet& objects_to_check,
                                  NodeIdSet& active_places,
                                  const NodeIdSet& active_neighborhood) {
  // we grab connected components using the subgraph of all active places and all
  // archived places that used to be a neighbor with an active place so that we don't
  // miss disconnected components that comprised of archived nodes and formed when an
  // active node or edge is removed. Limiting the connected component search to be
  // within N hops of the subgraph, where N is the min allowable component size
  // ensures that we don't search the entire places subgraph, but still preserve
  // archived places that connect to a component of at least size N
  const auto components = graph_utilities::getConnectedComponents(
      places, config_.min_places_component_size, active_neighborhood);

  for (const auto& component : components) {
    if (component.size() >= config_.min_places_component_size) {
      continue;
    }

    for (const auto to_delete : component) {
      deletePlaceNode(to_delete, objects_to_check);
      active_places.erase(to_delete);
    }
  }
}

void FrontendModule::deletePlaceNode(NodeId node_id, NodeIdSet& objects_to_check) {
  const auto to_check = dsg_->graph->getNode(node_id);
  if (!to_check) {
    return;
  }

  for (const auto& child : to_check->get().children()) {
    if (!dsg_->graph->isDynamic(child)) {
      objects_to_check.insert(child);
    } else {
      deleted_agent_edge_indices_.insert(child);
    }
  }

  dsg_->graph->removeNode(node_id);
}

void FrontendModule::extractPlaces(const ReconstructionOutput& msg) {
  if (!msg.tsdf || !msg.mesh || !msg.occupied) {
    LOG(ERROR) << "Cannot extract places from invalid input";
    return;
  }

  if (!gvd_) {
    gvd_.reset(
        new Layer<GvdVoxel>(msg.tsdf->voxel_size(), msg.tsdf->voxels_per_side()));
    gvd_integrator_.reset(new GvdIntegrator(config_.gvd, gvd_, graph_extractor_));
  }

  {  // start critical section
    std::unique_lock<std::mutex> lock(gvd_mutex_);
    ScopedTimer timer("places/gvd", msg.timestamp_ns);
    gvd_integrator_->updateFromTsdf(
        msg.timestamp_ns, *msg.tsdf, *msg.occupied, *msg.mesh, true);
    gvd_integrator_->updateGvd(msg.timestamp_ns);
    gvd_integrator_->archiveBlocks(msg.archived_blocks);
  }  // end critical section

  for (const auto& callback : places_visualization_callbacks_) {
    callback(msg.timestamp_ns, *gvd_, graph_extractor_.get());
  }

  // TODO(nathan) do the other place stuff
}

void FrontendModule::updatePlaces(const ReconstructionOutput& input) {
  ScopedTimer timer("frontend/update_places", input.timestamp_ns, true, 2, false);
  VLOG(3) << "[Hydra Frontend] Received " << input.places->active_attributes.size()
          << " place nodes and " << input.places->edges.size()
          << " edges from hydra_places";

  NodeIdSet active_nodes;
  NodeIdSet active_neighborhood;
  for (const auto& id_attr_pair : input.places->active_attributes) {
    active_nodes.insert(id_attr_pair.first);
    active_neighborhood.insert(id_attr_pair.first);
    id_attr_pair.second->is_active = true;
    id_attr_pair.second->last_update_time_ns = input.timestamp_ns;
  }

  const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);

  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

    NodeIdSet objects_to_check;
    for (const auto& node_id : input.places->deleted_nodes) {
      const auto node = dsg_->graph->getNode(node_id);
      if (node) {
        const auto& siblings = node->get().siblings();
        active_neighborhood.insert(siblings.begin(), siblings.end());
      }
      deletePlaceNode(node_id, objects_to_check);
    }

    const auto& deleted_edges = input.places->deleted_edges;
    for (size_t i = 0; i < deleted_edges.size(); i += 2) {
      const auto n1 = deleted_edges.at(i);
      const auto n2 = deleted_edges.at(i + 1);
      active_neighborhood.insert(n1);
      active_neighborhood.insert(n2);
      dsg_->graph->removeEdge(n1, n2);
    }

    for (auto&& [node, attrs] : input.places->active_attributes) {
      dsg_->graph->addOrUpdateNode(DsgLayers::PLACES, node, attrs->clone());
    }

    for (auto& edge : input.places->edges) {
      dsg_->graph->addOrUpdateEdge(edge.source, edge.target, edge.info->clone());
    }

    if (config_.filter_places) {
      auto iter = active_neighborhood.begin();
      while (iter != active_neighborhood.end()) {
        if (places.hasNode(*iter)) {
          ++iter;
        } else {
          iter = active_neighborhood.erase(iter);
        }
      }
      filterPlaces(places, objects_to_check, active_nodes, active_neighborhood);
    }

    places_nn_finder_.reset(new NearestNodeFinder(places, active_nodes));

    addPlaceAgentEdges(input.timestamp_ns);
    addPlaceObjectEdges(input.timestamp_ns, &objects_to_check);

    state_->latest_places = active_nodes;
  }  // end graph update critical section

  archivePlaces(active_nodes);
  previous_active_places_ = active_nodes;
}

void FrontendModule::updatePoseGraph(const ReconstructionOutput& input) {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const auto& agents = dsg_->graph->getLayer(DsgLayers::AGENTS, prefix_.key);

  lcd_input_->new_agent_nodes.clear();
  for (const auto& pose_graph : input.pose_graphs) {
    if (pose_graph->nodes.empty()) {
      continue;
    }

    for (const auto& node : pose_graph->nodes) {
      if (node.key < agents.numNodes()) {
        continue;
      }

      Eigen::Vector3d position;
      tf2::convert(node.pose.position, position);
      Eigen::Quaterniond rotation;
      tf2::convert(node.pose.orientation, rotation);

      // TODO(nathan) implicit assumption that pgmo ids are sequential starting at 0
      // TODO(nathan) implicit assumption that gtsam symbol and dsg node symbol are
      // same
      NodeSymbol pgmo_key(prefix_.key, node.key);

      const std::chrono::nanoseconds stamp(node.header.stamp.toNSec());
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
      lcd_input_->new_agent_nodes.push_back(agents.prefix.makeId(last_index));
    }
  }

  addPlaceAgentEdges(input.timestamp_ns);
  if (config_.lcd_use_bow_vectors) {
    assignBowVectors(agents);
  }
}

void FrontendModule::assignBowVectors(const DynamicLayer& agents) {
  // TODO(nathan) take care of synchronization better
  // lcd_input_->new_agent_nodes.clear();

  // add bow messages received since last spin
  while (!state_->visual_lcd_queue.empty()) {
    cached_bow_messages_.push_back(state_->visual_lcd_queue.pop());
  }

  const auto prior_size = cached_bow_messages_.size();

  auto iter = cached_bow_messages_.begin();
  while (iter != cached_bow_messages_.end()) {
    const auto& msg = *iter;
    if (static_cast<int>(msg->robot_id) != prefix_.id) {
      VLOG(1) << "[Hydra Frontend] rejected bow message from robot " << msg->robot_id;
      iter = cached_bow_messages_.erase(iter);
    }

    const NodeSymbol pgmo_key(prefix_.key, msg->pose_id);

    auto agent_index = agent_key_map_.find(pgmo_key);
    if (agent_index == agent_key_map_.end()) {
      ++iter;
      continue;
    }

    const auto& node = agents.getNodeByIndex(agent_index->second)->get();
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
      lcd_input_->archived_places.insert(prev);
    }

  }  // end graph update critical section
}

void FrontendModule::addPlaceObjectEdges(uint64_t timestamp_ns,
                                         NodeIdSet* extra_objects_to_check) {
  ScopedTimer timer("frontend/place_object_edges", timestamp_ns);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  NodeIdSet objects_to_check = segmenter_->getObjectsToCheckForPlaces();
  if (extra_objects_to_check) {
    objects_to_check.insert(extra_objects_to_check->begin(),
                            extra_objects_to_check->end());
  }

  for (const auto& object_id : objects_to_check) {
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

  segmenter_->pruneObjectsToCheckForPlaces(*dsg_->graph);
}

void FrontendModule::addPlaceAgentEdges(uint64_t timestamp_ns) {
  ScopedTimer timer("frontend/place_agent_edges", timestamp_ns);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  std::set<NodeId> to_find;
  for (const auto& pair : dsg_->graph->dynamicLayersOfType(DsgLayers::AGENTS)) {
    const LayerPrefix prefix = pair.first;
    const auto& layer = *pair.second;

    if (!last_agent_edge_index_.count(prefix)) {
      last_agent_edge_index_[prefix] = 0;
    }

    for (size_t i = last_agent_edge_index_[prefix]; i < layer.numNodes(); ++i) {
      bool found = false;
      places_nn_finder_->find(
          layer.getPositionByIndex(i), 1, false, [&](NodeId place_id, size_t, double) {
            CHECK(dsg_->graph->insertEdge(place_id, prefix.makeId(i)));
            found = true;
          });
      if (!found) {
        to_find.insert(prefix.makeId(i));
      }
    }

    last_agent_edge_index_[prefix] = layer.numNodes();
  }

  for (const auto& node : deleted_agent_edge_indices_) {
    const Eigen::Vector3d pos = dsg_->graph->getPosition(node);

    bool found = false;
    places_nn_finder_->find(pos, 1, false, [&](NodeId place_id, size_t, double) {
      CHECK(dsg_->graph->insertEdge(place_id, node));
      found = true;
    });

    if (!found) {
      to_find.insert(node);
    }
  }

  deleted_agent_edge_indices_.clear();
  deleted_agent_edge_indices_ = to_find;
  VLOG(5) << "Pending agents: "
          << displayNodeSymbolContainer(deleted_agent_edge_indices_);
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

void FrontendModule::updatePlaceMeshMapping(const ReconstructionOutput& input) {
  ScopedTimer timer("frontend/place_mesh_mapping", input.timestamp_ns);
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);
  const auto& graph_mapping = mesh_frontend_.getVoxbloxMsgToGraphMapping();

  voxblox::BlockIndexList allocated_list;
  input.mesh->getAllocatedBlockIndices(allocated_list);

  voxblox::IndexSet allocated(allocated_list.begin(), allocated_list.end());
  voxblox::IndexSet archived(input.archived_blocks.begin(), input.archived_blocks.end());

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
