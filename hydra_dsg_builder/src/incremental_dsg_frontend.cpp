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
#include "hydra_dsg_builder/incremental_dsg_frontend.h"

#include <hydra_utils/timing_utilities.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <glog/logging.h>

#include <fstream>

namespace hydra {
namespace incremental {

using hydra::timing::ScopedTimer;
using pose_graph_tools::PoseGraph;

using LabelClusters = MeshSegmenter::LabelClusters;

DsgFrontend::DsgFrontend(const DsgFrontendConfig& config,
                         const SharedDsgInfo::Ptr& dsg,
                         const SharedModuleState::Ptr& state,
                         int robot_id)
    : config_(config),
      dsg_(dsg),
      state_(state),
      robot_prefix_(kimera_pgmo::robot_id_to_prefix.at(robot_id)) {
  config_.pgmo_config.robot_id = robot_id;
  label_map_.reset(new kimera::SemanticLabel2Color(config_.semantic_label_file));
  queue_.reset(new FrontendInputQueue());

  // add placeholder mesh to allow mesh edges to be added
  dsg_->graph->initMesh();
  dsg_->graph->createDynamicLayer(DsgLayers::AGENTS, robot_prefix_);

  CHECK(mesh_frontend_.initialize(config_.pgmo_config));
  mesh_frontend_.addOutputCallback([&](const auto& frontend, const std_msgs::Header) {
    std::unique_lock<std::mutex> lock(state_->mesh_mutex);
    state_->deformation_graphs.emplace_back(
        new PoseGraph(frontend.getLastProcessedMeshGraph()));
  });
  segmenter_.reset(
      new MeshSegmenter(config_.object_config, mesh_frontend_.getFullMeshVertices()));

  if (config_.should_log) {
    LOG(INFO) << "[Hydra Frontend] logging to " << (config_.log_path + "/frontend");
    frontend_graph_logger_.setOutputPath(config_.log_path + "/frontend");
    frontend_graph_logger_.setLayerName(DsgLayers::OBJECTS, "objects");
    frontend_graph_logger_.setLayerName(DsgLayers::PLACES, "places");
  }

  input_callbacks_.push_back(
      [this](const FrontendInput& input) { this->updateMeshAndObjects(input); });
  input_callbacks_.push_back(
      [this](const FrontendInput& input) { this->updatePoseGraph(input); });
  input_callbacks_.push_back(
      [this](const FrontendInput& input) { this->updatePlaces(input); });
}

DsgFrontend::~DsgFrontend() { stop(); }

void DsgFrontend::start() {
  spin_thread_.reset(new std::thread(&DsgFrontend::spin, this));
  LOG(INFO) << "[Hydra Frontend] started!";
}

void DsgFrontend::stop() {
  should_shutdown_ = true;

  if (spin_thread_) {
    VLOG(2) << "[Hydra Frontend] stopping frontend!";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Hydra Frontend] stopped!";
  }
}

void DsgFrontend::save(const std::string& output_path) {
  dsg_->graph->save(output_path + "/dsg.json", false);
  dsg_->graph->save(output_path + "/dsg_with_mesh.json");

  pcl::PolygonMesh mesh;
  mesh.polygons = mesh_frontend_.getFullMeshFaces();

  const auto vertices = mesh_frontend_.getFullMeshVertices();
  if (mesh.polygons.empty() || vertices->empty()) {
    return;
  }

  pcl::toPCLPointCloud2(*vertices, mesh.cloud);

  kimera_pgmo::WriteMeshWithStampsToPly(
      output_path + "/mesh.ply", mesh, mesh_frontend_.getFullMeshTimes());
}

void DsgFrontend::spin() {
  while (!should_shutdown_) {
    bool has_data = queue_->poll();
    if (!has_data) {
      continue;
    }

    const auto msg = queue_->pop();
    VLOG(5) << "[Hydra Frontend] Popped input packet @ " << msg.timestamp_ns << " [ns]";

    std::list<std::thread> threads;
    for (const auto& callback : input_callbacks_) {
      threads.emplace_back(callback, msg);
    }

    for (auto& thread : threads) {
      thread.join();
    }

    {
      ScopedTimer timer("frontend/place_mesh_mapping", msg.timestamp_ns);
      updatePlaceMeshMapping();
    }

    dsg_->last_update_time = msg.timestamp_ns;
    dsg_->updated = true;

    if (config_.should_log) {
      // mutex not required because nothing is modifying the graph
      frontend_graph_logger_.logGraph(dsg_->graph);
    }
  }
}

void DsgFrontend::updateMeshAndObjects(const FrontendInput& input) {
  {  // start timing scope
    ScopedTimer timer("frontend/mesh_compression", input.timestamp_ns, true, 1, false);
    mesh_frontend_.voxbloxCallback(input.mesh->mesh);
    mesh_frontend_.clearArchivedMeshFull(input.mesh->archived_blocks);
  }  // end timing scope

  {  // start critical section
    ScopedTimer t1("frontend/copy_mesh", input.timestamp_ns);
    std::unique_lock<std::mutex> lock(state_->mesh_mutex);
    state_->latest_mesh.reset(new pcl::PolygonMesh());

    {  // start timing scope
      ScopedTimer t2("frontend/copy_mesh_faces", input.timestamp_ns);
      state_->latest_mesh->polygons = mesh_frontend_.getFullMeshFaces();
    }  // end timing scope

    const auto vertices = mesh_frontend_.getFullMeshVertices();
    {  // start timing scope
      ScopedTimer t3("frontend/copy_mesh_vertices", input.timestamp_ns);
      if (!vertices->empty()) {
        pcl::toPCLPointCloud2(*vertices, state_->latest_mesh->cloud);
      }
    }  // end timing scope

    ScopedTimer t4("frontend/copy_mesh_info", input.timestamp_ns);
    state_->mesh_vertex_stamps.reset(
        new std::vector<ros::Time>(mesh_frontend_.getFullMeshTimes()));

    state_->mesh_vertex_graph_indices.reset(new std::vector<int>(vertices->size(), -1));
    state_->mesh_vertex_graph_indices->resize(vertices->size());
    const auto& index_mapping = mesh_frontend_.getFullMeshToGraphMapping();
    for (size_t i = 0; i < vertices->size(); ++i) {
      const auto iter = index_mapping.find(i);
      if (iter != index_mapping.end()) {
        state_->mesh_vertex_graph_indices->push_back(iter->second);
      }
    }

    state_->have_new_mesh = true;
  }  // end critical section

  LabelClusters object_clusters;

  {  // timing scope
    ScopedTimer timer("frontend/object_detection", input.timestamp_ns, true, 1, false);
    invalidateMeshEdges();
    // add latest position somehow
    object_clusters = segmenter_->detect(*label_map_,
                                         mesh_frontend_.getActiveFullMeshVertices(),
                                         input.current_position);
  }

  {  // start dsg critical section
    ScopedTimer timer("frontend/object_graph_update", input.timestamp_ns);
    std::unique_lock<std::mutex> lock(dsg_->mutex);
    state_->archived_objects =
        segmenter_->updateGraph(*dsg_->graph, object_clusters, input.timestamp_ns);
    addPlaceObjectEdges(input.timestamp_ns);
  }  // end dsg critical section
}

void DsgFrontend::updatePlaces(const FrontendInput& input) {
  ScopedTimer timer("frontend/update_places", input.timestamp_ns, true, 2, false);
  SceneGraphLayer temp_layer(DsgLayers::PLACES);
  auto edges = temp_layer.deserializeLayer(input.places->layer_contents);
  VLOG(3) << "[Places] Received " << temp_layer.numNodes() << " nodes and "
          << edges->size() << " edges from hydra_topology";

  NodeIdSet active_nodes;
  for (const auto& id_node_pair : temp_layer.nodes()) {
    active_nodes.insert(id_node_pair.first);
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    attrs.is_active = true;
    attrs.last_update_time_ns = input.timestamp_ns;
  }

  const auto& objects = dsg_->graph->getLayer(DsgLayers::OBJECTS);
  const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);

  NodeIdSet objects_to_check;
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    for (const auto& node_id : input.places->deleted_nodes) {
      if (dsg_->graph->hasNode(node_id)) {
        const SceneGraphNode& to_check = dsg_->graph->getNode(node_id).value();
        for (const auto& child : to_check.children()) {
          // TODO(nathan) this isn't very robust
          if (objects.hasNode(child)) {
            objects_to_check.insert(child);
          } else {
            deleted_agent_edge_indices_.insert(child);
          }
        }
      }
      dsg_->graph->removeNode(node_id);
    }

    // TODO(nathan) figure out reindexing (for more logical node ids)
    dsg_->graph->updateFromLayer(temp_layer, std::move(edges));

    places_nn_finder_.reset(new NearestNodeFinder(places, active_nodes));

    addPlaceAgentEdges(input.timestamp_ns);
    addPlaceObjectEdges(input.timestamp_ns, &objects_to_check);

    state_->latest_places = active_nodes;
  }  // end graph update critical section

  VLOG(3) << "[Places] " << places.numNodes() << " nodes, " << places.numEdges()
          << " edges";

  archivePlaces(active_nodes);
  previous_active_places_ = active_nodes;
}

void DsgFrontend::updatePoseGraph(const FrontendInput& input) {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const auto& agents = dsg_->graph->getLayer(DsgLayers::AGENTS, robot_prefix_);

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
      // TODO(nathan) implicit assumption that gtsam symbol and dsg node symbol are same
      NodeSymbol pgmo_key(robot_prefix_, node.key);

      const std::chrono::nanoseconds stamp(node.header.stamp.toNSec());
      auto attrs = std::make_unique<AgentNodeAttributes>(rotation, position, pgmo_key);
      if (!dsg_->graph->emplaceNode(
              agents.id, agents.prefix, stamp, std::move(attrs))) {
        VLOG(1) << "repeated timestamp " << stamp.count() << "[ns] found";
        continue;
      }

      state_->agent_key_map[pgmo_key] = agents.nodes().size() - 1;
    }
  }

  addPlaceAgentEdges(input.timestamp_ns);
}

void DsgFrontend::invalidateMeshEdges() {
  const auto& invalid_indices = mesh_frontend_.getInvalidIndices();
  {  // start dsg critical section
    std::unique_lock<std::mutex> lock(dsg_->mutex);
    for (const auto& idx : invalid_indices) {
      dsg_->graph->invalidateMeshVertex(idx);
    }

    std::vector<NodeId> objects_to_delete;
    const auto& objects = dsg_->graph->getLayer(DsgLayers::OBJECTS);
    for (const auto& id_node_pair : objects.nodes()) {
      auto connections = dsg_->graph->getMeshConnectionIndices(id_node_pair.first);
      if (connections.size() < config_.min_object_vertices) {
        objects_to_delete.push_back(id_node_pair.first);
      }
    }

    for (const auto& node : objects_to_delete) {
      dsg_->graph->removeNode(node);
    }
  }  // end dsg critical section
}

void DsgFrontend::archivePlaces(const NodeIdSet active_places) {
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

    // find node ids that are valid, but outside active place window
    for (const auto& prev : previous_active_places_) {
      if (active_places.count(prev)) {
        continue;
      }

      if (!dsg_->graph->hasNode(prev)) {
        continue;
      }

      // mark archived places as inactive
      if (dsg_->graph->hasNode(prev)) {
        dsg_->graph->getNode(prev)
            .value()
            .get()
            .attributes<PlaceNodeAttributes>()
            .is_active = false;
      }

      state_->archived_places.insert(prev);
    }

  }  // end graph update critical section
}

void DsgFrontend::addPlaceObjectEdges(uint64_t timestamp_ns,
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

void DsgFrontend::addPlaceAgentEdges(uint64_t timestamp_ns) {
  ScopedTimer timer("frontend/place_agent_edges", timestamp_ns);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  for (const auto& pair : dsg_->graph->dynamicLayersOfType(DsgLayers::AGENTS)) {
    const LayerPrefix prefix = pair.first;
    const auto& layer = *pair.second;

    if (!last_agent_edge_index_.count(prefix)) {
      last_agent_edge_index_[prefix] = 0;
    }

    for (size_t i = last_agent_edge_index_[prefix]; i < layer.numNodes(); ++i) {
      places_nn_finder_->find(
          layer.getPositionByIndex(i), 1, false, [&](NodeId place_id, size_t, double) {
            CHECK(dsg_->graph->insertEdge(place_id, prefix.makeId(i)));
          });
    }
    last_agent_edge_index_[prefix] = layer.numNodes();
  }

  for (const auto& node : deleted_agent_edge_indices_) {
    const Eigen::Vector3d pos = dsg_->graph->getPosition(node);
    places_nn_finder_->find(pos, 1, false, [&](NodeId place_id, size_t, double) {
      CHECK(dsg_->graph->insertEdge(place_id, node));
    });
  }

  deleted_agent_edge_indices_.clear();
}

void DsgFrontend::updatePlaceMeshMapping() {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);
  const auto& graph_mappings = mesh_frontend_.getVoxbloxMsgToGraphMapping();
  const auto& mesh_mappings = mesh_frontend_.getVoxbloxMsgToMeshMapping();

  size_t num_deform_invalid = 0;
  size_t num_mesh_invalid = 0;
  size_t num_processed = 0;
  size_t num_vertices_processed = 0;
  for (const auto& id_node_pair : places.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (!attrs.is_active) {
      continue;
    }

    if (attrs.voxblox_mesh_connections.empty()) {
      continue;
    }

    ++num_processed;

    // reset connections (and mark inactive to avoid processing outside active window)
    attrs.pcl_mesh_connections.clear();
    attrs.pcl_mesh_connections.reserve(attrs.voxblox_mesh_connections.size());
    attrs.mesh_vertex_labels.clear();
    attrs.mesh_vertex_labels.reserve(attrs.voxblox_mesh_connections.size());
    attrs.deformation_connections.clear();
    attrs.deformation_connections.reserve(attrs.voxblox_mesh_connections.size());

    for (const auto& connection : attrs.voxblox_mesh_connections) {
      voxblox::BlockIndex index =
          Eigen::Map<const voxblox::BlockIndex>(connection.block);

      ++num_vertices_processed;
      if (!mesh_mappings.count(index)) {
        continue;
      }

      const auto& conn_idx = connection.vertex;

      const auto& graph_mapping = graph_mappings.at(index);
      if (!graph_mapping.count(conn_idx)) {
        num_deform_invalid++;
      } else {
        attrs.deformation_connections.push_back(graph_mapping.at(conn_idx));
      }

      const auto& vertex_mapping = mesh_mappings.at(index);
      if (!vertex_mapping.count(conn_idx)) {
        num_mesh_invalid++;
        continue;
      }

      const auto vertex_idx = vertex_mapping.at(conn_idx);
      const auto label = segmenter_->getVertexLabel(*label_map_, vertex_idx);
      if (!label) {
        continue;
      }

      attrs.pcl_mesh_connections.push_back(vertex_idx);
      attrs.mesh_vertex_labels.push_back(*label);
    }
  }

  VLOG(2) << "[Hydra Frontend] Mesh-Remapping: " << num_processed << " places, "
          << num_vertices_processed << " vertices";

  if (num_deform_invalid || num_mesh_invalid) {
    VLOG(2) << "[Hydra Frontend] Place-Mesh Update: " << num_deform_invalid
              << " invalid deformation graph connections, " << num_mesh_invalid
              << " invalid mesh connections";
  }
}

}  // namespace incremental
}  // namespace hydra
