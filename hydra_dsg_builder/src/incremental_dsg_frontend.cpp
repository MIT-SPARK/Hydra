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
#include "hydra_dsg_builder/incremental_mesh_segmenter.h"

#include <hydra_utils/timing_utilities.h>
#include <kimera_semantics_ros/ros_params.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <glog/logging.h>

#include <fstream>

namespace hydra {
namespace incremental {

using hydra::timing::ScopedTimer;
using pose_graph_tools::PoseGraph;

using LabelClusters = MeshSegmenter::LabelClusters;

DsgFrontend::DsgFrontend(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg)
    : nh_(nh), dsg_(dsg) {
  config_ = load_config<DsgFrontendConfig>(nh_);

  ros::NodeHandle pgmo_nh(nh_, "pgmo");
  CHECK(mesh_frontend_.initialize(pgmo_nh, false));
  last_mesh_timestamp_ = 0;
  last_places_timestamp_ = 0;

  if (config_.should_log) {
    frontend_graph_logger_.setOutputPath(config_.log_path + "/frontend");
    ROS_INFO("Logging frontend graph to %s", (config_.log_path + "/frontend").c_str());
    frontend_graph_logger_.setLayerName(DsgLayers::OBJECTS, "objects");
    frontend_graph_logger_.setLayerName(DsgLayers::PLACES, "places");
  }
}

void DsgFrontend::stop() {
  VLOG(2) << "[DSG Frontend] stopping frontend!";

  should_shutdown_ = true;
  if (mesh_frontend_thread_) {
    VLOG(2) << "[DSG Frontend] joining mesh thread";
    mesh_frontend_thread_->join();
    mesh_frontend_thread_.reset();
    VLOG(2) << "[DSG Frontend] joined mesh thread";
  }

  if (places_thread_) {
    VLOG(2) << "[DSG Frontend] joining places thread";
    places_thread_->join();
    places_thread_.reset();
    VLOG(2) << "[DSG Frontend] joined places thread";
  }
}

DsgFrontend::~DsgFrontend() {
  stop();
  segmenter_.reset();
}

void DsgFrontend::handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg) {
  std::unique_lock<std::mutex> queue_lock(places_queue_mutex_);
  places_queue_.push(msg);
}

void DsgFrontend::handleLatestMesh(const hydra_msgs::ActiveMesh::ConstPtr& msg) {
  {  // start mesh frontend critical section
    std::unique_lock<std::mutex> mesh_lock(mesh_frontend_mutex_);
    if (mesh_queue_.size() < config_.mesh_queue_size) {
      mesh_queue_.push(msg);
      return;
    }
  }  // end mesh frontend critical section

  // we warn outside to keep I/O out of the way of the mutex
  ROS_WARN_STREAM("[DSG Frontend] Dropping mesh update @ "
                  << msg->header.stamp.toSec() << " [s] (" << msg->header.stamp.toNSec()
                  << " [ns])");
}

void DsgFrontend::handleLatestPoseGraph(const PoseGraph::ConstPtr& msg) {
  if (msg->nodes.empty()) {
    return;
  }

  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const auto& agents = dsg_->graph->getLayer(DsgLayers::AGENTS, robot_prefix_);

  for (const auto& node : msg->nodes) {
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
    if (!dsg_->graph->emplaceNode(agents.id, agents.prefix, stamp, std::move(attrs))) {
      VLOG(1) << "repeated timestamp " << stamp.count() << "[ns] found";
      continue;
    }

    dsg_->agent_key_map[pgmo_key] = agents.nodes().size() - 1;
  }

  addAgentPlaceEdges();
}

void DsgFrontend::start() {
  // TODO(nathan) rethink
  int robot_id = 0;
  nh_.getParam("robot_id", robot_id);
  robot_prefix_ = kimera_pgmo::robot_id_to_prefix.at(robot_id);

  dsg_->graph->createDynamicLayer(DsgLayers::AGENTS, robot_prefix_);
  pose_graph_sub_ = nh_.subscribe(
      "pose_graph_incremental", 100, &DsgFrontend::handleLatestPoseGraph, this);

  startMeshFrontend();
  startPlaces();

  LOG(INFO) << "[DSG Frontend] started!";
}

void DsgFrontend::startMeshFrontend() {
  mesh_frontend_ros_queue_.reset(new ros::CallbackQueue());
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  ros::NodeHandle mesh_nh(nh_, config_.mesh_ns);
  mesh_nh.setCallbackQueue(mesh_frontend_ros_queue_.get());
  const auto semantic_config = kimera::getSemanticTsdfIntegratorConfigFromRosParam(nh_);
  segmenter_.reset(new MeshSegmenter(mesh_nh, semantic_config, mesh_frontend_.getFullMeshVertices()));

  // allow mesh edges to be added
  DynamicSceneGraph::MeshVertices fake_vertices;
  pcl::PolygonMesh fake_mesh;
  pcl::toPCLPointCloud2(fake_vertices, fake_mesh.cloud);
  dsg_->graph->setMeshDirectly(fake_mesh);

  mesh_frontend_thread_.reset(new std::thread(&DsgFrontend::runMeshFrontend, this));

  mesh_sub_ = nh_.subscribe("voxblox_mesh", 5, &DsgFrontend::handleLatestMesh, this);
}

std::optional<Eigen::Vector3d> DsgFrontend::getLatestPose() {
  if (!config_.prune_mesh_indices) {
    return std::nullopt;
  }

  geometry_msgs::TransformStamped msg;
  try {
    msg = tf_buffer_.lookupTransform("world", config_.sensor_frame, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    LOG_FIRST_N(WARNING, 3) << "failed to look up transform to " << config_.sensor_frame
                            << " @ " << last_mesh_timestamp_ << ": " << ex.what()
                            << " Not filtering indices.";
    return std::nullopt;
  }

  return Eigen::Vector3d(msg.transform.translation.x,
                         msg.transform.translation.y,
                         msg.transform.translation.z);
}

void DsgFrontend::runMeshFrontend() {
  ros::WallRate r(10);
  while (ros::ok() && !should_shutdown_) {
    // identify if the places thread is waiting on a new mesh message
    PlacesQueueState state = getPlacesQueueState();
    bool newer_place_msg = !state.empty && state.timestamp_ns > last_mesh_timestamp_;

    if (last_mesh_timestamp_ > last_places_timestamp_ && !newer_place_msg) {
      r.sleep();  // the mesh thread is running ahead, spin for a little bit
      continue;
    }

    hydra_msgs::ActiveMesh::ConstPtr msg(nullptr);
    {  // start mesh critical region
      std::unique_lock<std::mutex> mesh_lock(mesh_frontend_mutex_);
      if (!mesh_queue_.empty()) {
        msg = mesh_queue_.front();
        mesh_queue_.pop();
      }
    }  // end mesh critical region

    if (!msg) {
      r.sleep();
      continue;
    }

    voxblox_msgs::Mesh::ConstPtr mesh_msg(new voxblox_msgs::Mesh(msg->mesh));

    // let the places thread start working on queued messages
    last_mesh_timestamp_ = msg->header.stamp.toNSec();
    uint64_t object_timestamp = msg->header.stamp.toNSec();
    {  // start timing scope
      ScopedTimer timer(
          "frontend/mesh_compression", last_mesh_timestamp_, true, 1, false);

      mesh_frontend_ros_queue_->callAvailable(ros::WallDuration(0.0));
      mesh_frontend_.voxbloxCallback(mesh_msg);
    }  // end timing scope

    mesh_frontend_.clearArchivedMeshFull(msg->archived_blocks);
    LabelClusters object_clusters;

    {  // timing scope
      ScopedTimer timer("frontend/object_detection", object_timestamp, true, 1, false);
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

      object_clusters = segmenter_->detectObjects(
          mesh_frontend_.getActiveFullMeshVertices(), getLatestPose());
    }

    {  // start dsg critical section
      ScopedTimer timer("frontend/object_graph_update", last_places_timestamp_);
      std::unique_lock<std::mutex> lock(dsg_->mutex);
      segmenter_->updateGraph(*dsg_->graph, object_clusters, last_places_timestamp_);
      addPlaceObjectEdges();
    }  // end dsg critical section

    if (state.timestamp_ns != last_mesh_timestamp_) {
      dsg_->updated = true;
      continue;  // places dropped a message or is ahead of us, so we don't need
                 // to update the mapping
    }

    // wait for the places thread to finish the latest message
    while (!should_shutdown_ && last_mesh_timestamp_ > last_places_timestamp_) {
      r.sleep();
    }

    {
      ScopedTimer timer("frontend/place_mesh_mapping", last_places_timestamp_);
      updatePlaceMeshMapping();
    }

    dsg_->updated = true;
  }
}

void DsgFrontend::startPlaces() {
  active_places_sub_ =
      nh_.subscribe("active_places", 5, &DsgFrontend::handleActivePlaces, this);

  places_thread_.reset(new std::thread(&DsgFrontend::runPlaces, this));
}

PlacesQueueState DsgFrontend::getPlacesQueueState() {
  std::unique_lock<std::mutex> places_lock(places_queue_mutex_);
  if (places_queue_.empty()) {
    return {};
  }

  return {false, places_queue_.front()->header.stamp.toNSec()};
}

void DsgFrontend::runPlaces() {
  ros::WallRate r(10);
  while (ros::ok() && !should_shutdown_) {
    PlacesQueueState state = getPlacesQueueState();
    if (state.empty || state.timestamp_ns > last_mesh_timestamp_) {
      // we only sleep if there's no pending work
      r.sleep();
      continue;
    }

    // we only peek at the current message (to gate mesh processing)
    PlacesLayerMsg::ConstPtr curr_message;
    {  // start places queue critical section
      std::unique_lock<std::mutex> places_lock(places_queue_mutex_);
      curr_message = places_queue_.front();
    }  // end places queue critical section

    processLatestPlacesMsg(curr_message);

    // note that we don't need a mutex because this is the same thread as
    // processLatestPlacesMsg
    auto latest_places = *dsg_->latest_places;

    // pop the most recently processed message (to inform mesh processing that the
    // timestamp is valid)
    {  // start places queue critical section
      std::unique_lock<std::mutex> places_lock(places_queue_mutex_);
      places_queue_.pop();
    }  // end places queue critical section

    {  // start graph update critical section
      std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

      // find node ids that are valid, but outside active place window
      for (const auto& prev : previous_active_places_) {
        if (latest_places.count(prev)) {
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

        dsg_->archived_places.insert(prev);
      }

      dsg_->last_update_time = curr_message->header.stamp.toNSec();
    }  // end graph update critical section

    previous_active_places_ = latest_places;

    // TODO(nathan) consider moving timestamp solely to dsg structure
    last_places_timestamp_ = curr_message->header.stamp.toNSec();

    if (config_.should_log) {
      std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
      frontend_graph_logger_.logGraph(dsg_->graph);
    }
    // dsg_->updated = true;
  }
}

void DsgFrontend::processLatestPlacesMsg(const PlacesLayerMsg::ConstPtr& msg) {
  const uint64_t msg_time_ns = msg->header.stamp.toNSec();
  ScopedTimer timer("frontend/update_places", msg_time_ns, true, 2, false);
  SceneGraphLayer temp_layer(DsgLayers::PLACES);
  std::unique_ptr<SceneGraphLayer::Edges> edges =
      temp_layer.deserializeLayer(msg->layer_contents);
  VLOG(3) << "[Places Frontend] Received " << temp_layer.numNodes() << " nodes and "
          << edges->size() << " edges from hydra_topology";

  NodeIdSet active_nodes;
  for (const auto& id_node_pair : temp_layer.nodes()) {
    active_nodes.insert(id_node_pair.first);
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    attrs.is_active = true;
    attrs.last_update_time_ns = msg_time_ns;
  }

  const auto& objects = dsg_->graph->getLayer(DsgLayers::OBJECTS);
  const auto& places = dsg_->graph->getLayer(DsgLayers::PLACES);

  NodeIdSet objects_to_check;
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    for (const auto& node_id : msg->deleted_nodes) {
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

    addAgentPlaceEdges();
    addPlaceObjectEdges(&objects_to_check);

    *dsg_->latest_places = active_nodes;
  }  // end graph update critical section

  VLOG(3) << "[Places Frontend] Places layer: " << places.numNodes() << " nodes, "
          << places.numEdges() << " edges";
}

void DsgFrontend::addPlaceObjectEdges(NodeIdSet* extra_objects_to_check) {
  ScopedTimer timer("frontend/place_object_edges", last_places_timestamp_);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  NodeIdSet objects_to_check = segmenter_->getObjectsToCheckForPlaces();
  if (extra_objects_to_check) {
    objects_to_check.insert(extra_objects_to_check->begin(),
                            extra_objects_to_check->end());
  }

  for (const auto& object_id : objects_to_check) {
    if (!dsg_->graph->hasNode(object_id)) {
      continue;
    }
    const Eigen::Vector3d object_position = dsg_->graph->getPosition(object_id);
    places_nn_finder_->find(
        object_position, 1, false, [&](NodeId place_id, size_t, double) {
          dsg_->graph->insertEdge(place_id, object_id);
        });
  }

  segmenter_->pruneObjectsToCheckForPlaces(*dsg_->graph);
}

void DsgFrontend::addAgentPlaceEdges() {
  ScopedTimer timer("frontend/place_agent_edges", last_places_timestamp_);
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
  const auto& mesh_mappings = mesh_frontend_.getVoxbloxMsgToGraphMapping();

  size_t num_invalid = 0;
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

    for (const auto& connection : attrs.voxblox_mesh_connections) {
      voxblox::BlockIndex index =
          Eigen::Map<const voxblox::BlockIndex>(connection.block);

      ++num_vertices_processed;
      if (!mesh_mappings.count(index)) {
        num_invalid++;
        continue;
      }

      const auto& vertex_mapping = mesh_mappings.at(index);
      if (!vertex_mapping.count(connection.vertex)) {
        num_invalid++;
        continue;
      }

      attrs.pcl_mesh_connections.push_back(vertex_mapping.at(connection.vertex));
    }
  }

  VLOG(2) << "[DSG Frontend] Mesh-Remapping: " << num_processed << " places, "
          << num_vertices_processed << " vertices";

  if (num_invalid) {
    VLOG(2) << "[DSG Backend] Place-Mesh Update: " << num_invalid
            << " invalid connections";
  }
}

}  // namespace incremental
}  // namespace hydra
