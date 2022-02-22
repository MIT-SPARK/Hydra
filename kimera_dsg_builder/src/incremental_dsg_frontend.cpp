#include "kimera_dsg_builder/incremental_dsg_frontend.h"
#include "kimera_dsg_builder/serialization_helpers.h"
#include "kimera_dsg_builder/timing_utilities.h"

#include <kimera_pgmo/utils/CommonFunctions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <glog/logging.h>

#include <fstream>

namespace kimera {
namespace incremental {

using lcd::LayerRegistrationConfig;
using pose_graph_tools::PoseGraph;

DsgFrontend::DsgFrontend(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg)
    : nh_(nh), dsg_(dsg), lcd_graph_(new DynamicSceneGraph()) {
  config_ = load_config<DsgFrontendConfig>(nh_);

  ros::NodeHandle pgmo_nh(nh_, "pgmo");
  CHECK(mesh_frontend_.initialize(pgmo_nh, false));
  last_mesh_timestamp_ = 0;
  last_places_timestamp_ = 0;

  if (config_.should_log) {
    frontend_graph_logger_.setOutputPath(config_.log_path + "/frontend");
    ROS_INFO("Logging frontend graph to %s", (config_.log_path + "/frontend").c_str());
    frontend_graph_logger_.setLayerName(KimeraDsgLayers::OBJECTS, "objects");
    frontend_graph_logger_.setLayerName(KimeraDsgLayers::PLACES, "places");
  }
}

void DsgFrontend::stop() {
  VLOG(2) << "[DSG Frontend] stopping frontend!";

  lcd_shutting_down_ = true;
  if (lcd_thread_) {
    VLOG(2) << "[DSG Frontend] joining lcd thread";
    lcd_thread_->join();
    lcd_thread_.reset();
    VLOG(2) << "[DSG Frontend] joined lcd thread";
  }

  lcd_visualizer_.reset();

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

void DsgFrontend::handleLatestMesh(const kimera_topology::ActiveMesh::ConstPtr& msg) {
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
  const DynamicSceneGraphLayer& agent_layer =
      dsg_->graph->getDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_)->get();

  for (const auto& node : msg->nodes) {
    if (node.key < agent_layer.numNodes()) {
      continue;
    }

    Eigen::Vector3d position;
    tf2::convert(node.pose.position, position);
    Eigen::Quaterniond rotation;
    tf2::convert(node.pose.orientation, rotation);

    // TODO(nathan) implicit assumption that pgmo ids are sequential starting at 0
    // TODO(nathan) implicit assumption that gtsam symbol and dsg node symbol are same
    NodeSymbol pgmo_key(robot_prefix_, node.key);

    bool node_valid = dsg_->graph->emplaceDynamicNode(
        agent_layer.id,
        agent_layer.prefix,
        std::chrono::nanoseconds(node.header.stamp.toNSec()),
        std::make_unique<AgentNodeAttributes>(rotation, position, pgmo_key));

    if (!node_valid) {
      VLOG(1) << "repeated timestamp " << node.header.stamp.toNSec() << "[ns] found";
      continue;
    }

    agent_key_map_[pgmo_key] = agent_layer.nodes().size() - 1;
  }

  addAgentPlaceEdges();
}

void DsgFrontend::handleDbowMsg(const kimera_vio_ros::BowQuery::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  bow_messages_.push_back(msg);
}

void DsgFrontend::start() {
  // TODO(nathan) rethink
  int robot_id = 0;
  nh_.getParam("robot_id", robot_id);
  robot_prefix_ = kimera_pgmo::robot_id_to_prefix.at(robot_id);

  dsg_->graph->createDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_);
  pose_graph_sub_ = nh_.subscribe(
      "pose_graph_incremental", 100, &DsgFrontend::handleLatestPoseGraph, this);

  startMeshFrontend();
  startPlaces();

  if (config_.enable_lcd) {
    LOG(INFO) << "[DSG Frontend] LCD enabled!";
    startLcd();
  }
  LOG(INFO) << "[DSG Frontend] started!";
}

void DsgFrontend::startMeshFrontend() {
  mesh_frontend_ros_queue_.reset(new ros::CallbackQueue());
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  ros::NodeHandle mesh_nh(nh_, config_.mesh_ns);
  mesh_nh.setCallbackQueue(mesh_frontend_ros_queue_.get());
  segmenter_.reset(new MeshSegmenter(mesh_nh, mesh_frontend_.getFullMeshVertices()));

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
  // ros::Time lookup_time;
  // lookup_time.fromNSec(last_mesh_timestamp_);
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

    kimera_topology::ActiveMesh::ConstPtr msg(nullptr);
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

      // TODO(nathan) revisit for a more formal handshake with pgmo
      ros::WallRate spin_rate(100);
      while (ros::ok() && !should_shutdown_) {
        if (mesh_frontend_.getLastFullCompressionStamp() >= msg->header.stamp) {
          break;
        }

        spin_rate.sleep();
      }
    }  // end timing scope

    mesh_frontend_.clearArchivedMeshFull(msg->archived_blocks);

    {  // timing scope
      ScopedTimer timer("frontend/object_detection", object_timestamp, true, 1, false);
      const auto& invalid_indices = mesh_frontend_.getInvalidIndices();
      {  // start dsg critical section
        std::unique_lock<std::mutex> lock(dsg_->mutex);
        for (const auto& idx : invalid_indices) {
          dsg_->graph->invalidateMeshVertex(idx);
        }

        std::vector<NodeId> objects_to_delete;
        const SceneGraphLayer& objects =
            dsg_->graph->getLayer(KimeraDsgLayers::OBJECTS).value();
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

      segmenter_->detectObjects(
          dsg_, mesh_frontend_.getActiveFullMeshVertices(), getLatestPose());
    }

    {  // start dsg critical section
      std::unique_lock<std::mutex> lock(dsg_->mutex);
      // TODO(nathan) unlike agent-place edges, we can't guarantee that all objects will
      // be connected to parents inside the dsg critical section (would need to make
      // detectObjects non-threadsafe / integrate more cleanly)
      addPlaceObjectEdges();
    }  // end dsg critical section

    if (state.timestamp_ns != last_mesh_timestamp_) {
      dsg_->updated = true;
      continue;  // places dropped a message or is ahead of us, so we don't need to
                 // update the mapping
    }

    // wait for the places thread to finish the latest message
    while (!should_shutdown_ && last_mesh_timestamp_ > last_places_timestamp_) {
      r.sleep();
    }

    {
      ScopedTimer timer(
          "frontend/place_mesh_mapping", last_places_timestamp_, true, 1, false);
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

      // find node ids that are valid, but outside active place window
      for (const auto& prev_node : previous_active_places_) {
        if (latest_places.count(prev_node)) {
          continue;
        }

        if (!dsg_->graph->hasNode(prev_node)) {
          continue;
        }

        archived_places_.insert(prev_node);
        {  // start graph update critical section
          std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
          // march archived places as inactive
          if (dsg_->graph->hasNode(prev_node)) {
            auto& attrs = dsg_->graph->getNode(prev_node)
                              .value()
                              .get()
                              .attributes<PlaceNodeAttributes>();
            attrs.is_active = false;
          }
        }
      }
    }  // end places queue critical section
    previous_active_places_ = latest_places;

    last_places_timestamp_ = curr_message->header.stamp.toNSec();

    if (config_.should_log) {
      std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
      frontend_graph_logger_.logGraph(dsg_->graph);
    }
    // dsg_->updated = true;
  }
}

void DsgFrontend::processLatestPlacesMsg(const PlacesLayerMsg::ConstPtr& msg) {
  ScopedTimer timer(
      "frontend/update_places", msg->header.stamp.toNSec(), true, 2, false);
  SceneGraphLayer temp_layer(KimeraDsgLayers::PLACES);
  std::unique_ptr<SceneGraphLayer::Edges> edges =
      temp_layer.deserializeLayer(msg->layer_contents);
  VLOG(3) << "[Places Frontend] Received " << temp_layer.numNodes() << " nodes and "
          << edges->size() << " edges from kimera_topology";

  NodeIdSet active_nodes;
  for (const auto& id_node_pair : temp_layer.nodes()) {
    active_nodes.insert(id_node_pair.first);
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    attrs.is_active = true;
  }

  const SceneGraphLayer& places_layer =
      dsg_->graph->getLayer(KimeraDsgLayers::PLACES).value();
  const SceneGraphLayer& object_layer =
      dsg_->graph->getLayer(KimeraDsgLayers::OBJECTS).value();

  NodeIdSet objects_to_check;
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    for (const auto& node_id : msg->deleted_nodes) {
      if (dsg_->graph->hasNode(node_id)) {
        const SceneGraphNode& to_check = dsg_->graph->getNode(node_id).value();
        for (const auto& child : to_check.children()) {
          // TODO(nathan) this isn't very robust
          if (object_layer.hasNode(child)) {
            objects_to_check.insert(child);
          } else {
            NodeSymbol dynamic_node(child);
            if (!deleted_agent_edge_indices_.count(dynamic_node.category())) {
              deleted_agent_edge_indices_[dynamic_node.category()] = std::set<NodeId>();
            }
            deleted_agent_edge_indices_[dynamic_node.category()].insert(child);
          }
        }
      }
      dsg_->graph->removeNode(node_id);
    }

    // TODO(nathan) figure out reindexing (for more logical node ids)
    dsg_->graph->updateFromLayer(temp_layer, std::move(edges));

    places_nn_finder_.reset(new NearestNodeFinder(places_layer, active_nodes));

    addAgentPlaceEdges();
    addPlaceObjectEdges(&objects_to_check);

    *dsg_->latest_places = active_nodes;
  }  // end graph update critical section

  VLOG(3) << "[Places Frontend] Places layer: " << places_layer.numNodes() << " nodes, "
          << places_layer.numEdges() << " edges";
}

void DsgFrontend::addPlaceObjectEdges(NodeIdSet* extra_objects_to_check) {
  ScopedTimer timer(
      "frontend/place_object_edges", last_places_timestamp_, true, 2, false);
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
  ScopedTimer timer(
      "frontend/place_agent_edges", last_places_timestamp_, true, 2, false);
  if (!places_nn_finder_) {
    return;  // haven't received places yet
  }

  for (const auto& prefix_layer_pair :
       dsg_->graph->dynamicLayersOfType(KimeraDsgLayers::AGENTS)) {
    const char prefix = prefix_layer_pair.first;
    const auto& layer = *prefix_layer_pair.second;

    if (!last_agent_edge_index_.count(prefix)) {
      last_agent_edge_index_[prefix] = 0;
    }

    for (size_t i = last_agent_edge_index_[prefix]; i < layer.numNodes(); ++i) {
      places_nn_finder_->find(
          layer.getPositionByIndex(i), 1, false, [&](NodeId place_id, size_t, double) {
            CHECK(dsg_->graph->insertEdge(place_id, NodeSymbol(prefix, i)));
          });
    }
    last_agent_edge_index_[prefix] = layer.numNodes();

    if (!deleted_agent_edge_indices_.count(prefix)) {
      continue;
    }

    for (const auto& node : deleted_agent_edge_indices_[prefix]) {
      places_nn_finder_->find(
          layer.getPosition(node), 1, false, [&](NodeId place_id, size_t, double) {
            CHECK(dsg_->graph->insertEdge(place_id, node));
          });
    }

    deleted_agent_edge_indices_.erase(prefix);
  }
}

void DsgFrontend::startLcd() {
  bow_sub_ = nh_.subscribe("bow_vectors", 100, &DsgFrontend::handleDbowMsg, this);

  auto config = load_config<lcd::DsgLcdConfig>(nh_, "lcd");
  for (auto& kv_pair : config.registration_configs) {
    kv_pair.second.registration_output_path = config_.log_path + "/lcd/";
  }
  config.agent_search_config.min_registration_score = config.agent_search_config.min_score;
  lcd_module_.reset(new lcd::DsgLcdModule(config));

  if (config_.visualize_dsg_lcd) {
    ros::NodeHandle nh(config_.lcd_visualizer_ns);
    visualizer_queue_.reset(new ros::CallbackQueue());
    nh.setCallbackQueue(visualizer_queue_.get());

    lcd_visualizer_.reset(new lcd::LcdVisualizer(nh, config.object_radius_m));
    lcd_visualizer_->setGraph(lcd_graph_);
    lcd_visualizer_->setLcdModule(lcd_module_.get());
  }

  lcd_thread_.reset(new std::thread(&DsgFrontend::runLcd, this));
}

std::optional<NodeId> DsgFrontend::getLatestAgentId() {
  if (lcd_queue_.empty()) {
    return std::nullopt;
  }

  bool has_parent;
  std::chrono::nanoseconds prev_time;
  const DynamicSceneGraphNode& node =
      lcd_graph_->getDynamicNode(lcd_queue_.top()).value();
  prev_time = node.timestamp;
  has_parent = node.hasParent();

  if (!has_parent) {
    LOG(ERROR) << "Found agent node without parent: "
               << NodeSymbol(lcd_queue_.top()).getLabel() << ". Discarding!";
    lcd_queue_.pop();
    return std::nullopt;
  }

  const std::chrono::nanoseconds curr_time(last_places_timestamp_);
  std::chrono::duration<double> diff_s = curr_time - prev_time;
  // we consider lcd_shutting_down_ here to make sure we're not waiting on popping from
  // the LCD queue while not getting new place messages
  if (!lcd_shutting_down_ && diff_s.count() < config_.lcd_agent_horizon_s) {
    return std::nullopt;
  }

  const bool forced_pop = (diff_s.count() < config_.lcd_agent_horizon_s);
  if (lcd_shutting_down_ && forced_pop) {
    LOG(ERROR) << "Forcing pop of node " << NodeSymbol(lcd_queue_.top()).getLabel()
               << " from lcd queue due to shutdown: parent? "
               << (has_parent ? "yes" : "no") << ", diff: " << diff_s.count() << " / "
               << config_.lcd_agent_horizon_s;
  }

  auto valid_node = lcd_queue_.top();
  lcd_queue_.pop();
  return valid_node;
}

void DsgFrontend::runLcd() {
  ros::WallRate r(10);
  while (ros::ok()) {
    assignBowVectors();

    {  // start critical section
      std::unique_lock<std::mutex> lock(dsg_->mutex);
      lcd_graph_->mergeGraph(*dsg_->graph);
    }  // end critical section

    if (lcd_graph_->getLayer(KimeraDsgLayers::PLACES).value().get().numNodes() == 0) {
      r.sleep();
      continue;
    }

    if (lcd_shutting_down_ && lcd_queue_.empty()) {
      break;
    }

    {  // start critical section
      std::unique_lock<std::mutex> place_lock(places_queue_mutex_);
      potential_lcd_root_nodes_.insert(potential_lcd_root_nodes_.end(),
                                       archived_places_.begin(),
                                       archived_places_.end());
      archived_places_.clear();
    }  // end critical section

    auto latest_agent_id = getLatestAgentId();
    if (!latest_agent_id) {
      r.sleep();
      continue;
    }

    const Eigen::Vector3d latest_pos = lcd_graph_->getPosition(*latest_agent_id);

    NodeIdSet to_cache;
    auto iter = potential_lcd_root_nodes_.begin();
    while (iter != potential_lcd_root_nodes_.end()) {
      const Eigen::Vector3d pos = lcd_graph_->getPosition(*iter);
      if ((latest_pos - pos).norm() < config_.descriptor_creation_horizon_m) {
        ++iter;
      } else {
        to_cache.insert(*iter);
        iter = potential_lcd_root_nodes_.erase(iter);
      }
    }

    if (!to_cache.empty()) {
      auto curr_time = ros::Time::now();
      lcd_module_->updateDescriptorCache(*lcd_graph_, to_cache, curr_time.toNSec());
    }

    uint64_t timestamp;
    timestamp =
        lcd_graph_->getDynamicNode(*latest_agent_id).value().get().timestamp.count();

    auto results = lcd_module_->detect(*lcd_graph_, *latest_agent_id, timestamp);
    if (lcd_visualizer_) {
      lcd_visualizer_->setGraphUpdated();
      lcd_visualizer_->redraw();
    }

    if (results.size() == 0) {
      if (!lcd_shutting_down_) {
        r.sleep();
      }
      continue;
    }

    {  // start lcd critical section
      // TODO(nathan) double check logic here
      std::unique_lock<std::mutex> lcd_lock(dsg_->lcd_mutex);
      for (const auto& result : results) {
        dsg_->loop_closures.push(result);
        LOG(WARNING) << "Found valid loop-closure: "
                     << NodeSymbol(result.from_node).getLabel() << " -> "
                     << NodeSymbol(result.to_node).getLabel();
      }
    }  // end lcd critical section

    if (!lcd_shutting_down_) {
      r.sleep();
    }
  }
}

void DsgFrontend::updatePlaceMeshMapping() {
  std::unique_lock<std::mutex> lock(dsg_->mutex);

  const SceneGraphLayer& places_layer =
      dsg_->graph->getLayer(KimeraDsgLayers::PLACES).value();

  const auto& mesh_mappings = mesh_frontend_.getVoxbloxMsgMapping();

  size_t num_invalid = 0;
  size_t num_processed = 0;
  size_t num_vertices_processed = 0;
  for (const auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (!attrs.is_active) {
      continue;
    }

    if (attrs.voxblox_mesh_connections.empty()) {
      continue;
    }

    ++num_processed;

    // reset connections (and mark inactive to avoid processing outside active
    // window)
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

void DsgFrontend::assignBowVectors() {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const DynamicSceneGraphLayer& agent_layer =
      dsg_->graph->getDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_)->get();

  const size_t prior_size = bow_messages_.size();
  auto iter = bow_messages_.begin();
  while (iter != bow_messages_.end()) {
    // TODO(nathan) implicit assumption that gtsam symbol and dsg node symbol are same
    const auto& msg = *iter;
    char prefix = kimera_pgmo::robot_id_to_prefix.at(msg->robot_id);
    NodeSymbol pgmo_key(prefix, msg->pose_id);
    if (agent_key_map_.count(pgmo_key)) {
      const DynamicSceneGraphNode& node =
          agent_layer.getNodeByIndex(agent_key_map_.at(pgmo_key)).value();
      lcd_queue_.push(node.id);

      AgentNodeAttributes& attrs = node.attributes<AgentNodeAttributes>();
      attrs.dbow_ids = Eigen::Map<const AgentNodeAttributes::BowIdVector>(
          msg->bow_vector.word_ids.data(), msg->bow_vector.word_ids.size());
      attrs.dbow_values = Eigen::Map<const Eigen::VectorXf>(
          msg->bow_vector.word_values.data(), msg->bow_vector.word_values.size());

      iter = bow_messages_.erase(iter);
    } else {
      ++iter;
    }
  }

  VLOG(3) << "[DSG Frontend] " << bow_messages_.size() << " of " << prior_size
          << " bow vectors unassigned";
}

void DsgFrontend::saveState(const std::string& filepath) const {
  using nlohmann::json;
  json record;
  record["mesh"]["vertices"] = *mesh_frontend_.getFullMeshVertices();
  record["mesh"]["faces"] = mesh_frontend_.getFullMeshFaces();
  std::vector<ros::Time> mesh_stamps = mesh_frontend_.getFullMeshTimes();
  std::vector<double> mesh_seconds;
  mesh_seconds.reserve(mesh_stamps.size());
  std::transform(mesh_stamps.begin(),
                 mesh_stamps.end(),
                 std::back_inserter(mesh_seconds),
                 [&](auto timestamp) { return timestamp.toSec(); });
  record["mesh"]["times"] = mesh_seconds;

  std::ofstream outfile(filepath);
  outfile << record;
}

}  // namespace incremental
}  // namespace kimera
