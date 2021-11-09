#include "kimera_dsg_builder/incremental_dsg_frontend.h"
#include "kimera_dsg_builder/common.h"
#include "kimera_dsg_builder/timing_utilities.h"

#include <kimera_pgmo/utils/CommonFunctions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <glog/logging.h>

namespace kimera {
namespace incremental {

using lcd::LayerRegistrationConfig;
using pose_graph_tools::PoseGraph;

DsgFrontend::DsgFrontend(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg)
    : nh_(nh), dsg_(dsg) {
  ros::NodeHandle pgmo_nh(nh_, "pgmo");
  CHECK(mesh_frontend_.initialize(pgmo_nh, false));
  last_mesh_timestamp_ = 0;
  last_places_timestamp_ = 0;

  nh_.getParam("dsg_log_output", log_);
  if (log_ && nh_.getParam("dsg_output_path", log_path_)) {
    frontend_graph_logger_.setOutputPath(log_path_ + "/frontend");
    ROS_INFO("Logging frontend graph to %s", (log_path_ + "/frontend").c_str());
    frontend_graph_logger_.setLayerName(KimeraDsgLayers::OBJECTS, "objects");
    frontend_graph_logger_.setLayerName(KimeraDsgLayers::PLACES, "places");
  } else {
    ROS_WARN("DSG Frontend logging is disabled. ");
  }
}

DsgFrontend::~DsgFrontend() {
  should_shutdown_ = true;
  if (mesh_frontend_thread_) {
    VLOG(2) << "[DSG Frontend] joining mesh thread";
    mesh_frontend_thread_->join();
    VLOG(2) << "[DSG Frontend] joined mesh thread";
  }

  if (places_thread_) {
    VLOG(2) << "[DSG Frontend] joining places thread";
    places_thread_->join();
    VLOG(2) << "[DSG Frontend] joined places thread";
  }

  if (lcd_thread_) {
    VLOG(2) << "[DSG Frontend] joining lcd thread";
    lcd_thread_->join();
    VLOG(2) << "[DSG Frontend] joined lcd thread";
  }
}

void DsgFrontend::handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg) {
  std::unique_lock<std::mutex> queue_lock(places_queue_mutex_);
  places_queue_.push(msg);
}

void DsgFrontend::handleLatestMesh(const voxblox_msgs::Mesh::ConstPtr& msg) {
  {  // start mesh frontend critical section
    std::unique_lock<std::mutex> mesh_lock(mesh_frontend_mutex_);
    if (!latest_mesh_msg_) {
      latest_mesh_msg_ = msg;
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
      LOG(ERROR) << "repeated timestamp " << node.header.stamp.toNSec() << "[ns] found";
      continue;
    }

    agent_key_map_[pgmo_key] = node.key;
  }
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
      "pose_graph_incremental", 20, &DsgFrontend::handleLatestPoseGraph, this);

  startMeshFrontend();
  startPlaces();

  bool enable_lcd;
  nh_.param<bool>("enable_lcd", enable_lcd, false);
  if (enable_lcd) {
    LOG(INFO) << "[DSG Frontend] LCD enabled!";
    startLcd();
  }
  LOG(INFO) << "[DSG Frontend] started!";
}

void DsgFrontend::startMeshFrontend() {
  std::string mesh_ns;
  nh_.param<std::string>("mesh_ns", mesh_ns, "");

  mesh_frontend_ros_queue_.reset(new ros::CallbackQueue());

  ros::NodeHandle mesh_nh(nh_, mesh_ns);
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

void DsgFrontend::runMeshFrontend() {
  ros::Rate r(10);
  bool have_new_mesh = false;
  while (ros::ok() && !should_shutdown_) {
    // identify if the places thread is waiting on a new mesh message
    PlacesQueueState state = getPlacesQueueState();
    bool newer_place_msg = !state.empty && state.timestamp_ns > last_mesh_timestamp_;

    if (last_mesh_timestamp_ > last_places_timestamp_ && !newer_place_msg) {
      r.sleep();  // the mesh thread is running ahead, spin for a little bit
      continue;
    }

    if (have_new_mesh && last_mesh_timestamp_ == last_places_timestamp_) {
      ScopedTimer timer("frontend/place_mesh_mapping", true, 1, false);
      updatePlaceMeshMapping();
      dsg_->updated = true;
      have_new_mesh = false;
    }

    voxblox_msgs::Mesh::ConstPtr msg;
    {  // start mesh critical region
      std::unique_lock<std::mutex> mesh_lock(mesh_frontend_mutex_);
      msg = latest_mesh_msg_;
    }  // end mesh critical region

    if (!msg) {
      r.sleep();  // we don't have any work to do, wait for a little
      continue;
    }

    // let the places thread start working on queued messages
    last_mesh_timestamp_ = msg->header.stamp.toNSec();

    mesh_frontend_ros_queue_->callAvailable(ros::WallDuration(0.0));
    mesh_frontend_.voxbloxCallback(msg);

    // TODO(nathan) revisit for a more formal handshake with pgmo
    ros::WallRate spin_rate(100);
    while (ros::ok() && !should_shutdown_) {
      if (mesh_frontend_.getLastFullCompressionStamp() >= msg->header.stamp) {
        break;
      }

      spin_rate.sleep();
    }

    // inform the mesh callback we can accept more meshes
    {  // start mesh critical region
      std::unique_lock<std::mutex> mesh_lock(mesh_frontend_mutex_);
      latest_mesh_msg_.reset();
    }  // end mesh critical region

    {  // timing scope
      ScopedTimer timer("frontend/object_detection", true, 1, false);
      segmenter_->detectObjects(dsg_, mesh_frontend_.getActiveFullMeshVertices());
    }
    addPlaceObjectEdges();
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
  ros::Rate r(10);
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
    addAgentPlaceEdges();

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
      }
    }  // end places queue critical section
    previous_active_places_ = latest_places;

    last_places_timestamp_ = curr_message->header.stamp.toNSec();

    if (log_) {
      std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
      frontend_graph_logger_.logGraph(dsg_->graph);
    }
  }
}

void DsgFrontend::processLatestPlacesMsg(const PlacesLayerMsg::ConstPtr& msg) {
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

  NodeIdSet rooms_to_check;
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

        std::optional<NodeId> parent = to_check.getParent();
        if (parent) {
          rooms_to_check.insert(*parent);
        }
      }
      dsg_->graph->removeNode(node_id);
    }

    // TODO(nathan) figure out reindexing (for more logical node ids)
    dsg_->graph->updateFromLayer(temp_layer, std::move(edges));

    places_nn_finder_.reset(new NearestNodeFinder(places_layer, active_nodes));

    for (const auto& node_id : rooms_to_check) {
      if (!dsg_->graph->getNode(node_id).value().get().hasChildren()) {
        dsg_->graph->removeNode(node_id);
      }
    }

    // TODO(nathan) use active nodes or copy color before update
    for (const auto& id_node_pair : places_layer.nodes()) {
      const auto& node = *id_node_pair.second;
      if (node.hasParent()) {
        node.attributes<SemanticNodeAttributes>().color =
            dsg_->graph->getNode(*node.getParent())
                .value()
                .get()
                .attributes<SemanticNodeAttributes>()
                .color;
      } else {
        node.attributes<SemanticNodeAttributes>().color = NodeColor::Zero();
      }
    }

    *dsg_->latest_places = active_nodes;
  }  // end graph update critical section

  addPlaceObjectEdges(&objects_to_check);
  dsg_->updated = true;

  VLOG(3) << "[Places Frontend] Places layer: " << places_layer.numNodes() << " nodes, "
          << places_layer.numEdges() << " edges";
}

void DsgFrontend::addPlaceObjectEdges(NodeIdSet* extra_objects_to_check) {
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    if (!places_nn_finder_) {
      return;  // haven't received places yet
    }

    NodeIdSet objects_to_check = segmenter_->getObjectsToCheckForPlaces();
    if (extra_objects_to_check) {
      objects_to_check.insert(extra_objects_to_check->begin(),
                              extra_objects_to_check->end());
    }

    for (const auto& object_id : objects_to_check) {
      const Eigen::Vector3d object_position = dsg_->graph->getPosition(object_id);
      places_nn_finder_->find(
          object_position, 1, false, [&](NodeId place_id, size_t, double) {
            dsg_->graph->insertEdge(place_id, object_id);
          });
    }

    segmenter_->pruneObjectsToCheckForPlaces(*dsg_->graph);
  }  // end graph update critical section
}

void DsgFrontend::addAgentPlaceEdges() {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
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
            dsg_->graph->insertEdge(place_id, NodeSymbol(prefix, i));
          });
    }

    if (!deleted_agent_edge_indices_.count(prefix)) {
      continue;
    }

    for (const auto& node : deleted_agent_edge_indices_[prefix]) {
      places_nn_finder_->find(
          layer.getPosition(node), 1, false, [&](NodeId place_id, size_t, double) {
            dsg_->graph->insertEdge(place_id, node);
          });
    }

    deleted_agent_edge_indices_.erase(prefix);
    last_agent_edge_index_[prefix] = layer.numNodes();
  }
}

float readMatchScore(ros::NodeHandle nh, double default_value) {
  double min_score;
  nh.param<double>("min_score", min_score, default_value);
  return static_cast<float>(min_score);
}

lcd::DescriptorScoreType readScoreType(ros::NodeHandle nh) {
  std::string type;
  nh.param<std::string>("type", type, "COSINE");
  std::transform(
      type.begin(), type.end(), type.begin(), [](char c) { return std::toupper(c); });
  if (type == "L1") {
    return lcd::DescriptorScoreType::L1;
  } else if (type == "COSINE") {
    return lcd::DescriptorScoreType::COSINE;
  } else {
    ROS_WARN_STREAM("Read invalid descriptor score type " << type << " for "
                                                          << nh.resolveName("type")
                                                          << ". Defaulting to COSINE");
    return lcd::DescriptorScoreType::COSINE;
  }
}

size_t readUnsignedParam(ros::NodeHandle nh,
                         const std::string& name,
                         size_t default_value) {
  int value;
  nh.param<int>(name, value, static_cast<int>(default_value));
  CHECK(value > 0);
  return static_cast<size_t>(value);
}

lcd::DsgLcdConfig DsgFrontend::initializeLcdStructures() {
  ros::NodeHandle lcd_nh(nh_, "lcd");
  lcd_nh.param<double>("agent_horizon_s", lcd_agent_horizon_s_, 1.5);

  double radius;
  lcd_nh.param<double>("radius_m", radius, 5.0);

  int num_classes;
  lcd_nh.param<int>("num_semantic_classes", num_classes, 20);

  double hist_min;
  lcd_nh.param<double>("min_distance_m", hist_min, 0.5);
  double hist_max;
  lcd_nh.param<double>("max_distance_m", hist_max, 2.5);
  double hist_bins;
  lcd_nh.param<double>("distance_bins", hist_bins, 30);
  lcd::HistogramConfig<double> hist_config(hist_min, hist_max, hist_bins);

  object_lcd_factory_.reset(new lcd::ObjectDescriptorFactory(radius, num_classes));
  place_lcd_factory_.reset(new lcd::PlaceDescriptorFactory(radius, hist_config));

  LayerRegistrationConfig reg_config;
  reg_config.min_correspondences =
      readUnsignedParam(lcd_nh, "min_correspondences", reg_config.min_correspondences);
  reg_config.min_inliers =
      readUnsignedParam(lcd_nh, "min_inliers", reg_config.min_inliers);

  lcd::TeaserParams teaser_params;
  object_lcd_registration_.reset(
      new lcd::ObjectRegistrationFunctor(reg_config, teaser_params));

  bool register_places;
  lcd_nh.param<bool>("register_places", register_places, false);

  if (register_places) {
    places_lcd_registration_.reset(
        new lcd::PlaceRegistrationFunctor(reg_config, teaser_params));
  }

  ros::NodeHandle agent_nh(lcd_nh, "agent");
  lcd::DsgLcdConfig config;
  config.agent_search_config.layer = KimeraDsgLayers::AGENTS;
  config.agent_search_config.min_score = readMatchScore(agent_nh, 0.8);
  config.agent_search_config.type = readScoreType(agent_nh);

  ros::NodeHandle object_nh(lcd_nh, "object");
  lcd::DescriptorMatchConfig object_config;
  object_config.layer = KimeraDsgLayers::OBJECTS;
  object_config.min_score = readMatchScore(object_nh, 0.8);
  object_config.type = readScoreType(object_nh);
  config.search_configs.push_back(object_config);

  ros::NodeHandle place_nh(lcd_nh, "place");
  lcd::DescriptorMatchConfig place_config;
  place_config.layer = KimeraDsgLayers::PLACES;
  place_config.min_score = readMatchScore(place_nh, 0.8);
  place_config.type = readScoreType(place_nh);
  config.search_configs.push_back(place_config);
  return config;
}

void DsgFrontend::startLcd() {
  bow_sub_ = nh_.subscribe("bow_vectors", 1, &DsgFrontend::handleDbowMsg, this);

  lcd::DsgLcdConfig config = initializeLcdStructures();

  std::map<LayerId, lcd::DescriptorFactoryFunc> descriptor_factories;
  descriptor_factories[KimeraDsgLayers::OBJECTS] =
      [&](const DynamicSceneGraph& graph, const DynamicSceneGraphNode& node) {
        return (*object_lcd_factory_)(graph, node);
      };
  descriptor_factories[KimeraDsgLayers::PLACES] =
      [&](const DynamicSceneGraph& graph, const DynamicSceneGraphNode& node) {
        return (*place_lcd_factory_)(graph, node);
      };

  std::map<LayerId, lcd::RegistrationFunc> registration_funcs;
  registration_funcs[KimeraDsgLayers::OBJECTS] =
      [&](SharedDsgInfo& dsg, const lcd::LayerSearchResults& match) {
        return (*object_lcd_registration_)(dsg, match);
      };
  if (places_lcd_registration_) {
    registration_funcs[KimeraDsgLayers::PLACES] =
        [&](SharedDsgInfo& dsg, const lcd::LayerSearchResults& match) {
          return (*places_lcd_registration_)(dsg, match);
        };
  }

  // unused at the moment
  std::map<LayerId, lcd::ValidationFunc> validation_funcs;

  lcd_module_.reset(new lcd::DsgLcdModule(
      config, descriptor_factories, registration_funcs, validation_funcs));

  lcd_thread_.reset(new std::thread(&DsgFrontend::runLcd, this));
}

std::optional<NodeId> DsgFrontend::getLatestAgentId() {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  if (!dsg_->graph->hasDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_)) {
    return std::nullopt;
  }

  const DynamicSceneGraphLayer& agent_layer =
      dsg_->graph->getDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_).value();

  if (agent_layer.nodes().empty()) {
    return std::nullopt;
  }

  auto last_time = agent_layer.nodes().back()->timestamp;

  auto iter = agent_layer.nodes().rbegin();
  while (iter != agent_layer.nodes().rend()) {
    std::chrono::duration<double> diff_s = last_time - (*iter)->timestamp;
    if (diff_s.count() >= lcd_agent_horizon_s_) {
      return (*iter)->id;
    }

    ++iter;
  }

  return std::nullopt;
}

void DsgFrontend::runLcd() {
  std::optional<NodeId> last_agent_id;

  ros::Rate r(10);
  while (ros::ok() && !should_shutdown_) {
    assignBowVectors();

    NodeIdSet archived_places;
    {  // start critical section
      std::unique_lock<std::mutex> place_lock(places_queue_mutex_);
      archived_places = archived_places_;
      // TODO(nathan) think more about this
      archived_places_.clear();
    }  // end critical section

    lcd_module_->updateDescriptorCache(*dsg_, archived_places);

    auto latest_agent_id = getLatestAgentId();
    if (!latest_agent_id) {
      r.sleep();
      continue;
    }

    if (last_agent_id == latest_agent_id) {
      r.sleep();
      continue;
    }

    auto result = lcd_module_->detect(*dsg_, *latest_agent_id);
    last_agent_id = latest_agent_id;
    if (!result.valid) {
      r.sleep();
      continue;
    }

    LOG(WARNING) << "Found valid loop-closure: "
                 << NodeSymbol(result.from_node).getLabel() << " -> "
                 << NodeSymbol(result.to_node).getLabel();

    {  // start lcd critical section
      std::unique_lock<std::mutex> lcd_lock(dsg_->lcd_mutex);
      dsg_->loop_closures.push(result);
    }  // end lcd critical section

    r.sleep();
  }
}

void DsgFrontend::updatePlaceMeshMapping() {
  std::unique_lock<std::mutex> lock(dsg_->mutex);

  const SceneGraphLayer& places_layer =
      dsg_->graph->getLayer(KimeraDsgLayers::PLACES).value();

  const auto& index_mapping = mesh_frontend_.getVoxbloxMsgMapping();

  size_t num_invalid = 0;
  for (const auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (!attrs.is_active) {
      continue;
    }

    if (attrs.voxblox_mesh_connections.empty()) {
      continue;
    }

    // reset connections (and mark inactive to avoid processing outside active
    // window)
    attrs.is_active = false;
    attrs.pcl_mesh_connections.clear();
    attrs.pcl_mesh_connections.reserve(attrs.voxblox_mesh_connections.size());

    for (const auto& connection : attrs.voxblox_mesh_connections) {
      voxblox::BlockIndex index =
          Eigen::Map<const voxblox::BlockIndex>(connection.block);
      if (!index_mapping.count(index)) {
        num_invalid++;
        continue;
      }

      const auto& vertex_mapping = index_mapping.at(index);
      if (!vertex_mapping.count(connection.vertex)) {
        num_invalid++;
        continue;
      }

      attrs.pcl_mesh_connections.push_back(vertex_mapping.at(connection.vertex));
    }
  }

  // TODO(nathan) prune removed blocks? requires more of a handshake with gvd
  // integrator

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

}  // namespace incremental
}  // namespace kimera
