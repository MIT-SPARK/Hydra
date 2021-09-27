#include "kimera_dsg_builder/incremental_dsg_frontend.h"
#include "kimera_dsg_builder/common.h"
#include "kimera_dsg_builder/timing_utilities.h"

#include <glog/logging.h>

namespace kimera {
namespace incremental {

template <typename T>
void parseParam(const ros::NodeHandle& nh, const std::string& name, T& param) {
  int value = param;
  nh.getParam(name, value);
  param = value;
}

void parseRoomClusterMode(const ros::NodeHandle& nh,
                          const std::string& name,
                          RoomFinder::Config::ClusterMode& param) {
  std::string clustering_mode = "MODULARITY";
  nh.getParam(name, clustering_mode);

  std::string to_check = clustering_mode;
  std::transform(to_check.begin(), to_check.end(), to_check.begin(), [](const auto& c) {
    return std::toupper(c);
  });

  if (to_check == "SPECTRAL") {
    param = RoomFinder::Config::ClusterMode::SPECTRAL;
  } else if (to_check == "MODULARITY") {
    param = RoomFinder::Config::ClusterMode::MODULARITY;
  } else if (to_check == "NONE") {
    param = RoomFinder::Config::ClusterMode::NONE;
  } else {
    ROS_ERROR_STREAM("Unrecognized room clustering mode: " << to_check
                                                           << ". Defaulting to NONE");
    param = RoomFinder::Config::ClusterMode::NONE;
  }
}

DsgFrontend::DsgFrontend(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg)
    : nh_(nh), dsg_(dsg) {
  ros::NodeHandle pgmo_nh(nh_, "pgmo");
  CHECK(mesh_frontend_.initialize(pgmo_nh, false));
  last_mesh_timestamp_ = 0;
  last_places_timestamp_ = 0;

  bool enable_rooms = true;
  nh_.getParam("enable_rooms", enable_rooms);
  if (enable_rooms) {
    // TODO(nathan) clean up
    RoomFinder::Config config;
    nh_.getParam("room_finder/min_dilation_m", config.min_dilation_m);
    nh_.getParam("room_finder/max_dilation_m", config.max_dilation_m);
    parseParam(nh_, "room_finder/num_steps", config.num_steps);
    parseParam(nh_, "room_finder/min_component_size", config.min_component_size);
    parseParam(nh_, "room_finder/max_kmeans_iters", config.max_kmeans_iters);
    parseParam(nh_, "room_finder/min_room_size", config.min_room_size);
    nh_.getParam("room_finder/room_vote_min_overlap", config.room_vote_min_overlap);
    nh_.getParam("room_finder/use_sparse_eigen_decomp", config.use_sparse_eigen_decomp);
    nh_.getParam("room_finder/sparse_decomp_tolerance", config.sparse_decomp_tolerance);
    nh_.getParam("room_finder/max_modularity_iters", config.max_modularity_iters);
    nh_.getParam("room_finder/modularity_gamma", config.modularity_gamma);
    parseRoomClusterMode(nh_, "room_finder/clustering_mode", config.clustering_mode);
    room_finder_.reset(new RoomFinder(config));
  }

  // purple
  std::vector<double> building_color{0.662, 0.0313, 0.7607};
  nh_.getParam("building_color", building_color);
  if (building_color.size() != 3) {
    ROS_ERROR_STREAM("supplied building color size " << building_color.size()
                                                     << " != 3");
    building_color = std::vector<double>{0.662, 0.0313, 0.7607};
  }

  building_color_ << std::clamp(static_cast<int>(255 * building_color.at(0)), 0, 255),
      std::clamp(static_cast<int>(255 * building_color.at(1)), 0, 255),
      std::clamp(static_cast<int>(255 * building_color.at(2)), 0, 255);
}

DsgFrontend::~DsgFrontend() {
  should_shutdown_ = true;
  VLOG(2) << "[DSG Frontend] joining mesh thread";
  if (mesh_frontend_thread_) {
    mesh_frontend_thread_->join();
  }
  VLOG(2) << "[DSG Frontend] joined mesh thread";

  VLOG(2) << "[DSG Frontend] joining places thread";
  if (places_thread_) {
    places_thread_->join();
  }
  VLOG(2) << "[DSG Frontend] joined places thread";
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

void DsgFrontend::start() {
  startMeshFrontend();
  startPlaces();
  LOG(INFO) << "[DSG Frontend] started!";
}

void DsgFrontend::startMeshFrontend() {
  std::string mesh_ns;
  nh_.param<std::string>("mesh_ns", mesh_ns, "");

  mesh_frontend_ros_queue_.reset(new ros::CallbackQueue());

  ros::NodeHandle mesh_nh(nh_, mesh_ns);
  mesh_nh.setCallbackQueue(mesh_frontend_ros_queue_.get());
  segmenter_.reset(new MeshSegmenter(mesh_nh, mesh_frontend_.getFullMeshVertices()));

  mesh_frontend_thread_.reset(new std::thread(&DsgFrontend::runMeshFrontend, this));

  mesh_sub_ = nh_.subscribe("voxblox_mesh", 5, &DsgFrontend::handleLatestMesh, this);
}

void DsgFrontend::runMeshFrontend() {
  ros::Rate r(10);
  bool have_new_mesh = false;
  while (ros::ok() && !should_shutdown_) {
    // identify if the places thread is waiting on a new mesh message to start running
    // again
    PlacesQueueState state = getPlacesQueueState();
    bool places_has_newer_msg =
        !state.empty && state.timestamp_ns > last_mesh_timestamp_;

    if (last_mesh_timestamp_ > last_places_timestamp_ && !places_has_newer_msg) {
      // the mesh thread is running ahead, so we sleep and spin for a little bit
      r.sleep();
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
      // we don't have any work to do, wait for a little
      r.sleep();
      continue;
    }

    // let the places thread start working on queued messages before we process the
    // latest mesh
    last_mesh_timestamp_ = msg->header.stamp.toNSec();

    // TODO(nathan) this probably doesn't do anything at the moment
    mesh_frontend_ros_queue_->callAvailable(ros::WallDuration(0.0));

    mesh_frontend_.voxbloxCallback(msg);
    have_new_mesh = true;

    // inform the mesh callback we can accept more meshes
    {  // start mesh critical region
      std::unique_lock<std::mutex> mesh_lock(mesh_frontend_mutex_);
      latest_mesh_msg_.reset();
    }

    segmenter_->detectObjects(dsg_, mesh_frontend_.getActiveFullMeshVertices());
    addPlaceObjectEdges();
    dsg_->updated = true;

    VLOG(2) << "[Object Detection] Object layer: "
            << dsg_->graph->getLayer(KimeraDsgLayers::OBJECTS).value().get().numNodes()
            << " nodes";
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

ActiveNodeSet DsgFrontend::getNodesForRoomDetection(const NodeIdSet& latest_places) {
  std::unordered_set<NodeId> active_places(latest_places.begin(), latest_places.end());

  // TODO(nathan) this is threadsafe as long as places and rooms are on the same thread
  // TODO(nathan) grab this from a set of active rooms
  const SceneGraphLayer& rooms = dsg_->graph->getLayer(KimeraDsgLayers::ROOMS).value();
  for (const auto& id_node_pair : rooms.nodes()) {
    active_places.insert(id_node_pair.second->children().begin(),
                         id_node_pair.second->children().end());
  }

  // TODO(nathan) this is threadsafe as long as places and rooms are on the same thread
  const SceneGraphLayer& places =
      dsg_->graph->getLayer(KimeraDsgLayers::PLACES).value();
  for (const auto& node_id : unlabeled_place_nodes_) {
    if (!places.hasNode(node_id)) {
      continue;
    }

    active_places.insert(node_id);
  }

  return active_places;
}

void DsgFrontend::storeUnlabeledPlaces(const ActiveNodeSet active_nodes) {
  // TODO(nathan) this is threadsafe as long as places and rooms are on the same thread
  const SceneGraphLayer& places =
      dsg_->graph->getLayer(KimeraDsgLayers::PLACES).value();

  unlabeled_place_nodes_.clear();
  for (const auto& node_id : active_nodes) {
    if (!places.hasNode(node_id)) {
      continue;
    }

    if (places.getNode(node_id)->get().hasParent()) {
      continue;
    }

    unlabeled_place_nodes_.insert(node_id);
  }
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

    NodeIdSet latest_places = processLatestPlacesMsg(curr_message);

    // pop the most recently processed message (to inform mesh processing that the
    // timestamp is valid)
    {  // start places queue critical section
      std::unique_lock<std::mutex> places_lock(places_queue_mutex_);
      places_queue_.pop();
    }  // end places queue critical section

    if (room_finder_) {
      ScopedTimer timer("frontend/room_detection", true, 1, false);
      ActiveNodeSet active_place_nodes = getNodesForRoomDetection(latest_places);
      room_finder_->findRooms(*dsg_, active_place_nodes);
      storeUnlabeledPlaces(active_place_nodes);
    }

    updateBuildingNode();

    last_places_timestamp_ = curr_message->header.stamp.toNSec();
  }
}

DsgFrontend::NodeIdSet DsgFrontend::processLatestPlacesMsg(
    const PlacesLayerMsg::ConstPtr& msg) {
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

  NodeIdSet rooms_to_check;
  NodeIdSet objects_to_check;
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    for (const auto& node_id : msg->deleted_nodes) {
      if (dsg_->graph->hasNode(node_id)) {
        const SceneGraphNode& to_check = dsg_->graph->getNode(node_id).value();
        for (const auto& child : to_check.children()) {
          objects_to_check.insert(child);
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
  }  // end graph update critical section

  addPlaceObjectEdges(&objects_to_check);
  dsg_->updated = true;

  VLOG(3) << "[Places Frontend] Places layer: " << places_layer.numNodes() << " nodes, "
          << places_layer.numEdges() << " edges";

  return active_nodes;
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

    // reset connections (and mark inactive to avoid processing outside active window)
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

  // TODO(nathan) prune removed blocks? requires more of a handshake with gvd integrator

  if (num_invalid) {
    VLOG(2) << "[DSG Frontend] Place-Mesh Update: " << num_invalid
            << " invalid connections";
  }
}

void DsgFrontend::updateBuildingNode() {
  const NodeSymbol building_node_id('B', 0);
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const SceneGraphLayer& rooms_layer =
      dsg_->graph->getLayer(KimeraDsgLayers::ROOMS).value();

  if (!rooms_layer.numNodes()) {
    if (dsg_->graph->hasNode(building_node_id)) {
      dsg_->graph->removeNode(building_node_id);
    }

    return;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& id_node_pair : rooms_layer.nodes()) {
    centroid += id_node_pair.second->attributes().position;
  }
  centroid /= rooms_layer.numNodes();

  if (!dsg_->graph->hasNode(building_node_id)) {
    SemanticNodeAttributes::Ptr attrs(new SemanticNodeAttributes());
    attrs->position = centroid;
    attrs->color = building_color_;
    attrs->semantic_label = kBuildingSemanticLabel;
    attrs->name = building_node_id.getLabel();
    dsg_->graph->emplaceNode(
        KimeraDsgLayers::BUILDINGS, building_node_id, std::move(attrs));
  } else {
    dsg_->graph->getNode(building_node_id)->get().attributes().position = centroid;
  }

  for (const auto& id_node_pair : rooms_layer.nodes()) {
    dsg_->graph->insertEdge(building_node_id, id_node_pair.first);
  }
}

}  // namespace incremental
}  // namespace kimera
