#include "kimera_dsg_builder/incremental_dsg_frontend.h"
#include "kimera_dsg_builder/common.h"
#include "kimera_dsg_builder/timing_utilities.h"

#include <glog/logging.h>

namespace kimera {
namespace incremental {

DsgFrontend::DsgFrontend(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg)
    : nh_(nh), dsg_(dsg) {
  ros::NodeHandle pgmo_nh(nh_, "pgmo");
  CHECK(mesh_frontend_.initialize(pgmo_nh, false));
  last_mesh_timestamp_ = 0;
  last_places_timestamp_ = 0;
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
      {  // start dsg critical region
        std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
        *dsg_->block_mesh_mapping = mesh_frontend_.getVoxbloxMsgMapping();
        // TODO(Yun) add faces also? (Currently not needed)
        dsg_->graph->setMesh(mesh_frontend_.getFullMeshVertices(),
                             std::make_shared<std::vector<pcl::Vertices>>());
      }

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

    // pop the most recently processed message (to inform mesh processing that the
    // timestamp is valid)
    {  // start places queue critical section
      std::unique_lock<std::mutex> places_lock(places_queue_mutex_);
      places_queue_.pop();
    }  // end places queue critical section

    last_places_timestamp_ = curr_message->header.stamp.toNSec();
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

    *dsg_->latest_places = active_nodes;
  }  // end graph update critical section

  addPlaceObjectEdges(&objects_to_check);
  dsg_->updated = true;

  VLOG(3) << "[Places Frontend] Places layer: " << places_layer.numNodes() << " nodes, "
          << places_layer.numEdges() << " edges";

  return;
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

}  // namespace incremental
}  // namespace kimera
