#include "kimera_dsg_builder/incremental_dsg_frontend.h"

#include <glog/logging.h>

namespace kimera {
namespace incremental {

DsgFrontend::DsgFrontend(const ros::NodeHandle& nh) : nh_(nh) {
  scene_graph_ = std::make_shared<DynamicSceneGraph>();

  active_places_sub_ =
      nh_.subscribe("active_places", 5, &DsgFrontend::handleActivePlaces, this);

  startVisualizer();
  startSegmenter();
}

DsgFrontend::~DsgFrontend() {
  should_shutdown_ = true;
  LOG(INFO) << "joining segmentation thread";
  if (segmenter_thread_) {
    segmenter_thread_->join();
  }

  LOG(INFO) << "joining visualizer thread";
  if (visualizer_thread_) {
    visualizer_thread_->join();
  }

  LOG(INFO) << "joined threads";
}

void DsgFrontend::spin() { ros::spin(); }

void DsgFrontend::startVisualizer() {
  std::string visualizer_ns;
  nh_.param<std::string>("visualizer_ns", visualizer_ns, "/kimera_dsg_visualizer");

  visualizer_queue_.reset(new ros::CallbackQueue());

  ros::NodeHandle nh(visualizer_ns);
  nh.setCallbackQueue(visualizer_queue_.get());

  visualizer_.reset(new DynamicSceneGraphVisualizer(nh, getDefaultLayerIds()));
  visualizer_->setGraph(scene_graph_);

  visualizer_thread_.reset(new std::thread(&DsgFrontend::runVisualizer, this));
}

void DsgFrontend::runVisualizer() {
  ros::Rate r(10);
  while (ros::ok() && !should_shutdown_) {
    // process any config changes
    visualizer_queue_->callAvailable(ros::WallDuration(0));
    {  // start graph update critical section
      std::unique_lock<std::mutex> graph_lock(scene_graph_mutex_);
      visualizer_->redraw();
    }  // end graph update critical section

    r.sleep();
  }
}

void DsgFrontend::startSegmenter() {
  std::string mesh_ns;
  nh_.param<std::string>("mesh_ns", mesh_ns, "");

  segmenter_queue_.reset(new ros::CallbackQueue());

  ros::NodeHandle mesh_nh(nh_, mesh_ns);
  mesh_nh.setCallbackQueue(segmenter_queue_.get());
  segmenter_.reset(new MeshSegmenter(mesh_nh, scene_graph_));

  segmenter_thread_.reset(new std::thread(&DsgFrontend::runSegmenter, this));
}

void DsgFrontend::runSegmenter() {
  ros::Rate r(10);
  while (ros::ok() && !should_shutdown_) {
    // process everything for the mesh frontend
    // note that at the moment, this is mostly thread safe: it only adds vertices to
    // the mesh (and doesn't touch anything in the graph)
    // TODO(nathan) callOne?
    segmenter_queue_->callAvailable(ros::WallDuration(0.0));

    // this isn't threadsafe, and takes a mutex
    if (segmenter_->detectObjects(scene_graph_mutex_)) {
      addPlaceObjectEdges();
      visualizer_->setGraphUpdated();
    }

    r.sleep();
  }
}

void DsgFrontend::handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg) {
  SceneGraphLayer temp_layer(to_underlying(KimeraDsgLayers::PLACES));
  std::unique_ptr<SceneGraphLayer::Edges> edges =
      temp_layer.deserializeLayer(msg->layer_contents);
  VLOG(1) << "[Places Frontend] Received " << temp_layer.numNodes() << " nodes and "
          << edges->size() << " edges from kimera_topology";

  NodeIdSet active_nodes;
  for (const auto& id_node_pair : temp_layer.nodes()) {
    active_nodes.insert(id_node_pair.first);
  }

  const SceneGraphLayer& places_layer =
      scene_graph_->getLayer(KimeraDsgLayers::PLACES).value();

  NodeIdSet objects_to_check;
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(scene_graph_mutex_);
    for (const auto& node_id : msg->deleted_nodes) {
      if (scene_graph_->hasNode(node_id)) {
        const SceneGraphNode& to_check = scene_graph_->getNode(node_id).value();
        for (const auto& child : to_check.children()) {
          objects_to_check.insert(child);
        }
      }
      scene_graph_->removeNode(node_id);
    }

    // TODO(nathan) figure out reindexing (for more logical node ids)
    scene_graph_->updateFromLayer(temp_layer, std::move(edges));

    places_nn_finder_.reset(new NearestNodeFinder(places_layer, active_nodes));
  }  // end graph update critical section

  addPlaceObjectEdges(&objects_to_check);

  LOG(INFO) << "[Places Frontend] Places layer has " << places_layer.numNodes()
            << " nodes and " << places_layer.numEdges() << " edges";

  visualizer_->setGraphUpdated();
}

void DsgFrontend::addPlaceObjectEdges(NodeIdSet* extra_objects_to_check) {
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(scene_graph_mutex_);
    if (!places_nn_finder_) {
      return; // haven't received places yet
    }

    NodeIdSet objects_to_check = segmenter_->getObjectsToCheckForPlaces();
    if (extra_objects_to_check) {
      objects_to_check.insert(extra_objects_to_check->begin(),
                              extra_objects_to_check->end());
    }

    for (const auto& object_id : objects_to_check) {
      const Eigen::Vector3d object_position = scene_graph_->getPosition(object_id);
      places_nn_finder_->find(object_position, 1, false, [&](NodeId place_id, size_t, double) {
        scene_graph_->insertEdge(place_id, object_id);
      });
    }

    segmenter_->pruneObjectsToCheckForPlaces();
  }  // end graph update critical section
}

}  // namespace incremental
}  // namespace kimera
