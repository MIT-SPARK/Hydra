#pragma once
#include "kimera_dsg_builder/incremental_mesh_segmenter.h"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg/node_symbol.h>
#include <kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h>
#include <kimera_topology/ActiveLayer.h>
#include <kimera_topology/nearest_neighbor_utilities.h>

#include <memory>
#include <mutex>

namespace kimera {
namespace incremental {

using PlacesLayerMsg = kimera_topology::ActiveLayer;
using topology::NearestNodeFinder;

class DsgFrontend {
 public:
  using NodeIdSet = std::unordered_set<NodeId>;

  explicit DsgFrontend(const ros::NodeHandle& nh);

  virtual ~DsgFrontend();

  void spin();

 private:
  void startVisualizer();

  void runVisualizer();

  void startSegmenter();

  void runSegmenter();

  void handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg);

  void addPlaceObjectEdges(NodeIdSet* extra_objects_to_check = nullptr);

 private:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};

  DynamicSceneGraph::Ptr scene_graph_;
  std::mutex scene_graph_mutex_;

  std::unique_ptr<DynamicSceneGraphVisualizer> visualizer_;
  std::unique_ptr<ros::CallbackQueue> visualizer_queue_;
  std::unique_ptr<std::thread> visualizer_thread_;

  std::unique_ptr<MeshSegmenter> segmenter_;
  std::unique_ptr<ros::CallbackQueue> segmenter_queue_;
  std::unique_ptr<std::thread> segmenter_thread_;

  ros::Subscriber active_places_sub_;
  std::unique_ptr<NearestNodeFinder> places_nn_finder_;
};

}  // namespace incremental
}  // namespace kimera
