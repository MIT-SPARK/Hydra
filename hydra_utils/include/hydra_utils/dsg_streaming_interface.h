#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>

#include <hydra_msgs/DsgUpdate.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>

namespace hydra {

class DsgSender {
 public:
  explicit DsgSender(const ros::NodeHandle& nh);

  void sendGraph(kimera::DynamicSceneGraph& graph, const ros::Time& stamp) const;

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

class DsgReceiver {
 public:
  using LogCallback = std::function<void(const ros::Time&, size_t)>;

  explicit DsgReceiver(const ros::NodeHandle& nh);

  DsgReceiver(const ros::NodeHandle& nh, const LogCallback& cb);

  inline kimera::DynamicSceneGraph::Ptr graph() const { return graph_; }

  inline bool updated() const { return has_update_; }

  inline void clearUpdated() { has_update_ = false; }

 private:
  void handleUpdate(const hydra_msgs::DsgUpdate::ConstPtr& msg);

  void handleMesh(const mesh_msgs::TriangleMeshStamped::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Subscriber mesh_sub_;

  bool has_update_;
  kimera::DynamicSceneGraph::Ptr graph_;
  std::unique_ptr<pcl::PolygonMesh> mesh_;

  std::unique_ptr<LogCallback> log_callback_;
};

}  // namespace hydra
