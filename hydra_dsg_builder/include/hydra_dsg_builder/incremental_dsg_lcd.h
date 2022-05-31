#pragma once
#include "hydra_dsg_builder/lcd_module_config.h"
#include "hydra_dsg_builder/dsg_lcd_detector.h"
#include "hydra_dsg_builder/lcd_visualizer.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <kimera_dsg/node_symbol.h>
#include <pose_graph_tools/BowQuery.h>

#include <memory>
#include <mutex>
#include <thread>

namespace hydra {
namespace incremental {

class DsgLcd {
 public:
  DsgLcd(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg);

  virtual ~DsgLcd();

  void start();

  void stop();

 private:
  void handleDbowMsg(const pose_graph_tools::BowQuery::ConstPtr& msg);

  void runLcd();

  void assignBowVectors();

  std::optional<NodeId> getLatestAgentId();

 private:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};

  DsgLcdModuleConfig config_;
  SharedDsgInfo::Ptr dsg_;

  std::priority_queue<NodeId, std::vector<NodeId>, std::greater<NodeId>> lcd_queue_;
  std::unique_ptr<std::thread> lcd_thread_;
  std::unique_ptr<lcd::DsgLcdDetector> lcd_detector_;
  std::unique_ptr<lcd::LcdVisualizer> lcd_visualizer_;
  std::unique_ptr<ros::CallbackQueue> visualizer_queue_;
  DynamicSceneGraph::Ptr lcd_graph_;
  // TODO(nathan) replace with struct passed in through constructor
  char robot_prefix_;

  ros::Subscriber bow_sub_;
  std::list<pose_graph_tools::BowQuery::ConstPtr> bow_messages_;
  std::list<NodeId> potential_lcd_root_nodes_;
};

}  // namespace incremental
}  // namespace hydra
