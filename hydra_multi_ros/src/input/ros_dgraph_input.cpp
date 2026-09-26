#include "hydra_multi_ros/input/ros_dgraph_input.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <kimera_pgmo/deformation_graph.h>
#include <pose_graph_tools_ros/conversions.h>

#include <format>

#include "hydra_multi_ros/common.h"

namespace hydra_multi {
using pose_graph_tools::PoseGraphTypeAdapter;

void declare_config(RosDGraphInput::Config& config) {
  using namespace config;
  name("RosDGraphInput::Config");
  field(config.mesh_graph_queue_size, "mesh_graph_queue_size");
  field(config.pose_graph_queue_size, "pose_graph_queue_size");
  check(config.mesh_graph_queue_size, GT, 0, "mesh_graph_queue_size");
  check(config.pose_graph_queue_size, GT, 0, "pose_graph_queue_size");
}

RosDGraphInput::RosDGraphInput(const Config& config,
                               UnitInterfaceState::Ptr state,
                               std::string name,
                               size_t id)
    : Input(state, name, id), config(config) {}

void RosDGraphInput::init() {
  auto nh = getHydraMultiNodeHandle(name_);
  mesh_graph_sub_ = nh.create_subscription<PoseGraphTypeAdapter>(
      std::format("/{}/hydra/backend/mesh_graph", name_),
      config.mesh_graph_queue_size,
      &RosDGraphInput::meshGraphCallback,
      this);

  pose_graph_sub_ = nh.create_subscription<PoseGraphTypeAdapter>(
      std::format("/{}/hydra/backend/pose_graph", name_),
      config.pose_graph_queue_size,
      &RosDGraphInput::poseGraphCallback,
      this);
  spin_thread_.reset(new std::thread(&RosDGraphInput::spin, this));
}

void RosDGraphInput::stop() {
  should_shutdown_ = true;
  if (spin_thread_) {
    spin_thread_->join();
    spin_thread_.reset();
  }
}

void RosDGraphInput::meshGraphCallback(
    const pose_graph_tools::PoseGraph& mesh_graph_msg) {
  std::lock_guard<std::mutex> state_lock(state_->mutex);
  // Convert and push to mesh graph queue
  mesh_graph_queue_.push({mesh_graph_msg, mesh_graph_msg.stamp_ns});
}

void RosDGraphInput::poseGraphCallback(
    const pose_graph_tools::PoseGraph& pose_graph_msg) {
  std::lock_guard<std::mutex> state_lock(state_->mutex);
  // Convert and push to pose graph queue
  pose_graph_queue_.push({pose_graph_msg, pose_graph_msg.stamp_ns});
}

void RosDGraphInput::spin() {
  rclcpp::Rate rate(config.rate);

  while (rclcpp::ok() && !should_shutdown_) {
    processQueues();
    rate.sleep();
  }
}

void RosDGraphInput::processQueues() {
  if (pose_graph_queue_.empty() || mesh_graph_queue_.empty()) {
    return;
  }

  {
    std::lock_guard lock(mutex_);
    std::lock_guard<std::mutex> state_lock(state_->mutex);
    uint64_t latest_pg = pose_graph_queue_.back().timestamp;
    uint64_t latest_mg = mesh_graph_queue_.back().timestamp;

    uint64_t common_stamp = std::min(latest_pg, latest_mg);
    while (!pose_graph_queue_.empty() &&
           pose_graph_queue_.front().timestamp < common_stamp) {
      pose_graph_queue_.pop();
    }

    (*state_->pose_graph_operator_)(pose_graph_queue_.front().update,
                                    OperationType::MERGE);
    pose_graph_queue_.pop();

    while (!mesh_graph_queue_.empty() &&
           mesh_graph_queue_.front().timestamp < common_stamp) {
      mesh_graph_queue_.pop();
    }

    (*state_->mesh_graph_operator_)(mesh_graph_queue_.front().update,
                                    OperationType::MERGE);
    mesh_graph_queue_.pop();

    state_->updated = true;
    state_->stamp = std::max(state_->stamp.load(), common_stamp);
  }
}

}  // namespace hydra_multi
