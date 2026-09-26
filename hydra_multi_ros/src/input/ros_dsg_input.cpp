#include "hydra_multi_ros/input/ros_dsg_input.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo_ros/conversion/mesh_delta.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include <format>

#include "hydra_multi_ros/common.h"

namespace hydra_multi {

using BaseInterface = rclcpp::node_interfaces::NodeBaseInterface;
using rclcpp::CallbackGroupType;
using MeshDeltaPtr = RosDsgInput::MeshDeltaPtr;

void declare_config(RosDsgInput::Config& config) {
  using namespace config;
  name("RosDsgInput::Config");
  field(config.dsg_queue_size, "dsg_queue_size");
  field(config.mesh_update_queue_size, "mesh_update_queue_size");
  field(config.rate, "rate");
  field(config.reliable_mesh, "reliable_mesh");
  field(config.subscribe_to_mesh, "subscribe_to_mesh");
  check(config.dsg_queue_size, GT, 0, "dsg_queue_size");
  check(config.mesh_update_queue_size, GT, 0, "mesh_update_queue_size");
  check(config.rate, GT, 0, "rate");
}

RosDsgInput::RosDsgInput(const Config& config,
                         UnitInterfaceState::Ptr state,
                         std::string name,
                         size_t id)
    : Input(state, name, id), config(config) {}

void RosDsgInput::init() {
  auto nh = getHydraMultiNodeHandle(name_);
  dsg_sub_ = nh.create_subscription<hydra_msgs::msg::DsgUpdate>(
      std::format("/{}/hydra/frontend/dsg", name_),
      config.dsg_queue_size,
      &RosDsgInput::dsgCallback,
      this);

  if (config.subscribe_to_mesh) {
    mesh_update_sub_ = nh.create_subscription<MeshDeltaMsg>(
        std::format("/{}/hydra/frontend/full_mesh_update", name_),
        config.mesh_update_queue_size,
        &RosDsgInput::meshUpdateCallback,
        this);

    auto group = nh.as<BaseInterface>()->create_callback_group(
        CallbackGroupType::MutuallyExclusive);
    mesh_update_client_ = nh.create_client<MeshDeltaQuery>(
        std::format("/{}/hydra/frontend/mesh_delta_query", name_),
        rclcpp::ServicesQoS(),
        group);
  }

  spin_thread_.reset(new std::thread(&RosDsgInput::spin, this));
}

void RosDsgInput::stop() {
  should_shutdown_ = true;
  if (spin_thread_) {
    spin_thread_->join();
    spin_thread_.reset();
  }
}

void RosDsgInput::dsgCallback(const DsgUpdate::ConstSharedPtr& msg) {
  const uint64_t timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  dsg_queue_.push({msg->layer_contents, timestamp_ns});
}

void RosDsgInput::meshUpdateCallback(const MeshDeltaMsg::ConstSharedPtr& msg) {
  auto delta = kimera_pgmo::conversions::from_ros(*msg);
  const auto timestamp_ns = delta->timestamp_ns;
  mesh_queue_.push({std::move(delta), timestamp_ns});
}

void RosDsgInput::spin() {
  rclcpp::Rate rate(config.rate);

  while (rclcpp::ok() && !should_shutdown_) {
    processQueues();
    rate.sleep();
  }
}

std::vector<MeshDeltaPtr> RosDsgInput::requestDeltas(
    const std::vector<uint16_t>& sequence_numbers) {
  auto request = std::make_shared<MeshDeltaQuery::Request>();
  request->sequence_numbers = sequence_numbers;

  LOG(INFO) << "Before call service for robot" << id_;
  auto nh = getHydraMultiNodeHandle(name_);
  const auto response = ianvs::call_service(*mesh_update_client_, request, 0, &nh);
  if (!response) {
    LOG(ERROR) << "Mesh Delta Request service call failed!";
    return {};
  }

  std::vector<MeshDeltaPtr> deltas;
  for (const auto& delta_msg : response->deltas) {
    deltas.push_back(kimera_pgmo::conversions::from_ros(delta_msg));
  }

  return deltas;
}

void RosDsgInput::processQueues() {
  if (dsg_queue_.empty() || (config.subscribe_to_mesh && mesh_queue_.empty())) {
    return;
  }

  std::lock_guard lock(mutex_);
  std::lock_guard<std::mutex> state_lock(state_->mutex);

  auto common_stamp = dsg_queue_.back().timestamp;
  if (config.subscribe_to_mesh) {
    const auto latest_mesh = mesh_queue_.back().timestamp;
    common_stamp = std::min(common_stamp, latest_mesh);
  }

  while (!dsg_queue_.empty() && dsg_queue_.front().timestamp <= common_stamp) {
    VLOG(2) << "Got dsg @ " << dsg_queue_.front().timestamp << " [ns] for input '"
            << name_ << "'";
    if (!state_->dsg_) {
      state_->initDsg(spark_dsg::io::binary::readGraph(dsg_queue_.front().update));
    }

    // TODO(nathan) technically can skip everything but last
    state_->dsg_operator_->incrementalAppend(dsg_queue_.front().update);
    dsg_queue_.pop();
  }

  while (!mesh_queue_.empty() && mesh_queue_.front().timestamp <= common_stamp) {
    const auto& update = mesh_queue_.front().update;
    const auto seq = update->info.sequence_number;
    if (config.reliable_mesh && seq != static_cast<uint16_t>(mesh_sequence_ + 1)) {
      // Prepare list of missing sequence numbers
      std::vector<uint16_t> missing_seqs;
      for (uint16_t s = mesh_sequence_ + 1; s < seq; ++s) {
        missing_seqs.push_back(s);
      }

      // Request and append missing deltas
      LOG(WARNING) << "Requesting " << missing_seqs.size() << " missing mesh deltas...";
      auto recovered = requestDeltas(missing_seqs);
      LOG(WARNING) << "Recieved " << recovered.size() << " mesh deltas...";
      for (const auto& delta : recovered) {
        const auto new_seq = delta->info.sequence_number;
        if (new_seq != mesh_sequence_ + 1) {
          LOG(FATAL) << "Still missing mesh delta for sequence " << mesh_sequence_ + 1
                     << ", got " << new_seq;
        }

        state_->mesh_operator_->incrementalAppend(*delta);
        ++mesh_sequence_;
      }

      if (static_cast<uint16_t>(mesh_sequence_ + 1) != seq) {
        LOG(FATAL) << "Unable to recover all missing mesh deltas before current: "
                   << mesh_sequence_ << " vs expected " << seq - 1;
      }
    }

    mesh_sequence_ = seq;
    state_->mesh_operator_->incrementalAppend(*mesh_queue_.front().update);
    mesh_queue_.pop();
  }

  state_->updated = true;
  state_->stamp = std::max(state_->stamp.load(), common_stamp);
}

}  // namespace hydra_multi
