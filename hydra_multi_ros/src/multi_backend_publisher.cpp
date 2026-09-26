#include "hydra_multi_ros/multi_backend_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra_multi/common/multi_global_info.h>
#include <kimera_pgmo_ros/visualization_functions.h>
#include <pose_graph_tools_ros/conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace hydra_multi {
namespace {

inline MultiRosBackendPublisher::Config get_config() {
  const auto world_frame = MultiGlobalInfo::instance().getWorldFrame();
  auto config = config::fromContext<MultiRosBackendPublisher::Config>("backend");
  config.dsg_sender = config.dsg_sender.with_name("backend").with_frame(world_frame);
  return config::checkValid(config);
}

}  // namespace

using kimera_pgmo::DeformationGraph;
using pose_graph_tools::PoseGraphTypeAdapter;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(MultiRosBackendPublisher::Config& config) {
  using namespace config;
  name("MultiRosBackendPublisher::Config");
  field(config.dsg_sender, "");
}

MultiRosBackendPublisher::MultiRosBackendPublisher(ianvs::NodeHandle nh)
    : config(get_config()),
      nh_(nh),
      marker_pub_(nh.create_publisher<MarkerArray>("deformation_graph_markers", 10)),
      pose_graph_pub_(nh.create_publisher<PoseGraphTypeAdapter>("pose_graph", 10)),
      dsg_sender_(new hydra::DsgSender(config.dsg_sender, nh_)) {}

void MultiRosBackendPublisher::call(uint64_t timestamp_ns,
                                    const DynamicSceneGraph& graph,
                                    const DeformationGraph& dgraph,
                                    const MultiBackendModuleStatus&) const {
  const rclcpp::Time stamp(timestamp_ns);
  dsg_sender_->sendGraph(graph, stamp);

  if (pose_graph_pub_->get_subscription_count() > 0) {
    publishPoseGraph(graph, dgraph);
  }

  if (marker_pub_->get_subscription_count() > 0) {
    publishDeformationGraphViz(dgraph, timestamp_ns);
  }
}

void MultiRosBackendPublisher::publishPoseGraph(const DynamicSceneGraph& graph,
                                                const DeformationGraph& dgraph) const {
  std::map<size_t, std::vector<size_t>> id_timestamps;
  const auto agent_layer_id = graph.getLayerKey(DsgLayers::AGENTS)->layer;
  for (const auto& [prefix, layer] : graph.layer_partition(agent_layer_id)) {
    for (const auto& [node_id, node] : layer->nodes()) {
      NodeSymbol node_symb(node->id);
      size_t robot_id = kimera_pgmo::robot_prefix_to_id.at(node_symb.category());
      if (!id_timestamps.count(robot_id)) {
        id_timestamps[robot_id] = std::vector<size_t>();
      }
      id_timestamps[robot_id].push_back(
          node->attributes<AgentNodeAttributes>().timestamp.count());
    }
  }

  auto pose_graph = *dgraph.getPoseGraph(id_timestamps, false, true);
  pose_graph_pub_->publish(pose_graph);
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA msg;
  msg.r = r;
  msg.g = g;
  msg.b = b;
  msg.a = a;
  return msg;
}

void MultiRosBackendPublisher::publishDeformationGraphViz(
    const DeformationGraph& dgraph, uint64_t timestamp_ns) const {
  const rclcpp::Time stamp(timestamp_ns);

  const auto values = dgraph.getValues();
  const auto factors = dgraph.getFactors();
  const auto N = values->size();
  if (!N || !factors->size()) {
    return;
  }

  using BoolMat = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
  BoolMat adjacency = BoolMat::Zero(N, N);

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = MultiGlobalInfo::instance().getWorldFrame();

  auto msg = std::make_unique<MarkerArray>();
  msg->markers.resize(2);

  auto& nodes = msg->markers[0];
  nodes.header = header;
  nodes.id = 0;
  nodes.ns = "pose_graph_nodes";
  nodes.action = Marker::ADD;
  nodes.type = Marker::SPHERE_LIST;
  nodes.scale.x = 0.02;
  nodes.scale.y = 0.02;
  nodes.scale.z = 0.02;

  auto& edges = msg->markers[1];
  edges.header = header;
  edges.id = 0;
  edges.ns = "pose_graph_edges";
  edges.action = Marker::ADD;
  edges.type = Marker::LINE_LIST;
  edges.scale.x = 0.02;

  std_msgs::msg::ColorRGBA mesh_mesh_color = makeColor(1.0, 0.0, 0.0, 0.8);
  std_msgs::msg::ColorRGBA pose_mesh_color = makeColor(1.0, 1.0, 0.2, 0.3);
  std_msgs::msg::ColorRGBA pose_pose_color = makeColor(0.0, 1.0, 0.0, 0.8);

  for (const auto& factor : *factors) {
    // Only interested in edges here
    if (factor->keys().size() != 2) {
      continue;
    }

    const gtsam::Symbol front = factor->front();
    const gtsam::Symbol back = factor->back();
    auto& p_front = edges.points.emplace_back();
    tf2::convert(values->at<gtsam::Pose3>(front).translation(), p_front);
    auto& p_back = edges.points.emplace_back();
    tf2::convert(values->at<gtsam::Pose3>(back).translation(), p_back);

    const bool front_is_pose_vertex =
        kimera_pgmo::robot_prefix_to_id.count(front.chr());
    const bool back_is_pose_vertex = kimera_pgmo::robot_prefix_to_id.count(back.chr());
    if (front_is_pose_vertex && back_is_pose_vertex) {
      edges.colors.push_back(pose_pose_color);
      edges.colors.push_back(pose_pose_color);
    } else if (!front_is_pose_vertex && !back_is_pose_vertex) {
      edges.colors.push_back(mesh_mesh_color);
      edges.colors.push_back(mesh_mesh_color);
    } else {
      edges.colors.push_back(pose_mesh_color);
      edges.colors.push_back(pose_mesh_color);
    }
  }

  marker_pub_->publish(std::move(msg));
}

}  // namespace hydra_multi
