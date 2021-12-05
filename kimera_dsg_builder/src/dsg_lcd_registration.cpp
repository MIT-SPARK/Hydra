#include "kimera_dsg_builder/dsg_lcd_registration.h"
#include "kimera_dsg_builder/timing_utilities.h"

#include <kimera_dsg/serialization_helpers.h>
#include <kimera_vio_ros/LcdFrameRegistration.h>
#include <ros/service.h>
#include <tf2_eigen/tf2_eigen.h>
#include <fstream>

namespace kimera {
namespace lcd {

using incremental::SharedDsgInfo;
using DsgNode = DynamicSceneGraphNode;
using nlohmann::json;

struct AgentNodePose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  gtsam::Pose3 world_T_body;
  NodeId id;
};

AgentNodePose getAgentPose(const DynamicSceneGraph& graph, NodeId root_id) {
  if (!graph.hasNode(root_id)) {
    return {};
  }

  const SceneGraphNode& root_node = graph.getNode(root_id).value();

  std::optional<NodeId> child_id = std::nullopt;
  for (const auto& root_child : root_node.children()) {
    if (graph.isDynamic(root_child)) {
      child_id = root_child;
      break;
    }
  }

  if (!child_id) {
    return {};
  }

  CHECK(child_id);

  const auto& attrs =
      graph.getNode(*child_id).value().get().attributes<AgentNodeAttributes>();
  return {
      true, gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position), *child_id};
}

AgentNodePose getQueryPose(const DynamicSceneGraph& graph, NodeId query_agent_id) {
  const auto& attrs =
      graph.getNode(query_agent_id).value().get().attributes<AgentNodeAttributes>();
  return {true,
          gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position),
          query_agent_id};
}

DsgRegistrationSolution getFullSolutionFromLayer(
    const DynamicSceneGraph& graph,
    const LayerRegistrationSolution& solution,
    NodeId query_agent_id,
    NodeId match_root) {
  if (!solution.valid || solution.inliers.empty()) {
    return {};
  }

  auto from_pose_info = getQueryPose(graph, query_agent_id);
  auto to_pose_info = getAgentPose(graph, match_root);

  if (!from_pose_info.valid || !to_pose_info.valid) {
    return {};
  }

  VLOG(3) << "=================================================";
  VLOG(3) << "world_T_from:";
  VLOG(3) << from_pose_info.world_T_body;
  VLOG(3) << "";
  VLOG(3) << "world_T_to:";
  VLOG(3) << to_pose_info.world_T_body;
  VLOG(3) << "";
  VLOG(3) << "dest_T_src:";
  VLOG(3) << solution.dest_T_src;
  VLOG(3) << "";

  gtsam::Pose3 to_T_from = to_pose_info.world_T_body.inverse() * solution.dest_T_src *
                           from_pose_info.world_T_body;
  VLOG(3) << "to_T_from:";
  VLOG(3) << to_T_from;
  VLOG(3) << "";

  return {true, from_pose_info.id, to_pose_info.id, to_T_from, -1};
}

PlaceRegistrationFunctor::PlaceRegistrationFunctor(
    const LayerRegistrationConfig& config,
    const TeaserParams& params)
    : config(config), solver(params) {}

DsgRegistrationSolution PlaceRegistrationFunctor::operator()(
    SharedDsgInfo& dsg,
    const DsgRegistrationInput& match,
    NodeId query_agent_id) {
  if (match.query_nodes.size() <= 3 || match.match_nodes.size() <= 3) {
    return {};
  }

  LayerRegistrationProblem<std::set<NodeId>> problem;
  problem.src_nodes = match.query_nodes;
  problem.dest_nodes = match.match_nodes;
  problem.src_mutex = &dsg.mutex;

  const SceneGraphLayer& layer = dsg.graph->getLayer(KimeraDsgLayers::PLACES).value();
  auto solution = registerDsgLayerPairwise(config, solver, problem, layer);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  return getFullSolutionFromLayer(
      *dsg.graph, solution, query_agent_id, match.match_root);
}

ObjectRegistrationFunctor::ObjectRegistrationFunctor(
    const LayerRegistrationConfig& config,
    const TeaserParams& params)
    : config(config), solver(params) {}

void logRegistrationProblem(const std::string& dir_path,
                            SharedDsgInfo& info,
                            const LayerRegistrationSolution& solution,
                            const DsgRegistrationInput& match,
                            NodeId query_agent_id) {
  std::unique_lock<std::mutex> lock(info.mutex);

  json record;
  record["query_id"] = query_agent_id;
  record["query_node"] =
      info.graph->getNode(query_agent_id).value().get().attributes().toJson();
  record["query_set"] = match.query_nodes;
  record["match_set"] = match.match_nodes;
  record["nodes"] = {};

  auto match_pose_info = getAgentPose(*info.graph, match.match_root);
  record["match_id"] = match_pose_info.id;
  record["world_q_match"] = match_pose_info.world_T_body.rotation().quaternion();
  record["world_t_match"] = match_pose_info.world_T_body.translation();
  record["match_valid"] = match_pose_info.valid;

  for (const auto& node : match.query_nodes) {
    record["nodes"][std::to_string(node)] =
        info.graph->getNode(node).value().get().attributes().toJson();
  }

  record["solution_valid"] = solution.valid;
  record["dest_q_src"] = solution.dest_T_src.rotation().quaternion();
  record["dest_t_src"] = solution.dest_T_src.translation();
  record["solution_inliers"] = solution.inliers;

  static size_t registration_index = 0;
  std::stringstream ss;
  ss << dir_path << "/object_registration_" << registration_index << ".json";
  ++registration_index;

  std::ofstream outfile(ss.str());
  outfile << record;
}

DsgRegistrationSolution ObjectRegistrationFunctor::operator()(
    SharedDsgInfo& dsg,
    const DsgRegistrationInput& match,
    NodeId query_agent_id) {
  uint64_t timestamp;
  {  // start critical section
    std::unique_lock<std::mutex> lock(dsg.mutex);
    timestamp =
        dsg.graph->getDynamicNode(query_agent_id).value().get().timestamp.count();
  }  // end critical section

  ScopedTimer timer("lcd/register_object", timestamp, true, 2, false);
  if (match.query_nodes.size() <= 3 || match.match_nodes.size() <= 3) {
    return {};
  }

  LayerRegistrationProblem<std::set<NodeId>> problem;
  problem.src_nodes = match.query_nodes;
  problem.dest_nodes = match.match_nodes;
  problem.src_mutex = &dsg.mutex;

  const SceneGraphLayer& layer = dsg.graph->getLayer(KimeraDsgLayers::OBJECTS).value();
  auto solution = registerDsgLayerSemantic(config, solver, problem, layer);

  if (config.log_registration_problem) {
    logRegistrationProblem(
        config.registration_output_path, dsg, solution, match, query_agent_id);
  }

  std::unique_lock<std::mutex> lock(dsg.mutex);
  return getFullSolutionFromLayer(
      *dsg.graph, solution, query_agent_id, match.match_root);
}

inline size_t getFrameIdFromNode(const DynamicSceneGraph& graph, NodeId node_id) {
  const auto& attrs =
      graph.getNode(node_id).value().get().attributes<AgentNodeAttributes>();
  return NodeSymbol(attrs.external_key).categoryId();
}

inline size_t getTimestampFromNode(const DynamicSceneGraph& graph, NodeId node_id) {
  const auto& attrs =
      graph.getNode(node_id).value().get().attributes<AgentNodeAttributes>();
  return NodeSymbol(attrs.external_key).categoryId();
}

DsgRegistrationSolution registerAgentMatch(incremental::SharedDsgInfo& dsg,
                                           const DsgRegistrationInput& match,
                                           NodeId) {
  if (match.query_nodes.empty() || match.match_nodes.empty()) {
    return {};
  }

  if (!ros::service::exists("frame_registration", true)) {
    LOG(ERROR) << "Frame registration service missing!";
    return {};
  }

  // at the agent level, match sets are one node each
  const NodeId query_id = *match.query_nodes.begin();
  const NodeId match_id = *match.match_nodes.begin();

  uint64_t timestamp;
  kimera_vio_ros::LcdFrameRegistration msg;
  {  // start critical section
    std::unique_lock<std::mutex> lock(dsg.mutex);
    msg.request.query = getFrameIdFromNode(*dsg.graph, query_id);
    msg.request.match = getFrameIdFromNode(*dsg.graph, match_id);
    timestamp = dsg.graph->getDynamicNode(query_id).value().get().timestamp.count();
  }  // end critical section

  ScopedTimer timer("lcd/register_agent", timestamp, true, 2, false);

  if (!ros::service::call("frame_registration", msg)) {
    LOG(ERROR) << "Frame registration service failed!";
    return {};
  }

  VLOG(3) << "Visual Registration Request: " << msg.request;
  VLOG(3) << "Visual Registration Response: " << msg.response;

  if (!msg.response.valid) {
    LOG(INFO) << "registration failed: " << NodeSymbol(query_id).getLabel() << " -> "
              << NodeSymbol(match_id).getLabel();
    return {};
  }

  Eigen::Quaterniond match_q_query;
  Eigen::Vector3d match_t_query;
  tf2::convert(msg.response.match_T_query.orientation, match_q_query);
  tf2::convert(msg.response.match_T_query.position, match_t_query);
  LOG(INFO) << "registration worked " << NodeSymbol(query_id).getLabel() << " -> "
            << NodeSymbol(match_id).getLabel();
  return {true,
          query_id,
          match_id,
          gtsam::Pose3(gtsam::Rot3(match_q_query), match_t_query),
          -1};
}

}  // namespace lcd
}  // namespace kimera
