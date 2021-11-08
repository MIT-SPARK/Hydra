#include "kimera_dsg_builder/dsg_lcd_registration.h"
#include <kimera_vio_ros/LcdFrameRegistration.h>
#include <ros/service.h>
#include <tf2_eigen/tf2_eigen.h>

namespace kimera {
namespace lcd {

using incremental::SharedDsgInfo;
using DsgNode = DynamicSceneGraphNode;

struct AgentNodePose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  gtsam::Pose3 world_T_body;
  NodeId id;
};

AgentNodePose getAgentPose(const DynamicSceneGraph& graph, NodeId root_id) {
  const SceneGraphNode& root_node = graph.getNode(root_id).value();

  NodeId child_id = 0;
  for (const auto& root_child : root_node.children()) {
    if (graph.isDynamic(root_child)) {
      child_id = root_child;
      break;
    }
  }

  // TODO(nathan) consider some sort of assert that a child will actually always be
  // found

  const auto& attrs =
      graph.getNode(child_id).value().get().attributes<AgentNodeAttributes>();
  return {gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position), child_id};
}

DsgRegistrationSolution getFullSolutionFromLayer(
    const DynamicSceneGraph& graph,
    const LayerRegistrationSolution& solution,
    NodeId src_root,
    NodeId dest_root) {
  if (!solution.valid || solution.inliers.empty()) {
    return {};
  }

  auto from_pose_info = getAgentPose(graph, src_root);
  auto to_pose_info = getAgentPose(graph, dest_root);

  VLOG(4) << "=================================================";
  VLOG(4) << "world_T_from:";
  VLOG(4) << from_pose_info.world_T_body;
  VLOG(4) << "";
  VLOG(4) << "world_T_to:";
  VLOG(4) << to_pose_info.world_T_body;
  VLOG(4) << "";
  VLOG(4) << "dest_T_src:";
  VLOG(4) << solution.dest_T_src;
  VLOG(4) << "";

  gtsam::Pose3 to_T_from = to_pose_info.world_T_body.inverse() * solution.dest_T_src *
                           from_pose_info.world_T_body;
  VLOG(4) << "to_T_from:";
  VLOG(4) << to_T_from;
  VLOG(4) << "";

  return {true, from_pose_info.id, to_pose_info.id, to_T_from};
}

PlaceRegistrationFunctor::PlaceRegistrationFunctor(
    const LayerRegistrationConfig& config,
    const TeaserParams& params)
    : config(config), solver(params) {}

DsgRegistrationSolution PlaceRegistrationFunctor::operator()(
    SharedDsgInfo& dsg,
    const LayerSearchResults& match) {
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
      *dsg.graph, solution, match.query_root, match.match_root);
}

ObjectRegistrationFunctor::ObjectRegistrationFunctor(
    const LayerRegistrationConfig& config,
    const TeaserParams& params)
    : config(config), solver(params) {}

DsgRegistrationSolution ObjectRegistrationFunctor::operator()(
    SharedDsgInfo& dsg,
    const LayerSearchResults& match) {
  if (match.query_nodes.size() <= 3 || match.match_nodes.size() <= 3) {
    return {};
  }

  LayerRegistrationProblem<std::set<NodeId>> problem;
  problem.src_nodes = match.query_nodes;
  problem.dest_nodes = match.match_nodes;
  problem.src_mutex = &dsg.mutex;

  const SceneGraphLayer& layer = dsg.graph->getLayer(KimeraDsgLayers::OBJECTS).value();
  auto solution = registerDsgLayerSemantic(config, solver, problem, layer);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  return getFullSolutionFromLayer(
      *dsg.graph, solution, match.query_root, match.match_root);
}

inline size_t getFrameIdFromNode(const DynamicSceneGraph& graph, NodeId node_id) {
  const auto& attrs =
      graph.getNode(node_id).value().get().attributes<AgentNodeAttributes>();
  return NodeSymbol(attrs.external_key).categoryId();
}

DsgRegistrationSolution registerAgentMatch(incremental::SharedDsgInfo& dsg,
                                           const LayerSearchResults& match) {
  if (match.query_nodes.empty() || match.match_nodes.empty()) {
    return {};
  }

  if (!ros::service::exists("frame_registration", true)) {
    return {};
  }

  // at the agent level, match sets are one node each
  const NodeId query_id = *match.query_nodes.begin();
  const NodeId match_id = *match.match_nodes.begin();

  kimera_vio_ros::LcdFrameRegistration msg;
  {  // start critical section
    std::unique_lock<std::mutex> lock(dsg.mutex);
    msg.request.query = getFrameIdFromNode(*dsg.graph, query_id);
    msg.request.match = getFrameIdFromNode(*dsg.graph, match_id);
  }  // end critical section

  if (!ros::service::call("frame_registration", msg)) {
    return {};
  }

  VLOG(3) << "Visual Registration Request: " << msg.request;
  VLOG(3) << "Visual Registration Response: " << msg.response;

  if (!msg.response.valid) {
    return {};
  }

  Eigen::Quaterniond match_q_query;
  Eigen::Vector3d match_t_query;
  tf2::convert(msg.response.match_T_query.orientation, match_q_query);
  tf2::convert(msg.response.match_T_query.position, match_t_query);
  return {true,
          query_id,
          match_id,
          gtsam::Pose3(gtsam::Rot3(match_q_query), match_t_query)};
}

}  // namespace lcd
}  // namespace kimera
