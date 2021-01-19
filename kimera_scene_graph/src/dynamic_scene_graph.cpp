#include "kimera_scene_graph/dynamic_scene_graph.h"

#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <limits.h>

namespace kimera {

DynamicSceneGraph::DynamicSceneGraph(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_private_(nh_private),
      human_sub_(),
      skeleton_points_pubs_("human_skeleton_points", nh_private),
      skeleton_edge_pubs_("human_skeleton_edges", nh_private),
      agent_centroids_("agent_centroids", nh_private),
      agent_graph_edges_("agent_graph_edges", nh_private),
      mesh_pub_(),
      edges_pub_(),
      motion_noise_model_(),
      detection_noise_model_(),
      br_(),
      serialize_graph_srv_(),
      deserialize_graph_srv_() {
  // Params
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("human_topic", human_topic_, human_topic_);
  nh_private_.param("check_position_closeness",
                    check_position_closeness_,
                    check_position_closeness_);
  nh_private_.param("check_pose_dynamic_feasibility",
                    check_pose_dynamic_feasibility_,
                    check_pose_dynamic_feasibility_);
  nh_private_.param("check_mesh_dynamic_feasibility",
                    check_mesh_dynamic_feasibility_,
                    check_mesh_dynamic_feasibility_);
  nh_private_.param("check_shape_feasibility",
                    check_shape_feasibility_,
                    check_shape_feasibility_);
  nh_private_.param(
      "visualize_pose_graphs", visualize_pose_graphs_, visualize_pose_graphs_);
  nh_private_.param("visualize_joints", visualize_joints_, visualize_joints_);
  nh_private_.param(
      "feasible_dyn_rate", feasible_dyn_rate_, feasible_dyn_rate_);
  nh_private_.param(
      "feasible_mesh_rate", feasible_mesh_rate_, feasible_mesh_rate_);
  nh_private_.param("node_association_time_cap", time_cap_, time_cap_);
  nh_private_.param(
      "min_node_association_dist", min_node_dist_, min_node_dist_);
  nh_private_.param("max_pose_rot", max_pose_rot_, max_pose_rot_);
  nh_private_.param("human_color", human_color_, human_color_);
  nh_private_.param("centroid_color", centroid_color_, centroid_color_);
  nh_private_.param(
      "pgo_trans_threshold", pgo_trans_threshold_, pgo_trans_threshold_);
  nh_private_.param(
      "pgo_rot_threshold", pgo_rot_threshold_, pgo_rot_threshold_);
  nh_private_.param(
      "beta_diff_threshold", beta_diff_threshold_, beta_diff_threshold_);
  nh_private_.param("prune_threshold", prune_threshold_, prune_threshold_);
  nh_private_.param("single_sequence_smpl_mode",
                    single_sequence_smpl_mode_,
                    single_sequence_smpl_mode_);
  nh_private_.param(
      "draw_skeleton_edges", draw_skeleton_edges_, draw_skeleton_edges_);
  nh_private_.param("edge_thickness", edge_thickness_, edge_thickness_);
  nh_private_.param("detection_position_variance",
                    detection_position_variance_,
                    detection_position_variance_);
  nh_private_.param("detection_rotation_variance",
                    detection_rotation_variance_,
                    detection_rotation_variance_);
  nh_private_.param("motion_position_variance",
                    motion_position_variance_,
                    motion_position_variance_);
  nh_private_.param("motion_rotation_variance",
                    motion_rotation_variance_,
                    motion_rotation_variance_);

  // GTSAM noise model
  CHECK_NE(motion_position_variance_, 0.0);
  gtsam::Vector motion_precision(6);
  motion_precision.head<3>().setConstant(1.0 / motion_position_variance_);
  motion_precision.tail<3>().setConstant(1.0 / motion_rotation_variance_);
  motion_noise_model_ =
      gtsam::noiseModel::Diagonal::Precisions(motion_precision);

  CHECK_NE(detection_rotation_variance_, 0.0);
  gtsam::Vector detection_precision(6);
  detection_precision.tail<3>().setConstant(1.0 / detection_rotation_variance_);
  detection_precision.head<3>().setConstant(1.0 / detection_position_variance_);
  detection_noise_model_ =
      gtsam::noiseModel::Diagonal::Precisions(detection_precision);

  // RPGO Setup
  rpgo_params_ = KimeraRPGO::RobustSolverParams();
  rpgo_params_.setPcmSimple3DParams(
      pgo_trans_threshold_, pgo_rot_threshold_, KimeraRPGO::Verbosity::QUIET);

  // Publishers
  if (visualize_joints_) {
    mesh_pub_ = nh_private_.advertise<graph_cmr_ros::SMPLList>(
        "optimized_human_meshes", 1, true);
  }
  if (visualize_pose_graphs_) {
    semantic_instance_centroid_pub_ =
        nh_private_.advertise<ColorPointCloud>("all_human_centroids", 1, true);
    edges_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
        "all_graph_edges", 1, true);
  }

  // Subscribe to human meshes
  human_sub_ = nh_private_.subscribe<graph_cmr_ros::SMPLList>(
      human_topic_, 1, &DynamicSceneGraph::humanCallback, this);

  // Service
  serialize_graph_srv_ =
      nh_private_.advertiseService("serialize_dynamic_scene_graph",
                                   &DynamicSceneGraph::serializeServiceCall,
                                   this);
  deserialize_graph_srv_ =
      nh_private_.advertiseService("deserialize_dynamic_scene_graph",
                                   &DynamicSceneGraph::deserializeServiceCall,
                                   this);

  LOG(INFO) << "DynamicSceneGraph Finished Initializing";
}

bool DynamicSceneGraph::deserializeServiceCall(
    voxblox_msgs::FilePath::Request& request,
    voxblox_msgs::FilePath::Response& response) {
  LOG(INFO) << "DynamicSceneNode: Deserialization Requested...";
  if (!deserializeAndPublish(request.file_path)) {
    LOG(ERROR) << "Failed to load map!";
    return false;
  } else {
    LOG(INFO) << "Successful deserialization request.";
    return true;
  }
}

inline auto get_agent_id(const std::string& topic) -> AgentId {
  size_t chop_idx = topic.find_last_of('_');
  std::string id_str = topic.substr(chop_idx + 1);
  AgentId id;
  try {
    id = std::stol(id_str);
  } catch (...) {
    LOG(ERROR) << "Not parsing non-numeric agent id: " << id_str.c_str();
  }
  return id;
}

bool DynamicSceneGraph::deserializeAndPublish(
    const std::string& deserialization_file) {
  SerializationHandle<DeserializerHandleInfo> handle(nh_private_,
                                                     deserialization_file);
  if (not handle.get()) {
    LOG(ERROR) << "Deserialization failed";
    return false;
  }

  for (const EntryInfo& info : *(handle.get())) {
    LOG(INFO) << "Deserializing from Topic " << info.topic;

    if (info.topic.compare(mesh_pub_.getTopic()) == 0) {
      if (single_sequence_smpl_mode_) {
        continue;
      }
      graph_cmr_ros::SMPLList::ConstPtr smpls = info.getSMPL();
      CHECK(smpls);
      mesh_pub_.publish(*smpls);
      continue;
    }

    if (info.topic.compare(edges_pub_.getTopic()) == 0) {
      visualization_msgs::Marker::ConstPtr marker = info.getMarker();
      CHECK(marker);
      edges_pub_.publish(*marker);
      continue;
    }

    if (info.topic.compare(semantic_instance_centroid_pub_.getTopic()) == 0) {
      ColorPointCloud::ConstPtr pt_cloud = info.getPCL();
      CHECK(pt_cloud);
      semantic_instance_centroid_pub_.publish(*pt_cloud);
      continue;
    }

    AgentId id = get_agent_id(info.topic);
    std::string smpl_msg_name = "human_smpl_msg_" + std::to_string(id);

    if (info.topic.compare(agent_centroids_.getTopic(id)) == 0) {
      ColorPointCloud::ConstPtr pt_cloud = info.getPCL();
      CHECK(pt_cloud);
      agent_centroids_.publish(id, *pt_cloud);
    } else if (info.topic.compare(agent_graph_edges_.getTopic(id)) == 0) {
      visualization_msgs::Marker::ConstPtr marker = info.getMarker();
      CHECK(marker);
      agent_graph_edges_.publish(id, *marker);
    } else if (info.topic.compare(skeleton_points_pubs_.getTopic(id)) == 0) {
      ColorPointCloud::ConstPtr pt_cloud = info.getPCL();
      CHECK(pt_cloud);
      skeleton_points_pubs_.publish(id, *pt_cloud);
    } else if (info.topic.compare(skeleton_edge_pubs_.getTopic(id)) == 0) {
      visualization_msgs::Marker::ConstPtr marker = info.getMarker();
      CHECK(marker);
      skeleton_edge_pubs_.publish(id, *marker);
    } else if (info.topic.compare(smpl_msg_name) == 0 &&
               single_sequence_smpl_mode_) {
      graph_cmr_ros::SMPLList::ConstPtr smpls = info.getSMPL();
      CHECK(smpls);
      mesh_pub_.publish(*smpls);
      continue;
    }
  }

  return true;
}

bool DynamicSceneGraph::serializeServiceCall(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
  LOG(INFO) << "DynamicSceneNode: Serialization Requested";
  return serialize();
}

void DynamicSceneGraph::humanCallback(
    const graph_cmr_ros::SMPLList::ConstPtr& msg) {
  for (auto mesh : msg->human_meshes) {
    DynamicHumanNode human_node;
    DynamicSceneGraph::DynamicHumanNodeFromSMPL(
        mesh, human_color_, &human_node);
    addSceneNode(human_node);
  }
  if (visualize_pose_graphs_) visualizePoseGraphs();

  if (visualize_joints_) publishOptimizedMeshesAndVis();
}

// TODO(marcus): unit test this and other small helpers
// 3 detections and manually check solution of pose-graph optimization
void DynamicSceneGraph::DynamicHumanNodeFromSMPL(
    const graph_cmr_ros::SMPL& mesh,
    const std::vector<float>& human_color,
    DynamicHumanNode* node) {
  node->layer_id_ = LayerId::kAgentsLayerId;

  // Initialize global position
  node->attributes_.position_ =
      NodePosition(mesh.centroid[0], mesh.centroid[1], mesh.centroid[2]);

  // Initialize joints
  // NOTE: use const JointMatrix because mesh is const ref, joint data is
  // never mutated.
  node->joints_ = Eigen::Map<const JointMatrix>(mesh.joints.data());

  pcl_conversions::toPCL(mesh.header.stamp, node->attributes_.timestamp_);

  node->betas_ = mesh.betas;

  // Keep the mesh for later
  node->msg_ = mesh;

  node->pose_ = gtsam::Pose3(
      gtsam::Rot3(mesh.orientation[0],
                  mesh.orientation[1],
                  mesh.orientation[2],
                  mesh.orientation[3],
                  mesh.orientation[4],
                  mesh.orientation[5],
                  mesh.orientation[6],
                  mesh.orientation[7],
                  mesh.orientation[8]),
      gtsam::Point3(mesh.centroid[0], mesh.centroid[1], mesh.centroid[2]));

  DynamicSceneGraph::colorPclFromJoints(human_color, node);
}

int DynamicSceneGraph::findClosestFeasibleHuman(
    const DynamicHumanNode& scene_node) const {
  // Find the scene node that passes all time-checks
  float closest_dist = std::numeric_limits<float>::max();
  int closest_idx = -1;
  for (int i = 0; i < human_db_.size(); i++) {
    auto last_node = human_db_[i].human_nodes_.back();
    float time_diff =
        (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) *
        pow(10, -6);

    bool is_spatially_or_temporally_close =
        (check_position_closeness_ ? isPositionClose(scene_node, last_node)
                                   : true) ||
        (time_diff < time_cap_);

    bool is_pose_dynamically_feasible =
        check_pose_dynamic_feasibility_
            ? isPoseDynamicallyFeasible(scene_node, last_node)
            : true;

    bool is_mesh_dynamically_feasible =
        check_mesh_dynamic_feasibility_
            ? isMeshDynamicallyFeasible(scene_node, last_node)
            : true;

    bool is_shape_feasible = check_shape_feasibility_
                                 ? isShapeFeasible(scene_node, last_node)
                                 : true;

    if (is_spatially_or_temporally_close && is_pose_dynamically_feasible &&
        is_mesh_dynamically_feasible && is_shape_feasible) {
      float c_dist = calcNodeDistance(scene_node, last_node);
      if (c_dist < closest_dist) {
        closest_dist = c_dist;
        closest_idx = i;
      }
    }
  }
  return closest_idx;
}

void DynamicSceneGraph::addHumanNodeToPoseGraphs(DynamicHumanNode& scene_node) {
  int closest_idx = findClosestFeasibleHuman(scene_node);

  if (closest_idx == -1) {
    // Add a new pose graph for a new human because no matches were
    // found in the db.
    human_db_.push_back(DynamicHumanNodesAndPGO(rpgo_params_));

    gtsam::NonlinearFactorGraph pose_graph;
    gtsam::Values prior_val;

    // Add world-pose prior factor (identity)
    prior_val.insert(gtsam::Key(0), gtsam::Pose3::identity());
    pose_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Key(0), gtsam::Pose3::identity(), detection_noise_model_));

    // Add detection factor
    prior_val.insert(gtsam::Key(1), scene_node.pose_);
    pose_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Key(0),
                                                      gtsam::Key(1),
                                                      scene_node.pose_,
                                                      detection_noise_model_));

    // Assign correct ID to current detection
    scene_node.agent_id_ = human_db_.size() - 1;

    human_db_.back().pgo_->update(pose_graph, prior_val);
    human_db_.back().human_nodes_.push_back(scene_node);
  } else {
    // Update the pose-graph of the matched human.
    // The graph-key of the last detection of the matched human:
    int last_node_key = human_db_[closest_idx].human_nodes_.size();
    int new_node_key = last_node_key + 1;

    // Sanity check
    CHECK_EQ(human_db_[closest_idx].human_nodes_.back().agent_id_, closest_idx);

    gtsam::NonlinearFactorGraph pose_graph;
    gtsam::Values prior_val;

    // Assume no motion
    prior_val.insert(gtsam::Key(new_node_key), scene_node.pose_);
    pose_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Key(last_node_key),
                                                      gtsam::Key(new_node_key),
                                                      gtsam::Pose3::identity(),
                                                      motion_noise_model_));
    // NOTE: PCM requires BetweenFactors, cannot use PriorFactors.
    pose_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Key(0),
                                                      gtsam::Key(new_node_key),
                                                      scene_node.pose_,
                                                      detection_noise_model_));

    // Assign correct ID to current detection
    scene_node.agent_id_ = closest_idx;

    human_db_[closest_idx].pgo_->update(pose_graph, prior_val);
    human_db_[closest_idx].human_nodes_.push_back(scene_node);
  }
}

inline void DynamicSceneGraph::setupEdgeMarker(
    visualization_msgs::Marker& edges,
    std::vector<float>& color) {
  edges.header.frame_id = world_frame_;
  edges.header.stamp = ros::Time();

  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.action = visualization_msgs::Marker::ADD;
  edges.id = 1;
  edges.ns = "node_edges";

  edges.scale.x = edge_thickness_;

  edges.color.a = 1.0;
  edges.color.r = color[0];
  edges.color.g = color[1];
  edges.color.b = color[2];

  edges.pose.position.x = 0.0;
  edges.pose.position.y = 0.0;
  edges.pose.position.z = 0.0;
  edges.pose.orientation.x = 0.0;
  edges.pose.orientation.y = 0.0;
  edges.pose.orientation.z = 0.0;
  edges.pose.orientation.w = 1.0;
}

void DynamicSceneGraph::drawEdgesBetweenSkeletons(
    const DynamicHumanNode& start,
    const DynamicHumanNode& end,
    visualization_msgs::Marker& out_edges) {
  for (size_t idx = 0; idx < NUM_JOINTS; idx++) {
    geometry_msgs::Point joint_begin, joint_end;
    joint_begin.x = start.joints_(idx, 0);
    joint_begin.y = start.joints_(idx, 1);
    joint_begin.z = start.joints_(idx, 2);
    joint_end.x = end.joints_(idx, 0);
    joint_end.y = end.joints_(idx, 1);
    joint_end.z = end.joints_(idx, 2);
    out_edges.points.push_back(joint_begin);
    out_edges.points.push_back(joint_end);
  }
}

void DynamicSceneGraph::drawSkeletonEdges(DynamicHumanNode& node,
                                          visualization_msgs::Marker& marker) {
  for (size_t idx = 0; idx < NUM_JOINTS; idx++) {
    int parent_idx = joint_parents_[idx];
    if (parent_idx != -1) {
      geometry_msgs::Point point, parent;
      point.x = node.joints_(idx, 0);
      point.y = node.joints_(idx, 1);
      point.z = node.joints_(idx, 2);
      parent.x = node.joints_(parent_idx, 0);
      parent.y = node.joints_(parent_idx, 1);
      parent.z = node.joints_(parent_idx, 2);
      marker.points.push_back(point);
      marker.points.push_back(parent);
    }
  }
}

// TODO(marcus): use PoseGraphTools in place of all this!
void DynamicSceneGraph::visualizePoseGraphs(HumansSerializer* serializer) {
  AgentId id = 0;
  ColorPointCloud all_person_pointcloud;
  visualization_msgs::Marker all_edges;
  setupEdgeMarker(all_edges, edge_color_);

  // Get all pruned optimized pose lists
  std::vector<std::vector<gtsam::Pose3>> optimized_pose_lists;
  for (size_t i = 0; i < human_db_.size(); i++) {
    std::vector<gtsam::Pose3> i_optimized_poses;
    if (getOptimizedAndPrunedPoses(i, &i_optimized_poses)) {
      optimized_pose_lists.push_back(i_optimized_poses);
    }
  }
  for (const std::vector<gtsam::Pose3>& optimized_poses :
       optimized_pose_lists) {
    visualization_msgs::Marker marker;
    setupEdgeMarker(marker, edge_color_);
    ColorPointCloud centroid_pointcloud;
    ColorPoint last_centroid;
    int iter_count = 0;
    for (const gtsam::Pose3& pose : optimized_poses) {
      const gtsam::Point3& centroid = pose.translation();
      ColorPoint pcl_centroid(centroid_color_[0] * 255,
                              centroid_color_[1] * 255,
                              centroid_color_[2] * 255);
      pcl_centroid.x = float(centroid.x());
      pcl_centroid.y = float(centroid.y());
      pcl_centroid.z = float(centroid.z());
      centroid_pointcloud.push_back(pcl_centroid);
      all_person_pointcloud.push_back(pcl_centroid);

      // Make sure the last_centroid is filled in.
      if (iter_count++ > 0) {
        geometry_msgs::Point last_vtx, curr_vtx;
        last_vtx.x = last_centroid.x;
        last_vtx.y = last_centroid.y;
        last_vtx.z = last_centroid.z;

        curr_vtx.x = pcl_centroid.x;
        curr_vtx.y = pcl_centroid.y;
        curr_vtx.z = pcl_centroid.z;
        marker.points.push_back(last_vtx);
        marker.points.push_back(curr_vtx);
        all_edges.points.push_back(last_vtx);
        all_edges.points.push_back(curr_vtx);
      }
      last_centroid = pcl_centroid;
    }
    centroid_pointcloud.header.frame_id = world_frame_;
    if (serializer) {
      LOG(INFO) << "writing centroid and edges";
      serializer->write(agent_centroids_.getTopic(id), centroid_pointcloud);
      serializer->write(agent_graph_edges_.getTopic(id), marker);
    }
    agent_centroids_.publish(id, centroid_pointcloud);
    if (!draw_skeleton_edges_) {
      agent_graph_edges_.publish(id, marker);
    }
    id++;
  }
  all_person_pointcloud.header.frame_id = world_frame_;

  if (serializer) {
    LOG(INFO) << "writing edges and pointcloud";
    serializer->write(edges_pub_.getTopic(), all_edges);
    serializer->write(semantic_instance_centroid_pub_.getTopic(),
                      all_person_pointcloud);
  }

  edges_pub_.publish(all_edges);
  semantic_instance_centroid_pub_.publish(all_person_pointcloud);
}

void DynamicSceneGraph::publishOptimizedMeshesAndVis(
    HumansSerializer* serializer) {
  DynamicHumanNodeList optimized_human_nodes;
  getAllOptimizedHumanNodes(&optimized_human_nodes);
  graph_cmr_ros::SMPLList msg;

  for (size_t i = 0; i < optimized_human_nodes.size(); i++) {
    graph_cmr_ros::SMPLList agent_msg;
    visualization_msgs::Marker marker;
    DynamicHumanNodes node_vec = optimized_human_nodes[i];  // for i-th human
    setupEdgeMarker(marker, human_color_);
    DynamicHumanNode last_node = node_vec.back();

    if (draw_skeleton_edges_) {
      drawSkeletonEdges(node_vec[0], marker);
      drawSkeletonEdges(last_node, marker);
      visualization_msgs::Marker skeleton_edges;
      setupEdgeMarker(skeleton_edges, edge_color_);
      drawEdgesBetweenSkeletons(node_vec[0], node_vec.back(), skeleton_edges);
      agent_graph_edges_.publish(last_node.agent_id_, skeleton_edges);
    } else {
      for (auto node : node_vec) {
        drawSkeletonEdges(node, marker);
        if (single_sequence_smpl_mode_ && !draw_skeleton_edges_) {
          agent_msg.human_meshes.push_back(node.msg_);
        }
      }
    }
    last_node.attributes_.pcl_->header.frame_id = world_frame_;

    if (serializer) {
      LOG(INFO) << "writing smpl information";
      serializer->write("human_smpl_msg_" + std::to_string(last_node.agent_id_),
                        agent_msg);
      serializer->write(skeleton_points_pubs_.getTopic(last_node.agent_id_),
                        *last_node.attributes_.pcl_);
      serializer->write(skeleton_edge_pubs_.getTopic(last_node.agent_id_),
                        marker);
    }

    skeleton_points_pubs_.publish(last_node.agent_id_,
                                  *last_node.attributes_.pcl_);
    skeleton_edge_pubs_.publish(last_node.agent_id_, marker);
    msg.human_meshes.push_back(last_node.msg_);
    if (single_sequence_smpl_mode_) {
      if (draw_skeleton_edges_) {
        agent_msg.human_meshes.push_back(node_vec[0].msg_);
        agent_msg.human_meshes.push_back(last_node.msg_);
      }
      mesh_pub_.publish(agent_msg);
    }
  }

  if (serializer) {
    serializer->write(mesh_pub_.getTopic(), msg);
  }
  if (!single_sequence_smpl_mode_) {
    mesh_pub_.publish(msg);
  }
}

void DynamicSceneGraph::getAllOptimizedHumanNodes(
    DynamicHumanNodeList* optimized_node_list) const {
  CHECK_NOTNULL(optimized_node_list);

  for (size_t i = 0; i < human_db_.size(); i++) {
    // Only for pose-graphs that haven't been pruned:
    std::vector<gtsam::Pose3> poses;
    if (getOptimizedAndPrunedPoses(i, &poses)) {
      const DynamicHumanNodes& scene_nodes = human_db_[i].human_nodes_;

      DynamicHumanNodes optimized_scene_nodes;
      for (size_t j = 0; j < poses.size(); j++) {
        DynamicHumanNode dynamic_node =
            scene_nodes.at(j);  // Purposefully copy for opt node.
        dynamic_node.pose_ = poses.at(j);
        optimized_scene_nodes.push_back(dynamic_node);
      }
      optimized_node_list->push_back(optimized_scene_nodes);
    }
  }
}

void DynamicSceneGraph::getOptimizedPoses(size_t idx,
                                          std::vector<gtsam::Pose3>* pose_list,
                                          bool include_prior) const {
  CHECK_NOTNULL(pose_list);
  pose_list->clear();

  gtsam::Values values =
      human_db_[idx].pgo_->calculateBestEstimate();  // TOOD(marcus): ref?

  // NOTE: Start at j=1 because the first factor in the pose-graph
  // is the identity (world pose prior), unless include_prior is true.
  for (size_t j = include_prior ? 0 : 1; j < values.size(); j++) {
    pose_list->push_back(values.at<gtsam::Pose3>(j));
  }
}

bool DynamicSceneGraph::getOptimizedAndPrunedPoses(
    size_t idx,
    std::vector<gtsam::Pose3>* pose_list,
    bool include_prior) const {
  CHECK_NOTNULL(pose_list);
  pose_list->clear();

  // Check if this PGO is to be pruned based on number of detections.
  if (human_db_[idx].human_nodes_.size() < prune_threshold_) {
    return false;
  } else {
    getOptimizedPoses(idx, pose_list, include_prior);
  }
  return true;
}

void DynamicSceneGraph::getAllOptimizedAndPrunedPoses(
    std::vector<std::vector<gtsam::Pose3>>* pose_lists,
    bool include_prior) const {
  CHECK_NOTNULL(pose_lists);
  pose_lists->clear();
  pose_lists->reserve(human_db_.size());

  for (size_t i = 0; i < human_db_.size(); i++) {
    std::vector<gtsam::Pose3> poses_i;
    bool pass = getOptimizedAndPrunedPoses(i, &poses_i, include_prior);
    pose_lists->push_back(poses_i);
  }
}

bool DynamicSceneGraph::isPoseDynamicallyFeasible(
    const DynamicHumanNode& scene_node,
    const DynamicHumanNode& last_node) const {
  float distance = calcNodeDistance(scene_node, last_node);
  double time_diff =
      (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) *
      pow(10, -6);
  double rot_dist = gtsam::Rot3::Logmap(last_node.pose_.rotation().between(
                                            scene_node.pose_.rotation()))
                        .norm();
  return (distance <= feasible_dyn_rate_ * time_diff) &&
         (rot_dist <= max_pose_rot_);
}

bool DynamicSceneGraph::isMeshDynamicallyFeasible(
    const DynamicHumanNode& scene_node,
    const DynamicHumanNode& last_node) const {
  JointMatrix diff = scene_node.joints_ - last_node.joints_;
  double max_dist = diff.rowwise().norm().maxCoeff();
  double time_diff =
      (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) *
      pow(10, -6);
  return max_dist <= feasible_mesh_rate_ * time_diff;
}

bool DynamicSceneGraph::isShapeFeasible(
    const DynamicHumanNode& scene_node,
    const DynamicHumanNode& last_node) const {
  CHECK_EQ(scene_node.betas_.size(), last_node.betas_.size());

  size_t num_betas = scene_node.betas_.size();
  // double beta_diffs [num_betas];
  // for (size_t i = 0; i < num_betas; i++) {
  //   beta_diffs[i] = abs(scene_node.betas_[i] - last_node.betas_[i]);
  // }

  double avg_beta_diff = 0.0;
  for (size_t i = 0; i < num_betas; i++) {
    avg_beta_diff +=
        (abs(scene_node.betas_[i] - last_node.betas_[i]) / num_betas);
  }
  return avg_beta_diff <= beta_diff_threshold_;
}

bool DynamicSceneGraph::serialize() {
  std::string name = "humans_" + std::to_string(human_db_.size());
  SerializationHandle<SerializerHandleInfo> handle(nh_private_, name);
  if (not handle.get()) {
    return false;
  }
  publishOptimizedMeshesAndVis(handle.get());
  visualizePoseGraphs(handle.get());
  return true;
}

void DynamicSceneGraph::colorPclFromJoints(
    const std::vector<float>& human_color,
    DynamicHumanNode* scene_node) {
  ColorPointCloud node_pcl;
  scene_node->attributes_.pcl_ = node_pcl.makeShared();
  for (size_t idx = 0; idx < NUM_JOINTS; idx++) {
    ColorPoint colorpoint(
        human_color[0] * 255, human_color[1] * 255, human_color[2] * 255);
    colorpoint.x = (float)scene_node->joints_(idx, 0);
    colorpoint.y = (float)scene_node->joints_(idx, 1);
    colorpoint.z = (float)scene_node->joints_(idx, 2);
    scene_node->attributes_.pcl_->points.push_back(colorpoint);
  }
}

}  // namespace kimera
