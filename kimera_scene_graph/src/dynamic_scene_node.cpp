#include "kimera_scene_graph/dynamic_scene_node.h"

namespace kimera {

DynamicSceneGraph::DynamicSceneGraph(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : SceneGraph(nh, nh_private),
      human_sub_(),
      skeleton_points_pubs_("human_skeleton_points", nh_private),
      skeleton_edge_pubs_("human_skeleton_edges", nh_private),
      agent_centroids_("agent_centroids", nh_private),
      agent_graph_edges_("agent_graph_edges", nh_private),
      mesh_pub_(),
      edges_pub_(),
      serialize_graph_srv_(),
      deserialize_graph_srv_(),
      bag_(),
      br_() {
  // GTSAM noise model
  gtsam::Vector motion_precision(6);
  motion_precision.head<3>().setConstant(1 / (motion_position_variance_));
  motion_precision.tail<3>().setConstant(1 / (motion_rotation_variance_));
  motion_noise_model_ =
      gtsam::noiseModel::Diagonal::Precisions(motion_precision);

  gtsam::Vector detection_precision(6);
  detection_precision.tail<3>().setConstant(1 / (detection_rotation_variance_));
  detection_precision.head<3>().setConstant(1 / (detection_position_variance_));
  detection_noise_model_ =
      gtsam::noiseModel::Diagonal::Precisions(detection_precision);

  // Params
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("human_topic", human_topic_, human_topic_);
  nh_private_.param("filter_by_centroid", filter_centroid_, true);
  nh_private_.param("filter_by_mesh", filter_mesh_, true);
  nh_private_.param(
      "feasible_dyn_rate", feasible_dyn_rate_, feasible_dyn_rate_);
  nh_private_.param(
      "feasible_mesh_rate", feasible_mesh_rate_, feasible_mesh_rate_);
  nh_private_.param("node_association_time_cap", time_cap_, time_cap_);
  nh_private_.param(
      "min_node_association_dist", min_node_dist_, min_node_dist_);
  nh_private_.param("human_color", human_color_, human_color_);
  nh_private_.param("centroid_color", centroid_color_, centroid_color_);
  nh_private_.param(
      "serialization_dir", serialization_dir_, serialization_dir_);
  nh_private_.param("prune_theshold", prune_threshold_, prune_threshold_);
  nh_private_.param("single_sequence_smpl_mode",
                    single_sequence_smpl_mode_,
                    single_sequence_smpl_mode_);
  nh_private_.param(
      "draw_skeleton_edges", draw_skeleton_edges_, draw_skeleton_edges_);
  nh_private_.param("edge_thickness", edge_thickness_, edge_thickness_);
  nh_private_.param("merge_close", merge_close_, merge_close_);

  // Publishers
  mesh_pub_ =
      nh_private_.advertise<graph_cmr_ros::SMPLList>("human_meshes", 1, true);
  semantic_instance_centroid_pub_ =
      nh_private_.advertise<ColorPointCloud>("all_human_centroids", 1, true);
  edges_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "all_graph_edges", 1, true);

  // Service
  serialize_graph_srv_ =
      nh_private_.advertiseService("serialize_dynamic_scene_graph",
                                   &DynamicSceneGraph::serializeServiceCall,
                                   this);
  deserialize_graph_srv_ =
      nh_private_.advertiseService("deserialize_dynamic_scene_graph",
                                   &DynamicSceneGraph::deserializeServiceCall,
                                   this);

  // Subscribe to human meshes
  human_sub_ = nh_private_.subscribe<graph_cmr_ros::SMPLList>(
      human_topic_, 1, &DynamicSceneGraph::humanCallback, this);
}

bool DynamicSceneGraph::deserializeAndPublish(
    const std::string& deserialization_file) {
  if (makedirs(deserialization_file, false)) {
    try {
      bag_.open(deserialization_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
      LOG(ERROR) << "Failed to load rosbag at: " << deserialization_file.c_str();
      LOG(ERROR) << e.what();
      return false;
    }

    rosbag::View view(bag_);
    for (const rosbag::MessageInstance& m : view) {
      std::string topic = m.getTopic();
      LOG(INFO) << "Deserializing from Topic " << topic;

      if (topic.compare(mesh_pub_.getTopic()) == 0) {
        if (single_sequence_smpl_mode_) {
          continue;
        }
        graph_cmr_ros::SMPLList::ConstPtr smpls =
            m.instantiate<graph_cmr_ros::SMPLList>();
        CHECK(smpls);
        mesh_pub_.publish(*smpls);
        continue;
      } else if (topic.compare(edges_pub_.getTopic()) == 0) {
        visualization_msgs::Marker::ConstPtr marker =
            m.instantiate<visualization_msgs::Marker>();
        CHECK(marker);
        edges_pub_.publish(*marker);
        continue;
      } else if (topic.compare(semantic_instance_centroid_pub_.getTopic()) ==
                 0) {
        ColorPointCloud::ConstPtr pt_cloud = m.instantiate<ColorPointCloud>();
        CHECK(pt_cloud);
        semantic_instance_centroid_pub_.publish(*pt_cloud);
        continue;
      }

      size_t chop_idx = topic.find_last_of('_');
      std::string id_str = topic.substr(chop_idx + 1);
      AgentId id;
      try {
        id = std::stol(id_str);
      } catch (...) {
        LOG(ERROR) << "Not parsing non-numeric agent id: " << id_str.c_str();
      }
      std::string smpl_msg_name = "human_smpl_msg_" + std::to_string(id);

      if (topic.compare(agent_centroids_.getTopic(id)) == 0) {
        ColorPointCloud::ConstPtr pt_cloud = m.instantiate<ColorPointCloud>();
        CHECK(pt_cloud);
        agent_centroids_.publish(id, *pt_cloud);
      } else if (topic.compare(agent_graph_edges_.getTopic(id)) == 0) {
        visualization_msgs::Marker::ConstPtr marker =
            m.instantiate<visualization_msgs::Marker>();
        CHECK(marker);
        agent_graph_edges_.publish(id, *marker);
      } else if (topic.compare(skeleton_points_pubs_.getTopic(id)) == 0) {
        ColorPointCloud::ConstPtr pt_cloud = m.instantiate<ColorPointCloud>();
        CHECK(pt_cloud);
        skeleton_points_pubs_.publish(id, *pt_cloud);
      } else if (topic.compare(skeleton_edge_pubs_.getTopic(id)) == 0) {
        visualization_msgs::Marker::ConstPtr marker =
            m.instantiate<visualization_msgs::Marker>();
        CHECK(marker);
        skeleton_edge_pubs_.publish(id, *marker);
      } else if (topic.compare(smpl_msg_name) == 0 &&
                 single_sequence_smpl_mode_) {
        graph_cmr_ros::SMPLList::ConstPtr smpls =
            m.instantiate<graph_cmr_ros::SMPLList>();
        CHECK(smpls);
        mesh_pub_.publish(*smpls);
        continue;
      }
    }
    bag_.close();
    return true;
  }
  return false;
}

void DynamicSceneGraph::humanCallback(
    const graph_cmr_ros::SMPLList::ConstPtr& msg) {
  for (auto mesh : msg->human_meshes) {
    DynamicSceneNode human_node;
    dynamicSceneNodeFromSMPL(mesh, human_node);
    addSceneNode(human_node);
  }
  visualizePoseGraphs();
  visualizeJoints();
}

void DynamicSceneGraph::dynamicSceneNodeFromSMPL(graph_cmr_ros::SMPL& mesh,
                                                 DynamicSceneNode& node) {
  node.attributes_.position_ =
      NodePosition(mesh.centroid[0], mesh.centroid[1], mesh.centroid[2]);

  // Initialize joints
  node.joints_ = Eigen::Map<JointMatrix>(mesh.joints.data());

  pcl_conversions::toPCL(mesh.header.stamp, node.attributes_.timestamp_);

  // Keep the mesh for later
  node.msg_ = mesh;

  node.pose_ = gtsam::Pose3(
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

  colorPclFromJoints(node);
}

void DynamicSceneGraph::addSceneNodeToPoseGraphs(DynamicSceneNode& scene_node) {
  // Find the scene node that passes all time-checks
  bool node_exists = false;
  float closest_dist = 10000.0;
  size_t closest_idx = 0;
  for (int i = 0; i < pose_graphs_.size(); i++) {
    auto last_node = last_poses_[i][last_poses_[i].size() - 1];
    float time_diff =
        (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) *
        pow(10, -6);
    float c_dist = calcNodeDistance(scene_node, last_node);
    if (!checkCloseness(scene_node, last_node) || time_diff < time_cap_) {
      if (checkDynamicFeasibility(scene_node, last_node) &&
          checkMeshFeasibility(scene_node, last_node)) {
        node_exists = true;
        if (c_dist < closest_dist) {
          closest_dist = c_dist;
          closest_idx = i;
        }
      }
    }
  }

  if (node_exists) {
    // Make sure the node is not too close.
    counts_[closest_idx]++;
    std::vector<DynamicSceneNode>& pose_vec = last_poses_[closest_idx];
    size_t last_node_key = pose_vec.size() - 1;
    auto last_node = pose_vec[pose_vec.size() - 1];
    // Identify the scene node as having the same id as the nearby id.
    scene_node.agent_id_ = last_node.agent_id_;
    if (checkCloseness(scene_node, last_node)) {
      // Assume no motion
      pose_graphs_[closest_idx].push_back(
          boost::make_shared<OdometryFactor>(last_node_key,
                                             last_node_key + 1,
                                             gtsam::Pose3(),
                                             motion_noise_model_));
      pose_graphs_[closest_idx].push_back(
          boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
              last_node_key + 1, scene_node.pose_, detection_noise_model_));
      graph_priors_[closest_idx].insert(last_node_key + 1, scene_node.pose_);

      pose_vec.push_back(scene_node);
    } else {
      pose_vec[pose_vec.size() - 1] = scene_node;
    }
  } else {
    pose_graphs_.push_back(DynamicNodePoseGraph());
    size_t last_idx = pose_graphs_.size() - 1;
    graph_priors_.push_back(gtsam::Values());
    pose_graphs_[last_idx].add(gtsam::PriorFactor<gtsam::Pose3>(
        0, scene_node.pose_, detection_noise_model_));
    graph_priors_[last_idx].insert(0, scene_node.pose_);
    // ID of the node is the same as the graph index
    scene_node.agent_id_ = last_idx + 1;
    last_poses_.push_back(std::vector<DynamicSceneNode>());
    last_poses_[last_idx].push_back(scene_node);
    counts_.push_back(1);
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
    const DynamicSceneNode& start,
    const DynamicSceneNode& end,
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

void DynamicSceneGraph::drawSkeletonEdges(DynamicSceneNode& node,
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

void DynamicSceneGraph::visualizeJoints(bool serialize) {
  graph_cmr_ros::SMPLList msg;
  for (size_t i = 0; i < last_poses_.size(); i++) {
    if (counts_[i] < prune_threshold_) continue;
    auto node_vec = last_poses_[i];
    visualization_msgs::Marker marker;
    setupEdgeMarker(marker, human_color_);
    auto last_node = node_vec[node_vec.size() - 1];
    graph_cmr_ros::SMPLList agent_msg;
    if (draw_skeleton_edges_) {
      drawSkeletonEdges(node_vec[0], marker);
      drawSkeletonEdges(last_node, marker);
      visualization_msgs::Marker skeleton_edges;
      setupEdgeMarker(skeleton_edges, edge_color_);
      drawEdgesBetweenSkeletons(
          node_vec[0], node_vec[node_vec.size() - 1], skeleton_edges);
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

    if (serialize) {
      bag_.write("human_smpl_msg_" + std::to_string(last_node.agent_id_),
                 ros::Time::now(),
                 agent_msg);
      bag_.write(skeleton_points_pubs_.getTopic(last_node.agent_id_),
                 ros::Time::now(),
                 *last_node.attributes_.pcl_);
      bag_.write(skeleton_edge_pubs_.getTopic(last_node.agent_id_),
                 ros::Time::now(),
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
  // publish the final positions of everyone .
  if (serialize) {
    bag_.write(mesh_pub_.getTopic(), ros::Time::now(), msg);
  }
  if (!single_sequence_smpl_mode_) {
    mesh_pub_.publish(msg);
  }
}

void DynamicSceneGraph::visualizePoseGraphs(bool serialize) {
  AgentId id = 0;
  ColorPointCloud all_person_pointcloud;
  visualization_msgs::Marker all_edges;
  setupEdgeMarker(all_edges, edge_color_);
  for (size_t i = 0; i < graph_priors_.size(); i++) {
    if (counts_[i] < prune_threshold_) continue;
    auto values = graph_priors_[i];
    visualization_msgs::Marker marker;
    setupEdgeMarker(marker, edge_color_);
    ColorPointCloud centroid_pointcloud;
    ColorPoint last_centroid;
    int iter_count = 0;
    for (auto key_value_pair : values) {
      const gtsam::Point3& centroid =
          key_value_pair.value.cast<gtsam::Pose3>().translation();
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
    if (serialize) {
      bag_.write(
          agent_centroids_.getTopic(id), ros::Time::now(), centroid_pointcloud);
      bag_.write(agent_graph_edges_.getTopic(id), ros::Time::now(), marker);
    }
    agent_centroids_.publish(id, centroid_pointcloud);
    if (!draw_skeleton_edges_) {
      agent_graph_edges_.publish(id, marker);
    }
    id++;
  }
  all_person_pointcloud.header.frame_id = world_frame_;
  if (serialize) {
    bag_.write(edges_pub_.getTopic(), ros::Time::now(), all_edges);
    bag_.write(semantic_instance_centroid_pub_.getTopic(),
               ros::Time::now(),
               all_person_pointcloud);
  }
  edges_pub_.publish(all_edges);
  semantic_instance_centroid_pub_.publish(all_person_pointcloud);
}

}  // namespace kimera
