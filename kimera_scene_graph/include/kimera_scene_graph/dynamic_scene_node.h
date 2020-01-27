#pragma once

#include <map>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include "graph_cmr_ros/SMPLList.h"
#include "graph_cmr_ros/SMPL.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include "kimera_scene_graph/scene_node.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"
#include "kimera_scene_graph/CentroidErrorRequest.h"

// To iterate over the bag file
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace kimera {

typedef gtsam::NonlinearFactorGraph DynamicNodePoseGraph;
typedef gtsam::BetweenFactor<gtsam::Pose3> OdometryFactor;

const static int NUM_JOINTS = 19;
typedef Eigen::Matrix<double, NUM_JOINTS, 3, Eigen::RowMajor> JointMatrix;

typedef long int AgentId;
struct DynamicSceneNode : SceneNode {
  gtsam::Pose3 pose_;
  JointMatrix joints_;
  AgentId agent_id_;
  graph_cmr_ros::SMPL msg_;
};

typedef std::pair<DynamicSceneNode, size_t> NodeIndex;
 
class DynamicSceneGraph : public SceneGraph {
 public:
    DynamicSceneGraph(const ros::NodeHandle& nh, 
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
        optimized_centroid_check_srv_(),
        bag_(),
        listener_(),
        br_(){

        // GTSAM noise model
        gtsam::Vector motion_precision(6);
        motion_precision.head<3>().setConstant(1 / (motion_position_variance_));
        motion_precision.tail<3>().setConstant(1 / (motion_rotation_variance_));
        motion_noise_model_ = gtsam::noiseModel::Diagonal::Precisions(motion_precision);

        gtsam::Vector detection_precision(6);
        detection_precision.tail<3>().setConstant(1 / (detection_rotation_variance_));
        detection_precision.head<3>().setConstant(1 / (detection_position_variance_));
        detection_noise_model_ = gtsam::noiseModel::Diagonal::Precisions(detection_precision);

        // Params
        nh_private_.param("world_frame", world_frame_, world_frame_);
        nh_private_.param("human_topic", human_topic_, human_topic_);
        nh_private_.param("filter_by_centroid", filter_centroid_, true);
        nh_private_.param("filter_by_mesh", filter_mesh_, true);
        nh_private_.param("feasible_dyn_rate", feasible_dyn_rate_, feasible_dyn_rate_);
        nh_private_.param("feasible_mesh_rate", feasible_mesh_rate_, feasible_mesh_rate_);
        nh_private_.param("node_association_time_cap", time_cap_, time_cap_);
        nh_private_.param("min_node_association_dist", min_node_dist_, min_node_dist_);
        nh_private_.param("human_color", human_color_, human_color_);
        nh_private_.param("centroid_color", centroid_color_, centroid_color_);
        nh_private_.param("serialization_dir", serialization_dir_, serialization_dir_);
        nh_private_.param("deserialization_file", deserialization_file_, deserialization_file_);
        nh_private_.param("prune_theshold", prune_threshold_, prune_threshold_);
        nh_private_.param("single_sequence_smpl_mode", single_sequence_smpl_mode_, single_sequence_smpl_mode_);
        nh_private_.param("outlier_rejection_threshold", outlier_rejection_threshold_, outlier_rejection_threshold_);
        nh_private_.param("human_height", human_height_, human_height_);
        nh_private_.param("draw_skeleton_edges", draw_skeleton_edges_, draw_skeleton_edges_);
        nh_private_.param("edge_thickness", edge_thickness_, edge_thickness_);
        nh_private_.param("merge_close", merge_close_, merge_close_);

        // Publishers
        mesh_pub_ = nh_private_.advertise<graph_cmr_ros::SMPLList>("human_meshes", 1, true);
        semantic_instance_centroid_pub_ = nh_private_.advertise<ColorPointCloud>(
            "all_human_centroids", 1, true);
        edges_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
            "all_graph_edges", 1, true);
        
        // Service
        serialize_graph_srv_ = nh_private_.advertiseService("serialize_dynamic_scene_graph", &DynamicSceneGraph::serializeServiceCall, this);
        deserialize_graph_srv_ = nh_private_.advertiseService("deserialize_dynamic_scene_graph", &DynamicSceneGraph::deserializeServiceCall, this);
        optimized_centroid_check_srv_ = nh_private_.advertiseService("compute_optimized_error", &DynamicSceneGraph::errorServiceCall, this);

        // Subscribe to human meshes
        human_sub_ = nh_private_.subscribe<graph_cmr_ros::SMPLList>(human_topic_, 1, &DynamicSceneGraph::humanCallback, this);

    };

    virtual ~DynamicSceneGraph() = default;

 public:
    // Serialization functionality and interfaces
    bool deserializeServiceCall(
        std_srvs::Empty::Request& request,
        std_srvs::Empty::Response& response) {
        LOG(INFO) << "DynamicSceneNode: Deserialization Requested";
        deserializeAndPublish();
        return true;
    }

    bool serializeServiceCall(
        std_srvs::Empty::Request& request,
        std_srvs::Empty::Response& response) {
        LOG(INFO) << "DynamicSceneNode: Serialization Requested";
        serialize();
        return true;
    }

    bool errorServiceCall(
        kimera_scene_graph::CentroidErrorRequest::Request& request,
        kimera_scene_graph::CentroidErrorRequest::Response& response) {
        LOG(INFO) << "DynamicSceneNode: Optimized Error Requested";
        computeOptimizedError(response);
        response.raw_error = avg_centroid_error_ / meshes_recieved_;
        response.no_outlier_error = no_outlier_error_ / inlier_mesh_count_;
        response.raw_variance = calcVariance(avg_centroid_error_, squared_error_, meshes_recieved_);
        response.no_outlier_variance = calcVariance(no_outlier_error_, squared_no_outlier_error_, inlier_mesh_count_);
        response.max_no_outlier_error = max_no_outlier_error_;
        response.no_outlier_count = inlier_mesh_count_;
        response.raw_count = meshes_recieved_;
        return true;
    }

    void computeOptimizedError(kimera_scene_graph::CentroidErrorRequest::Response& res){
        // --- compute "Precision"
        float total_error = 0;
        float total_squared_error = 0;
        float total_error_pruned = 0;
        float total_pruned_squared_error = 0;
        float max_error = 0;
        float max_error_pruned = 0;
        size_t count = 0;
        size_t total_count = 0;
        for (size_t i = 0 ; i < graph_priors_.size() ; i++){
            if (counts_[i] < prune_threshold_) 
                continue;
            auto values = graph_priors_[i];
            auto graph = pose_graphs_[i];

            // TODO (argupta) Do some pose-graph optimization here.
            gtsam::GaussNewtonOptimizer optimizer(graph, values);
            gtsam::Values result = optimizer.optimize();
            for (size_t j = 0 ; j < result.size() ; j++){
                auto dynamic_node = last_poses_[i][j];
                auto gt_tf = gt_poses_[i][j];
                auto t_old = values.at(j).cast<gtsam::Pose3>().translation();
                auto t = result.at(j).cast<gtsam::Pose3>().translation();
                tf::Vector3 center(t[0], t[1], t[2] - human_height_ / 2.0);
                tf::Vector3 center_old(t_old[0], t_old[1], t_old[2] - human_height_ / 2.0);

                tf::Vector3 diff = center - gt_tf.getOrigin();
                tf::Vector3 diff_old = center_old - gt_tf.getOrigin();

                float error = diff.length();
                if (error > max_error) {
                    max_error = error;
                }
                total_error += error;
                total_squared_error += error * error;

                float error_pruned = diff_old.length();
                if (error_pruned > max_error_pruned) {
                    max_error_pruned = error_pruned;
                }
                total_error_pruned += error_pruned;
                total_pruned_squared_error += error_pruned * error_pruned;
                count++; 
            }
            // total_error += counts_[i] * error / result.size();
            // total_error_pruned += counts_[i] * error_pruned / result.size();
            total_count += counts_[i];
        }
        //LOG(INFO) << "Average Optimized Error: " << total_error / count;
        res.optimized_error = total_error / count;
        res.pruned_error = total_error_pruned / count;
        res.optimized_variance = calcVariance(total_error, total_squared_error, count);
        res.pruned_variance = calcVariance(total_error_pruned, total_pruned_squared_error, count);
        res.max_optimized_error = max_error;
        res.max_pruned_error = max_error_pruned;
        res.pruned_count = total_count;
        res.pruned_no_static = count;
    }

    float calcVariance(float err, float err_sq, int sample_size){
        float avg_sq = err_sq / sample_size;
        float avg = err / sample_size;
        return (avg_sq - pow(avg, 2)) * (sample_size / (sample_size - 1));
    }

    void deserializeAndPublish(){
        if(makedirs(deserialization_file_, false)){
            bag_.open(deserialization_file_, rosbag::bagmode::Read);
            rosbag::View view(bag_);
            foreach(rosbag::MessageInstance const m, view){
                std::string topic = m.getTopic();
                LOG(INFO) << "Deserializing from Topic " << topic;
                if(topic.compare(mesh_pub_.getTopic()) == 0){
                    if(single_sequence_smpl_mode_) continue;
                    graph_cmr_ros::SMPLList::ConstPtr smpls = m.instantiate<graph_cmr_ros::SMPLList>();
                    mesh_pub_.publish(*smpls);
                    continue;
                }
                else if(topic.compare(edges_pub_.getTopic()) == 0){
                    visualization_msgs::Marker::ConstPtr marker = m.instantiate<visualization_msgs::Marker>();
                    edges_pub_.publish(*marker);
                    continue;
                }
                else if(topic.compare(semantic_instance_centroid_pub_.getTopic()) == 0){
                    ColorPointCloud::ConstPtr pt_cloud = m.instantiate<ColorPointCloud>();
                    semantic_instance_centroid_pub_.publish(*pt_cloud);
                    continue;
                }

                size_t chop_idx = topic.find_last_of('_');
                AgentId id = std::stol(topic.substr(chop_idx + 1, topic.size()));
                std::string smpl_msg_name = "human_smpl_msg_" + std::to_string(id);
                if(topic.compare(agent_centroids_.getTopic(id)) == 0){
                    ColorPointCloud::ConstPtr pt_cloud = m.instantiate<ColorPointCloud>();
                    agent_centroids_.publish(id, *pt_cloud);
                }
                else if(topic.compare(agent_graph_edges_.getTopic(id)) == 0){
                    visualization_msgs::Marker::ConstPtr marker = m.instantiate<visualization_msgs::Marker>();
                    agent_graph_edges_.publish(id, *marker);
                }
                else if(topic.compare(skeleton_points_pubs_.getTopic(id)) == 0){
                    ColorPointCloud::ConstPtr pt_cloud = m.instantiate<ColorPointCloud>();
                    skeleton_points_pubs_.publish(id, *pt_cloud);
                }
                else if(topic.compare(skeleton_edge_pubs_.getTopic(id)) == 0){
                    visualization_msgs::Marker::ConstPtr marker = m.instantiate<visualization_msgs::Marker>();
                    skeleton_edge_pubs_.publish(id, *marker);
                }
                else if(topic.compare(smpl_msg_name) == 0 && single_sequence_smpl_mode_) {
                    graph_cmr_ros::SMPLList::ConstPtr smpls = m.instantiate<graph_cmr_ros::SMPLList>();
                    mesh_pub_.publish(*smpls);
                    continue;
                }
            }
            bag_.close();
        }
    }

    inline void serialize(){
        if (makedirs(serialization_dir_)){
            // Put in the number of humans as the file name so we know how many topics. 
            std::string file_path = serialization_dir_ + "/" + std::to_string(graph_priors_.size());
            bag_.open(file_path, rosbag::bagmode::Write);
            visualizeJoints(true);
            visualizePoseGraphs(true);
            bag_.close();
        }
    }

    bool makedirs(std::string& filepath, bool create_dirs=true){
        // Ensure that the directory exists
        boost::filesystem::path dir(filepath);

        if(!(boost::filesystem::exists(dir))){
            LOG(WARNING) << "Serialization Directory: " << filepath << " Doesn't Exist \n Creating Directory ...";
            if (create_dirs){
                if (boost::filesystem::create_directory(dir)){
                    LOG(WARNING) << "...Successfully Created !";
                    return true;
                }
                else {
                    LOG(ERROR) << "...Failed! Aborting Serialization. Please change in launch file!";
                    return false;
                }
            }
            else{
                return false;
            }
        }
        return true;
    }

    /**
     *  Get transformation to every human at a given timestep.
     * 
     */
    void getAllTransforms(std::vector<tf::StampedTransform>& out_transforms, ros::Time& time){
        // All human frames are labelled object_n
        size_t i = 0;
        while(true){
            tf::StampedTransform new_transform;
            try{
                listener_.lookupTransform(world_frame_, "object_" + std::to_string(i), time, new_transform);
                out_transforms.push_back(new_transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
                break;
            }
            i++;
        }
    }

    /**
     * Find the average error
     */
    float evaluateCentroidDistances(DynamicSceneNode& node, tf::StampedTransform& closest_transform){
        std::vector<tf::StampedTransform> gt_human_transforms;
        ros::Time time(node.msg_.header.stamp.sec, node.msg_.header.stamp.nsec);
        getAllTransforms(gt_human_transforms, time);
        tf::Vector3 center(node.attributes_.position_.x, node.attributes_.position_.y, node.attributes_.position_.z - human_height_ / 2.0);
        float min_distance = 10000;
        for (auto human : gt_human_transforms){
            tf::Vector3 diff = center - human.getOrigin();
            float distance = diff.length();
            if (distance < min_distance){
                min_distance = distance;
                closest_transform = human;
            }
        }
        return min_distance;
    }

    void humanCallback(const graph_cmr_ros::SMPLList::ConstPtr& msg){
        for (auto mesh : msg->human_meshes){
            DynamicSceneNode human_node;
            dynamicSceneNodeFromSMPL(mesh, human_node);
            tf::StampedTransform gt_human;
            float dist = evaluateCentroidDistances(human_node, gt_human);
            avg_centroid_error_ += dist;
            meshes_recieved_++;
            // (avg_centroid_error_ * meshes_recieved_ + dist) / (++meshes_recieved_);
            squared_error_ += dist * dist;
            // TODO compute variance online and use to decide whether outlier
            if (dist < outlier_rejection_threshold_){
                no_outlier_error_ += dist ; //(no_outlier_error_ * inlier_mesh_count_ + dist) / (++inlier_mesh_count_);
                squared_no_outlier_error_ += dist * dist;
                inlier_mesh_count_ ++;
                if (dist > max_no_outlier_error_){
                    max_no_outlier_error_ = dist;
                }
                addSceneNode(human_node, gt_human);
            }
        }
        visualizePoseGraphs();
        visualizeJoints();
    }

    inline void dynamicSceneNodeFromSMPL(graph_cmr_ros::SMPL& mesh, DynamicSceneNode& node) {
        node.attributes_.position_ = NodePosition(mesh.centroid[0], mesh.centroid[1], mesh.centroid[2]);
        // FIXME (argupta) add in global orientation information for human mesh;
        // human_node.attributes_.orientation_ = NodeOrientation(0, 0, 0, 1);

        // Initialize joints
        node.joints_ = Eigen::Map<JointMatrix>(mesh.joints.data());

        pcl_conversions::toPCL(mesh.header.stamp, node.attributes_.timestamp_);

        // Keep the mesh for later
        node.msg_ = mesh;

        node.pose_ = gtsam::Pose3(gtsam::Rot3(mesh.orientation[0], mesh.orientation[1], mesh.orientation[2],
                                              mesh.orientation[3], mesh.orientation[4], mesh.orientation[5],
                                              mesh.orientation[6], mesh.orientation[7], mesh.orientation[8]),
                                  gtsam::Point3(mesh.centroid[0], mesh.centroid[1], mesh.centroid[2]));

        colorPclFromJoints(node);

        // TODO (argupta) REMOVE
        gtsam::Quaternion quat = node.pose_.rotation().toQuaternion(); 
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(mesh.centroid[0], mesh.centroid[1], mesh.centroid[2] - human_height_ / 2.0) );
        transform.setRotation( tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()) );
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame_, "human"));
    }

    inline void colorPclFromJoints(DynamicSceneNode& scene_node){
        ColorPointCloud node_pcl;
        scene_node.attributes_.pcl_ = node_pcl.makeShared();
        for (size_t idx = 0 ; idx < NUM_JOINTS; idx++){
            ColorPoint colorpoint(human_color_[0] * 255, human_color_[1] * 255, human_color_[2] * 255);
            colorpoint.x = (float) scene_node.joints_(idx, 0);
            colorpoint.y = (float) scene_node.joints_(idx, 1);
            colorpoint.z = (float) scene_node.joints_(idx, 2);
            scene_node.attributes_.pcl_->points.push_back(colorpoint);
        }
    }

    inline void addSceneNode(DynamicSceneNode& scene_node, tf::StampedTransform& gt_human) {
        // Expectation that scene_node.id_ is blank, use addSceneNodetoPoseGraphs to get id 
        addSceneNodeToPoseGraphs(scene_node, gt_human);
        SceneGraph::addSceneNode(scene_node);
    }
    
    void addSceneNodeToPoseGraphs(DynamicSceneNode& scene_node, tf::StampedTransform& gt_node) {
        //LOG(WARNING) << "Number of graphs: " << pose_graphs_.size(); 
        // Find the scene node that passes all time-checks
        bool node_exists = false;
        float closest_dist = 10000.0;
        size_t closest_idx = 0;
        for(int i = 0; i < pose_graphs_.size(); i++){
            auto last_node = last_poses_[i][last_poses_[i].size()-1];
            float time_diff = (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) * pow(10, -6);
            float c_dist = calcNodeDistance(scene_node, last_node);
            if(!checkCloseness(scene_node, last_node) || time_diff < time_cap_){ 
                if (checkDynamicFeasibility(scene_node, last_node) && checkMeshFeasibility(scene_node, last_node)) {
                    node_exists = true;
                    if(c_dist < closest_dist){
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
            if (checkCloseness(scene_node, last_node)){
                // Assume no motion
                pose_graphs_[closest_idx].push_back(boost::make_shared<OdometryFactor>(last_node_key, last_node_key+1, gtsam::Pose3(), motion_noise_model_));
                pose_graphs_[closest_idx].push_back(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(last_node_key + 1,
                                                                                  scene_node.pose_, detection_noise_model_));
                graph_priors_[closest_idx].insert(last_node_key+1, scene_node.pose_);

                pose_vec.push_back(scene_node);
                gt_poses_[closest_idx].push_back(gt_node);
            }
            else{
                pose_vec[pose_vec.size()-1] = scene_node;
            }
        }
        else {
            pose_graphs_.push_back(DynamicNodePoseGraph());
            size_t last_idx = pose_graphs_.size()-1;
            graph_priors_.push_back(gtsam::Values());
            pose_graphs_[last_idx].add(gtsam::PriorFactor<gtsam::Pose3>(
                0, scene_node.pose_, detection_noise_model_));
            graph_priors_[last_idx].insert(0, scene_node.pose_);
            // ID of the node is the same as the graph index
            scene_node.agent_id_ = last_idx + 1;
            last_poses_.push_back(std::vector<DynamicSceneNode>());
            gt_poses_.push_back(std::vector<tf::StampedTransform>());
            last_poses_[last_idx].push_back(scene_node);
            gt_poses_[last_idx].push_back(gt_node);
            counts_.push_back(1);
        }
    }

    inline float calcNodeDistance(const DynamicSceneNode& scene_node, const DynamicSceneNode& last_node) {
        Eigen::Vector3f diff = scene_node.attributes_.position_.getArray3fMap() - last_node.attributes_.position_.getArray3fMap();
        return diff.norm();
    }

    inline bool checkDynamicFeasibility(const DynamicSceneNode& scene_node, const DynamicSceneNode& last_node) {
        float distance = calcNodeDistance(scene_node, last_node);
        double time_diff = (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) * pow(10, -6);
        //LOG(WARNING) << "checkDynamicFeasibility -- Distance: " << distance << "  Threshold: " << feasible_dyn_rate_ * time_diff;
        return !filter_centroid_ || distance <= feasible_dyn_rate_ * time_diff; // Fail if the node is too far
    }

    inline bool checkCloseness(const DynamicSceneNode& scene_node, const DynamicSceneNode& last_node) {
        Eigen::Vector3f diff = scene_node.attributes_.position_.getArray3fMap() - last_node.attributes_.position_.getArray3fMap();
        double distance = diff.norm();
        return !merge_close_ || distance > min_node_dist_; // Fail if the node is too close
    }

    inline bool checkMeshFeasibility(const DynamicSceneNode& scene_node, const DynamicSceneNode& last_node) {
        JointMatrix diff = scene_node.joints_ - last_node.joints_;
        double max_dist = diff.rowwise().norm().maxCoeff();
        double time_diff = (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) * pow(10, -6);
        //LOG(WARNING) << "checkMeshFeasibility -- Distance: " << max_dist << "  Threshold: " << feasible_mesh_rate_ * time_diff;
        return !filter_mesh_ || max_dist <= feasible_mesh_rate_ * time_diff;
    }

    void setupEdgeMarker(visualization_msgs::Marker& edges, std::vector<float>& color){
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

    void drawEdgesBetweenSkeletons(const DynamicSceneNode& start, const DynamicSceneNode& end, visualization_msgs::Marker& out_edges){
        for (size_t idx = 0 ; idx < NUM_JOINTS; idx++){
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

    void drawSkeletonEdges(DynamicSceneNode& node, visualization_msgs::Marker& marker){
        for (size_t idx = 0 ; idx < NUM_JOINTS; idx++){
            int parent_idx = joint_parents_[idx];
            if (parent_idx != -1){
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

    void visualizeJoints(bool serialize=false){
        graph_cmr_ros::SMPLList msg;
        for(size_t i = 0; i < last_poses_.size() ; i++){
            if (counts_[i] < prune_threshold_) 
                continue;
            auto node_vec = last_poses_[i];
            visualization_msgs::Marker marker;
            setupEdgeMarker(marker, human_color_);
            auto last_node = node_vec[node_vec.size() - 1];
            graph_cmr_ros::SMPLList agent_msg;
            if (draw_skeleton_edges_){
                drawSkeletonEdges(node_vec[0], marker);
                drawSkeletonEdges(last_node, marker);
                visualization_msgs::Marker skeleton_edges;
                setupEdgeMarker(skeleton_edges, edge_color_);
                drawEdgesBetweenSkeletons(node_vec[0], node_vec[node_vec.size() - 1], skeleton_edges);
                agent_graph_edges_.publish(last_node.agent_id_, skeleton_edges);
            }
            else{
                for(auto node : node_vec){
                    drawSkeletonEdges(node, marker);
                    if (single_sequence_smpl_mode_ && !draw_skeleton_edges_){
                        agent_msg.human_meshes.push_back(node.msg_);
                    }
                }
            }
            last_node.attributes_.pcl_->header.frame_id = world_frame_;

            if (serialize){
               bag_.write("human_smpl_msg_" + std::to_string(last_node.agent_id_), ros::Time::now(), agent_msg);
               bag_.write(skeleton_points_pubs_.getTopic(last_node.agent_id_), ros::Time::now(), *last_node.attributes_.pcl_);
               bag_.write(skeleton_edge_pubs_.getTopic(last_node.agent_id_), ros::Time::now(), marker);
            }

            skeleton_points_pubs_.publish(last_node.agent_id_, *last_node.attributes_.pcl_);
            skeleton_edge_pubs_.publish(last_node.agent_id_, marker);
            msg.human_meshes.push_back(last_node.msg_);
            if (single_sequence_smpl_mode_){
                if(draw_skeleton_edges_){
                    agent_msg.human_meshes.push_back(node_vec[0].msg_);
                    agent_msg.human_meshes.push_back(last_node.msg_);
                }
                mesh_pub_.publish(agent_msg);
            }
        }
        // publish the final positions of everyone .
        if (serialize){
            bag_.write(mesh_pub_.getTopic(), ros::Time::now(), msg);
        }
        if (!single_sequence_smpl_mode_){
            mesh_pub_.publish(msg);
        }
    }

    void visualizePoseGraphs(bool serialize=false){
        AgentId id = 0;
        ColorPointCloud all_person_pointcloud;
        visualization_msgs::Marker all_edges;
        setupEdgeMarker(all_edges, edge_color_);
        for ( size_t i = 0 ; i < graph_priors_.size() ; i++){
            if (counts_[i] < prune_threshold_) 
                continue;
            auto values = graph_priors_[i];
            visualization_msgs::Marker marker;
            setupEdgeMarker(marker, edge_color_);
            ColorPointCloud centroid_pointcloud;
            ColorPoint last_centroid;
            int iter_count = 0;
            for(auto key_value_pair : values){
                const gtsam::Point3& centroid = key_value_pair.value.cast<gtsam::Pose3>().translation();
                ColorPoint pcl_centroid(centroid_color_[0] * 255, centroid_color_[1] * 255, centroid_color_[2] * 255);
                pcl_centroid.x = float(centroid.x());
                pcl_centroid.y = float(centroid.y());
                pcl_centroid.z = float(centroid.z());
                centroid_pointcloud.push_back(pcl_centroid);
                all_person_pointcloud.push_back(pcl_centroid);

                // Make sure the last_centroid is filled in.
                if (iter_count++ > 0){
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
            if (serialize){
                bag_.write(agent_centroids_.getTopic(id), ros::Time::now(), centroid_pointcloud);
                bag_.write(agent_graph_edges_.getTopic(id), ros::Time::now(), marker);
            }
            agent_centroids_.publish(id, centroid_pointcloud);
            if (!draw_skeleton_edges_){
                agent_graph_edges_.publish(id, marker);
            }
            id++;
        }
        all_person_pointcloud.header.frame_id = world_frame_;
        if (serialize){
            bag_.write(edges_pub_.getTopic(), ros::Time::now(), all_edges);
            bag_.write(semantic_instance_centroid_pub_.getTopic(), ros::Time::now(), all_person_pointcloud);
        }
        edges_pub_.publish(all_edges);
        semantic_instance_centroid_pub_.publish(all_person_pointcloud);
    }

  public:
    // Average Walking speed is 1.4 m/s
    double feasible_dyn_rate_ = 2.0;
    // Move body-length in 1 sec.
    double feasible_mesh_rate_ = 1.5;

    float human_height_ = 1.5;

    double time_cap_ = 2.0;
    double min_node_dist_ = 0.3;
    std::vector<float> human_color_ = {0.0, 1.0, 0.0};
    std::vector<float> edge_color_ = {0.0, 0.0, 1.0};
    std::vector<float> centroid_color_ = {0.0, 0.0, 1.0};
    int prune_threshold_ = 3;
    int outlier_rejection_threshold_ = 100;
    bool single_sequence_smpl_mode_ = false;
    bool draw_skeleton_edges_ = false;
    bool merge_close_ = true;
    const std::vector<int> joint_parents_ =  {1, 2, 8, 9, 3, 4, 7, 8, 12, 12, 9, 10, 14, -1, 13, -1, -1, 15, 16};

    std::vector<DynamicNodePoseGraph> pose_graphs_;
    std::vector<std::vector<DynamicSceneNode>> last_poses_;
    std::vector<size_t> counts_;
    std::vector<std::vector<tf::StampedTransform>> gt_poses_;
    std::vector<gtsam::Values> graph_priors_;

    // Noise models + params
    gtsam::SharedNoiseModel detection_noise_model_, motion_noise_model_;
    double detection_position_variance_ = 0.01;
    double detection_rotation_variance_ = 0.1;
    double motion_position_variance_ = 0.5;
    double motion_rotation_variance_ = 1.0;
    

  protected:
    // ROS Subscriber
    ros::Subscriber human_sub_;
    ros::Publisher mesh_pub_;
    ros::Publisher edges_pub_;

    ros::ServiceServer serialize_graph_srv_;
    ros::ServiceServer deserialize_graph_srv_;
    ros::ServiceServer optimized_centroid_check_srv_;
    SemanticRosPublishers<AgentId, ColorPointCloud> skeleton_points_pubs_;
    SemanticRosPublishers<AgentId, visualization_msgs::Marker> skeleton_edge_pubs_;

    SemanticRosPublishers<AgentId, ColorPointCloud> agent_centroids_;
    SemanticRosPublishers<AgentId, visualization_msgs::Marker> agent_graph_edges_;

    tf::TransformListener listener_;
    tf::TransformBroadcaster br_;

    // Serializers for the visualization data.
    rosbag::Bag bag_;
    std::string serialization_dir_ = ".";
    std::string deserialization_file_ = ".";
    std::string joint_prefix_ = "joints_";
    std::string joint_edges_prefix_ = "joint_edges_";
    std::string graph_edges_prefix_ = "graph_edges_";
    std::string graph_nodes_prefix_ = "graph_nodes_";

    std::string human_topic_;
    bool filter_centroid_ = true;
    bool filter_mesh_ = true;
    size_t meshes_recieved_ = 0;
    float avg_centroid_error_ = 0.0;
    size_t inlier_mesh_count_ = 0;
    float no_outlier_error_ = 0.0;
    float max_no_outlier_error_ = 0.0;
    float squared_no_outlier_error_ = 0.0;
    float squared_error_ = 0.0;
    float edge_thickness_ = 0.02;
};

} // namespace kimera