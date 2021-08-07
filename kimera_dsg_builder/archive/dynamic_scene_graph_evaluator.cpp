#include "kimera_scene_graph/dynamic_scene_graph_evaluator.h"

#include <glog/logging.h>
#include <fstream>

namespace kimera {

DynamicSceneGraphEvaluator::DynamicSceneGraphEvaluator(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private)
    : dsg_raw_(nullptr),
      dsg_pcm_(nullptr),
      dsg_pcm_mesh_(nullptr),
      dsg_pcm_mesh_shape_(nullptr),
      nh_(nh),
      nh_private_(nh_private),
      human_sub_(),
      optimized_centroid_check_srv_(),
      num_humans_(-1) {
  nh_private_.getParam("human_topic", human_topic_);
  nh_private_.getParam("world_frame", world_frame_);
  nh_private_.getParam("output_dir", results_output_dir_);
  nh_private_.getParam("output_file_name", results_filename_);

  // turn off everything
  dsg_raw_ = kimera::make_unique<kimera::DynamicSceneGraph>(nh, nh_private);
  CHECK(dsg_raw_);
  dsg_raw_->rpgo_params_.setPcmSimple3DParams(
      1000.0, 1000.0, KimeraRPGO::Verbosity::QUIET);  // Turn off pcm
  dsg_raw_->check_mesh_dynamic_feasibility_ = false;
  dsg_raw_->check_shape_feasibility_ = false;
  dsg_raw_->visualize_pose_graphs_ = false;
  dsg_raw_->visualize_joints_ = false;

  // pcm enabled
  dsg_pcm_ = kimera::make_unique<kimera::DynamicSceneGraph>(nh, nh_private);
  CHECK(dsg_pcm_);
  dsg_pcm_->check_mesh_dynamic_feasibility_ = false;
  dsg_pcm_->check_shape_feasibility_ = false;
  dsg_pcm_->visualize_pose_graphs_ = false;
  dsg_pcm_->visualize_joints_ = false;

  // mesh check enabled
  dsg_pcm_mesh_ =
      kimera::make_unique<kimera::DynamicSceneGraph>(nh, nh_private);
  CHECK(dsg_pcm_mesh_);
  dsg_pcm_mesh_->check_mesh_dynamic_feasibility_ = true;
  dsg_pcm_mesh_->check_shape_feasibility_ = false;
  dsg_pcm_mesh_->visualize_pose_graphs_ = false;
  dsg_pcm_mesh_->visualize_joints_ = false;

  // shape check enabled
  dsg_pcm_mesh_shape_ =
      kimera::make_unique<kimera::DynamicSceneGraph>(nh, nh_private);
  CHECK(dsg_pcm_mesh_shape_);
  dsg_pcm_mesh_shape_->check_mesh_dynamic_feasibility_ = true;
  dsg_pcm_mesh_shape_->check_shape_feasibility_ = true;
  dsg_pcm_mesh_shape_->visualize_pose_graphs_ = false;
  dsg_pcm_mesh_shape_->visualize_joints_ = false;

  human_color_ = dsg_raw_->human_color_;

  optimized_centroid_check_srv_ = nh_private_.advertiseService(
      "compute_optimized_error",
      &DynamicSceneGraphEvaluator::errorServiceCall,
      this);

  human_sub_ = nh_private_.subscribe<graph_cmr_ros::SMPLList>(
      human_topic_, 1, &DynamicSceneGraphEvaluator::humanCallback, this);

  LOG(INFO) << "DynamicSceneGraphEvaluator Finished Initializing";
}

bool DynamicSceneGraphEvaluator::errorServiceCall(
    kimera_scene_graph::CentroidErrorRequest::Request&,
    kimera_scene_graph::CentroidErrorRequest::Response& response) {
  LOG(INFO) << "DynamicHumanNode: Optimized Error Requested.";

  std::ofstream of_stream(results_output_dir_ + "/" + results_filename_);
  of_stream << "name,pos_error,pose_error_variance,max_pos_error,"
            << "rot_error,rot_error_variance,max_rot_error,num_count"
            << std::endl;

  of_stream << "raw_detections"
            << "," << raw_pos_err_ / num_meshes_ << ","
            << calcVariance(raw_pos_err_, raw_pos_sq_err_, num_meshes_) << ","
            << max_raw_pos_err_ << "," << raw_rot_err_ / num_meshes_ << ","
            << calcVariance(raw_rot_err_, raw_rot_sq_err_, num_meshes_) << ","
            << max_raw_rot_err_ << "," << num_meshes_ << std::endl;

  kimera::DynamicHumanNodeList raw_nodes;
  CHECK(dsg_raw_);
  dsg_raw_->getAllOptimizedHumanNodes(&raw_nodes);
  DynamicSceneGraphEvaluator::ErrorResult raw_res;
  computeOptimizedError(raw_nodes, &raw_res);
  of_stream << "raw_dsg"
            << "," << raw_res.pos_error_ << "," << raw_res.pos_variance_ << ","
            << raw_res.max_pos_error_ << "," << raw_res.rot_error_ << ","
            << raw_res.rot_variance_ << "," << raw_res.max_rot_error_ << ","
            << raw_res.count_ << std::endl;

  kimera::DynamicHumanNodeList pcm_nodes;
  CHECK(dsg_pcm_);
  dsg_pcm_->getAllOptimizedHumanNodes(&pcm_nodes);
  DynamicSceneGraphEvaluator::ErrorResult pcm_res;
  computeOptimizedError(pcm_nodes, &pcm_res);
  of_stream << "pcm_dsg"
            << "," << pcm_res.pos_error_ << "," << pcm_res.pos_variance_ << ","
            << pcm_res.max_pos_error_ << "," << pcm_res.rot_error_ << ","
            << pcm_res.rot_variance_ << "," << pcm_res.max_rot_error_ << ","
            << pcm_res.count_ << std::endl;

  kimera::DynamicHumanNodeList pcm_mesh_nodes;
  CHECK(dsg_pcm_mesh_);
  dsg_pcm_mesh_->getAllOptimizedHumanNodes(&pcm_mesh_nodes);
  DynamicSceneGraphEvaluator::ErrorResult pcm_mesh_res;
  computeOptimizedError(pcm_mesh_nodes, &pcm_mesh_res);
  of_stream << "pcm_mesh_dsg"
            << "," << pcm_mesh_res.pos_error_ << ","
            << pcm_mesh_res.pos_variance_ << "," << pcm_mesh_res.max_pos_error_
            << "," << pcm_mesh_res.rot_error_ << ","
            << pcm_mesh_res.rot_variance_ << "," << pcm_mesh_res.max_rot_error_
            << "," << pcm_mesh_res.count_ << std::endl;

  kimera::DynamicHumanNodeList pcm_mesh_shape_nodes;
  CHECK(dsg_pcm_mesh_shape_);
  dsg_pcm_mesh_shape_->getAllOptimizedHumanNodes(&pcm_mesh_shape_nodes);
  DynamicSceneGraphEvaluator::ErrorResult pcm_mesh_shape_res;
  computeOptimizedError(pcm_mesh_shape_nodes, &pcm_mesh_shape_res);
  of_stream << "pcm_mesh_shape_dsg"
            << "," << pcm_mesh_shape_res.pos_error_ << ","
            << pcm_mesh_shape_res.pos_variance_ << ","
            << pcm_mesh_shape_res.max_pos_error_ << ","
            << pcm_mesh_shape_res.rot_error_ << ","
            << pcm_mesh_shape_res.rot_variance_ << ","
            << pcm_mesh_shape_res.max_rot_error_ << ","
            << pcm_mesh_shape_res.count_ << std::endl;

  of_stream.close();
  response.success = true;
  return true;
}

void DynamicSceneGraphEvaluator::computeOptimizedError(
    const kimera::DynamicHumanNodeList& nodes,
    DynamicSceneGraphEvaluator::ErrorResult* res) {
  // Compute "Precision" of how close we are to the nearest human.
  float total_pos_err = 0.0;
  float total_pos_sq_err = 0.0;
  float max_pos_err = 0.0;

  float total_rot_err = 0.0;
  float total_rot_sq_err = 0.0;
  float max_rot_err = 0.0;

  size_t count = 0;

  // Compute the statics on the optimized data.
  for (size_t i = 0; i < nodes.size(); i++) {
    for (size_t j = 0; j < nodes[i].size(); j++) {
      std::pair<float, float> error = smallestCentroidDistAndAng(nodes[i][j]);
      float pos_err = error.first;
      float rot_err = error.second;

      // There was an error with getting the ground-truth.
      if (pos_err < 0 || rot_err < 0) continue;

      // One of the errors is NaN (TODO: haven't figured out why this happens)
      if (pos_err != pos_err || rot_err != rot_err) continue;

      if (pos_err > max_pos_err) {
        max_pos_err = pos_err;
      }
      if (rot_err > max_rot_err) {
        max_rot_err = rot_err;
      }

      total_pos_err += pos_err;
      total_pos_sq_err += pow(pos_err, 2);
      total_rot_err += rot_err;
      total_rot_sq_err += pow(rot_err, 2);
      count++;
    }
  }

  res->pos_error_ = total_pos_err / count;
  res->pos_variance_ = calcVariance(total_pos_err, total_pos_sq_err, count);
  res->max_pos_error_ = max_pos_err;

  res->rot_error_ = total_rot_err / count;
  res->rot_variance_ = calcVariance(total_rot_err, total_rot_sq_err, count);
  res->max_rot_error_ = max_rot_err;

  res->count_ = count;
}

void DynamicSceneGraphEvaluator::humanCallback(
    const graph_cmr_ros::SMPLList::ConstPtr& msg) {
  if (msg->human_meshes.size() > 0) {
    std::vector<tf::StampedTransform> gt_human_transforms;
    ros::Time time(msg->human_meshes[0].header.stamp.sec,
                   msg->human_meshes[0].header.stamp.nsec);

    // Ensure that we could get all transforms before adding them to
    // ground-truth.
    if (!getAllTransforms(time, &gt_human_transforms)) return;

    time_to_gt_[time.toSec()] = gt_human_transforms;

    for (auto mesh : msg->human_meshes) {
      kimera::DynamicHumanNode human_node;
      DynamicSceneGraph::DynamicHumanNodeFromSMPL(
          mesh, human_color_, &human_node);
      std::pair<float, float> error = smallestCentroidDistAndAng(human_node);
      float dist = error.first;
      float ang = error.second;

      if (dist > 0 && ang > 0) {
        // Record Keeping for raw smpl meshes.
        raw_pos_err_ += dist;
        raw_pos_sq_err_ += dist * dist;
        raw_rot_err_ += ang;
        raw_rot_sq_err_ += ang * ang;
        if (dist > max_raw_pos_err_) {
          max_raw_pos_err_ = dist;
        }
        if (ang > max_raw_rot_err_) {
          max_raw_rot_err_ = ang;
        }
        num_meshes_++;
      }
    }
  }
}

inline bool DynamicSceneGraphEvaluator::getAllTransforms(
    const ros::Time& time,
    std::vector<tf::StampedTransform>* out_transforms) {
  CHECK_NOTNULL(out_transforms);
  out_transforms->clear();

  // All human frames are labeled object_n
  int i = 0;
  bool is_finished = false;
  while (!is_finished) {
    tf::StampedTransform new_transform;
    try {
      listener_.lookupTransform(
          world_frame_, "object_" + std::to_string(i), time, new_transform);
      out_transforms->push_back(new_transform);
      i++;
    } catch (tf::TransformException ex) {
      // ROS_ERROR("%s", ex.what());
      is_finished = true;
    }
  }

  if (num_humans_ == -1) {
    num_humans_ = i;
  }

  if (i != num_humans_) {
    LOG(ERROR) << "Could not get all human transforms: Found " << i
               << " GT poses for humans but previously found " << num_humans_
               << ".";
    return false;
  }
  return true;
}

std::pair<float, float> DynamicSceneGraphEvaluator::smallestCentroidDistAndAng(
    const kimera::DynamicHumanNode& node) {
  // Get the correct ground-truth.
  ros::Time time(node.msg_.header.stamp.sec, node.msg_.header.stamp.nsec);
  double key = time.toSec();
  const auto& gt_loc = time_to_gt_.find(key);
  if (gt_loc == time_to_gt_.end()) return std::make_pair(-1, -1);
  std::vector<tf::StampedTransform>& gt_human_transforms = time_to_gt_.at(key);

  auto& t = node.pose_.translation();
  tf::Vector3 center(t.x(), t.y(), t.z());
  auto& rotation = node.pose_.rotation();
  float min_distance = -1;
  float min_angle = -1;
  for (auto human : gt_human_transforms) {
    tf::Vector3 diff = center - human.getOrigin();
    float distance = diff.length();

    gtsam::Rot3 gt_rotation(human.getRotation().w(),
                            human.getRotation().x(),
                            human.getRotation().y(),
                            human.getRotation().z());
    gtsam::Rot3 est_R_gt = gt_rotation.between(rotation);
    double angle = gtsam::Rot3::Logmap(est_R_gt).norm();

    if (distance < min_distance || min_distance == -1) {
      min_distance = distance;
    }
    if (angle < min_angle || min_angle == -1) {
      min_angle = angle;
    }
  }
  return std::make_pair(min_distance, min_angle);
}

}  // namespace kimera
