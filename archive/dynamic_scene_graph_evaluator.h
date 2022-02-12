#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <graph_cmr_ros/SMPL.h>
#include <graph_cmr_ros/SMPLList.h>

#include "kimera_scene_graph/dynamic_scene_graph.h"
#include "kimera_scene_graph/CentroidErrorRequest.h"

namespace kimera {

class DynamicSceneGraphEvaluator {
 public:
  DynamicSceneGraphEvaluator(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private);
  virtual ~DynamicSceneGraphEvaluator() = default;

 public:
  struct ErrorResult {
    float pos_error_;
    float pos_variance_;
    float max_pos_error_;
    float rot_error_;
    float rot_variance_;
    float max_rot_error_;
    int count_;
  };

 public:
  /**
   * @brief Service wrapper to calculate and return the current error values.
   * @param request: The empty request filled in by the ros service call
   * @param response: The response with all error means, variances, and max
   * values.
   * @return True always.
   */
  bool errorServiceCall(
      kimera_scene_graph::CentroidErrorRequest::Request& request,
      kimera_scene_graph::CentroidErrorRequest::Response& response);

  /**
   * @brief computeOptimizedError prunes graphs, runs the optimization and
   * computes the mean, variance, and maximum centroid error of the result.
   * @param nodes: list of list of human nodes.
   * @param res: The service response to be filled in by the function.
   */
  void computeOptimizedError(const kimera::DynamicHumanNodeList& nodes,
                             DynamicSceneGraphEvaluator::ErrorResult* res);
  /**
   * @brief humanCallback wraps the humanCallback function for the
   * DynamicHumanNode and tracks the
   *   corresponding ground-truth data
   * @param msg: The SMPLList message with the corresponding humans
   */
  void humanCallback(const graph_cmr_ros::SMPLList::ConstPtr& msg);

  /**
   * @brief getAllTransforms finds all transformations to the human frames at
   * the given timestep
   */
  bool getAllTransforms(
      const ros::Time& time,
      std::vector<tf::StampedTransform>* out_transforms);

  /**
   * @brief smallestCentroidDistAndAng
   * @param node: The node to find the smallest distance for.
   * @return -1 if the node's timestamp is not in the ground-truth directory.
   * The distance between the node and
   *  the closest ground-truth node for its timestamp otherwise.
   */
  std::pair<float, float> smallestCentroidDistAndAng(const kimera::DynamicHumanNode& node);

  /**
   * @brief calcVariance computes the sample variance
   * @param err: The summed error for all samples.
   * @param err_sq: The sum of squared errors for all samples.
   * @param sample_size: The number or samples.
   * @return The variance of the samples.
   */
  inline float calcVariance(float err, float err_sq, int sample_size) {
    float avg_sq = err_sq / sample_size;
    float avg = err / sample_size;
    return (avg_sq - pow(avg, 2)) * (sample_size / (sample_size - 1));
  }

 public:
  std::unique_ptr<kimera::DynamicSceneGraph> dsg_raw_;
  std::unique_ptr<kimera::DynamicSceneGraph> dsg_pcm_;
  std::unique_ptr<kimera::DynamicSceneGraph> dsg_pcm_mesh_;
  std::unique_ptr<kimera::DynamicSceneGraph> dsg_pcm_mesh_shape_;

  std::unordered_map<double, std::vector<tf::StampedTransform>> time_to_gt_;
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber human_sub_;
  ros::ServiceServer optimized_centroid_check_srv_;
  tf::TransformListener listener_;
  std::vector<float> human_color_;
  int num_humans_;

  // ROS Params
  std::string human_topic_ = "";
  std::string world_frame_ = "";
  std::string results_output_dir_ = "";
  std::string results_filename_ = "";

 protected:
  // Record Keeping
  float raw_pos_err_ = 0;
  float raw_pos_sq_err_ = 0;
  float max_raw_pos_err_ = 0;
  float raw_rot_err_ = 0;
  float raw_rot_sq_err_ = 0;
  float max_raw_rot_err_ = 0;
  int num_meshes_ = 0;
};

}  // namespace kimera
