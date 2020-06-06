#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point32.h>
#include <object_db/ObjectRegistrationAction.h>
#include <object_db/object_registration_server.h>
#include <ros/ros.h>
#include <teaser/registration.h>

namespace object_registration {

MatcherParams loadMatcherParams(ros::NodeHandle nh) {
  object_registration::MatcherParams params;
  // get threads
  int num_threads;
  if (nh.getParam("matcher_num_threads", num_threads)) {
    params.num_threads = num_threads;
  }

  bool radius_flag = nh.getParam("matcher_radius", params.radius);
  if (!radius_flag) {
    ROS_ERROR("Not reading radius!");
  }
  nh.getParam("matcher_nms_threshold", params.nms_threshold);
  nh.getParam("matcher_use_nms", params.non_max_suppression);
  nh.getParam("matcher_use_refine", params.set_refine);
  return params;
}

}  // namespace object_registration

int main(int argc, char *argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "object_db_node");

  ros::NodeHandle nh_private("~");
  ROS_INFO("Starting TEASER++ Object Registration node.");

  // Initialize TEASER++ solver params
  teaser::RobustRegistrationSolver::Params params;
  nh_private.getParam("barc2", params.cbar2);
  nh_private.getParam("noise_bound", params.noise_bound);
  nh_private.getParam("estimate_scaling", params.estimate_scaling);
  nh_private.getParam("rotation_gnc_factor", params.rotation_gnc_factor);
  nh_private.getParam("rotation_cost_threshold",
                      params.rotation_cost_threshold);
  nh_private.getParam("max_clique_find_exact",
                      params.max_clique_exact_solution);

  // Set rotation GNC maximum iterations
  int max_iters;
  nh_private.getParam("rotation_max_iterations", max_iters);
  params.rotation_max_iterations = max_iters;

  // Set rotation GNC algorithm
  std::string rot_alg;
  nh_private.getParam("rotation_algorithm", rot_alg);
  std::cout << rot_alg << std::endl;
  if (rot_alg == "GNC_TLS") {
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::
        ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  } else {
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::FGR;
  }

  // Object DB path
  std::string db_path;
  if (nh_private.getParam("object_db_path", db_path)) {
    ROS_INFO("Object database path:%s", db_path.c_str());
  } else {
    ROS_ERROR("Cannot retrieve object database path. Abort.");
    return EXIT_FAILURE;
  }

  // Target object label
  std::string object_label;
  if (nh_private.getParam("target_object_label", object_label)) {
    ROS_INFO("Target object label:%s", object_label.c_str());
  } else {
    ROS_ERROR("Cannot retrieve target object label. Abort.");
    return EXIT_FAILURE;
  }

  // Matcher params
  object_registration::MatcherParams matcher_params =
      object_registration::loadMatcherParams(nh_private);

  // GT Path
  std::string gt_path;
  std::unique_ptr<object_registration::ObjectRegistrationServer> server;
  if (nh_private.getParam("gt_csv_path", gt_path)) {
    ROS_INFO("Ground truth csv path:%s", gt_path.c_str());
    std::string label_path;
    nh_private.getParam("semantic_label_csv_path", label_path);
    ROS_INFO("Preparing object database action server.");
    server = std::make_unique<object_registration::ObjectRegistrationServer>(
        "object_db", db_path, gt_path, label_path, object_label, params, matcher_params);
  } else {
    ROS_INFO("Preparing object database action server.");
    server = std::make_unique<object_registration::ObjectRegistrationServer>(
        "object_db", db_path, object_label, params, matcher_params);
  }
  assert(server != nullptr);

  ROS_INFO("Object registration server ready.");
  ros::spin();
  return EXIT_SUCCESS;
}
