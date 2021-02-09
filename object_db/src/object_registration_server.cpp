#include <dirent.h>
#include <stdlib.h>
#include <sys/types.h>

#include "object_db/object_registration_server.h"

namespace object_registration {

ObjectRegistrationServer::ObjectRegistrationServer(
    const std::string &name,
    const std::string &target_object_label,
    const std::string &db_path,
    const std::string &gt_path,
    const std::string &label_path,
    const teaser::RobustRegistrationSolver::Params &solver_params,
    const MatcherParams &matcher_params)
    : nh_(),
      nh_private_("~"),
      action_server_(
          nh_,
          name,
          boost::bind(&ObjectRegistrationServer::executeCB, this, _1),
          false),
      action_name_(name),
      solver_params_(solver_params),
      world_frame_id_("map"),
      matcher_(matcher_params),
      target_object_label_(target_object_label) {
  // Get ROS Params
  nh_private_.getParam("world_frame_id", world_frame_id_);

  action_server_.start();
  if (!db_path.empty()) {
    ROS_INFO("Loading Object database.");
    loadObjectDB(db_path);
  } else {
    ROS_WARN("No Object database path provided...");
  }
  if (!label_path.empty()) {
    ROS_INFO("Loading Label database.");
    loadLabelDB(label_path);
  } else {
    ROS_WARN("No Label database path provided...");
  }
  if (!gt_path.empty()) {
    ROS_INFO("Loading GT object database.");
    loadGTDB(gt_path);
  } else {
    ROS_WARN("No GT Object database path provided...");
  }
}

/**
 * Helper function to read in csv.
 * @param str
 * @return
 */
std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream &str) {
  // https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
  std::vector<std::string> result;
  std::string line;
  std::getline(str, line);

  std::stringstream lineStream(line);
  std::string cell;

  while (std::getline(lineStream, cell, ',')) {
    result.push_back(cell);
  }
  // This checks for a trailing comma with no data after it.
  if (!lineStream && cell.empty()) {
    // If there was a trailing comma then add an empty element.
    result.push_back("");
  }
  return result;
}

/**
 * Helper function to convert pcl point cloud to an Eigen matrix
 * @param cloud
 * @param mat
 */
void pclPointCloud2Eigen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                         Eigen::Matrix<double, 3, Eigen::Dynamic> *mat) {
  mat->resize(3, cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    auto cpoint = cloud->points[i];
    mat->col(i) << cpoint.x, cpoint.y, cpoint.z;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr eigen2PclPointCloud(
    const Eigen::Matrix<double, 3, Eigen::Dynamic> &mat) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < mat.cols(); ++i) {
    pcl::PointXYZ cpoint;
    cpoint.x = mat(0, i);
    cpoint.y = mat(1, i);
    cpoint.z = mat(2, i);
    cloud->points.push_back(cpoint);
  }
  return cloud;
}

float calcCentroidErrors(pcl::PointCloud<pcl::PointXYZ> &est_cloud,
                         const std::vector<pcl::PointXYZ> &gt_centroids) {
  pcl::CentroidPoint<pcl::PointXYZ> est_centroid;
  for (size_t idx = 0; idx < est_cloud.size(); ++idx) {
    est_centroid.add(est_cloud.points[idx]);
  }
  pcl::PointXYZ est_centroid_point;
  est_centroid.get(est_centroid_point);
  ROS_INFO("EST Centroid x: %f, y: %f, z: %f",
           est_centroid_point.x,
           est_centroid_point.y,
           est_centroid_point.z);
  // calculate centroid error
  float centroid_error = std::numeric_limits<float>::max();
  for (const auto &gt_centroid : gt_centroids) {
    float c_centroid_error =
        sqrt(std::pow(gt_centroid.x - est_centroid_point.x, 2) +
             std::pow(gt_centroid.y - est_centroid_point.y, 2) +
             std::pow(gt_centroid.z - est_centroid_point.z, 2));
    // ROS_INFO("Current error: %f", c_centroid_error);
    if (c_centroid_error < centroid_error) {
      centroid_error = c_centroid_error;
    }
  }
  return centroid_error;
}

float calcMean(const std::vector<float> &data) {
  float avg = 0;
  for (const auto &num : data) {
    avg += num;
  }
  return avg / static_cast<float>(data.size());
}

float calcSD(const std::vector<float> &data) {
  float mean = calcMean(data);
  float var = 0;
  int numPoints = data.size();
  for (int n = 0; n < numPoints; n++) {
    var += (data[n] - mean) * (data[n] - mean);
  }
  var /= numPoints;
  float sd = sqrt(var);
  return sd;
}

/**
 * Helper function for publishing point clouds given a point cloud db
 * @param db
 */
void ObjectRegistrationServer::dbPublishHelper(
    std::string prefix,
    std::string s_label,
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>> &db,
    std::unordered_map<std::string, ros::Publisher> &pubs,
    ros::NodeHandle &nh) {
  point_cloud->header.frame_id = world_frame_id_;
  bool not_in_reg_object_db = db.find(s_label) == db.end();
  // Create or merge point cloud depending on whether one already exists
  if (not_in_reg_object_db) {
    db[s_label] = *point_cloud;
  } else {
    db[s_label] += *point_cloud;
  }

  // Publish the registrated point clouds
  bool not_in_pubs = pubs.find(s_label) == pubs.end();
  if (not_in_pubs) {
    pubs[s_label] =
        nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(prefix + s_label, 1, true);
  }
  pubs.at(s_label).publish(db[s_label]);
}

/**
 * Server for solving registration with TEASER++
 */
void ObjectRegistrationServer::executeCB(
    const object_db::ObjectRegistrationGoalConstPtr &goal) {
  feedback_.percent_complete = 0;

  teaser::RobustRegistrationSolver solver_(solver_params_);
  // Load some parameters
  auto dst_cloud = goal->dst;
  auto s_label_str = goal->semantic_label;
  auto s_label = std::stoi(s_label_str);
  ROS_INFO("Executing object registration server callback for %s.",
           s_label_str.c_str());

  // Return the original point cloud if label doest not exist
  if (object_db_.find(s_label) == object_db_.end()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < dst_cloud.points.size(); ++i) {
      geometry_msgs::Point32 cpoint;
      cpoint.x = dst_cloud.points[i].x;
      cpoint.y = dst_cloud.points[i].y;
      cpoint.z = dst_cloud.points[i].z;
      result_.aligned_object.points.push_back(cpoint);

      pcl::PointXYZ pcl_point;
      pcl_point.x = dst_cloud.points[i].x;
      pcl_point.y = dst_cloud.points[i].y;
      pcl_point.z = dst_cloud.points[i].z;
      temp_cloud->points.push_back(pcl_point);
    }

    // Calculate unknown centroid errors
    if (gt_centroids_db_[s_label].size() != 0) {
      float centroid_error =
          calcCentroidErrors(*temp_cloud, gt_centroids_db_[s_label]);
      unknown_centroid_errors_.push_back(centroid_error);

      ROS_INFO("Current KNOWN mean centroid error:   %f",
               calcMean(known_centroid_errors_));
      ROS_INFO("Current KNOWN centroid SD:           %f",
               calcSD(known_centroid_errors_));
      ROS_INFO("Current UNKNOWN mean centroid error: %f",
               calcMean(unknown_centroid_errors_));
      ROS_INFO("Current UNKNOWN centroid SD:         %f",
               calcSD(unknown_centroid_errors_));
    }

    ROS_WARN("Label not in database. Aborting Teaser Registration.");
    action_server_.setAborted();
    return;
  }

  auto dst_points = dst_cloud.points;

  // Load src model
  auto src_cloud = object_db_[s_label];
  ROS_INFO("Dst size: %lu.", dst_points.size());
  ROS_INFO("Src size: %lu.", src_cloud.point_cloud->size());

  // Calculate dst keypoints
  ROS_INFO("Calculate Harris keypoints");
  auto dst_keypoints = matcher_.calculateKeypoints(dst_cloud);

  // Calculate src keypoints
  auto src_keypoints = matcher_.calculateKeypoints(src_cloud.point_cloud);
  ROS_INFO("Number of Harris keypoints: src -- %lu, dst -- %lu",
           src_keypoints->size(),
           dst_keypoints->size());

  // Prepare src and dst correspondences
  ROS_INFO("Generate correspondences.");
  size_t N = src_keypoints->size() * dst_keypoints->size();
  // Abort if somehow keypoints size equal to zero
  if (N == 0) {
    ROS_WARN("Keypoints size equal to zero. Abort.");
    action_server_.setAborted();
    return;
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> dst(3, N);
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
  matcher_.generateCorrespondences(src_keypoints, dst_keypoints, &src, &dst);

  // Solve with TEASER++ solver
  ROS_INFO("Solve with TEASER++.");
  solver_.solve(src, dst);
  ROS_INFO("Solve complete!");
  auto solution = solver_.getSolution();
  if (!solution.valid) {
    ROS_WARN("TEASER++ failed. Abort.");
    action_server_.setAborted();
    return;
  }
  ROS_INFO("Solution valid!");

  // Calculate transformed CAD
  Eigen::Matrix<double, 3, 3> R = solution.rotation;
  Eigen::Matrix<double, 3, 1> t = solution.translation;
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_mat(
      3, src_cloud.point_cloud->size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> transformed_object(
      3, src_cloud.point_cloud->size());
  pclPointCloud2Eigen(src_cloud.point_cloud, &src_mat);
  transformed_object = (R * src_mat).colwise() + t;
  for (int i = 0; i < transformed_object.cols(); ++i) {
    geometry_msgs::Point32 cpoint;
    cpoint.x = transformed_object(0, i);
    cpoint.y = transformed_object(1, i);
    cpoint.z = transformed_object(2, i);
    result_.aligned_object.points.push_back(cpoint);
  }
  std::cout << "R:" << std::endl;
  std::cout << R << std::endl;
  std::cout << "==========" << std::endl;
  std::cout << "t:" << std::endl;
  std::cout << t << std::endl;
  std::cout << "==========" << std::endl;

  // Update registrated object db
  auto transformed_cloud = eigen2PclPointCloud(transformed_object);
  transformed_cloud->header.frame_id = world_frame_id_;
  dbPublishHelper("reg_pcl_",
                  s_label_str,
                  transformed_cloud,
                  registrated_object_db_,
                  registrated_object_publishers_,
                  nh_);

  // Calculate GT errors
  ROS_INFO("Calculating GT errors.");
  float centroid_error =
      calcCentroidErrors(*transformed_cloud, gt_centroids_db_[s_label]);
  known_centroid_errors_.push_back(centroid_error);

  // Update src keypoints db
  Eigen::Matrix4f trans_mat = Eigen::Matrix4f::Identity();
  trans_mat.topLeftCorner(3, 3) = R.cast<float>();
  trans_mat.col(3) << t.cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_trans(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*src_keypoints, *src_keypoints_trans, trans_mat);
  src_keypoints_trans->header.frame_id = world_frame_id_;
  dbPublishHelper("src_keypoints_",
                  s_label_str,
                  src_keypoints_trans,
                  src_keypoints_db_,
                  src_keypoints_publishers_,
                  nh_);

  // Update dst keypoints
  dst_keypoints->header.frame_id = world_frame_id_;
  dbPublishHelper("dst_keypoints_",
                  s_label_str,
                  dst_keypoints,
                  dst_keypoints_db_,
                  dst_keypoints_publishers_,
                  nh_);

  feedback_.percent_complete = 1;
  ROS_INFO("%s: Successful object alignment.", action_name_.c_str());
  ROS_INFO("Aligned object size: %lu", result_.aligned_object.points.size());
  ROS_INFO("Current KNOWN mean centroid error:   %f",
           calcMean(known_centroid_errors_));
  ROS_INFO("Current KNOWN centroid SD:           %f",
           calcSD(known_centroid_errors_));
  ROS_INFO("Current UNKNOWN mean centroid error: %f",
           calcMean(unknown_centroid_errors_));
  ROS_INFO("Current UNKNOWN centroid SD:         %f",
           calcSD(unknown_centroid_errors_));
  // set the action state to succeeded
  action_server_.setSucceeded(result_);
}

int ObjectRegistrationServer::loadObjectDB(const std::string &db_root_path) {
  // to absolute path
  char actualpath[PATH_MAX];
  char *ptr = realpath(db_root_path.c_str(), actualpath);
  std::string db_path(ptr);

  // Get immediate subdirectories
  struct dirent *de;
  DIR *dr = opendir(db_path.c_str());
  if (dr == NULL) {
    printf("Could not open provided object database directory");
    return 0;
  }
  std::vector<std::string> sub_dirs;
  while ((de = readdir(dr)) != NULL) {
    std::string dir_name(de->d_name);
    if (de->d_type == DT_DIR) {
      if (dir_name != "." && dir_name != "..") {
        sub_dirs.push_back(dir_name);
      }
    }
  }
  closedir(dr);

  // Load obj files in those sub directories
  PLYReader reader;
  for (const auto &dir_name : sub_dirs) {
    // assume directory name is semantic label
    int slabel = std::stoi(dir_name);

    // assume the following naming conventions
    std::string obj_path = db_path + "/" + dir_name + "/" + "model.obj";
    std::string ply_path = db_path + "/" + dir_name + "/" + "model.ply";
    std::string mtl_path = db_path + "/" + dir_name + "/" + "model.mtl";

    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    ROS_ASSERT(reader.read(ply_path, cloud) == 0);

    // save to object db
    object_db_.insert(
        {slabel, {dir_name, mtl_path, obj_path, ply_path, cloud}});
  }

  return 0;
}

int ObjectRegistrationServer::loadLabelDB(const std::string &label_file) {
  // Read file
  std::ifstream file(label_file);
  auto result = getNextLineAndSplitIntoTokens(file);
  while (file) {
    result = getNextLineAndSplitIntoTokens(file);
    if (result.size() <= 1) {
      break;
    }
    auto rend_name = result[0];
    int id = std::stoi(result[5]);

    ROS_INFO("Loaded pair: %s, %i", rend_name.c_str(), id);
    semantic_label_map_[rend_name] = id;
  }
  return 0;
}

int ObjectRegistrationServer::loadGTDB(const std::string &gt_file) {
  // Read file
  std::ifstream file(gt_file);
  auto result = getNextLineAndSplitIntoTokens(file);
  std::unordered_map<std::string, size_t> counters;
  while (file) {
    result = getNextLineAndSplitIntoTokens(file);
    if (result.size() <= 1) {
      break;
    }
    std::istringstream name_stream(result[0]);
    std::vector<std::string> name_tokens;
    std::copy(std::istream_iterator<std::string>(name_stream),
              std::istream_iterator<std::string>(),
              std::back_inserter(name_tokens));

    // For everything
    std::string object_name = name_tokens[0];
    int id = semantic_label_map_[object_name];
    float centroid_x = std::stof(result[1]);
    float centroid_y = std::stof(result[2]);
    float centroid_z = std::stof(result[3]);

    Eigen::Matrix<float, 3, 1> point_mat(3, 1);
    point_mat << centroid_x, centroid_y, centroid_z;
    Eigen::Matrix<float, 3, 1> transformed_point(3, 1);
    // Eigen::Matrix<float, 3, 3> enu_R_unity_;
    // enu_R_unity_ << 1, 0, 0, 0, 0, 1, 0, 1, 0;
    // transformed_point = enu_R_unity_ * point_mat;
    transformed_point = point_mat;

    pcl::PointXYZ centroid;
    centroid.x = transformed_point[0];
    centroid.y = transformed_point[1];
    centroid.z = transformed_point[2];

    ROS_INFO("Label: %s, Centroid x: %f, y: %f, z: %f",
             object_name.c_str(),
             centroid.x,
             centroid.y,
             centroid.z);
    gt_centroids_db_[id].push_back(centroid);

    // Publish centroid as static tf for RVIZ visualization
    geometry_msgs::Transform static_tf;
    static_tf.translation.x = centroid.x;
    static_tf.translation.y = centroid.y;
    static_tf.translation.z = centroid.z;
    static_tf.rotation.w = std::stof(result[4]);
    static_tf.rotation.x = std::stof(result[5]);
    static_tf.rotation.y = std::stof(result[6]);
    static_tf.rotation.z = std::stof(result[7]);
    counters[object_name]++;
    publishObjectStaticTf(static_tf,
                          world_frame_id_,
                          object_name + std::to_string(counters[object_name]));

    // Search for couch only
    if (name_tokens[0] == target_object_label_) {
      gt_centroids_.push_back(centroid);
    }
  }

  return 0;
}

}  // namespace object_registration
