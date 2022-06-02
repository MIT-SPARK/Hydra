#include <hydra_utils/colormap_utils.h>
#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_msgs/Mesh.h>
#include <yaml-cpp/yaml.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_double(alpha, 0.6, "bbox alpha");
DEFINE_double(luminance, 0.6, "bbox luminance");
DEFINE_double(saturation, 0.8, "bbox saturation");
DEFINE_string(bbox_file, "", "bounding box config file");
DEFINE_string(mesh_file, "", "mesh file to read");

using namespace kimera;
using namespace hydra;

void redrawBoundingBoxes(ros::Publisher& bbox_pub) {
  YAML::Node root = YAML::LoadFile(FLAGS_bbox_file);

  const double angle_degrees = !root["angle"] ? 0.0 : root["angle"].as<double>();
  LOG(INFO) << "Using angle: " << angle_degrees;
  const double angle = angle_degrees * M_PI / 180.0;

  visualization_msgs::MarkerArray msg;
  size_t num_rooms = root["rooms"].size();
  size_t idx = 0;
  for (size_t room = 0; room < root["rooms"].size(); ++room) {
    const double hue = static_cast<double>(room) / static_cast<double>(num_rooms);
    auto rgb = dsg_utils::getRgbFromHls(hue, FLAGS_luminance, FLAGS_saturation);

    std_msgs::ColorRGBA color;
    color.r = rgb(0) / 255.0;
    color.g = rgb(1) / 255.0;
    color.b = rgb(2) / 255.0;
    color.a = FLAGS_alpha;

    for (const auto& bbox : root["rooms"][room]) {
      visualization_msgs::Marker bbox_marker;
      bbox_marker.header.frame_id = "world";
      bbox_marker.header.stamp = ros::Time::now();
      bbox_marker.type = visualization_msgs::Marker::CUBE;
      bbox_marker.action = visualization_msgs::Marker::ADD;
      bbox_marker.id = idx;
      bbox_marker.ns = "room_bboxes";
      bbox_marker.color = color;
      bbox_marker.pose.position.x = bbox["pos"][0].as<double>();
      bbox_marker.pose.position.y = bbox["pos"][1].as<double>();
      bbox_marker.pose.position.z = bbox["pos"][2].as<double>();
      bbox_marker.pose.orientation.w = std::cos(angle / 2);
      bbox_marker.pose.orientation.x = 0.0;
      bbox_marker.pose.orientation.y = 0.0;
      bbox_marker.pose.orientation.z = std::sin(angle / 2);
      bbox_marker.scale.x = bbox["scale"][0].as<double>();
      bbox_marker.scale.y = bbox["scale"][1].as<double>();
      bbox_marker.scale.z = bbox["scale"][2].as<double>();

      msg.markers.push_back(bbox_marker);
      ++idx;
    }
  }

  if (!msg.markers.empty()) {
    bbox_pub.publish(msg);
  }
}

struct RedrawFunctor {
  explicit RedrawFunctor(ros::NodeHandle& nh) {
    bbox_pub = nh.advertise<visualization_msgs::MarkerArray>("room_bboxes", 1, true);
  }

  bool call(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    redrawBoundingBoxes(bbox_pub);
    return true;
  };

  ros::Publisher bbox_pub;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "room_visualizer_node");
  ros::NodeHandle nh;

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::SetUsageMessage("utility for comparing visualizing room bounding boxes");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_mesh_file == "") {
    LOG(FATAL) << "Mesh file is required!";
  }

  if (FLAGS_bbox_file == "") {
    LOG(FATAL) << "bounding-box file is required!";
  }

  LOG(INFO) << "Reading boxes from " << FLAGS_bbox_file;

  ros::Publisher mesh_pub =
      nh.advertise<mesh_msgs::TriangleMeshStamped>("full_mesh", 1, true);

  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  kimera_pgmo::ReadMeshFromPly(FLAGS_mesh_file, mesh);

  mesh_msgs::TriangleMeshStamped msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(*mesh);
  mesh_pub.publish(msg);

  std_srvs::Empty temp;
  RedrawFunctor functor(nh);
  functor.call(temp.request, temp.response);

  ros::ServiceServer service =
      nh.advertiseService("redraw_boxes", &RedrawFunctor::call, &functor);

  ros::spin();

  return 0;
}
