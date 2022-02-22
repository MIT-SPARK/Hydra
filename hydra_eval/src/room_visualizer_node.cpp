#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg_builder/voxblox_utils.h>
#include <kimera_dsg_visualizer/colormap_utils.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/utils/planning_utils.h>
#include <voxblox_msgs/Mesh.h>
#include <yaml-cpp/yaml.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_int32(voxels_per_side, 16, "voxels per side");
DEFINE_double(voxel_size, 0.1, "voxel size");
DEFINE_double(alpha, 0.6, "bbox alpha");
DEFINE_double(luminance, 0.6, "bbox luminance");
DEFINE_double(saturation, 0.8, "bbox saturation");
DEFINE_string(tsdf_file, "", "tsdf file to read");
DEFINE_string(bbox_file, "", "bounding box config file");
DEFINE_string(dsg_file, "", "dsg file to read");

using namespace kimera;
using voxblox::Layer;
using voxblox::TsdfVoxel;

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
    auto rgb = kimera::dsg_utils::getRgbFromHls(hue, FLAGS_luminance, FLAGS_saturation);

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

void fillLayerFromPlaces(const DynamicSceneGraph& graph,
                         std::map<NodeId, voxblox::LongIndexSet>& room_indices) {
  Layer<TsdfVoxel> layer(FLAGS_voxel_size, FLAGS_voxels_per_side);

  const SceneGraphLayer& rooms = graph.getLayer(KimeraDsgLayers::ROOMS).value();
  const SceneGraphLayer& places = graph.getLayer(KimeraDsgLayers::PLACES).value();

  for (const auto& id_node_pair : rooms.nodes()) {
    const NodeId room = id_node_pair.first;
    room_indices[room] = voxblox::LongIndexSet();

    for (const auto& child : id_node_pair.second->children()) {
      const SceneGraphNode& place = places.getNode(child).value();
      const auto& attrs = place.attributes<PlaceNodeAttributes>();

      voxblox::HierarchicalIndexMap indices;
      voxblox::utils::getSphereAroundPoint(
          layer, attrs.position.cast<float>(), attrs.distance, &indices);

      for (const auto& block_idx_pair : indices) {
        for (const auto& voxel_idx : block_idx_pair.second) {
          room_indices[room].insert(voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              block_idx_pair.first, voxel_idx, FLAGS_voxels_per_side));
        }
      }
    }
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "room_visualizer_node");
  ros::NodeHandle nh;

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::SetUsageMessage("utility for comparing visualizing room bounding boxes");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_tsdf_file == "") {
    LOG(FATAL) << "TSDF file is required!";
  }

  LOG(INFO) << "Reading boxes from " << FLAGS_bbox_file;

  Layer<TsdfVoxel> tsdf(FLAGS_voxel_size, FLAGS_voxels_per_side);
  const auto strat = Layer<TsdfVoxel>::BlockMergingStrategy::kReplace;
  if (!voxblox::io::LoadBlocksFromFile(FLAGS_tsdf_file, strat, true, &tsdf)) {
    LOG(FATAL) << "Failed to load TSDF from: " << FLAGS_tsdf_file;
  }

  ros::Publisher room_pub =
      nh.advertise<visualization_msgs::Marker>("room_freespace", 1, true);

  if (FLAGS_dsg_file != "") {
    kimera::DynamicSceneGraph graph;
    graph.load(FLAGS_dsg_file);

    std::map<NodeId, voxblox::LongIndexSet> indices;
    fillLayerFromPlaces(graph, indices);

    visualization_msgs::Marker msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.ns = "room_freespace";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::CUBE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.w = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.scale.x = FLAGS_voxel_size;
    msg.scale.y = FLAGS_voxel_size;
    msg.scale.z = FLAGS_voxel_size;

    for (const auto& id_index_pair : indices) {
      const auto& attrs = graph.getNode(id_index_pair.first)
                              .value()
                              .get()
                              .attributes<SemanticNodeAttributes>();
      std_msgs::ColorRGBA color;
      color.r = attrs.color(0) / 255.0;
      color.g = attrs.color(1) / 255.0;
      color.b = attrs.color(2) / 255.0;
      color.a = 0.7;

      for (const auto& index : id_index_pair.second) {
        voxblox::BlockIndex block_idx;
        voxblox::VoxelIndex voxel_idx;
        voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
            index, tsdf.voxels_per_side(), &block_idx, &voxel_idx);

        if (!tsdf.hasBlock(block_idx)) {
          continue;
        }

        auto pos =
            tsdf.getBlockByIndex(block_idx).computeCoordinatesFromVoxelIndex(voxel_idx);
        geometry_msgs::Point point;
        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);
        msg.points.push_back(point);
        msg.colors.push_back(color);
      }
    }

    room_pub.publish(msg);
  }

  ros::Publisher vxblx_mesh_pub =
      nh.advertise<voxblox_msgs::Mesh>("vxblx_mesh", 1, true);
  ros::Publisher mesh_pub =
      nh.advertise<mesh_msgs::TriangleMeshStamped>("full_mesh", 1, true);

  pcl::PolygonMesh::Ptr mesh;
  kimera::utils::makeMeshFromTsdf(tsdf, mesh, &vxblx_mesh_pub);

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
