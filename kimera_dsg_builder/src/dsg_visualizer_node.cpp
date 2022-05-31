#include <glog/logging.h>
#include <hydra_utils/dsg_streaming_interface.h>
#include <hydra_utils/timing_utilities.h>
#include <kimera_dsg_visualizer/dsg_mesh_plugin.h>
#include <kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h>
#include <ros/ros.h>

#include <fstream>

#include "kimera_dsg_builder/config_utils.h"
#include "kimera_dsg_builder/visualizer_plugins.h"

using DsgVisualizer = kimera::DynamicSceneGraphVisualizer;
using kimera::DsgMeshPlugin;
using kimera::getDefaultLayerIds;
using kimera::PgmoMeshPlugin;
using kimera::VoxbloxMeshPlugin;

namespace hydra {

struct NodeConfig {
  enum class MeshPluginType {
    VOXBLOX,
    RVIZ,
    PGMO,
  } mesh_plugin_type = MeshPluginType::PGMO;
  bool load_graph = false;
  std::string scene_graph_filepath = "";
  std::string visualizer_ns = "/kimera_dsg_visualizer";
  std::string mesh_plugin_ns = "dsg_mesh";
  std::string output_path = "";
};

template <typename Visitor>
void visit_config(const Visitor& v, NodeConfig& config) {
  v.visit("mesh_plugin_type", config.mesh_plugin_type);
  v.visit("load_graph", config.load_graph);
  v.visit("scene_graph_filepath", config.scene_graph_filepath);
  v.visit("visualizer_ns", config.visualizer_ns);
  v.visit("mesh_plugin_ns", config.mesh_plugin_ns);
  v.visit("output_path", config.output_path);
}

using MeshPluginEnum = NodeConfig::MeshPluginType;

struct RosParamLogger : config_parser::Logger {
  inline void log_missing(const std::string& message) const override {
    ROS_INFO_STREAM(message);
  }
};

}  // namespace hydra

DECLARE_CONFIG_ENUM(hydra,
                    MeshPluginEnum,
                    {MeshPluginEnum::VOXBLOX, "VOXBLOX"},
                    {MeshPluginEnum::PGMO, "PGMO"},
                    {MeshPluginEnum::RVIZ, "RVIZ"})

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, NodeConfig)

namespace hydra {

std::string getSpecificMeshNs(const std::string& mesh_ns, MeshPluginEnum mesh_type) {
  switch (mesh_type) {
    case MeshPluginEnum::VOXBLOX:
      return mesh_ns + "/voxblox";
    case MeshPluginEnum::PGMO:
      return mesh_ns + "/pgmo";
    case MeshPluginEnum::RVIZ:
      return mesh_ns + "/rviz";
    default:
      return mesh_ns;
  }
}

struct VisualizerNode {
  VisualizerNode(const ros::NodeHandle& nh) : nh_(nh) {
    auto logger = std::make_shared<RosParamLogger>();
    config_ = config_parser::load_from_ros_nh<hydra::NodeConfig>(nh_, "", logger);
    ROS_INFO_STREAM("Config: " << std::endl << config_);

    ros::NodeHandle viz_nh(config_.visualizer_ns);
    visualizer_.reset(new DsgVisualizer(viz_nh, getDefaultLayerIds()));

    auto mesh_ns = getSpecificMeshNs(config_.mesh_plugin_ns, config_.mesh_plugin_type);
    kimera::DsgVisualizerPlugin::Ptr plugin;
    switch (config_.mesh_plugin_type) {
      case MeshPluginEnum::VOXBLOX:
        plugin = std::make_shared<VoxbloxMeshPlugin>(viz_nh, mesh_ns);
        break;
      case MeshPluginEnum::RVIZ:
        plugin = std::make_shared<DsgMeshPlugin>(viz_nh, mesh_ns);
        break;
      case MeshPluginEnum::PGMO:
      default:
        plugin = std::make_shared<PgmoMeshPlugin>(viz_nh, mesh_ns);
        break;
    }

    visualizer_->addPlugin(plugin);

    if (!config_.output_path.empty()) {
      size_log_file_.reset(
          new std::ofstream(config_.output_path + "/dsg_message_sizes.csv"));
      *size_log_file_ << "time_ns,bytes" << std::endl;
    }

    if (!config_.load_graph || config_.scene_graph_filepath.empty()) {
      receiver_.reset(new DsgReceiver(nh_, [&](const ros::Time& stamp, size_t bytes) {
        if (size_log_file_) {
          *size_log_file_ << stamp.toNSec() << "," << bytes << std::endl;
        }
      }));
    } else {
      loadGraph();
    }
  }

  ~VisualizerNode() {
    std::cout << "timing stats: "
              << hydra::timing::ElapsedTimeRecorder::instance().getStats("receive_dsg")
              << std::endl;
    std::cout << "mesh timing stats: "
              << hydra::timing::ElapsedTimeRecorder::instance().getStats("receive_mesh")
              << std::endl;

    if (config_.output_path.empty()) {
      return;
    }

    timing::ElapsedTimeRecorder::instance().logAllElapsed(config_.output_path);
  }

  void loadGraph() {
    ROS_INFO_STREAM("Loading dsg from: " << config_.scene_graph_filepath);
    auto dsg = kimera::DynamicSceneGraph::load(config_.scene_graph_filepath);
    ROS_INFO_STREAM("Loaded dsg: " << dsg->numNodes() << " nodes, " << dsg->numEdges()
                                   << " edges, has mesh? "
                                   << (dsg->hasMesh() ? "yes" : "no"));
    visualizer_->setGraph(dsg);
  }

  void spin() {
    ROS_DEBUG("Visualizer running");

    if (!config_.load_graph) {
      bool graph_set = false;

      ros::Rate r(10);
      while (ros::ok()) {
        ros::spinOnce();

        if (receiver_ && receiver_->updated()) {
          if (!graph_set) {
            visualizer_->setGraph(receiver_->graph());
            graph_set = true;
          } else {
            visualizer_->setGraphUpdated();
          }

          visualizer_->redraw();
          receiver_->clearUpdated();
        }

        r.sleep();
      }
    } else {
      visualizer_->start();
      ros::spin();
    }
  }

  ros::NodeHandle nh_;
  std::unique_ptr<DsgVisualizer> visualizer_;
  std::unique_ptr<DsgReceiver> receiver_;
  NodeConfig config_;
  std::unique_ptr<std::ofstream> size_log_file_;
};

}  // namespace hydra

int main(int argc, char** argv) {
  ros::init(argc, argv, "dsg_visualizer_node");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("~");
  hydra::VisualizerNode node(nh);
  node.spin();

  return 0;
}
