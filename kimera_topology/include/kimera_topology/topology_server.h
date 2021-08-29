#pragma once
#include "kimera_topology/config_parser.h"
#include "kimera_topology/gvd_integrator.h"
#include "kimera_topology/topology_server_visualizer.h"

#include <kimera_topology/ActiveLayer.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ros_params.h>

#include <glog/logging.h>
#include <ros/ros.h>

namespace kimera {
namespace topology {

std::string getHumanReadableMemoryString(size_t bytes) {
  const size_t log_bytes = static_cast<size_t>(std::floor(std::log2(bytes)));
  const size_t unit_index = log_bytes / 10;

  std::stringstream ss;
  ss << std::setprecision(3);
  if (unit_index == 0) {
    ss << bytes << " bytes";
  } else if (unit_index == 1) {
    ss << (static_cast<double>(bytes) / std::pow(2.0, 10)) << " KiB";
  } else if (unit_index == 2) {
    ss << (static_cast<double>(bytes) / std::pow(2.0, 20)) << " MiB";
  } else {
    ss << (static_cast<double>(bytes) / std::pow(2.0, 30)) << " GiB";
  }

  return ss.str();
}

template <typename BaseTsdfServerType>
class TopologyServer {
 public:
  class TsdfServerType : public BaseTsdfServerType {
   public:
    TsdfServerType(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
        : BaseTsdfServerType(nh, pnh), has_pose(false) {}

    virtual void newPoseCallback(const voxblox::Transformation& T_G_C) override {
      has_pose = true;
      T_G_C_last = T_G_C;
    }

    bool has_pose;
    voxblox::Transformation T_G_C_last;
  };

  explicit TopologyServer(const ros::NodeHandle& nh) : nh_(nh) {
    setupConfig("~");
    setupLayers();

    visualizer_.reset(new TopologyServerVisualizer("~"));

    mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("mesh", 1, true);
    layer_pub_ = nh_.advertise<kimera_topology::ActiveLayer>("active_layer", 2, false);

    update_timer_ = nh_.createTimer(ros::Duration(config_.update_period_s),
                                    [&](const ros::TimerEvent&) { runUpdate(); });
  }

  void spin() const { ros::spin(); }

 private:
  void setupLayers() {
    // this intentionally disables marching cubes in the native voxblox server
    nh_.setParam("update_mesh_every_n_sec", 0.0);
    // TODO(nathan) explicit configs
    tsdf_server_.reset(new TsdfServerType(ros::NodeHandle(), nh_));

    tsdf_layer_ = tsdf_server_->getTsdfMapPtr()->getTsdfLayerPtr();
    CHECK_NOTNULL(tsdf_layer_);

    gvd_layer_.reset(
        new Layer<GvdVoxel>(tsdf_layer_->voxel_size(), tsdf_layer_->voxels_per_side()));
    mesh_layer_.reset(new MeshLayer(tsdf_layer_->block_size()));

    gvd_integrator_.reset(
        new GvdIntegrator(gvd_config_, tsdf_layer_, gvd_layer_, mesh_layer_));
  }

  void setupConfig(const std::string& config_ns) {
    fillGvdIntegratorConfig(ros::NodeHandle(config_ns), gvd_config_);
    fillTopologyServerConfig(ros::NodeHandle(config_ns), config_);
  }

  void publishMesh() {
    voxblox_msgs::Mesh mesh_msg;
    generateVoxbloxMeshMsg(mesh_layer_, config_.mesh_color_mode, &mesh_msg);
    mesh_msg.header.frame_id = config_.world_frame;
    mesh_pub_.publish(mesh_msg);
  }

  void publishActiveLayer() const {
    kimera_topology::ActiveLayer msg;
    // TODO(nathan) we might care about more exact timestamping
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = config_.world_frame;

    // non-const, as clearDeletedNodes modifies internal state
    GraphExtractor& extractor = gvd_integrator_->getGraphExtractor();
    std::unordered_set<NodeId> active_nodes = extractor.getActiveNodes();
    std::unordered_set<NodeId> removed_nodes = extractor.getDeletedNodes();
    extractor.clearDeletedNodes();
    msg.layer_contents = extractor.getGraph().serializeLayer(active_nodes);
    msg.deleted_nodes.insert(
        msg.deleted_nodes.begin(), removed_nodes.begin(), removed_nodes.end());
    layer_pub_.publish(msg);
  }

  void showStats() const {
    ROS_INFO_STREAM("Timings: " << std::endl << voxblox::timing::Timing::Print());
    const std::string tsdf_memory_str =
        getHumanReadableMemoryString(tsdf_layer_->getMemorySize());
    const std::string gvd_memory_str =
        getHumanReadableMemoryString(gvd_layer_->getMemorySize());
    const std::string mesh_memory_str =
        getHumanReadableMemoryString(mesh_layer_->getMemorySize());
    ROS_INFO_STREAM("Memory used: [TSDF=" << tsdf_memory_str
                                          << ", GVD=" << gvd_memory_str
                                          << ", Mesh= " << mesh_memory_str << "]");
  }

  void runUpdate() {
    if (!tsdf_layer_ || tsdf_layer_->getNumberOfAllocatedBlocks() == 0) {
      return;
    }

    gvd_integrator_->updateFromTsdfLayer(true);

    if (config_.clear_distant_blocks && tsdf_server_->has_pose) {
      gvd_integrator_->removeDistantBlocks(tsdf_server_->T_G_C_last.getPosition(),
                                           config_.dense_representation_radius_m);

      // this needs to be paired with publishMesh (i.e. generateVoxbloxMeshMsg)
      // to actual remove allocated blocks (instead of getting rid of the contents).
      // handled through publishMesh
      mesh_layer_->clearDistantMesh(tsdf_server_->T_G_C_last.getPosition(),
                                    config_.dense_representation_radius_m);
    }

    publishMesh();
    publishActiveLayer();

    visualizer_->visualize(gvd_integrator_->getGraphExtractor(),
                           gvd_integrator_->getGraph(),
                           *gvd_layer_,
                           *tsdf_layer_);

    if (config_.show_stats) {
      showStats();
    }
  }

 private:
  ros::NodeHandle nh_;

  TopologyServerConfig config_;
  GvdIntegratorConfig gvd_config_;

  std::unique_ptr<TopologyServerVisualizer> visualizer_;

  ros::Publisher mesh_pub_;
  ros::Publisher layer_pub_;

  Layer<TsdfVoxel>* tsdf_layer_;
  Layer<GvdVoxel>::Ptr gvd_layer_;
  MeshLayer::Ptr mesh_layer_;

  std::unique_ptr<TsdfServerType> tsdf_server_;
  std::unique_ptr<GvdIntegrator> gvd_integrator_;

  ros::Timer update_timer_;
};

}  // namespace topology
}  // namespace kimera
