#include "kimera_scene_graph/scene_graph_server.h"

#include <ros/ros.h>

#include <glog/logging.h>

#include <voxblox/core/common.h>

namespace kimera {

class SceneGraphSimulationServerImpl : public SceneGraphSimulationServer {
 public:
  SceneGraphSimulationServerImpl(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
      : SceneGraphSimulationServer(nh, nh_private) {}

  void prepareWorld() override {
    CHECK(world_);
    world_->addObject(make_unique<vxb::Sphere>(
        vxb::Point(0.0, 0.0, 2.0), 2.0, vxb::Color::Red()));

    world_->addObject(make_unique<vxb::PlaneObject>(
        vxb::Point(-2.0, -4.0, 2.0), vxb::Point(0, 1, 0), vxb::Color::White()));

    world_->addObject(make_unique<vxb::PlaneObject>(
        vxb::Point(4.0, 0.0, 0.0), vxb::Point(-1, 0, 0), vxb::Color::Pink()));

    world_->addObject(make_unique<vxb::Cube>(
        vxb::Point(-4.0, 4.0, 2.0), vxb::Point(4, 4, 4), vxb::Color::Green()));

    world_->addGroundLevel(0.03);

    world_->generateSdfFromWorld(truncation_distance_, tsdf_gt_.get());
    world_->generateSdfFromWorld(esdf_max_distance_, esdf_gt_.get());
  }
};

}  // namespace kimera

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_scene_graph_simulator");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  kimera::SceneGraphSimulationServerImpl sim_eval(nh, nh_private);

  LOG(INFO) << "Starting sim evaluation.";
  //sim_eval.run();
  LOG(INFO) << "Finished sim evaluation.";
  LOG(INFO) << "Starting scene graph construction.";
  sim_eval.sceneGraphReconstruction();
  LOG(INFO) << "Finishing scene graph construction.";

  ROS_INFO("Done.");
  ros::spin();
  return 0;
}
