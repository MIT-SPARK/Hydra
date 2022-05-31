#include "hydra_dsg_builder/incremental_dsg_backend.h"
#include "hydra_dsg_builder/incremental_dsg_frontend.h"
#include "hydra_dsg_builder/visualizer_plugins.h"

#include <kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h>

#include "kimera_pgmo/DeformationGraph.h"

namespace hydra {
namespace incremental {

using kimera::getDefaultLayerIds;

struct DsgOptimizer {
  DsgOptimizer(const ros::NodeHandle& node_handle)
      : nh(node_handle), reset_backend(false) {
    CHECK(nh.getParam("dsg_filepath", dsg_filepath)) << "missing dsg_filepath!";
    CHECK(nh.getParam("dgrf_filepath", dgrf_filepath)) << "missing dgrf_filepath!";
    CHECK(nh.getParam("frontend_filepath", frontend_filepath))
        << "missing frontend_state_filepath";

    const LayerId mesh_layer_id = 1;
    const std::map<LayerId, char>& layer_id_map{{KimeraDsgLayers::OBJECTS, 'o'},
                                                {KimeraDsgLayers::PLACES, 'p'},
                                                {KimeraDsgLayers::ROOMS, 'r'},
                                                {KimeraDsgLayers::BUILDINGS, 'b'}};

    frontend_dsg = std::make_shared<SharedDsgInfo>(layer_id_map, mesh_layer_id);
    backend_dsg = std::make_shared<SharedDsgInfo>(layer_id_map, mesh_layer_id);

    frontend_dsg->graph->load(dsg_filepath);
    frontend_dsg->updated = true;

    ros::NodeHandle vnh("/kimera_dsg_visualizer");
    visualizer.reset(
        new kimera::DynamicSceneGraphVisualizer(vnh, getDefaultLayerIds()));

    bool use_voxblox_mesh_plugin;
    nh.param<bool>("use_voxblox_mesh_plugin", use_voxblox_mesh_plugin, false);
    if (use_voxblox_mesh_plugin) {
      visualizer->addPlugin(std::make_shared<VoxbloxMeshPlugin>(vnh, "dsg_mesh"));
    } else {
      visualizer->addPlugin(std::make_shared<PgmoMeshPlugin>(vnh, "dsg_mesh"));
    }

    visualizer->setGraph(frontend_dsg->graph);
    visualizer->redraw();

    do_optimize();

    optimize_service =
        nh.advertiseService("optimize", &DsgOptimizer::handle_service, this);
  }

  bool handle_service(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    reset_backend = true;
    return true;
  }

  void do_optimize() {
    backend.reset(new DsgBackend(nh, frontend_dsg, backend_dsg));
    backend->startPgmo();
    LOG(ERROR) << "Loading backend state!";
    backend->loadState(frontend_filepath, dgrf_filepath);
    LOG(ERROR) << "Loaded backend state!";

    backend->updateDsgMesh();

    frontend_dsg->updated = true;
    backend->updatePrivateDsg();

    backend->optimize();

    visualizer->setGraph(backend_dsg->graph);
    visualizer->redraw();

    backend->visualizePoseGraph();
    backend->visualizeDeformationGraphEdges();
  }

  void run() {
    ros::WallRate r(10);
    while (ros::ok()) {
      if (reset_backend) {
        reset_backend = false;
        do_optimize();
      }
      r.sleep();
      ros::spinOnce();
    }
  }

  ros::NodeHandle nh;
  bool reset_backend;

  std::string dsg_filepath;
  std::string frontend_filepath;
  std::string dgrf_filepath;

  SharedDsgInfo::Ptr frontend_dsg;
  SharedDsgInfo::Ptr backend_dsg;

  DsgBackend::Ptr backend;
  std::unique_ptr<kimera::DynamicSceneGraphVisualizer> visualizer;

  ros::ServiceServer optimize_service;
};

}  // namespace incremental
}  // namespace hydra

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "dsg_optimizer_node");

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");
  nh.setParam("dsg/call_update_periodically", false);
  hydra::incremental::DsgOptimizer optimizer(nh);
  optimizer.run();

  return 0;
}
