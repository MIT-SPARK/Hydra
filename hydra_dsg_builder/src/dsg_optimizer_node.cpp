/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_dsg_builder/incremental_dsg_backend.h"
#include "hydra_dsg_builder/incremental_dsg_frontend.h"

#include <hydra_utils/dsg_mesh_plugins.h>
#include <hydra_utils/dynamic_scene_graph_visualizer.h>

#include "kimera_pgmo/DeformationGraph.h"

namespace hydra {
namespace incremental {

struct DsgOptimizer {
  DsgOptimizer(const ros::NodeHandle& node_handle)
      : nh(node_handle), reset_backend(false) {
    CHECK(nh.getParam("dsg_filepath", dsg_filepath)) << "missing dsg_filepath!";
    CHECK(nh.getParam("dgrf_filepath", dgrf_filepath)) << "missing dgrf_filepath!";
    CHECK(nh.getParam("frontend_filepath", frontend_filepath))
        << "missing frontend_state_filepath";

    const LayerId mesh_layer_id = 1;
    const std::map<LayerId, char>& layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                                {DsgLayers::PLACES, 'p'},
                                                {DsgLayers::ROOMS, 'r'},
                                                {DsgLayers::BUILDINGS, 'b'}};

    frontend_dsg = std::make_shared<SharedDsgInfo>(layer_id_map, mesh_layer_id);
    backend_dsg = std::make_shared<SharedDsgInfo>(layer_id_map, mesh_layer_id);

    frontend_dsg->graph->load(dsg_filepath);
    frontend_dsg->updated = true;

    ros::NodeHandle vnh("/hydra_dsg_visualizer");
    visualizer.reset(new DynamicSceneGraphVisualizer(vnh, getDefaultLayerIds()));

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
  std::unique_ptr<DynamicSceneGraphVisualizer> visualizer;

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
