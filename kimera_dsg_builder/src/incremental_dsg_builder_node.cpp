#include "kimera_dsg_builder/incremental_dsg_backend.h"
#include "kimera_dsg_builder/incremental_dsg_frontend.h"

using kimera::KimeraDsgLayers;
using kimera::LayerId;
using kimera::incremental::SharedDsgInfo;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "incremental_dsg_builder_node");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");
  std::string dsg_output_path = "";
  nh.getParam("dsg_output_path", dsg_output_path);

  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char>& layer_id_map{{KimeraDsgLayers::OBJECTS, 'o'},
                                              {KimeraDsgLayers::PLACES, 'p'},
                                              {KimeraDsgLayers::ROOMS, 'r'},
                                              {KimeraDsgLayers::BUILDINGS, 'b'}};

  SharedDsgInfo::Ptr frontend_dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  SharedDsgInfo::Ptr backend_dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));

  {  // scope for frontend / backend pair
    kimera::incremental::DsgBackend backend(nh, frontend_dsg, backend_dsg);
    kimera::incremental::DsgFrontend frontend(nh, frontend_dsg);

    frontend.start();
    backend.start();

    ros::spin();
  }

  if (!dsg_output_path.empty()) {
    LOG(INFO) << "[DSG Node] Saving scene graph to " << dsg_output_path;
    backend_dsg->graph->save(dsg_output_path);
    LOG(INFO) << "[DSG Node] Saved scene graph to " << dsg_output_path;
  }

  return 0;
}
