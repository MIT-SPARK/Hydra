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

  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char>& layer_id_map{
      {to_underlying(KimeraDsgLayers::OBJECTS), 'o'},
      {to_underlying(KimeraDsgLayers::PLACES), 'p'},
      {to_underlying(KimeraDsgLayers::ROOMS), 'r'},
      {to_underlying(KimeraDsgLayers::BUILDINGS), 'b'}};

  SharedDsgInfo::Ptr dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));

  { // scope for frontend / backend pair
    kimera::incremental::DsgBackend backend(nh, dsg);
    kimera::incremental::DsgFrontend frontend(nh, dsg);

    frontend.start();
    backend.start();

    ros::spin();
  }

  return 0;
}
