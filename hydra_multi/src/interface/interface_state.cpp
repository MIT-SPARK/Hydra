#include "hydra_multi/interface/interface_state.h"

#include <glog/logging.h>

#include <fstream>
#include <iostream>

namespace hydra_multi {

using spark_dsg::SceneGraph;

UnitInterfaceState::UnitInterfaceState()
    : mesh_graph_(new pose_graph_tools::PoseGraph),
      pose_graph_(new pose_graph_tools::PoseGraph),
      mesh_data_(new MeshData),
      mesh_graph_operator_(new DeformationGraphOperator(mesh_graph_)),
      pose_graph_operator_(new PoseGraphOperator(pose_graph_)),
      mesh_operator_(new MeshOperator(mesh_data_)),
      updated(false),
      rebased(false),
      stamp(0) {}

void UnitInterfaceState::save(const std::filesystem::path& log_dir) const {
  std::filesystem::path file_path = log_dir / "world_T_robot.txt";
  std::ofstream ofs(file_path);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Failed to open file: " << file_path << std::endl;
    return;
  }

  Eigen::IOFormat cleanFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n");
  ofs << world_T_robot.matrix().format(cleanFmt) << std::endl;
  ofs.close();

  dsg_->save(log_dir / "dsg.json", false);
}

void UnitInterfaceState::initDsg(SceneGraph::Ptr dsg) {
  dsg_ = dsg;
  dsg_operator_.reset(new SceneGraphOperator(dsg));
}

}  // namespace hydra_multi
