#include "hydra_eval/room_utils.h"

#include <kimera_dsg/dynamic_scene_graph.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/utils/planning_utils.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_int32(voxels_per_side, 16, "voxels per side");
DEFINE_double(voxel_size, 0.1, "voxel size");
DEFINE_string(tsdf_file, "", "tsdf file to read");
DEFINE_string(bbox_file, "", "bounding box config file");
DEFINE_string(dsg_file, "", "dsg file to read");

using kimera::NodeId;
using kimera::BoundingBox;
using kimera::DynamicSceneGraph;
using kimera::KimeraDsgLayers;
using voxblox::Layer;
using voxblox::TsdfVoxel;

void getGtRoomIndices(const Layer<TsdfVoxel>& tsdf,
                      std::map<NodeId, voxblox::LongIndexSet>& room_indices) {
  YAML::Node root = YAML::LoadFile(FLAGS_bbox_file);

  const double angle_degrees = !root["angle"] ? 0.0 : root["angle"].as<double>();
  LOG(INFO) << "Using angle: " << angle_degrees;
  const double angle = angle_degrees * M_PI / 180.0;

  std::vector<std::pair<size_t, BoundingBox>> boxes;
  for (size_t room = 0; room < root["rooms"].size(); ++room) {
    room_indices[room] = voxblox::LongIndexSet();
    for (const auto& bbox : root["rooms"][room]) {
      Eigen::Vector3f pos(bbox["pos"][0].as<double>(),
                          bbox["pos"][1].as<double>(),
                          bbox["pos"][2].as<double>());
      Eigen::Vector3f scale(bbox["scale"][0].as<double>(),
                            bbox["scale"][1].as<double>(),
                            bbox["scale"][2].as<double>());
      Eigen::Quaternionf rot(
          std::cos(angle / 2.0f), 0.0f, 0.0f, std::sin(angle / 2.0f));

      boxes.push_back(std::make_pair(room,
                                     BoundingBox(BoundingBox::Type::RAABB,
                                                 -scale / 2.0f,
                                                 scale / 2.0f,
                                                 pos,
                                                 rot.toRotationMatrix())));
    }
  }

  voxblox::BlockIndexList blocks;
  tsdf.getAllAllocatedBlocks(&blocks);

  for (const auto& block_idx : blocks) {
    auto block = tsdf.getBlockPtrByIndex(block_idx);
    for (size_t idx = 0; idx < block->num_voxels(); ++idx) {
      const auto& voxel = block->getVoxelByLinearIndex(idx);
      if (voxel.weight < 1.0e-6 || voxel.distance < 0.0) {
        continue;
      }

      Eigen::Vector3d pos =
          block->computeCoordinatesFromLinearIndex(idx).cast<double>();
      for (const auto& id_bbox_pair : boxes) {
        if (id_bbox_pair.second.isInside(pos)) {
          auto voxel_idx = block->computeVoxelIndexFromLinearIndex(idx);
          room_indices[id_bbox_pair.first].insert(
              voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
                  block_idx, voxel_idx, FLAGS_voxels_per_side));
          break;
        }
      }
    }
  }
}

Eigen::MatrixXd computeOverlap(
    const std::map<NodeId, voxblox::LongIndexSet>& gt_rooms,
    const std::map<NodeId, voxblox::LongIndexSet>& est_rooms) {
  Eigen::MatrixXd overlaps = Eigen::MatrixXd::Zero(gt_rooms.size(), est_rooms.size());
  size_t gt_idx = 0;
  for (const auto& gt_room_pair : gt_rooms) {
    size_t est_idx = 0;
    for (const auto& est_room_pair : est_rooms) {
      for (const auto& index : est_room_pair.second) {
        if (gt_room_pair.second.count(index)) {
          overlaps(gt_idx, est_idx) += 1.0;
        }
      }

      ++est_idx;
    }

    ++gt_idx;
  }

  return overlaps;
}

void eval_rooms(const Layer<TsdfVoxel>& tsdf, const DynamicSceneGraph& graph) {
  std::map<NodeId, voxblox::LongIndexSet> gt_rooms;
  getGtRoomIndices(tsdf, gt_rooms);
  std::vector<size_t> gt_sizes;
  for (const auto& room : gt_rooms) {
    gt_sizes.push_back(room.second.size());
  }

  std::map<NodeId, voxblox::LongIndexSet> est_rooms;
  hydra::fillRoomIndicesFromDsg(
      graph, FLAGS_voxel_size, FLAGS_voxels_per_side, est_rooms);

  std::vector<size_t> est_sizes;
  for (const auto& room : est_rooms) {
    est_sizes.push_back(room.second.size());
  }

  Eigen::MatrixXd overlaps = computeOverlap(gt_rooms, est_rooms);

  std::vector<double> recalls;
  double recall = 0.0;
  for (int i = 0; i < overlaps.rows(); ++i) {
    double max_overlap = 0.0;
    for (int j = 0; j < overlaps.cols(); ++j) {
      if (overlaps(i, j) > max_overlap) {
        max_overlap = overlaps(i, j);
      }
    }

    recall += gt_sizes[i] ? max_overlap / gt_sizes[i] : 0.0;
    recalls.push_back(gt_sizes[i] ? max_overlap / gt_sizes[i] : 0.0);
  }
  recall /= overlaps.rows();

  std::vector<double> precisions;
  double precision = 0.0;
  for (int j = 0; j < overlaps.cols(); ++j) {
    double max_overlap = 0.0;
    for (int i = 0; i < overlaps.rows(); ++i) {
      if (overlaps(i, j) > max_overlap) {
        max_overlap = overlaps(i, j);
      }
    }

    precision += est_sizes[j] ? max_overlap / est_sizes[j] : 0.0;
    precisions.push_back(est_sizes[j] ? max_overlap / est_sizes[j] : 0.0);
  }
  precision /= overlaps.cols();

  nlohmann::json json_results = {
      {"precision", precision},
      {"recall", recall},
      {"precisions", precisions},
      {"recalls", recalls},
  };
  std::cout << json_results;
}

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::SetUsageMessage("utility for comparing visualizing room bounding boxes");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_tsdf_file == "") {
    LOG(FATAL) << "TSDF file is required!";
  }

  if (FLAGS_dsg_file == "") {
    LOG(FATAL) << "DSG file is required!";
  }

  if (FLAGS_bbox_file == "") {
    LOG(FATAL) << "Bounding box file is required!";
  }

  Layer<TsdfVoxel> tsdf(FLAGS_voxel_size, FLAGS_voxels_per_side);
  const auto strat = Layer<TsdfVoxel>::BlockMergingStrategy::kReplace;
  if (!voxblox::io::LoadBlocksFromFile(FLAGS_tsdf_file, strat, true, &tsdf)) {
    LOG(FATAL) << "Failed to load TSDF from: " << FLAGS_tsdf_file;
  }

  DynamicSceneGraph graph;
  graph.load(FLAGS_dsg_file);
  if (!graph.hasLayer(KimeraDsgLayers::ROOMS)) {
    LOG(FATAL) << "Graph file: " << FLAGS_dsg_file << " does not have rooms";
  }

  eval_rooms(tsdf, graph);

  return 0;
}
