#pragma once
#include <config_utilities/virtual_config.h>
#include <hydra/utils/nearest_neighbor_utilities.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "hydra/common/dsg_types.h"
#include "hydra/frontend/frontier_places_interface.h"
#include "hydra/reconstruction/reconstruction_output.h"

namespace hydra {

typedef std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Quaterniond, size_t>
    Frontier;

class FrontierExtractor : public FrontierPlacesInterface {
 public:
  struct Config {
    char prefix = 'f';
    double cluster_tolerance = .3;
    size_t min_cluster_size = 10;
    size_t max_cluster_size = 100000;
    double max_place_radius = 5;
    bool dense_frontiers = false;
    double frontier_splitting_threshold = 0.2;
    size_t point_threshold = 10;
    size_t culling_point_threshold = 10;
    double recent_block_distance = 25;
  } const config;

  explicit FrontierExtractor(const Config& config);

  void updateRecentBlocks(Eigen::Vector3d current_position, double block_size) override;
  void detectFrontiers(const ReconstructionOutput& input,
                       DynamicSceneGraph& graph,
                       NearestNodeFinder& finder) override;
  void addFrontiers(uint64_t timestamp_ns,
                    DynamicSceneGraph& graph,
                    NearestNodeFinder& finder) override;

 private:
  NodeSymbol next_node_id_;
  std::vector<std::pair<NodeId, BlockIndex>> nodes_to_remove_;

  BlockIndices recently_archived_blocks_;

  std::vector<Frontier> frontiers_;
  std::vector<Frontier> archived_frontiers_;

  void populateDenseFrontiers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr archived_cloud,
                              const double voxel_scale);

  inline static const auto registration_ =
      config::RegistrationWithConfig<FrontierPlacesInterface,
                                     FrontierExtractor,
                                     Config>("voxel_clustering");

  // Helper functions.
  void computeSparseFrontiers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              std::vector<Frontier>& frontiers) const;
};

Eigen::Vector3d frontiersToCenters(const std::vector<Eigen::Vector3f>& positions);

void declare_config(FrontierExtractor::Config& conf);

}  // namespace hydra
