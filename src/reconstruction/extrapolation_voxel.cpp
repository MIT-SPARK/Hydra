#include <hydra/reconstruction/semantic_voxel.h>

#include <limits>
#include <random>

namespace hydra {

float getRandomDistance() {
  static std::default_random_engine e;
  // approximately the range of sdf values in a block?
  static std::uniform_real_distribution<> dis(0, 26);
  return dis(e);
}

ExtrapolationVoxel::ExtrapolationVoxel() {
  // Initialize SDF to unit norm gradients in random directions
  gradient = Eigen::Vector3f::Random();
  gradient = gradient / (gradient.norm() + 1e-6);

  distance = getRandomDistance();
  nearest_distance = std::numeric_limits<float>::max();
}

}  // namespace hydra
