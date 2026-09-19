#pragma once

#include <spark_dsg/mesh.h>

#include <array>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>

namespace hydra {

// Group each sample with an existing representative of the same label within
// tolerance. Representatives keep their original positions and remain separate.
class MeshSampleGrouping {
 public:
  // A zero tolerance uses exact position/label equality.
  explicit MeshSampleGrouping(double tolerance, size_t expected_vertices = 0);

  // Returns a dense representative index in order of first observation.
  size_t add(const Eigen::Vector3f& point, uint32_t label = 0);

 private:
  using Cell = std::array<int64_t, 4>;
  using ExactKey = std::tuple<uint32_t, float, float, float>;
  struct CellHash {
    size_t operator()(const Cell& cell) const;
  };

  Cell cellFor(const Eigen::Vector3f& point, uint32_t label) const;
  std::optional<size_t> find(const Cell& cell, const Eigen::Vector3f& point) const;

  const double tolerance_;
  const double radius_squared_;
  std::vector<Eigen::Vector3f> representatives_;
  std::unordered_map<Cell, std::vector<size_t>, CellHash> cells_;
  std::map<ExactKey, size_t> exact_;
};

}  // namespace hydra
