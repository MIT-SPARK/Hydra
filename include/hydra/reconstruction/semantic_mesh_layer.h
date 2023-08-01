#pragma once
#include <voxblox/mesh/mesh_layer.h>
#include <kimera_pgmo/utils/VoxbloxMeshInterface.h>

#include <memory>

namespace hydra {

class SemanticMeshLayer {
 public:
  using Ptr = std::shared_ptr<SemanticMeshLayer>;
  using SemanticMeshMap = voxblox::AnyIndexHashMapType<std::vector<uint32_t>>::type;

  explicit SemanticMeshLayer(voxblox::FloatingPoint block_size);

  voxblox::Mesh::Ptr allocateBlock(const voxblox::BlockIndex& index, bool use_semantics);

  void removeBlock(const voxblox::BlockIndex& index);

  voxblox::Mesh::Ptr getMeshBlock(const voxblox::BlockIndex& index) const;

  std::vector<uint32_t>* getSemanticBlock(const voxblox::BlockIndex& index) const;

  void getAllocatedBlockIndices(voxblox::BlockIndexList& allocated) const;

  size_t numBlocks() const;

  size_t getMemorySize() const;

  SemanticMeshLayer::Ptr getActiveMesh(const voxblox::IndexSet& archived_blocks);

  voxblox::MeshLayer::Ptr getVoxbloxMesh() const;

  kimera_pgmo::SemanticVoxbloxMeshInterface getMeshInterface() const;

 protected:
  voxblox::MeshLayer::Ptr mesh_;
  std::shared_ptr<SemanticMeshMap> semantics_;
};

}  // namespace hydra
