#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <kimera_dsg/scene_graph_layer.h>

namespace kimera {

template <typename CloudType>
struct PclLayer {
  typename CloudType::Ptr cloud;
  std::map<size_t, NodeId> cloud_to_layer_ids;
};

template <typename CloudType>
PclLayer<CloudType> convertLayerToPcl(const SceneGraphLayer& layer) {
  PclLayer<CloudType> to_return;
  to_return.cloud.reset(new CloudType());
  to_return.cloud->resize(layer.numNodes());

  size_t next_point_id = 0u;
  for (const auto& id_node_pair : layer.nodes) {
    Eigen::Vector3d position = id_node_pair.second->attributes().position;
    typename CloudType::PointType point;
    point.x = position(0);
    point.y = position(1);
    point.z = position(2);
    to_return.cloud->at(next_point_id) = point;
    to_return.cloud_to_layer_ids[next_point_id] = id_node_pair.first;
    next_point_id++;
  }

  return to_return;
}

struct SubMesh {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertices;
  pcl::PolygonMesh::Ptr mesh;
  std::vector<size_t> vertex_map;
};

template <typename PointT>
Eigen::Vector3d toEigen(const PointT& point) {
  Eigen::Vector3d to_return;
  to_return << point.x, point.y, point.z;
  return to_return;
}

template <typename PointT>
Eigen::MatrixXd getNormals(const pcl::PolygonMesh& mesh, const pcl::PointCloud<PointT>& vertices) {
  Eigen::MatrixXd normals = Eigen::MatrixXd::Zero(3, vertices.size());
  for (const auto face : mesh.polygons) {
    Eigen::Vector3d p1 = toEigen(vertices.at(face.vertices.at(0)));
    Eigen::Vector3d p2 = toEigen(vertices.at(face.vertices.at(1)));
    Eigen::Vector3d p3 = toEigen(vertices.at(face.vertices.at(2)));
    Eigen::Vector3d v1 = p2 - p1;
    Eigen::Vector3d v2 = p3 - p1;
    Eigen::Vector3d normal = v1.cross(v2).normalized();
    normals.block<3, 1>(face.vertices.at(0), 0) += normal;
    normals.block<3, 1>(face.vertices.at(1), 0) += normal;
    normals.block<3, 1>(face.vertices.at(2), 0) += normal;
  }

  // do normal averaging
  for (int i = 0; i < normals.cols(); ++i) {
    normals.block<3, 1>(i, 0).normalize();
  }

  return normals;
}

}  // namespace kimera
