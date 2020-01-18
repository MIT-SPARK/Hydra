#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <map>

#include <glog/logging.h>

#include <voxblox/core/layer.h>

#include <dynamic_reconfigure/server.h>
#include <kimera_scene_graph/kimera_scene_graphConfig.h>
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>

#include <kimera_semantics_ros/semantic_simulation_server.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_node.h"
#include "kimera_scene_graph/dynamic_scene_node.h"
#include "kimera_scene_graph/object_finder.h"
#include "kimera_scene_graph/room_finder.h"

namespace kimera {

typedef ColoredPointCloud SemanticPointCloud;
typedef std::unordered_map<SemanticLabel, SemanticPointCloud::Ptr> SemanticPointCloudMap;

class SceneGraphSimulationServer : public SemanticSimulationServer {
 private:
  typedef kimera_scene_graph::kimera_scene_graphConfig RqtSceneGraphConfig;

 public:
  SceneGraphSimulationServer(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
      : SemanticSimulationServer(nh, nh_private),
        room_finder_(nullptr),
        object_finder_(nullptr),
        scene_graph_(nh, nh_private),
        reconstruct_scene_graph_service_( ){
    // Build object finder
    int object_finder_type = 0;
    nh_private.param(
        "object_finder_type", object_finder_type, object_finder_type);
    object_finder_ = kimera::make_unique<ObjectFinder<ColorPoint>>(
        world_frame_, static_cast<ObjectFinderType>(object_finder_type));

    // Build room finder
    room_finder_ = kimera::make_unique<RoomFinder>();

    // ROS publishers.
    object_finder_pub_ = ros::Publisher(
        nh_private_.advertise<ColoredPointCloud>("object_centroids", 1, true));

    // Attach rqt reconfigure
    rqt_callback_ = boost::bind(
          &SceneGraphSimulationServer::rqtReconfigureCallback,
          this, _1, _2);
    rqt_server_.setCallback(rqt_callback_);

    // Add rosservice to reconstruct scene graph, this avoids having to
    // rebuild the simulation world and reintegrate the generated pointclouds.
    reconstruct_scene_graph_service_  =
        nh_private_.advertiseService(
          "reconstruct_scene_graph",
          &SceneGraphSimulationServer::sceneGraphReconstructionServiceCall,
          this);
  }

  bool sceneGraphReconstructionServiceCall(
      std_srvs::Empty::Request& request,
      std_srvs::Empty::Response& response) {
    LOG(INFO) << "Requested scene graph reconstruction.";
    sceneGraphReconstruction();
    return true;
  }

  void sceneGraphReconstruction() {
    // Also generate test mesh
    // TODO(Toni): this can be called from a rosservice now, so make sure
    // the sim world is ready before...
    // TODO(Toni): can't we load the mesh for now, this takes very long...
    vxb::MeshIntegratorConfig mesh_config;
    CHECK(tsdf_test_);
    vxb::MeshLayer::Ptr mesh_test(new vxb::MeshLayer(tsdf_test_->block_size()));
    vxb::MeshIntegrator<vxb::TsdfVoxel> mesh_integrator_test(
        mesh_config, tsdf_test_.get(), mesh_test.get());
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_test.generateMesh(only_mesh_updated_blocks,
                                      clear_updated_flag);

    // Get Semantic PCLs
    static const vxb::ColorMode& color_mode = vxb::ColorMode::kLambertColor;
    SemanticPointCloudMap semantic_pointclouds;
    getSemanticPointcloudsFromMesh(mesh_test, color_mode, &semantic_pointclouds);
    for (const auto& semantic_pcl_it: semantic_pointclouds) {
      const SemanticLabel& semantic_label = semantic_pcl_it.first;
      const SemanticPointCloud::Ptr& semantic_pcl = semantic_pcl_it.second;
      const auto& pub_it = semantic_pcl_pubs_.find(semantic_label);
      if (pub_it == semantic_pcl_pubs_.end()) {
        semantic_pcl_pubs_[semantic_label] =
            nh_private_.advertise<SemanticPointCloud>(
            "semantic_pcl_" + std::to_string(semantic_label), 1, true);
      }
      // Publish semantic pointcloud
      semantic_pcl_pubs_.at(semantic_label).publish(*semantic_pcl);

      // Estimate objects in semantic pointcloud.
      // Only for those semantic labels that are not stuff (aka only for things).
      static constexpr int kSemanticStuffLabel = 0u;
      if (semantic_label != kSemanticStuffLabel) {
        LOG(INFO) << "Extracting objects for label: " << std::to_string(semantic_label);
        CHECK(object_finder_);
        Centroids centroids;
        ObjectPointClouds object_pcls;
        // TODO(Toni): do not publish right away the semantic pcl...
        object_finder_pub_.publish(
              object_finder_->findObjects(semantic_pcl, &centroids,
                                          &object_pcls));

        // Create semantic instance for each centroid
        InstanceId instance_id = 0;
        for(size_t idx = 0; idx < centroids.size(); ++idx) {
          // Create SceneNode out of centroids
          SceneNode semantic_instance;
          semantic_instance.attributes_.semantic_label_ = semantic_label;
          semantic_instance.attributes_.instance_id_ = instance_id;
          semantic_instance.id_ = std::to_string(semantic_label) +
              std::to_string(instance_id);
          ++instance_id;
          centroids.at(idx).get(semantic_instance.attributes_.position_);
          semantic_instance.attributes_.pcl_ = object_pcls.at(idx);

          // Add to database
          scene_graph_.addSceneNode(semantic_instance);
        }
      } else {
        LOG(INFO) << "Skipping object extraction for semantic label: "
                  << std::to_string(semantic_label);
      }
    }

    LOG(INFO) << "Publishing objects' centroids.";
    scene_graph_.visualize();
  }

  inline void getSemanticPointcloudsFromMesh(
      const vxb::MeshLayer::ConstPtr& mesh_layer,
      vxb::ColorMode color_mode,
      SemanticPointCloudMap* semantic_pointclouds) {
    CHECK_NOTNULL(semantic_pointclouds);
    semantic_pointclouds->clear();

    vxb::BlockIndexList mesh_indices;
    mesh_layer->getAllAllocatedMeshes(&mesh_indices);

    for (const vxb::BlockIndex& block_index : mesh_indices) {
      vxb::Mesh::ConstPtr mesh = mesh_layer->getMeshPtrByIndex(block_index);

      if (!mesh->hasVertices()) {
        continue;
      }

      // Check that we can actually do the color stuff.
      if (color_mode == vxb::kColor || color_mode == vxb::kLambertColor) {
        CHECK(mesh->hasColors());
      }
      if (color_mode == vxb::kNormals || color_mode == vxb::kLambert ||
          color_mode == vxb::kLambertColor) {
        CHECK(mesh->hasNormals());
      }

      for (size_t i = 0u; i < mesh->vertices.size(); i++) {
        ColorPoint point;
        point.x = mesh->vertices[i].x();
        point.y = mesh->vertices[i].y();
        point.z = mesh->vertices[i].z();

        vxb::Color color;
        vxb::colorMsgToVoxblox(getVertexColor(mesh, color_mode, i), &color);
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;

        // Add points that are red to centroid, very approx...
        const vxb::Color& original_color = mesh->colors[i];
        CHECK(semantic_config_.semantic_label_to_color_);
        const SemanticLabel& semantic_label =
            semantic_config_.semantic_label_to_color_->getSemanticLabelFromColor(
              HashableColor(original_color.r, original_color.g, original_color.b, 255u));
        if (semantic_pointclouds->find(semantic_label) ==
            semantic_pointclouds->end()) {
          // We have never seen this color (aka semantic label) before.
          // Store idx in vector of semantic poinclouds for the this new semantic
          // label.
          // Make new semantic pointcloud for the new semantic label.
          SemanticPointCloud::Ptr new_semantic_pcl =
              boost::make_shared<SemanticPointCloud>();
          new_semantic_pcl->header.frame_id = world_frame_;
          (*semantic_pointclouds)[semantic_label] = new_semantic_pcl;
        }
        // Get semantic pointcloud corresponding to the current semantic label.
        SemanticPointCloud::Ptr semantic_pcl =
            semantic_pointclouds->at(semantic_label);

        // Create and accumulate points
        semantic_pcl->push_back(point);
      }
    }
  }

  inline void getPointcloudFromMesh(
      const vxb::MeshLayer::ConstPtr& mesh_layer,
      vxb::ColorMode color_mode,
      ColoredPointCloud::Ptr pointcloud) {
    CHECK_NOTNULL(pointcloud);
    pointcloud->clear();

    vxb::BlockIndexList mesh_indices;
    mesh_layer->getAllAllocatedMeshes(&mesh_indices);

    // Testing: only one centroid...
    std::vector<std::pair<vxb::Color, pcl::CentroidPoint<ColorPoint>>>
        centroids(1);
    centroids.at(0).first = vxb::Color::Red();
    for (const vxb::BlockIndex& block_index : mesh_indices) {
      vxb::Mesh::ConstPtr mesh = mesh_layer->getMeshPtrByIndex(block_index);

      if (!mesh->hasVertices()) {
        continue;
      }
      // Check that we can actually do the color stuff.
      if (color_mode == vxb::kColor || color_mode == vxb::kLambertColor) {
        CHECK(mesh->hasColors());
      }
      if (color_mode == vxb::kNormals || color_mode == vxb::kLambert ||
          color_mode == vxb::kLambertColor) {
        CHECK(mesh->hasNormals());
      }

      for (size_t i = 0u; i < mesh->vertices.size(); i++) {
        ColorPoint point;
        point.x = mesh->vertices[i].x();
        point.y = mesh->vertices[i].y();
        point.z = mesh->vertices[i].z();

        vxb::Color color;
        vxb::colorMsgToVoxblox(getVertexColor(mesh, color_mode, i), &color);
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;

        // Add points that are red to centroid, very approx...
        const vxb::Color& original_color = mesh->colors[i];
        const vxb::Color& instance_color = centroids.at(0).first;
        if (original_color.r == instance_color.r &&
            original_color.g == instance_color.g &&
            original_color.b == instance_color.b) {
          // Add to centroids
          centroids.at(0).second.add(point);
        }

        // Create and accumulate points
        pointcloud->push_back(point);
      }
    }

    // Create intermediate semantic instance
    SceneNode semantic_instance;
    semantic_instance.id_ = "x1";
    semantic_instance.attributes_.semantic_label_ = 0;
    centroids.at(0).second.get(semantic_instance.attributes_.position_);
    // Overwrites...
    scene_graph_.addSceneNode(semantic_instance);
  }


private:
  void rqtReconfigureCallback(
      RqtSceneGraphConfig& config,
      uint32_t level) {
    object_finder_->updateClusterEstimator(
          static_cast<ObjectFinderType>(config.object_finder_type));

    EuclideanClusterEstimatorParams ec_params;
    ec_params.cluster_tolerance_ = config.cluster_tolerance;
    ec_params.max_cluster_size_ = config.ec_max_cluster_size;
    ec_params.min_cluster_size_ = config.ec_min_cluster_size;
    LOG(INFO) << ec_params.print();
    object_finder_->updateEuclideanClusterParams(ec_params);

    RegionGrowingClusterEstimatorParams rg_params;
    rg_params.curvature_threshold_ = config.curvature_threshold;
    rg_params.max_cluster_size_ =  config.rg_max_cluster_size;
    rg_params.min_cluster_size_ = config.rg_min_cluster_size;
    rg_params.normal_estimator_neighbour_size_ =  config.normal_estimator_neighbour_size;
    rg_params.number_of_neighbours_ = config.number_of_neighbours;
    rg_params.smoothness_threshold_ = config.smoothness_threshold;
    LOG(INFO) << rg_params.print();
    object_finder_->updateRegionGrowingParams(rg_params);

    LOG(WARNING) << "Clearing Scene Graph";
    scene_graph_.clear();

    LOG(INFO) << "Object Finder params have been updated. "
                 "Run scene graph reconstruction ros service to see the effects.";
}

 private:
  ros::Publisher object_finder_pub_;

  // Dynamically change params for scene graph reconstruction
  dynamic_reconfigure::Server<RqtSceneGraphConfig> rqt_server_;
  dynamic_reconfigure::Server<RqtSceneGraphConfig>::CallbackType rqt_callback_;

  // Dynamically request a scene graph reconstruction
  ros::ServiceServer reconstruct_scene_graph_service_;

  // Object finder
  std::unique_ptr<ObjectFinder<ColorPoint>> object_finder_;

  // Room finder
  std::unique_ptr<RoomFinder> room_finder_;

  // KimeraX
  SceneGraph scene_graph_;
};

}  // namespace kimera
