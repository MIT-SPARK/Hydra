#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_pgmo/KimeraPgmo.h>
#include <ros/ros.h>

#include <map>
#include <memory>

namespace kimera {

class DsgBackend {
 public:
  DsgBackend(const ros::NodeHandle nh,
             const std::map<LayerId, char>& layer_id_map,
             LayerId mesh_layer_id = 1);

  ~DsgBackend() = default;

  DsgBackend(const DsgBackend& other) = delete;

  DsgBackend& operator=(const DsgBackend& other) = delete;

  inline bool initialized() const { return initialized_; }

 private:
  void addLoopClosure();

  void addPlacesToDeformationGraph();

  void optimize();

  void deformGraph();

  ros::NodeHandle nh_;

  bool initialized_;
  std::map<LayerId, char> layer_id_map_;

  DynamicSceneGraph::Ptr graph_;
  std::unique_ptr<kimera_pgmo::KimeraPgmo> pgmo_;
};

}  // namespace kimera
