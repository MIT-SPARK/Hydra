/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <hydra/backend/backend_module.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "hydra_ros/config/ros_utilities.h"
#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

using kimera_pgmo::KimeraPgmoMesh;
using pose_graph_tools::PoseGraph;

class RosBackend : public BackendModule {
 public:
  using Policy =
      message_filters::sync_policies::ApproximateTime<KimeraPgmoMesh, PoseGraph>;
  using Sync = message_filters::Synchronizer<Policy>;

  RosBackend(const ros::NodeHandle& nh,
             const RobotPrefixConfig& prefix,
             const SharedDsgInfo::Ptr& dsg,
             const SharedDsgInfo::Ptr& backend_dsg,
             const SharedModuleState::Ptr& state);

  ~RosBackend();

  void inputCallback(const kimera_pgmo::KimeraPgmoMesh::ConstPtr& mesh,
                     const pose_graph_tools::PoseGraph::ConstPtr& deformation_graph);

  void poseGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

 protected:
  void publishOutputs(const pcl::PolygonMesh& mesh, size_t timestamp_ns) const;

  void publishDeformationGraphViz() const;

  void publishPoseGraphViz() const;

  void publishUpdatedMesh(const pcl::PolygonMesh& mesh, size_t timestamp_ns) const;

 protected:
  ros::NodeHandle nh_;
  std::list<PoseGraph::ConstPtr> pose_graph_queue_;
  kimera_pgmo::KimeraPgmoMesh::ConstPtr latest_mesh_msg_;

  ros::Subscriber pose_graph_sub_;
  std::unique_ptr<message_filters::Subscriber<PoseGraph>> deformation_graph_sub_;
  std::unique_ptr<message_filters::Subscriber<KimeraPgmoMesh>> mesh_sub_;
  std::unique_ptr<Sync> sync_;
};

class RosBackendVisualizer {
 public:
  RosBackendVisualizer(const ros::NodeHandle& nh,
                       const BackendConfig& config,
                       const RobotPrefixConfig& prefix);

  virtual ~RosBackendVisualizer() = default;

  void publishOutputs(const DynamicSceneGraph& graph,
                      const kimera_pgmo::DeformationGraph& dgraph,
                      size_t timestamp_ns);

 protected:
  virtual void publishPoseGraph(const DynamicSceneGraph& graph,
                                const kimera_pgmo::DeformationGraph& dgraph) const;

  virtual void publishDeformationGraphViz(const kimera_pgmo::DeformationGraph& dgraph,
                                          size_t timestamp_ns) const;

 protected:
  ros::NodeHandle nh_;
  BackendConfig config_;
  RobotPrefixConfig prefix_;

  ros::Publisher mesh_mesh_edges_pub_;
  ros::Publisher pose_mesh_edges_pub_;
  ros::Publisher pose_graph_pub_;

  // Hack for temporary removal of label flickering
  size_t last_zmq_pub_time_;

  std::unique_ptr<spark_dsg::ZmqSender> zmq_sender_;
  std::unique_ptr<DsgSender> dsg_sender_;
};

}  // namespace hydra
