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
#pragma once
#include "hydra_utils/dsg_types.h"

#include <hydra_msgs/DsgUpdate.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>

namespace hydra {

class DsgSender {
 public:
  explicit DsgSender(const ros::NodeHandle& nh);

  void sendGraph(DynamicSceneGraph& graph, const ros::Time& stamp) const;

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

class DsgReceiver {
 public:
  using LogCallback = std::function<void(const ros::Time&, size_t)>;

  explicit DsgReceiver(const ros::NodeHandle& nh);

  DsgReceiver(const ros::NodeHandle& nh, const LogCallback& cb);

  inline DynamicSceneGraph::Ptr graph() const { return graph_; }

  inline bool updated() const { return has_update_; }

  inline void clearUpdated() { has_update_ = false; }

 private:
  void handleUpdate(const hydra_msgs::DsgUpdate::ConstPtr& msg);

  void handleMesh(const mesh_msgs::TriangleMeshStamped::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Subscriber mesh_sub_;

  bool has_update_;
  DynamicSceneGraph::Ptr graph_;
  std::unique_ptr<pcl::PolygonMesh> mesh_;

  std::unique_ptr<LogCallback> log_callback_;
};

}  // namespace hydra
