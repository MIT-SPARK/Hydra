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
#include <gtest/gtest.h>
#include <hydra/backend/updates/update_frontiers_functor.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph.h>

using namespace spark_dsg;

namespace hydra {

TEST(UpdateFrontiersFunctor, CleanupUsesLatestPoseInRobotPartition) {
  SharedDsgInfo dsg({});
  auto& graph = *dsg.graph;
  ASSERT_TRUE(graph.emplaceNode(
      DsgLayers::OBJECTS, "O0"_id, std::make_unique<ObjectNodeAttributes>()));

  const auto rotation = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(graph.emplaceNode(
      2,
      "a8"_id,
      std::make_unique<AgentNodeAttributes>(
          std::chrono::seconds(3), rotation, Eigen::Vector3d(10, 0, 0), "a8"_id),
      'a'));
  ASSERT_TRUE(graph.emplaceNode(
      2,
      "b2"_id,
      std::make_unique<AgentNodeAttributes>(
          std::chrono::seconds(1), rotation, Eigen::Vector3d(5, 0, 0), "b2"_id),
      'b'));
  ASSERT_TRUE(graph.emplaceNode(
      2,
      "b5"_id,
      std::make_unique<AgentNodeAttributes>(
          std::chrono::seconds(2), rotation, Eigen::Vector3d::Zero(), "b5"_id),
      'b'));

  auto frontier = std::make_unique<FrontierNodeAttributes>();
  frontier->real_place = false;
  frontier->position = Eigen::Vector3d(0.5, 0, 0);
  ASSERT_TRUE(graph.emplaceNode(DsgLayers::PLACES, "f0"_id, std::move(frontier)));

  auto unmerged = graph.clone();
  auto info = std::make_shared<UpdateInfo>();
  info->robot_id = 1;
  const UpdateFrontiersFunctor functor({});
  functor.hooks().cleanup(info, *unmerged, &dsg);

  EXPECT_TRUE(
      graph.getNode("f0"_id).attributes<FrontierNodeAttributes>().anti_frontier);
}

}  // namespace hydra
