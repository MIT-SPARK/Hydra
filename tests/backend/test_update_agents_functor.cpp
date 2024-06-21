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
#include <gtsam/geometry/Pose3.h>
#include <hydra/backend/update_agents_functor.h>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {

TEST(UpdateAgentsFunctor, AgentUpdate) {
  const LayerId agent_layer = DsgLayers::AGENTS;
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('a', 0));
    graph.emplaceNode(agent_layer, 'a', std::chrono::seconds(1), std::move(attrs));
  }
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('a', 5));
    graph.emplaceNode(agent_layer, 'a', std::chrono::seconds(2), std::move(attrs));
  }
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('c', 5));
    graph.emplaceNode(agent_layer, 'b', std::chrono::seconds(2), std::move(attrs));
  }

  gtsam::Values agent_values;
  agent_values.insert(
      NodeSymbol('a', 0),
      gtsam::Pose3(gtsam::Rot3(0.0, 1.0, 0.0, 0.0), gtsam::Point3(4.0, 5.0, 6.0)));
  agent_values.insert(
      NodeSymbol('a', 5),
      gtsam::Pose3(gtsam::Rot3(0.0, 0.0, 1.0, 0.0), gtsam::Point3(7.0, 8.0, 9.0)));
  // unused
  agent_values.insert(
      NodeSymbol('b', 5),
      gtsam::Pose3(gtsam::Rot3(0.0, 0.0, 1.0, 0.0), gtsam::Point3(7.0, 8.0, 9.0)));

  UpdateInfo::ConstPtr info(
      new UpdateInfo{nullptr, nullptr, false, 0, false, {}, &agent_values});
  UpdateAgentsFunctor functor;
  functor.call(*dsg->graph, *dsg, info);

  {  // external_key == node_id and in values
    const auto& attrs =
        graph.getNode(NodeSymbol('a', 0)).attributes<AgentNodeAttributes>();
    Eigen::Vector3d expected_pos(4.0, 5.0, 6.0);
    Eigen::Quaterniond expected_rot(0.0, 1.0, 0.0, 0.0);
    EXPECT_NEAR(0.0, (attrs.position - expected_pos).norm(), 1.0e-7);
    EXPECT_NEAR(0.0, attrs.world_R_body.angularDistance(expected_rot), 1.0e-7);
  }

  {  // external_key != node_id, but in values
    const auto& attrs =
        graph.getNode(NodeSymbol('a', 1)).attributes<AgentNodeAttributes>();
    Eigen::Vector3d expected_pos(7.0, 8.0, 9.0);
    Eigen::Quaterniond expected_rot(0.0, 0.0, 1.0, 0.0);
    EXPECT_NEAR(0.0, (attrs.position - expected_pos).norm(), 1.0e-7);
    EXPECT_NEAR(0.0, attrs.world_R_body.angularDistance(expected_rot), 1.0e-7);
  }

  {  // missing key in values, so doesn't get updated
    const auto& attrs =
        graph.getNode(NodeSymbol('b', 0)).attributes<AgentNodeAttributes>();
    Eigen::Vector3d expected_pos(1.0, 2.0, 3.0);
    Eigen::Quaterniond expected_rot(1.0, 0.0, 0.0, 0.0);
    EXPECT_NEAR(0.0, (attrs.position - expected_pos).norm(), 1.0e-7);
    EXPECT_NEAR(0.0, attrs.world_R_body.angularDistance(expected_rot), 1.0e-7);
  }
}

}  // namespace hydra
