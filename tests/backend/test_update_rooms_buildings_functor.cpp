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
#include <hydra/backend/update_rooms_buildings_functor.h>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {

TEST(UpdateRoomsBuildingsFunctor, BuildingUpdate) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  graph.emplaceNode(DsgLayers::BUILDINGS,
                    "B0"_id,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 2.0, 3.0)));

  graph.emplaceNode(DsgLayers::ROOMS,
                    3,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(DsgLayers::ROOMS,
                    4,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(DsgLayers::ROOMS,
                    5,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));

  graph.insertEdge("B0"_id, 3);
  graph.insertEdge("B0"_id, 4);
  graph.insertEdge("B0"_id, 5);

  UpdateInfo::ConstPtr info(new UpdateInfo{nullptr, nullptr, false, 0, false, {}});
  UpdateBuildingsFunctor functor(Color(), 0);
  const auto unmerged = dsg->graph->clone();
  functor.call(*unmerged, *dsg, info);

  Eigen::Vector3d first_expected(-1.0, 0.0, 1.0);
  Eigen::Vector3d first_result = graph.getPosition("B0"_id);
  EXPECT_NEAR(0.0, (first_expected - first_result).norm(), 1.0e-7);
}

}  // namespace hydra
