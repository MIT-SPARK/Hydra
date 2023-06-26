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
#include <hydra/rooms/room_utilities.h>

namespace hydra {

namespace {

void addNode(IsolatedSceneGraphLayer& layer,
             size_t node_id,
             const Eigen::Vector3d& position,
             double distance) {
  auto attrs = std::make_unique<PlaceNodeAttributes>();
  attrs->position = position;
  attrs->distance = distance;
  layer.emplaceNode(node_id, std::move(attrs));
}

}  // namespace

TEST(RoomHelpersTests, TestGetRoomPosition) {
  IsolatedSceneGraphLayer places(DsgLayers::PLACES);
  addNode(places, 0, Eigen::Vector3d(1.0, 2.0, 3.0), 1.0);
  addNode(places, 1, Eigen::Vector3d(1.5, 0.0, 0.0), 0.1);
  addNode(places, 2, Eigen::Vector3d(1.0, 0.0, 0.0), 1.0);
  addNode(places, 3, Eigen::Vector3d(10.0, 0.0, 0.0), 1.0);

  {  // empty cluster -> origin
    std::unordered_set<NodeId> cluster;
    Eigen::Vector3d result = getRoomPosition(places, cluster);
    Eigen::Vector3d expected = Eigen::Vector3d::Zero();
    EXPECT_NEAR((result - expected).norm(), 0.0, 1.0e-9);
  }

  {  // single node -> same position as place node
    std::unordered_set<NodeId> cluster{0};
    Eigen::Vector3d result = getRoomPosition(places, cluster);
    Eigen::Vector3d expected(1.0, 2.0, 3.0);
    EXPECT_NEAR((result - expected).norm(), 0.0, 1.0e-9);
  }

  {  // centroid outside freespace -> projected to boundary
    std::unordered_set<NodeId> cluster{1, 2, 3};
    Eigen::Vector3d result = getRoomPosition(places, cluster);
    Eigen::Vector3d expected(2.0, 0.0, 0.0);
    EXPECT_NEAR((result - expected).norm(), 0.0, 1.0e-9);
  }
}

TEST(RoomHelpersTests, TestAddRoomEdges) {
  IsolatedSceneGraphLayer places(DsgLayers::PLACES);
  for (size_t i = 0; i < 10; ++i) {
    places.emplaceNode(i, std::make_unique<NodeAttributes>());
  }
  places.insertEdge(0, 1);  // connects rooms 0 and 1
  places.insertEdge(5, 7);  // forces a sibling not being in a cluster
  places.insertEdge(6, 4);  // forces a sibling not being in a room
  places.insertEdge(0, 6);  // forces siblings with the same room

  IsolatedSceneGraphLayer rooms(DsgLayers::ROOMS);
  rooms.emplaceNode(0, std::make_unique<NodeAttributes>());
  rooms.emplaceNode(1, std::make_unique<NodeAttributes>());
  rooms.emplaceNode(2, std::make_unique<NodeAttributes>());

  std::map<NodeId, size_t> labels{
      {0, 0}, {1, 1}, {2, 5}, {3, 3}, {4, 5}, {5, 0}, {6, 0}};
  std::map<size_t, NodeId> label_map{{0, 0}, {1, 1}, {3, 2}};

  addEdgesToRoomLayer(places, labels, label_map, rooms);
  EXPECT_TRUE(rooms.hasEdge(0, 1));
  EXPECT_FALSE(rooms.hasEdge(0, 2));
  EXPECT_FALSE(rooms.hasEdge(1, 2));
}

}  // namespace hydra
