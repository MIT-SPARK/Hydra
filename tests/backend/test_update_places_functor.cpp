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
#include <hydra/backend/update_places_functor.h>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {

namespace {

MergeList callWithUnmerged(const UpdateFunctor& functor,
                           SharedDsgInfo& dsg,
                           const UpdateInfo::ConstPtr& info) {
  const auto unmerged = dsg.graph->clone();
  return functor.call(*unmerged, dsg, info);
}

}  // namespace

TEST(UpdatePlacesFunctor, PlaceUpdate) {
  const LayerId place_layer = DsgLayers::PLACES;
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  auto attrs1 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs1->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  graph.emplaceNode(place_layer, NodeSymbol('p', 0), std::move(attrs1));

  auto attrs2 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs2->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  graph.emplaceNode(place_layer, NodeSymbol('p', 5), std::move(attrs2));

  auto attrs3 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs3->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  attrs3->is_active = true;  // make sure it doesn't get dropped
  graph.emplaceNode(place_layer, NodeSymbol('p', 6), std::move(attrs3));

  gtsam::Values values;
  values.insert(NodeSymbol('p', 0),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert(NodeSymbol('p', 5),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));

  UpdateInfo::ConstPtr info(new UpdateInfo{&values, nullptr, true, 0, false, {}});
  UpdatePlacesFunctor functor(0.4, 0.3);
  callWithUnmerged(functor, *dsg, info);

  {  // first key exists: new value
    Eigen::Vector3d expected(4.0, 5.0, 6.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 0));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }

  {  // non-zero key exists: new value
    Eigen::Vector3d expected(7.0, 8.0, 9.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 5));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }

  {  // key doesn't exist: original value
    Eigen::Vector3d expected(1.0, 2.0, 3.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 6));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }
}

TEST(UpdatePlacesFunctor, PlaceUpdateNodeFinderBug) {
  const LayerId place_layer = DsgLayers::PLACES;
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  auto attrs1 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs1->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  graph.emplaceNode(place_layer, NodeSymbol('p', 0), std::move(attrs1));

  auto attrs2 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs2->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  graph.emplaceNode(place_layer, NodeSymbol('p', 5), std::move(attrs2));

  auto attrs3 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs3->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  attrs3->is_active = true;  // make sure it doesn't get dropped
  graph.emplaceNode(place_layer, NodeSymbol('p', 6), std::move(attrs3));

  gtsam::Values values;
  values.insert(NodeSymbol('p', 0),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert(NodeSymbol('p', 5),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));

  UpdateInfo::ConstPtr info(new UpdateInfo{&values, nullptr, false, 0, true, {}});
  UpdatePlacesFunctor functor(0.4, 0.3);
  // initialize node finder
  callWithUnmerged(functor, *dsg, info);

  // remove one archived node and unarchive the other
  graph.removeNode(NodeSymbol('p', 0));
  graph.getNode(NodeSymbol('p', 5)).attributes().is_active = true;
  callWithUnmerged(functor, *dsg, info);
}

TEST(UpdatePlacesFunctor, PlaceUpdateMerge) {
  const LayerId place_layer = DsgLayers::PLACES;
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  PlaceNodeAttributes::Ptr attrs0(new PlaceNodeAttributes);
  attrs0->position << 1.0, 2.0, 3.0;
  attrs0->distance = 1.0;
  attrs0->is_active = false;
  PlaceNodeAttributes::Ptr attrs5(new PlaceNodeAttributes);
  attrs5->position << 1.0, 2.0, 3.0;
  attrs5->distance = 1.0;
  attrs5->is_active = false;
  PlaceNodeAttributes::Ptr attrs6(new PlaceNodeAttributes);
  attrs6->position << 1.0, 2.0, 3.0;
  attrs6->distance = 1.0;
  attrs6->is_active = true;

  graph.emplaceNode(place_layer, NodeSymbol('p', 0), std::move(attrs0));
  graph.emplaceNode(place_layer, NodeSymbol('p', 5), std::move(attrs5));
  graph.emplaceNode(place_layer, NodeSymbol('p', 6), std::move(attrs6));

  gtsam::Values values;
  values.insert(NodeSymbol('p', 0),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert(NodeSymbol('p', 5),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));
  values.insert(NodeSymbol('p', 6),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));

  UpdateInfo::ConstPtr info(new UpdateInfo{&values, nullptr, true, 0, true, {}});
  UpdatePlacesFunctor functor(0.4, 0.3);
  const auto result_merges = callWithUnmerged(functor, *dsg, info);

  {  // first key exists: new value
    Eigen::Vector3d expected(4.0, 5.0, 6.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 0));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }

  {  // merge target key exists: new value
    Eigen::Vector3d expected(7.0, 8.0, 9.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 5));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }

  // node p6 proposed for merge with node p5
  MergeList expected{{"p6"_id, "p5"_id}};
  EXPECT_EQ(result_merges, expected);
}

}  // namespace hydra
