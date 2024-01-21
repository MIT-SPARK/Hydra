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
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <hydra/loop_closure/visual_registration.h>

#include "hydra_test/resources.h"
#include "hydra_test/visual_lcd_fixtures.h"

namespace hydra::lcd {

TEST(VisualRegistration, PoseRecoveryAccurate) {
  using namespace std::chrono_literals;
  const auto vocab_path =
      test::get_resource_path("loop_closure/visual/small_voc.yml.gz");
  auto factory = test::loadDBoW2Factory(vocab_path);
  auto camera = test::makeUHumans2Camera();
  auto data0 = test::loadFrameData(*camera, 0);
  auto data1 = test::loadFrameData(*camera, 1);

  DynamicSceneGraphNode node(0, 0, std::make_unique<NodeAttributes>(), 0ns);

  auto result0 = factory->construct(*camera, node, *data0);
  ASSERT_TRUE(result0.features != nullptr);
  auto result1 = factory->construct(*camera, node, *data1);
  ASSERT_TRUE(result1.features != nullptr);

  SensorRegistrationInput problem{0, result0.features, 0, result1.features};

  DynamicSceneGraph graph;
  TwoViewTeaserSolver::Config config;
  TwoViewTeaserSolver solver(config);
  auto result = solver.solve(graph, problem);
  ASSERT_TRUE(result.valid);

  const auto pose_path =
      test::get_resource_path("loop_closure/visual/image_poses.yaml");
  const auto poses = test::loadPoses(pose_path);
  ASSERT_EQ(poses.size(), 2u);
  const auto expected = poses[1].inverse() * poses[0];
  gtsam::Pose3 result_pose(gtsam::Rot3(result.to_R_from), result.to_p_from);
  EXPECT_TRUE(gtsam::assert_equal(expected, result_pose, 1.0e-1));
}

}  // namespace hydra::lcd
