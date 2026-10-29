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

#include "hydra/frontend/keyframe_gate.h"

namespace hydra {

TEST(KeyframeGate, FirstCallAlwaysTriggers) {
  KeyframeGate gate({0.5, 15.0});
  EXPECT_TRUE(
      gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity()));
}

TEST(KeyframeGate, SmallMotionDoesNotTrigger) {
  KeyframeGate gate({0.5, 15.0});
  gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  EXPECT_FALSE(
      gate.shouldTrigger(Eigen::Vector3d(0.1, 0, 0), Eigen::Quaterniond::Identity()));
}

TEST(KeyframeGate, TranslationOverThresholdTriggers) {
  KeyframeGate gate({0.5, 15.0});
  gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  EXPECT_TRUE(
      gate.shouldTrigger(Eigen::Vector3d(0.6, 0, 0), Eigen::Quaterniond::Identity()));
}

TEST(KeyframeGate, RotationOverThresholdTriggers) {
  KeyframeGate gate({10.0, 15.0});  // large translation thresh so only rotation matters
  gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  const Eigen::Quaterniond r(
      Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), r));
}

TEST(KeyframeGate, StateAdvancesOnlyOnTrigger) {
  KeyframeGate gate({0.5, 90.0});
  gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  // 0.3 then 0.3 again: neither individually >=0.5 from a non-advancing anchor,
  // but cumulative from the origin anchor the second (0.6) must trigger.
  EXPECT_FALSE(
      gate.shouldTrigger(Eigen::Vector3d(0.3, 0, 0), Eigen::Quaterniond::Identity()));
  EXPECT_TRUE(
      gate.shouldTrigger(Eigen::Vector3d(0.6, 0, 0), Eigen::Quaterniond::Identity()));
}

}  // namespace hydra
