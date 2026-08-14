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
#include <hydra/places/gvd_places/gvd_graph.h>
#include <hydra/places/gvd_places/gvd_merge_policies.h>

namespace hydra::places {

TEST(GvdMergePolicies, BasisPolicyCorrect) {
  GvdMemberInfo info1;
  info1.num_basis_points = 1;
  info1.distance = 0.3;
  GvdMemberInfo info2;
  info2.num_basis_points = 3;
  info2.distance = 0.1;
  GvdMemberInfo info3;
  info3.num_basis_points = 3;
  info3.distance = 0.3;

  BasisPointMergePolicy basis_policy;
  EXPECT_EQ(basis_policy.compare(info1, info2), -1);
  EXPECT_EQ(basis_policy.compare(info1, info3), -1);
  EXPECT_EQ(basis_policy.compare(info2, info3), 0);
  EXPECT_EQ(basis_policy.compare(info2, info1), 1);
  EXPECT_EQ(basis_policy.compare(info3, info1), 1);
  EXPECT_EQ(basis_policy.compare(info3, info2), 0);
}

TEST(GvdMergePolicies, DistancePolicyCorrect) {
  GvdMemberInfo info1;
  info1.num_basis_points = 1;
  info1.distance = 0.3;
  GvdMemberInfo info2;
  info2.num_basis_points = 3;
  info2.distance = 0.1;
  GvdMemberInfo info3;
  info3.num_basis_points = 3;
  info3.distance = 0.3;

  DistanceMergePolicy distance_policy;
  EXPECT_EQ(distance_policy.compare(info1, info2), 1);
  EXPECT_EQ(distance_policy.compare(info1, info3), 0);
  EXPECT_EQ(distance_policy.compare(info2, info3), -1);
  EXPECT_EQ(distance_policy.compare(info2, info1), -1);
  EXPECT_EQ(distance_policy.compare(info3, info1), 0);
  EXPECT_EQ(distance_policy.compare(info3, info2), 1);
}

}  // namespace hydra::places
