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
#include <ros/package.h>

#include "hydra/gnn/gnn_interface.h"

namespace hydra::gnn {

TEST(GnnInterfaceTests, TestSimpleNetwork) {
  std::string package_path = ros::package::getPath("hydra");
  std::string model_path = package_path + "/src/gnn/tests/resources/simple_model.onnx";

  GnnInterface model(model_path, {{"output", {0}}});

  Tensor x(5, 2);
  auto x_map = x.map<float>();
  for (int i = 0; i < x.rows(); ++i) {
    x_map(i, 0) = i + 1;
    x_map(i, 1) = i + 1;
  }

  Tensor edge_index(2, 4, Tensor::Type::INT64);
  auto index_map = edge_index.map<int64_t>();
  for (int i = 0; i < edge_index.cols(); ++i) {
    index_map(0, i) = i;
    index_map(1, i) = i + 1;
  }

  TensorMap inputs{{"x", x}, {"edge_index", edge_index}};
  auto y = model(inputs, {5}).at("output");
  EXPECT_TRUE(y);

  auto y_map = y.map<float>();

  Eigen::MatrixXf expected(5, 2);
  expected(0, 0) = 1.0f;
  expected(0, 1) = 1.0f;
  expected(1, 0) = 3.0f;
  expected(1, 1) = 3.0f;
  expected(2, 0) = 5.0f;
  expected(2, 1) = 5.0f;
  expected(3, 0) = 7.0f;
  expected(3, 1) = 7.0f;
  expected(4, 0) = 9.0f;
  expected(4, 1) = 9.0f;

  Eigen::MatrixXf result = expected - y_map;
  double diff = (expected - y_map).norm();
  EXPECT_NEAR(diff, 0.0, 1.0e-9);
}

}  // namespace::hydra
