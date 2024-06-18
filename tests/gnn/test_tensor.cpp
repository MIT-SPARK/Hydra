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

#include "hydra/gnn/tensor.h"

namespace hydra::gnn {

TEST(GnnTensorTests, TestEmptyConstructorInvariants) {
  {  // test case 1: default tensor
    Tensor tensor;
    EXPECT_TRUE(!tensor);
    EXPECT_EQ(tensor.type(), Tensor::Type::FLOAT32);
    EXPECT_EQ(tensor.size(), 0u);
    EXPECT_EQ(tensor.num_bytes(), 0u);
    EXPECT_EQ(tensor.dims(), std::vector<int64_t>());
    EXPECT_EQ(tensor.num_dims(), 0u);
    EXPECT_EQ(tensor.rows(), 0);
    EXPECT_EQ(tensor.cols(), 0);
  }

  {  // test case 2: custom type
    Tensor tensor(Tensor::Type::INT64);
    EXPECT_TRUE(!tensor);
    EXPECT_EQ(tensor.type(), Tensor::Type::INT64);
    EXPECT_EQ(tensor.size(), 0u);
    EXPECT_EQ(tensor.num_bytes(), 0u);
    EXPECT_EQ(tensor.dims(), std::vector<int64_t>());
    EXPECT_EQ(tensor.num_dims(), 0u);
    EXPECT_EQ(tensor.rows(), 0);
    EXPECT_EQ(tensor.cols(), 0);
  }
}

TEST(GnnTensorTests, TestRowColConstructorInvariants) {
  {  // test case 1: default tensor
    Tensor tensor(5, 1);
    EXPECT_TRUE(tensor);
    EXPECT_EQ(tensor.type(), Tensor::Type::FLOAT32);
    EXPECT_EQ(tensor.size(), 5u);
    EXPECT_EQ(tensor.num_bytes(), 20u);
    std::vector<int64_t> expected_dims{5, 1};
    EXPECT_EQ(tensor.dims(), expected_dims);
    EXPECT_EQ(tensor.num_dims(), 2u);
    EXPECT_EQ(tensor.rows(), 5);
    EXPECT_EQ(tensor.cols(), 1);
  }

  {  // test case 2: custom type
    Tensor tensor(1, 5, Tensor::Type::INT64);
    EXPECT_TRUE(tensor);
    EXPECT_EQ(tensor.type(), Tensor::Type::INT64);
    EXPECT_EQ(tensor.size(), 5u);
    EXPECT_EQ(tensor.num_bytes(), 40u);
    std::vector<int64_t> expected_dims{1, 5};
    EXPECT_EQ(tensor.dims(), expected_dims);
    EXPECT_EQ(tensor.num_dims(), 2u);
    EXPECT_EQ(tensor.rows(), 1);
    EXPECT_EQ(tensor.cols(), 5);
  }
}

TEST(GnnTensorTests, AbitraryConstructorInvariants) {
  {  // test case 1: row vector
    Tensor tensor({5}, Tensor::Type::FLOAT32);
    EXPECT_TRUE(tensor);
    EXPECT_EQ(tensor.type(), Tensor::Type::FLOAT32);
    EXPECT_EQ(tensor.size(), 5u);
    EXPECT_EQ(tensor.num_bytes(), 20u);
    std::vector<int64_t> expected_dims{5};
    EXPECT_EQ(tensor.dims(), expected_dims);
    EXPECT_EQ(tensor.num_dims(), 1u);
    EXPECT_EQ(tensor.rows(), 5);
    EXPECT_EQ(tensor.cols(), 1);
  }

  {  // test case 2: col vector
    Tensor tensor({1, 5}, Tensor::Type::INT64);
    EXPECT_TRUE(tensor);
    EXPECT_EQ(tensor.type(), Tensor::Type::INT64);
    EXPECT_EQ(tensor.size(), 5u);
    EXPECT_EQ(tensor.num_bytes(), 40u);
    std::vector<int64_t> expected_dims{1, 5};
    EXPECT_EQ(tensor.dims(), expected_dims);
    EXPECT_EQ(tensor.num_dims(), 2u);
    EXPECT_EQ(tensor.rows(), 1);
    EXPECT_EQ(tensor.cols(), 5);
  }

  {  // test case 2: multi-dimensional vector
    Tensor tensor({3, 5, 2}, Tensor::Type::INT16);
    EXPECT_TRUE(tensor);
    EXPECT_EQ(tensor.type(), Tensor::Type::INT16);
    EXPECT_EQ(tensor.size(), 30u);
    EXPECT_EQ(tensor.num_bytes(), 60u);
    std::vector<int64_t> expected_dims{3, 5, 2};
    EXPECT_EQ(tensor.dims(), expected_dims);
    EXPECT_EQ(tensor.num_dims(), 3u);
    EXPECT_EQ(tensor.rows(), 3);
    EXPECT_EQ(tensor.cols(), 5);
  }
}

}  // namespace::hydra
