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
#include <hydra/common/config_utilities.h>

namespace hydra {

using spark_dsg::LayerId;

TEST(HydraConversions, LayerMapConversion) {
  std::map<std::string, float> original{
      {"objects", 1.0}, {"places", 2.0}, {"rooms", 5.0}};
  std::map<LayerId, float> expected{{2, 1.0}, {3, 2.0}, {4, 5.0}};

  std::string err;
  std::map<LayerId, float> result;
  LayerMapConversion<float>::fromIntermediate(original, result, err);
  EXPECT_EQ(result, expected);
  EXPECT_TRUE(err.empty());
  err = "";

  const auto result_rt = LayerMapConversion<float>::toIntermediate(result, err);
  EXPECT_EQ(result, expected);
  EXPECT_TRUE(err.empty());
}

TEST(HydraConversions, QuaternionConversion) {
  std::map<std::string, double> valid{{"w", 0.5}, {"x", 0.5}, {"y", 0.0}, {"z", 0.0}};
  std::map<std::string, double> invalid{{"x", 0.5}, {"y", 0.0}, {"z", 0.0}};
  {  // test that conversion from dict works as expected
    std::string err;
    Eigen::Quaterniond result;
    QuaternionConverter::fromIntermediate(valid, result, err);
    EXPECT_TRUE(err.empty());
    EXPECT_TRUE(result.isApprox(Eigen::Quaterniond(0.5, 0.5, 0.0, 0.0)));
  }

  {  // test that we can't construct a quaternion without all info
    std::string err;
    Eigen::Quaterniond result;
    QuaternionConverter::fromIntermediate(invalid, result, err);
    EXPECT_FALSE(err.empty());
  }

  {  // test that conversion back to a dict makes sense
    Eigen::Quaterniond q(1.0, 2.0, 3.0, 4.0);
    std::string err;
    std::map<std::string, double> expected{
        {"w", 1.0}, {"x", 2.0}, {"y", 3.0}, {"z", 4.0}};
    const auto result = QuaternionConverter::toIntermediate(q, err);
    EXPECT_EQ(result, expected);
  }
}

}  // namespace hydra
