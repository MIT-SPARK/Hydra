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
#include <hydra/places/gvd_thinning.h>

namespace hydra {
namespace places {

struct InputOutputPair {
  std::bitset<26> neighborhood;
  bool no_euler_change;
  bool is_simple_point;
};

const InputOutputPair THINNING_TEST_CASES[] = {
    {0b000'000'000'000'00'000'000'000'000, false, true},
    {0b000'000'000'111'11'100'000'000'011, false, true},
    //{0b000'010'000'000'00'000'010'101'010, true, false},
};

std::ostream& operator<<(std::ostream& out, const InputOutputPair& pair) {
  out << std::endl
      << std::boolalpha << "simple: " << pair.is_simple_point
      << ", no euler delta: " << pair.no_euler_change;
  for (size_t z = 0; z < 3; ++z) {
    out << std::endl << "[";
    for (size_t r = 0; r < 3; ++r) {
      out << "[";
      for (size_t c = 0; c < 3; ++c) {
        size_t index = 9 * z + 3 * r + c;
        if (index == 13) {
          out << "x, ";
          continue;
        }

        if (index > 13) {
          index -= 1;
        }

        out << (pair.neighborhood[index] ? "1" : "0");
        if (c != 2) {
          out << ", ";
        } else {
          out << "]";
        }
      }
      if (r != 2) {
        out << ", ";
      } else {
        out << "]";
      }
    }
  }
  return out;
}

struct GvdThinningFixture : public testing::TestWithParam<InputOutputPair> {
  GvdThinningFixture() = default;

  virtual ~GvdThinningFixture() = default;
};

TEST_P(GvdThinningFixture, EulerChangeCorrect) {
  const auto test_pair = GetParam();
  EXPECT_EQ(test_pair.no_euler_change, noEulerChange(test_pair.neighborhood));
}

TEST_P(GvdThinningFixture, SimplePointCorrect) {
  const auto test_pair = GetParam();
  EXPECT_EQ(test_pair.is_simple_point, isSimplePoint(test_pair.neighborhood));
}

INSTANTIATE_TEST_CASE_P(GvdThinning,
                        GvdThinningFixture,
                        testing::ValuesIn(THINNING_TEST_CASES));

}  // namespace places
}  // namespace hydra
