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
#include <hydra/places/graph_extractor_utilities.h>

namespace hydra::places {

TEST(GraphExtractionUtilities, TestBresenhamLine) {
  {  // x primary axis
    GlobalIndex start(1, 2, 3);
    GlobalIndex end(6, 4, 5);
    GlobalIndices expected(4);
    expected[0] << 2, 2, 3;
    expected[1] << 3, 3, 4;
    expected[2] << 4, 3, 4;
    expected[3] << 5, 4, 5;

    GlobalIndices result = makeBresenhamLine(start, end);
    ASSERT_EQ(4u, result.size());
    for (size_t i = 0; i < 4u; i++) {
      EXPECT_EQ(expected[i], result[i]);
    }
  }

  {  // y primary axis
    GlobalIndex start(2, 1, 3);
    GlobalIndex end(4, 6, 5);
    GlobalIndices expected(4);
    expected[0] << 2, 2, 3;
    expected[1] << 3, 3, 4;
    expected[2] << 3, 4, 4;
    expected[3] << 4, 5, 5;

    GlobalIndices result = makeBresenhamLine(start, end);
    ASSERT_EQ(4u, result.size());
    for (size_t i = 0; i < 4u; i++) {
      EXPECT_EQ(expected[i], result[i]);
    }
  }

  {  // z primary axis
    GlobalIndex start(2, 3, 1);
    GlobalIndex end(4, 5, 6);
    GlobalIndices expected(4);
    expected[0] << 2, 3, 2;
    expected[1] << 3, 4, 3;
    expected[2] << 3, 4, 4;
    expected[3] << 4, 5, 5;

    GlobalIndices result = makeBresenhamLine(start, end);
    ASSERT_EQ(4u, result.size());
    for (size_t i = 0; i < 4u; i++) {
      EXPECT_EQ(expected[i], result[i]);
    }
  }

  {  // equal slope
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(6, 6, 6);
    GlobalIndices expected(4);
    expected[0] << 2, 2, 2;
    expected[1] << 3, 3, 3;
    expected[2] << 4, 4, 4;
    expected[3] << 5, 5, 5;

    GlobalIndices result = makeBresenhamLine(start, end);
    ASSERT_EQ(4u, result.size());
    for (size_t i = 0; i < 4u; i++) {
      EXPECT_EQ(expected[i], result[i]);
    }
  }
}

TEST(GraphExtractionUtilities, TestBresenhamLineEmpty) {
  {  // same start and end
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(1, 1, 1);
    EXPECT_TRUE(makeBresenhamLine(start, end).empty());
  }

  {  // one step away x
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(2, 1, 1);
    EXPECT_TRUE(makeBresenhamLine(start, end).empty());
  }

  {  // one step away y
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(1, 2, 1);
    EXPECT_TRUE(makeBresenhamLine(start, end).empty());
  }

  {  // one step away z
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(1, 1, 2);
    EXPECT_TRUE(makeBresenhamLine(start, end).empty());
  }
}

}  // namespace hydra::places
