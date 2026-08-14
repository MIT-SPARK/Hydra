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
#include <hydra_visualizer/utils/layer_key_selector.h>

namespace hydra {

TEST(LayerKeySelector, ParsingCorrect) {
  {  // normal layer
    LayerKeySelector expected{{3}, false, false};
    const auto result = LayerKeySelector::parse("3");
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, expected);
  }

  {  // partition
    LayerKeySelector expected{{3, 2}, false, false};
    const auto result = LayerKeySelector::parse("3p2");
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, expected);
  }

  {  // partitions >= 1
    LayerKeySelector expected{{3}, true, false};
    const auto result = LayerKeySelector::parse("3p*");
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, expected);
  }

  {  // partitions >= 0
    LayerKeySelector expected{{3}, true, true};
    const auto result = LayerKeySelector::parse("3*");
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, expected);
  }

  EXPECT_FALSE(LayerKeySelector::parse("3p"));
}

TEST(LayerKeySelector, MatchesCorrect) {
  {  // normal layer
    LayerKeySelector selector{{3}, false, false};
    EXPECT_FALSE(selector.matches({2}));
    EXPECT_TRUE(selector.matches({3}));
    EXPECT_FALSE(selector.matches({3, 1}));
    EXPECT_FALSE(selector.matches({3, 1}));
  }

  {  // partition
    LayerKeySelector selector{{3, 2}, false, false};
    EXPECT_FALSE(selector.matches({2}));
    EXPECT_FALSE(selector.matches({3}));
    EXPECT_FALSE(selector.matches({3, 1}));
    EXPECT_TRUE(selector.matches({3, 2}));
  }

  {  // any partition
    LayerKeySelector selector{{3}, true, false};
    EXPECT_FALSE(selector.matches({2}));
    EXPECT_FALSE(selector.matches({3}));
    EXPECT_TRUE(selector.matches({3, 1}));
    EXPECT_TRUE(selector.matches({3, 2}));
  }

  {  // any partition + default
    LayerKeySelector selector{{3}, true, true};
    EXPECT_FALSE(selector.matches({2}));
    EXPECT_TRUE(selector.matches({3}));
    EXPECT_TRUE(selector.matches({3, 1}));
    EXPECT_TRUE(selector.matches({3, 2}));
  }
}

}  // namespace hydra
