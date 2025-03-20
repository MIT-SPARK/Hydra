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
#include <hydra/reconstruction/integration_masking.h>

#include <opencv2/core.hpp>

namespace hydra {

TEST(IntegrationMasking, InitCorrect) {
  {  // empty mask and input means empty output
    cv::Mat input;
    cv::Mat mask;
    EXPECT_TRUE(maskNonZero(input, mask));
    EXPECT_TRUE(mask.empty());
  }

  {  // non-empty input -> non-empty output
    cv::Mat input = cv::Mat::zeros(4, 3, CV_8UC1);
    cv::Mat mask;
    EXPECT_TRUE(maskNonZero(input, mask));
    EXPECT_EQ(mask.rows, 4);
    EXPECT_EQ(mask.cols, 3);
    EXPECT_EQ(mask.type(), CV_32SC1);
  }

  {  // mismatch -> no masking performed
    cv::Mat input = cv::Mat::zeros(4, 3, CV_8UC1);
    cv::Mat mask(3, 4, CV_32SC1);
    EXPECT_FALSE(maskNonZero(input, mask));
    EXPECT_EQ(mask.rows, 3);
    EXPECT_EQ(mask.cols, 4);
    EXPECT_EQ(mask.type(), CV_32SC1);
  }

  {  // non-int32_t type -> conversion happens
    cv::Mat input = cv::Mat::zeros(4, 3, CV_8UC1);
    cv::Mat mask(4, 3, CV_8UC1);
    EXPECT_TRUE(maskNonZero(input, mask));
    EXPECT_EQ(mask.rows, 4);
    EXPECT_EQ(mask.cols, 3);
    EXPECT_EQ(mask.type(), CV_32SC1);
  }
}

TEST(IntegrationMasking, SemanticsCorrect) {
  {  // empty labels, empty mask
    cv::Mat labels;
    cv::Mat mask;
    std::set<int32_t> to_mask{1, 2, 3};
    EXPECT_TRUE(maskInvalidSemantics(labels, to_mask, mask));
    EXPECT_TRUE(mask.empty());
  }

  {  // no labels to mask, empty mask
    cv::Mat labels = cv::Mat::zeros(4, 3, CV_32SC1);
    cv::Mat mask;
    std::set<int32_t> to_mask;
    EXPECT_TRUE(maskInvalidSemantics(labels, to_mask, mask));
    EXPECT_TRUE(mask.empty());
  }

  {  // invalid label image, empty mask
    cv::Mat labels = cv::Mat::zeros(4, 3, CV_8UC1);
    cv::Mat mask;
    std::set<int32_t> to_mask{1, 2, 3};
    EXPECT_FALSE(maskInvalidSemantics(labels, to_mask, mask));
    EXPECT_TRUE(mask.empty());
  }

  {  // valid input -> correct output
    cv::Mat labels = cv::Mat::zeros(4, 3, CV_32SC1);
    labels.at<int32_t>(0, 1) = 1;
    labels.at<int32_t>(1, 2) = 2;
    labels.at<int32_t>(2, 1) = 4;
    labels.at<int32_t>(3, 2) = 3;

    cv::Mat mask;
    std::set<int32_t> to_mask{1, 2, 3};
    EXPECT_TRUE(maskInvalidSemantics(labels, to_mask, mask));
    EXPECT_EQ(cv::countNonZero(mask), 3);
    EXPECT_EQ(mask.at<int32_t>(0, 1), 1);
    EXPECT_EQ(mask.at<int32_t>(1, 2), 1);
    EXPECT_EQ(mask.at<int32_t>(3, 2), 1);
  }
}

TEST(IntegrationMasking, NonZeroCorrect) {
  {  // non integer model, empty mask
    cv::Mat input = cv::Mat::zeros(4, 3, CV_32FC1);
    cv::Mat mask;
    EXPECT_FALSE(maskNonZero(input, mask));
    EXPECT_TRUE(mask.empty());
  }

  {  // valid input -> correct output
    cv::Mat input = cv::Mat::zeros(4, 3, CV_8SC1);
    input.at<int8_t>(0, 1) = 1;
    input.at<int8_t>(1, 2) = 2;
    input.at<int8_t>(2, 1) = 4;
    input.at<int8_t>(3, 2) = 3;

    cv::Mat mask;
    EXPECT_TRUE(maskNonZero(input, mask));
    EXPECT_EQ(cv::countNonZero(mask), 4);
    EXPECT_EQ(mask.at<int32_t>(0, 1), 1);
    EXPECT_EQ(mask.at<int32_t>(1, 2), 1);
    EXPECT_EQ(mask.at<int32_t>(2, 1), 1);
    EXPECT_EQ(mask.at<int32_t>(3, 2), 1);
  }
}

}  // namespace hydra
