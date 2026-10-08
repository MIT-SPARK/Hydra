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
#include <hydra/utils/image_folder.h>

namespace hydra {

TEST(ImageFolder, PathUnderDirectory) {
  EXPECT_TRUE(utils::isPathUnder("/a/images/O_1", "/a/images"));
  EXPECT_TRUE(utils::isPathUnder("/a/images/O_1", "/a/images/"));
  EXPECT_FALSE(utils::isPathUnder("/a/images", "/a/images"));
  EXPECT_FALSE(utils::isPathUnder("/a/images2/O_1", "/a/images"));
  EXPECT_TRUE(utils::isPathUnder("images/temp/O_1", "images/temp"));
}

TEST(ImageFolder, BaseIsParentOfRoot) {
  EXPECT_EQ(utils::imageFolderBase("/run/images").string(), "/run");
  EXPECT_EQ(utils::imageFolderBase("/run/images/").string(), "/run");
  EXPECT_EQ(utils::imageFolderBase("/run/./images//").string(), "/run");
  EXPECT_EQ(utils::imageFolderBase("images").string(), "");
}

TEST(ImageFolder, ResolveRelativeAgainstParentOfRoot) {
  EXPECT_EQ(utils::resolveImageFolder("/run/images", "images/O_5").string(),
            "/run/images/O_5");
  EXPECT_EQ(utils::resolveImageFolder("/run/images/", "images/temp/O_3").string(),
            "/run/images/temp/O_3");
  EXPECT_EQ(utils::resolveImageFolder("/run/agents", "agents/agent_10").string(),
            "/run/agents/agent_10");
}

TEST(ImageFolder, ResolvePassesAbsoluteAndEmptyThrough) {
  EXPECT_EQ(utils::resolveImageFolder("/run/images", "/old/images/O_5").string(),
            "/old/images/O_5");
  EXPECT_EQ(utils::resolveImageFolder("/run/images", "").string(), "");
}

TEST(ImageFolder, RelativeToParentOfRoot) {
  EXPECT_EQ(utils::relativeImageFolder("/run/images", "/run/images/O_5"), "images/O_5");
  EXPECT_EQ(utils::relativeImageFolder("/run/images/", "/run/images/O_5"),
            "images/O_5");
  EXPECT_EQ(
      utils::relativeImageFolder("/run/subkeyframes", "/run/subkeyframes/subkf_7"),
      "subkeyframes/subkf_7");
  EXPECT_EQ(utils::relativeImageFolder("/run/images", "/run/./images/temp/../O_5"),
            "images/O_5");
}

TEST(ImageFolder, RelativeKeepsPathsOutsideBase) {
  EXPECT_EQ(utils::relativeImageFolder("/run/images", "/elsewhere/O_5"),
            "/elsewhere/O_5");
  // a relative root has no base, so relative paths are kept as is
  EXPECT_EQ(utils::relativeImageFolder("images", "images/O_5"), "images/O_5");
}

TEST(ImageFolder, RoundTrip) {
  const std::string root = "/run/images";
  const auto value = utils::relativeImageFolder(root, "/run/images/temp/O_3");
  EXPECT_EQ(value, "images/temp/O_3");
  EXPECT_EQ(utils::resolveImageFolder(root, value).string(), "/run/images/temp/O_3");
}

}  // namespace hydra
