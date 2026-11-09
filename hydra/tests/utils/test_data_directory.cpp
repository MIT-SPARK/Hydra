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
#include <hydra/utils/data_directory.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

namespace hydra {

namespace {

std::string readFile(const std::filesystem::path& path) {
  std::ifstream fin(path, std::ios::binary);
  std::stringstream ss;
  ss << fin.rdbuf();
  return ss.str();
}

std::filesystem::path makeTempDir() {
  const auto path = std::filesystem::temp_directory_path() /
                    ("hydra_test_write_atomic_" + std::to_string(::getpid()));
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path;
}

}  // namespace

TEST(WriteFileAtomic, WritesAndReplaces) {
  const auto dir = makeTempDir();
  const auto path = dir / "out.json";
  writeFileAtomic(path, "first");
  EXPECT_EQ(readFile(path), "first");

  const std::string binary("a\0b\nc", 5);
  writeFileAtomic(path, binary);
  EXPECT_EQ(readFile(path), binary);

  // only the target remains (no temporary files)
  size_t num_files = 0;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    EXPECT_EQ(entry.path().filename(), "out.json");
    ++num_files;
  }
  EXPECT_EQ(num_files, 1u);
  std::filesystem::remove_all(dir);
}

TEST(WriteFileAtomic, ThrowsForMissingDirectory) {
  const auto dir = makeTempDir();
  const auto path = dir / "missing" / "out.json";
  EXPECT_THROW(writeFileAtomic(path, "contents"), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path));
  std::filesystem::remove_all(dir);
}

}  // namespace hydra
