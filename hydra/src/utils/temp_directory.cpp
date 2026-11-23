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
#include "hydra/utils/temp_directory.h"

#include <glog/logging.h>

#include <array>
#include <random>

namespace hydra::utils {

namespace fs = std::filesystem;

namespace {

constexpr static std::array<char, 62> valid_characters{
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
    'w', 'x', 'y', 'z', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
    'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};

std::string random_suffix(size_t characters = 10) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint8_t> distrib(0, 61);
  std::string result(characters, 0);
  for (size_t i = 0; i < characters; ++i) {
    result.at(i) = valid_characters.at(distrib(gen));
  }

  return result;
}

std::filesystem::path make_temp_directory(const fs::path& root,
                                          const std::string& prefix) {
  if (!std::filesystem::exists(root)) {
    std::filesystem::create_directories(root);
  }

  try {
    std::filesystem::path temp_path = root / (prefix + random_suffix());
    while (!fs::create_directory(temp_path)) {
      temp_path = root / (prefix + random_suffix());
    }

    return temp_path;
  } catch (const fs::filesystem_error& ex) {
    LOG(ERROR) << "Unable to make temp directory: " << ex.what();
    return {};
  }
}

}  // namespace

TempDirectory::TempDirectory(const fs::path& root,
                             const std::string& prefix,
                             bool _remove_on_exit)
    : path(make_temp_directory(root, prefix)), remove_on_exit(_remove_on_exit) {}

TempDirectory::~TempDirectory() {
  if (remove_on_exit && !path.empty()) {
    fs::remove_all(path);
  }
}

TempDirectory TempDirectory::system_default(const std::string& prefix,
                                            bool remove_on_exit) {
  return TempDirectory(std::filesystem::temp_directory_path(), prefix, remove_on_exit);
}

std::filesystem::path TempDirectory::operator/(const std::string& other) const {
  if (path.empty()) {
    throw std::domain_error("Invalid temporary directory, path not set");
  }

  return path / other;
}

}  // namespace hydra::utils
