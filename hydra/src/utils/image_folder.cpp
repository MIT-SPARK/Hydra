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
#include "hydra/utils/image_folder.h"

namespace hydra::utils {

std::string keyframeStem(const std::string& prefix, uint64_t timestamp_ns) {
  return prefix + std::to_string(timestamp_ns);
}

bool isPathUnder(const std::filesystem::path& path,
                 const std::filesystem::path& directory) {
  auto normed_dir = directory.lexically_normal();
  if (!normed_dir.empty() && !normed_dir.has_filename()) {
    normed_dir = normed_dir.parent_path();  // trailing separator
  }

  const auto relative = path.lexically_normal().lexically_relative(normed_dir);
  return !relative.empty() && relative != "." && *relative.begin() != "..";
}

std::filesystem::path imageFolderBase(const std::filesystem::path& image_root) {
  auto root = image_root.lexically_normal();
  if (!root.empty() && !root.has_filename()) {
    root = root.parent_path();  // trailing separator
  }

  return root.parent_path();
}

std::filesystem::path resolveImageFolder(const std::filesystem::path& image_root,
                                         const std::string& value) {
  const std::filesystem::path folder(value);
  if (value.empty() || folder.is_absolute()) {
    return folder;
  }

  return imageFolderBase(image_root) / folder;
}

std::string relativeImageFolder(const std::filesystem::path& image_root,
                                const std::filesystem::path& path) {
  const auto base = imageFolderBase(image_root);
  if (base.empty()) {
    return path.is_absolute() ? path.string() : path.lexically_normal().string();
  }

  if (!isPathUnder(path, base)) {
    return path.string();
  }

  return path.lexically_normal().lexically_relative(base).string();
}

}  // namespace hydra::utils
