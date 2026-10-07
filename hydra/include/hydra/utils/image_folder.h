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
#pragma once

#include <filesystem>
#include <string>

namespace hydra::utils {

//! Subdirectory of an object image root holding the per-track folders written by the
//! frontend before the backend assigns them to a node
inline constexpr char kTempImageFolder[] = "temp";

//! @brief Check whether path lies strictly inside directory (lexical check, no
//! filesystem access)
bool isPathUnder(const std::filesystem::path& path,
                 const std::filesystem::path& directory);

/**
 * @brief Directory that image folder values for an image root are relative to.
 *
 * Image folder values stored in node attributes are relative to the parent directory
 * of the configured image root they belong to, e.g., `images/O_5` for the root
 * `/run/images`, so that run directories can be moved. Absolute values are accepted
 * and pass through unchanged. Trailing separators of the root are ignored.
 */
std::filesystem::path imageFolderBase(const std::filesystem::path& image_root);

//! @brief Filesystem path of a stored image folder value (empty stays empty)
std::filesystem::path resolveImageFolder(const std::filesystem::path& image_root,
                                         const std::string& value);

//! @brief Image folder value to store for a path (relative to imageFolderBase if the
//! path lies inside it, otherwise unchanged)
std::string relativeImageFolder(const std::filesystem::path& image_root,
                                const std::filesystem::path& path);

}  // namespace hydra::utils
