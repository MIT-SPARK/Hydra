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
// Parts of the image store that do not depend on SQLite (built with and without
// HYDRA_ENABLE_IMAGE_STORE)
#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>

#include <sstream>

#include "hydra/utils/image_store.h"

namespace hydra {

void declare_config(ImageStoreConfig& config) {
  using namespace config;
  name("ImageStoreConfig");
  field<Path>(config.path, "path");
  enum_field(config.on_existing,
             "on_existing",
             {{ImageStoreConfig::OnExisting::Append, "append"},
              {ImageStoreConfig::OnExisting::Rotate, "rotate"},
              {ImageStoreConfig::OnExisting::Fail, "fail"}});
  field<Path>(config.session_artifacts_dir, "session_artifacts_dir");
  field(config.session_artifacts, "session_artifacts");
  field(config.max_images_per_object, "max_images_per_object");
  field(config.jpeg_quality, "jpeg_quality");
  field(config.write_depth, "write_depth");
  field(config.flush_period_ms, "flush_period_ms", "ms");
  field(config.max_queue_size, "max_queue_size");
  checkInRange(config.jpeg_quality, 0, 100, "jpeg_quality");
  check(config.max_queue_size, GT, size_t(0), "max_queue_size");
}

ImageStore::Ptr ImageStore::open(const std::filesystem::path& path) {
  Config config;
  config.path = path;
  return open(config);
}

uint64_t ImageStore::makeId(int robot_id, uint64_t sequence) {
  return (static_cast<uint64_t>(robot_id) << 48) | (sequence & kSequenceMask);
}

std::string ImageStore::encodeMaskRle(const cv::Mat& mask) {
  std::stringstream ss;
  ss << "[";
  if (mask.empty()) {
    ss << "]";
    return ss.str();
  }

  CV_Assert(mask.type() == CV_8UC1);
  bool foreground = false;
  size_t run = 0;
  bool first = true;
  for (int r = 0; r < mask.rows; ++r) {
    const auto row = mask.ptr<uint8_t>(r);
    for (int c = 0; c < mask.cols; ++c) {
      const bool value = row[c] != 0;
      if (value == foreground) {
        ++run;
        continue;
      }

      ss << (first ? "" : ",") << run;
      first = false;
      foreground = value;
      run = 1;
    }
  }

  ss << (first ? "" : ",") << run << "]";
  return ss.str();
}

}  // namespace hydra
